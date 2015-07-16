/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(NICTA_GPL)
 */

#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/list.h>

#include "vmm_manager.h"
#include "sel4libvchan.h"
#include "vmm_driver.h"


/*
    Represents an active vchan connection
        Handles relaying state of vchan buffer, and blocking threads
*/
typedef struct event_instance {
    int port, domain, self; /* vchan connection */
    /*
        vchan monitor linked to sel4 vmm vchan instance
            vmm changes values stored in this monitor, whenever the vchan changes
    */
    vchan_alert_t *event_mon;
    /*
        Used for pinging an user level eventfd
            FIXME: this is currently unsupported
    */
    struct eventfd_ctx *efd_ctx;
    struct list_head node;
} einstance_t;


static struct vchan_event_control {
    int num_instances;
    /* Mutex for modifying the vchan instance list */
    struct semaphore inst_sem;
    /* All active vchan instances in the system */
    struct list_head instances;

    /*
        Used by event_thread_run, a top half interrupt handler
    */
    struct workqueue_struct *event_work;
    struct work_struct *event_work_struct;
} vchan_ctrl = {
    .num_instances = 0,
};

DECLARE_WAIT_QUEUE_HEAD(vchan_wait_queue);

// static uint64_t plus_one = 1;

static int check_valid_action(int domain, int port, int type, size_t request_size);
static irqreturn_t event_irq_handler(int, void *);
static void event_thread_run(struct work_struct *work);
static einstance_t *get_event_instance(int domain, int port);
static void print_vchan_inst(void);

/*
    Bottom half interrupt handler
        When the hypervisor changes the state of a vchan connection, it triggers an interrupt
        We register a function to run later (via a workqueue), to see what state has changed
*/
static irqreturn_t event_irq_handler(int irq, void *dev_id) {
    struct vchan_event_control *dv = (struct vchan_event_control *)dev_id;
    queue_work(dv->event_work, dv->event_work_struct);
    return IRQ_HANDLED;
}

/*
    Functions for setting up the hypervisor interrupt
*/
int reg_event_irq_handler() {
    return request_irq(VCHAN_EVENT_IRQ, &event_irq_handler, IRQF_SHARED, DRIVER_NAME, &vchan_ctrl);
}

void free_event_irq_handler() {
    free_irq(VCHAN_EVENT_IRQ, &vchan_ctrl);
}

/*
    Initialise the instance event monitor
*/
int init_event_thread(void) {
    /* Set up interrupt handler */
    if(reg_event_irq_handler() < 0) {
        printk(KERN_ERR "sel4-vchan-driver-irq: failed to register handler");
        return -1;
    }

    /* Set up workqueue */
    vchan_ctrl.event_work = create_singlethread_workqueue("sel4-vchan-driver-irq-workqueue");
    if(vchan_ctrl.event_work) {
        vchan_ctrl.event_work_struct = (struct work_struct *)kmalloc(sizeof(struct work_struct), GFP_ATOMIC);
        if(vchan_ctrl.event_work_struct) {
            INIT_WORK( vchan_ctrl.event_work_struct, event_thread_run );
        } else {
            printk(KERN_ERR "sel4-vchan-driver-irq: failed to alloc mem for workqueue");
            return -1;
        }
    } else {
        printk(KERN_ERR "sel4-vchan-driver-irq: failed to init workqueue");
        return -1;
    }

    INIT_LIST_HEAD(&vchan_ctrl.instances);
    sema_init(&vchan_ctrl.inst_sem, 1);

    return 0;
}

/*
    Connect a vchan instance to the event monitor
*/
int new_event_instance(int domain, int port, int eventfd, vchan_alert_t *mon, int self) {
    einstance_t *inst;
    pid_t pid = task_pid_nr(current);
    struct task_struct * userspace_task = NULL;
    struct file * efd_file = NULL;

    if(mon == NULL) {
        printk(KERN_ERR "sel4-vchan-driver-irq: bad vchan monitor\n");
        return -EINVAL;
    }

    down(&vchan_ctrl.inst_sem);

    inst = get_event_instance(domain, port);
    if(inst != NULL) {
        printk(KERN_ERR "sel4-vchan-driver-irq: vchan monitor instance already active\n");
        printk(KERN_ERR "sel4-vchan-driver-irq: can only have one monitor instance per vchan\n");
        up(&vchan_ctrl.inst_sem);
        return -EINVAL;
    }

    inst = kmalloc(sizeof(einstance_t), GFP_ATOMIC);
    if(inst == NULL) {
        printk(KERN_ERR "sel4-vchan-driver-irq: failed to allocate vchan monitor instance mem\n");
        up(&vchan_ctrl.inst_sem);
        return -EINVAL;
    }

    vchan_ctrl.num_instances++;
    inst->domain = domain;
    inst->port = port;
    inst->event_mon = mon;
    inst->self = self;

    /* Find the eventfd a user level vchan connection uses and link it in */
    userspace_task = pid_task(find_vpid(pid), PIDTYPE_PID);
    BUG_ON(userspace_task == NULL);

    rcu_read_lock();
    efd_file = fcheck_files(userspace_task->files, eventfd);
    rcu_read_unlock();

    BUG_ON(efd_file == NULL);

    inst->efd_ctx = eventfd_ctx_fileget(efd_file);
    BUG_ON(inst->efd_ctx == NULL);

    /* Add to the instance list */
    INIT_LIST_HEAD(&inst->node);
    list_add(&inst->node, &vchan_ctrl.instances);

    up(&vchan_ctrl.inst_sem);

    return 0;
}

/*
    Disconnect a vchan instance from the event monitor
*/
void rem_event_instance(int domain, int port) {
    einstance_t *inst, *next;
    struct list_head *head = &vchan_ctrl.instances;

    down(&vchan_ctrl.inst_sem);

    list_for_each_entry_safe(inst, next, head, node) {
        if(inst->domain == domain && inst->port == port) {
            printk(KERN_DEBUG "linux-sel4-vchan-driver: found instance to delete\n");
            list_del(&inst->node);
            eventfd_ctx_put(inst->efd_ctx);
            kfree(inst->event_mon);
            kfree(inst);
            vchan_ctrl.num_instances--;
            up(&vchan_ctrl.inst_sem);
            return;
        }
    }

    printk(KERN_ERR "linux-sel4-vchan-driver: trying to delete non-existent instance\n");

    up(&vchan_ctrl.inst_sem);
}

static vchan_alert_t *get_vchan_status(int domain, int port) {
    einstance_t *inst;

    inst = get_event_instance(domain, port);
    if(inst == NULL) {
        return NULL;
    }

    return inst->event_mon;
}

/*
    Check to see if the state of a given vchan is blocking or not
*/
static int check_valid_action(int domain, int port, int type, size_t request_size) {
    vchan_alert_t *alrt;

    down(&vchan_ctrl.inst_sem);
    alrt = get_vchan_status(domain, port);
    if(alrt == NULL) {
        up(&vchan_ctrl.inst_sem);
        return -1;
    }

    if(type == VCHAN_RECV) {
        if(request_size <= alrt->data_ready) {
            return 1;
        /* closed vchan connection, and no more data ready to read */
        } else if(alrt->data_ready == 0 && alrt->buffer_space == -1) {
            return -1;
        }
    } else {
        if(alrt->buffer_space >= 0) {
            if(alrt->buffer_space >= request_size) {
                return 1;
            }
        } else {
            /* closed vchan connection, cannot send any more data */
            return -1;
        }
    }
    up(&vchan_ctrl.inst_sem);

    return 0;
}

int event_thread_info(int domain, int port, int type, int *closed) {
    int rval;
    vchan_alert_t *alrt;

    down(&vchan_ctrl.inst_sem);
    alrt = get_vchan_status(domain, port);
    if(alrt == NULL) {
        up(&vchan_ctrl.inst_sem);
        return -1;
    }

    if(type == NOWAIT_DATA_READY || type == VCHAN_RECV) {
        rval = alrt->data_ready;
    } else {
        if(alrt->buffer_space >= 0) {
            rval = alrt->buffer_space;
        } else {
            rval = 0;
        }
    }

    if(alrt->buffer_space == -1)
        *closed = 1;
    else
        *closed = 0;

    up(&vchan_ctrl.inst_sem);

    return rval;
}

/*
    Top half interrupt handler for hypervisor state change interrupt
*/
static void event_thread_run(struct work_struct *work) {
    wake_up(&vchan_wait_queue);
}

/*
    Returns an event instance
*/
static einstance_t *get_event_instance(int domain, int port) {
    einstance_t *inst, *next;
    struct list_head *head = &vchan_ctrl.instances;

    list_for_each_entry_safe(inst, next, head, node) {
        if(inst->port == port && inst->domain == domain) {
            up(&vchan_ctrl.inst_sem);
            return inst;
        }
    }

    return NULL;
}

/*
    Wait for a desired event to happen, blocking if it has not happened already
*/
int wait_for_event(int domain, int port, int type, size_t request_size) {

    int closed;
    int status = check_valid_action(domain, port, type, request_size);
    int vchan_info = event_thread_info(domain, port, type, &closed);
    /* Cannot perform action, vchan is closed */
    if(status < 0 || closed) {
        printk(KERN_ERR "sel4-vchan-driver: closed status\n");
        return -1;
    } else if(status == 0) {
        /* Vchan is blocking, sleep until non-block */
        printk(KERN_DEBUG "linux-sel4-vchan-driver: sleeping thread until action possible\n");
        if(type == VCHAN_RECV) {
            printk(KERN_DEBUG "linux-sel4-vchan-driver: action: recv request:|sz:%d| have:|%d|\n", request_size, vchan_info);
        } else {
            printk(KERN_DEBUG "linux-sel4-vchan-driver: action: send request:|sz:%d| have:|%d|\n", request_size, vchan_info);
        }

        do {
            wait_event(vchan_wait_queue, check_valid_action(domain, port, type, request_size) != 0);
            status = check_valid_action(domain, port, type, request_size);
            printk(KERN_DEBUG "linux-sel4-vchan-driver: walking away now with %d..\n", status);
        } while(status == 0);
    }

    printk(KERN_DEBUG "sel4-vchan-driver: status is %d\n", status);

    return status;
}

/*
    Debug printing routine
*/
static void print_vchan_inst(void) {
    einstance_t *inst, *next;
    struct list_head *head = &vchan_ctrl.instances;
    printk(KERN_DEBUG "sel4-vchan-driver-irq: %d active instances = ", vchan_ctrl.num_instances);
    list_for_each_entry_safe(inst, next, head, node) {
        printk(KERN_DEBUG "|inst p:%d d:%d|-", inst->port, inst->domain);
    }
}
