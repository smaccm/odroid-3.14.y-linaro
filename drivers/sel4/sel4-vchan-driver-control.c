/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(NICTA_GPL)
 */

/* Necessary includes for device drivers */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h> /* printk() */
#include <linux/slab.h> /* kmalloc() */
#include <linux/ioctl.h>
#include <linux/fs.h> /* everything... */
#include <linux/errno.h> /* error codes */
#include <linux/proc_fs.h>
#include <linux/fcntl.h> /* O_ACCMODE */
#include <asm/uaccess.h> /* copy_from/to_user */
#include <linux/wait.h>
#include <linux/sched.h> /*Helps fix TASK_UNINTERRUPTIBLE */
#include <linux/pid.h>
#include <linux/eventfd.h>
#include <linux/kthread.h>  // for threads
#include <linux/time.h>   // for using jiffies
#include <linux/timer.h>
#include <linux/of.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <asm/compiler.h>
#include <asm/opcodes-sec.h>
#include <asm/opcodes-virt.h>
#include <asm/psci.h>
#include <linux/platform_device.h>

 #include <linux/module.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/moduleparam.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/unaligned.h>

#include "vmm_manager.h"
#include "sel4libvchan.h"
#include "vmm_driver.h"
#include "vchan_copy.h"
#include "sel4.h"

#define DRIVER_NAME "sel4-vchan"
#define DRIVER_AUTH "<nicta.com.au>"
#define DRIVER_DESC "SeL4 IPC inter-vm communication"
#define _ASM_VMCALL ".byte 0x0f,0x01,0xc1" /* ia32 instruction to perform a vmcall hypervisor exception */

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTH);
MODULE_DESCRIPTION(DRIVER_DESC);

#define max(X, Y)               \
    ({ typeof(X) __x = (X);         \
        typeof(Y) __y = (Y);        \
        (__x > __y) ? __x : __y; })

#define min(X, Y)               \
    ({ typeof(X) __x = (X);         \
        typeof(Y) __y = (Y);        \
        (__x < __y) ? __x : __y; })

/* Driver management functions */
int vmm_manager_open(struct inode *inode, struct file *filp);
int vmm_manager_release(struct inode *inode, struct file *filp);
static long vmm_manager_ioctl(struct file *f, unsigned int cmd, unsigned long arg);

static void sel4_vchan_exit(void);
static int sel4_vchan_init(void);

/* Actions that correspond to sel4 vmm manager actions */
int vmm_num_of_dom(void *cont, ioctl_arg_t *args, int cmd);
int vmm_def_action(void *cont, ioctl_arg_t *args, int cmd);
int vmm_read_write(void *cont, ioctl_arg_t *args, int cmd);
int vmm_guest_num(void *cont, ioctl_arg_t *args, int cmd);
int driver_sleep_vm(void *cont, ioctl_arg_t *args, int cmd);
int driver_wakeup_vm(void *cont, ioctl_arg_t *args, int cmd);

int sel4_driver_vchan_connect(void *cont, ioctl_arg_t *args, int cmd);
int sel4_driver_vchan_close(void *cont, ioctl_arg_t *args, int cmd);
int sel4_driver_vchan_wait(void *cont, ioctl_arg_t *args, int cmd);
int sel4_driver_vchan_state(void *cont, ioctl_arg_t *args, int cmd);

/* Function lookup table for driver actions */
static struct vmm_op_lookup_table {
    int (*op_func[NUM_VMM_OPS])(void *, ioctl_arg_t *, int);
} vmm_op_lookup_table = {

    .op_func[VMM_NUM_OF_DOM] = &vmm_def_action,

    .op_func[VCHAN_SEND] = &vmm_read_write,
    .op_func[VCHAN_RECV] = &vmm_read_write,

    .op_func[VMM_GUEST_NUM] = &vmm_guest_num,

    .op_func[SEL4_VCHAN_CONNECT] = &sel4_driver_vchan_connect,
    .op_func[SEL4_VCHAN_CLOSE] = &sel4_driver_vchan_close,
    .op_func[SEL4_VCHAN_WAIT] = &sel4_driver_vchan_wait,
    .op_func[SEL4_VCHAN_STATE] = &sel4_driver_vchan_state,

    // .op_func[VCHAN_SLEEP] = &driver_sleep_vm,
    // .op_func[VCHAN_WAKEUP] = &driver_wakeup_vm,
};


static int sel4_vchan_drv_probe(struct platform_device *pdev);
static int sel4_vchan_drv_rem(struct platform_device *pdev);

#ifdef CONFIG_OF
static const struct of_device_id sel4_vchan_match[] = {
    { .compatible = "sel4,vchan" },
    { .compatible = "sel4,vchan" },
    {},
};
MODULE_DEVICE_TABLE(of, sel4_vchan_match);
#endif

static struct platform_driver sel4_vchan_driver = {
    .probe = sel4_vchan_drv_probe,
    .remove = sel4_vchan_drv_rem,
    .driver = {
        .name = "sel4-vchan",
        .owner   = THIS_MODULE,
        .of_match_table = of_match_ptr(sel4_vchan_match),
    },
};

static struct resource vchan_resources[] = {
    {
        .start = 0x2040000,
        .end = 0x2041000,
        .flags = IORESOURCE_MEM,
    },
    {
        .start = VCHAN_EVENT_IRQ,
        .end = VCHAN_EVENT_IRQ,
        .flags = IORESOURCE_IRQ,
    }
};

static struct platform_device sel4_vchan_device = {
    .name = DRIVER_NAME,
    .id = -1,
    .num_resources = ARRAY_SIZE(vchan_resources),
    .resource = vchan_resources,
};

struct file_operations vmm_fops = {
    open: vmm_manager_open,
    release: vmm_manager_release,
    unlocked_ioctl: vmm_manager_ioctl,
};

int vchan_dev_major = 1;
int vmm_run_num;

/* Memory buffer for storing arguments passed to vmm */
void *vmm_mem_buf = NULL;
struct semaphore hyp_sem; /* Mutex for modifying the vchan instance list */
struct vmcall_args *vargs = NULL;
vmm_args_t *uvargs = NULL;

static void *hyp_call_phys_addr;

static dev_t first;             // Global variable for the first device number
static struct cdev c_dev;       // Global variable for the character device structure
static struct class *cl;        // Global variable for the device class

static int sel4_vchan_drv_probe(struct platform_device *pdev) {
    // printk(KERN_ALERT "sel4-vchan-driver: ##### Salut, Mundi, vmm_manager: assigned major: %d #####\n", vchan_dev_major);
    // res = devm_ioremap(&pdev->dev, res->start, resource_size(res));
    // if(!res) {
    //     dev_err(&pdev->dev, "Failed to remap I/O memory\n");
    //     err = -ENOMEM;
    //     goto fail;
    // }

    return 0;
}

static int sel4_vchan_drv_rem(struct platform_device *pdev){
    return 0;
}

/*
    Initialise the vmm_manager
*/
static int __init sel4_vchan_init(void) {
    int err, irq, res, *data;
    struct platform_device *pdev = &sel4_vchan_device;


    res = alloc_chrdev_region( &first, 0, 1, DRIVER_NAME );
    if(res < 0) {
        dev_err(&pdev->dev, "Unable to alloc chardev region");
        err = -1;
        goto fail;
    }

    // res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    // if(!res) {
    //     dev_err(&pdev->dev, "Failed to get I/O memory\n");
    //     err = -ENXIO;
    //     goto fail;
    // }

    if ( (cl = class_create( THIS_MODULE, "vmm_manager" ) ) == NULL ) {
        dev_err(&pdev->dev, "chardev - class creation failed\n");
        unregister_chrdev_region( first, 1 );
        err = -1;
        goto fail;
    }

    if( device_create( cl, NULL, first, NULL, "vmm_manager" ) == NULL ) {
        dev_err(&pdev->dev, "chardev - device creation failed\n");
        class_destroy(cl);
        unregister_chrdev_region( first, 1 );
        err = -1;
        goto fail;
    }

    irq = platform_get_irq(pdev, 0);
    if (!irq) {
        dev_err(&pdev->dev, "Failed to get IRQ\n");
        err = -ENODEV;
        goto fail;
    }

    sema_init(&hyp_sem, 1);

    if(init_event_thread() < 0) {
        dev_err(&pdev->dev, "vchan irq - could not init irq thread\n");
        class_destroy(cl);
        unregister_chrdev_region( first, 1 );
        err = -1;
        goto fail;
    }

    // uvargs = (vmm_args_t *)vargs->data;
    hyp_call_phys_addr = ioremap(0x2040000, 0x1000);

    /* Register this device with the linux kernel */
    vchan_dev_major = register_chrdev(0, DRIVER_NAME, &vmm_fops);
    if (vchan_dev_major < 0) {
        dev_err(&pdev->dev, "chardev - failed to register\n");
        err = -EINVAL;
        goto fail;
    }

    /* Create a memory buffer in the kernel for copying in/out non vchan arguments */
    vmm_mem_buf = kmalloc(sizeof(struct vmcall_args), 0);
    if (vmm_mem_buf == NULL) {
        printk ("k_vmm_manager: Failed to allocate buffer memory\n");
        err = -ENOMEM;
        goto fail;
    }

    vargs = (struct vmcall_args *)vmm_mem_buf;
    vargs->data = kmalloc(sizeof(struct vmm_args), 0);
    if (vargs->data == NULL) {
        printk ("k_vmm_manager: Failed to allocate buffer memory\n");
        kfree(vmm_mem_buf);
        err = -ENOMEM;
        goto fail;
    }

    /* Store physical addresses of memory buffers, as the vmm uses physical addresses */
    uvargs = (vmm_args_t *)vargs->data;
    hyp_call_phys_addr = ioremap(0x2040000, 0x1000);

    /* Check with the hypervisor for ok connection */
    err = call_into_hypervisor(VMM_CONNECT, uvargs, DRIVER_ARGS_MAX_SIZE, vargs);
    if (err) {
        printk ("k_vmm_manager: failed on sel4 hypervisor connection %d", err);
        kfree(vargs->data);
        kfree(vmm_mem_buf);
        err = -EINVAL;
        goto fail;
    }

    /* Set the running number for this vm */
    data = (int *)uvargs->ret_data;
    vmm_run_num = *data;
    // printk(KERN_INFO "k_vmm_manager: vmm driver running on guest %d|%d\n", vmm_run_num,  *data);

    cdev_init( &c_dev, &vmm_fops );
    if( cdev_add( &c_dev, first, 1 ) == -1) {
        dev_err(&pdev->dev, "chardev - failed device addition \n");
        device_destroy( cl, first );
        class_destroy( cl );
        unregister_chrdev_region( first, 1 );
        err = -EINVAL;
        goto fail;
    }

    pr_info("%s: " DRIVER_DESC "\n", DRIVER_NAME);


    return platform_driver_register(&sel4_vchan_driver);

    fail:
        return err;



    // printk(KERN_INFO "k_vmm_manager: Salut, Mundi, vmm_manager: assigned major: %d\n", vchan_dev_major);

    return 0;
}

/* Removal of the vmm_driver */
static void __exit sel4_vchan_exit(void) {
    // unregister_chrdev(vchan_dev_major, DRIVER_NAME);

    // kfree(vargs->data);
    // kfree(vmm_mem_buf);

    printk(KERN_INFO "k_vmm_manager: Removed vmm manager module\n");
    return;
}

// Open - Does nothing
int vmm_manager_open(struct inode *inode, struct file *filp) {
    return 0;
}

// Open - Does nothing
int vmm_manager_release(struct inode *inode, struct file *filp) {
    return 0;
}

/*
    Vmm Iocotl, main interaction handler for the vmm_manager

    Recieves cmd, and pointer to user space memory for arguments
        - Performs an operation based on the command passed in,
*/
static long vmm_manager_ioctl(struct file *f, unsigned int cmd, unsigned long arg) {
    ioctl_arg_t ioctl_args;
    void *in_arg_addr, *user_arg;
    int err;

    size_t size = _IOC_SIZE(cmd);
    cmd = _IOC_NR(cmd);
    user_arg = (void __user *)arg;

    if(cmd > NUM_VMM_OPS) {
        return -EINVAL;
    } else {
        /* Check for a valid command and a non-null pointer */
        if(vmm_op_lookup_table.op_func[cmd] == NULL ) {
            printk("drvr: bad cmd %d\n", cmd);
            return -ENOTTY;
        }

        in_arg_addr = kmalloc(size, GFP_KERNEL);
        if(in_arg_addr == NULL) {
            printk("drvr: bad malloc\n");
            return -ENOMEM;
        }

        err = copy_from_user(in_arg_addr, user_arg, size);
        if(err) {
            printk("drvr: bad cpy %d\n", err);
            kfree(in_arg_addr);
            return -EINVAL;
        }

        ioctl_args.size = size;
        ioctl_args.ptr = user_arg;

        err = (*vmm_op_lookup_table.op_func[cmd])(in_arg_addr, &ioctl_args, cmd);
        kfree(in_arg_addr);

        return err;
    }
}

/*
    Function to call into the sel4 vmm
    - Given command/request pointer is written to cause a page fault

    The underlying vm is expected to operate on the pointer and store results in it
*/
int call_into_hypervisor(int cmd, void *data, size_t sz, vmcall_args_t *vmcall) {
    unsigned phys_ptr = virt_to_phys(vmcall);

    vmcall->data = data;
    vmcall->cmd = cmd;
    vmcall->phys_data = virt_to_phys(data);
    vmcall->size = sz;

    down(&hyp_sem);
    if(hyp_call_phys_addr == NULL) {
        return -1;
        printk("vchan: >>> bad call\n");
    } else {
        unsigned *write = (unsigned *)hyp_call_phys_addr;
        *write = phys_ptr;
    }

    up(&hyp_sem);

    return vmcall->err;
}

/*
    Perform a vmm manager action that can be fit into the standard vmm_manager argument
*/
int vmm_def_action(void *cont, ioctl_arg_t *args, int cmd) {
    int err;
    /* Call into the sel4 vmm, return early if error encountered */
    err = call_into_hypervisor(cmd, cont, args->size, vargs);
    if(err) {
        return err;
    }

    /* Finished, copy out arguments */
    err = copy_to_user(args->ptr, cont, args->size);
    if(err) {
        return -EINVAL;
    }

    return args->size;
}

/*
    Perform a read/write to a given operation
*/
int vmm_read_write(void *cont, ioctl_arg_t *args, int cmd) {
    int err, res;
    size_t send_size, remaining, total;
    void *user_ptr;

    vchan_args_t *vchan_args = (vchan_args_t *)cont;
    user_ptr = vchan_args->mmap_ptr;

    /* Simple sanity checking of size and ptr */
    if(user_ptr == NULL) {
        printk("k_vmm_manager_vchan: bad ptr\n");
        return -EINVAL;
    } else if (vchan_args->size <= 0) {
        printk("k_vmm_manager_vchan: bad size\n");
        return 0;
    }

    /* Make an argument that can be passed into the kernel */
    vchan_args->mmap_ptr = kmalloc(vchan_args->size, GFP_KERNEL);
    if(vchan_args->mmap_ptr == NULL) {
        printk("k_vmm_manager_vchan: bad malloc\n");
        return -ENOMEM;
    }

    /* Copy data from user level if valid */
    if(cmd == VCHAN_SEND) {
        err = copy_from_user(vchan_args->mmap_ptr, user_ptr, vchan_args->size);
        if(err) {
            printk("sel4-vchan-driver-readwrite: BAD 2nd COPY\n");
            kfree(vchan_args->mmap_ptr);
            return -EINVAL;
        }
    }


    remaining = vchan_args->size;
    total = 0;
    while(remaining > 0) {
        printk(KERN_DEBUG "sel4-vchan-driver: readwrite req: %d|%d size: %d|%d\n",
                cmd, vchan_args->v.port, remaining, vchan_args->stream);
        vchan_args->mmap_phys_ptr = virt_to_phys(vchan_args->mmap_ptr + total);
        if(vchan_args->mmap_phys_ptr == 0) {
            printk("sel4-vchan-driver-readwrite: bad phys pointer\n");
            return -EINVAL;
        }

        vchan_args->size = max(1, event_thread_info(vchan_args->v.dest, vchan_args->v.port, cmd));
        vchan_args->size = min(vchan_args->size, remaining);
        res = wait_for_event(vchan_args->v.dest, vchan_args->v.port, cmd, vchan_args->size);

        /* Connection closed and/or there is no data */
        if(res == -1) {
            printk("sel4-vchan-driver-readwrite: error in wait!\n");
            kfree(vchan_args->mmap_ptr);
            return 0;
        }

        err = call_into_hypervisor(cmd, vchan_args, args->size, vargs);
        if(err) {
            printk("sel4-vchan-driver-readwrite: bad hypervisor call %d\n", err);
            kfree(vchan_args->mmap_ptr);
            return err;
        }

        total += vchan_args->size;
        if(vchan_args->stream)
            remaining = 0;
        else
            remaining -= vchan_args->size;
    }

    send_size = total;

    if(cmd == VCHAN_RECV) {
        err = copy_to_user(user_ptr, vchan_args->mmap_ptr, send_size);
        if(err) {
            printk("sel4-vchan-driver-readwrite: copying out %d failed\n", send_size);
            kfree(vchan_args->mmap_ptr);
            return -EINVAL;
        }
    }

    kfree(vchan_args->mmap_ptr);
    return send_size;
}

/*
    Return the number vm guest this driver is running on
*/
int vmm_guest_num(void *cont, ioctl_arg_t *args, int cmd) {
    int err;

    int *ret = (int *) uvargs->ret_data;
    *ret = vmm_run_num;
    uvargs->datatype = DATATYPE_INT;

    /* Finished, copy out arguments */
    err = copy_to_user(args->ptr, uvargs, sizeof(vmm_args_t));
    if(err) {
        return -EINVAL;
    }

    return sizeof(vmm_args_t);
}

/*
    Creates a new vchan instance,
        by notifying the vmm and creating kernel level state
*/
int sel4_driver_vchan_connect(void *cont, ioctl_arg_t *args, int cmd) {
    int err = 0;
    vchan_alert_t *event_mon;
    vchan_connect_t *pass = (vchan_connect_t *)cont;
    // printk("cn: %d %d %d %d\n", pass->v.dest, pass->v.port, pass->event_mon, pass->eventfd);
    /* Set up event monitor */
    event_mon = kmalloc(sizeof(vchan_alert_t), GFP_KERNEL);
    if(event_mon == NULL) {
        return -ENOMEM;
    }

    event_mon->dest = pass->v.dest;
    event_mon->port = pass->v.port;
    event_mon->is_closed = 0;
    event_mon->buffer_space = 0;
    event_mon->data_ready = 0;

    pass->event_mon = virt_to_phys(event_mon);

    if(new_event_instance(pass->v.dest, pass->v.port, pass->eventfd, event_mon, vmm_run_num) < 0) {
        printk("k_vmm_manager_vchan_connect: bad event creation\n");
        return -EINVAL;
    }

    err = call_into_hypervisor(cmd, cont, args->size, vargs);
    if (err) {
        printk("k_vmm_manager_vchan_connect: bad hypervisor call\n");
        rem_event_instance(pass->v.dest, pass->v.port);
        return err;
    }

    return sizeof(vchan_connect_t);
}

/*
    Closes a vchan instance,
        by notifying the vmm and closing kernel level state
*/
int sel4_driver_vchan_close(void *cont, ioctl_arg_t *args, int cmd) {
    int err;
    vchan_connect_t *pass;

    pass = (vchan_connect_t *)cont;

    // printk("cn close: %d %d %d %d\n", pass->v.dest, pass->v.port, pass->event_mon, pass->eventfd);
    rem_event_instance(pass->v.dest, pass->v.port);

    err = call_into_hypervisor(cmd, cont, args->size, vargs);
    if (err) {
        printk("k_vmm_manager_vchan_close: bad hypervisor call\n");
        return err;
    }

    return sizeof(vchan_connect_t);
}

/*
    nowait = 0: Wait until a read/write action is possible, blocking
    nowait > 0: check the state of the shared vchan buffer
*/
int sel4_driver_vchan_wait(void *cont, ioctl_arg_t *args, int cmd) {
    int err;
    vchan_check_args_t *in_wait = (vchan_check_args_t *)cont;

    if(in_wait->nowait) {
        in_wait->state = event_thread_info(in_wait->v.dest, in_wait->v.port, in_wait->checktype);
        if(in_wait->state < 0)
            return -1;
    } else {
        // printk("kwait: perfoming wait\n");
        /* Wait for data, or closed connection */
        in_wait->state = wait_for_event(in_wait->v.dest, in_wait->v.port, VCHAN_RECV, 1);
    }

    err = copy_to_user(args->ptr, cont, sizeof(vchan_check_args_t));
    if(err) {
        printk("k_vmm_manager_vchan_wait: failed copyout\n");
        return -1;
    }

    // printk("%d;;retval\n", in_wait->state);

    return sizeof(vchan_check_args_t);

}

/*
    Check the status of a vchan connection
*/
int sel4_driver_vchan_state(void *cont, ioctl_arg_t *args, int cmd) {
    int err;
    // printk("k_vmm_manager_vchan_state: checking state\n");

    err = call_into_hypervisor(cmd, cont, args->size, vargs);
    if (err) {
        printk("k_vmm_manager_vchan_state: bad hypervisor call %d err\n", err);
        return -EINVAL;
    }

    err = copy_to_user(args->ptr, cont, sizeof(vchan_check_args_t));
    if(err) {
        printk("k_vmm_manager_vchan_state: bad copy\n");
        return -EINVAL;
    }

    return sizeof(vchan_check_args_t);
}

/*
    Sleep this vm until it is woken up by an event
*/
int driver_sleep_vm(void *cont, ioctl_arg_t *args, int cmd) {
    int err;
    err = call_into_hypervisor(cmd, cont, args->size, vargs);
    if (err) {
        printk("k_vmm_manager_vchan_state: bad sleep call\n");
        return err;
    }

    return 0;
}

/*
    Wakeup a given vm by sending it an event
*/
int driver_wakeup_vm(void *cont, ioctl_arg_t *args, int cmd) {
    int err;

    err = call_into_hypervisor(cmd, cont, args->size, vargs);
    if (err) {
        printk("k_vmm_manager_vchan_state: bad wakeup call\n");
        return err;
    }

    return  0;
}


module_init(sel4_vchan_init);
module_exit(sel4_vchan_exit);
