/*
 * Virtual USB driver for seL4 VMM
 *
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See the COPYING file in the top-level directory.
 */

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

#include "sel4.h"

//#define DEBUG_VUSB
//#define DEBUG_ROOTHUB

#define max_packet(wMaxPacketSize) ((wMaxPacketSize) & 0x07ff)

#define CNORMAL   "\033[0m"

#define CFRED     "\033[31m"
#define CFGREEN   "\033[32m"
#define CFYELLOW  "\033[33m"
#define CFBLUE    "\033[34m"
#define CFMAGENTA "\033[35m"
#define CFCYAN    "\033[36m"
#define CFWHITE   "\033[37m"

#define CBRED     "\033[41m"
#define CBGREEN   "\033[42m"
#define CBYELLOW  "\033[43m"
#define CBBLUE    "\033[44m"
#define CBMAGENTA "\033[45m"
#define CBCYAN    "\033[46m"
#define CBWHITE   "\033[47m"

#define note(...)                                \
	do {                                         \
		printk(CFCYAN "SEL4USB:" __VA_ARGS__);   \
		printk(CNORMAL);                         \
	} while (0)


#ifdef DEBUG_VUSB
#define dvusb(...)                               \
	do {                                         \
		printk(CFYELLOW "SEL4USB:" __VA_ARGS__); \
		printk(CNORMAL);                         \
	} while (0)
#else
#define dvusb(...) do {} while (0)
#endif

#ifdef DEBUG_ROOTHUB
#define DROOTHUB(...)                                      \
	do {                                                   \
		printk(CFYELLOW "SEL4USB root hub: " __VA_ARGS__); \
		printk(CNORMAL);                                   \
	} while (0)
#else
#define DROOTHUB(...) do {} while (0)
#endif


#define DRIVER_AUTHOR "Alexander Kroh"
#define DRIVER_DESC "USB 2.0 'Virtualised' Host Controller (VHCI) Driver"


#define MAX_ACTIVE_URB   (0x1000 / sizeof(struct sel4urb))

#define SURBT_PARAM_GET_TYPE(param) (((param) >> 30) & 0x3)
#define SURBT_PARAM_GET_SIZE(param) (((param) >>  0) & 0x0fffffff)
#define SURBT_PARAM_TYPE(type)      ((type) << 30)
#define SURBT_PARAM_SIZE(size)      ((size) << 0)

#define SURB_EPADDR_STATE_INVALID   (0 << 28)
#define SURB_EPADDR_STATE_PENDING   (1 << 28)
#define SURB_EPADDR_STATE_ACTIVE    (2 << 28)
#define SURB_EPADDR_STATE_COMPLETE  (4 << 28)
#define SURB_EPADDR_STATE_SUCCESS   (SURB_EPADDR_STATE_COMPLETE | (0 << 28))
#define SURB_EPADDR_STATE_ERROR     (SURB_EPADDR_STATE_COMPLETE | (1 << 28))
#define SURB_EPADDR_STATE_CANCELLED (SURB_EPADDR_STATE_COMPLETE | (2 << 28))
#define SURB_EPADDR_STATE_MASK      (SURB_EPADDR_STATE_COMPLETE | (3 << 28))
#define SURB_EPADDR_GET_STATE(x)    ((x) & SURB_EPADDR_STATE_MASK)
#define SURB_EPADDR_GET_ADDR(x)     (((x) >>  0) & 0x7f)
#define SURB_EPADDR_GET_HUB_ADDR(x) (((x) >>  7) & 0x7f)
#define SURB_EPADDR_GET_HUB_PORT(x) (((x) >> 14) & 0x7f)
#define SURB_EPADDR_GET_EP(x)       (((x) >> 21) & 0x0f)
#define SURB_EPADDR_GET_SPEED(x)    (((x) >> 25) & 0x03)
#define SURB_EPADDR_GET_DT(x)       (((x) >> 27) & 0x01)
#define SURB_EPADDR_ADDR(x)         ((x) <<  0)
#define SURB_EPADDR_HUB_ADDR(x)     ((x) <<  7)
#define SURB_EPADDR_HUB_PORT(x)     ((x) << 14)
#define SURB_EPADDR_EP(x)           ((x) << 21)
#define SURB_EPADDR_SPEED(x)        ((x) << 25)
#define SURB_EPADDR_DT              (1 << 27)

const char* pipe_type_str(unsigned int pipe)
{
	switch (usb_pipetype(pipe)) {
	case PIPE_ISOCHRONOUS: return "isoc";
	case PIPE_INTERRUPT:   return "int ";
	case PIPE_CONTROL:     return "ctrl";
	case PIPE_BULK:        return "bulk";
	default:               return "????";
	}
}

static const char hcd_name[] = "vhci_hcd";

static struct platform_driver vhci_driver;

static const struct usb_device_descriptor _hub_device_desc = {
	.bLength = sizeof(_hub_device_desc),
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x200,
	.bDeviceClass = USB_CLASS_HUB,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 2,
	.bMaxPacketSize0  = 64,
	.idVendor = 0xFEED,
	.idProduct = 0xBEEF,
	.bcdDevice = 1234,
	.iManufacturer = 0,
	.iProduct = 0,
	.iSerialNumber = 0,
	.bNumConfigurations = 1
};

static const struct usb_interface_descriptor _hub_iface_desc = {
	.bLength = sizeof(_hub_iface_desc),
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = 9,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 1,
	.iInterface = 0
};

static const struct usb_endpoint_descriptor _hub_endpoint_desc = {
	.bLength = sizeof(_hub_endpoint_desc),
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = 0x3,
	.wMaxPacketSize = 0x1,
	.bInterval = 0xc
};

static const struct usb_config_descriptor _hub_config_desc = {
	.bLength = sizeof(_hub_config_desc),
	.bDescriptorType = USB_DT_CONFIG,
	.wTotalLength = sizeof(_hub_config_desc) +
	sizeof(_hub_iface_desc)  +
	sizeof(_hub_endpoint_desc),
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = (1 << 7),
	.bMaxPower = 100/*mA*/ / 2
};

static struct usb_hub_descriptor _hub_hub_desc = {
	.bDescLength = 0x8,
	.bDescriptorType = USB_DT_HUB,
	.bNbrPorts = 0,
	.wHubCharacteristics = 0,
	.bPwrOn2PwrGood = 0xff,
	.bHubContrCurrent = 0,
/*	.portcfg = {0} */
};

struct sel4urbt {
	uintptr_t paddr;
	uint32_t param;
} __packed;

struct sel4urb {
	uint32_t epaddr;
	uint16_t max_pkt;
	uint16_t rate_ms;
	uint16_t urb_bytes_remaining;
	uint16_t nxact;
	struct sel4urbt desc[2];
} __packed;

struct vusb_ctrl_regs {
	uint32_t status;
	uint32_t req_reply;
	uint32_t intr;
	uint32_t notify;
	uint32_t cancel_transaction;
	uint32_t NbrPorts;
	struct usb_ctrlrequest req;
};

struct vusb_data_regs {
	struct sel4urb surb[MAX_ACTIVE_URB];
};

struct vhci_hcd {
	spinlock_t lock;
	struct vusb_ctrl_regs *ctrl_regs;
	struct vusb_data_regs *data_regs;
	struct urb *urb[MAX_ACTIVE_URB];
	int status[MAX_ACTIVE_URB];
	/* Represents the index of the first in flight urb */
	int surb_head;
	/* Represents the index of the next entry to attempt to consume */
	int surb_tail;
	struct list_head urb_overflow;
};

static inline void
surb_epaddr_change_state(struct sel4urb *u, uint32_t s)
{
	uint32_t e;
	e = u->epaddr;
	e &= ~SURB_EPADDR_STATE_MASK;
	e |= s;
	u->epaddr = e;
}


static struct sel4urb *surb_next_free(struct vhci_hcd *vhci, struct urb *urb)
{
	int start = vhci->surb_tail;
	struct sel4urb *surb = NULL;
	/* Since we may have an INT packet, we need to scan */
	do {
		uint32_t epaddr = vhci->data_regs->surb[vhci->surb_tail].epaddr;
		if (SURB_EPADDR_GET_STATE(epaddr) == SURB_EPADDR_STATE_INVALID) {
			surb = &vhci->data_regs->surb[vhci->surb_tail];
			vhci->urb[vhci->surb_tail] = urb;
			dvusb("Claiming SURB %d\n", vhci->surb_tail);
			return surb;
		}
		vhci->surb_tail = (vhci->surb_tail + 1) % MAX_ACTIVE_URB;
	} while (vhci->surb_tail != start);
	return NULL;
}

static int surb_next_complete(struct vhci_hcd *vhci)
{
	int start = vhci->surb_head;
	do {
		int cur_idx = vhci->surb_head;
		uint32_t status;
		vhci->surb_head = (vhci->surb_head + 1) % MAX_ACTIVE_URB;
		status = SURB_EPADDR_GET_STATE(vhci->data_regs->surb[cur_idx].epaddr);
		if (status & SURB_EPADDR_STATE_COMPLETE) {
			dvusb("Releasing SURB %d\n", cur_idx);
			return cur_idx;
		}
	} while (vhci->surb_head != start);
	return -1;
}

static inline struct vhci_hcd *hcd_to_vhci(struct usb_hcd *hcd)
{
	return (struct vhci_hcd *) (hcd->hcd_priv);
}

static inline struct usb_hcd *vhci_to_hcd(struct vhci_hcd *vhci)
{
	return container_of((void *) vhci, struct usb_hcd, hcd_priv);
}

static int __init vhci_hcd_init(void)
{
	if (usb_disabled())
		return -ENODEV;
	pr_info("%s: " DRIVER_DESC "\n", hcd_name);
	return platform_driver_register(&vhci_driver);
}
module_init(vhci_hcd_init);

static void __exit vhci_hcd_cleanup(void)
{
	note("%s:%d\n", __func__, __LINE__);
}
module_exit(vhci_hcd_cleanup);

static int vhci_run(struct usb_hcd *hcd)
{
	return 0;
}

int vhci_setup(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);

	spin_lock_init(&vhci->lock);
	vhci->ctrl_regs = hcd->regs + 0x1000;
	vhci->data_regs = hcd->regs;
	vhci->surb_head = 0;
	vhci->surb_tail = 0;
	INIT_LIST_HEAD(&vhci->urb_overflow);

	return 0;
}

static int urb_to_surb(struct urb *urb, struct sel4urb *surb, int data_toggle)
{
	uint32_t epaddr;
	switch (usb_pipetype(urb->pipe)) {
	case PIPE_CONTROL:
	case PIPE_INTERRUPT:
	case PIPE_BULK:
		break;
	case PIPE_ISOCHRONOUS:
		pr_err("Isochronos packets not supported\n");
	default:
		pr_err("Unknown pipe type\n");
		return -1;
	}

	surb->nxact = 0;
	if (urb->setup_packet) {
		uint32_t param;
		param = SURBT_PARAM_TYPE(2) | SURBT_PARAM_SIZE(8);
		surb->desc[0].paddr = urb->setup_dma;
		surb->desc[0].param = param;
		surb->nxact++;
	}
	if (urb->transfer_dma) {
		uint32_t param;
		param = SURBT_PARAM_TYPE(usb_pipein(urb->pipe) ? 0 : 1);
		param |= SURBT_PARAM_SIZE(urb->transfer_buffer_length);
		surb->desc[surb->nxact].paddr = urb->transfer_dma;
		surb->desc[surb->nxact].param = param;
		surb->nxact++;
	}

	surb->max_pkt = max_packet(usb_maxpacket(urb->dev, urb->pipe, usb_pipeout(urb->pipe)));
	surb->rate_ms = urb->interval;

	epaddr = SURB_EPADDR_ADDR(usb_pipedevice(urb->pipe));
	epaddr |= SURB_EPADDR_EP(usb_pipeendpoint(urb->pipe));
	epaddr |= SURB_EPADDR_SPEED(urb->dev->speed);
	if (urb->dev->tt) {
		epaddr |= SURB_EPADDR_HUB_ADDR(urb->dev->tt->hub->devnum);
		epaddr |= SURB_EPADDR_HUB_PORT(urb->dev->ttport);
	}
	if (data_toggle & 1)
		epaddr |= SURB_EPADDR_DT;
	surb->epaddr = epaddr;
	surb_epaddr_change_state(surb, SURB_EPADDR_STATE_PENDING);

	return 0;
}

static int surb_to_urb(struct sel4urb *surb, struct urb *urb, int cancel_status)
{
	int status;
	uint32_t surb_status = SURB_EPADDR_GET_STATE(surb->epaddr);
	switch (surb_status) {
	case SURB_EPADDR_STATE_PENDING:
	case SURB_EPADDR_STATE_ACTIVE:
		dvusb("INCOMPLETE URB\n");
		status = -EINPROGRESS;
		break;
	case SURB_EPADDR_STATE_SUCCESS:
		status = 0;
		if (urb->transfer_buffer)
			urb->actual_length = urb->transfer_buffer_length - surb->urb_bytes_remaining;
		else
			urb->actual_length = 0;
		break;
	case SURB_EPADDR_STATE_ERROR:
	case SURB_EPADDR_STATE_CANCELLED:
		urb->actual_length = 0;
		status = cancel_status;
		break;
	default:
		pr_err("Invalid URB state! 0x%x\n", surb_status);
		status = -EIO;
	}
	dvusb("Returning URB, status: 0x%x length %d actual length %d\n",
		surb_status, urb->transfer_buffer_length, urb->actual_length);
	return status;
}

static int
vhci_schedule_urb(struct vhci_hcd *vhci, struct usb_host_endpoint *ep)
{
	struct sel4urb *surb;
	struct urb *urb;

	urb = list_first_entry(&ep->urb_list, struct urb, urb_list);
	/* Grab a handle to a sel4 URB descriptor */
	surb = surb_next_free(vhci, urb);
	if (surb == NULL) {
		pr_debug("URB overlow in vhci driver\n");
		list_add_tail(&urb->urb_list, &vhci->urb_overflow);
		return -1;
	} else {
		/* Fill the sel4 URB descriptor */
		int dt = (int)urb->ep->hcpriv;
		int ret;
		urb->ep->hcpriv++;
		ret = urb_to_surb(urb, surb, dt);
		if (!ret)
#if 0
			seL4_Notify(5);
#else
			vhci->ctrl_regs->notify = 0;
#endif
		else
			pr_err("URB translation error\n");
		return ret;
	}
}

static irqreturn_t vhci_irq(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	unsigned long flags;

	spin_lock_irqsave(&vhci->lock, flags);
	vhci->ctrl_regs->intr = 0;
	while (1) {
		int idx;
		idx = surb_next_complete(vhci);

		if (idx < 0) {
			break;
		} else {
			struct sel4urb *surb = &vhci->data_regs->surb[idx];
			struct urb *urb = vhci->urb[idx];
			int status;
			int ret;
			status = surb_to_urb(surb, urb, vhci->status[idx]);
			if (status) {
				/* Untoggle */
				int new_hcpriv;
				new_hcpriv = (uint32_t)urb->ep->hcpriv;
				urb->ep->hcpriv = (void *)(new_hcpriv ^ 1);
				status = vhci->status[idx];
			}

			surb->epaddr = 0;
			vhci->urb[idx] = NULL;

			ret = usb_hcd_check_unlink_urb(hcd, urb, status);
			if (ret == 0) {
				struct usb_host_endpoint *ep = urb->ep;
				usb_hcd_unlink_urb_from_ep(hcd, urb);
				usb_hcd_giveback_urb(hcd, urb, status);
				/* If the list attached to this EP is not empty,
				 * we need to schedule the next */
				if (!list_empty(&ep->urb_list)) {
					int ret;
					ret = vhci_schedule_urb(vhci, ep);
					if (ret)
						dvusb("vhci Failed to chain URB\n");
				}
			} else
				pr_err("Failed to unlink urb!\n");
		}
	}
	spin_unlock_irqrestore(&vhci->lock, flags);
	return IRQ_HANDLED;
}



static int vhci_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	unsigned long flags;
	int ret;
	int ep_was_idle;

	dvusb("Enqueue URB @ %p : pipe 0x%x (%s %s %d:%d) rate %d | pkts %d\n",
			urb, urb->pipe, usb_pipein(urb->pipe) ? "IN" : "OUT",
			pipe_type_str(urb->pipe), usb_pipedevice(urb->pipe),
			usb_pipeendpoint(urb->pipe),
			urb->interval, urb->number_of_packets);
	if (urb->setup_packet) {
		dvusb("setup: 0x%02x 0x%02x 0x%02x%02x 0x%02x%02x 0x%02x%02x\n",
				urb->setup_packet[0], urb->setup_packet[1],
				urb->setup_packet[3], urb->setup_packet[2],
				urb->setup_packet[5], urb->setup_packet[4],
				urb->setup_packet[7], urb->setup_packet[6]);
	}
	dvusb("dma: 0x%x 0x%x (%d)\n", (uint32_t)urb->setup_dma,
		  (uint32_t)urb->transfer_dma, urb->transfer_buffer_length);

	spin_lock_irqsave(&vhci->lock, flags);

	ep_was_idle = list_empty(&urb->ep->urb_list);
	ret = usb_hcd_link_urb_to_ep(hcd, urb);
	if (ret)
		pr_err("link error\n");
	else if (ep_was_idle)
		/* seL4 data toggle management is incompatible with that of
		 * Linux. If there is already a queued transfer, simply add
		 * it to the list. It will be scheduled later with the correct
		 * explicit data toggle bit. */
		ret = vhci_schedule_urb(vhci, urb->ep);

	spin_unlock_irqrestore(&vhci->lock, flags);

	return ret;
}

static int vhci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	unsigned long flags;
	int i = 0;
	int ret = 0;
	printk(CFRED "Dequeue request for URB @ %p" CNORMAL "\n", urb);

	spin_lock_irqsave(&vhci->lock, flags);

	if (urb == list_first_entry(&urb->ep->urb_list, struct urb, urb_list)) {
		/* Cancel the transaction. An IRQ tells us when to clean up */
		for (i = 0; i < MAX_ACTIVE_URB; i++) {
			if (vhci->urb[i] == urb) {
				struct sel4urb *surb = &vhci->data_regs->surb[i];
				vhci->ctrl_regs->cancel_transaction = i;
				vhci->status[i] = status;
				break;
			}
		}
		if (i == MAX_ACTIVE_URB)
			pr_err("Could not find SURB for dequeue\n");
	} else {
		/* Not queued. Just unlink it */
		ret = usb_hcd_check_unlink_urb(hcd, urb, status);
		if (ret == 0) {
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			usb_hcd_giveback_urb(hcd, urb, status);
		} else
			pr_err("Failed to unlink urb!\n");
	}

	spin_unlock_irqrestore(&vhci->lock, flags);

	return ret;
}

static void
vhci_endpoint_reset(struct usb_hcd *hcd, struct usb_host_endpoint *ep)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	unsigned long flags;
//	note("%s:%d\n", __func__, __LINE__);
	spin_lock_irqsave(&vhci->lock, flags);
	ep->hcpriv = NULL;
	spin_unlock_irqrestore(&vhci->lock, flags);
}

static int
vhci_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	struct vusb_ctrl_regs *ctrl_regs;
	unsigned long flags;
	uint32_t port_sc;
	int ret;

	spin_lock_irqsave(&vhci->lock, flags);

	ctrl_regs = vhci->ctrl_regs;

	port_sc = ctrl_regs->status;
	ret = (ctrl_regs->NbrPorts > 7) ? 2 : 1;
	memcpy(buf, &port_sc, ret);

	spin_unlock_irqrestore(&vhci->lock, flags);

	return ret;
}

static int vhci_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char	*buf,
	u16		wLength
)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	struct vusb_ctrl_regs *ctrl_regs;
	unsigned long flags;
	int retval = -1;

	spin_lock_irqsave(&vhci->lock, flags);

	ctrl_regs = vhci->ctrl_regs;

	switch (typeReq & 0xff) {
	case 0x6: /* GET_DESCRIPTOR */
		switch (wValue >> 8) {
		case USB_DT_HUB:
			memcpy(buf, &_hub_hub_desc, wLength);
			((struct usb_hub_descriptor *)buf)->bNbrPorts = ctrl_regs->NbrPorts;
			retval = 0;
			break;
		default:
			dvusb("ERR: Unknown descriptor type 0x%x\n", wValue);
		}
		break;
	case 0x00: /* GET_STATUS */
	case 0x01: /* CLR_FEATURE */
	case 0x03: /* SET_FEATURE */
		ctrl_regs->req.bRequestType = typeReq >> 8;
		ctrl_regs->req.bRequest = typeReq >> 0;
		ctrl_regs->req.wValue = wValue;
		ctrl_regs->req.wIndex = wIndex;
		ctrl_regs->req.wLength = wLength;
		asm volatile("dsb");
		ctrl_regs->status = 1;
		DROOTHUB("hub ctrl> typeReq 0x%04x | wValue 0x%04x | wIndex 0x%04x | "
				 "wLength 0x%04x\n", typeReq, wValue, wIndex, wLength);
		if (wLength > 0) {
			uint32_t reply;
			int len;
			if (wLength > sizeof(reply))
				len = sizeof(reply);
			else
				len = wLength;
			reply = ctrl_regs->req_reply;
			memcpy(buf, &reply, len);
		}
		retval = 0;
		break;
	default:
		dvusb("ERR: Unknown request type 0x%x\n", typeReq);
	};

	spin_unlock_irqrestore(&vhci->lock, flags);

	return retval;
}

static void vhci_stop(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	(void)vhci;
	note("%s:%d\n", __func__, __LINE__);
}

static void vhci_shutdown(struct usb_hcd *hcd)
{
	(void)hcd;
	note("%s:%d\n", __func__, __LINE__);
}

static void
vhci_endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *ep)
{
	(void)hcd;
	(void)ep;
//	note("%s:%d\n", __func__, __LINE__);
}

static int vhci_bus_suspend(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	note("%s:%d\n", __func__, __LINE__);
	(void)vhci;
	return 0;
}

static int vhci_bus_resume(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	note("%s:%d\n", __func__, __LINE__);
	(void)vhci;
	return 0;
}


static void vhci_relinquish_port(struct usb_hcd *hcd, int portnum)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	note("%s:%d\n", __func__, __LINE__);
	(void)vhci;
}

static int vhci_port_handed_over(struct usb_hcd *hcd, int portnum)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	note("%s:%d\n", __func__, __LINE__);
	(void)vhci;
	return 0;
}

static void vhci_remove_device(struct usb_hcd *hcd, struct usb_device *udev)
{
	(void)hcd;
	(void)udev;
	note("%s:%d\n", __func__, __LINE__);
}

static int vhci_get_frame(struct usb_hcd *hcd)
{
	(void)hcd;
	note("%s:%d\n", __func__, __LINE__);
	return 0;
}

static const struct hc_driver __read_mostly vhci_hc_driver = {
	.description   =	hcd_name,
	.product_desc  =	"VHCI Host Controller",
	.hcd_priv_size =	sizeof(struct vhci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq		   =	vhci_irq,
	.flags		 =	HCD_MEMORY | HCD_USB2 | HCD_BH,

	/*
	 * basic lifecycle operations
	 */
	.reset		 =	vhci_setup,
	.start		 =	vhci_run,
	.stop		  =	vhci_stop,
	.shutdown	  =	vhci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue	  = vhci_urb_enqueue,
	.urb_dequeue	  = vhci_urb_dequeue,
	.endpoint_disable = vhci_endpoint_disable,
	.endpoint_reset   = vhci_endpoint_reset,
	.clear_tt_buffer_complete = NULL,

	/*
	 * scheduling support
	 */
	.get_frame_number = vhci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data  =	vhci_hub_status_data,
	.hub_control	  =	vhci_hub_control,
	.bus_suspend	  =	vhci_bus_suspend,
	.bus_resume	   =	vhci_bus_resume,
	.relinquish_port  =	vhci_relinquish_port,
	.port_handed_over =	vhci_port_handed_over,

	/*
	 * device support
	 */
	.free_dev =	 vhci_remove_device,
};

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");


static int sel4_vhci_probe(struct platform_device *pdev)
{
	struct vhci_hcd *vhci;
	struct usb_hcd *hcd;
	struct resource *res;
	int irq;
	int err;

	hcd = usb_create_hcd(&vhci_hc_driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		return -ENOMEM;
	}
	vhci = hcd_to_vhci(hcd);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get I/O memory\n");
		err = -ENXIO;
		goto fail;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = devm_ioremap(&pdev->dev, res->start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "Failed to remap I/O memory\n");
		err = -ENOMEM;
		goto fail;
	}

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		err = -ENODEV;
		goto fail;
	}

	err = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (err) {
		dev_err(&pdev->dev, "Failed to add USB HCD\n");
		goto fail;
	}
	device_wakeup_enable(hcd->self.controller);

	platform_set_drvdata(pdev, hcd);

	return 0;

fail:
	usb_put_hcd(hcd);
	return err;
}

static int sel4_vhci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	hcd = platform_get_drvdata(pdev);
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	return 0;
}

static const struct dev_pm_ops vhci_pm_ops = {
	.suspend	= NULL,
	.resume		= NULL,
};

#ifdef CONFIG_OF
static const struct of_device_id sel4_vhci_match[] = {
	{ .compatible = "sel4,vhci" },
	{ .compatible = "sel4,vhci" },
	{},
};
MODULE_DEVICE_TABLE(of, sel4_vhci_match);
#endif

static struct platform_driver vhci_driver = {
	.probe	  = sel4_vhci_probe,
	.remove	  = sel4_vhci_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver = {
		.name	= "sel4-vhci",
		.owner   = THIS_MODULE,
		.pm	  = &vhci_pm_ops,
		.of_match_table = of_match_ptr(sel4_vhci_match),
	}
};

