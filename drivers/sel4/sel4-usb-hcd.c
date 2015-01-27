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

//#define DEBUG_ROOTHUB

#define max_packet(wMaxPacketSize) ((wMaxPacketSize) & 0x07ff)

#define CNORMAL   "\033[0m"

#define CFRED	 "\033[31m"
#define CFGREEN   "\033[32m"
#define CFYELLOW  "\033[33m"
#define CFBLUE	"\033[34m"
#define CFMAGENTA "\033[35m"
#define CFCYAN	"\033[36m"
#define CFWHITE   "\033[37m"

#define CBRED	 "\033[41m"
#define CBGREEN   "\033[42m"
#define CBYELLOW  "\033[43m"
#define CBBLUE	"\033[44m"
#define CBMAGENTA "\033[45m"
#define CBCYAN	"\033[46m"
#define CBWHITE   "\033[47m"



#define dprintf(...)						\
	do {									\
		printk(CFYELLOW "SEL4USB:" __VA_ARGS__); \
		printk(CNORMAL);					\
	} while (0)

#define ddprintf(...)						\
	do {									\
		printk(CFCYAN "SEL4USB:" __VA_ARGS__); \
		printk(CNORMAL);					\
	} while (0)

#ifdef DEBUG_ROOTHUB
#define DROOTHUB(...) printk(CFYELLOW "SEL4USB root hub: " __VA_ARGS__);
#else
#define DROOTHUB(...) do {} while (0)
#endif

const char* pipe_type_str(unsigned int pipe)
{
	switch (usb_pipetype(pipe)) {
	case PIPE_ISOCHRONOUS: return "isoc";
	case PIPE_INTERRUPT:   return "int ";
	case PIPE_CONTROL:	 return "ctrl";
	case PIPE_BULK:		return "bulk";
	default:			   return "????";
	}
}


#define dputs(s)		seL4_DebugPuts(CFCYAN "SEL4USB:" s CNORMAL)

#define poke() ddprintf("%s:%d\n", __func__, __LINE__)

#define DRIVER_AUTHOR "Alexander Kroh"
#define DRIVER_DESC "USB 2.0 'Virtualised' Host Controller (VHCI) Driver"

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

struct vusb_ctrl_regs {
	uint32_t status;
	uint32_t req_reply;
	uint32_t NbrPorts;
	struct usb_ctrlrequest req;
};

struct vhci_hcd {
	spinlock_t lock;
};

struct sel4urbt {
	uintptr_t paddr;
	int size;
	int type;
	int res;
} __packed;

struct sel4urb {
	uint8_t addr;
	uint8_t hub_addr;
	uint8_t hub_port;
	uint8_t speed;
	uint16_t ep;
	uint16_t max_pkt;
	uint16_t rate_ms;
	uint16_t nxact;
	void    *token;
	uint32_t urb_status;
	struct sel4urbt desc[2];
} __packed;


static inline struct vhci_hcd *hcd_to_vhci(struct usb_hcd *hcd)
{
	return (struct vhci_hcd *) (hcd->hcd_priv);
}

static inline struct usb_hcd *vhci_to_hcd(struct vhci_hcd *vhci)
{
	return container_of((void *) vhci, struct usb_hcd, hcd_priv);
}

static inline struct usb_device *ep_to_udev(struct usb_host_endpoint *ep)
{
	return (struct usb_device *)ep->hcpriv;
}

static int __init vhci_hcd_init(void)
{
	if (usb_disabled())
		return -ENODEV;
	poke();
	dprintf(KERN_INFO "%s: " DRIVER_DESC "\n", hcd_name);
	return platform_driver_register(&vhci_driver);
}
module_init(vhci_hcd_init);

static void __exit vhci_hcd_cleanup(void)
{
	poke();
}
module_exit(vhci_hcd_cleanup);

static void vhci_stop(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	(void)vhci;
	poke();
}

static int vhci_run(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	(void)vhci;
	poke();
	hcd->uses_new_polling = 0;
	hcd->self.root_hub->parent = 0;
	return 0;
}

int vhci_setup(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);

	spin_lock_init(&vhci->lock);

	poke();
	return 0;
}

static void vhci_shutdown(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	poke();
	(void)vhci;
}

static void
vhci_endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *ep)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	poke();
	(void)vhci;
}

static void
vhci_endpoint_reset(struct usb_hcd *hcd, struct usb_host_endpoint *ep)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	int epnum = usb_endpoint_num(&ep->desc);
	int is_out = usb_endpoint_dir_out(&ep->desc);
	struct usb_device *udev = ep_to_udev(ep);
	unsigned long flags;
	poke();
	spin_lock_irqsave(&vhci->lock, flags);
	(void)epnum;
	(void)is_out;
	dprintf("ep reset:: 0x%x\n", (uint32_t)udev);
	//usb_settoggle(udev, epnum, is_out, 0);
	spin_unlock_irqrestore(&vhci->lock, flags);
}

static void vhci_remove_device(struct usb_hcd *hcd, struct usb_device *udev)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	poke();
	(void)vhci;
}

static int vhci_get_frame(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	poke();
	(void)vhci;
	return 0;
}



struct urb *_cur_urb = NULL;

static int urb_to_surb(struct urb *urb, struct sel4urb *surb)
{
	switch (usb_pipetype(urb->pipe)) {
	case PIPE_CONTROL:
	case PIPE_INTERRUPT:
	case PIPE_ISOCHRONOUS:
		break;
	default:
		dprintf("blah\n");
	}

	surb->addr = usb_pipedevice(urb->pipe);
	if (urb->dev->tt) {
		surb->hub_addr = urb->dev->tt->hub->devnum;
		surb->hub_port = urb->dev->ttport;
	} else {
		surb->hub_addr = 0;
		surb->hub_port = 0;
	}
	surb->speed = urb->dev->speed;
	surb->ep = usb_pipeendpoint(urb->pipe);
	surb->max_pkt = max_packet(usb_maxpacket(urb->dev, urb->pipe, usb_pipeout(urb->pipe)));
	surb->rate_ms = urb->interval;
	surb->token = urb;
	surb->urb_status = (1u << 31);

	surb->nxact = 0;
	if (urb->setup_packet) {
		surb->nxact++;
		surb->desc[0].paddr = urb->setup_dma;
		surb->desc[0].size = 8;
		surb->desc[0].type = -1;
	}
	if (urb->transfer_dma) {
		surb->desc[surb->nxact].paddr = urb->transfer_dma;
		surb->desc[surb->nxact].size = urb->transfer_buffer_length;
		surb->desc[surb->nxact].type = usb_pipein(urb->pipe) ? 0 : 1;
		surb->nxact++;
	}
	return 0;
}

static int surb_to_urb(struct sel4urb *surb, struct urb *urb)
{
	if (surb->urb_status == (1u << 31)) {
		dprintf("INCOMPLETE URB\n");
		return 0;
	}
	if (urb->transfer_buffer) {
		urb->actual_length = urb->transfer_buffer_length;
		dprintf("Returning URB, status: 0x%x length 0x%x\n", surb->urb_status, urb->actual_length);
	} else {
		urb->actual_length = 0;
	}
	return 0;
}

static irqreturn_t vhci_irq(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	unsigned long flags;
	struct sel4urb *surb;
	struct urb *urb;
	int status = 0;
	int ret;

	spin_lock_irqsave(&vhci->lock, flags);

	surb = (struct sel4urb *)hcd->regs;

	urb = _cur_urb;
	_cur_urb = NULL;

	if (urb == NULL) {
		dprintf("vhci IRQ: NO PENDING URB\n");
		return IRQ_HANDLED;
	} else {
		ret = surb_to_urb(surb, urb);
		if (ret)
			dprintf("vhci IRQ: Failed to finalise URB!\n");

		dprintf("vhci IRQ: Return URB @ 0x%0x\n", (uint32_t)urb);
		usb_hcd_unlink_urb_from_ep(hcd, urb);
		usb_hcd_giveback_urb(hcd, urb, status);
	}

	spin_unlock_irqrestore(&vhci->lock, flags);

	return IRQ_HANDLED;
}

static int vhci_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	unsigned long flags;
	struct sel4urb *surb;
	int ret;

	if (urb->interval) {
		dprintf("WARNING: ignoring INT packets for now\n");
		return 0;
	}

	spin_lock_irqsave(&vhci->lock, flags);
	surb = (struct sel4urb *)hcd->regs;
	ret = urb_to_surb(urb, surb);
	if (ret) {
		dprintf("AAAAH!\n");
		return -1;
	}

	dprintf("Enqueue URB @ 0x%x : pipe 0x%x (%s %s %d:%d) interval %d | npackets %d\n", (uint32_t)urb,
			urb->pipe, usb_pipein(urb->pipe) ? "IN" : "OUT",
			pipe_type_str(urb->pipe), usb_pipedevice(urb->pipe), usb_pipeendpoint(urb->pipe),
			urb->interval, urb->number_of_packets);
	if (urb->setup_packet) {
		dprintf("setup: 0x%02x 0x%02x 0x%02x%02x 0x%02x%02x 0x%02x%02x\n",
				urb->setup_packet[0], urb->setup_packet[1],
				urb->setup_packet[3], urb->setup_packet[2],
				urb->setup_packet[5], urb->setup_packet[4],
				urb->setup_packet[7], urb->setup_packet[6]);
	}
	dprintf("setup dma: 0x%x 0x%x (%d)\n", (uint32_t)urb->setup_dma, (uint32_t)urb->transfer_dma, urb->transfer_buffer_length);

	if (_cur_urb != NULL) {
		dprintf("ERROR: Multipe packets in flight\n");
		while (1);
	}

	_cur_urb = urb;
	ret = usb_hcd_link_urb_to_ep(hcd, urb);
	if (ret)
		dprintf("link error\n");

	spin_unlock_irqrestore(&vhci->lock, flags);
	seL4_Notify(5);

	//usb_settoggle (urb->dev, usb_pipeendpoint (urb->pipe), !is_input, 1);
	//if (unlikely(!usb_gettoggle(qh->ps.udev, epnum, is_out))) {

	return 0;
}

static int vhci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	unsigned long flags;
	int ret;
	poke();

	spin_lock_irqsave(&vhci->lock, flags);

	dprintf("Cancel URB @ 0x%0x, we have 0x%x\n", (uint32_t)urb, (uint32_t)_cur_urb);
	ret = usb_hcd_check_unlink_urb(hcd, urb, status);
	if (ret == 0) {
		usb_hcd_unlink_urb_from_ep(hcd, urb);
		usb_hcd_giveback_urb(hcd, urb, status);
	}

	spin_unlock_irqrestore(&vhci->lock, flags);

	return ret;
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

	ctrl_regs = (struct vusb_ctrl_regs *)(hcd->regs + 0x1000);

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

	ctrl_regs = (struct vusb_ctrl_regs *)(hcd->regs + 0x1000);

	switch (typeReq & 0xff) {
	case 0x6: /* GET_DESCRIPTOR */
		switch (wValue >> 8) {
		case USB_DT_HUB:
			memcpy(buf, &_hub_hub_desc, wLength);
			((struct usb_hub_descriptor *)buf)->bNbrPorts = ctrl_regs->NbrPorts;
			retval = 0;
			break;
		default:
			dprintf("ERR: Unknown descriptor type 0x%x\n", wValue);
		}
	case 0x00: /* GET_STATUS */
	case 0x01: /* CLR_FEATURE */
	case 0x03: /* SET_FEATURE */
		ctrl_regs->req.bRequestType = typeReq >> 8;
		ctrl_regs->req.bRequest = typeReq >> 0;
		ctrl_regs->req.wValue = wValue;
		ctrl_regs->req.wIndex = wIndex;
		ctrl_regs->req.wLength = wLength;
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
		dprintf("ERR: Unknown request type 0x%x\n", typeReq);
	};

	spin_unlock_irqrestore(&vhci->lock, flags);

	return retval;
}

static int vhci_bus_suspend(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	poke();
	(void)vhci;
	return 0;
}

static int vhci_bus_resume(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	poke();
	(void)vhci;
	return 0;
}


static void vhci_relinquish_port(struct usb_hcd *hcd, int portnum)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	poke();
	(void)vhci;
}

static int vhci_port_handed_over(struct usb_hcd *hcd, int portnum)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	poke();
	(void)vhci;
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

	poke();

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
	return 0;
	poke();
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
	.probe	   = sel4_vhci_probe,
	.remove	  = sel4_vhci_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver = {
		.name	= "sel4-vhci",
		.owner   = THIS_MODULE,
		.pm	  = &vhci_pm_ops,
		.of_match_table = of_match_ptr(sel4_vhci_match),
	}
};

