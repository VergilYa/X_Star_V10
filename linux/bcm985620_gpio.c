/*
 * bcm985620_gpio.c -- char device interface for mpc8309 gpio 
 *
 * Copyright (C) 2011 Broadcom ltd.
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <asm/qe.h>
#include <asm/uaccess.h>
#include <linux/of_platform.h>
#include <sysdev/fsl_soc.h>
#include <linux/interrupt.h>

#include <linux/bcm985620_gpio.h>
#include <platforms/83xx/mpc8309_bcm985620.h>

/* Definitions */

static struct gpio985620_data {
	dev_t             devt;
	int	              is_probed;
	struct mpc8309_gpio __iomem *regs[2];
	struct mpc8309_gpio shadow[2];
	int               irq;
	int               irqref;
} gpio_priv_data = {
	.devt = 0,
	.regs = {NULL, NULL},
	.shadow = {{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}},
	.is_probed = 0,
	.irq = -1,
	.irqref = 0,
};

static DEFINE_MUTEX(gpioint_list_lock);
static LIST_HEAD(gpioint_list);
struct gpio985620_interrupt {
	struct list_head list;
	int gpioline;
	int irq;
	irq_handler_t handler;
	void* dev_id;
	const char *device_type;
};

#define DRV_NAME "bcm985620_mpc-gpio"


#define GPIO_BIT(pin) (0x1 << (31 - (pin % 32)))

#define GPIO_SET_BIT(reg, gpio) \
	{ \
		volatile __be32 __val;   \
		__val = in_be32(&reg);   \
		__val |= GPIO_BIT(gpio); \
		out_be32(&reg, __val);   \
	}

#define GPIO_CLR_BIT(reg, gpio) \
	{ \
		volatile __be32 __val;   \
		__val = in_be32(&reg);   \
		__val &= ~GPIO_BIT(gpio);\
		out_be32(&reg, __val);   \
	}

#define gpio985620_do_write(...) 0
#define gpio985620_do_read(...) 0

static struct mpc8309_gpio __iomem *__gpio985620_get_regs(u8 pin)
{
	if (pin >= MPC8309_NUMBER_OF_GPIOS) return NULL;
	if (pin >= 32) 
		return gpio_priv_data.regs[1];
	return gpio_priv_data.regs[0];
}

static struct mpc8309_gpio *__gpio985620_get_shadow(u8 pin)
{
	if (pin >= MPC8309_NUMBER_OF_GPIOS) return NULL;
	if (pin >= 32) 
		return &gpio_priv_data.shadow[1];
	return &gpio_priv_data.shadow[0];
}

static int __gpio985620_get_pin_direction(u8 pin)
{
	int rc;
	struct mpc8309_gpio *shadow = __gpio985620_get_shadow(pin);
	if (!shadow) {
		rc = -ENXIO;
	} else if (shadow->gpio_dir & GPIO_BIT(pin)) {
		rc = MPC8309_GPIO_DIR_OUTPUT;
	} else {
		rc = MPC8309_GPIO_DIR_INPUT;
	}
	return rc;	
}

static int gpio985620_assert_pin(u8 pin)
{
	int rc;
	struct mpc8309_gpio __iomem *regs = __gpio985620_get_regs(pin);
	struct mpc8309_gpio *shadow = __gpio985620_get_shadow(pin);
	if (!regs || !shadow) {
		rc = -ENXIO;
	} else {
		rc = __gpio985620_get_pin_direction(pin);
		if (rc == MPC8309_GPIO_DIR_INPUT) {
			rc = -EINVAL;
		} else if (rc == MPC8309_GPIO_DIR_OUTPUT) {
			shadow->gpio_dat |= GPIO_BIT(pin);
			out_be32(&regs->gpio_dat, shadow->gpio_dat);
			rc = 0;
		}
	}
	return rc;
}

static int gpio985620_deassert_pin(u8 pin)
{
	int rc;
	struct mpc8309_gpio *shadow = __gpio985620_get_shadow(pin);
	struct mpc8309_gpio __iomem *regs = __gpio985620_get_regs(pin);
	if (!regs || !shadow) {
		rc = -ENXIO;
	} else {
		rc = __gpio985620_get_pin_direction(pin);
		if (rc == MPC8309_GPIO_DIR_INPUT) {
			rc = -EINVAL;
		} else if (rc == MPC8309_GPIO_DIR_OUTPUT) {
			shadow->gpio_dat &= ~(GPIO_BIT(pin));
			out_be32(&regs->gpio_dat, shadow->gpio_dat);
			rc = 0;
		}
	}
	return rc;
}

static int gpio985620_poll_pin(u8 pin)
{
	int rc;
	__be32 reg_save;
	struct mpc8309_gpio *shadow = __gpio985620_get_shadow(pin);
	struct mpc8309_gpio __iomem *regs = __gpio985620_get_regs(pin);
	if (!regs || !shadow) {
		return -ENXIO;
	}
	rc = __gpio985620_get_pin_direction(pin);
	if (rc == MPC8309_GPIO_DIR_OUTPUT) {
		rc = -EINVAL;
	} else if (rc == MPC8309_GPIO_DIR_INPUT) {
		reg_save = in_be32(&regs->gpio_dat);
		rc = (reg_save & GPIO_BIT(pin)) ? 1 : 0;
		if (rc) {
			shadow->gpio_dat |= GPIO_BIT(pin);
		} else {
			shadow->gpio_dat &= ~(GPIO_BIT(pin));
		}
	}
	return rc;
}

static int gpio985620_set_dir_in(u8 pin)
{
	struct mpc8309_gpio *shadow = __gpio985620_get_shadow(pin);
	struct mpc8309_gpio __iomem *regs = __gpio985620_get_regs(pin);
	if (!regs || !shadow) {
		return -ENXIO;
	}
	shadow->gpio_dir &= ~(GPIO_BIT(pin));
	out_be32(&regs->gpio_dir, shadow->gpio_dir);
	return 0;
}

static int gpio985620_set_dir_out(u8 pin)
{
	struct mpc8309_gpio *shadow = __gpio985620_get_shadow(pin);
	struct mpc8309_gpio __iomem *regs = __gpio985620_get_regs(pin);
	if (!regs || !shadow) {
		return -ENXIO;
	}
	shadow->gpio_dir |= GPIO_BIT(pin);
	out_be32(&regs->gpio_dir, shadow->gpio_dir);
	return 0;
}

static ssize_t gpio985620_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	int ret;
	if (!gpio_priv_data.is_probed) {
		ret = -ENODEV;
	} else if (access_ok(VERIFY_WRITE, (void __user *)buf, count)) {
		ret = gpio985620_do_write(buf, count);
	} else {
		ret = -EFAULT;
	}
	return count;
	return ret;
}

static ssize_t gpio985620_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int ret;
	if (!gpio_priv_data.is_probed) {
		ret = -ENODEV;
	} else if (access_ok(VERIFY_READ, (void __user *)buf, count)) {
		ret = gpio985620_do_read(buf, count);
	} else {
		ret = -EFAULT;
	}
	return ret;
}

static long gpio985620_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int ret = 0;
	u8 pin;

	if (!gpio_priv_data.is_probed)
		return -ENODEV;

	if (_IOC_TYPE(cmd) != BCM985620_GPIO_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	switch (cmd) {
		
	case BCM985620_GPIO_IOC_DAT_ASSERT:
		pin = (u8)arg;
		ret = gpio985620_assert_pin(pin);
	break;
	
	case BCM985620_GPIO_IOC_DAT_DEASSERT:
		pin = (u8)arg;
		ret = gpio985620_deassert_pin(pin);
	break;

	case BCM985620_GPIO_IOC_DAT_POLL:
		pin = (u8)arg;
		ret = gpio985620_poll_pin(pin);
	break;

	case BCM985620_GPIO_IOC_DIR_SET_OUT:
		pin = (u8)arg;
		ret = gpio985620_set_dir_out(pin);
	break;
	
	case BCM985620_GPIO_IOC_DIR_SET_IN:
		pin = (u8)arg;
		ret = gpio985620_set_dir_in(pin);
	break;

	}

	return ret;
}

static int gpio985620_open(struct inode *inode, struct file *filp)
{
	if (!gpio_priv_data.is_probed)
		return -ENODEV;

	if (inode->i_rdev != gpio_priv_data.devt) {
		pr_debug("gpio985620: nothing for minor %d\n", iminor(inode));
		return -EFAULT;
	}

	nonseekable_open(inode, filp);
	return 0;
}

static int gpio985620_release(struct inode *inode, struct file *filp)
{
	if (!gpio_priv_data.is_probed)
		return -ENODEV;
	return 0;
}

static const struct file_operations gpio985620_fops = {
	.owner =	THIS_MODULE,
	.write =	gpio985620_write,
	.read =		gpio985620_read,
	.unlocked_ioctl = gpio985620_ioctl,
	.open =		gpio985620_open,
	.release =	gpio985620_release,
};

/*-------------------------------------------------------------------------*/

static irqreturn_t gpio985620_gpio_isr(int irq, void * dev_id)
{
	struct gpio985620_interrupt *gpioint, *ptmp;
	irq_handler_t handler;

	handler = NULL;

	mutex_lock(&gpioint_list_lock);

	list_for_each_entry_safe(gpioint, ptmp, &gpioint_list, list) {
		/* ack IRQ */
		if (gpioint->handler && gpio985620_gpio_clear_pending_irq(gpioint->irq) == 0) {

			/* check level - act only on active high  */
			if (gpio985620_poll_pin(gpioint->irq) > 0) {
				handler = gpioint->handler;
				break;
			}
		}
	}
	mutex_unlock(&gpioint_list_lock);

	if (!handler) {
		return NO_IRQ;
	}

	bcm985620_gpio_mask_irq(gpioint->irq);

	return handler(gpioint->irq, gpioint->dev_id);
}

/*-------------------------------------------------------------------------*/

int gpio985620_gpio_assert(unsigned char pin)
{
	if (!gpio_priv_data.is_probed)
		return -ENODEV;
	return gpio985620_assert_pin(pin);
}
EXPORT_SYMBOL(gpio985620_gpio_assert);

int gpio985620_gpio_deassert(unsigned char pin)
{
	if (!gpio_priv_data.is_probed)
		return -ENODEV;
	return gpio985620_deassert_pin(pin);
}
EXPORT_SYMBOL(gpio985620_gpio_deassert);

int gpio985620_gpio_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags, const char *name, void *dev_id)
{
	struct gpio985620_interrupt *gpioint;
	struct gpio985620_interrupt *ptmp;
	int match, ret;

	if (!gpio_priv_data.is_probed)
		return -ENODEV;

	/* check if irq is already requested */
	match = 0;
	gpioint = NULL;

	mutex_lock(&gpioint_list_lock);

	list_for_each_entry_safe(gpioint, ptmp, &gpioint_list, list) {
		if (irq == gpioint->irq) {
			match = 1;
			break;
		}
	}

	if (!match) {
		ret = -ENODEV;
		goto gpio985620_gpio_req_irq_UNLOCK;
	}

	if (gpioint->handler) {
		ret = -EBUSY;
		goto gpio985620_gpio_req_irq_UNLOCK;
	}
	

	gpioint->handler = handler;
	gpioint->dev_id = dev_id;

	if (gpio_priv_data.irqref == 0) {

		ret = request_irq(gpio_priv_data.irq, gpio985620_gpio_isr, IRQF_DISABLED, DRV_NAME, &gpio_priv_data);

		if (ret < 0) {
			pr_err(DRV_NAME " request_irq(irq = %d) failed (%d)\n", gpio_priv_data.irq, ret);
			goto gpio985620_gpio_req_irq_CLEAR;
		}
	}

	gpio_priv_data.irqref++;

	mutex_unlock(&gpioint_list_lock);

	gpio985620_gpio_clear_pending_irq(irq);
	bcm985620_gpio_unmask_irq(irq);

	ret = 0;
	goto gpio985620_gpio_req_irq_RETURN;

gpio985620_gpio_req_irq_CLEAR:

	gpioint->dev_id = NULL;
	gpioint->handler = NULL;

gpio985620_gpio_req_irq_UNLOCK:

	mutex_unlock(&gpioint_list_lock);

gpio985620_gpio_req_irq_RETURN:

	return ret;
}
EXPORT_SYMBOL(gpio985620_gpio_request_irq);

int gpio985620_gpio_release_irq(unsigned int irq, void *dev_id)
{
	struct gpio985620_interrupt *gpioint;
	struct gpio985620_interrupt *ptmp;
	int match, ret;

	if (!gpio_priv_data.is_probed)
		return -ENODEV;

	match = 0;
	gpioint = NULL;

	mutex_lock(&gpioint_list_lock);

	list_for_each_entry_safe(gpioint, ptmp, &gpioint_list, list) {
		if (irq == gpioint->irq) {
			match = 1;
			break;
		}
	}

	if (!match) {
		ret = -ENODEV;
		goto gpio985620_gpio_rel_irq_RETURN;
	}

	if (gpioint->handler == NULL) {
		ret = -ENODEV;
		goto gpio985620_gpio_rel_irq_RETURN;
	}

	gpioint->dev_id = NULL;
	gpioint->handler = NULL;

	if (gpio_priv_data.irqref) {
		gpio_priv_data.irqref--;
	}

	if (gpio_priv_data.irqref == 0) {
		free_irq(gpio_priv_data.irq, &gpio_priv_data);
	}
	ret = 0;

gpio985620_gpio_rel_irq_RETURN:

	mutex_unlock(&gpioint_list_lock);
	return ret;
}
EXPORT_SYMBOL(gpio985620_gpio_release_irq);

int gpio985620_gpio_config_irq(unsigned char pin, unsigned int mode)
{
	int rc;
	struct mpc8309_gpio __iomem *regs = __gpio985620_get_regs(pin);
	if (!gpio_priv_data.is_probed)
		return -ENODEV;
	if (!regs) {
		rc = -ENXIO;
	} else {
		rc = __gpio985620_get_pin_direction(pin);
		if (rc != MPC8309_GPIO_DIR_INPUT) {
			rc = -EINVAL;
		} else {
			if (mode == GPIO985620_GPIO_IRQ_HIGH_TO_LOW) {
				GPIO_SET_BIT(regs->gpio_icr, pin);
				rc = 0;
			} else if (mode == GPIO985620_GPIO_IRQ_ANY) {
				GPIO_CLR_BIT(regs->gpio_icr, pin);
				rc = 0;
			} else {
				rc = -ERANGE;
			}
		}
	}
	return rc;
}
EXPORT_SYMBOL(gpio985620_gpio_config_irq);

int gpio985620_gpio_clear_pending_irq(unsigned char pin)
{
	int rc;
	struct mpc8309_gpio __iomem *regs = __gpio985620_get_regs(pin);
	volatile __be32 val; 
	__be32 bit; 
	if (!gpio_priv_data.is_probed) {
		rc = -ENODEV;
	} else if (!regs) {
		rc = -ENXIO;
	} else {
		val = in_be32(&regs->gpio_ier);   
		bit = GPIO_BIT(pin);
		if (!(val & bit)) {
			/* there was no interrupt */
			rc = -EAGAIN;
		} else {
			/* clear interrupt */
			out_be32(&regs->gpio_ier, bit);
			rc = 0;
		}
	}
	return rc;
}
EXPORT_SYMBOL(gpio985620_gpio_clear_pending_irq);

int gpio985620_gpio_do_mask_irq(unsigned char pin, int do_mask)
{
	int rc;
	struct mpc8309_gpio __iomem *regs = __gpio985620_get_regs(pin);
	if (!gpio_priv_data.is_probed)
		return -ENODEV;
	if (!regs) {
		rc = -ENXIO;
	} else {
		rc = __gpio985620_get_pin_direction(pin);
		if (rc != MPC8309_GPIO_DIR_INPUT) {
			rc = -EINVAL;
		} else {
			if (do_mask) {
				GPIO_CLR_BIT(regs->gpio_imr, pin);
			} else {
				GPIO_SET_BIT(regs->gpio_imr, pin);
			} 
			rc = 0;
		}
	}
	return rc;
}
EXPORT_SYMBOL(gpio985620_gpio_do_mask_irq);

void* gpio985620_gpio_get_next_irq(void *info, int *irq, const char** devname)
{
	struct gpio985620_interrupt *gpioint = (struct gpio985620_interrupt*)info;
	struct gpio985620_interrupt *ptmp;
	int last = 1;

	mutex_lock(&gpioint_list_lock);
	if (gpioint == NULL) {
		list_for_each_entry_safe(gpioint, ptmp, &gpioint_list, list) {
			last = 0;
			break;
		}
	} else {
		list_for_each_entry_safe_continue(gpioint, ptmp, &gpioint_list, list) {
			last = 0;
			break;
		}
	}
	mutex_unlock(&gpioint_list_lock);
	if (gpioint) {
 		if (irq) *irq = gpioint->irq;
		if (devname) *devname = gpioint->device_type;
	}
	return last ? NULL : (void*)gpioint;
}
EXPORT_SYMBOL(gpio985620_gpio_get_next_irq);

/* -------------------------------------------------------------------- */

static int __devexit bcm985620gpio_remove(struct of_device *op)
{
	static struct gpio985620_interrupt *gpioint, *ptmp;
	int i; 

	mutex_lock(&gpioint_list_lock);
	list_for_each_entry_safe(gpioint, ptmp, &gpioint_list, list) {
		kfree(gpioint);	
	}
	mutex_unlock(&gpioint_list_lock);

	for (i = 0; i < (sizeof(gpio_priv_data.regs) / sizeof(gpio_priv_data.regs[0])); i++) {
		if(gpio_priv_data.regs[i]) {
			iounmap(gpio_priv_data.regs[i]);
			gpio_priv_data.regs[i] = NULL;
		}
		memset(&gpio_priv_data.shadow[i], 0, sizeof(gpio_priv_data.shadow[i]));
	}

	if (gpio_priv_data.irq != -1) {
		irq_dispose_mapping(gpio_priv_data.irq);
		gpio_priv_data.irq = -1;
	}

	return 0;
}

static int __devinit bcm985620gpio_probe(struct of_device *op, const struct of_device_id *match)
{
	static struct gpio985620_interrupt *gpioint, *ptmp;
	struct device_node *cnp; /* node pointer and child node pointer */
	const void *prop;
	struct resource res;
	int i, ret, len;

	memset(&res, 0, sizeof(res));
	i = ret = 0;

	prop = of_get_property(op->node, "cell-index", NULL);
	if (prop) {
		i = (*(u32 *)prop);
	}

	if (i < 0 || i >= (sizeof(gpio_priv_data.regs) / sizeof(gpio_priv_data.regs[0]))) {
		pr_err(DRV_NAME " Could not find cell-index (%d)", i);
		ret = -EINVAL;
		goto probe985620gpio_DO_UNMAP;
	}

	gpio_priv_data.regs[i] = NULL;
	gpio_priv_data.regs[i] = of_iomap(op->node, 0);
	if (!gpio_priv_data.regs[i]) {
		pr_err(DRV_NAME " of_iomap() failed\n");
		ret = -ENOMEM;
		goto probe985620gpio_DO_UNMAP;
	}

	if (gpio_priv_data.irq == -1) {
		ret = of_irq_to_resource(op->node, 0, &res);
		if (!ret) {
			pr_err(DRV_NAME " failed no map irq (%d)\n", ret);
			ret = -EINVAL;
			goto probe985620gpio_DO_UNMAP;
		}
		gpio_priv_data.irq = res.start;
	}

	for_each_child_of_node(op->node, cnp) {

			gpioint = kzalloc(sizeof(*gpioint), GFP_KERNEL);
			if (!gpioint) {
				pr_err(DRV_NAME " memory allocation failed\n");
				ret = -ENOMEM;
				goto probe985620gpio_RELEASE_LIST;
			}

			mutex_lock(&gpioint_list_lock);
			INIT_LIST_HEAD(&gpioint->list);
			list_add(&gpioint->list, &gpioint_list);
			mutex_unlock(&gpioint_list_lock);

			prop = of_get_property(cnp, "interrupts", &len);
			if (!prop) {
				pr_err(DRV_NAME " can't find interrupts\n");
				ret = -ENODEV;
				goto probe985620gpio_RELEASE_LIST;
			}
			gpioint->irq = *(u32 *)prop;

			prop = of_get_property(op->node, "cell-index", NULL);
			if (!prop) {
				pr_err(DRV_NAME " can't find cell-index\n");
				ret = -ENODEV;
				goto probe985620gpio_RELEASE_LIST;
			}
			gpioint->gpioline = *(u32 *)prop;

			gpioint->device_type = (const char*)of_get_property(cnp, "device_type", &len);
			if (!gpioint->device_type) {
				pr_err(DRV_NAME " can't find device_type\n");
				ret = -ENODEV;
				goto probe985620gpio_RELEASE_LIST;
			}
		}
		ret = 0;

	goto probe985620gpio_RET;

probe985620gpio_RELEASE_LIST:

	mutex_lock(&gpioint_list_lock);
	list_for_each_entry_safe(gpioint, ptmp, &gpioint_list, list) {
		kfree(gpioint);	
	}
	mutex_unlock(&gpioint_list_lock);

probe985620gpio_DO_UNMAP:

	for (i = 0; i < (sizeof(gpio_priv_data.regs) / sizeof(gpio_priv_data.regs[0])); i++) {
		if(gpio_priv_data.regs[i]) {
			iounmap(gpio_priv_data.regs[i]);
			gpio_priv_data.regs[i] = NULL;
		}
		memset(&gpio_priv_data.shadow[i], 0, sizeof(gpio_priv_data.shadow[i]));
	}
	
	if (gpio_priv_data.irq != -1) {
		irq_dispose_mapping(gpio_priv_data.irq);
		gpio_priv_data.irq = -1;
	}

probe985620gpio_RET:

	return ret;
}

static const struct of_device_id bcm985620gpio_of_match[] = {
	{.compatible = "fsl,mpc8315-gpio"},
	{},
};
MODULE_DEVICE_TABLE(of, bcm985620gpio_of_match);

/* Structure for a device driver */
static struct of_platform_driver bcm985620_gpio_driver = {
	.name    = DRV_NAME,
	.match_table = bcm985620gpio_of_match,
	.probe       = bcm985620gpio_probe,
	.remove      = __devexit_p(bcm985620gpio_remove),
	.driver      = {
		.owner   = THIS_MODULE,
		.name    = DRV_NAME,
	},
};

/* -------------------------------------------------------------------- */

static struct class *gpio985620_class;

static int __init gpio985620_init(void)
{
	int ret; 
	struct device *dev;

#define GPIO_MAJOR 201
	gpio_priv_data.devt = MKDEV(GPIO_MAJOR, 0);

	ret = register_chrdev(MAJOR(gpio_priv_data.devt), "gpio985620", &gpio985620_fops);
	if (ret < 0) {
		goto gpio985620_init_UNREG_REGION;
	}

	gpio985620_class = class_create(THIS_MODULE, "gpio985620");
	if (IS_ERR(gpio985620_class)) {
		ret = PTR_ERR(gpio985620_class);
		goto gpio985620_init_UNREG_CHARDEV;
	}

	dev = device_create(gpio985620_class, 
					NULL, 
					gpio_priv_data.devt,
					NULL, 
					"gpio985620.%d", MINOR(gpio_priv_data.devt));
	ret = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if (ret != 0) {
		goto gpio985620_init_CLASS_DESTROY;
	}

	ret = of_register_platform_driver(&bcm985620_gpio_driver);
	if (ret < 0 || gpio_priv_data.regs[0] == NULL || gpio_priv_data.regs[1] == NULL) {
		pr_err(DRV_NAME " Could not register of platform driver");
		goto gpio985620_init_DO_UNMAP;
	}

	/* set initial directions - intially, all lines are input */
	gpio_priv_data.shadow[0].gpio_dir = 0;
	out_be32(&gpio_priv_data.regs[0]->gpio_dir, gpio_priv_data.shadow[0].gpio_dir);
	gpio_priv_data.shadow[1].gpio_dir = 0;
	out_be32(&gpio_priv_data.regs[1]->gpio_dir, gpio_priv_data.shadow[1].gpio_dir);

	/* assert ("pull up") GPIOs of the SPI chip select */
	gpio_priv_data.shadow[0].gpio_dat |= (
		GPIO_BIT(MPC8309_BCM985620_GPIO_SPI_CS_CPLD1)|
		GPIO_BIT(MPC8309_BCM985620_GPIO_SPI_CS_CPLD2));
	out_be32(&gpio_priv_data.regs[0]->gpio_dat, gpio_priv_data.shadow[0].gpio_dat);

	/* set output direction for SPI chip select gpio pins and all other outputs */
	gpio_priv_data.shadow[0].gpio_dir |= (
		GPIO_BIT(MPC8309_BCM985620_GPIO_SPI_CS_CPLD1)|
		GPIO_BIT(MPC8309_BCM985620_GPIO_SPI_CS_CPLD2)|
		GPIO_BIT(MPC8309_BCM985620_GPIO_SPI1_CLK)|
		GPIO_BIT(MPC8309_BCM985620_GPIO_SPI1_MOSI)|
		GPIO_BIT(MPC8309_BCM985620_GPIO_MPC_GPIO0));
	out_be32(&gpio_priv_data.regs[0]->gpio_dir, gpio_priv_data.shadow[0].gpio_dir);

/*	gpio_priv_data.shadow[1].gpio_dir |= (
		GPIO_BIT(MPC8309_BCM985620_GPIO_MPC_GPIO0));
	out_be32(&gpio_priv_data.regs[1]->gpio_dir, gpio_priv_data.shadow[1].gpio_dir);
*/
	gpio_priv_data.is_probed = 1;

	/* mask all interrupts */
	out_be32(&gpio_priv_data.regs[0]->gpio_imr, 0);
	out_be32(&gpio_priv_data.regs[1]->gpio_imr, 0);

	/* ack all interrupts */
	out_be32(&gpio_priv_data.regs[0]->gpio_ier, 0xffffffff);
	out_be32(&gpio_priv_data.regs[1]->gpio_ier, 0xffffffff);

	goto gpio985620_init_ERROR_RETURN;

gpio985620_init_DO_UNMAP:

	if(gpio_priv_data.regs[0]) iounmap(gpio_priv_data.regs[0]);
	if(gpio_priv_data.regs[1]) iounmap(gpio_priv_data.regs[1]);
	gpio_priv_data.regs[0] = NULL;
	gpio_priv_data.regs[1] = NULL;
	memset(&gpio_priv_data.shadow[0], 0, sizeof(gpio_priv_data.shadow[0]));
	memset(&gpio_priv_data.shadow[1], 0, sizeof(gpio_priv_data.shadow[1]));

	device_destroy(gpio985620_class, gpio_priv_data.devt);

gpio985620_init_CLASS_DESTROY:

	class_destroy(gpio985620_class);

gpio985620_init_UNREG_CHARDEV:

	unregister_chrdev(MAJOR(gpio_priv_data.devt), "");

gpio985620_init_UNREG_REGION:

	gpio_priv_data.is_probed = 0;

gpio985620_init_ERROR_RETURN:

	return ret;
}
module_init(gpio985620_init)

static void __exit gpio985620_exit(void)
{
	of_unregister_platform_driver(&bcm985620_gpio_driver);
	device_destroy(gpio985620_class, gpio_priv_data.devt);
	class_destroy(gpio985620_class);
	unregister_chrdev(MAJOR(gpio_priv_data.devt), "");
	unregister_chrdev_region(gpio_priv_data.devt, 1);
	gpio_priv_data.is_probed = 0;
}
module_exit(gpio985620_exit)

MODULE_DESCRIPTION("Broadcom 985620 gpio driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL");

