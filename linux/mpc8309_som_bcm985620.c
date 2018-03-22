/*
 * arch/powerpc/platforms/83xx/mpc8309_som_bcm985620.c
 *
 * Description: MPC8309 SOM board on BCM 985620 platform specific routines.
 *
 * Copyright (C) Freescale Semiconductor, Inc. 2010. All rights reserved.
 * Copyright (C) Broadcom Corp. (Provigent ltd.) 2011. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 * 
 */

#include <linux/pci.h>
#include <linux/of_platform.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <asm/time.h>
#include <asm/ipic.h>
#include <asm/udbg.h>
#include <asm/qe.h>
#include <asm/qe_ic.h>
#include <sysdev/fsl_pci.h>
#include <sysdev/fsl_soc.h>
#include <linux/bcm985620_gpio.h>
#include "mpc83xx.h"
#include "mpc8309_bcm985620.h"

/*
 * Setup the architecture
 */
static void __init mpc8309_som_bcm985620_setup_arch(void)
{
#ifdef CONFIG_PCI
	struct device_node *np;
#endif
	if (ppc_md.progress)
		ppc_md.progress("mpc8309_som_bcm985620_setup_arch()", 0);

#ifdef CONFIG_PCI
	for_each_compatible_node(np, "pci", "fsl,mpc8349-pci")
		mpc83xx_add_bridge(np);
#endif
	mpc8309_usb_cfg();

#ifdef CONFIG_QUICC_ENGINE
	qe_reset();
#endif				/* CONFIG_QUICC_ENGINE */

}

static void __init mpc8309_som_bcm985620_init_IRQ(void)
{
	struct device_node *np;

	np = of_find_node_by_type(NULL, "ipic");
	if (!np)
		return;

	ipic_init(np, 0);

	/* Initialize the default interrupt mapping priorities,
	 * in case the boot rom changed something on us.
	 */
	ipic_set_default_priority();
	of_node_put(np);

#ifdef CONFIG_QUICC_ENGINE
	np = of_find_compatible_node(NULL, NULL, "fsl,qe-ic");
	if (!np) {
		np = of_find_node_by_type(NULL, "qeic");
		if (!np)
			return;
	}
	qe_ic_init(np, 0, qe_ic_cascade_low_ipic, qe_ic_cascade_high_ipic);
	of_node_put(np);
#endif				/* CONFIG_QUICC_ENGINE */
}

/*
 * Called very early, MMU is off, device-tree isn't unflattened
 */
static int __init mpc8309_som_bcm985620_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	return of_flat_dt_is_compatible(root, "fsl,mpc8309som_bcm985620");
}

static struct of_device_id __initdata of_bus_ids[] = {
	{ .compatible = "simple-bus" },
	{ .compatible = "fsl,qe" },
	{},
};

static int __init declare_of_platform_devices(void)
{
	of_platform_bus_probe(NULL, of_bus_ids, NULL);
	return 0;
}
machine_device_initcall(mpc8309_som_bcm985620, declare_of_platform_devices);

define_machine(mpc8309_som_bcm985620) {
	.name			= "MPC8309 SOM BCM985620",
	.probe			= mpc8309_som_bcm985620_probe,
	.setup_arch		= mpc8309_som_bcm985620_setup_arch,
	.init_IRQ		= mpc8309_som_bcm985620_init_IRQ,
	.get_irq		= ipic_get_irq,
	.restart		= mpc83xx_restart,
	.time_init		= mpc83xx_time_init,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= udbg_progress,
};

/*
 * Setup spi
 */

/*
 * This is to set the cs for EEPROM which is SPI device connected.
 * on = TRUE for setting to logic '1'
 *    = FALSE for setting to logic '0'
 */

/* GPIO I2C address to set/clear the SPI EEPROM CS line*/
#define GPIO_EXPND_I2C_ADDR		0x20
#define SPI_EEPROM_CS_HIGH		0x02

static int enable = 0;	
static u8 expander1;

/* Write to IO expander thru I2C interface */
int gpio_expander_i2c_write_byte(u8 devaddr, u8 regoffset, u8 value, u8 i2c_id)
{
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	adap = i2c_get_adapter(i2c_id);
	if (!adap)
		return -ENODEV;
	msg->addr = devaddr;	/* I2C address of chip */
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = regoffset;	/* register num */
	data[1] = value;		/* register data */
	err = i2c_transfer(adap, msg, 1);
	i2c_put_adapter(adap);
	if (err >= 0)
		return 0;
	return err;
}

/* Read from IO expander thru I2C interface */
int gpio_expander_i2c_read_byte(u8 devaddr, u8 regoffset, u8 *value, u8 i2c_id)
{
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	adap = i2c_get_adapter(i2c_id);
	if (!adap)
		return -ENODEV;

	msg->addr = devaddr;	/* I2C address of chip */
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;
	data[0] = regoffset;	/* register num */
	err = i2c_transfer(adap, msg, 1);

	msg->addr = devaddr;	/* I2C address */
	msg->flags = I2C_M_RD;
	msg->len = 1;
	msg->buf = data;
	err = i2c_transfer(adap, msg, 1);
	 *value = data[0];
	i2c_put_adapter(adap);

	if (err >= 0)
		return 0;
	return err;
}

static int ioexpander_enable (void)
{
	u32 status = 0;
	u8 config_reg;
	
	/*Set up IOexpander configuration register for Input/Ouput via I2C2 */
	status = gpio_expander_i2c_read_byte(GPIO_EXPND_I2C_ADDR, 0x03,
					&config_reg, 1);
	if (status != 0) {
		printk(KERN_INFO "IO Expander value read error\n");
		return status;
	}
	config_reg &= 0xF9;
	status = gpio_expander_i2c_write_byte(GPIO_EXPND_I2C_ADDR, 0x03,
					config_reg, 1);
	if (status != 0) {
		printk(KERN_INFO "IO Expander value write error\n");
		return status;
	}
	
	/* mux_ctl=0 eeprom_cs=0(inactive) slic_cs=1(inactive) 
	   can_en=0(inactive) can_stby=0(inactive)*/
	expander1 = 0x01;
	status = gpio_expander_i2c_write_byte(GPIO_EXPND_I2C_ADDR, 0x01,
					expander1, 1);
	if (status != 0) {
		printk(KERN_INFO "IO Expander value write error\n");
		return status;
	}
	
	return status;
}


static void mpc830x_eeprom_spi_cs_control(struct spi_device *spi, bool on)
{
	u32 status;
	u8 tmp;

	if (enable == 0) {
		ioexpander_enable ();
		enable++;
	}

	tmp = expander1;
	if (on)
		tmp |= SPI_EEPROM_CS_HIGH;
	else
		tmp &= ~SPI_EEPROM_CS_HIGH;

	expander1 = tmp;
	status = gpio_expander_i2c_write_byte(GPIO_EXPND_I2C_ADDR, 0x01, tmp, 1);
	if (status != 0) {
		printk(KERN_INFO "IO Expander value write error\n");
		return;
	}

}

/*
 * Setting cs bcm985620 devices
 * on = TRUE for setting to logic '1'
 *    = FALSE for setting to logic '0'
 */

static int immrinit = 0;

static int mpc8309_bcm985620_immr_probe(void)
{
	int ret = 0;
	phys_addr_t immrbase;
	__be32 reg_save;
	__be32 __iomem *SICR1, *SICR2, *GPR1;

	if (immrinit) {
		goto immr_probe_end;
	}

	immrbase = get_immrbase();
	if (immrbase < 0) {
		ret = -ENOMEM;
		goto immr_probe_end;
	}

	/* mux in GPIO 17 (disable HDLC on that signal) and GPIO32-39 (disable USB application on these signals) */
	SICR2 = (__be32*) ioremap(immrbase + MPC8309_IMMR_SICR2_OFFSET, sizeof(__be32));
	if (!SICR2) {
		ret = -ENOMEM;
		goto immr_probe_end;
	}

	reg_save = in_be32(SICR2);
	reg_save &= ~(MPC8309_IMMR_SICR2_GPIO17_MUX_MASK | MPC8309_IMMR_SICR2_GPIO20_MUX_MASK | MPC8309_IMMR_SICR2_GPIO32_TO_39_MUX_MASK);
	reg_save |=  (MPC8309_IMMR_SICR2_GPIO17_MUX_BITS | MPC8309_IMMR_SICR2_GPIO20_MUX_BITS | MPC8309_IMMR_SICR2_GPIO32_TO_39_MUX_BITS);
	out_be32(SICR2, reg_save);
	iounmap(SICR2);

	/* mux in DUART2 (disable USB on that signal) */
	SICR1 = (__be32*) ioremap(immrbase + MPC8309_IMMR_SICR1_OFFSET, sizeof(__be32));
	if (!SICR1) {
		ret = -ENOMEM;
		goto immr_probe_end;
	}

	reg_save = in_be32(SICR1);
	reg_save &= ~(MPC8309_IMMR_SICR1_UART2_1_MUX_MASK | MPC8309_IMMR_SICR1_UART2_2_MUX_MASK);
	reg_save |=  (MPC8309_IMMR_SICR1_UART2_1_MUX_BITS | MPC8309_IMMR_SICR1_UART2_2_MUX_BITS);
	out_be32(SICR1, reg_save);
	iounmap(SICR1);

	/* disable FEC{1,2,3} error and collision pull-ups */
	GPR1 = (__be32*) ioremap(immrbase + MPC8309_IMMR_GPR1_OFFSET, sizeof(__be32));
	if (!GPR1) {
		ret = -ENOMEM;
		goto immr_probe_end;
	}

	reg_save = in_be32(GPR1);
	reg_save &= ~MPC8309_IMMR_GPR1_FEC_PULLUP_MUX_MASK;
	reg_save |=  MPC8309_IMMR_GPR1_FEC_PULLUP_MUX_BITS;
	out_be32(GPR1, reg_save);
	iounmap(GPR1);

	immrinit = 1;

immr_probe_end:

	return ret;
}

/*
 * Board specific chipselect functions.
 */
static void mpc830x_bcm985620_spi_cs_control(struct spi_device *spi, bool on)
{
	if (!spi) {
		return;
	}

	switch (spi->chip_select) 
	{
	case MPC8309_BCM985620_GPIO_SPI_CS_CPLD1:
	case MPC8309_BCM985620_GPIO_SPI_CS_CPLD2:
		if (on) {
			gpio985620_gpio_assert((u8)spi->chip_select);
		} else {
			gpio985620_gpio_deassert((u8)spi->chip_select);
		}
		break;
	default:
		mpc830x_eeprom_spi_cs_control(spi, on);
	}
}

struct bcm985620_gpio_platform_data {
	int                 range_start;
	int                 range_end;
	struct mpc8309_gpio *regs;
};

static int __init of_fsl_bcm985620_gpio_probe(void)
{
	struct device_node *np;
	unsigned int i = 0;
	u32 offset;
	int ret = 0;
	const void *prop;
	struct resource res;
	struct platform_device *pdev;
	struct bcm985620_gpio_platform_data pdata;
	char busname[32];


/* TODO : GPIO device probe from dts. Until then -  */
return 0;
	
	for_each_compatible_node(np, "gpio", "fsl,mpc8315-gpio") {
		memset(&res, 0, sizeof(res));
		memset(busname, 0, sizeof(busname));
		memset(&pdata, 0, sizeof(pdata));

		prop = of_get_property(np, "reg", NULL);
		if (!prop) continue;
		offset = *(u32*)prop;

		if (offset != MPC8309_IMMR_GPIO1_OFFSET  &&
		    offset != MPC8309_IMMR_GPIO2_OFFSET) {
			continue;
		}

		sprintf(busname, "mpc8315-gpio-0x%3x", offset);

		if (!request_mem_region(get_immrbase() + offset, sizeof(struct mpc8309_gpio), busname)) {
			ret = -ENOMEM;
			goto gpio_probe_end;
		}
	
		pdata.regs = (struct mpc8309_gpio*) ioremap(get_immrbase() + offset, sizeof(struct mpc8309_gpio));
		if (!pdata.regs) {
			ret = -ENOMEM;
			goto do_release;
		}

		prop = of_get_property(np, "cell-index", NULL);
		if (prop)
			i = *(u32 *)prop;

		pdata.range_start = i * 8 /* bits in */ * sizeof(u32);
		pdata.range_end   = pdata.range_start + (8 * sizeof(u32)) - 1;

		ret = of_address_to_resource(np, 0, &res);
		if (ret)
			goto do_unmap;

		pdev = platform_device_alloc(busname, i);
		if (!pdev)
			goto do_unmap;

		ret = platform_device_add_data(pdev, &pdata, sizeof(pdata));
		if (ret)
			goto do_unmap;

		ret = platform_device_add_resources(pdev, &res, 1);
		if (ret)
			goto do_unmap;

		ret = platform_device_add(pdev);
		if (ret)
			goto do_unmap;

		goto do_next;

do_unmap:

		iounmap(pdata.regs);

do_release:

		release_mem_region(get_immrbase() + offset, sizeof(struct mpc8309_gpio));

		break;

do_next:

		i++;
	}

gpio_probe_end:

	return ret;
}

static int __init of_fsl_spi_probe(char *type, char *compatible, u32 sysclk,
				   struct spi_board_info *board_infos,
				   unsigned int num_board_infos,
			       void (*cs_control)(struct spi_device *spi, bool on))
{
	struct device_node *np;
	unsigned int i = 0;

	for_each_compatible_node(np, type, compatible) {
		int ret;
		unsigned int j;
		const void *prop;
		struct resource res[2];
		struct platform_device *pdev;
		struct fsl_spi_platform_data pdata = {
			.cs_control = cs_control,
		};

		memset(res, 0, sizeof(res));

		pdata.sysclk = sysclk;

		prop = of_get_property(np, "reg", NULL);
		if (!prop)
			goto err;
		pdata.bus_num = *(u32 *)prop;

		prop = of_get_property(np, "cell-index", NULL);
		if (prop)
			i = *(u32 *)prop;

		prop = of_get_property(np, "mode", NULL);
		if (prop && !strcmp(prop, "cpu-qe"))
			pdata.flags = SPI_QE_CPU_MODE;

		for (j = 0; j < num_board_infos; j++) {
			if (board_infos[j].bus_num == pdata.bus_num)
				pdata.max_chipselect++;
		}

		if (!pdata.max_chipselect)
			continue;

		ret = of_address_to_resource(np, 0, &res[0]);
		if (ret)
			goto err;

		ret = of_irq_to_resource(np, 0, &res[1]);
		if (ret == NO_IRQ)
			goto err;

		pdev = platform_device_alloc("mpc83xx_spi", i);
		if (!pdev)
			goto err;

		ret = platform_device_add_data(pdev, &pdata, sizeof(pdata));
		if (ret)
			goto unreg;

		ret = platform_device_add_resources(pdev, res,
						    ARRAY_SIZE(res));
		if (ret)
			goto unreg;

		ret = platform_device_add(pdev);
		if (ret)
			goto unreg;

		goto next;
unreg:
		platform_device_del(pdev);
err:
		pr_err("%s: registration failed\n", np->full_name);
next:
		i++;
	}

	return i;
}

int __init fsl_spi_init(struct spi_board_info *board_infos,
			unsigned int num_board_infos,
			void (*cs_control)(struct spi_device *dev, bool on))
{
	u32 sysclk = -1;
	int ret;

	if (sysclk == -1) {
		sysclk = fsl_get_sys_freq();
		if (sysclk == -1)
			return -ENODEV;
	}

	ret = of_fsl_spi_probe(NULL, "fsl,spi", sysclk, board_infos,
			       num_board_infos, cs_control);
	if (!ret)
		of_fsl_spi_probe("spi", "fsl_spi", sysclk, board_infos,
				 num_board_infos, cs_control);

	ret = mpc8309_bcm985620_immr_probe();
	if (ret) {
		return ret;
	}

	ret = of_fsl_bcm985620_gpio_probe();
	if (ret) {
		return ret;
	}

	return spi_register_board_info(board_infos, num_board_infos);
}

/*
 * This structure gives the information of the SPI slave connected on the board.
 */
static struct spi_board_info mpc8309_bcm985620_spi_boardinfo[] = {
	/* cpld1 */
	{
	.modalias = "spidev",
	.bus_num = 0x7000,
	.chip_select = MPC8309_BCM985620_GPIO_SPI_CS_CPLD1,
	.max_speed_hz = 2000000,
	.mode = SPI_MODE_3,
	.controller_data = (void*)&mpc830x_bcm985620_spi_cs_control,
	},
	/* cpld2 */
	{
	.modalias = "spidev",
	.bus_num = 0x7000,
	.chip_select = MPC8309_BCM985620_GPIO_SPI_CS_CPLD2,
	.max_speed_hz = 2000000,
	.mode = SPI_MODE_3,
	.controller_data = (void*)&mpc830x_bcm985620_spi_cs_control,
	}
};

static int __init mpc8309_bcm985620_spi_init(void)
{
	return fsl_spi_init(mpc8309_bcm985620_spi_boardinfo, 
	 		ARRAY_SIZE(mpc8309_bcm985620_spi_boardinfo),
			mpc830x_bcm985620_spi_cs_control);
}
machine_device_initcall(mpc8309_som_bcm985620, mpc8309_bcm985620_spi_init);


