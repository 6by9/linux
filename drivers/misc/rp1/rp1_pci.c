// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-24 Raspberry Pi Ltd.
 * All rights reserved.
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "rp1_pci.h"

#define RP1_DRIVER_NAME		"rp1"

#define RP1_HW_IRQ_MASK		GENMASK(5, 0)

#define REG_SET			0x800
#define REG_CLR			0xc00

/* MSI-X CFG registers start at 0x8 */
#define MSIX_CFG(x) (0x8 + (4 * (x)))

#define MSIX_CFG_IACK_EN        BIT(3)
#define MSIX_CFG_IACK           BIT(2)
#define MSIX_CFG_ENABLE         BIT(0)

/* Address map */
#define RP1_PCIE_APBS_BASE	0x108000

/* Interrupts */
#define RP1_INT_IO_BANK0	0
#define RP1_INT_IO_BANK1	1
#define RP1_INT_IO_BANK2	2
#define RP1_INT_AUDIO_IN	3
#define RP1_INT_AUDIO_OUT	4
#define RP1_INT_PWM0		5
#define RP1_INT_ETH		6
#define RP1_INT_I2C0		7
#define RP1_INT_I2C1		8
#define RP1_INT_I2C2		9
#define RP1_INT_I2C3		10
#define RP1_INT_I2C4		11
#define RP1_INT_I2C5		12
#define RP1_INT_I2C6		13
#define RP1_INT_I2S0		14
#define RP1_INT_I2S1		15
#define RP1_INT_I2S2		16
#define RP1_INT_SDIO0		17
#define RP1_INT_SDIO1		18
#define RP1_INT_SPI0		19
#define RP1_INT_SPI1		20
#define RP1_INT_SPI2		21
#define RP1_INT_SPI3		22
#define RP1_INT_SPI4		23
#define RP1_INT_SPI5		24
#define RP1_INT_UART0		25
#define RP1_INT_TIMER_0		26
#define RP1_INT_TIMER_1		27
#define RP1_INT_TIMER_2		28
#define RP1_INT_TIMER_3		29
#define RP1_INT_USBHOST0	30
#define RP1_INT_USBHOST0_0	31
#define RP1_INT_USBHOST0_1	32
#define RP1_INT_USBHOST0_2	33
#define RP1_INT_USBHOST0_3	34
#define RP1_INT_USBHOST1	35
#define RP1_INT_USBHOST1_0	36
#define RP1_INT_USBHOST1_1	37
#define RP1_INT_USBHOST1_2	38
#define RP1_INT_USBHOST1_3	39
#define RP1_INT_DMA		40
#define RP1_INT_PWM1		41
#define RP1_INT_UART1		42
#define RP1_INT_UART2		43
#define RP1_INT_UART3		44
#define RP1_INT_UART4		45
#define RP1_INT_UART5		46
#define RP1_INT_MIPI0		47
#define RP1_INT_MIPI1		48
#define RP1_INT_VIDEO_OUT	49
#define RP1_INT_PIO_0		50
#define RP1_INT_PIO_1		51
#define RP1_INT_ADC_FIFO	52
#define RP1_INT_PCIE_OUT	53
#define RP1_INT_SPI6		54
#define RP1_INT_SPI7		55
#define RP1_INT_SPI8		56
#define RP1_INT_SYSCFG		58
#define RP1_INT_CLOCKS_DEFAULT	59
#define RP1_INT_VBUSCTRL	60
#define RP1_INT_PROC_MISC	57
#define RP1_INT_END		61

struct rp1_dev {
	struct pci_dev *pdev;
	struct device *dev;
	struct clk *sys_clk;
	struct irq_domain *domain;
	struct irq_data *pcie_irqds[64];
	void __iomem *bar1;
	int ovcs_id;
	bool level_triggered_irq[RP1_INT_END];
};

static void dump_bar(struct pci_dev *pdev, unsigned int bar)
{
	dev_info(&pdev->dev,
		 "bar%d %pR\n",
		 bar,
		 pci_resource_n(pdev, bar));
}

static void msix_cfg_set(struct rp1_dev *rp1, unsigned int hwirq, u32 value)
{
	iowrite32(value, rp1->bar1 + RP1_PCIE_APBS_BASE + REG_SET + MSIX_CFG(hwirq));
}

static void msix_cfg_clr(struct rp1_dev *rp1, unsigned int hwirq, u32 value)
{
	iowrite32(value, rp1->bar1 + RP1_PCIE_APBS_BASE + REG_CLR + MSIX_CFG(hwirq));
}

static void rp1_mask_irq(struct irq_data *irqd)
{
	struct rp1_dev *rp1 = irqd->domain->host_data;
	struct irq_data *pcie_irqd = rp1->pcie_irqds[irqd->hwirq];

	pci_msi_mask_irq(pcie_irqd);
}

static void rp1_unmask_irq(struct irq_data *irqd)
{
	struct rp1_dev *rp1 = irqd->domain->host_data;
	struct irq_data *pcie_irqd = rp1->pcie_irqds[irqd->hwirq];

	pci_msi_unmask_irq(pcie_irqd);
}

static int rp1_irq_set_type(struct irq_data *irqd, unsigned int type)
{
	struct rp1_dev *rp1 = irqd->domain->host_data;
	unsigned int hwirq = (unsigned int)irqd->hwirq;

	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
		dev_dbg(rp1->dev, "MSIX IACK EN for irq %d\n", hwirq);
		msix_cfg_set(rp1, hwirq, MSIX_CFG_IACK_EN);
		rp1->level_triggered_irq[hwirq] = true;
	break;
	case IRQ_TYPE_EDGE_RISING:
		msix_cfg_clr(rp1, hwirq, MSIX_CFG_IACK_EN);
		rp1->level_triggered_irq[hwirq] = false;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct irq_chip rp1_irq_chip = {
	.name            = "rp1_irq_chip",
	.irq_mask        = rp1_mask_irq,
	.irq_unmask      = rp1_unmask_irq,
	.irq_set_type    = rp1_irq_set_type,
};

static void rp1_chained_handle_irq(struct irq_desc *desc)
{
	unsigned int hwirq = desc->irq_data.hwirq & RP1_HW_IRQ_MASK;
	struct rp1_dev *rp1 = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int virq;

	chained_irq_enter(chip, desc);

	virq = irq_find_mapping(rp1->domain, hwirq);
	generic_handle_irq(virq);
	if (rp1->level_triggered_irq[hwirq])
		msix_cfg_set(rp1, hwirq, MSIX_CFG_IACK);

	chained_irq_exit(chip, desc);
}

static int rp1_irq_xlate(struct irq_domain *d, struct device_node *node,
			 const u32 *intspec, unsigned int intsize,
			 unsigned long *out_hwirq, unsigned int *out_type)
{
	struct rp1_dev *rp1 = d->host_data;
	struct irq_data *pcie_irqd;
	unsigned long hwirq;
	int pcie_irq;
	int ret;

	ret = irq_domain_xlate_twocell(d, node, intspec, intsize,
				       &hwirq, out_type);
	if (ret)
		return ret;

	pcie_irq = pci_irq_vector(rp1->pdev, hwirq);
	pcie_irqd = irq_get_irq_data(pcie_irq);
	rp1->pcie_irqds[hwirq] = pcie_irqd;
	*out_hwirq = hwirq;

	return 0;
}

static int rp1_irq_activate(struct irq_domain *d, struct irq_data *irqd,
			    bool reserve)
{
	struct rp1_dev *rp1 = d->host_data;

	msix_cfg_set(rp1, (unsigned int)irqd->hwirq, MSIX_CFG_ENABLE);

	return 0;
}

static void rp1_irq_deactivate(struct irq_domain *d, struct irq_data *irqd)
{
	struct rp1_dev *rp1 = d->host_data;

	msix_cfg_clr(rp1, (unsigned int)irqd->hwirq, MSIX_CFG_ENABLE);
}

static const struct irq_domain_ops rp1_domain_ops = {
	.xlate      = rp1_irq_xlate,
	.activate   = rp1_irq_activate,
	.deactivate = rp1_irq_deactivate,
};

static int rp1_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct device_node *rp1_node;
	struct reset_control *reset;
	struct rp1_dev *rp1;
	int err  = 0;
	int i;

	rp1_node = dev_of_node(dev);
	if (!rp1_node) {
		dev_err(dev, "Missing of_node for device\n");
		return -EINVAL;
	}

	rp1 = devm_kzalloc(&pdev->dev, sizeof(*rp1), GFP_KERNEL);
	if (!rp1)
		return -ENOMEM;

	rp1->pdev = pdev;
	rp1->dev = &pdev->dev;

	reset = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(reset))
		return PTR_ERR(reset);
	reset_control_reset(reset);

	dump_bar(pdev, 0);
	dump_bar(pdev, 1);

	if (pci_resource_len(pdev, 1) <= 0x10000) {
		dev_err(&pdev->dev,
			"Not initialised - is the firmware running?\n");
		return -EINVAL;
	}

	err = pcim_enable_device(pdev);
	if (err < 0) {
		dev_err(&pdev->dev, "Enabling PCI device has failed: %d",
			err);
		return err;
	}

	rp1->bar1 = pcim_iomap(pdev, 1, 0);
	if (!rp1->bar1) {
		dev_err(&pdev->dev, "Cannot map PCI BAR\n");
		return -EIO;
	}

	u32 dtbo_size = __dtbo_rp1_pci_end - __dtbo_rp1_pci_begin;
	void *dtbo_start = __dtbo_rp1_pci_begin;

	err = of_overlay_fdt_apply(dtbo_start, dtbo_size, &rp1->ovcs_id, rp1_node);
	if (err)
		return err;

	pci_set_master(pdev);

	err = pci_alloc_irq_vectors(pdev, RP1_INT_END, RP1_INT_END,
				    PCI_IRQ_MSIX);
	if (err != RP1_INT_END) {
		dev_err(&pdev->dev, "pci_alloc_irq_vectors failed - %d\n", err);
		goto err_unload_overlay;
	}

	pci_set_drvdata(pdev, rp1);
	rp1->domain = irq_domain_add_linear(of_find_node_by_name(NULL, "pci-ep-bus"), RP1_INT_END,
					    &rp1_domain_ops, rp1);

	for (i = 0; i < RP1_INT_END; i++) {
		int irq = irq_create_mapping(rp1->domain, i);

		if (irq < 0) {
			dev_err(&pdev->dev, "failed to create irq mapping\n");
			err = irq;
			goto err_unload_overlay;
		}
		irq_set_chip_and_handler(irq, &rp1_irq_chip, handle_level_irq);
		irq_set_probe(irq);
		irq_set_chained_handler_and_data(pci_irq_vector(pdev, i),
						 rp1_chained_handle_irq, rp1);
	}

	err = of_platform_default_populate(rp1_node, NULL, dev);
	if (err)
		goto err_unload_overlay;

	return 0;

err_unload_overlay:
	of_overlay_remove(&rp1->ovcs_id);

	return err;
}

static void rp1_remove(struct pci_dev *pdev)
{
	struct rp1_dev *rp1 = pci_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	of_platform_depopulate(dev);
	of_overlay_remove(&rp1->ovcs_id);

	clk_unregister(rp1->sys_clk);
}

static const struct pci_device_id dev_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_RPI, PCI_DEVICE_ID_RPI_RP1_C0), },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, dev_id_table);

static struct pci_driver rp1_driver = {
	.name		= RP1_DRIVER_NAME,
	.id_table	= dev_id_table,
	.probe		= rp1_probe,
	.remove		= rp1_remove,
};

module_pci_driver(rp1_driver);

MODULE_AUTHOR("Phil Elwell <phil@raspberrypi.com>");
MODULE_AUTHOR("Andrea della Porta <andrea.porta@suse.com>");
MODULE_DESCRIPTION("RP1 wrapper");
MODULE_LICENSE("GPL");
