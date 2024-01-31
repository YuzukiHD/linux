// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * PCIe RC driver for Allwinner Core
 *
 * Copyright (C) 2022 Allwinner Co., Ltd.
 *
 * Author: songjundong <songjundong@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/phy/phy.h>
#include <linux/pm_runtime.h>

#include "pci.h"
#include "pcie-sunxi-dma.h"
#include "pcie-sunxi.h"

void sunxi_pcie_writel(u32 val, struct sunxi_pcie *pcie, u32 offset)
{
	writel(val, pcie->app_base + offset);
}

u32 sunxi_pcie_readl(struct sunxi_pcie *pcie, u32 offset)
{
	return readl(pcie->app_base + offset);
}

void sunxi_pcie_writel_dbi(struct sunxi_pcie *pci, u32 reg, u32 val)
{
	sunxi_pcie_write_dbi(pci, reg, 0x4, val);
}

u32 sunxi_pcie_readl_dbi(struct sunxi_pcie *pci, u32 reg)
{
	return sunxi_pcie_read_dbi(pci, reg, 0x4);
}

void sunxi_pcie_writew_dbi(struct sunxi_pcie *pci, u32 reg, u16 val)
{
	sunxi_pcie_write_dbi(pci, reg, 0x2, val);
}

u16 sunxi_pcie_readw_dbi(struct sunxi_pcie *pci, u32 reg)
{
	return sunxi_pcie_read_dbi(pci, reg, 0x2);
}

void sunxi_pcie_writeb_dbi(struct sunxi_pcie *pci, u32 reg, u8 val)
{
	sunxi_pcie_write_dbi(pci, reg, 0x1, val);
}

u8 sunxi_pcie_readb_dbi(struct sunxi_pcie *pci, u32 reg)
{
	return sunxi_pcie_read_dbi(pci, reg, 0x1);
}

void sunxi_pcie_dbi_ro_wr_en(struct sunxi_pcie *pci)
{
	u32 val;

	val = sunxi_pcie_readl_dbi(pci, PCIE_MISC_CONTROL_1_CFG);
	val |= (0x1 << 0);
	sunxi_pcie_writel_dbi(pci, PCIE_MISC_CONTROL_1_CFG, val);
}

void sunxi_pcie_dbi_ro_wr_dis(struct sunxi_pcie *pci)
{
	u32 val;

	val = sunxi_pcie_readl_dbi(pci, PCIE_MISC_CONTROL_1_CFG);
	val &= ~(0x1 << 0);
	sunxi_pcie_writel_dbi(pci, PCIE_MISC_CONTROL_1_CFG, val);
}

static void sunxi_pcie_set_mode(struct sunxi_pcie *pcie)
{
	u32 val;

	switch (pcie->drvdata->mode) {
	case SUNXI_PCIE_EP_TYPE:
		val = sunxi_pcie_readl(pcie, PCIE_LTSSM_CTRL);
		val |= ~DEVICE_TYPE_MASK;
		sunxi_pcie_writel(val, pcie, PCIE_LTSSM_CTRL);
		break;
	case SUNXI_PCIE_RC_TYPE:
		val = sunxi_pcie_readl(pcie, PCIE_LTSSM_CTRL);
		val |= DEVICE_TYPE_RC;
		sunxi_pcie_writel(val, pcie, PCIE_LTSSM_CTRL);
		break;
	default:
		dev_err(&pcie->dev, "sunxi pcie unsupported mode:%d !\n",
							pcie->drvdata->mode);
		break;
	}
}

static u8 __sunxi_pcie_find_next_cap(struct sunxi_pcie *pci, u8 cap_ptr,
						u8 cap)
{
	u8 cap_id, next_cap_ptr;
	u16 reg;

	if (!cap_ptr)
		return 0;

	reg = sunxi_pcie_readw_dbi(pci, cap_ptr);
	cap_id = (reg & CAP_ID_MASK);

	if (cap_id > PCI_CAP_ID_MAX)
		return 0;

	if (cap_id == cap)
		return cap_ptr;

	next_cap_ptr = (reg & NEXT_CAP_PTR_MASK) >> 8;
	return __sunxi_pcie_find_next_cap(pci, next_cap_ptr, cap);
}

static u8 sunxi_pcie_find_capability(struct sunxi_pcie *pci, u8 cap)
{
	u8 next_cap_ptr;
	u16 reg;

	reg = sunxi_pcie_readw_dbi(pci, PCI_CAPABILITY_LIST);
	next_cap_ptr = (reg & CAP_ID_MASK);

	return __sunxi_pcie_find_next_cap(pci, next_cap_ptr, cap);
}

int sunxi_pcie_cfg_read(void __iomem *addr, int size, u32 *val)
{
	if ((uintptr_t)addr & (size - 1)) {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	if (size == 4) {
		*val = readl(addr);
	} else if (size == 2) {
		*val = readw(addr);
	} else if (size == 1) {
		*val = readb(addr);
	} else {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}
EXPORT_SYMBOL_GPL(sunxi_pcie_cfg_read);

int sunxi_pcie_cfg_write(void __iomem *addr, int size, u32 val)
{
	if ((uintptr_t)addr & (size - 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (size == 4)
		writel(val, addr);
	else if (size == 2)
		writew(val, addr);
	else if (size == 1)
		writeb(val, addr);
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}
EXPORT_SYMBOL_GPL(sunxi_pcie_cfg_write);

void sunxi_pcie_write_dbi(struct sunxi_pcie *pci, u32 reg, size_t size, u32 val)
{
	int ret;

	ret = sunxi_pcie_cfg_write(pci->dbi_base + reg, size, val);
	if (ret)
		dev_err(&pci->dev, "Write DBI address failed\n");
}
EXPORT_SYMBOL_GPL(sunxi_pcie_write_dbi);

u32 sunxi_pcie_read_dbi(struct sunxi_pcie *pci, u32 reg, size_t size)
{
	int ret;
	u32 val;

	ret = sunxi_pcie_cfg_read(pci->dbi_base + reg, size, &val);
	if (ret)
		dev_err(&pci->dev, "Read DBI address failed\n");

	return val;
}
EXPORT_SYMBOL_GPL(sunxi_pcie_read_dbi);

static void sunxi_pcie_set_link_cap(struct sunxi_pcie *pci, u32 link_gen)
{
	u32 cap, ctrl2, link_speed;

	u8 offset = sunxi_pcie_find_capability(pci, PCI_CAP_ID_EXP);

	cap = sunxi_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCAP);
	ctrl2 = sunxi_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCTL2);
	ctrl2 &= ~PCI_EXP_LNKCTL2_TLS;

	switch (pcie_link_speed[link_gen]) {
	case PCIE_SPEED_2_5GT:
		link_speed = PCI_EXP_LNKCTL2_TLS_2_5GT;
		break;
	case PCIE_SPEED_5_0GT:
		link_speed = PCI_EXP_LNKCTL2_TLS_5_0GT;
		break;
	case PCIE_SPEED_8_0GT:
		link_speed = PCI_EXP_LNKCTL2_TLS_8_0GT;
		break;
	case PCIE_SPEED_16_0GT:
		link_speed = PCI_EXP_LNKCTL2_TLS_16_0GT;
		break;
	default:
		/* Use hardware capability */
		link_speed = FIELD_GET(PCI_EXP_LNKCAP_SLS, cap);
		ctrl2 &= ~PCI_EXP_LNKCTL2_HASD;
		break;
	}

	sunxi_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCTL2, ctrl2 | link_speed);

	cap &= ~((u32)PCI_EXP_LNKCAP_SLS);
	sunxi_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCAP, cap | link_speed);
}

void sunxi_pcie_set_rate(struct pcie_port *pp)
{
	struct sunxi_pcie *pci = to_sunxi_pcie_from_pp(pp);
	u32 val;

	sunxi_pcie_set_link_cap(pci, pci->link_gen);
	/* set the number of lanes */
	val = sunxi_pcie_readl_dbi(pci, PCIE_PORT_LINK_CONTROL);
	val &= ~PORT_LINK_MODE_MASK;
	switch (pp->lanes) {
	case 1:
		val |= PORT_LINK_MODE_1_LANES;
		break;
	case 2:
		val |= PORT_LINK_MODE_2_LANES;
		break;
	case 4:
		val |= PORT_LINK_MODE_4_LANES;
		break;
	default:
		dev_err(pp->dev, "num-lanes %u: invalid value\n", pp->lanes);
		return;
	}
	sunxi_pcie_writel_dbi(pci, PCIE_PORT_LINK_CONTROL, val);

	/* set link width speed control register */
	val = sunxi_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
	switch (pp->lanes) {
	case 1:
		val |= PORT_LOGIC_LINK_WIDTH_1_LANES;
		break;
	case 2:
		val |= PORT_LOGIC_LINK_WIDTH_2_LANES;
		break;
	case 4:
		val |= PORT_LOGIC_LINK_WIDTH_4_LANES;
		break;
	}
	sunxi_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);
}
EXPORT_SYMBOL_GPL(sunxi_pcie_set_rate);

static unsigned int sunxi_pcie_ep_func_conf_select(struct sunxi_pcie_ep *ep,
						u8 func_no)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie_from_ep(ep);

	WARN_ON(func_no && !pcie->drvdata->func_offset);
	return pcie->drvdata->func_offset * func_no;
}

static const struct sunxi_pcie_ep_ops sunxi_ep_ops = {
	.func_conf_select = sunxi_pcie_ep_func_conf_select,
};

static const struct sunxi_pcie_of_data sunxi_pcie_rc_v210_of_data = {
	.mode = SUNXI_PCIE_RC_TYPE,
	.cpu_pcie_addr_quirk = true,
};

static const struct sunxi_pcie_of_data sunxi_pcie_rc_v300_of_data = {
	.mode = SUNXI_PCIE_RC_TYPE,
};

static const struct sunxi_pcie_of_data sunxi_pcie_ep_v300_of_data = {
	.mode = SUNXI_PCIE_EP_TYPE,
	.func_offset = 0x1000,
	.ops = &sunxi_ep_ops,
};

static const struct of_device_id sunxi_plat_pcie_of_match[] = {
	{
		.compatible = "allwinner,sunxi-pcie-v210",
		.data = &sunxi_pcie_rc_v210_of_data,
	},
	{
		.compatible = "allwinner,sunxi-pcie-v300-rc",
		.data = &sunxi_pcie_rc_v300_of_data,
	},
	{
		.compatible = "allwinner,sunxi-pcie-v300-ep",
		.data = &sunxi_pcie_ep_v300_of_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_plat_pcie_of_match);

static inline void sunxi_pcie_writel_phy(struct pcie_port *pp, u32 val, u32 reg)
{
	writel(val, pp->phy_base + reg);
}

static inline u32 sunxi_pcie_readl_phy(struct pcie_port *pp, u32 reg)
{
	return readl(pp->phy_base + reg);
}

void sunxi_pcie_ltssm_enable(struct sunxi_pcie *pcie, bool enable)
{
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_LTSSM_CTRL);
	if (enable)
		val |= PCIE_LINK_TRAINING;
	else
		val &= ~PCIE_LINK_TRAINING;
	sunxi_pcie_writel(val, pcie, PCIE_LTSSM_CTRL);
}
EXPORT_SYMBOL_GPL(sunxi_pcie_ltssm_enable);

static void sunxi_pcie_irqpending(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie_from_pp(pp);
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_INT_ENABLE_CLR);
	val &= ~(0x3<<0);
	sunxi_pcie_writel(val, pcie, PCIE_INT_ENABLE_CLR);
}

static void sunxi_pcie_irqmask(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie_from_pp(pp);
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_INT_ENABLE_CLR);
	val |= 0x3<<0;
	sunxi_pcie_writel(val, pcie, PCIE_INT_ENABLE_CLR);
}

static int sunxi_pcie_enable_power(struct sunxi_pcie *pcie)
{
	int ret = 0;
	struct device *dev = &pcie->dev;

	if (IS_ERR_OR_NULL(pcie->pcie3v3))
		return ret;

	ret = regulator_set_voltage(pcie->pcie3v3, 3300000, 3300000);
	if (ret)
		dev_info(dev, "set regulator voltage failed, use default\n");

	ret = regulator_enable(pcie->pcie3v3);
	if (ret)
		dev_err(dev, "fail to enable pcie3v3 regulator\n");

	return ret;
}

static int sunxi_pcie_disable_power(struct sunxi_pcie *pcie)
{
	int ret = 0;
	struct device *dev = &pcie->dev;

	if (IS_ERR_OR_NULL(pcie->pcie3v3))
		return ret;

	ret = regulator_disable(pcie->pcie3v3);
	if (ret)
		dev_err(dev, "fail to disable pcie3v3 regulator\n");

	return ret;
}
int sunxi_pcie_link_up(struct pcie_port *pp)
{
	if (pp->ops->link_up)
		return pp->ops->link_up(pp);
	else
		return 0;
}
EXPORT_SYMBOL_GPL(sunxi_pcie_link_up);

int sunxi_pcie_wait_for_link(struct pcie_port *pp)
{
	int retries;

	for (retries = 0; retries < LINK_WAIT_MAX_RETRIE; retries++) {
		if (sunxi_pcie_link_up(pp)) {
			dev_info(pp->dev, "sunxi pcie link up success\n");
			return 0;
		}
		usleep_range(LINK_WAIT_USLEEP_MIN, LINK_WAIT_USLEEP_MAX);
	}

	return -ETIMEDOUT;
}
EXPORT_SYMBOL_GPL(sunxi_pcie_wait_for_link);

int sunxi_pcie_establish_link(struct pcie_port *pp)
{
	struct sunxi_pcie *pci = to_sunxi_pcie_from_pp(pp);

	if (sunxi_pcie_link_up(pp)) {
		dev_info(pp->dev, "PCIe link is already up\n");
		return 0;
	}

	sunxi_pcie_ltssm_enable(pci, true);

	if (sunxi_pcie_wait_for_link(pp))
		dev_err(pp->dev, "PCIe link failure !\n");

	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_pcie_establish_link);

static int sunxi_pcie_clk_setup(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie_from_pp(pp);
	int ret;

	ret = clk_prepare_enable(pcie->pcie_ref);
	if (ret) {
		dev_err(pp->dev, "PCIe: cannot prepare/enable ref clock\n");
		goto err_clk_ref;
	}

	if (pcie->clk_freq_100M) {
		ret = clk_set_rate(pcie->pcie_ref, 100000000);
		if (ret) {
			dev_err(pp->dev, "PCIe: fail to set clock freq 100M!\n");
			goto err_clk_aux;
		}
	}

	ret = clk_prepare_enable(pcie->pcie_aux);
	if (ret) {
		dev_err(pp->dev, "PCIe: cannot prepare/enable aux clock\n");
		goto err_clk_aux;
	}

	ret = reset_control_deassert(pcie->pcie_rst);
	if (ret) {
		dev_err(pp->dev, "PCIe: cannot reset pcie\n");
		goto err_clk_rst;
	}

	return 0;

err_clk_rst:
	clk_disable_unprepare(pcie->pcie_aux);

err_clk_aux:
	clk_disable_unprepare(pcie->pcie_ref);

err_clk_ref:

	return ret;
}

static void sunxi_pcie_clk_exit(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie_from_pp(pp);

	reset_control_assert(pcie->pcie_rst);
	clk_disable_unprepare(pcie->pcie_aux);
	clk_disable_unprepare(pcie->pcie_ref);
}

static int sunxi_pcie_clk_get(struct platform_device *pdev, struct sunxi_pcie *sunxi_pcie)
{
	int ret;

	sunxi_pcie->pcie_ref = devm_clk_get(&pdev->dev, "pclk_ref");
	if (IS_ERR(sunxi_pcie->pcie_ref))
		return PTR_ERR(sunxi_pcie->pcie_ref);

	if (sunxi_pcie->clk_freq_24M) {
		sunxi_pcie->pcie_hosc = devm_clk_get(&pdev->dev, "hosc");
		if (IS_ERR(sunxi_pcie->pcie_hosc))
			return PTR_ERR(sunxi_pcie->pcie_hosc);

		ret = clk_set_parent(sunxi_pcie->pcie_ref, sunxi_pcie->pcie_hosc);
		if (ret != 0) {
			return -1;
		}
	} else if (sunxi_pcie->clk_freq_100M) {
			sunxi_pcie->pcie_per = devm_clk_get(&pdev->dev, "pclk_per");
			if (IS_ERR(sunxi_pcie->pcie_per))
				return PTR_ERR(sunxi_pcie->pcie_per);

			ret = clk_set_parent(sunxi_pcie->pcie_ref, sunxi_pcie->pcie_per);
			if (ret != 0) {
				return -1;
		}
	} else {
		dev_err(&pdev->dev, "fail to get clock freq from dts!");
		return -1;
	}

	sunxi_pcie->pcie_aux = devm_clk_get(&pdev->dev, "pclk_aux");
	if (IS_ERR(sunxi_pcie->pcie_aux))
		return PTR_ERR(sunxi_pcie->pcie_aux);

	sunxi_pcie->pcie_rst = devm_reset_control_get(&pdev->dev, "pclk_rst");
	if (IS_ERR(sunxi_pcie->pcie_rst)) {
		return PTR_ERR(sunxi_pcie->pcie_rst);
	}

	return 0;
}

static int sunxi_pcie_combo_phy_init(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie_from_pp(pp);
	int ret;

	pcie->phy = devm_phy_get(pp->dev, "pcie-phy");
	if (IS_ERR(pcie->phy)) {
		return dev_err_probe(pp->dev, PTR_ERR(pcie->phy),
				     "missing PHY\n");
	}

	ret = phy_init(pcie->phy);
	if (ret < 0) {
		dev_err(pp->dev, "fail to init phy, err %d\n", ret);
		return ret;
	}

	return 0;
}

static irqreturn_t sunxi_pcie_plat_linkup_handler(int irq, void *arg)
{
	struct pcie_port *pp = (struct pcie_port *)arg;

	sunxi_pcie_irqpending(pp);

	return IRQ_HANDLED;
}

static void sunxi_pcie_dma_handle_interrupt(u32 chnl, enum dma_dir dma_trx)
{
	int ret = 0;

	ret = pcie_dma_chan_release(chnl, dma_trx);
	if (unlikely(ret < 0)) {
		pr_err("%s is error release chnl%d !\n", __func__, chnl);
	}
}

#define SUNXI_PCIE_DMA_IRQ_HANDLER(name, chn, dir)				\
static irqreturn_t sunxi_pcie_##name##_irq_handler				\
						(int irq, void *arg)		\
{										\
	struct sunxi_pcie *pci = arg;						\
	union int_status sta = {0};						\
	union int_clear  clr = {0};                                             \
												  \
	sta.dword = sunxi_pcie_readl_dbi(pci, PCIE_DMA_OFFSET +					  \
					(dir ? PCIE_DMA_RD_INT_STATUS : PCIE_DMA_WR_INT_STATUS)); \
												  \
	if (sta.done & BIT(chn)) {							          \
		clr.doneclr = BIT(chn);								  \
		sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET +					  \
				(dir ? PCIE_DMA_RD_INT_CLEAR : PCIE_DMA_WR_INT_CLEAR), clr.dword);\
		sunxi_pcie_dma_handle_interrupt(chn, dir);					  \
	}											  \
												  \
	if (sta.abort & BIT(chn)) {								  \
		clr.abortclr = BIT(chn);							  \
		sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET +					  \
				(dir ? PCIE_DMA_RD_INT_CLEAR : PCIE_DMA_WR_INT_CLEAR), clr.dword);\
		dev_err(&pci->dev, "DMA %s channel %d is abort\n",				  \
							dir ? "read":"write", chn);		  \
	}											  \
												  \
	return IRQ_HANDLED;									  \
}

SUNXI_PCIE_DMA_IRQ_HANDLER(dma_w0, 0, PCIE_DMA_WRITE)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_w1, 1, PCIE_DMA_WRITE)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_w2, 2, PCIE_DMA_WRITE)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_w3, 3, PCIE_DMA_WRITE)

SUNXI_PCIE_DMA_IRQ_HANDLER(dma_r0, 0, PCIE_DMA_READ)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_r1, 1, PCIE_DMA_READ)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_r2, 2, PCIE_DMA_READ)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_r3, 3, PCIE_DMA_READ)

static void sunxi_pcie_dma_read(struct sunxi_pcie *pci, struct dma_table *table, int PCIE_OFFSET)
{
	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_RD_ENB,
							table->enb.dword);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_RD_CTRL_LO,
							table->ctx_reg.ctrllo.dword);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_RD_CTRL_HI,
							table->ctx_reg.ctrlhi.dword);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_RD_XFERSIZE,
							table->ctx_reg.xfersize);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_RD_SAR_LO,
							table->ctx_reg.sarptrlo);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_RD_SAR_HI,
							table->ctx_reg.sarptrhi);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_RD_DAR_LO,
							table->ctx_reg.darptrlo);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_RD_DAR_HI,
							table->ctx_reg.darptrhi);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_RD_WEILO,
							table->weilo.dword);
	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_RD_DOORBELL,
							table->start.dword);
}

static void sunxi_pcie_dma_write(struct sunxi_pcie *pci, struct dma_table *table, int PCIE_OFFSET)
{
	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_WR_ENB,
							table->enb.dword);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_WR_CTRL_LO,
							table->ctx_reg.ctrllo.dword);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_WR_CTRL_HI,
							table->ctx_reg.ctrlhi.dword);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_WR_XFERSIZE,
							table->ctx_reg.xfersize);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_WR_SAR_LO,
							table->ctx_reg.sarptrlo);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_WR_SAR_HI,
							table->ctx_reg.sarptrhi);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_WR_DAR_LO,
							table->ctx_reg.darptrlo);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_WR_DAR_HI,
							table->ctx_reg.darptrhi);
	sunxi_pcie_writel_dbi(pci, PCIE_OFFSET + PCIE_DMA_WR_WEILO,
							table->weilo.dword);
	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_WR_DOORBELL,
							table->start.dword);
}

/*
 * DMA controller: I/O and Type 0 or Type 1 configuration DMA
 * transfers are not supported.
 * Transfer size: 1B - 4GB
 */
static void sunxi_pcie_dma_start(struct dma_table *table, struct dma_trx_obj *obj)
{
	struct sunxi_pcie *pci = dev_get_drvdata(obj->dev);
	int offset = PCIE_DMA_OFFSET + table->start.chnl * 0x200;

	if (table->dir == PCIE_DMA_READ) {
		sunxi_pcie_dma_read(pci, table, offset);
	} else if (table->dir == PCIE_DMA_WRITE) {
		sunxi_pcie_dma_write(pci, table, offset);
	}
}

static int sunxi_pcie_dma_config(struct dma_table *table, phys_addr_t sar_addr, phys_addr_t dar_addr,
					unsigned int size, enum dma_dir dma_trx)
{
	dma_channel_t *chn = NULL;

	table->ctx_reg.ctrllo.lie   = 0x1;
	table->ctx_reg.ctrllo.rie   = 0x0;
	table->ctx_reg.ctrllo.td    = 0x1;
	table->ctx_reg.ctrlhi.dword = 0x0;
	table->ctx_reg.xfersize = size;
	table->ctx_reg.sarptrlo = (u32)(sar_addr & 0xffffffff);
	table->ctx_reg.sarptrhi = (u32)(sar_addr >> 32);
	table->ctx_reg.darptrlo = (u32)(dar_addr & 0xffffffff);
	table->ctx_reg.darptrhi = (u32)(dar_addr >> 32);
	table->start.stop = 0x0;
	table->dir = dma_trx;

	chn = (dma_channel_t *)pcie_dma_chan_request(dma_trx);
	if (!chn) {
		pr_err("pcie request %s channel error! \n", (dma_trx ? "DMA_READ":"DMA_WRITE"));
		return -ENOMEM;
	}

	table->start.chnl = chn->chnl_num;

	if (dma_trx == PCIE_DMA_WRITE) {
		table->weilo.dword = (PCIE_WEIGHT << (5*chn->chnl_num));
		table->enb.enb = 0x1;
	} else {
		table->weilo.dword = (PCIE_WEIGHT << (5*chn->chnl_num));
		table->enb.enb = 0x1;
	}

	return 0;
}

static int sunxi_pcie_request_irq(struct sunxi_pcie *sunxi_pcie, struct platform_device *pdev)
{
	int irq, ret;

	irq  = platform_get_irq_byname(pdev, "sii");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq,
				 sunxi_pcie_plat_linkup_handler,
					IRQF_SHARED, "pcie-linkup", &sunxi_pcie->pp);
	if (ret) {
		dev_err(&pdev->dev, "PCIe failed to request linkup IRQ\n");
		return ret;
	}
	ret = pcie_dma_get_chan(pdev);
	if (ret)
		return -EINVAL;

	irq = platform_get_irq_byname(pdev, "edma-w0");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_w0_irq_handler,
			       IRQF_SHARED, "pcie-dma-w0", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-w1");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_w1_irq_handler,
			       IRQF_SHARED, "pcie-dma-w1", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-w2");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_w2_irq_handler,
			       IRQF_SHARED, "pcie-dma-w2", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-w3");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_w3_irq_handler,
			       IRQF_SHARED, "pcie-dma-w3", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-r0");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_r0_irq_handler,
			       IRQF_SHARED, "pcie-dma-r0", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-r1");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_r1_irq_handler,
			       IRQF_SHARED, "pcie-dma-r1", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-r2");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_r2_irq_handler,
			       IRQF_SHARED, "pcie-dma-r2", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-r3");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_r3_irq_handler,
			       IRQF_SHARED, "pcie-dma-r3", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	return 0;
}

int sunxi_pcie_init_dma_trx(struct pcie_port *pp)
{
	struct sunxi_pcie *pci = to_sunxi_pcie_from_pp(pp);

	pp->dma_obj = sunxi_pcie_dma_obj_probe(pp->dev);

	if (IS_ERR(pp->dma_obj)) {
		dev_err(pp->dev, "failed to prepare dma obj probe\n");
		return -EINVAL;
	}

	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_WR_INT_MASK, 0x0);
	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_RD_INT_MASK, 0x0);

	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_pcie_init_dma_trx);

static int sunxi_pcie_parse_dts_res(struct platform_device *pdev, struct sunxi_pcie *sunxi_pcie)
{
	struct pcie_port *pp = &sunxi_pcie->pp;
	struct device_node *np = pp->dev->of_node;
	struct resource *dbi_res;
	int ret;

	dbi_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (!dbi_res) {
		dev_err(&pdev->dev, "get pcie dbi failed\n");
		return -ENODEV;
	}

	sunxi_pcie->dbi_base = devm_ioremap_resource(&pdev->dev, dbi_res);
	if (IS_ERR(sunxi_pcie->dbi_base)) {
		dev_err(&pdev->dev, "ioremap pcie dbi failed\n");
		return PTR_ERR(sunxi_pcie->dbi_base);
	}

	pp->dbi_base = sunxi_pcie->dbi_base;
	sunxi_pcie->app_base = sunxi_pcie->dbi_base + USER_DEFINED_REGISTER_LIST;

	sunxi_pcie->link_gen = of_pci_get_max_link_speed(pdev->dev.of_node);
	if (sunxi_pcie->link_gen < 0) {
		dev_warn(&pdev->dev, "PCIe get speed Gen failed\n");
		sunxi_pcie->link_gen = 0x1;
	}

	sunxi_pcie->prsnt_gpio = of_get_named_gpio(pdev->dev.of_node, "reset-gpios", 0);
	if (!gpio_is_valid(sunxi_pcie->prsnt_gpio))
		dev_err(pp->dev, "Failed to get \"reset-gpios\"\n");
	else {
		gpio_direction_output(sunxi_pcie->prsnt_gpio, 1);
		__gpio_set_value(sunxi_pcie->prsnt_gpio, 1);
	}

	sunxi_pcie->pcie3v3 = devm_regulator_get_optional(&pdev->dev, "pcie3v3");
	if (IS_ERR(sunxi_pcie->pcie3v3)) {
		dev_info(pp->dev, "no pcie3v3 regulator found\n");
	}

	ret = sunxi_pcie_enable_power(sunxi_pcie);
	if (ret)
		dev_info(pp->dev, "enable regulator failed\n");

	ret = of_property_read_u32(np, "num-viewport", &pp->num_viewport);
	if (ret) {
		dev_err(pp->dev, "Failed to parse the number of num_viewport\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "num-lanes", &pp->lanes)) {
		dev_err(pp->dev, "Failed to parse the number of lanes\n");
		return -EINVAL;
	}

	sunxi_pcie->clk_freq_24M = false;
	sunxi_pcie->clk_freq_100M = false;

	if (device_property_read_bool(&pdev->dev, "clk-freq-24M")) {
		sunxi_pcie->clk_freq_24M = true;
	}

	if (device_property_read_bool(&pdev->dev, "clk-freq-100M")) {
		sunxi_pcie->clk_freq_100M = true;
	}

	pp->cpu_pcie_addr_quirk = sunxi_pcie->drvdata->cpu_pcie_addr_quirk;

	ret = sunxi_pcie_clk_get(pdev, sunxi_pcie);
	if (ret) {
		dev_err(pp->dev, "pcie get clk init failed\n");
		return -ENODEV;
	}

	ret = sunxi_pcie_clk_setup(pp);
	if (ret) {
		dev_err(pp->dev, "error: PCIe clk setup failed. \n");
		return -ENODEV;
	}

	ret = sunxi_pcie_combo_phy_init(pp);
	if (ret)
		goto deinit_clk;

	sunxi_pcie_irqmask(pp);

	return 0;

deinit_clk:
	sunxi_pcie_clk_exit(pp);

	return ret;
}

static int sunxi_pcie_plat_probe(struct platform_device *pdev)
{
	struct sunxi_pcie *sunxi_pcie;
	struct pcie_port *pp;
	const struct of_device_id *match;
	const struct sunxi_pcie_of_data *data;
	enum sunxi_pcie_device_mode	mode;
	int ret;

	match = of_match_device(sunxi_plat_pcie_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "PCIe probe match device id failed\n");
		return -EINVAL;
	}

	data = (struct sunxi_pcie_of_data *)match->data;
	mode = (enum sunxi_pcie_device_mode)data->mode;

	sunxi_pcie = devm_kzalloc(&pdev->dev, sizeof(*sunxi_pcie),
					GFP_KERNEL);
	if (!sunxi_pcie)
		return -ENOMEM;

	pp = &sunxi_pcie->pp;
	pp->dev = &pdev->dev;
	sunxi_pcie->dev = pdev->dev;
	sunxi_pcie->drvdata = data;

	ret = sunxi_pcie_parse_dts_res(pdev, sunxi_pcie);
	if (ret) {
		dev_err(pp->dev, "pcie parse dts failed: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, sunxi_pcie);

	ret = sunxi_pcie_request_irq(sunxi_pcie, pdev);
	if (ret) {
		dev_err(&pdev->dev, "pcie link irq init failed\n");
		goto deinit_clk;
	}

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "pm_runtime_get_sync failed\n");
		goto err_get_sync;
	}

	switch (sunxi_pcie->drvdata->mode) {
	case SUNXI_PCIE_RC_TYPE:
		ret = sunxi_plat_add_pcie_port(pp, pdev);
		break;
	case SUNXI_PCIE_EP_TYPE:
		sunxi_pcie_set_mode(sunxi_pcie);
		sunxi_pcie->ep.ops = &sunxi_ep_ops;
		ret = sunxi_plat_ep_init(&sunxi_pcie->ep);
		break;
	default:
		dev_err(&pdev->dev, "INVALID device type %d\n", sunxi_pcie->drvdata->mode);
		ret = -EINVAL;
		break;
	}

	if (ret)
		goto err_pcie_mode;

	if (pp->dma_obj) {
		pp->dma_obj->start_dma_trx_func  = sunxi_pcie_dma_start;
		pp->dma_obj->config_dma_trx_func = sunxi_pcie_dma_config;
	}

	return 0;

err_pcie_mode:
	pm_runtime_put(&pdev->dev);

err_get_sync:
	pm_runtime_disable(&pdev->dev);

deinit_clk:
	sunxi_pcie_clk_exit(pp);

	return ret;
}

static int sunxi_pcie_plat_remove(struct platform_device *pdev)
{
	struct sunxi_pcie *pcie = platform_get_drvdata(pdev);
	struct pcie_port *pp = &pcie->pp;
	int ret;

	if (pp->dma_obj) {
		sunxi_pcie_dma_obj_remove(pp->dev);
	}

	if (pp->bridge->bus) {
		pci_stop_root_bus(pp->bridge->bus);
		pci_remove_root_bus(pp->bridge->bus);
	}

	if (IS_ENABLED(CONFIG_PCI_MSI) && !pp->msi_ext) {
		irq_domain_remove(pp->msi_domain);
		irq_domain_remove(pp->irq_domain);
	}

	sunxi_pcie_ltssm_enable(pcie, false);

	ret = pm_runtime_put_sync(&pdev->dev);
	if (ret < 0)
		dev_err(&pdev->dev, "pm_runtime_put_sync failed\n");

	pm_runtime_disable(&pdev->dev);

	phy_exit(pcie->phy);

	sunxi_pcie_clk_exit(pp);

	sunxi_pcie_disable_power(pcie);

	return 0;
}

static int sunxi_pcie_hw_resume(struct sunxi_pcie *pci)
{
	struct pcie_port *pp = &pci->pp;
	int ret;

	sunxi_pcie_enable_power(pci);

	if (sunxi_pcie_clk_setup(pp)) {
		dev_err(pp->dev, "Failed to setup clk in hw resume\n");
		return -1;
	}

	ret = phy_init(pci->phy);
	if (ret < 0) {
		dev_err(pp->dev, "fail to init phy in resume, err %d\n", ret);
		return ret;
	}

	usleep_range(100, 300);

	sunxi_pcie_ltssm_enable(pci, false);

	sunxi_pcie_setup_rc(pp);

	if (IS_ENABLED(CONFIG_PCI_MSI) && !pp->msi_ext) {

		phys_addr_t pa = ALIGN_DOWN(virt_to_phys(pp), SZ_4K);

		sunxi_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_LO, 4,	lower_32_bits(pa));
		sunxi_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_HI, 4, upper_32_bits(pa));
	}

	sunxi_pcie_establish_link(pp);

	sunxi_pcie_speed_chang(pci, pci->link_gen);

	return 0;
}

static int sunxi_pcie_hw_suspend(struct sunxi_pcie *pcie)
{
	struct pcie_port *pp = &pcie->pp;

	phy_exit(pcie->phy);

	sunxi_pcie_clk_exit(pp);

	sunxi_pcie_disable_power(pcie);

	return 0;
}

static int sunxi_pcie_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sunxi_pcie *sunxi_pcie = platform_get_drvdata(pdev);

	sunxi_pcie_ltssm_enable(sunxi_pcie, false);

	usleep_range(200, 300);

	sunxi_pcie_hw_suspend(sunxi_pcie);

	return 0;
}

static int sunxi_pcie_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sunxi_pcie *pcie = platform_get_drvdata(pdev);
	int ret;

	ret = sunxi_pcie_hw_resume(pcie);
	if (ret < 0) {
		dev_err(&pcie->dev, "fail to resume, err %d\n", ret);
		return ret;
	}

	return 0;
}

static struct dev_pm_ops sunxi_pcie_pm_ops = {
	.suspend_noirq = sunxi_pcie_suspend,
	.resume_noirq = sunxi_pcie_resume,
};

static struct platform_driver sunxi_plat_pcie_driver = {
	.driver = {
		.name	= "sunxi-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = sunxi_plat_pcie_of_match,
		.pm = &sunxi_pcie_pm_ops,
	},
	.probe  = sunxi_pcie_plat_probe,
	.remove = sunxi_pcie_plat_remove,
};

module_platform_driver(sunxi_plat_pcie_driver);

MODULE_AUTHOR("songjundong <songjundong@allwinnertech.com>");
MODULE_DESCRIPTION("Allwinner PCIe controller platform driver");
MODULE_VERSION("1.0.6");
MODULE_LICENSE("GPL v2");
