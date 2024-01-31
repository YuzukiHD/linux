/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022 Allwinner.
 *
 * SUNXI SPI Controller DBI Driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#include "spi-sunxi.h"
#include "spi-sunxi-debug.h"
#include "spi-sunxi-dbi.h"

static int sunxi_dbi_debug_mask;
module_param_named(dbi_debug_mask, sunxi_dbi_debug_mask, int, 0664);

void sunxi_dbi_disable_dma(void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + SUNXI_DBI_CTL2_REG);

	reg_val &= ~(SUNXI_DBI_CTL2_FIFO_DRQ_EN);
	writel(reg_val, base_addr + SUNXI_DBI_CTL2_REG);
}

void sunxi_dbi_enable_dma(void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + SUNXI_DBI_CTL2_REG);

	reg_val |= SUNXI_DBI_CTL2_FIFO_DRQ_EN;
	writel(reg_val, base_addr + SUNXI_DBI_CTL2_REG);
}

u32 sunxi_dbi_qry_irq_pending(void __iomem *base_addr)
{
	return (SUNXI_DBI_INT_STA_MASK & readl(base_addr + SUNXI_DBI_INT_REG));
}

void sunxi_dbi_clr_irq_pending(void __iomem *base_addr, u32 pending_bit)
{
	pending_bit &= SUNXI_DBI_INT_STA_MASK;
	writel(pending_bit, base_addr + SUNXI_DBI_INT_REG);
}

void sunxi_dbi_disable_irq(void __iomem *base_addr, u32 bitmap)
{
	u32 reg_val = readl(base_addr + SUNXI_DBI_INT_REG);

	bitmap &= SUNXI_DBI_INT_STA_MASK;
	reg_val &= ~bitmap;
	writel(reg_val, base_addr + SUNXI_DBI_INT_REG);
}

void sunxi_dbi_enable_irq(void __iomem *base_addr, u32 bitmap)
{
	u32 reg_val = readl(base_addr + SUNXI_DBI_INT_REG);


	bitmap &= SUNXI_DBI_INT_STA_MASK;
	reg_val |= bitmap;
	writel(reg_val, base_addr + SUNXI_DBI_INT_REG);
}

static s32 sunxi_dbi_set_timer_param(struct spi_device *spi)
{
	return -1;
}

void sunxi_dbi_config(struct spi_device *spi)
{
	struct sunxi_spi *sspi = spi_controller_get_devdata(spi->controller);
	u32 reg_val = 0;
	u32 reg_tmp = 0;
	u32 dbi_mode = sspi->dbi_config->dbi_mode;

	/* command type */
	if (dbi_mode & SUNXI_DBI_COMMAND_READ) {
		reg_val |= SUNXI_DBI_CTL0_CMDT;
		reg_tmp = readl(sspi->base_addr + SUNXI_DBI_CTL1_REG);
		writel(reg_tmp | sspi->dbi_config->dbi_read_bytes,
			sspi->base_addr + SUNXI_DBI_CTL1_REG);
	} else
		reg_val &= ~SUNXI_DBI_CTL0_CMDT;

	/* output data sequence */
	if (dbi_mode & SUNXI_DBI_LSB_FIRST)
		reg_val |= SUNXI_DBI_CTL0_DAT_SEQ;
	else
		reg_val &= ~SUNXI_DBI_CTL0_DAT_SEQ;

	/* transmit data type */
	if (dbi_mode & SUNXI_DBI_TRANSMIT_VIDEO) {
		reg_val |= SUNXI_DBI_CTL0_TRAN_MOD;
		writel((sspi->dbi_config->dbi_video_v << 16) | (sspi->dbi_config->dbi_video_h),
			sspi->base_addr + SUNXI_DBI_VIDEO_SIZE_REG);
		if (sspi->dbi_config->dbi_te_en)
			sunxi_dbi_enable_irq(sspi->base_addr, SUNXI_DBI_TE_INT_EN);
		else
			sunxi_dbi_enable_irq(sspi->base_addr, SUNXI_DBI_FRAM_DONE_INT_EN);
	} else {
		reg_val &= ~SUNXI_DBI_CTL0_TRAN_MOD;

		writel(0x0, sspi->base_addr + SUNXI_DBI_VIDEO_SIZE_REG);
		sunxi_dbi_disable_irq(sspi->base_addr, SUNXI_DBI_FRAM_DONE_INT_EN | SUNXI_DBI_TE_INT_EN);
		sunxi_dbi_enable_irq(sspi->base_addr, SUNXI_DBI_FIFO_EMPTY_INT_EN);
	}


	/* output data format */
	reg_val &= ~(SUNXI_DBI_CTL0_DAT_FMT_MASK);
	if (sspi->dbi_config->dbi_format == SUNXI_DBI_RGB111)
		reg_val &= ~(0x7 << SUNXI_DBI_CTL0_DAT_FMT);
	else
		reg_val |= ((sspi->dbi_config->dbi_format) << SUNXI_DBI_CTL0_DAT_FMT);

	/* dbi interface select */
	reg_val &= ~(SUNXI_DBI_CTL0_INTF_MASK);

	if (sspi->dbi_config->dbi_interface == SUNXI_DBI_L3I1)
		reg_val &= ~((0x7) << SUNXI_DBI_CTL0_INTF);
	else
		reg_val |= ((sspi->dbi_config->dbi_interface) << SUNXI_DBI_CTL0_INTF);

	if (sspi->dbi_config->dbi_format <= SUNXI_DBI_RGB565)
		reg_val |= 0x1;
	else
		reg_val &= ~0x1;

	if (sspi->dbi_config->dbi_out_sequence == SUNXI_DBI_OUT_RGB)
		reg_val &= ~((0x7) << 16);
	else
		reg_val |= ((sspi->dbi_config->dbi_out_sequence) << 16);

	if (sspi->dbi_config->dbi_src_sequence == SUNXI_DBI_SRC_RGB)
		reg_val &= ~((0xf) << 4);
	else
		reg_val |= ((sspi->dbi_config->dbi_src_sequence) << 4);

	if (sspi->dbi_config->dbi_rgb_bit_order == 1)
		reg_val |= ((0x1) << 2);
	else
		reg_val &= ~((0x1) << 2);

	if (sspi->dbi_config->dbi_rgb32_alpha_pos == 1)
		reg_val |= ((0x1) << 1);
	else
		reg_val &= ~((0x1) << 1);

	writel(reg_val, sspi->base_addr + SUNXI_DBI_CTL0_REG);

	reg_val = 0;

	if (sspi->dbi_config->dbi_interface == SUNXI_DBI_D2LI) {
		reg_val |= SUNXI_DBI_CTL2_DCX_SEL;
		reg_val &= ~SUNXI_DBI_CTL2_SDI_OUT_SEL;
	} else {
		reg_val |= SUNXI_DBI_CTL2_SDI_OUT_SEL;
		reg_val &= ~SUNXI_DBI_CTL2_DCX_SEL;
	}

	if ((sspi->dbi_config->dbi_te_en == SUNXI_DBI_TE_DISABLE) ||
		!(dbi_mode & SUNXI_DBI_TRANSMIT_VIDEO)) {
		reg_val &= ~(0x3 << 0); /* te disable */
	} else {
		/* te enable */
		reg_val |= 0x1;
		if (sspi->dbi_config->dbi_te_en == SUNXI_DBI_TE_FALLING_EDGE)
			reg_val |= (0x1 << 1);
		else
			reg_val &= ~(0x1 << 1);
	}

	writel(reg_val, sspi->base_addr + SUNXI_DBI_CTL2_REG);

	dev_dbg(sspi->dev, "DBI mode configurate : 0x%x\n", reg_val);

	reg_val = 0;
	if (dbi_mode & SUNXI_DBI_DCX_DATA)
		reg_val |= SUNXI_DBI_CTL1_DCX_DATA;
	else
		reg_val &= ~SUNXI_DBI_CTL1_DCX_DATA;

	if (sspi->dbi_config->dbi_rgb16_pixel_endian == 1)
		reg_val |= ((0x1) << 21);
	else
		reg_val &= ~((0x1) << 21);

	/* dbi en mode sel */
	if ((sspi->dbi_config->dbi_te_en == SUNXI_DBI_TE_DISABLE) ||
		!(dbi_mode & SUNXI_DBI_TRANSMIT_VIDEO)) {
		if (!sunxi_dbi_set_timer_param(spi))
			reg_val |= (0x2 << 29); /* timer trigger mode */
		else
			reg_val &= ~(0x3 << 29); /* always on mode */
	} else {
		/* te trigger mode */
		reg_val |= ((0x3) << 29);
	}

	/* config dbi clock mode: auto gating */
	if (sspi->dbi_config->dbi_clk_out_mode == SUNXI_DBI_CLK_ALWAYS_ON)
		reg_val &= ~(SUNXI_DBI_CTL1_CLKO_MOD);
	else
		reg_val |= SUNXI_DBI_CTL1_CLKO_MOD;

	writel(reg_val, sspi->base_addr + SUNXI_DBI_CTL1_REG);

	if (sunxi_dbi_debug_mask & SUNXI_SPI_DEBUG_DUMP_REG)
		sunxi_spi_dump_reg(sspi->dev, sspi->base_addr + 0x100, sspi->base_addr_phy + 0x100, 0x30);
}

int sunxi_dbi_get_config(const struct spi_device *spi, struct sunxi_dbi_config *dbi_config)
{
	struct sunxi_spi *sspi = spi_controller_get_devdata(spi->controller);

	if (sspi->bus_mode != SUNXI_SPI_BUS_DBI)
		return -EINVAL;

	memcpy(dbi_config, sspi->dbi_config, sizeof(*dbi_config));

	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_dbi_get_config);

int sunxi_dbi_set_config(struct spi_device *spi, const struct sunxi_dbi_config *dbi_config)
{
	struct sunxi_spi *sspi = spi_controller_get_devdata(spi->controller);

	if (sspi->bus_mode != SUNXI_SPI_BUS_DBI)
		return -EINVAL;

	memcpy(sspi->dbi_config, dbi_config, sizeof(*dbi_config));

	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_dbi_set_config);
