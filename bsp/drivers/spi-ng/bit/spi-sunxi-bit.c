/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022 Allwinner.
 *
 * SUNXI SPI Controller BIT Driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>

#include "spi-sunxi.h"
#include "spi-sunxi-debug.h"
#include "spi-sunxi-bit.h"

/* SPI Controller Hardware Register Operation Start */

void sunxi_spi_bit_start_xfer(void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + SUNXI_SPI_BATC_REG);

	reg_val |= SUNXI_SPI_BATC_TCE;
	writel(reg_val, base_addr + SUNXI_SPI_BATC_REG);
}

static void sunxi_spi_bit_sample_mode(void __iomem *base_addr, u8 mode)
{
	u32 reg_val = readl(base_addr + SUNXI_SPI_BATC_REG);

	if (mode)
		reg_val |= SUNXI_SPI_BATC_MSMS;
	else
		reg_val &= ~SUNXI_SPI_BATC_MSMS;

	writel(reg_val, base_addr + SUNXI_SPI_BATC_REG);
}

u32 sunxi_spi_bit_qry_irq_pending(void __iomem *base_addr)
{
	return (SUNXI_SPI_BATC_TBC & readl(base_addr + SUNXI_SPI_BATC_REG));
}

void sunxi_spi_bit_clr_irq_pending(void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + SUNXI_SPI_BATC_REG);

	reg_val |= SUNXI_SPI_BATC_TBC;
	writel(reg_val, base_addr + SUNXI_SPI_BATC_REG);
}

void sunxi_spi_bit_enable_irq(void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + SUNXI_SPI_BATC_REG);

	reg_val |= SUNXI_SPI_BATC_TBC_INT_EN;
	writel(reg_val, base_addr + SUNXI_SPI_BATC_REG);
}

void sunxi_spi_bit_disable_irq(void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + SUNXI_SPI_BATC_REG);

	reg_val &= ~SUNXI_SPI_BATC_TBC_INT_EN;
	writel(reg_val, base_addr + SUNXI_SPI_BATC_REG);
}

void sunxi_spi_bit_set_bc(void __iomem *base_addr, u32 tx_len, u32 rx_len)
{
	u32 reg_val = readl(base_addr + SUNXI_SPI_BATC_REG);

	reg_val &= ~(SUNXI_SPI_BATC_RX_FRM_LEN | SUNXI_SPI_BATC_TX_FRM_LEN);
	reg_val |= FIELD_PREP(SUNXI_SPI_BATC_TX_FRM_LEN, tx_len);
	reg_val |= FIELD_PREP(SUNXI_SPI_BATC_RX_FRM_LEN, rx_len);

	writel(reg_val, base_addr + SUNXI_SPI_BATC_REG);
}

static void sunxi_spi_bit_ss_level(void __iomem *base_addr, u8 status)
{
	u32 reg_val = readl(base_addr + SUNXI_SPI_BATC_REG);

	if (status)
		reg_val |= SUNXI_SPI_BATC_SS_LEVEL;
	else
		reg_val &= ~SUNXI_SPI_BATC_SS_LEVEL;
	writel(reg_val, base_addr + SUNXI_SPI_BATC_REG);
}

void sunxi_spi_bit_ss_owner(void __iomem *base_addr, u32 owner)
{
	u32 reg_val = readl(base_addr + SUNXI_SPI_BATC_REG);

	switch (owner) {
	case SUNXI_SPI_CS_AUTO:
		reg_val &= ~SUNXI_SPI_BATC_SS_OWNER;
		break;
	case SUNXI_SPI_CS_SOFT:
		reg_val |= SUNXI_SPI_BATC_SS_OWNER;
		break;
	}

	writel(reg_val, base_addr + SUNXI_SPI_BATC_REG);
}

void sunxi_spi_bit_ss_polarity(void __iomem *base_addr, bool pol)
{
	u32 reg_val = readl(base_addr + SUNXI_SPI_BATC_REG);

	if (pol)
		reg_val |= SUNXI_SPI_BATC_SPOL; /* default SSPOL = 1,Low level effect */
	else
		reg_val &= ~SUNXI_SPI_BATC_SPOL;

	writel(reg_val, base_addr + SUNXI_SPI_BATC_REG);
}

static int sunxi_spi_bit_ss_select(void __iomem *base_addr, u16 cs)
{
	int ret = 0;
	u32 reg_val;

	if (cs < SUNXI_SPI_CS_MAX) {
		reg_val = readl(base_addr + SUNXI_SPI_BATC_REG);
		reg_val &= ~SUNXI_SPI_BATC_SS_SEL;
		reg_val |= FIELD_PREP(SUNXI_SPI_BATC_SS_SEL, cs);
		writel(reg_val, base_addr + SUNXI_SPI_BATC_REG);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

/* SPI Controller Hardware Register Operation End */

void sunxi_spi_bit_set_cs(struct spi_device *spi, bool status)
{
	struct sunxi_spi *sspi = spi_controller_get_devdata(spi->controller);
	int ret;

	ret = sunxi_spi_bit_ss_select(sspi->base_addr, spi->chip_select);
	if (ret < 0) {
		dev_warn(sspi->dev, "bit cs %d over range, need control by software\n", spi->chip_select);
		return ;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	if (spi->cs_gpiod)
#else
	if (spi->cs_gpiod || gpio_is_valid(spi->cs_gpio))
#endif
		return ;

	sunxi_spi_bit_ss_polarity(sspi->base_addr, !(spi->mode & SPI_CS_HIGH));

	if (sspi->cs_mode == SUNXI_SPI_CS_SOFT)
		sunxi_spi_bit_ss_level(sspi->base_addr, status);
}

int sunxi_spi_bit_sample_delay(struct spi_device *spi, u32 clk)
{
	struct sunxi_spi *sspi = spi_controller_get_devdata(spi->controller);

	if (clk > SUNXI_SPI_SAMP_LOW_FREQ)
		sunxi_spi_bit_sample_mode(sspi->base_addr, 1);
	else
		sunxi_spi_bit_sample_mode(sspi->base_addr, 0);

	return 0;
}

void sunxi_spi_bit_config_tc(void __iomem *base_addr, u32 config)
{
	u32 reg_val = readl(base_addr + SUNXI_SPI_BATC_REG);

	reg_val &= ~SUNXI_SPI_BATC_WMS;
	if (config & SPI_3WIRE)
		reg_val |= FIELD_PREP(SUNXI_SPI_BATC_WMS, SUNXI_SPI_BIT_3WIRE_MODE);
	else
		reg_val |= FIELD_PREP(SUNXI_SPI_BATC_WMS, SUNXI_SPI_BIT_STD_MODE);

	writel(reg_val, base_addr + SUNXI_SPI_BATC_REG);
}

int sunxi_spi_bit_cpu_rx(struct spi_device *spi, struct spi_transfer *t)
{
	struct sunxi_spi *sspi = spi_controller_get_devdata(spi->controller);
	u8 bits = t->bits_per_word;
	u8 *buf = (u8 *)t->rx_buf;
	u32 data;

	if (bits <= 8) {
		data = readb(sspi->base_addr + SUNXI_SPI_RB_REG);
		*((u8 *)buf) = data & (BIT(bits) - 1);
	} else if (bits <= 16) {
		data = readw(sspi->base_addr + SUNXI_SPI_RB_REG);
		*((u16 *)buf) = data & (BIT(bits) - 1);
	} else {
		data = readl(sspi->base_addr + SUNXI_SPI_RB_REG);
		*((u32 *)buf) = data & (BIT(bits) - 1);
	}

	dev_dbg(sspi->dev, "bit cpu rx data %#x\n", data);

	return 0;
}

int sunxi_spi_bit_cpu_tx(struct spi_device *spi, struct spi_transfer *t)
{
	struct sunxi_spi *sspi = spi_controller_get_devdata(spi->controller);
	u8 bits = t->bits_per_word;
	u8 *buf = (u8 *)t->tx_buf;
	u32 data;

	if (bits <= 8) {
		data = *((u8 *)buf) & (BIT(bits) - 1);
		writeb(data, sspi->base_addr + SUNXI_SPI_TB_REG);
	} else if (bits <= 16) {
		data = *((u16 *)buf) & (BIT(bits) - 1);
		writew(data, sspi->base_addr + SUNXI_SPI_TB_REG);
	} else {
		data = *((u32 *)buf) & (BIT(bits) - 1);
		writel(data, sspi->base_addr + SUNXI_SPI_TB_REG);
	}

	dev_dbg(sspi->dev, "bit cpu tx data %#x\n", data);

	return 0;
}
