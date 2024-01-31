/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022 Allwinner.
 *
 * SUNXI SPI Controller DBI Register Definition
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#ifndef _SUNXI_SPI_DBI_H_
#define _SUNXI_SPI_DBI_H_

#include <linux/ctype.h>
#include <linux/spi/spi.h>

/* DBI Control Register 0 */
#define SUNXI_DBI_CTL0_REG		(0x100)
	#define SUNXI_DBI_CTL0_CMDT			(1 << 31)	/* Command Type */
	#define SUNXI_DBI_CTL0_WCDC			(0x7ff << 20)	/* Write Command Dummy Cycles */
	#define SUNXI_DBI_CTL0_DAT_SEQ		(1 << 19)	/* Output Data Sequence */
	#define SUNXI_DBI_CTL0_RGB_SEQ		(7 << 16)	/* Output RGB Sequence */
	#define SUNXI_DBI_CTL0_TRAN_MOD		(1 << 15)	/* Transmit Mode */
	#define SUNXI_DBI_CTL0_DAT_FMT		(12)		/* Output Data Format */
	#define SUNXI_DBI_CTL0_DAT_FMT_MASK	(7 << SUNXI_DBI_CTL0_DAT_FMT)
	#define SUNXI_DBI_CTL0_INTF			(8)			/* DBI Interface */
	#define SUNXI_DBI_CTL0_INTF_MASK	(7 << SUNXI_DBI_CTL0_INTF)
	#define SUNXI_DBI_CTL0_RGB_FMT		(4)			/* RGB Source Format */
	#define SUNXI_DBI_CTL0_RGB_FMT_MASK	(0xf << SUNXI_DBI_CTL0_RGB_FMT)
	#define SUNXI_DBI_CTL0_DUM_VAL		(1 << 3)	/* Dummy Cycle Value */
	#define SUNXI_DBI_CTL0_RGB_BO		(1 << 2)	/* RGB Bit Order */
	#define SUNXI_DBI_CTL0_ELEMENT_A_POS	(1 << 1)/* Element A position, only For RGB32 Data Format */
	#define SUNXI_DBI_CTL0_VI_SRC_TYPE	(1 << 0)	/* RGB Source Type */

/* DBI Control Register 1 */
#define SUNXI_DBI_CTL1_REG		(0x104)
	#define SUNXI_DBI_CTL1_SOFT_TRG		(1 << 31)
	#define SUNXI_DBI_CTL1_EN_MOD_SEL	(3 << 29)	/* DBI Enable Mode Select */
	#define SUNXI_DBI_CTL1_RGB666_FMT	(3 << 26)	/* 2 Data Lane RGB666 Format */
	#define SUNXI_DBI_CTL1_RXCLK_INV	(1 << 25)	/* DBI RX clock inverse */
	#define SUNXI_DBI_CTL1_CLKO_MOD		(1 << 24)	/* DBI output clock mode */
	#define SUNXI_DBI_CTL1_CLKO_INV		(1 << 23)	/* DBI clock output inverse */
	#define SUNXI_DBI_CTL1_DCX_DATA		(1 << 22)	/* DCX Data Value */
	#define SUNXI_DBI_CTL1_RGB16_SEL	(1 << 21)	/* RGB 16 Data Source Select */
	#define SUNXI_DBI_CTL1_RDAT_LSB		(1 << 20)	/* Bit Order of Read Data */
	#define SUNXI_DBI_CTL1_RCDC			(0xff << 8)	/* Read Command Dummy Cycles */
	#define SUNXI_DBI_CTL1_RDBN			(0xff << 0)	/* Read Data Number of Bytes */

/* DBI Control Register 2 */
#define SUNXI_DBI_CTL2_REG		(0x108)
	#define SUNXI_DBI_CTL2_HRDY_BYP			(1 << 31)	/* AHB Ready bypass */
	#define SUNXI_DBI_CTL2_MOD02_BYP_MOSI	(1 << 30)	/* Mode 0/2 last bit lost bug bypass (mosi) */
	#define SUNXI_DBI_CTL2_MOD02_BYP_MISO	(1 << 29)	/* Mode 0/2 last bit lost bug bypass (miso) */
	#define SUNXI_DBI_CTL2_MOD02_BYP_OEN	(1 << 28)	/* Mode 0/2 last bit lost bug bypass (oen) */
	#define SUNXI_DBI_CTL2_MOD02_BYP_QUAEN	(1 << 27)	/* Mode 0/2 last bit lost bug bypass (quad_en) */
	#define SUNXI_DBI_CTL2_FIFO_DRQ_EN		(1 << 15)	/* DBI FIFO DMA Request Enable */
	#define SUNXI_DBI_CTL2_TX_TRIG_LEVEL	(0x7f << 8)	/* DBI FIFO Empty Request Trigger Level */
	#define SUNXI_DBI_CTL2_SDI_OUT_SEL		(1 << 6)	/* DBI SDI PIN Output select */
	#define SUNXI_DBI_CTL2_DCX_SEL			(1 << 5)	/* DBI SDX PIN Function select */
	#define SUNXI_DBI_CTL2_SDI_SEL			(3 << 3)	/* DBI SDI PIN Function select */
	#define SUNXI_DBI_CTL2_TE_DBC_SEL		(1 << 2)	/* TE debounce function select */
	#define SUNXI_DBI_CTL2_TE_TRIG_SEL		(1 << 1)	/* TE edge trigger select */
	#define SUNXI_DBI_CTL2_TE_EN			(1 << 0)	/* TE enable */

/* DBI Timer Register */
#define SUNXI_DBI_TIMER_REG		(0x10C)
	#define SUNXI_DBI_TIMER_TM_EN	(1 << 31)			/* DBI Timer enable */
	#define SUNXI_DBI_TIMER_TM_VAL	(0x7fffffff << 0)	/* DBI Timer value */

/* DBI Video Size Register */
#define SUNXI_DBI_VIDEO_SIZE_REG	(0x110)
	#define SUNXI_DBI_VIDEO_SIZE_V		(0x7ff << 16)	/* V Frame int */
	#define SUNXI_DBI_VIDEO_SIZE_H		(0x7ff << 0)	/* H Line int */

/* DBI Interrupter Register */
#define SUNXI_DBI_INT_REG		(0x120)
	#define SUNXI_DBI_FIFO_EMPTY_INT		(1 << 14)	/* DBI FIFO Empty Interrupt Status */
	#define SUNXI_DBI_FIFO_FULL_INT			(1 << 13)	/* DBI FIFO Full Interrupt Status */
	#define SUNXI_DBI_TIMER_INT				(1 << 12)	/* Timer Interrupt Status */
	#define SUNXI_DBI_RD_DONE_INT			(1 << 11)	/* Read Done Interrupt Status */
	#define SUNXI_DBI_TE_INT				(1 << 10)	/* TE Interrupt Status */
	#define SUNXI_DBI_FRAM_DONE_INT			(1 << 9)	/* Frame Done Interrupt Status */
	#define SUNXI_DBI_LINE_DONE_INT			(1 << 8)	/* Line Done Interrupt Status */
	#define SUNXI_DBI_FIFO_EMPTY_INT_EN		(1 << 6)	/* DBI FIFO Empty Interrupt Enable */
	#define SUNXI_DBI_FIFO_FULL_INT_EN		(1 << 5)	/* DBI FIFO Full Interrupt Enable */
	#define SUNXI_DBI_TIMER_INT_EN			(1 << 4)	/* Timer Interrupt Enable */
	#define SUNXI_DBI_RD_DONE_INT_EN		(1 << 3)	/* Read Done Interrupt Enable */
	#define SUNXI_DBI_TE_INT_EN				(1 << 2)	/* TE Interrupt Enable */
	#define SUNXI_DBI_FRAM_DONE_INT_EN		(1 << 1)	/* Frame Done Interrupt Enable */
	#define SUNXI_DBI_LINE_DONE_INT_EN		(1 << 0)	/* Line Done Interrupt Enable */
	#define SUNXI_DBI_INT_STA_MASK			(0x7f7f)

/* DBI debug Register 0 */
#define SUNXI_DBI_DEBUG_REG0	(0x124)

/* DBI debug Register 1 */
#define SUNXI_DBI_DEBUG_REG1	(0x128)


#define SUNXI_DBI_RGB111	(0x0)
#define SUNXI_DBI_RGB444	(0x1)
#define SUNXI_DBI_RGB565	(0x2)
#define SUNXI_DBI_RGB666	(0x3)
#define SUNXI_DBI_RGB888	(0x4)

#define SUNXI_DBI_L3I1		(0x0)
#define SUNXI_DBI_L3I2		(0x1)
#define SUNXI_DBI_L4I1		(0x2)
#define SUNXI_DBI_L4I2		(0x3)
#define SUNXI_DBI_D2LI		(0x4)

#define SUNXI_DBI_COMMAND_READ		(0x10)
#define SUNXI_DBI_LSB_FIRST			(0x20)
#define SUNXI_DBI_TRANSMIT_VIDEO	(0x40)
#define SUNXI_DBI_DCX_DATA			(0x80)

#define SUNXI_DBI_CLK_AUTO_GATING	(0x0)
#define SUNXI_DBI_CLK_ALWAYS_ON		(0x1)

enum sunxi_dbi_out_seq {
	SUNXI_DBI_OUT_RGB = 0,
	SUNXI_DBI_OUT_RBG = 1,
	SUNXI_DBI_OUT_GRB = 2,
	SUNXI_DBI_OUT_GBR = 3,
	SUNXI_DBI_OUT_BRG = 4,
	SUNXI_DBI_OUT_BGR = 5,
};

enum sunxi_dbi_src_seq {
	SUNXI_DBI_SRC_RGB = 0,
	SUNXI_DBI_SRC_RBG = 1,
	SUNXI_DBI_SRC_GRB = 2,
	SUNXI_DBI_SRC_GBR = 3,
	SUNXI_DBI_SRC_BRG = 4,
	SUNXI_DBI_SRC_BGR = 5,
	/* following definition only for rgb565
	 * to change the RGB order in two byte(16 bit).
	 * format:R(5bit)--G_1(3bit)--G_0(3bit)--B(5bit)
	 * G_0 mean the low 3 bit of G component
	 * G_1 mean the high 3 bit of G component
	 *  */
	SUNXI_DBI_SRC_GRBG_0 = 6,
	SUNXI_DBI_SRC_GRBG_1 = 7,
	SUNXI_DBI_SRC_GBRG_0 = 8,
	SUNXI_DBI_SRC_GBRG_1 = 9,
};

enum sunxi_dbi_te_en {
	SUNXI_DBI_TE_DISABLE = 0,
	SUNXI_DBI_TE_RISING_EDGE = 1,
	SUNXI_DBI_TE_FALLING_EDGE = 2,
};

struct sunxi_dbi_config {
	enum sunxi_dbi_src_seq	dbi_src_sequence;
	enum sunxi_dbi_out_seq	dbi_out_sequence;
	enum sunxi_dbi_te_en dbi_te_en;
	u8 dbi_rgb_bit_order;
	u8 dbi_rgb32_alpha_pos;
	u8 dbi_rgb16_pixel_endian;
	u8 dbi_format; /* DBI OUT format */
	u8 dbi_interface;
	u16 dbi_mode;
	u8 dbi_clk_out_mode;
	u16 dbi_video_v;
	u16 dbi_video_h;
	u8 dbi_fps;
	void (*dbi_vsync_handle)(unsigned long data);
	u8 dbi_read_bytes;
};

extern void sunxi_dbi_disable_dma(void __iomem *base_addr);
extern void sunxi_dbi_enable_dma(void __iomem *base_addr);
extern u32 sunxi_dbi_qry_irq_pending(void __iomem *base_addr);
extern void sunxi_dbi_clr_irq_pending(void __iomem *base_addr, u32 pending_bit);
extern void sunxi_dbi_disable_irq(void __iomem *base_addr, u32 bitmap);
extern void sunxi_dbi_enable_irq(void __iomem *base_addr, u32 bitmap);
extern void sunxi_dbi_config(struct spi_device *spi);

extern int sunxi_dbi_get_config(const struct spi_device *spi, struct sunxi_dbi_config *dbi_config);
extern int sunxi_dbi_set_config(struct spi_device *spi, const struct sunxi_dbi_config *dbi_config);

#endif /* _SUNXI_SPI_DBI_H_ */