/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022 Allwinner.
 *
 * SUNXI SPI Controller Debug Function Definition
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef _SUNXI_SPI_DEBUG_H_
#define _SUNXI_SPI_DEBUG_H_

/* Debug Dump */
#define HEXADECIMAL	(0x10)
#define REG_END		(0x0f)
#define SAMPLE_NUMBER	(0x80)
#define REG_INTERVAL	(0x04)
#define REG_CL		(0x0c)

enum {
	SUNXI_SPI_DEBUG_DUMP_REG	= BIT(0),
	SUNXI_SPI_DEBUG_DUMP_DATA	= BIT(1),
};

__maybe_unused static void sunxi_spi_dump_reg(struct device *dev, void __iomem *vaddr, u32 paddr, u32 len)
{
	u32 i;
	u8 buf[64], cnt = 0;

	for (i = 0; i < len; i = i + REG_INTERVAL) {
		if (i%HEXADECIMAL == 0)
			cnt += sprintf(buf + cnt, "0x%08x: ",
					(u32)(paddr + i));

		cnt += sprintf(buf + cnt, "%08x ",
				readl(vaddr + i));

		if (i%HEXADECIMAL == REG_CL) {
			dev_info(dev, "%s\n", buf);
			cnt = 0;
		}
	}
}

__maybe_unused static void sunxi_spi_dump_data(struct device *dev, const u8 *buf, u32 len)
{
	u32 i, cnt = 0;
	u8 *tmp;

	tmp = kzalloc(len, GFP_KERNEL);
	if (!tmp)
		return;

	for (i = 0; i < len; i++) {
		if (i%HEXADECIMAL == 0)
			cnt += sprintf(tmp + cnt, "0x%08x: ", i);

		cnt += sprintf(tmp + cnt, "%02x ", buf[i]);

		if ((i%HEXADECIMAL == REG_END) || (i == (len - 1))) {
			dev_info(dev, "%s\n", tmp);
			cnt = 0;
		}
	}

	kfree(tmp);
}

#endif /* _SUNXI_SPI_DEBUG_H_ */
