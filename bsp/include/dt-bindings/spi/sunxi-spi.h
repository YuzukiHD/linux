/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Author: jingyanliang@allwinner.com
 */

#ifndef __DT_SUNXI_SPI_H
#define __DT_SUNXI_SPI_H

#include <vdso/bits.h>

#define SUNXI_SPI_BUS_MASTER	BIT(0)
#define SUNXI_SPI_BUS_SLAVE		BIT(1)
#define SUNXI_SPI_BUS_DBI		BIT(2)
#define SUNXI_SPI_BUS_BIT		BIT(3)
#define SUNXI_SPI_BUS_NOR		BIT(4)
#define SUNXI_SPI_BUS_NAND		BIT(5)

#define SUNXI_SPI_CS_AUTO	0
#define SUNXI_SPI_CS_SOFT	1

#endif /* __DT_SUNXI_SPI_H */
