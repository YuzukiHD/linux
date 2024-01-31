/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner SoCs ite driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include "iTE6805_Global.h"

int itedbg_init(void);
int itedbg_exit(void);

extern iTE_u8 vendor_id[4];
extern _iTE6805_DATA iTE6805_DATA;
extern iTE_u8 Customer_EDID[EDID_COUNT][256];
extern _iTE6805_VTiming iTE6805_CurVTiming;
