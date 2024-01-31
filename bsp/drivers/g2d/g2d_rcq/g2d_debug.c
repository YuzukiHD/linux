/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Copyright (c) 2007-2019 Allwinnertech Co., Ltd.
 * Author: zhengxiaobin <zhengxiaobin@allwinnertech.com>
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "g2d_debug.h"

void print_image(g2d_image_enh *p_image)
{
	printk("image.format          :%d\n", p_image->format);
	printk("image.bbuff           :%d\n", p_image->bbuff);
	printk("image.color           :0x%x\n", p_image->color);
	printk("image.mode            :%d\n", p_image->mode);
	printk("image.width           :%d\n", p_image->width);
	printk("image.height          :%d\n", p_image->height);
	printk("image.clip_rect.x     :%d\n", p_image->clip_rect.x);
	printk("image.clip_rect.y     :%d\n", p_image->clip_rect.y);
	printk("image.clip_rect.w     :%d\n", p_image->clip_rect.w);
	printk("image.clip_rect.h     :%d\n", p_image->clip_rect.h);
	printk("image.coor.x          :%d\n", p_image->coor.x);
	printk("image.coor.y          :%d\n", p_image->coor.y);
	printk("image.resize.w        :%d\n", p_image->resize.w);
	printk("image.resize.h        :%d\n", p_image->resize.h);
	printk("image.fd              :%d\n", p_image->fd);
	printk("image.alpha           :%d\n", p_image->alpha);

}
