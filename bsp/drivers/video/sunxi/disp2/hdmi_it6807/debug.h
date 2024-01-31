/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Copyright (C) 2009-2019
 *  ITE Tech. Inc. All Rights Reserved
 *  Proprietary and Confidential
 *
 *   @file   <debug.h>
 *   @author Kuro.Chung@ite.com.tw
 *   @date   2019/09/30
 *   @fileversion: iTE6807_MCUSRC_1.07
 */

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/cdev.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/log2.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>

#ifndef Debug_message
#define Debug_message 1
#endif

#if Debug_message

#ifdef CONFIG_IT6807_HDMI_VBYONE_DEBUG
extern u32 print_level;
#define MHLRX_DEBUG_PRINTF(fmt, ...)                                          \
	do {                                                                      \
		if (print_level & (0x1 << 0))                                         \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define EQ_DEBUG_PRINTF(fmt, ...)                                             \
	do {                                                                      \
		if (print_level & (0x1 << 1))                                         \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define VIDEOTIMNG_DEBUG_PRINTF(fmt, ...)                                     \
	do {                                                                      \
		if (print_level & (0x1 << 2))                                         \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define iTE6805_DEBUG_INT_PRINTF(fmt, ...)                                    \
	do {                                                                      \
		if (print_level & (0x1 << 3))                                         \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define HDMIRX_VIDEO_PRINTF(fmt, ...)                                         \
	do {                                                                      \
		if (print_level & (0x1 << 4))                                         \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define HDMIRX_AUDIO_PRINTF(fmt, ...)                                         \
	do {                                                                      \
		if (print_level & (0x1 << 5))                                         \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define HDMIRX_DEBUG_PRINT(fmt, ...)                                          \
	do {                                                                      \
		if (print_level & (0x1 << 6))                                         \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define CEC_DEBUG_PRINTF(fmt, ...)                                            \
	do {                                                                      \
		if (print_level & (0x1 << 7))                                         \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define EDID_DEBUG_PRINTF(fmt, ...)                                           \
	do {                                                                      \
		if (print_level & (0x1 << 8))                                         \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define RCP_DEBUG_PRINTF(fmt, ...)                                            \
	do {                                                                      \
		if (print_level & (0x1 << 9))                                         \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define MHL3D_DEBUG_PRINTF(fmt, ...)                                          \
	do {                                                                      \
		if (print_level & (0x1 << 10))                                        \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define MHL_MSC_DEBUG_PRINTF(fmt, ...)                                        \
	do {                                                                      \
		if (print_level & (0x1 << 11))                                        \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define HDCP_DEBUG_PRINTF(fmt, ...)                                           \
	do {                                                                      \
		if (print_level & (0x1 << 12))                                        \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define HDCP_DEBUG_PRINTF1(fmt, ...)                                          \
	do {                                                                      \
		if (print_level & (0x1 << 13))                                        \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define HDCP_DEBUG_PRINTF2(fmt, ...)                                          \
	do {                                                                      \
		if (print_level & (0x1 << 14))                                        \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define REG_PRINTF(fmt, ...)                                                  \
	do {                                                                      \
		if (print_level & (0x1 << 15))                                        \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define REG_MHL_PRINTF(fmt, ...)                                              \
	do {                                                                      \
		if (print_level & (0x1 << 16))                                        \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define SPI_PRINTF(fmt, ...)                                                  \
	do {                                                                      \
		if (print_level & (0x1 << 17))                                        \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define printinfo(fmt, ...)                                                   \
	do {                                                                      \
		if (print_level & (0x1 << 18))                                        \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#define printinfo_s(fmt, ...)                                                 \
	do {                                                                      \
		if (print_level & (0x1 << 19))                                        \
			pr_info("[it6807]:%s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)
#else
#define MHLRX_DEBUG_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define EQ_DEBUG_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define VIDEOTIMNG_DEBUG_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define iTE6805_DEBUG_INT_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define HDMIRX_VIDEO_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define HDMIRX_AUDIO_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define HDMIRX_DEBUG_PRINT(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define CEC_DEBUG_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define EDID_DEBUG_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define RCP_DEBUG_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define MHL3D_DEBUG_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define MHL_MSC_DEBUG_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define HDCP_DEBUG_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define HDCP_DEBUG_PRINTF1(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define HDCP_DEBUG_PRINTF2(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define REG_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define REG_MHL_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define SPI_PRINTF(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#define printinfo(fmt, ...) \
	pr_info("%s-<%d>:" fmt, __func__, __LINE__, ##__VA_ARGS__);
#define printinfo_s(fmt, ...) pr_info(fmt, ##__VA_ARGS__);
#endif
#else
#define MHLRX_DEBUG_PRINTF(fmt, ...)
#define EQ_DEBUG_PRINTF(fmt, ...)
#define VIDEOTIMNG_DEBUG_PRINTF(fmt, ...)
#define iTE6805_DEBUG_INT_PRINTF(fmt, ...)
#define HDMIRX_VIDEO_PRINTF(fmt, ...)
#define HDMIRX_AUDIO_PRINTF(fmt, ...)
#define HDMIRX_DEBUG_PRINT(fmt, ...)
#define CEC_DEBUG_PRINTF(fmt, ...)
#define EDID_DEBUG_PRINTF(fmt, ...)
#define IT680X_DEBUG_PRINTF(fmt, ...)
#define RCP_DEBUG_PRINTF(fmt, ...)
#define MHL3D_DEBUG_PRINTF(fmt, ...)
#define MHL_MSC_DEBUG_PRINTF(fmt, ...)
#define REG_PRINTF(fmt, ...)
#define SPI_PRINTF(fmt, ...)
#define printinfo(fmt, ...)
#define printinfo_s(fmt, ...)
#endif

#endif
