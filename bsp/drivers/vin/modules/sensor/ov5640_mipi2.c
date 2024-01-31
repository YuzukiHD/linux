/*
 * A V4L2 driver for rn6854m cameras and AHD Coax protocol.
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Chen Weihong <chenweihong@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>
#include "camera.h"
#include "sensor_helper.h"


MODULE_AUTHOR("cwh");
MODULE_DESCRIPTION("A low-level driver for ov5640 mipi chip for cvbs sensor");
MODULE_LICENSE("GPL");

/*define module timing*/
#define MCLK				(24*1000*1000)
#define VREF_POL			V4L2_MBUS_VSYNC_ACTIVE_HIGH
#define HREF_POL			V4L2_MBUS_HSYNC_ACTIVE_HIGH
#define CLK_POL				V4L2_MBUS_PCLK_SAMPLE_RISING

//#define DUMP_SENSOR

#define DRIVER_NAME "ov5640"

/*
    * OV5640 register definitions
	 */
#define REG_SYSTEM_CTROL0			0x3008
#define REG_CHIP_ID_HIGH		0x300a
#define REG_CHIP_ID_LOW			0x300b

#define REG_WINDOW_START_X_HIGH		0x3800
#define REG_WINDOW_START_X_LOW		0x3801
#define REG_WINDOW_START_Y_HIGH		0x3802
#define REG_WINDOW_START_Y_LOW		0x3803
#define REG_WINDOW_WIDTH_HIGH		0x3804
#define REG_WINDOW_WIDTH_LOW		0x3805
#define REG_WINDOW_HEIGHT_HIGH		0x3806
#define REG_WINDOW_HEIGHT_LOW		0x3807
#define REG_OUT_WIDTH_HIGH		0x3808
#define REG_OUT_WIDTH_LOW		0x3809
#define REG_OUT_HEIGHT_HIGH		0x380a
#define REG_OUT_HEIGHT_LOW		0x380b
#define REG_OUT_TOTAL_WIDTH_HIGH	0x380c
#define REG_OUT_TOTAL_WIDTH_LOW		0x380d
#define REG_OUT_TOTAL_HEIGHT_HIGH	0x380e
#define REG_OUT_TOTAL_HEIGHT_LOW	0x380f
#define REG_OUTPUT_FORMAT		0x4300
#define REG_ISP_CTRL_01			0x5001
#define REG_AVG_WINDOW_END_X_HIGH	0x5682
#define REG_AVG_WINDOW_END_X_LOW	0x5683
#define REG_AVG_WINDOW_END_Y_HIGH	0x5686
#define REG_AVG_WINDOW_END_Y_LOW	0x5687

/* active pixel array size */
#define OV5640_SENSOR_SIZE_X		2592
#define OV5640_SENSOR_SIZE_Y		1944

#define REG_NULL			0x0000	/* Array end token */

#define OV5640_SOFT_RESET			(1 << 7)

#define OV564X_ID(_msb, _lsb)		((_msb) << 8 | (_lsb))
#define OV5640_ID			0x5640

#define SENSOR_FRAME_RATE	30

#define I2C_ADDR  0x78

/*static struct delayed_work sensor_s_ae_ratio_work;*/
static bool restart;
#define SENSOR_NAME "ov5640_mipi2"

struct v4l2_subdev *g_ov5640_sd2;

/*
 * The default register settings
 *
 */
static struct regval_list sensor_default_regs[] = {

};

static struct regval_list ov5640_init_regs_5640_mipi[] = {
	{0x3103, 0x11}, {0x3008, 0x82}, {REG_DLY, 10}, {0x3008, 0x42},
	{0x3103, 0x03}, {0x3017, 0x00}, {0x3018, 0x00},
	{0x3034, 0x18}, {0x3035, 0x14}, {0x3036, 0x38},
	{0x3037, 0x13}, {0x3108, 0x01}, {0x3630, 0x36},
	{0x3631, 0x0e}, {0x3632, 0xe2}, {0x3633, 0x12},
	{0x3621, 0xe0}, {0x3704, 0xa0}, {0x3703, 0x5a},
	{0x3715, 0x78}, {0x3717, 0x01}, {0x370b, 0x60},
	{0x3705, 0x1a}, {0x3905, 0x02}, {0x3906, 0x10},
	{0x3901, 0x0a}, {0x3731, 0x12}, {0x3600, 0x08},
	{0x3601, 0x33}, {0x302d, 0x60}, {0x3620, 0x52},
	{0x371b, 0x20}, {0x471c, 0x50}, {0x3a13, 0x43},
	{0x3a18, 0x00}, {0x3a19, 0xf8}, {0x3635, 0x13},
	{0x3636, 0x03}, {0x3634, 0x40}, {0x3622, 0x01},
	{0x3c01, 0xa4}, {0x3c04, 0x28}, {0x3c05, 0x98},
	{0x3c06, 0x00}, {0x3c07, 0x08}, {0x3c08, 0x00},
	{0x3c09, 0x1c}, {0x3c0a, 0x9c}, {0x3c0b, 0x40},
	{0x3820, 0x41}, {0x3821, 0x07}, {0x3814, 0x31},
	{0x3815, 0x31}, {0x3800, 0x00}, {0x3801, 0x00},
	{0x3802, 0x00}, {0x3803, 0x04}, {0x3804, 0x0a},
	{0x3805, 0x3f}, {0x3806, 0x07}, {0x3807, 0x9b},
	{0x3808, 0x02}, {0x3809, 0x80}, {0x380a, 0x01},
	{0x380b, 0xe0}, {0x380c, 0x07}, {0x380d, 0x68},
	{0x380e, 0x03}, {0x380f, 0xd8}, {0x3810, 0x00},
	{0x3811, 0x10}, {0x3812, 0x00}, {0x3813, 0x06},
	{0x3618, 0x00}, {0x3612, 0x29}, {0x3708, 0x64},
	{0x3709, 0x52}, {0x370c, 0x03}, {0x3a02, 0x03},
	{0x3a03, 0xd8}, {0x3a08, 0x01}, {0x3a09, 0x27},
	{0x3a0a, 0x00}, {0x3a0b, 0xf6}, {0x3a0e, 0x03},
	{0x3a0d, 0x04}, {0x3a14, 0x03}, {0x3a15, 0xd8},
	{0x4001, 0x02}, {0x4004, 0x02}, {0x3000, 0x00},
	{0x3002, 0x1c}, {0x3004, 0xff}, {0x3006, 0xc3},
	{0x300e, 0x45}, {0x302e, 0x08}, {0x4300, 0x30},
	{0x501f, 0x00}, {0x4713, 0x03}, {0x4407, 0x04},
	{0x440e, 0x00}, {0x460b, 0x35}, {0x460c, 0x22},
	{0x4837, 0x0a}, {0x4800, 0x04}, {0x3824, 0x02},
	{0x5000, 0xa7}, {0x5001, 0xa3}, {0x5180, 0xff},
	{0x5181, 0xf2}, {0x5182, 0x00}, {0x5183, 0x14},
	{0x5184, 0x25}, {0x5185, 0x24}, {0x5186, 0x09},
	{0x5187, 0x09}, {0x5188, 0x09}, {0x5189, 0x88},
	{0x518a, 0x54}, {0x518b, 0xee}, {0x518c, 0xb2},
	{0x518d, 0x50}, {0x518e, 0x34}, {0x518f, 0x6b},
	{0x5190, 0x46}, {0x5191, 0xf8}, {0x5192, 0x04},
	{0x5193, 0x70}, {0x5194, 0xf0}, {0x5195, 0xf0},
	{0x5196, 0x03}, {0x5197, 0x01}, {0x5198, 0x04},
	{0x5199, 0x6c}, {0x519a, 0x04}, {0x519b, 0x00},
	{0x519c, 0x09}, {0x519d, 0x2b}, {0x519e, 0x38},
	{0x5381, 0x1e}, {0x5382, 0x5b}, {0x5383, 0x08},
	{0x5384, 0x0a}, {0x5385, 0x7e}, {0x5386, 0x88},
	{0x5387, 0x7c}, {0x5388, 0x6c}, {0x5389, 0x10},
	{0x538a, 0x01}, {0x538b, 0x98}, {0x5300, 0x08},
	{0x5301, 0x30}, {0x5302, 0x10}, {0x5303, 0x00},
	{0x5304, 0x08}, {0x5305, 0x30}, {0x5306, 0x08},
	{0x5307, 0x16}, {0x5309, 0x08}, {0x530a, 0x30},
	{0x530b, 0x04}, {0x530c, 0x06}, {0x5480, 0x01},
	{0x5481, 0x08}, {0x5482, 0x14}, {0x5483, 0x28},
	{0x5484, 0x51}, {0x5485, 0x65}, {0x5486, 0x71},
	{0x5487, 0x7d}, {0x5488, 0x87}, {0x5489, 0x91},
	{0x548a, 0x9a}, {0x548b, 0xaa}, {0x548c, 0xb8},
	{0x548d, 0xcd}, {0x548e, 0xdd}, {0x548f, 0xea},
	{0x5490, 0x1d}, {0x5580, 0x02}, {0x5583, 0x40},
	{0x5584, 0x10}, {0x5589, 0x10}, {0x558a, 0x00},
	{0x558b, 0xf8}, {0x5800, 0x23}, {0x5801, 0x14},
	{0x5802, 0x0f}, {0x5803, 0x0f}, {0x5804, 0x12},
	{0x5805, 0x26}, {0x5806, 0x0c}, {0x5807, 0x08},
	{0x5808, 0x05}, {0x5809, 0x05}, {0x580a, 0x08},
	{0x580b, 0x0d}, {0x580c, 0x08}, {0x580d, 0x03},
	{0x580e, 0x00}, {0x580f, 0x00}, {0x5810, 0x03},
	{0x5811, 0x09}, {0x5812, 0x07}, {0x5813, 0x03},
	{0x5814, 0x00}, {0x5815, 0x01}, {0x5816, 0x03},
	{0x5817, 0x08}, {0x5818, 0x0d}, {0x5819, 0x08},
	{0x581a, 0x05}, {0x581b, 0x06}, {0x581c, 0x08},
	{0x581d, 0x0e}, {0x581e, 0x29}, {0x581f, 0x17},
	{0x5820, 0x11}, {0x5821, 0x11}, {0x5822, 0x15},
	{0x5823, 0x28}, {0x5824, 0x46}, {0x5825, 0x26},
	{0x5826, 0x08}, {0x5827, 0x26}, {0x5828, 0x64},
	{0x5829, 0x26}, {0x582a, 0x24}, {0x582b, 0x22},
	{0x582c, 0x24}, {0x582d, 0x24}, {0x582e, 0x06},
	{0x582f, 0x22}, {0x5830, 0x40}, {0x5831, 0x42},
	{0x5832, 0x24}, {0x5833, 0x26}, {0x5834, 0x24},
	{0x5835, 0x22}, {0x5836, 0x22}, {0x5837, 0x26},
	{0x5838, 0x44}, {0x5839, 0x24}, {0x583a, 0x26},
	{0x583b, 0x28}, {0x583c, 0x42}, {0x583d, 0xce},
	{0x5025, 0x00}, {0x3a0f, 0x30}, {0x3a10, 0x28},
	{0x3a1b, 0x30}, {0x3a1e, 0x26}, {0x3a11, 0x60},
	{0x3a1f, 0x14}, {0x3008, 0x02}, {0x3c00, 0x04},
};

static struct regval_list ov5640_2592_1944[] = {
	{0x3008, 0x42},
	{0x3035, 0x21}, {0x3036, 0x54}, {0x3c07, 0x08},
	{0x3c09, 0x1c}, {0x3c0a, 0x9c}, {0x3c0b, 0x40},
	{0x3820, 0x40}, {0x3821, 0x06}, {0x3814, 0x11},
	{0x3815, 0x11}, {0x3800, 0x00}, {0x3801, 0x00},
	{0x3802, 0x00}, {0x3803, 0x00}, {0x3804, 0x0a},
	{0x3805, 0x3f}, {0x3806, 0x07}, {0x3807, 0x9f},
	{0x3808, 0x0a}, {0x3809, 0x20}, {0x380a, 0x07},
	{0x380b, 0x98}, {0x380c, 0x0b}, {0x380d, 0x1c},
	{0x380e, 0x07}, {0x380f, 0xb0}, {0x3810, 0x00},
	{0x3811, 0x10}, {0x3812, 0x00}, {0x3813, 0x04},
	{0x3618, 0x04}, {0x3612, 0x29}, {0x3708, 0x21},
	{0x3709, 0x12}, {0x370c, 0x00}, {0x3a02, 0x03},
	{0x3a03, 0xd8}, {0x3a08, 0x01}, {0x3a09, 0x27},
	{0x3a0a, 0x00}, {0x3a0b, 0xf6}, {0x3a0e, 0x03},
	{0x3a0d, 0x04}, {0x3a14, 0x03}, {0x3a15, 0xd8},
	{0x4001, 0x02}, {0x4004, 0x06}, {0x4713, 0x03},
	{0x4407, 0x04}, {0x460b, 0x35}, {0x460c, 0x22},
	{0x3824, 0x02}, {0x5001, 0x83}, {REG_DLY, 70},
	{0x3008, 0x02}, {0x3503, 0x00},	
};

static struct regval_list ov5640_1080p[] = {
	{0x3008, 0x42},
	{0x3035, 0x21}, {0x3036, 0x54}, {0x3c07, 0x08},
	{0x3c09, 0x1c}, {0x3c0a, 0x9c}, {0x3c0b, 0x40},
	{0x3820, 0x40}, {0x3821, 0x06}, {0x3814, 0x11},
	{0x3815, 0x11}, {0x3800, 0x00}, {0x3801, 0x00},
	{0x3802, 0x00}, {0x3803, 0x00}, {0x3804, 0x0a},
	{0x3805, 0x3f}, {0x3806, 0x07}, {0x3807, 0x9f},
	{0x3808, 0x0a}, {0x3809, 0x20}, {0x380a, 0x07},
	{0x380b, 0x98}, {0x380c, 0x0b}, {0x380d, 0x1c},
	{0x380e, 0x07}, {0x380f, 0xb0}, {0x3810, 0x00},
	{0x3811, 0x10}, {0x3812, 0x00}, {0x3813, 0x04},
	{0x3618, 0x04}, {0x3612, 0x29}, {0x3708, 0x21},
	{0x3709, 0x12}, {0x370c, 0x00}, {0x3a02, 0x03},
	{0x3a03, 0xd8}, {0x3a08, 0x01}, {0x3a09, 0x27},
	{0x3a0a, 0x00}, {0x3a0b, 0xf6}, {0x3a0e, 0x03},
	{0x3a0d, 0x04}, {0x3a14, 0x03}, {0x3a15, 0xd8},
	{0x4001, 0x02}, {0x4004, 0x06}, {0x4713, 0x03},
	{0x4407, 0x04}, {0x460b, 0x35}, {0x460c, 0x22},
	{0x3824, 0x02}, {0x5001, 0x83}, {0x3035, 0x21},
	{0x3036, 0x54}, {0x3c07, 0x07}, {0x3c08, 0x00},
	{0x3c09, 0x1c}, {0x3c0a, 0x9c}, {0x3c0b, 0x40},
	{0x3800, 0x01}, {0x3801, 0x50}, {0x3802, 0x01},
	{0x3803, 0xb2}, {0x3804, 0x08}, {0x3805, 0xef},
	{0x3806, 0x05}, {0x3807, 0xf1}, {0x3808, 0x07},
	{0x3809, 0x80}, {0x380a, 0x04}, {0x380b, 0x38},
	{0x380c, 0x09}, {0x380d, 0xc4}, {0x380e, 0x04},
	{0x380f, 0x60}, {0x3612, 0x2b}, {0x3708, 0x64},
	{0x3a02, 0x04}, {0x3a03, 0x60}, {0x3a08, 0x01},
	{0x3a09, 0x50}, {0x3a0a, 0x01}, {0x3a0b, 0x18},
	{0x3a0e, 0x03}, {0x3a0d, 0x04}, {0x3a14, 0x04},
	{0x3a15, 0x60}, {0x4713, 0x02}, {0x4407, 0x04},
	{0x460b, 0x37}, {0x460c, 0x20}, {0x3824, 0x04},
	{0x4005, 0x1a}, {0x3008, 0x02}, {0x3503, 0,  },
};

static struct regval_list ov5640_720p[] = {
	{0x3008, 0x42},
	{0x3035, 0x21}, {0x3036, 0x54}, {0x3c07, 0x07},
	{0x3c09, 0x1c}, {0x3c0a, 0x9c}, {0x3c0b, 0x40},
	{0x3820, 0x41}, {0x3821, 0x07}, {0x3814, 0x31},
	{0x3815, 0x31}, {0x3800, 0x00}, {0x3801, 0x00},
	{0x3802, 0x00}, {0x3803, 0xfa}, {0x3804, 0x0a},
	{0x3805, 0x3f}, {0x3806, 0x06}, {0x3807, 0xa9},
	{0x3808, 0x05}, {0x3809, 0x00}, {0x380a, 0x02},
	{0x380b, 0xd0}, {0x380c, 0x07}, {0x380d, 0x64},
	{0x380e, 0x02}, {0x380f, 0xe4}, {0x3810, 0x00},
	{0x3811, 0x10}, {0x3812, 0x00}, {0x3813, 0x04},
	{0x3618, 0x00}, {0x3612, 0x29}, {0x3708, 0x64},
	{0x3709, 0x52}, {0x370c, 0x03}, {0x3a02, 0x02},
	{0x3a03, 0xe4}, {0x3a08, 0x01}, {0x3a09, 0xbc},
	{0x3a0a, 0x01}, {0x3a0b, 0x72}, {0x3a0e, 0x01},
	{0x3a0d, 0x02}, {0x3a14, 0x02}, {0x3a15, 0xe4},
	{0x4001, 0x02}, {0x4004, 0x02}, {0x4713, 0x02},
	{0x4407, 0x04}, {0x460b, 0x37}, {0x460c, 0x20},
	{0x3824, 0x04}, {0x5001, 0x83}, {0x4005, 0x1a},
	{0x3008, 0x02}, {0x3503, 0,  },
};

/* 640X480 VGA */
static struct regval_list ov5640_vga[] = {
	{0x3008, 0x42},
	{0x3035, 0x14}, {0x3036, 0x38}, {0x3c07, 0x08},
	{0x3c09, 0x1c}, {0x3c0a, 0x9c}, {0x3c0b, 0x40},
	{0x3820, 0x41}, {0x3821, 0x07}, {0x3814, 0x31},
	{0x3815, 0x31}, {0x3800, 0x00}, {0x3801, 0x00},
	{0x3802, 0x00}, {0x3803, 0x04}, {0x3804, 0x0a},
	{0x3805, 0x3f}, {0x3806, 0x07}, {0x3807, 0x9b},
	{0x3808, 0x02}, {0x3809, 0x80}, {0x380a, 0x01},
	{0x380b, 0xe0}, {0x380c, 0x07}, {0x380d, 0x68},
	{0x380e, 0x04}, {0x380f, 0x38}, {0x3810, 0x00},
	{0x3811, 0x10}, {0x3812, 0x00}, {0x3813, 0x06},
	{0x3618, 0x00}, {0x3612, 0x29}, {0x3708, 0x64},
	{0x3709, 0x52}, {0x370c, 0x03}, {0x3a02, 0x03},
	{0x3a03, 0xd8}, {0x3a08, 0x01}, {0x3a09, 0x0e},
	{0x3a0a, 0x00}, {0x3a0b, 0xf6}, {0x3a0e, 0x03},
	{0x3a0d, 0x04}, {0x3a14, 0x03}, {0x3a15, 0xd8},
	{0x4001, 0x02}, {0x4004, 0x02}, {0x4713, 0x03},
	{0x4407, 0x04}, {0x460b, 0x35}, {0x460c, 0x22},
	{0x3824, 0x02}, {0x5001, 0xa3}, {0x4005, 0x1a},
	{0x3008, 0x02}, {0x3503, 0x00},	
};


static struct regval_list OV5640_EV_M2[]=
{
	{0x3a0f, 0x10},
	{0x3a10, 0x08},
	{0x3a1b, 0x10},
	{0x3a1e, 0x08},
	{0x3a11, 0x20},
	{0x3a1f, 0x10}
};
static struct regval_list OV5640_EV_M1[] =
{
	{0x3a0f, 0x20},
	{0x3a10, 0x18},
	{0x3a11, 0x41},
	{0x3a1b, 0x20},
	{0x3a1e, 0x18},
	{0x3a1f, 0x10}
};

static struct regval_list OV5640_EV_0[] =
{
	{0x3a0f, 0x38},
	{0x3a10, 0x30},
	{0x3a11, 0x61},
	{0x3a1b, 0x38},
	{0x3a1e, 0x30},
	{0x3a1f, 0x10}
};

static struct regval_list OV5640_EV_P1[] =
{
	{0x3a0f, 0x50},
	{0x3a10, 0x48},
	{0x3a11, 0x90},
	{0x3a1b, 0x50},
	{0x3a1e, 0x48},
	{0x3a1f, 0x20}
};
static struct regval_list  OV5640_EV_P2[] =
{
	{0x3a0f, 0x60},
	{0x3a10, 0x58},
	{0x3a11, 0xa0},
	{0x3a1b, 0x60},
	{0x3a1e, 0x58},
	{0x3a1f, 0x20}
};

static  struct regval_list ov5640_reg_stop_stream[] = {
	{0x4202, 0x0f},
};
static  struct regval_list ov5640_reg_start_stream[] = {
	{0x4202, 0x00},
};

static struct regval_list sensor_fmt_yuv422_yuyv[] = {
	{0x4300, 0x30},
};

static struct regval_list sensor_fmt_yuv422_uyvy[] = {
	{0x4300, 0x32},
};

static struct regval_list sensor_fmt_raw[] = {
	{0x4300, 0x30},	// 0x00
};

/* RGB565 */
static struct regval_list sensor_fmt_rgb565[] = {
	{ 0x4300, 0x60 }, /* RGB565 */
};

static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	return 0;
}

static int sensor_power(struct v4l2_subdev *sd, int on)
{
	switch (on) {
		case STBY_ON:
			sensor_s_sw_stby(sd, ON);
			break;
		case STBY_OFF:
			sensor_s_sw_stby(sd, OFF);
			break;
		case PWR_ON:
			cci_lock(sd);
		//	vin_gpio_set_status(sd, RESET, CSI_GPIO_HIGH);
		//	vin_gpio_set_status(sd, PWDN, CSI_GPIO_HIGH);
			//vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
			//vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
			vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
			vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
			usleep_range(1000, 1200);
			vin_set_mclk_freq(sd, MCLK);
			vin_set_mclk(sd, ON);
			usleep_range(10000, 12000);
			vin_gpio_write(sd, POWER_EN, CSI_GPIO_HIGH);
			vin_set_pmu_channel(sd, CAMERAVDD, ON);
			vin_set_pmu_channel(sd, IOVDD, ON);
			vin_set_pmu_channel(sd, DVDD, ON);
			vin_set_pmu_channel(sd, AVDD, ON);
			usleep_range(5000, 6000);
			vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
			usleep_range(20000, 22000);
			vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
			usleep_range(20000, 22000);
			cci_unlock(sd);

			break;
		case PWR_OFF:
			cci_lock(sd);
			vin_set_mclk(sd, OFF);
			//vin_gpio_set_status(sd, RESET, CSI_GPIO_HIGH);
			//vin_gpio_set_status(sd, PWDN, CSI_GPIO_HIGH);
			vin_gpio_write(sd, POWER_EN, CSI_GPIO_LOW);
			//vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
			//vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
			usleep_range(10000, 12000);

			vin_set_pmu_channel(sd, CAMERAVDD, OFF);
			vin_set_pmu_channel(sd, IOVDD, OFF);
			vin_set_pmu_channel(sd, DVDD, OFF);
			vin_set_pmu_channel(sd, AVDD, OFF);
			vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
			vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
			usleep_range(1000, 1200);
			vin_set_mclk(sd, OFF);
			//vin_gpio_set_status(sd, RESET, CSI_GPIO_LOW);
			cci_unlock(sd);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
	usleep_range(5000, 6000);
	vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
	usleep_range(5000, 6000);
	return 0;
}

//int sensor_read(struct v4l2_subdev *sd, data_type reg, data_type *val)
//{
//	return sensor_write(sd, reg, val);
//}
//
//int sensor_write(struct v4l2_subdev *sd, data_type reg, data_type *val)
//{
//	return sensor_read(sd, reg, val);
//}

static int sensor_detect(struct v4l2_subdev *sd)
{
	int ret;
	data_type rdval = 0;
	data_type pid, ver;

	ret = sensor_read(sd, REG_SYSTEM_CTROL0, &rdval);
	if (ret < 0)
		return -ENODEV;
	rdval |= OV5640_SOFT_RESET;
	ret = sensor_write(sd, REG_SYSTEM_CTROL0, rdval);
	if (ret < 0)
		return -ENODEV;
	usleep_range(20000, 22000);
	
	/* SW Powerdown */
	ret = sensor_write(sd, REG_SYSTEM_CTROL0, 0x40);
	if (ret != 0) {
		printk("Sensor soft Power Down failed\n");
		return -ENODEV;
	}
	usleep_range(1000, 2000);
	/* Check sensor revision */
	ret = sensor_read(sd, REG_CHIP_ID_HIGH, &pid);
	printk("ov5640 id_h:%x\n", pid);
	if (!ret)
		ret = sensor_read(sd, REG_CHIP_ID_LOW, &ver);
	if (!ret) {
		unsigned short id;

		id = OV564X_ID(pid, ver);
		printk("ov5640 id:%x\n", id);
		if (id != OV5640_ID) {
			ret = -ENODEV;
		} else {
			sensor_write_array(sd, ov5640_init_regs_5640_mipi, \
					sizeof(ov5640_init_regs_5640_mipi) / sizeof(ov5640_init_regs_5640_mipi[0]));
			msleep(500);
		}
	} else
		return -ENODEV;
	return 0;
}
static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	restart = 0;
	/*Make sure it is a target sensor */
	ret = sensor_detect(sd);
	if (ret) {
		sensor_err("chip found is not an target chip.\n");
		return ret;
	}

	info->focus_status = 0; 
	info->low_speed = 0; 
	info->width = 1280; 
	info->height = 720; 
	info->brightness = 0; 
	info->contrast = 0; 
	info->saturation = 0; 
	info->hue = 0; 
	info->hflip = 0; 
	info->vflip = 0; 
	info->gain = 0; 
	info->autogain = 1; 
	info->exp_bias = 0; 
	info->autoexp = 1; 
	info->autowb = 1; 

	info->wb = V4L2_WHITE_BALANCE_AUTO;
	info->clrfx = V4L2_COLORFX_NONE;
	info->band_filter = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;

	info->tpf.numerator = 1;
	info->tpf.denominator = 30;
	info->preview_first_flag = 1;

	return 0;
}

static int sensor_s_exp_gain(struct v4l2_subdev *sd,
		struct sensor_exp_gain *exp_gain)
{
	struct sensor_info *info = to_state(sd);
	int exp_val, gain_val;

	exp_val = exp_gain->exp_val;
	gain_val = exp_gain->gain_val;

	if (gain_val < 1 * 16)
		gain_val = 16;
	if (gain_val > 64 * 16 - 1)
		gain_val = 64 * 16 - 1;
	if (exp_val > 0xfffff)
		exp_val = 0xfffff;

	info->exp = exp_val;
	info->gain = gain_val;
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	data_type explow, exphigh;
	struct sensor_info *info = to_state(sd);

	if (exp_val > 0xffffff)
		exp_val = 0xfffff0;
	if (exp_val < 16)
		exp_val = 16;

	exp_val = (exp_val + 8) >> 4;   /*rounding to 1*/

	exphigh = (unsigned char)((0xff00 & exp_val) >> 8);
	explow = (unsigned char)((0x00ff & exp_val));
	info->exp = exp_val;
	return 0;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct sensor_info *info = to_state(sd);
	switch (cmd) {
	case GET_CURRENT_WIN_CFG:
		if (info->current_wins != NULL) {
			memcpy(arg, info->current_wins,
			       sizeof(struct sensor_win_size));
			ret = 0;
		} else {
			sensor_err("empty wins!\n");
			ret = -1;
		}
		break;
	case SET_FPS:
		break;
	case VIDIOC_VIN_SENSOR_EXP_GAIN:
		sensor_s_exp_gain(sd, (struct sensor_exp_gain *)arg);
		break;
	case VIDIOC_VIN_SENSOR_CFG_REQ:
		sensor_cfg_req(sd, (struct sensor_config *)arg);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static struct sensor_format_struct sensor_formats[] = {
	{
		.desc = "YUYV 4:2:2",
		.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.regs = sensor_fmt_yuv422_yuyv,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yuyv),
		.bpp = 2,
	}, {
		.desc = "UYVY 4:2:2",
		.mbus_code = MEDIA_BUS_FMT_UYVY8_2X8,
		.regs = sensor_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_uyvy),
		.bpp = 2,
	}, {
		.desc = "Raw RGB Bayer",
			.mbus_code = MEDIA_BUS_FMT_SBGGR8_1X8,
			.regs = sensor_fmt_raw,
			.regs_size = ARRAY_SIZE(sensor_fmt_raw),
			.bpp = 1,
	}, {
		.desc = "RGB565 Bayer",
			.mbus_code = MEDIA_BUS_FMT_RGB565_2X8_BE,
			.regs = sensor_fmt_rgb565,
			.regs_size = ARRAY_SIZE(sensor_fmt_rgb565),
			.bpp = 2,
	}
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */
static struct sensor_win_size sensor_win_sizes[] = {
	{
		.width = 640,
		.height = 480,
		.hoffset = 0,
		.voffset = 0,
		.fps_fixed = 30,
		.regs = ov5640_vga,
		.regs_size = ARRAY_SIZE(ov5640_vga),
		.set_size = NULL,
	},
	{
		.width = 1280,
		.height = 720,
		.hoffset = 0, 
		.voffset = 0, 
		.fps_fixed = 30, 
		.regs = ov5640_720p,
		.regs_size = ARRAY_SIZE(ov5640_720p),
		.set_size = NULL,
	},
	{
		.width = 1920,
		.height = 1080,
		.hoffset = 0,
		.voffset = 0,
		.fps_fixed = 30, 
		.regs = ov5640_1080p,
		.regs_size = ARRAY_SIZE(ov5640_1080p),
		.set_size = NULL,
	},
	{
		.width = 2592,
		.height = 1944,
		.hoffset = 0,
		.voffset = 0,
		.fps_fixed = 30, 
		.regs = ov5640_2592_1944,
		.regs_size = ARRAY_SIZE(ov5640_2592_1944),
		.set_size = NULL,
	},
};
#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static int sensor_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *cfg)

{
	/*
	   V4L2_MBUS_PARALLEL
	   parallel interface with hsync and vsync
	   V4L2_MBUS_BT656
	   parallel interface with embedded synchronisation, can also be used for BT.1120
	   V4L2_MBUS_CSI2
	   MIPI CSI-2 serial interface
	 */
		cfg->type = V4L2_MBUS_CSI2_DPHY;
//		cfg->flags = 0 | V4L2_MBUS_CSI2_4_LANE | V4L2_MBUS_CSI2_CHANNEL_0 | V4L2_MBUS_CSI2_CHANNEL_1 | V4L2_MBUS_CSI2_CHANNEL_2 | V4L2_MBUS_CSI2_CHANNEL_3;
//		cfg->flags = 0 | V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0;
		cfg->flags = 0 | V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0;

	return 0;
}
#ifdef DUMP_SENSOR
static void sensor_dump_reg(struct v4l2_subdev *sd, struct regval_list *regs, int size ,char* regs_name){
	int i = 0;
	addr_type addr;
	data_type data;
	if(regs != NULL && size > 0){
		printk("%s regs dump size is %d\n",regs_name,size);
		while ( i < size){
			if((i + 1) % 10 == 0){
				printk("\n>> %d <<",i/9);
			}
			addr = regs[i].addr;
			sensor_read(sd,addr,&data);
			i++;
			printk(KERN_CONT"%x  %x || ",addr,data);
		}
	}else{
		printk("regs list is null");
	}
}
#endif

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_info *info = to_state(sd);
	struct v4l2_subdev *ov5640_sd = sd;
	struct sensor_format_struct *sensor_fmt = info->fmt;
	struct sensor_win_size *wsize = info->current_wins;

	if (!enable) {
		return 0;
	}

	sensor_write_array(ov5640_sd, ov5640_reg_stop_stream, \
				sizeof(ov5640_reg_stop_stream) / sizeof(ov5640_reg_stop_stream[0]));

	if (wsize->regs)
		sensor_write_array(ov5640_sd, wsize->regs, wsize->regs_size);

	if (wsize->set_size)
		wsize->set_size(ov5640_sd);

	if (sensor_fmt->regs) 
		sensor_write_array(ov5640_sd, sensor_fmt->regs, sensor_fmt->regs_size);

	//test image
//	sensor_write(ov5640_sd, 0x503D, 0x80); // 彩条
//	sensor_write(ov5640_sd, 0x503D, 0x82); // 彩色四方块
//	sensor_write(ov5640_sd, 0x4741, 0x00);
	

	sensor_write(ov5640_sd, 0x5300, 0x08);
	sensor_write(ov5640_sd, 0x5301, 0x30);
	sensor_write(ov5640_sd, 0x5302, 0x10);
	sensor_write(ov5640_sd, 0x5304, 0x08);
	sensor_write(ov5640_sd, 0x5305, 0x30);
	sensor_write(ov5640_sd, 0x5306, 0x08);
	sensor_write(ov5640_sd, 0x5307, 0x16);
	sensor_write(ov5640_sd, 0x5309, 0x08);
	sensor_write(ov5640_sd, 0x530a, 0x30);
	sensor_write(ov5640_sd, 0x530b, 0x04);
	sensor_write(ov5640_sd, 0x530c, 0x06);

	sensor_write_array(ov5640_sd, ov5640_reg_start_stream, \
				sizeof(ov5640_reg_start_stream) / sizeof(ov5640_reg_start_stream[0]));
#ifdef DUMP_SENSOR
	//dump all reg
	sensor_dump_reg(ov5640_sd,ov5640_reg_start_stream,ARRAY_SIZE(ov5640_reg_start_stream),"ov5640_reg_start_stream");
	sensor_dump_reg(ov5640_sd,ov5640_init_regs_5640_mipi,ARRAY_SIZE(ov5640_init_regs_5640_mipi),"ov5640_init_regs_5640_mipi");
	sensor_dump_reg(ov5640_sd,ov5640_vga,ARRAY_SIZE(ov5640_vga),"ov5640_vga");
	sensor_dump_reg(ov5640_sd,OV5640_EV_M2,ARRAY_SIZE(OV5640_EV_M2),"OV5640_EV_M2");
	sensor_dump_reg(ov5640_sd,OV5640_EV_M1,ARRAY_SIZE(OV5640_EV_M1),"OV5640_EV_M1");
	sensor_dump_reg(ov5640_sd,OV5640_EV_0,ARRAY_SIZE(OV5640_EV_0),"OV5640_EV_0");
	sensor_dump_reg(ov5640_sd,OV5640_EV_P1,ARRAY_SIZE(OV5640_EV_P1),"OV5640_EV_P1");
	sensor_dump_reg(ov5640_sd,OV5640_EV_P2,ARRAY_SIZE(OV5640_EV_P2),"OV5640_EV_P2");
	sensor_dump_reg(ov5640_sd,sensor_fmt_yuv422_yuyv,ARRAY_SIZE(sensor_fmt_yuv422_yuyv),"sensor_fmt");
#endif
	return 0;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	struct sensor_info *info = to_state(sd);
	data_type ana_gain = 0;
	unsigned int digi_gain = 0;

	if (gain_val > 128) {
		ana_gain = 0x4f;
		digi_gain = gain_val << 1;
	} else if (gain_val > 64) {
		ana_gain = 0x4a;
		digi_gain = gain_val << 2;
	} else if (gain_val > 32) {
		ana_gain = 0x45;
		digi_gain = gain_val << 3;
	} else {
		ana_gain = 0x40;
		digi_gain = gain_val << 4;
	}

	info->gain = gain_val;

	return 0;
}


static int sensor_s_band_filter(struct v4l2_subdev *sd,
		enum v4l2_power_line_frequency value)
{
	struct sensor_info *info = to_state(sd);
	data_type rdval;

	if (info->band_filter == value)
		return 0;

	switch (value) {
		case V4L2_CID_POWER_LINE_FREQUENCY_DISABLED:
			sensor_read(sd, 0x3a00, &rdval);
			sensor_write(sd, 0x3a00, rdval & 0xdf);
			break;
		case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
			sensor_write(sd, 0x3c00, 0x04);
			sensor_write(sd, 0x3c01, 0x80);
			sensor_read(sd, 0x3a00, &rdval);
			sensor_write(sd, 0x3a00, rdval | 0x20);
			break;
		case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
			sensor_write(sd, 0x3c00, 0x00);
			sensor_write(sd, 0x3c01, 0x80);
			sensor_read(sd, 0x3a00, &rdval);
			sensor_write(sd, 0x3a00, rdval | 0x20);
			break;
		case V4L2_CID_POWER_LINE_FREQUENCY_AUTO:
			break;
		default:
			break;
	}

	info->band_filter = value;
	return 0;
}

static int sensor_s_continueous_af(struct v4l2_subdev *sd, int value)
{
	struct sensor_info *info = to_state(sd);

	if (info->focus_status == 1) {
		return -1;
	}
	if ((info->auto_focus == value)) {
		return 0;
	}

	if (value == 1) {
		sensor_write(sd, 0x3022, 0x04);
		sensor_write(sd, 0x3022, 0x80);
		info->auto_focus = 1;
	} else {
		sensor_write(sd, 0x3022, 0x06);
		info->auto_focus = 0;
	}
	return 0;
}

static int sensor_s_single_af(struct v4l2_subdev *sd)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	data_type rdval = 0xff;
	unsigned int cnt = 0;

	info->focus_status = 0;

	sensor_write(sd, 0x3023, 0x01);

	ret = sensor_write(sd, 0x3022, 0x03);
	if (ret < 0) {
		sensor_err("sensor tigger single af err !\n");
		return ret;
	}

	while (rdval != 0 && cnt < 10) {
		usleep_range(1000, 1200);
		ret = sensor_read(sd, 0x3023, &rdval);
		cnt++;
	}

	info->focus_status = 1;
	info->auto_focus = 0;
	return 0;
}

static int sensor_s_pause_af(struct v4l2_subdev *sd)
{
	sensor_write(sd, 0x3022, 0x06);
	return 0;
}


static int sensor_s_brightness(struct v4l2_subdev *sd, int val)
{
	struct sensor_info *info = to_state(sd);
	int err;
	switch (val)
	{
		case 5:
			err = sensor_write_array(sd, OV5640_EV_P2,
					ARRAY_SIZE(OV5640_EV_P2));
			if (err){
				printk(" OV5640 brightness OV5640_EV_P2  setting failed ");
			}
			break;
		case 4:
			err = sensor_write_array(sd, OV5640_EV_P1,
					ARRAY_SIZE(OV5640_EV_P1));
			if (err){
				printk(" OV5640 brightness OV5640_EV_P1  setting failed ");
			}
			break;
		case 3:
			err = sensor_write_array(sd, OV5640_EV_0,
					ARRAY_SIZE(OV5640_EV_0));
			if (err){
				printk(" OV5640 brightness OV5640_EV_0  setting failed ");
			}
			break;
		case 2:
			err = sensor_write_array(sd, OV5640_EV_M1,
					ARRAY_SIZE(OV5640_EV_M1));
			if (err){
				printk(" OV5640 brightness OV5640_EV_M1  setting failed ");
			}
			break;
		case 1:
			err = sensor_write_array(sd, OV5640_EV_M2,
					ARRAY_SIZE(OV5640_EV_M2));
			if (err){
				printk(" OV5640 brightness OV5640_EV_M2  setting failed ");
			}
			break;
		case -2:
			sensor_write(sd, 0x3212, 0x03);
			sensor_write(sd, 0x5587, 0x20);
			sensor_write(sd, 0x5588, 0x09);
			sensor_write(sd, 0x3212, 0x13);
			sensor_write(sd, 0x3212, 0xa3);
			break;
		case -4:
			sensor_write(sd, 0x3212, 0x03);
			sensor_write(sd, 0x5587, 0x40);
			sensor_write(sd, 0x5588, 0x09);
			sensor_write(sd, 0x3212, 0x13);
			sensor_write(sd, 0x3212, 0xa3);
			break;
		default:
			err = sensor_write_array(sd, OV5640_EV_0,
					ARRAY_SIZE(OV5640_EV_0));
			if (err){
				printk(" OV5640 brightness OV5640_EV_0  setting failed ");
			}
			break;
	}
	return 0;
}


static int sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
		container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
		case V4L2_CID_TEST_PATTERN:
			break;
		case V4L2_CID_GAIN:
			return sensor_s_gain(sd, ctrl->val);

		case V4L2_CID_EXPOSURE:
			return sensor_s_exp(sd, ctrl->val);
		case V4L2_CID_POWER_LINE_FREQUENCY:
			return sensor_s_band_filter(sd,
					(enum v4l2_power_line_frequency)ctrl->val);
		case V4L2_CID_FOCUS_AUTO:
			return sensor_s_continueous_af(sd, ctrl->val);    
		case V4L2_CID_AUTO_FOCUS_START:
			return sensor_s_single_af(sd);
		case V4L2_CID_AUTO_FOCUS_STOP:
			return sensor_s_pause_af(sd);
		case V4L2_CID_BRIGHTNESS:
			return sensor_s_brightness(sd, ctrl->val);
	}
	return 0;
}

static int sensor_g_band_filter(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	data_type rdval;

	sensor_read(sd, 0x3a00, &rdval);

	if ((rdval & (1 << 5)) == (1 << 5))
		info->band_filter = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;
	else {
		sensor_read(sd, 0x3c00, &rdval);
		if ((rdval & (1 << 2)) == (1 << 2))
			info->band_filter = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
		else
			info->band_filter = V4L2_CID_POWER_LINE_FREQUENCY_60HZ;
	}
	return 0;
}

static int sensor_g_contin_af(struct v4l2_subdev *sd)
{
	data_type rdval;
	struct sensor_info *info = to_state(sd);

	rdval = 0xff;
	sensor_read(sd, 0x3029, &rdval);

	if (rdval == 0x20 || rdval == 0x10) {
		info->focus_status = 0;
		sensor_read(sd, 0x3028, &rdval);
		if (rdval == 0)
			return V4L2_AUTO_FOCUS_STATUS_FAILED;
		else
			return V4L2_AUTO_FOCUS_STATUS_REACHED;
	} else if (rdval == 0x00) {
		info->focus_status = 1;
		return V4L2_AUTO_FOCUS_STATUS_BUSY;
	} else {
		info->focus_status = 0;
		return V4L2_AUTO_FOCUS_STATUS_IDLE;
	}
}

static int sensor_g_single_af(struct v4l2_subdev *sd)
{
	data_type rdval;
	struct sensor_info *info = to_state(sd);

	if (info->focus_status != 1)
		return V4L2_AUTO_FOCUS_STATUS_IDLE;
	rdval = 0xff;
	sensor_read(sd, 0x3029, &rdval);
	if (rdval == 0x10) {
		int ret = 0;

		info->focus_status = 0;
		sensor_read(sd, 0x3028, &rdval);
		if (rdval == 0) {
			printk("Single AF focus fail, 0x3028 = 0x%x\n",
					rdval);
			ret = V4L2_AUTO_FOCUS_STATUS_FAILED;
		} else {
			printk("Single AF focus ok, 0x3028 = 0x%x\n",
					rdval);
			ret = V4L2_AUTO_FOCUS_STATUS_REACHED;
		}
		return ret;
	} else if (rdval == 0x70) {
		info->focus_status = 0;
		return V4L2_AUTO_FOCUS_STATUS_IDLE;
	} else if (rdval == 0x00) {
		info->focus_status = 1;
		return V4L2_AUTO_FOCUS_STATUS_BUSY;
	}
	return V4L2_AUTO_FOCUS_STATUS_BUSY;
}


static int sensor_g_af_status(struct v4l2_subdev *sd)
{
	int ret = 0;
	struct sensor_info *info = to_state(sd);

	if (info->auto_focus == 1)
		ret = sensor_g_contin_af(sd);
	else
		ret = sensor_g_single_af(sd);

	return ret;
}

static int sensor_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
		container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
		case V4L2_CID_GAIN:
			return 0;
		case V4L2_CID_EXPOSURE:
			return 0;
		case V4L2_CID_POWER_LINE_FREQUENCY:
			return sensor_g_band_filter(sd, &ctrl->val);    
		case V4L2_CID_AUTO_FOCUS_STATUS:
			return sensor_g_af_status(sd);
	}
	return -EINVAL;
}


static const struct v4l2_ctrl_ops sensor_ctrl_ops = { 
	.s_ctrl = sensor_s_ctrl,
	.g_volatile_ctrl = sensor_g_ctrl,

};

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.reset = sensor_reset,	//override
	.init = sensor_init,	//override
	.s_power = sensor_power,	// override
	.ioctl = sensor_ioctl,		//override
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sensor_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
//	.s_parm = sensor_s_parm,
//	.g_parm = sensor_g_parm,
	.s_stream = sensor_s_stream,	//override
//	.g_mbus_config = sensor_g_mbus_config,	// override
//	.get_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_mbus_code,
	.enum_frame_size = sensor_enum_frame_size,
	.get_fmt = sensor_get_fmt,
	.set_fmt = sensor_set_fmt,
	.get_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
	.pad = &sensor_pad_ops,
};

static const char * const ov5640_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bars",
};


/* ----------------------------------------------------------------------- */
static struct cci_driver cci_drv = {
	.name = SENSOR_NAME,
	.addr_width = CCI_BITS_16,		//芯片寄存器地址位数
	.data_width = CCI_BITS_8,		//芯片寄存器数据位数
};


static int sensor_init_controls(struct v4l2_subdev *sd, const struct v4l2_ctrl_ops *ops)
{
	struct sensor_info *info = to_state(sd);
	struct v4l2_ctrl_handler *handler = &info->handler;
	struct v4l2_ctrl *ctrl;
	int ret = 0;

	v4l2_ctrl_handler_init(handler, 8);

	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_PIXEL_RATE,
			0,
			0xFFFFFFFF, 1,
			0x2BF2000);

	v4l2_ctrl_new_std_menu_items(handler, ops,
			V4L2_CID_TEST_PATTERN,
			ARRAY_SIZE(ov5640_test_pattern_menu) - 1,
			0, 0, ov5640_test_pattern_menu);

	v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN, 1 * 1600,
			256 * 1600, 1, 1 * 1600);
	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_EXPOSURE, 0,
			65536 * 16, 1, 0);

	v4l2_ctrl_new_std(handler, ops, V4L2_CID_AUTO_FOCUS_START, 0, 0, 0, 0);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_AUTO_FOCUS_STOP, 0, 0, 0, 0);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_AUTO_FOCUS_STATUS, 0, 7, 0, 0);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_FOCUS_AUTO, 0, 1, 1, 1);
	

	if (ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
	}    

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
	}
	sd->ctrl_handler = handler;

	return ret;
}

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	printk("alex hu ov5640 ----000---\n");

	sd = &info->sd;
	cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv);
	sensor_init_controls(sd, &sensor_ctrl_ops);

	g_ov5640_sd2 = sd;

	mutex_init(&info->lock);
//	info->sensor_i2c_addr = I2C_ADDR >> 1;
	restart = 0;

	info->fmt = &sensor_formats[0];
	info->fmt_pt = &sensor_formats[0];
	info->win_pt = &sensor_win_sizes[0];
	info->fmt_num = N_FMTS;
	info->win_size_num = N_WIN_SIZES;
	info->sensor_field = V4L2_FIELD_NONE;
	/* info->combo_mode = CMB_TERMINAL_RES | CMB_PHYA_OFFSET1 | MIPI_NORMAL_MODE; */
//	info->combo_mode = CMB_PHYA_OFFSET1 | MIPI_NORMAL_MODE;
	info->combo_mode = CMB_TERMINAL_RES | CMB_PHYA_OFFSET1 | MIPI_NORMAL_MODE;
	info->stream_seq = MIPI_BEFORE_SENSOR;
	info->af_first_flag = 1;
	info->exp = 0;
	info->gain = 0;
	printk("alex hu ov5640 ----111---\n");

	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	sd = cci_dev_remove_helper(client, &cci_drv);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME, 0},
	{"ov5640_mipi_cam2",0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = SENSOR_NAME,
		   },
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static __init int init_sensor(void)
{
	return cci_dev_init_helper(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	cci_dev_exit_helper(&sensor_driver);
}

//#ifdef CONFIG_VIDEO_SUNXI_VIN_SPECIAL
//subsys_initcall_sync(init_sensor);
//#else
//module_init(init_sensor);
//#endif
//module_exit(exit_sensor);


VIN_INIT_DRIVERS(init_sensor);
module_exit(exit_sensor);

