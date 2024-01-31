// SPDX-License-Identifier: GPL-2.0-or-later
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

#include "dev_ite_debugfs.h"

u32 print_level;

static struct dentry *my_itedbg_root;

/* ##########vendor_id############### */
static int itedbg_vendor_id_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int itedbg_vendor_id_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t itedbg_vendor_id_read(struct file *file, char __user *buf,
									 size_t count, loff_t *ppos)
{
	size_t buf_cnt = 0;
	char *temp_buf = NULL;

	temp_buf = kmalloc(256, GFP_KERNEL | __GFP_ZERO);
	if (!temp_buf) {
		pr_warn("Malloc buf fail!\n");
		return count;
	}
	buf_cnt += sprintf(
		temp_buf + buf_cnt,
		"VendorID:%02x %02x\nDeviceID:%02x %02x\nDevice Revision:%02x\n",
		vendor_id[0], vendor_id[1], vendor_id[2], vendor_id[3],
		iTE6805_DATA.ChipID);

	buf_cnt = simple_read_from_buffer(buf, count, ppos, temp_buf, buf_cnt);
	kfree(temp_buf);
	return buf_cnt;
}

static ssize_t itedbg_vendor_id_write(struct file *file, const char __user *buf,
									  size_t count, loff_t *ppos)
{
	return count;
}

static int itedbg_edid_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int itedbg_edid_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t itedbg_edid_read(struct file *file, char __user *buf,
								size_t count, loff_t *ppos)
{
	size_t buf_cnt = 0;
	char *temp_buf = NULL;
	iTE_u8 *piTE6805EDID;
	iTE_u8 *pEDID;
	iTE_u8 i, offset, sum = 0;
	iTE_u8 BlockNUM = 0;

	temp_buf = kmalloc(2048, GFP_KERNEL | __GFP_ZERO);
	if (!temp_buf) {
		pr_warn("Malloc buf fail!\n");
		return count;
	}

	piTE6805EDID =
		(iTE_u8 *)&Customer_EDID[iTE6805_DATA
									 .US_EDID_INDEX[iTE6805_DATA.CurrentPort]]
								[0]; /* Always 0 , define only one EDID */
									 /* BLOCK0 */
	pEDID = piTE6805EDID;
	if (BlockNUM == 0x02)
		offset = 0x00 + 128 * 0x01;
	else
		offset = 0x00 + 128 * BlockNUM;

	buf_cnt += sprintf(temp_buf + buf_cnt, "block No =%02X offset = %02X\n",
					   (int)BlockNUM, (int)offset);

	for (i = 0; i < 0x7F; i++) {
		buf_cnt += sprintf(temp_buf + buf_cnt, "%02x ", (int)*(pEDID + offset));
		sum += *(pEDID + offset);
		offset++;
		if ((i % 16) == 15) {
			buf_cnt += sprintf(temp_buf + buf_cnt, "\n");
		}
	}
	sum = 0x00 - sum;
	buf_cnt += sprintf(temp_buf + buf_cnt, "\nBlock0_CheckSum=%02x\n\n", sum);
	/* BLOCK1 */
	pEDID = piTE6805EDID;
	BlockNUM = 1;
	if (BlockNUM == 0x02)
		offset = 0x00 + 128 * 0x01;
	else
		offset = 0x00 + 128 * BlockNUM;

	buf_cnt += sprintf(temp_buf + buf_cnt, "block No =%02X offset = %02X\n",
					   (int)BlockNUM, (int)offset);

	for (i = 0; i < 0x7F; i++) {
		buf_cnt += sprintf(temp_buf + buf_cnt, "%02x ", (int)*(pEDID + offset));
		sum += *(pEDID + offset);
		offset++;
		if ((i % 16) == 15) {
			buf_cnt += sprintf(temp_buf + buf_cnt, "\n");
		}
	}
	sum = 0x00 - sum;
	buf_cnt += sprintf(temp_buf + buf_cnt, "\nBlock0_CheckSum=%02x\n", sum);

	buf_cnt = simple_read_from_buffer(buf, count, ppos, temp_buf, buf_cnt);
	kfree(temp_buf);
	return buf_cnt;
}

static ssize_t itedbg_edid_write(struct file *file, const char __user *buf,
								 size_t count, loff_t *ppos)
{
	return count;
}

static int itedbg_vid_info_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int itedbg_vid_info_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t itedbg_vid_info_read(struct file *file, char __user *buf,
									size_t count, loff_t *ppos)
{
	size_t buf_cnt = 0;
	char *temp_buf = NULL;
	int voutstb, lvpclk_hs;

	temp_buf = kmalloc(4096, GFP_KERNEL | __GFP_ZERO);
	if (!temp_buf) {
		pr_warn("Malloc buf fail!\n");
		return count;
	}

	chgbank(0);
	buf_cnt += sprintf(temp_buf + buf_cnt, "\n -------Video Timing-------\n");

	if (iTE6805_DATA.CurrentPort == PORT0) {
		if (hdmirxrd(0x15) & 0x02) {
			buf_cnt +=
				sprintf(temp_buf + buf_cnt, "HDMI2 Scramble Enable !! \n");
		} else {
			buf_cnt +=
				sprintf(temp_buf + buf_cnt, "HDMI2 Scramble Disable !! \n");
		}

		if (hdmirxrd(0x14) & 0x40) {
			buf_cnt +=
				sprintf(temp_buf + buf_cnt, "HDMI2 CLK Ratio 1/40 !! \n");
		} else {
			buf_cnt +=
				sprintf(temp_buf + buf_cnt, "HDMI CLK Ratio 1/10  !! \n");
		}
	} else {
		if (hdmirxrd(0x18) & 0x02) {
			buf_cnt +=
				sprintf(temp_buf + buf_cnt, "HDMI2 Scramble Enable !! \n");
		} else {
			buf_cnt +=
				sprintf(temp_buf + buf_cnt, "HDMI2 Scramble Disable !! \n");
		}

		if (hdmirxrd(0x17) & 0x40) {
			buf_cnt +=
				sprintf(temp_buf + buf_cnt, "HDMI2 CLK Ratio 1/40 !! \n");
		} else {
			buf_cnt +=
				sprintf(temp_buf + buf_cnt, "HDMI CLK Ratio 1/10  !! \n");
		}
	}

	voutstb = ((hdmirxrd(0x19) & 0x02) >> 1);
	lvpclk_hs = ((hdmirxrd(0x36) & 0x0C) >> 2);

	/* VIDEOTIMNG_DEBUG_PRINTF("Video Input Timing: %s\n", */
	/* mark because all can't read .. */
	/* s_VMTable[iTE6805_CurVTiming.VIC].format); */
	buf_cnt += sprintf(temp_buf + buf_cnt, "TMDSCLK = %lu.%03luMHz\n",
					   (long unsigned int)iTE6805_CurVTiming.TMDSCLK / 1000,
					   (long unsigned int)iTE6805_CurVTiming.TMDSCLK % 1000);
	buf_cnt += sprintf(temp_buf + buf_cnt, "PCLK = %ld.%03ldMHz\n",
					   (long int)iTE6805_CurVTiming.PCLK / 1000,
					   (long int)iTE6805_CurVTiming.PCLK % 1000);

	/* HSyncWidth,HBackPorch,HActive,HFrontPorch */
	/* If return to upper layer or show log, need to x = x*2 for YUV420
	 * condition */
	if (iTE6805_DATA.Flag_IS_YUV420) {
		buf_cnt += sprintf(temp_buf + buf_cnt, "HActive = %d\n",
						   iTE6805_CurVTiming.HActive * 2);
		buf_cnt += sprintf(temp_buf + buf_cnt, "HTotal = %d\n",
						   iTE6805_CurVTiming.HTotal * 2);
	} else {
		buf_cnt += sprintf(temp_buf + buf_cnt, "HActive = %d\n",
						   iTE6805_CurVTiming.HActive);
		buf_cnt += sprintf(temp_buf + buf_cnt, "HTotal = %d\n",
						   iTE6805_CurVTiming.HTotal);
	}

	buf_cnt += sprintf(temp_buf + buf_cnt, "VActive = %d\n",
					   iTE6805_CurVTiming.VActive);
	buf_cnt +=
		sprintf(temp_buf + buf_cnt, "VTotal = %d\n", iTE6805_CurVTiming.VTotal);

	if (iTE6805_DATA.Flag_IS_YUV420) {
		buf_cnt += sprintf(temp_buf + buf_cnt, "HFrontPorch = %d\n",
						   iTE6805_CurVTiming.HFrontPorch * 2);
		buf_cnt += sprintf(temp_buf + buf_cnt, "HSyncWidth = %d\n",
						   iTE6805_CurVTiming.HSyncWidth * 2);
		buf_cnt += sprintf(temp_buf + buf_cnt, "HBackPorch = %d\n",
						   iTE6805_CurVTiming.HBackPorch * 2);
	} else {
		buf_cnt += sprintf(temp_buf + buf_cnt, "HFrontPorch = %d\n",
						   iTE6805_CurVTiming.HFrontPorch);
		buf_cnt += sprintf(temp_buf + buf_cnt, "HSyncWidth = %d\n",
						   iTE6805_CurVTiming.HSyncWidth);
		buf_cnt += sprintf(temp_buf + buf_cnt, "HBackPorch = %d\n",
						   iTE6805_CurVTiming.HBackPorch);
	}

	buf_cnt += sprintf(temp_buf + buf_cnt, "VFrontPorch = %d\n",
					   iTE6805_CurVTiming.VFrontPorch);
	buf_cnt += sprintf(temp_buf + buf_cnt, "VSyncWidth = %d\n",
					   iTE6805_CurVTiming.VSyncWidth);
	buf_cnt += sprintf(temp_buf + buf_cnt, "VBackPorch = %d\n",
					   iTE6805_CurVTiming.VBackPorch);
	buf_cnt += sprintf(temp_buf + buf_cnt, "FrameRate = %ld\n",
					   (long int)iTE6805_CurVTiming.FrameRate);

	if (iTE6805_CurVTiming.ScanMode == 0) {
		buf_cnt += sprintf(temp_buf + buf_cnt, "ScanMode = Progressive\n");
	} else {
		buf_cnt += sprintf(temp_buf + buf_cnt, "ScanMode = InterLaced\n");
	}

	if (iTE6805_CurVTiming.VPolarity == 1) {
		buf_cnt += sprintf(temp_buf + buf_cnt, "VSyncPol = Positive\n");
	} else {
		buf_cnt += sprintf(temp_buf + buf_cnt, "VSyncPol = Negative\n");
	}

	if (iTE6805_CurVTiming.HPolarity == 1) {
		buf_cnt += sprintf(temp_buf + buf_cnt, "HSyncPol = Positive\n");
	} else {
		buf_cnt += sprintf(temp_buf + buf_cnt, "HSyncPol = Negative\n");
	}

	if (voutstb == 1) {
		buf_cnt +=
			sprintf(temp_buf + buf_cnt,
					"Video Output Detect Stable   LVPCLK_HS=%d\n", lvpclk_hs);
	} else {
		buf_cnt += sprintf(temp_buf + buf_cnt,
						   "Video Output Detect Non-Stable   LVPCLK_HS=%d\n",
						   lvpclk_hs);
	}

	buf_cnt = simple_read_from_buffer(buf, count, ppos, temp_buf, buf_cnt);
	kfree(temp_buf);
	return buf_cnt;
}

static ssize_t itedbg_vid_info_write(struct file *file, const char __user *buf,
									 size_t count, loff_t *ppos)
{
	return count;
}

static ssize_t itedbg_debug_level_write(struct file *file,
										const char __user *buf, size_t count,
										loff_t *ppos)
{
	char tmp_buf[20] = {0};
	const unsigned int tmp_buf_size = ARRAY_SIZE(tmp_buf);

	if (count == 0 || count > tmp_buf_size) {
		pr_warn("input length needs to be set from 1 to %d\n", tmp_buf_size);
		return 0;
	}

	if (copy_from_user(tmp_buf, buf, count)) {
		pr_warn("copy_from_user fail\n");
		return 0;
	}

	if (kstrtouint(tmp_buf, 0, &print_level) == 0) {
		pr_warn("debug level = %08x\n", print_level);
	}

	return count;
}

static ssize_t itedbg_debug_level_read(struct file *file, char __user *buf,
									   size_t count, loff_t *ppos)
{
	char temp_buf[32] = {0};
	size_t buf_cnt = 0;
	buf_cnt += snprintf(temp_buf, 32, "debug level:%08x\n", print_level);
	buf_cnt = simple_read_from_buffer(buf, count, ppos, temp_buf, buf_cnt);
	return buf_cnt;
}

static int itedbg_debug_level_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int itedbg_debug_level_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations vendor_id_ops = {
	.write = itedbg_vendor_id_write,
	.read = itedbg_vendor_id_read,
	.open = itedbg_vendor_id_open,
	.release = itedbg_vendor_id_release,
};
static const struct file_operations edid_ops = {
	.write = itedbg_edid_write,
	.read = itedbg_edid_read,
	.open = itedbg_edid_open,
	.release = itedbg_edid_release,
};
static const struct file_operations vid_info_ops = {
	.write = itedbg_vid_info_write,
	.read = itedbg_vid_info_read,
	.open = itedbg_vid_info_open,
	.release = itedbg_vid_info_release,
};
static const struct file_operations debug_level_ops = {
	.write = itedbg_debug_level_write,
	.read = itedbg_debug_level_read,
	.open = itedbg_debug_level_open,
	.release = itedbg_debug_level_release,
};

int itedbg_init(void)
{
	print_level = 0x0000;
	my_itedbg_root = debugfs_create_dir("itedbg", NULL);
	if (!debugfs_create_file("vendor_id", 0644, my_itedbg_root, NULL,
							 &vendor_id_ops))
		goto Fail;
	if (!debugfs_create_file("edid", 0644, my_itedbg_root, NULL, &edid_ops))
		goto Fail;
	if (!debugfs_create_file("vid_info", 0644, my_itedbg_root, NULL,
							 &vid_info_ops))
		goto Fail;
	if (!debugfs_create_file("debug_level", 0644, my_itedbg_root, NULL,
							 &debug_level_ops))
		goto Fail;

	return 0;

Fail:
	debugfs_remove_recursive(my_itedbg_root);
	my_itedbg_root = NULL;
	return -ENOENT;
}

int itedbg_exit(void)
{
	if (my_itedbg_root) {
		debugfs_remove_recursive(my_itedbg_root);
		my_itedbg_root = NULL;
	}
	return 0;
}
