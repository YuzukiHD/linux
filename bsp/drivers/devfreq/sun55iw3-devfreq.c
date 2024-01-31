// SPDX-License-Identifier: GPL-2.0-or-later
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner devfreq driver.
 *
 * Copyright (C) 2019 Allwinner Technology, Inc.
 *	fanqinghua <fanqinghua@allwinnertech.com>
 *
 * Supplied sunxi ddr devfreq.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/devfreq-event.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/suspend.h>
#include <linux/time.h>
#include <linux/version.h>
#include <../drivers/devfreq/governor.h>

#define PMU_PER_REG			0x1030
#define PMU_CFG_REG			0x1034
#define PMU_FC_EN_MASK		(0xff << 24)
#define PMU_IRQ_RATIO_MASK	(0x3 << 18)
#define PMU_IRQ_MODE_MASK	(0x3 << 16)
#define PMU_IRQ_MASK 		BIT(12)
#define PMU_MODE_MASK		(0xf << 8)
#define PMU_CLK_SEL_MASK	BIT(5)
#define PMU_PER1_MASK		0x7

#define PMU_FC_EN_VAL		(0x7e << 24)
#define PMU_IRQ_RATIO_VAL	(0x1 << 18)
#define PMU_IRQ_MODE_VAL	(0x1 << 16)
#define PMU_IRQ_VAL 		BIT(12)
#define PMU_MODE_VAL		(0x2 << 8)
#define PMU_CLK_SEL_VAL		(0x0 << 5)
#define PMU_PER1_VAL		0x1

#define PMU_FREQ0_HIGH_REG	0x1040
#define PMU_FREQ0_LOW_REG	0x1044
#define PMU_FREQ1_HIGH_REG	0x1048
#define PMU_FREQ1_LOW_REG	0x104C
#define PMU_FREQ2_HIGH_REG	0x1050
#define PMU_FREQ2_LOW_REG	0x1054
#define PMU_FREQ3_HIGH_REG	0x1058
#define PMU_FREQ3_LOW_REG	0x105C

#define PMU_SOFT_CTRL_REG	0x1060
#define PMU_CLR				BIT(1)
#define PMU_RESETN			BIT(0)

#define PMU_EN_REG			0x1064
#define PMU_EN				BIT(0)

#define PMU_IRQ_EN_REG		0x1068
#define PMU_IRQ_EN			0x3
#define PMU_UP_IRQ_EN		BIT(1)
#define PMU_DN_IRQ_EN		BIT(0)

#define PMU_IRQ_PEND_REG	0x106C
#define PMU_UP_IRQ_PEND		BIT(1)
#define PMU_DN_IRQ_PEND     BIT(0)

#define PMU_REQ_R_REG		0x1088
#define PMU_REQ_W_REG		0x108C
#define PMU_REQ_RW_REG		0x1090

#define MASTER_REG0			0x10000
#define DDR_TYPE_LPDDR4		BIT(5)

#define SUNXI_IA13PE	0x1AC0
#define SUNXI_IA13PC	0x1AC4
#define SUNXI_IA13PP	0x1AC8
#define SUNXI_IA13DR	0x1AD4
#define SUNXI_IA13DW	0x1AD8

#define SECOND		1000	/* 1ms(const) */
#define DFSO_UPTHRESHOLD	(90)
#define DFSO_DOWNDIFFERENCTIAL	(5)
#define DE_BD_THRESHOLD (200000000UL / SECOND)
#define DISP_ON_DOWN_THRESHOLD (50000000UL / SECOND)
#define DRIVER_NAME	"devfreq Driver"
#define DEVFREQ_EN BIT(2)

struct sunxi_dmcfreq {
	struct device *dev;
	struct regmap *regmap;
	void __iomem *base;
	struct devfreq *devfreq;
	struct devfreq_simple_ondemand_data ondemand_data;
	struct clk *dmc_clk;
	struct clk *bus_clk;
	struct mutex lock;
	unsigned int dram_div;
	unsigned int down_num;
	unsigned int rw_data;
	unsigned int de_rw_data;
	unsigned int normalvoltage, boostvoltage;
	int irq;
	unsigned long rate, target_rate;
	unsigned long freq0;
	unsigned long freq1;
	unsigned long freq2;
	unsigned long freq3;
	unsigned long freq3_upthreshold;
	unsigned long freq3_downthreshold;
	unsigned long freq2_upthreshold;
	unsigned long freq2_downthreshold;
	unsigned long freq1_upthreshold;
	unsigned long freq1_downthreshold;
	unsigned long freq0_upthreshold;
	unsigned long freq0_downthreshold;
};

static int dbg_enable;
static int pooling_ms = 1;
static int down_threshold = 200 * (SECOND / 1000);
module_param_named(dbg_level, dbg_enable, int, 0644);

#define DBG(args...) \
	do { \
		if (dbg_enable) { \
			pr_info(args); \
		} \
	} while (0)

static void sunxi_start_nsi_de_counter(struct sunxi_dmcfreq *dmcfreq)
{
	unsigned int val;

	/* Automatically updated every 100ms */
	val = (clk_get_rate(dmcfreq->bus_clk) / SECOND) * pooling_ms;
	writel(val, dmcfreq->base + SUNXI_IA13PP);
	val = readl(dmcfreq->base + SUNXI_IA13PE);
	val |= BIT(0);
	writel(val, dmcfreq->base + SUNXI_IA13PE);
}

static void sunxi_stop_nsi_de_counter(struct sunxi_dmcfreq *dmcfreq)
{
	u32 val;

	val = readl(dmcfreq->base + SUNXI_IA13PE);
	val &= ~BIT(0);
	writel(val, dmcfreq->base + SUNXI_IA13PE);

	val = readl(dmcfreq->base + SUNXI_IA13PC);
	val |= BIT(0);
	writel(val, dmcfreq->base + SUNXI_IA13PC);

	udelay(1);

	val = readl(dmcfreq->base + SUNXI_IA13PC);
	val &= ~BIT(0);
	writel(val, dmcfreq->base + SUNXI_IA13PC);
}

u32 sunxi_read_nsi_de_counter(struct sunxi_dmcfreq *dmcfreq)
{
	u32 read_data, write_data;

	read_data = readl(dmcfreq->base + SUNXI_IA13DR);
	write_data = readl(dmcfreq->base + SUNXI_IA13DW);
	return read_data + write_data;
}

static void sunxi_ddrpmu_init(struct sunxi_dmcfreq *dmcfreq)
{
	regmap_update_bits(dmcfreq->regmap, PMU_CFG_REG,
	(unsigned int)(PMU_FC_EN_MASK | PMU_IRQ_RATIO_MASK	| PMU_IRQ_MODE_MASK	|
	PMU_IRQ_MASK | PMU_MODE_MASK | PMU_CLK_SEL_MASK | PMU_PER1_MASK),
	(unsigned int)(PMU_FC_EN_VAL | PMU_IRQ_RATIO_VAL | PMU_IRQ_MODE_VAL	|
	PMU_IRQ_VAL | PMU_MODE_VAL | PMU_CLK_SEL_VAL | PMU_PER1_VAL));
	regmap_update_bits(dmcfreq->regmap, PMU_SOFT_CTRL_REG, PMU_RESETN, -1U);
}

static void sunxi_ddrpmu_start_hardware_counter(struct sunxi_dmcfreq *dmcfreq)
{
	unsigned int val;
	/* Automatically updated every 1ms */
	val = (200000000UL / SECOND) * pooling_ms;
	regmap_write(dmcfreq->regmap, PMU_PER_REG, val);
	regmap_update_bits(dmcfreq->regmap, PMU_EN_REG, PMU_EN, -1U);
	regmap_update_bits(dmcfreq->regmap, PMU_IRQ_EN_REG, PMU_IRQ_EN, -1);
}

static void sunxi_ddrpmu_stop_hardware_counter(struct sunxi_dmcfreq *dmcfreq)
{
	regmap_update_bits(dmcfreq->regmap, PMU_IRQ_EN_REG, PMU_IRQ_EN, 0);
	regmap_update_bits(dmcfreq->regmap, PMU_EN_REG, PMU_EN, 0);
	regmap_update_bits(dmcfreq->regmap, PMU_SOFT_CTRL_REG, PMU_CLR, -1U);
	udelay(1);
	regmap_update_bits(dmcfreq->regmap, PMU_SOFT_CTRL_REG, PMU_CLR, 0);
}

static irqreturn_t sunximon_thread_isr(int irq, void *data)
{
	struct sunxi_dmcfreq *dmcfreq = data;
	unsigned int irq_pending = 0;
	unsigned int rw_data = 0;

	if (IS_ERR_OR_NULL(dmcfreq->devfreq))
		return IRQ_NONE;

	mutex_lock(&dmcfreq->devfreq->lock);
	regmap_update_bits(dmcfreq->regmap, PMU_EN_REG, PMU_EN, 0);
	regmap_read(dmcfreq->regmap, PMU_IRQ_PEND_REG, &irq_pending);
	regmap_write(dmcfreq->regmap, PMU_IRQ_PEND_REG, irq_pending);
	regmap_read(dmcfreq->regmap, PMU_REQ_RW_REG, &rw_data);
	if (rw_data <= 100)
		goto handled;

	if (irq_pending & PMU_DN_IRQ_PEND) {
//		dmcfreq->rw_data =
//			dmcfreq->down_num == 0 ? rw_data : (dmcfreq->rw_data + rw_data) >> 1;
		dmcfreq->rw_data += rw_data;
		dmcfreq->de_rw_data += sunxi_read_nsi_de_counter(dmcfreq);
		dmcfreq->down_num++;
		if (dmcfreq->down_num < down_threshold)
			goto handled;
		else {
			dmcfreq->rw_data /= down_threshold;
			dmcfreq->de_rw_data /= down_threshold;
		}
	} else
		if (dmcfreq->freq0 / 1000000 == dmcfreq->rate / 1000000)
			dmcfreq->rw_data = (rw_data * 3) >> 1;
		else
			dmcfreq->rw_data = rw_data;
	DBG("rate:%ld, rw_data:0x%x\n", dmcfreq->rate, rw_data);
	update_devfreq(dmcfreq->devfreq);
	dmcfreq->down_num = 0;
	dmcfreq->rw_data = 0;
	dmcfreq->de_rw_data = 0;

handled:
	regmap_update_bits(dmcfreq->regmap, PMU_EN_REG, PMU_EN, -1U);
	mutex_unlock(&dmcfreq->devfreq->lock);
	return IRQ_HANDLED;
}

static int sunxi_dmc_target(struct device *dev,
						unsigned long *freq, u32 flags)
{
	struct sunxi_dmcfreq *dmcfreq = dev_get_drvdata(dev);
	unsigned long target_rate;
	struct dev_pm_opp *opp;
	int rc = 0;
	unsigned int timeout;

	if ((dmcfreq->freq0 / 1000000 == *freq / 1000000) &&
		(dmcfreq->de_rw_data >= 100))
		*freq += 1000000;
	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	target_rate = dev_pm_opp_get_freq(opp);
	dmcfreq->rate = clk_get_rate(dmcfreq->dmc_clk);

	if (dmcfreq->rate == target_rate)
		return 0;

	/* start frequency scaling */
	mutex_lock(&dmcfreq->lock);
	sunxi_ddrpmu_stop_hardware_counter(dmcfreq);
//	rc = clk_set_rate(dmcfreq->dmc_clk, target_rate);
	rc = dev_pm_opp_set_opp(dev, opp);
	if (rc)
		goto out;

	timeout = 200;
	while ((dmcfreq->rate = clk_get_rate(dmcfreq->dmc_clk)) / 1000000 != target_rate / 1000000) {
		if (!timeout) {
			dev_err(dev, "change dram clock error!,target_rate:%ld, cur_rate:%ld, req_rate:%ld\n",
					target_rate, dmcfreq->rate, *freq);
			rc = -ETIMEDOUT;
			goto out;
		}

		timeout--;
		cpu_relax();
		msleep(2);
	}

out:
	sunxi_ddrpmu_start_hardware_counter(dmcfreq);
	mutex_unlock(&dmcfreq->lock);
	return rc;
}

static int sunxi_get_event(struct sunxi_dmcfreq *dmcfreq,
				  struct devfreq_event_data *edata)
{
	unsigned int rw_data = 0, ddr_type = 0;

	rw_data = dmcfreq->rw_data;
	regmap_read(dmcfreq->regmap, MASTER_REG0, &ddr_type);
	ddr_type &= DDR_TYPE_LPDDR4;

	/*
	 * read/write: In byte
	 * Max Utilization
	 *
	 * load = (read + write) / (dram_clk * 2 * 4)
	 */
	if (ddr_type)
		edata->load_count = (unsigned long)rw_data << 6;
	else
		edata->load_count = (unsigned long)rw_data << 5;
	edata->total_count = (clk_get_rate(dmcfreq->dmc_clk) / SECOND) * 8 * pooling_ms;

	DBG("drate:%ldM load:%ld rw:%ldM total:%ldM\n",
				clk_get_rate(dmcfreq->dmc_clk) / 1000000,
				(edata->load_count >> 7) * 100 / (edata->total_count >> 7),
				(edata->load_count * SECOND) / pooling_ms / 1000000,
				(edata->total_count * SECOND) / pooling_ms / 1000000);

	return 0;
}

static int sunxi_get_dev_status(struct device *dev,
				struct devfreq_dev_status *stat)
{
	struct sunxi_dmcfreq *dmcfreq = dev_get_drvdata(dev);
	int ret = 0;

	struct devfreq_event_data edata;
	ret = sunxi_get_event(dmcfreq, &edata);
	if (ret < 0)
		return ret;
	/* To be used by the sunxi governor */
	stat->private_data = dmcfreq;

	stat->current_frequency = clk_get_rate(dmcfreq->dmc_clk);
	stat->busy_time = edata.load_count;
	stat->total_time = edata.total_count;

	return ret;
}

static int sunxi_dmcfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct sunxi_dmcfreq *dmcfreq = dev_get_drvdata(dev);

	*freq = clk_get_rate(dmcfreq->dmc_clk);

	return 0;
}

static void sunxi_init_monitor_threshold(struct sunxi_dmcfreq *dmcfreq)
{
	unsigned int ddr_type = 0;

	regmap_read(dmcfreq->regmap, MASTER_REG0, &ddr_type);
	ddr_type &= DDR_TYPE_LPDDR4;
	if (ddr_type) {
		regmap_write(dmcfreq->regmap, PMU_FREQ0_HIGH_REG, dmcfreq->freq3_upthreshold >> 1);
		regmap_write(dmcfreq->regmap, PMU_FREQ0_LOW_REG, dmcfreq->freq3_downthreshold >> 1);
		regmap_write(dmcfreq->regmap, PMU_FREQ1_HIGH_REG, dmcfreq->freq2_upthreshold >> 1);
		regmap_write(dmcfreq->regmap, PMU_FREQ1_LOW_REG, dmcfreq->freq2_downthreshold >> 1);
		regmap_write(dmcfreq->regmap, PMU_FREQ2_HIGH_REG, dmcfreq->freq1_upthreshold >> 1);
		regmap_write(dmcfreq->regmap, PMU_FREQ2_LOW_REG, dmcfreq->freq1_downthreshold >> 1);
		regmap_write(dmcfreq->regmap, PMU_FREQ3_HIGH_REG, dmcfreq->freq0_upthreshold >> 1);
		regmap_write(dmcfreq->regmap, PMU_FREQ3_LOW_REG, dmcfreq->freq0_downthreshold >> 1);
	} else {
		regmap_write(dmcfreq->regmap, PMU_FREQ0_HIGH_REG, dmcfreq->freq3_upthreshold);
		regmap_write(dmcfreq->regmap, PMU_FREQ0_LOW_REG, dmcfreq->freq3_downthreshold);
		regmap_write(dmcfreq->regmap, PMU_FREQ1_HIGH_REG, dmcfreq->freq2_upthreshold);
		regmap_write(dmcfreq->regmap, PMU_FREQ1_LOW_REG, dmcfreq->freq2_downthreshold);
		regmap_write(dmcfreq->regmap, PMU_FREQ2_HIGH_REG, dmcfreq->freq1_upthreshold);
		regmap_write(dmcfreq->regmap, PMU_FREQ2_LOW_REG, dmcfreq->freq1_downthreshold);
		regmap_write(dmcfreq->regmap, PMU_FREQ3_HIGH_REG, dmcfreq->freq0_upthreshold);
		regmap_write(dmcfreq->regmap, PMU_FREQ3_LOW_REG, dmcfreq->freq0_downthreshold);
	}
}

static void sunxi_adjust_freq(struct sunxi_dmcfreq *dmcfreq)
{
	struct device *dev = dmcfreq->dev;
	unsigned long freq = dmcfreq->rate;
	unsigned int dram_div = dmcfreq->dram_div;

	unsigned long freq1_boost = (dram_div >> 16) & 0x80;
	unsigned long freq2_boost = (dram_div >> 8) & 0x80;
	unsigned long freq3_boost = dram_div & 0x80;
	unsigned int upthreshold = dmcfreq->ondemand_data.upthreshold;
	unsigned int downthreshold =
			upthreshold - dmcfreq->ondemand_data.downdifferential;

	dmcfreq->freq0 = (freq << 2) / (((dram_div >> 24) & 0x1f) + 1);
	dmcfreq->freq1 = (freq << 2) / (((dram_div >> 16) & 0x1f) + 1);
	dmcfreq->freq2 = (freq << 2) / (((dram_div >> 8) & 0x1f) + 1);
	dmcfreq->freq3 = (freq << 2) / ((dram_div & 0x1f) + 1);

	pr_info("freq0:%ld, freq1:%ld, freq2:%ld, freq3:%ld, up:%d, down:%d\n",
		dmcfreq->freq0, dmcfreq->freq1, dmcfreq->freq2,
			dmcfreq->freq3, upthreshold, downthreshold);

	dmcfreq->freq0_upthreshold =
			((dmcfreq->freq0 / (100 * SECOND)) * (upthreshold - 20) * pooling_ms) >> 2;
	dmcfreq->freq0_downthreshold =
			((dmcfreq->freq0 / (100 * SECOND)) * (downthreshold - 20) * pooling_ms) >> 2;
	dmcfreq->freq1_upthreshold =
			((dmcfreq->freq1 / (100 * SECOND)) * upthreshold * pooling_ms) >> 2;
	dmcfreq->freq1_downthreshold = DISP_ON_DOWN_THRESHOLD >> 5;
	dmcfreq->freq2_upthreshold =
			((dmcfreq->freq2 / (100 * SECOND)) * upthreshold * pooling_ms) >> 2;
	dmcfreq->freq2_downthreshold =
			((dmcfreq->freq2 / (100 * SECOND)) * downthreshold * pooling_ms) >> 2;
	dmcfreq->freq3_upthreshold =
			((dmcfreq->freq3 / (100 * SECOND)) * upthreshold * pooling_ms) >> 2;
	dmcfreq->freq3_downthreshold =
			((dmcfreq->freq3 / (100 * SECOND)) * downthreshold * pooling_ms) >> 2;

	dev_pm_opp_of_remove_table(dev);
	dev_pm_opp_add(dev, dmcfreq->freq0, dmcfreq->normalvoltage);
	dev_pm_opp_add(dev, dmcfreq->freq1, freq1_boost ? dmcfreq->boostvoltage : dmcfreq->normalvoltage);
	dev_pm_opp_add(dev, dmcfreq->freq2, freq2_boost ? dmcfreq->boostvoltage : dmcfreq->normalvoltage);
	dev_pm_opp_add(dev, dmcfreq->freq3, freq3_boost ? dmcfreq->boostvoltage : dmcfreq->normalvoltage);
}

static struct devfreq_dev_profile sunxi_dmcfreq_profile = {
	.polling_ms     = 100,
	.target         = sunxi_dmc_target,
	.get_dev_status = sunxi_get_dev_status,
	.get_cur_freq   = sunxi_dmcfreq_get_cur_freq,
};

static void sunxi_actmon_pause(struct sunxi_dmcfreq *dmcfreq)
{
	disable_irq(dmcfreq->irq);
	sunxi_stop_nsi_de_counter(dmcfreq);
	sunxi_ddrpmu_stop_hardware_counter(dmcfreq);
}

static int sunxi_actmon_resume(struct sunxi_dmcfreq *dmcfreq)
{
	sunxi_ddrpmu_start_hardware_counter(dmcfreq);
	sunxi_start_nsi_de_counter(dmcfreq);
	enable_irq(dmcfreq->irq);

	return 0;
}

static int sunxi_actmon_start(struct sunxi_dmcfreq *dmcfreq)
{
	int ret = 0;

	ret = sunxi_actmon_resume(dmcfreq);

	return ret;
}

static void sunxi_actmon_stop(struct sunxi_dmcfreq *dmcfreq)
{
	sunxi_actmon_pause(dmcfreq);
}

static int sunxi_governor_get_target(struct devfreq *devfreq,
				     unsigned long *freq)
{
	struct devfreq_dev_status *stat;
	unsigned long long a, b;
	unsigned int dfso_upthreshold = DFSO_UPTHRESHOLD;
	unsigned int dfso_downdifferential = DFSO_DOWNDIFFERENCTIAL;
	struct devfreq_simple_ondemand_data *data = devfreq->data;
	int err;

	err = devfreq_update_stats(devfreq);
	if (err)
		return err;

	stat = &devfreq->last_status;

	if (data) {
		if (data->upthreshold)
			dfso_upthreshold = data->upthreshold;
		if (data->downdifferential)
			dfso_downdifferential = data->downdifferential;
	}
	if (dfso_upthreshold > 100 ||
	    dfso_upthreshold < dfso_downdifferential)
		return -EINVAL;

	/* Assume MAX if it is going to be divided by zero */
	if (stat->total_time == 0) {
		*freq = DEVFREQ_MAX_FREQ;
		return 0;
	}

	/* Prevent overflow */
	if (stat->busy_time >= (1 << 24) || stat->total_time >= (1 << 24)) {
		stat->busy_time >>= 7;
		stat->total_time >>= 7;
	}

	/* Set MAX if it's busy enough */
	if (stat->busy_time * 100 >
	    stat->total_time * dfso_upthreshold) {
//		*freq = DEVFREQ_MAX_FREQ;
		a = stat->busy_time;
		a *= stat->current_frequency;
		b = div_u64(a, stat->total_time);
		b *= 100;
		b = div_u64(b, (dfso_upthreshold - dfso_downdifferential));
		*freq = (unsigned long) b;
		return 0;
	}

	/* Set MAX if we do not know the initial frequency */
	if (stat->current_frequency == 0) {
		*freq = DEVFREQ_MAX_FREQ;
		return 0;
	}

	/* Keep the current frequency */
	if (stat->busy_time * 100 >
	    stat->total_time * (dfso_upthreshold - dfso_downdifferential)) {
		*freq = stat->current_frequency;
		return 0;
	}

	/* Set the desired frequency based on the load */
	a = stat->busy_time;
	a *= stat->current_frequency;
	b = div_u64(a, stat->total_time);
	b *= 100;
	b = div_u64(b, (dfso_upthreshold - dfso_downdifferential / 2));
	*freq = (unsigned long) b;

	return 0;
}

static int sunxi_governor_event_handler(struct devfreq *devfreq,
					unsigned int event, void *data)
{
	struct sunxi_dmcfreq *dmcfreq = dev_get_drvdata(devfreq->dev.parent);
	unsigned int *new_delay = data;
	int ret = 0;

	switch (event) {
	case DEVFREQ_GOV_START:
		devfreq_monitor_start(devfreq);
		ret = sunxi_actmon_start(dmcfreq);
		break;

	case DEVFREQ_GOV_STOP:
		sunxi_actmon_stop(dmcfreq);
		devfreq_monitor_stop(devfreq);
		break;

	case DEVFREQ_GOV_UPDATE_INTERVAL:
		/*
		 * ACTMON hardware supports up to 256 milliseconds for the
		 * sampling period.
		 */
		if (*new_delay > 256) {
			ret = -EINVAL;
			break;
		}

		sunxi_actmon_pause(dmcfreq);
		devfreq_update_interval(devfreq, new_delay);
		ret = sunxi_actmon_resume(dmcfreq);
		break;

	case DEVFREQ_GOV_SUSPEND:
		sunxi_actmon_stop(dmcfreq);
		devfreq_monitor_suspend(devfreq);
		break;

	case DEVFREQ_GOV_RESUME:
		devfreq_monitor_resume(devfreq);
		ret = sunxi_actmon_start(dmcfreq);
		break;
	}

	return ret;
}

static struct devfreq_governor sunxi_devfreq_governor = {
	.name = "sunxi_actmon",
	.attrs = DEVFREQ_GOV_ATTR_POLLING_INTERVAL,
	.flags = DEVFREQ_GOV_FLAG_IRQ_DRIVEN,// | DEVFREQ_GOV_FLAG_IMMUTABLE,
	.get_target_freq = sunxi_governor_get_target,
	.event_handler = sunxi_governor_event_handler,
};

static const struct of_device_id sunxi_dmcfreq_match[] = {
	{ .compatible = "allwinner,sun55iw3-dmc" },
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_dmcfreq_match);

static int sunxi_dmcfreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *dram_np;
	struct sunxi_dmcfreq *dmcfreq;
	const char *rname = "vddcore";
	unsigned int tpr13;
	unsigned int dram_div;
	int rc = 0;

	dram_np = of_find_node_by_path("/dram");
	if (!dram_np) {
		dev_err(&pdev->dev, "failed to find dram node\n");
		return -ENODEV;
	}

	rc = of_property_read_u32(dram_np, "dram_para[30]", &tpr13);
	if (rc) {
		rc = of_property_read_u32(dram_np, "dram_para30", &tpr13);
		if (rc) {
			dev_err(&pdev->dev, "failed to find tpr13\n");
			return -ENODEV;
		}
	}
	if ((tpr13 & DEVFREQ_EN) == 0) {
		dev_info(&pdev->dev, "disable devfreq\n");
		return -ENODEV;
	}

	rc = of_property_read_u32(dram_np, "dram_para[24]", &dram_div);
	if (rc) {
		rc = of_property_read_u32(dram_np, "dram_para24", &dram_div);
		if (rc) {
			dev_err(&pdev->dev, "failed to find dram_div\n");
			return -ENODEV;
		}
	}

	dmcfreq = devm_kzalloc(dev, sizeof(*dmcfreq), GFP_KERNEL);
	if (!dmcfreq)
		return -ENOMEM;

	mutex_init(&dmcfreq->lock);

	dmcfreq->dram_div = dram_div;
	dmcfreq->down_num = 0;
	dmcfreq->rw_data = 0;
	dmcfreq->dmc_clk = devm_clk_get(dev, "dram");
	if (IS_ERR(dmcfreq->dmc_clk)) {
		dev_err(&pdev->dev, "devm_clk_get error!\n");
		return PTR_ERR(dmcfreq->dmc_clk);
	}

	dmcfreq->bus_clk = devm_clk_get(dev, "bus");
	if (IS_ERR(dmcfreq->bus_clk)) {
		dev_err(&pdev->dev, "devm_clk_get error!\n");
		return PTR_ERR(dmcfreq->bus_clk);
	}

	dmcfreq->irq = platform_get_irq(pdev, 0);
	if (dmcfreq->irq < 0) {
		dev_err(&pdev->dev, "irq get error!\n");
		return dmcfreq->irq;
	}

	irq_set_status_flags(dmcfreq->irq, IRQ_NOAUTOEN);

	rc = devm_request_threaded_irq(&pdev->dev, dmcfreq->irq, NULL,
					sunximon_thread_isr, IRQF_ONESHOT,
					"sun55iw3-devfreq", dmcfreq);
	if (rc) {
		dev_err(&pdev->dev, "Interrupt request failed: %d\n", rc);
		return rc;
	}

	dmcfreq->regmap = syscon_node_to_regmap(dev->of_node);
	if (IS_ERR(dmcfreq->regmap)) {
		dev_err(&pdev->dev, "syscon_node_to_regmap error!\n");
		return PTR_ERR(dmcfreq->regmap);
	}

	dmcfreq->base = devm_of_iomap(&pdev->dev, np, 1, NULL);
	if (IS_ERR(dmcfreq->base)) {
		dev_err(&pdev->dev, "devm_ioremap_resource error!\n");
		return PTR_ERR(dmcfreq->base);
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	rc = devm_pm_opp_set_regulators(dev, &rname, 1);
#else
	rc = devm_pm_opp_set_regulators(dev, &rname);
#endif
	if (rc) {
		dev_err(&pdev->dev, "failed to set core OPP regulator\n");
		return rc;
	}

	/*
	 * We add a devfreq driver to our parent since it has a device tree node
	 * with operating points.
	 */
	rc = dev_pm_opp_of_add_table(dev);
	if (rc < 0) {
		dev_err(&pdev->dev, "dev_pm_opp_of_add_table error!\n");
		return rc;
	}

	of_property_read_u32(np, "upthreshold",
			     &dmcfreq->ondemand_data.upthreshold);
	of_property_read_u32(np, "downdifferential",
			     &dmcfreq->ondemand_data.downdifferential);
	of_property_read_u32(np, "normalvoltage",
			     &dmcfreq->normalvoltage);
	of_property_read_u32(np, "boostvoltage",
			     &dmcfreq->boostvoltage);

	dmcfreq->rate = clk_get_rate(dmcfreq->dmc_clk);
	dmcfreq->dev = dev;
	sunxi_adjust_freq(dmcfreq);
	sunxi_init_monitor_threshold(dmcfreq);
	sunxi_start_nsi_de_counter(dmcfreq);
	platform_set_drvdata(pdev, dmcfreq);

	rc = devfreq_add_governor(&sunxi_devfreq_governor);
	if (rc) {
		dev_err(&pdev->dev, "Failed to add governor: %d\n", rc);
		goto remove_table;
	}
	/* Add devfreq device to monitor */
	dmcfreq->devfreq = devm_devfreq_add_device(dev,
						   &sunxi_dmcfreq_profile,
						   DEVFREQ_GOV_PERFORMANCE,
						   &(dmcfreq->ondemand_data));
	if (IS_ERR(dmcfreq->devfreq)) {
		dev_err(&pdev->dev, "devm_devfreq_add_device error!\n");
		rc = PTR_ERR(dmcfreq->devfreq);
		goto remove_governor;
	}
	sunxi_ddrpmu_init(dmcfreq);

	return 0;

remove_governor:
	devfreq_remove_governor(&sunxi_devfreq_governor);
remove_table:
	dev_pm_opp_of_remove_table(dev);

	return rc;
}

static int sunxi_dmcfreq_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sunxi_dmcfreq *dmcfreq = platform_get_drvdata(pdev);

	if (dmcfreq == NULL)
		return 0;

	dev_pm_opp_of_remove_table(dev);
	devfreq_remove_governor(&sunxi_devfreq_governor);
	return 0;
}

static __maybe_unused int sunxi_dmcfreq_suspend(struct device *dev)
{
	struct sunxi_dmcfreq *dmcfreq = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	int ret = 0;

	opp = devfreq_recommended_opp(dev, &(dmcfreq->freq2), 0);
	if (IS_ERR(opp)) {
		dev_err(dev, "failed to get suspend freq\n");
		return PTR_ERR(opp);
	}

	ret = dev_pm_opp_set_opp(dev, opp);
	if (ret) {
		dev_err(dev, "failed to set suspend freq\n");
		return ret;
	}

	if (dmcfreq == NULL)
		return 0;

	ret = devfreq_suspend_device(dmcfreq->devfreq);
	if (ret < 0) {
		dev_err(dev, "failed to suspend the devfreq devices\n");
		return ret;
	}

	return 0;
}

static __maybe_unused int sunxi_dmcfreq_resume(struct device *dev)
{
	struct sunxi_dmcfreq *dmcfreq = dev_get_drvdata(dev);
	int ret = 0;

	if (dmcfreq == NULL)
		return 0;

	sunxi_init_monitor_threshold(dmcfreq);
	sunxi_ddrpmu_init(dmcfreq);
	ret = devfreq_resume_device(dmcfreq->devfreq);
	if (ret < 0) {
		dev_err(dev, "failed to resume the devfreq devices\n");
		return ret;
	}
	return ret;
}

static SIMPLE_DEV_PM_OPS(sunxi_dmcfreq_pm, sunxi_dmcfreq_suspend,
			 sunxi_dmcfreq_resume);

static struct platform_driver sunxi_dmcfreq_driver = {
	.probe  = sunxi_dmcfreq_probe,
	.remove  = sunxi_dmcfreq_remove,
	.driver = {
		.name = "sunxi-dmcfreq",
		.pm = &sunxi_dmcfreq_pm,
		.of_match_table = sunxi_dmcfreq_match,
	},
};

module_platform_driver(sunxi_dmcfreq_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SUNXI dmcfreq driver");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("fanqinghua <fanqinghua@allwinnertech.com>");
MODULE_VERSION("2.0.0");
