/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Copyright (c) 2020-2025, Allwinnertech
 *
 * This file is provided under a dual BSD/GPL license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* #define DEBUG */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <asm/io.h>
#include <linux/remoteproc.h>
#include <linux/mailbox_client.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/of_reserved_mem.h>
#include <linux/spinlock.h>
#include "sunxi_rproc_boot.h"
#include "sunxi_rproc_standby.h"

/*
 * Register define
 */
#define HIFI4_ALT_RESET_VEC_REG		(0x0000) /* HIFI4 Reset Control Register */
#define HIFI4_CTRL_REG0			(0x0004) /* HIFI4 Control Register0 */
#define HIFI4_PRID_REG			(0x000c) /* HIFI4 PRID Register */
#define HIFI4_STAT_REG			(0x0010) /* HIFI4 STAT Register */
#define HIFI4_BIST_CTRL_REG		(0x0014) /* HIFI4 BIST CTRL Register */
#define HIFI4_JTRST_REG			(0x001c) /* HIFI4 JTAG CONFIG RESET Register */
#define HIFI4_VER_REG			(0x0020) /* HIFI4 Version Register */

/*
 * HIFI4 Control Register0
 */
#define BIT_RUN_STALL			(0)
#define BIT_START_VEC_SEL		(1)
#define BIT_HIFI4_CLKEN			(2)

#define HIFI4_BOOT_SRAM_REMAP_REG	(0x8)
#define BIT_SRAM_REMAP_ENABLE		(0)

/* HIFI4 standby */
#if IS_ENABLED(CONFIG_AW_REMOTEPROC_HIFI4_STANDBY)
/*
 * message handler
 */
#define RPROC_STANDBY_HIFI4_NAME	"hifi4"
#define STANDBY_MBOX_NAME		"dsp-standby"
#define SUSPEND_TIMEOUT			msecs_to_jiffies(1000)
#define RESUME_TIMEOUT			msecs_to_jiffies(300)
#define PM_FINISH_FLAG			(0x10)
#define HIFI4_SUSPEND_FINISH_FIELD	(0x16aa0000)
#define HIFI4_STANDBY_SUSPEND		(0xf3f30101)
#define HIFI4_STANDBY_RESUME		(0xf3f30201)
struct sunxi_standby_mbox {
	struct mbox_chan *chan;
	struct mbox_client client;
};
struct sunxi_hifi4_standby {
	void __iomem *rec_reg;
	uint32_t running;
	uint32_t standby_state;
	uint32_t resume_entry;
	struct sunxi_standby_mbox mb;
	struct completion complete_ack;
	struct completion complete_dead;
	struct delayed_work pm_work;
};
#endif /* CONFIG_AW_REMOTEPROC_HIFI4_STANDBY */

extern int simulator_debug;

static int sunxi_rproc_hifi4_reset(struct sunxi_rproc_priv *rproc_priv);
static int sunxi_rproc_hifi4_set_runstall(struct sunxi_rproc_priv *rproc_priv, u32 value);
static int sunxi_rproc_hifi4_set_localram(struct sunxi_rproc_priv *rproc_priv, u32 value);
static int sunxi_rproc_hifi4_attach_pd(struct device *dev, const char *values_of_power_domain_names[], int count);
static void sunxi_rproc_hifi4_create_sysfs(struct device *dev);

static int sunxi_rproc_hifi4_resource_get(struct sunxi_rproc_priv *rproc_priv, struct platform_device *pdev)
{
	struct sunxi_rproc_hifi4_cfg *hifi4_cfg;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	u32 *map_array;
	int ret, i, count;
	const char *values_of_power_domain_names[16];

	rproc_priv->dev = dev;

	hifi4_cfg = devm_kzalloc(dev, sizeof(*hifi4_cfg), GFP_KERNEL);
	if (!hifi4_cfg) {
		dev_err(dev, "alloc hifi4 cfg error\n");
		return -ENOMEM;
	}

	hifi4_cfg->pll_clk = devm_clk_get(dev, "pll");
	if (IS_ERR_OR_NULL(hifi4_cfg->pll_clk)) {
		dev_err(dev, "no find pll in dts\n");
		return -ENXIO;
	}

	hifi4_cfg->mod_clk = devm_clk_get(dev, "mod");
	if (IS_ERR_OR_NULL(hifi4_cfg->mod_clk)) {
		dev_err(dev, "no find mod in dts\n");
		return -ENXIO;
	}

	hifi4_cfg->mcu_mod_clk = devm_clk_get(dev, "mcu-mod");
	if (IS_ERR_OR_NULL(hifi4_cfg->mcu_mod_clk)) {
		dev_err(dev, "no find mcu-mod in dts\n");
		return -ENXIO;
	}

	hifi4_cfg->cfg_clk = devm_clk_get(dev, "cfg");
	if (IS_ERR_OR_NULL(hifi4_cfg->cfg_clk)) {
		dev_err(dev, "no find bus in dts\n");
		return -ENXIO;
	}

	hifi4_cfg->ahbs_clk = devm_clk_get(dev, "ahbs");
	if (IS_ERR_OR_NULL(hifi4_cfg->ahbs_clk)) {
		dev_err(dev, "no find ahbs in dts\n");
		return -ENXIO;
	}

	/* hifi4 module rst clk */
	hifi4_cfg->mod_rst = devm_reset_control_get(dev, "mod-rst");
	if (IS_ERR_OR_NULL(hifi4_cfg->mod_rst)) {
		dev_err(dev, "no find mod_rst in dts\n");
		return -ENXIO;
	}

	/* hifi4 cfg rst clk */
	hifi4_cfg->cfg_rst = devm_reset_control_get(dev, "cfg-rst");
	if (IS_ERR_OR_NULL(hifi4_cfg->cfg_rst)) {
		dev_err(dev, "no find cfg in dts\n");
		return -ENXIO;
	}

	/* hifi4 dbg rst clk */
	hifi4_cfg->dbg_rst = devm_reset_control_get(dev, "dbg-rst");
	if (IS_ERR_OR_NULL(hifi4_cfg->dbg_rst)) {
		dev_err(dev, "no find dbg in dts\n");
		return -ENXIO;
	}

	hifi4_cfg->jtag_pins = devm_pinctrl_get_select(dev, "jtag");
	if (IS_ERR(hifi4_cfg->jtag_pins))
		/* Only for xplorer debug, not assumed as an error */
		dev_dbg(dev, "no find dsp jtag pins in dts or already been used\n");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram-for-cpux");
	if (IS_ERR_OR_NULL(res)) {
		dev_err(dev, "no find sram-for-cpux in dts\n");
		return -ENXIO;
	}

	hifi4_cfg->sram_remap = devm_ioremap_resource(dev, res);
	if (IS_ERR_OR_NULL(hifi4_cfg->sram_remap)) {
		dev_err(dev, "fail to ioremap sram-remap\n");
		return -ENXIO;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hifi4-cfg");
	if (IS_ERR_OR_NULL(res)) {
		dev_err(dev, "no find hifi4-cfg in dts\n");
		return -ENXIO;
	}

	hifi4_cfg->hifi4_cfg = devm_ioremap_resource(dev, res);
	if (IS_ERR_OR_NULL(hifi4_cfg->hifi4_cfg)) {
		dev_err(dev, "fail to ioremap hifi4-cfg\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(np, "clock-frequency", &hifi4_cfg->mod_clk_freq);
	if (ret) {
		dev_err(dev, "fail to get clock-frequency\n");
		return -ENXIO;
	}

	dev_dbg(dev, "%s,%d hifi4_cfg->mod_clk_freq = %d\n", __func__, __LINE__, hifi4_cfg->mod_clk_freq);

	count = of_property_count_strings(np, "power-domain-names");
	if (count > 0) {
		ret = of_property_read_string_array(np, "power-domain-names",
							values_of_power_domain_names, count);
		if (ret < 0) {
			dev_err(dev, "fail to get power domain names\n");
			return -ENXIO;
		}

		ret = sunxi_rproc_hifi4_attach_pd(dev, values_of_power_domain_names, count);
		if (ret) {
			dev_err(dev, "fail to attach pd\n");
			return -ENXIO;
		}

		pm_runtime_enable(dev);
	}

	ret = of_property_count_elems_of_size(np, "memory-mappings", sizeof(u32) * 3);
	if (ret <= 0) {
		dev_err(dev, "fail to get memory-mappings\n");
		ret = -ENXIO;
		goto disadle_pm;
	}
	rproc_priv->mem_maps_cnt = ret;
	rproc_priv->mem_maps = devm_kcalloc(dev, rproc_priv->mem_maps_cnt,
				       sizeof(*(rproc_priv->mem_maps)),
				       GFP_KERNEL);
	if (!rproc_priv->mem_maps) {
		ret = -ENOMEM;
		goto disadle_pm;
	}

	map_array = devm_kcalloc(dev, rproc_priv->mem_maps_cnt * 3, sizeof(u32), GFP_KERNEL);
	if (!map_array) {
		ret = -ENOMEM;
		goto disadle_pm;
	}

	ret = of_property_read_u32_array(np, "memory-mappings", map_array,
					 rproc_priv->mem_maps_cnt * 3);
	if (ret) {
		dev_err(dev, "fail to read memory-mappings\n");
		ret = -ENXIO;
		goto disadle_pm;
	}

	for (i = 0; i < rproc_priv->mem_maps_cnt; i++) {
		rproc_priv->mem_maps[i].da = map_array[i * 3];
		rproc_priv->mem_maps[i].len = map_array[i * 3 + 1];
		rproc_priv->mem_maps[i].pa = map_array[i * 3 + 2];
		dev_dbg(dev, "memory-mappings[%d]: da: 0x%llx, len: 0x%llx, pa: 0x%llx\n",
			i, rproc_priv->mem_maps[i].da, rproc_priv->mem_maps[i].len,
			rproc_priv->mem_maps[i].pa);
	}

	sunxi_rproc_hifi4_create_sysfs(dev);

	devm_kfree(dev, map_array);

	rproc_priv->rproc_cfg = hifi4_cfg;

	return 0;

disadle_pm:
	if (count > 0)
		pm_runtime_disable(dev);

	return ret;

}

static void sunxi_rproc_hifi4_resource_put(struct sunxi_rproc_priv *rproc_priv, struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	pm_runtime_disable(dev);
}

static int sunxi_rproc_hifi4_start(struct sunxi_rproc_priv *rproc_priv)
{
	struct sunxi_rproc_hifi4_cfg *hifi4_cfg = rproc_priv->rproc_cfg;
	struct device *dev = rproc_priv->dev;
	int ret, rate;
	u32 reg_val;

	dev_dbg(dev, "%s,%d\n", __func__, __LINE__);

	pm_runtime_get_sync(dev);

	if (simulator_debug) {
		dev_dbg(dev, "%s,%d hifi4 does not need to reset clk\n",
				__func__, __LINE__);
		return 0;
	}

	/* reset hifi4 */
	ret = sunxi_rproc_hifi4_reset(rproc_priv);
	if (ret) {
		dev_err(dev, "rproc reset err\n");
		return ret;
	}

	/* set pll_clk */
	ret = clk_prepare_enable(hifi4_cfg->pll_clk);
	if (ret) {
		dev_err(dev, "pll clk enable err\n");
		return ret;
	}

	ret = clk_set_parent(hifi4_cfg->mod_clk, hifi4_cfg->pll_clk);
	if (ret) {
		dev_err(dev, "set mod clk parent to pll_clk err\n");
		return ret;
	}

	/* set mod_clk_freq */
	rate = clk_round_rate(hifi4_cfg->mod_clk, hifi4_cfg->mod_clk_freq);
	ret = clk_set_rate(hifi4_cfg->mod_clk, rate);
	if (ret) {
		dev_err(dev, "set mod clk freq err\n");
		return ret;
	}

	/* set mod_clk */
	ret = clk_prepare_enable(hifi4_cfg->mod_clk);
	if (ret) {
		dev_err(dev, "mod clk enable err\n");
		return ret;
	}

	ret = clk_set_parent(hifi4_cfg->mcu_mod_clk, hifi4_cfg->mod_clk);
	if (ret) {
		dev_err(dev, "set mcu mod clk parent to mod clk err\n");
		return ret;
	}

	ret = clk_prepare_enable(hifi4_cfg->mcu_mod_clk);
	if (ret) {
		dev_err(dev, "mcu mod clk enable err\n");
		return ret;
	}

	/* set cfg_clk */
	ret = clk_prepare_enable(hifi4_cfg->cfg_clk);
	if (ret) {
		dev_err(dev, "bus clk enable err\n");
		return ret;
	}

	/* set ahbs_clk */
	ret = clk_prepare_enable(hifi4_cfg->ahbs_clk);
	if (ret) {
		dev_err(dev, "ahbs clk enable err\n");
		return ret;
	}

	/* set cfg to deassert  */
	ret = reset_control_deassert(hifi4_cfg->cfg_rst);
	if (ret) {
		dev_err(dev, "set cfg to deassert err\n");
		return ret;
	}

	/* set dbg to deassert  */
	ret = reset_control_deassert(hifi4_cfg->dbg_rst);
	if (ret) {
		dev_err(dev, "set dbg to deassert err\n");
		return ret;
	}

	/* set vector */
	writel(rproc_priv->pc_entry, (hifi4_cfg->hifi4_cfg + HIFI4_ALT_RESET_VEC_REG));

	/* set statVactorSel */
	reg_val = readl(hifi4_cfg->hifi4_cfg + HIFI4_CTRL_REG0);
	reg_val |= 1 << BIT_START_VEC_SEL;
	writel(reg_val, (hifi4_cfg->hifi4_cfg + HIFI4_CTRL_REG0));

	/* set runstall */
	sunxi_rproc_hifi4_set_runstall(rproc_priv, 1);

	/* set hifi4 clken */
	reg_val = readl(hifi4_cfg->hifi4_cfg + HIFI4_CTRL_REG0);
	reg_val |= 1 << BIT_HIFI4_CLKEN;
	writel(reg_val, (hifi4_cfg->hifi4_cfg + HIFI4_CTRL_REG0));


	/* set rst to deassert  */
	ret = reset_control_deassert(hifi4_cfg->mod_rst);
	if (ret) {
		dev_err(dev, "set mod_rst to deassert err\n");
		return ret;
	}

	/* hifi4 can use local ram */
	sunxi_rproc_hifi4_set_localram(rproc_priv, 0);

	/* hifi4 run */
	sunxi_rproc_hifi4_set_runstall(rproc_priv, 0);

	return 0;
}

static int sunxi_rproc_hifi4_stop(struct sunxi_rproc_priv *rproc_priv)
{
	struct sunxi_rproc_hifi4_cfg *hifi4_cfg = rproc_priv->rproc_cfg;
	int ret;

	dev_dbg(rproc_priv->dev, "%s,%d\n", __func__, __LINE__);

	if (simulator_debug) {
		dev_dbg(rproc_priv->dev, "%s,%d hifi4 does not need to close clk\n",
				__func__, __LINE__);
		return 0;
	}

	clk_disable_unprepare(hifi4_cfg->ahbs_clk);
	clk_disable_unprepare(hifi4_cfg->cfg_clk);
	clk_disable_unprepare(hifi4_cfg->mcu_mod_clk);
	clk_disable_unprepare(hifi4_cfg->mod_clk);

	ret = sunxi_rproc_hifi4_reset(rproc_priv);
	if (ret) {
		dev_err(rproc_priv->dev, "rproc reset err\n");
		return ret;
	}

	pm_runtime_put_sync(rproc_priv->dev);

	return 0;
}

static int sunxi_rproc_hifi4_attach(struct sunxi_rproc_priv *rproc_priv)
{
	struct sunxi_rproc_hifi4_cfg *hifi4_cfg = rproc_priv->rproc_cfg;
	struct device *dev = rproc_priv->dev;
	int ret, rate;

	dev_dbg(dev, "%s,%d\n", __func__, __LINE__);

	pm_runtime_get_sync(dev);

	/* set pll_clk */
	ret = clk_prepare_enable(hifi4_cfg->pll_clk);
	if (ret) {
		dev_err(dev, "pll clk enable err\n");
		return ret;
	}

	ret = clk_set_parent(hifi4_cfg->mod_clk, hifi4_cfg->pll_clk);
	if (ret) {
		dev_err(dev, "set mod clk parent to pll_clk err\n");
		return ret;
	}

	/* set mod_clk_freq */
	rate = clk_round_rate(hifi4_cfg->mod_clk, hifi4_cfg->mod_clk_freq);
	ret = clk_set_rate(hifi4_cfg->mod_clk, rate);
	if (ret) {
		dev_err(dev, "set mod clk freq err\n");
		return ret;
	}

	/* set mod_clk */
	ret = clk_prepare_enable(hifi4_cfg->mod_clk);
	if (ret) {
		dev_err(dev, "mod clk enable err\n");
		return ret;
	}

	ret = clk_set_parent(hifi4_cfg->mcu_mod_clk, hifi4_cfg->mod_clk);
	if (ret) {
		dev_err(dev, "set mcu mod clk parent to mod clk err\n");
		return ret;
	}

	ret = clk_prepare_enable(hifi4_cfg->mcu_mod_clk);
	if (ret) {
		dev_err(dev, "mcu mod clk enable err\n");
		return ret;
	}

	/* set cfg_clk */
	ret = clk_prepare_enable(hifi4_cfg->cfg_clk);
	if (ret) {
		dev_err(dev, "bus clk enable err\n");
		return ret;
	}

	/* set ahbs_clk */
	ret = clk_prepare_enable(hifi4_cfg->ahbs_clk);
	if (ret) {
		dev_err(dev, "ahbs clk enable err\n");
		return ret;
	}

	/* set cfg to deassert  */
	ret = reset_control_deassert(hifi4_cfg->cfg_rst);
	if (ret) {
		dev_err(dev, "set cfg to deassert err\n");
		return ret;
	}

	/* set dbg to deassert  */
	ret = reset_control_deassert(hifi4_cfg->dbg_rst);
	if (ret) {
		dev_err(dev, "set dbg to deassert err\n");
		return ret;
	}

	/* set rst to deassert  */
	ret = reset_control_deassert(hifi4_cfg->mod_rst);
	if (ret) {
		dev_err(dev, "set mod_rst to deassert err\n");
		return ret;
	}

	return 0;
}

static int sunxi_rproc_hifi4_reset(struct sunxi_rproc_priv *rproc_priv)
{
	struct sunxi_rproc_hifi4_cfg *hifi4_cfg = rproc_priv->rproc_cfg;
	int ret;

	ret = reset_control_assert(hifi4_cfg->mod_rst);
	if (ret)
		return -ENXIO;

	ret = reset_control_assert(hifi4_cfg->cfg_rst);
	if (ret)
		return -ENXIO;

	ret = reset_control_assert(hifi4_cfg->dbg_rst);
	if (ret)
		return -ENXIO;

	return 0;
}

static int sunxi_rproc_hifi4_set_localram(struct sunxi_rproc_priv *rproc_priv, u32 value)
{
	struct sunxi_rproc_hifi4_cfg *hifi4_cfg = rproc_priv->rproc_cfg;
	u32 reg_val;

	reg_val = readl(hifi4_cfg->sram_remap);
	reg_val &= ~(1 << BIT_SRAM_REMAP_ENABLE);
	reg_val |= (value << BIT_SRAM_REMAP_ENABLE);
	writel(reg_val, (hifi4_cfg->sram_remap));

	return 0;
}

static int sunxi_rproc_hifi4_set_runstall(struct sunxi_rproc_priv *rproc_priv, u32 value)
{
	struct sunxi_rproc_hifi4_cfg *hifi4_cfg = rproc_priv->rproc_cfg;
	u32 reg_val;

	reg_val = readl(hifi4_cfg->hifi4_cfg + HIFI4_CTRL_REG0);
	reg_val &= ~(1 << BIT_RUN_STALL);
	reg_val |= (value << BIT_RUN_STALL);
	writel(reg_val, (hifi4_cfg->hifi4_cfg + HIFI4_CTRL_REG0));

	return 0;
}

static bool sunxi_rproc_hifi4_is_booted(struct sunxi_rproc_priv *rproc_priv)
{
	struct sunxi_rproc_hifi4_cfg *hifi4_cfg = rproc_priv->rproc_cfg;

	return __clk_is_enabled(hifi4_cfg->mcu_mod_clk) && \
		__clk_is_enabled(hifi4_cfg->mod_clk) && \
		__clk_is_enabled(hifi4_cfg->cfg_clk);
}

static struct sunxi_rproc_ops sunxi_rproc_hifi4_ops = {
	.resource_get = sunxi_rproc_hifi4_resource_get,
	.resource_put = sunxi_rproc_hifi4_resource_put,
	.start = sunxi_rproc_hifi4_start,
	.stop = sunxi_rproc_hifi4_stop,
	.attach = sunxi_rproc_hifi4_attach,
	.reset = sunxi_rproc_hifi4_reset,
	.set_localram = sunxi_rproc_hifi4_set_localram,
	.set_runstall = sunxi_rproc_hifi4_set_runstall,
	.is_booted = sunxi_rproc_hifi4_is_booted,
};

static int sunxi_rproc_hifi4_attach_pd(struct device *dev, const char *values_of_power_domain_names[], int count)
{
	struct device_link *link;
	struct device *pd_dev;
	int i;

	/* Do nothing when in a single power domain */
	if (dev->pm_domain)
		return 0;
	for (i = 0; i < count; i++) {
		pd_dev = dev_pm_domain_attach_by_name(dev, values_of_power_domain_names[i]);
		if (IS_ERR(pd_dev))
			return PTR_ERR(pd_dev);
		/* Do nothing when power domain missing */
		else if (!pd_dev)
			return 0;
		else {
			link = device_link_add(dev, pd_dev,
					DL_FLAG_STATELESS |
					DL_FLAG_PM_RUNTIME |
					DL_FLAG_RPM_ACTIVE);
			if (!link) {
				dev_err(dev, "Failed to add device_link to %s.\n",
						values_of_power_domain_names[i]);
				return -EINVAL;
			}
		}
	}

	return 0;
}

#if IS_ENABLED(CONFIG_AW_REMOTEPROC_HIFI4_STANDBY)
static void pm_work_func(struct work_struct *work)
{
	struct delayed_work *pm_work = to_delayed_work(work);
	struct sunxi_hifi4_standby *hifi4_standby;
	uint32_t reg_val;

	hifi4_standby = container_of(pm_work, struct sunxi_hifi4_standby, pm_work);

	reg_val = readl(hifi4_standby->rec_reg);

	if (reg_val == (HIFI4_SUSPEND_FINISH_FIELD | PM_FINISH_FLAG))
		complete(&hifi4_standby->complete_dead);
	else
		schedule_delayed_work(&hifi4_standby->pm_work, msecs_to_jiffies(50));
}

static void sunxi_rproc_hifi4_standby_rxcallback(struct mbox_client *cl, void *data)
{
	uint32_t rec_data = *(uint32_t *)data;
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	struct sunxi_rproc_standby *rproc_standby;
	struct sunxi_hifi4_standby *hifi4_standby;

	rproc_standby = sunxi_rproc_get_standby_handler(rproc->priv);
	if (!rproc_standby) {
		dev_err(rproc_standby->dev, "rx get standby resource failed\n");
		return;
	}

	hifi4_standby = rproc_standby->priv;

	switch (rec_data) {
	case HIFI4_STANDBY_SUSPEND:
	case HIFI4_STANDBY_RESUME:
		complete(&hifi4_standby->complete_ack);
		break;
	default:
		dev_warn(rproc_standby->dev, "rx unkown data(0x%08x)\n", rec_data);
		break;
	}

	dev_dbg(rproc_standby->dev, "hifi4 standby recive msg: 0x%08x\n", rec_data);
}

static int sunxi_rproc_hifi4_standby_request_mbox(struct sunxi_rproc_standby *rproc_standby)
{
	struct sunxi_hifi4_standby *hifi4_standby = rproc_standby->priv;
	struct device *dev = rproc_standby->dev;

	hifi4_standby->mb.client.rx_callback = sunxi_rproc_hifi4_standby_rxcallback;
	hifi4_standby->mb.client.tx_done = NULL;
	hifi4_standby->mb.client.tx_block = false;
	hifi4_standby->mb.client.dev = dev;
	hifi4_standby->mb.chan = mbox_request_channel_byname(&hifi4_standby->mb.client,
							STANDBY_MBOX_NAME);

	if (IS_ERR(hifi4_standby->mb.chan)) {
		dev_err(dev, "standby get %s mbox failed\n", STANDBY_MBOX_NAME);
		hifi4_standby->mb.chan = NULL;
		return -EBUSY;
	}

	return 0;
}

static void sunxi_rproc_hifi4_standby_free_mbox(struct sunxi_rproc_standby *rproc_standby)
{
	struct sunxi_hifi4_standby *hifi4_standby = rproc_standby->priv;

	mbox_free_channel(hifi4_standby->mb.chan);
	hifi4_standby->mb.chan = NULL;
}

static int sunxi_rproc_hifi4_standby_init(struct sunxi_rproc_standby *rproc_standby, struct platform_device *pdev)
{
	struct sunxi_hifi4_standby *hifi4_standby;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	uint32_t rec_reg;
	int ret;

	if (!rproc_standby || !rproc_standby->rproc_priv)
		return -EINVAL;

	if (of_property_read_u32(np, "standby-record-reg", &rec_reg)) {
		dev_err(dev, "failed to get standby-record-reg property\n");
		return -ENXIO;
	}

	if (of_get_property(np, "standby-ctrl-en", NULL)) {
		if (of_property_read_u32(np, "standby-ctrl-en", &rproc_standby->ctrl_en)) {
			dev_err(dev, "failed to get standby-ctrl-en property\n");
			return -ENXIO;
		}
	} else
		rproc_standby->ctrl_en = 0;

	if (!rproc_standby->ctrl_en) {
		rproc_standby->dev = dev;
		rproc_standby->priv = NULL;
		dev_info(dev, "standby ctrl_en is 0\n");
		return 0;
	}

	rproc_standby->dev = dev;
	ret = sunxi_rproc_standby_init_prepare(rproc_standby);
	if (ret) {
		dev_err(dev, "standby prepare failed\n");
		return ret;
	}

	hifi4_standby = devm_kzalloc(dev, sizeof(*hifi4_standby), GFP_KERNEL);
	if (!hifi4_standby) {
		dev_err(dev, "alloc hifi4_standby failed\n");
		return -ENOMEM;
	}

	hifi4_standby->rec_reg = ioremap(rec_reg, 4);
	if (IS_ERR_OR_NULL(hifi4_standby->rec_reg)) {
		dev_err(dev, "fail to ioremap hifi4_standby rec_reg\n");
		ret = -ENXIO;
		goto init_free_priv;
	}

	hifi4_standby->running = 0;
	hifi4_standby->standby_state = 0;
	rproc_standby->priv = hifi4_standby;

	ret = sunxi_rproc_hifi4_standby_request_mbox(rproc_standby);
	if (ret) {
		dev_err(dev, "request hifi4 standby mbox failed, return %d\n", ret);
		goto init_rec_unmap;
	}

	init_completion(&hifi4_standby->complete_ack);
	init_completion(&hifi4_standby->complete_dead);
	INIT_DELAYED_WORK(&hifi4_standby->pm_work, pm_work_func);
	dev_info(dev, "hifi4 standby init end\n");

	return 0;

init_rec_unmap:
	iounmap(hifi4_standby->rec_reg);
init_free_priv:
	rproc_standby->priv = NULL;
	rproc_standby->ctrl_en = 0;
	return ret;
}

static void sunxi_rproc_hifi4_standby_exit(struct sunxi_rproc_standby *rproc_standby, struct platform_device *pdev)
{
	struct sunxi_hifi4_standby *hifi4_standby;

	if (!rproc_standby || !rproc_standby->priv)
		return;

	hifi4_standby = rproc_standby->priv;

	if (rproc_standby->ctrl_en) {
		cancel_delayed_work_sync(&hifi4_standby->pm_work);
		reinit_completion(&hifi4_standby->complete_dead);
		reinit_completion(&hifi4_standby->complete_ack);
		sunxi_rproc_hifi4_standby_free_mbox(rproc_standby);
		iounmap(hifi4_standby->rec_reg);
	}

	rproc_standby->priv = NULL;
}

static int sunxi_rproc_hifi4_standby_start(struct sunxi_rproc_standby *rproc_standby)
{
	struct sunxi_hifi4_standby *hifi4_standby = rproc_standby->priv;

	hifi4_standby->running = 1;
	hifi4_standby->standby_state = 1;

	return 0;
}

static int sunxi_rproc_hifi4_standby_stop(struct sunxi_rproc_standby *rproc_standby)
{
	struct sunxi_hifi4_standby *hifi4_standby = rproc_standby->priv;

	hifi4_standby->running = 0;
	hifi4_standby->standby_state = 0;

	return 0;
}

static int sunxi_rproc_hifi4_standby_suspend_late(struct sunxi_rproc_standby *rproc_standby)
{
	int ret;
	uint32_t data;
	struct sunxi_hifi4_standby *hifi4_standby = rproc_standby->priv;
	struct sunxi_rproc_hifi4_cfg *hifi4_cfg = rproc_standby->rproc_priv->rproc_cfg;

	if (!hifi4_standby->running || !sunxi_rproc_hifi4_is_booted(rproc_standby->rproc_priv))
		return 0;

	/* suspend  */
	data = HIFI4_STANDBY_SUSPEND;

	ret = mbox_send_message(hifi4_standby->mb.chan, &data);
	if (ret < 0) {
		dev_err(rproc_standby->dev, "suspend send mbox msg failed\n");
		return ret;
	}

	ret = wait_for_completion_timeout(&hifi4_standby->complete_ack, SUSPEND_TIMEOUT);
	if (!ret) {
		dev_err(rproc_standby->dev, "timeout return suspend ack\n");
		return -ETIME;
	}

	/* check whether hifi4 completes the suspend process */
	schedule_delayed_work(&hifi4_standby->pm_work, msecs_to_jiffies(50));
	ret = wait_for_completion_timeout(&hifi4_standby->complete_dead, SUSPEND_TIMEOUT);
	cancel_delayed_work_sync(&hifi4_standby->pm_work);
	if (!ret) {
		dev_err(rproc_standby->dev, "timeout return suspend dead\n");
		writel(HIFI4_SUSPEND_FINISH_FIELD & ~(PM_FINISH_FLAG), hifi4_standby->rec_reg);
		return -ETIME;
	}

	/* save resume entry, which written by the remote core before dead */
	hifi4_standby->resume_entry = readl(hifi4_cfg->hifi4_cfg + HIFI4_ALT_RESET_VEC_REG);

	hifi4_standby->standby_state = 0;

	dev_info(rproc_standby->dev, "suspend_late finish!\n");

	return 0;
}
static int sunxi_rproc_hifi4_standby_resume_early(struct sunxi_rproc_standby *rproc_standby)
{
	int ret;
	uint32_t reg_val;
	uint32_t data = HIFI4_STANDBY_RESUME;
	struct sunxi_hifi4_standby *hifi4_standby = rproc_standby->priv;
	struct sunxi_rproc_hifi4_cfg *hifi4_cfg = rproc_standby->rproc_priv->rproc_cfg;

	if (!hifi4_standby->running)
		return 0;

	/* set resume vector */
	writel(hifi4_standby->resume_entry, (hifi4_cfg->hifi4_cfg + HIFI4_ALT_RESET_VEC_REG));

	if (!sunxi_rproc_hifi4_is_booted(rproc_standby->rproc_priv)) {
		dev_err(rproc_standby->dev, "need get clk again\n");
		return -EBUSY;
	}

	reg_val = readl(hifi4_standby->rec_reg);
	if ((reg_val & 0xffff0000) == HIFI4_SUSPEND_FINISH_FIELD) {
		/* Note the asynchronous rewrite, the register is only used for one core */
		writel(reg_val & ~(PM_FINISH_FLAG), hifi4_standby->rec_reg);
	}

	ret = mbox_send_message(hifi4_standby->mb.chan, &data);
	if (ret < 0) {
		dev_err(rproc_standby->dev, "resume send mbox msg failed\n");
		return ret;
	}

	ret = wait_for_completion_timeout(&hifi4_standby->complete_ack, RESUME_TIMEOUT);
	if (!ret) {
		dev_err(rproc_standby->dev, "timeout return resume ack\n");
		return -EBUSY;
	}

	hifi4_standby->standby_state = 0;

	dev_info(rproc_standby->dev, "resume_early finish!\n");

	return 0;
}

static struct sunxi_rproc_standby_ops rproc_hifi4_standby_ops = {
	.init = sunxi_rproc_hifi4_standby_init,
	.exit = sunxi_rproc_hifi4_standby_exit,
	.attach = sunxi_rproc_hifi4_standby_start,
	.detach = sunxi_rproc_hifi4_standby_stop,
	.start = sunxi_rproc_hifi4_standby_start,
	.stop = sunxi_rproc_hifi4_standby_stop,
	.suspend_late = sunxi_rproc_hifi4_standby_suspend_late,
	.resume_early = sunxi_rproc_hifi4_standby_resume_early,
};

static ssize_t sunxi_rproc_hifi4_standby_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	struct sunxi_rproc *ddata = rproc->priv;
	struct sunxi_rproc_standby *rproc_standby = ddata->rproc_standby;
	struct sunxi_hifi4_standby *hifi4_standby = rproc_standby->priv;

	if (!hifi4_standby->running)
		return scnprintf(buf, PAGE_SIZE, "%s\n", "hifi4 not in running state!");

	return scnprintf(buf, PAGE_SIZE, "Current standby state is %s\n", hifi4_standby->standby_state ? "resume" : "suspend");
}

static ssize_t sunxi_rproc_hifi4_standby_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	struct sunxi_rproc *ddata = rproc->priv;
	struct sunxi_rproc_standby *rproc_standby = ddata->rproc_standby;
	struct sunxi_hifi4_standby *hifi4_standby = rproc_standby->priv;
	static char const *hifi4_standby_mode[] = { "suspend", "resume" };

	if (!hifi4_standby->running) {
		dev_err(dev, "hifi4 not in running state!\n");
		return -EINVAL;
	}

	if (!strncmp(buf, hifi4_standby_mode[0], strlen(hifi4_standby_mode[0])))
		sunxi_rproc_hifi4_standby_suspend_late(rproc_standby);
	else if (!strncmp(buf, hifi4_standby_mode[1], strlen(hifi4_standby_mode[1])))
		sunxi_rproc_hifi4_standby_resume_early(rproc_standby);
	else {
		dev_err(dev, "unsupport hifi4 standby mode %s\n", buf);
		return -EINVAL;
	}

	return count;
}
#endif /* CONFIG_AW_REMOTEPROC_HIFI4_STANDBY */

static struct device_attribute hifi4_debug_attr[] = {
#if IS_ENABLED(CONFIG_AW_REMOTEPROC_HIFI4_STANDBY)
	__ATTR(standby, S_IRUGO|S_IWUSR, sunxi_rproc_hifi4_standby_show, sunxi_rproc_hifi4_standby_store),
#endif /* CONFIG_AW_REMOTEPROC_HIFI4_STANDBY */
};

static void sunxi_rproc_hifi4_create_sysfs(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hifi4_debug_attr); i++)
		device_create_file(dev, &hifi4_debug_attr[i]);
}

/* hifi4_boot_init must run before sunxi_rproc probe */
static int __init sunxi_rproc_hifi4_boot_init(void)
{
	int ret;

	ret = sunxi_rproc_priv_ops_register("hifi4", &sunxi_rproc_hifi4_ops, NULL);
	if (ret) {
		pr_err("hifi4 register ops failed\n");
		return ret;
	}

#if IS_ENABLED(CONFIG_AW_REMOTEPROC_HIFI4_STANDBY)
	ret = sunxi_rproc_standby_ops_register(RPROC_STANDBY_HIFI4_NAME, &rproc_hifi4_standby_ops, NULL);
	if (ret) {
		pr_err("hifi4 registers standby ops failed\n");
		return ret;
	}
#endif

	return 0;
}
subsys_initcall(sunxi_rproc_hifi4_boot_init);

static void __exit sunxi_rproc_hifi4_boot_exit(void)
{
	int ret;

#if IS_ENABLED(CONFIG_AW_REMOTEPROC_HIFI4_STANDBY)
	ret = sunxi_rproc_standby_ops_unregister(RPROC_STANDBY_HIFI4_NAME);
	if (ret)
		pr_err("hifi4 unregister standby ops failed\n");
#endif

	ret = sunxi_rproc_priv_ops_unregister("hifi4");
	if (ret)
		pr_err("hifi4 unregister ops failed\n");
}
module_exit(sunxi_rproc_hifi4_boot_exit)

MODULE_DESCRIPTION("Allwinner sunxi rproc hifi4 boot driver");
MODULE_AUTHOR("xuminghui <xuminghui@allwinnertech.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.1.2");
