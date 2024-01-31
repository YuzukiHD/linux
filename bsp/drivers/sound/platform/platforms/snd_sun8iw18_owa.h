/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner's ALSA SoC Audio driver
 *
 * Copyright (c) 2022, lijingpsw <lijingpsw@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef __SND_SUN8IW18_OWA_H
#define __SND_SUN8IW18_OWA_H

struct sunxi_owa_clk {
	struct clk *clk_pll_audio;
	struct clk *clk_owa;
	struct clk *clk_bus;
	struct reset_control *clk_rst;
	unsigned int pll_fs_tx;
};

static int snd_sunxi_clk_init(struct platform_device *pdev, struct sunxi_owa_clk *clk);
static void snd_sunxi_clk_exit(struct sunxi_owa_clk *clk);
static int snd_sunxi_clk_bus_enable(struct sunxi_owa_clk *clk);
static int snd_sunxi_clk_enable(struct sunxi_owa_clk *clk);
static void snd_sunxi_clk_bus_disable(struct sunxi_owa_clk *clk);
static void snd_sunxi_clk_disable(struct sunxi_owa_clk *clk);
static int snd_sunxi_clk_rate(struct sunxi_owa_clk *clk, unsigned int freq_in,
			      unsigned int freq_out);

static inline int snd_sunxi_clk_init(struct platform_device *pdev, struct sunxi_owa_clk *clk)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	SND_LOG_DEBUG("\n");

	/* get rst clk */
	clk->clk_rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(clk->clk_rst)) {
		SND_LOG_ERR("clk rst get failed\n");
		ret =  PTR_ERR(clk->clk_rst);
		goto err_get_clk_rst;
	}

	/* get bus clk */
	clk->clk_bus = of_clk_get_by_name(np, "clk_bus_owa");
	if (IS_ERR_OR_NULL(clk->clk_bus)) {
		SND_LOG_ERR("clk bus get failed\n");
		ret = PTR_ERR(clk->clk_bus);
		goto err_get_clk_bus;
	}

	/* get parent clk */
	clk->clk_pll_audio = of_clk_get_by_name(np, "clk_pll_audio_4x");
	if (IS_ERR_OR_NULL(clk->clk_pll_audio)) {
		SND_LOG_ERR("clk pll get failed\n");
		ret = PTR_ERR(clk->clk_pll_audio);
		goto err_get_clk_pll_audio;
	}

	/* get owa clk */
	clk->clk_owa = of_clk_get_by_name(np, "clk_owa");
	if (IS_ERR_OR_NULL(clk->clk_owa)) {
		SND_LOG_ERR("clk owa get failed\n");
		ret = PTR_ERR(clk->clk_owa);
		goto err_get_clk_owa;
	}

	/* set clk owa parent of pllaudio */
	if (clk_set_parent(clk->clk_owa, clk->clk_pll_audio)) {
		SND_LOG_ERR("set parent clk owa failed\n");
		ret = -EINVAL;
		goto err_set_parent;
	}

	clk->pll_fs_tx = 1;

	return 0;

err_set_parent:
	clk_put(clk->clk_owa);
err_get_clk_owa:
	clk_put(clk->clk_pll_audio);
err_get_clk_pll_audio:
	clk_put(clk->clk_bus);
err_get_clk_bus:
err_get_clk_rst:
	return ret;
}

static inline void snd_sunxi_clk_exit(struct sunxi_owa_clk *clk)
{
	SND_LOG_DEBUG("\n");

	clk_put(clk->clk_owa);
	clk_put(clk->clk_pll_audio);
	clk_put(clk->clk_bus);
}

static inline int snd_sunxi_clk_bus_enable(struct sunxi_owa_clk *clk)
{
	int ret = 0;

	SND_LOG_DEBUG("\n");

	if (reset_control_deassert(clk->clk_rst)) {
		SND_LOG_ERR("clk_rst deassert failed\n");
		ret = -EINVAL;
		goto err_deassert_rst;
	}

	if (clk_prepare_enable(clk->clk_bus)) {
		SND_LOG_ERR("clk_bus enable failed\n");
		ret = -EINVAL;
		goto err_enable_clk_bus;
	}

	return 0;

err_enable_clk_bus:
	reset_control_assert(clk->clk_rst);
err_deassert_rst:
	return ret;
}

static inline int snd_sunxi_clk_enable(struct sunxi_owa_clk *clk)
{
	int ret = 0;

	SND_LOG_DEBUG("\n");

	if (clk_prepare_enable(clk->clk_pll_audio)) {
		SND_LOG_ERR("clk_pll_audio enable failed\n");
		ret = -EINVAL;
		goto err_enable_clk_parent;
	}

	if (clk_prepare_enable(clk->clk_owa)) {
		SND_LOG_ERR("clk_owa enable failed\n");
		ret = -EINVAL;
		goto err_enable_clk_owa;
	}

	return 0;

err_enable_clk_owa:
	clk_disable_unprepare(clk->clk_pll_audio);
err_enable_clk_parent:
	return ret;
}

static inline void snd_sunxi_clk_bus_disable(struct sunxi_owa_clk *clk)
{
	SND_LOG_DEBUG("\n");

	clk_disable_unprepare(clk->clk_bus);
	reset_control_assert(clk->clk_rst);
}

static inline void snd_sunxi_clk_disable(struct sunxi_owa_clk *clk)
{
	SND_LOG_DEBUG("\n");

	clk_disable_unprepare(clk->clk_owa);
	clk_disable_unprepare(clk->clk_pll_audio);
}

static inline int snd_sunxi_clk_rate(struct sunxi_owa_clk *clk, unsigned int freq_in,
				     unsigned int freq_out)
{
	SND_LOG_DEBUG("\n");

	if (clk_set_rate(clk->clk_owa, freq_out)) {
		SND_LOG_ERR("freq : %u module clk unsupport\n", freq_out);
		return -EINVAL;
	}

	return 0;
}

#endif /* __SND_SUN8IW18_OWA_H */
