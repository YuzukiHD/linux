/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner's ALSA SoC Audio driver
 *
 * Copyright (c) 2022, Dby <dby@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef __SND_SUN50IW10_I2S_H
#define __SND_SUN50IW10_I2S_H

struct sunxi_i2s_clk {
	struct reset_control *clk_rst;
	struct clk *clk_bus;

	struct clk *clk_pll_audio;
	struct clk *clk_pll_com;
	struct clk *clk_pll_com_audio;

	struct clk *clk_i2s;
};

static int snd_sunxi_clk_init(struct platform_device *pdev, struct sunxi_i2s_clk *clk);
static void snd_sunxi_clk_exit(struct sunxi_i2s_clk *clk);
static int snd_sunxi_clk_bus_enable(struct sunxi_i2s_clk *clk);
static int snd_sunxi_clk_enable(struct sunxi_i2s_clk *clk);
static void snd_sunxi_clk_bus_disable(struct sunxi_i2s_clk *clk);
static void snd_sunxi_clk_disable(struct sunxi_i2s_clk *clk);
static int snd_sunxi_clk_rate(struct sunxi_i2s_clk *clk, unsigned int freq_in,
			      unsigned int freq_out);

static inline int snd_sunxi_clk_init(struct platform_device *pdev, struct sunxi_i2s_clk *clk)
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
	clk->clk_bus = of_clk_get_by_name(np, "clk_bus_i2s");
	if (IS_ERR_OR_NULL(clk->clk_bus)) {
		SND_LOG_ERR("clk bus get failed\n");
		ret = PTR_ERR(clk->clk_bus);
		goto err_get_clk_bus;
	}

	/* get parent clk */
	clk->clk_pll_audio = of_clk_get_by_name(np, "clk_pll_audio");
	if (IS_ERR_OR_NULL(clk->clk_pll_audio)) {
		SND_LOG_ERR("clk_pll_audio get failed\n");
		ret = PTR_ERR(clk->clk_pll_audio);
		goto err_get_clk_pll_audio;
	}
	clk->clk_pll_com = of_clk_get_by_name(np, "clk_pll_com");
	if (IS_ERR_OR_NULL(clk->clk_pll_com)) {
		SND_LOG_ERR("clk_pll_com get failed\n");
		ret = PTR_ERR(clk->clk_pll_com);
		goto err_get_clk_pll_com;
	}
	clk->clk_pll_com_audio = of_clk_get_by_name(np, "clk_pll_com_audio");
	if (IS_ERR_OR_NULL(clk->clk_pll_com_audio)) {
		SND_LOG_ERR("clk_pll_com_audio get failed\n");
		ret = PTR_ERR(clk->clk_pll_com_audio);
		goto err_get_clk_pll_com_audio;
	}

	/* get module clk */
	clk->clk_i2s = of_clk_get_by_name(np, "clk_i2s");
	if (IS_ERR_OR_NULL(clk->clk_i2s)) {
		SND_LOG_ERR("clk i2s get failed\n");
		ret = PTR_ERR(clk->clk_i2s);
		goto err_get_clk_i2s;
	}

	if (clk_set_parent(clk->clk_pll_com_audio, clk->clk_pll_com)) {
		SND_LOG_ERR("set parent of clk_pll_com_audio to clk_pll_com failed\n");
		ret = -EINVAL;
		goto err_set_parent;
	}
	if (clk_set_parent(clk->clk_i2s, clk->clk_pll_audio)) {
		SND_LOG_ERR("set parent of clk_i2s to pll_audio failed\n");
		ret = -EINVAL;
		goto err_set_parent;
	}

	if (clk_set_rate(clk->clk_pll_audio, 98304000)) {		/* 24.576M * n */
		SND_LOG_ERR("set clk_pll_audio rate failed\n");
		ret = -EINVAL;
		goto err_set_rate;
	}
	if (clk_set_rate(clk->clk_pll_com, 451584000)) {
		SND_LOG_ERR("set clk_pll_com rate failed\n");
		ret = -EINVAL;
		goto err_set_rate;
	}
	if (clk_set_rate(clk->clk_pll_com_audio, 90316800)) {	/* 22.5792M * n */
		SND_LOG_ERR("set clk_pll_com_audio rate failed\n");
		ret = -EINVAL;
		goto err_set_rate;
	}

	return 0;

err_set_rate:
err_set_parent:
	clk_put(clk->clk_i2s);
err_get_clk_i2s:
	clk_put(clk->clk_pll_com_audio);
err_get_clk_pll_com_audio:
	clk_put(clk->clk_pll_com);
err_get_clk_pll_com:
	clk_put(clk->clk_pll_audio);
err_get_clk_pll_audio:
	clk_put(clk->clk_bus);
err_get_clk_bus:
err_get_clk_rst:
	return ret;
}

static inline void snd_sunxi_clk_exit(struct sunxi_i2s_clk *clk)
{
	SND_LOG_DEBUG("\n");

	clk_put(clk->clk_i2s);
	clk_put(clk->clk_pll_com_audio);
	clk_put(clk->clk_pll_com);
	clk_put(clk->clk_pll_audio);
	clk_put(clk->clk_bus);
}

static inline int snd_sunxi_clk_bus_enable(struct sunxi_i2s_clk *clk)
{
	int ret = 0;

	SND_LOG_DEBUG("\n");

	if (reset_control_deassert(clk->clk_rst)) {
		SND_LOG_ERR("clk rst deassert failed\n");
		ret = -EINVAL;
		goto err_deassert_rst;
	}

	if (clk_prepare_enable(clk->clk_bus)) {
		SND_LOG_ERR("clk bus enable failed\n");
		ret = -EINVAL;
		goto err_enable_clk_bus;
	}

	return 0;

err_enable_clk_bus:
	reset_control_assert(clk->clk_rst);
err_deassert_rst:
	return ret;
}

static inline int snd_sunxi_clk_enable(struct sunxi_i2s_clk *clk)
{
	int ret = 0;

	SND_LOG_DEBUG("\n");

	if (clk_prepare_enable(clk->clk_pll_audio)) {
		SND_LOG_ERR("pll_audio enable failed\n");
		ret = -EINVAL;
		goto err_enable_clk_pll_audio;
	}
	if (clk_prepare_enable(clk->clk_pll_com)) {
		SND_LOG_ERR("clk_pll_com enable failed\n");
		ret = -EINVAL;
		goto err_enable_clk_pll_com;
	}
	if (clk_prepare_enable(clk->clk_pll_com_audio)) {
		SND_LOG_ERR("clk_pll_com_audio enable failed\n");
		ret = -EINVAL;
		goto err_enable_clk_pll_com_audio;
	}

	if (clk_prepare_enable(clk->clk_i2s)) {
		SND_LOG_ERR("clk_i2s enable failed\n");
		ret = -EINVAL;
		goto err_enable_clk_i2s;
	}

	return 0;

err_enable_clk_i2s:
	clk_disable_unprepare(clk->clk_pll_com_audio);
err_enable_clk_pll_com_audio:
	clk_disable_unprepare(clk->clk_pll_com);
err_enable_clk_pll_com:
	clk_disable_unprepare(clk->clk_pll_audio);
err_enable_clk_pll_audio:
	return ret;
}


static inline void snd_sunxi_clk_bus_disable(struct sunxi_i2s_clk *clk)
{
	SND_LOG_DEBUG("\n");

	clk_disable_unprepare(clk->clk_bus);
	reset_control_assert(clk->clk_rst);
}

static inline void snd_sunxi_clk_disable(struct sunxi_i2s_clk *clk)
{
	SND_LOG_DEBUG("\n");

	clk_disable_unprepare(clk->clk_i2s);
	clk_disable_unprepare(clk->clk_pll_com_audio);
	clk_disable_unprepare(clk->clk_pll_com);
	clk_disable_unprepare(clk->clk_pll_audio);
}

static inline int snd_sunxi_clk_rate(struct sunxi_i2s_clk *clk, unsigned int freq_in,
				     unsigned int freq_out)
{
	SND_LOG_DEBUG("\n");

	if (clk_set_rate(clk->clk_i2s, freq_out)) {
		SND_LOG_ERR("set clk_i2s rate failed, rate: %u\n", freq_out);
		return -EINVAL;
	}

	return 0;
}

#endif /* __SND_SUN50IW10_I2S_H */
