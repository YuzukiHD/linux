/*
 * Copyright (C) 2019 Allwinner Technology Limited. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Author: Albert Yu <yuxyun@allwinnertech.com>
 */

#include <sunxi-log.h>
#include <linux/ioport.h>
#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_config.h>
#include "mali_kbase_config_platform.h"

#include "platform.h"
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>

#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>

#if defined(CONFIG_ARCH_SUN55IW3)
#define SMC_REG_BASE 0x03110000
#else
#error "Need configure SMC for GPU"
#endif
#define SMC_GPU_DRM_REG (SMC_REG_BASE + 0x54)

#define SUNXI_BAK_CLK_RATE 648000000

static struct sunxi_data *sunxi_data;
static struct kbase_device *s_kbdev;

void kbase_pm_get_dvfs_metrics(struct kbase_device *kbdev,
			       struct kbasep_pm_metrics *last,
			       struct kbasep_pm_metrics *diff);

static inline void ioremap_regs(void)
{
	struct reg *p_drm;

	p_drm = &sunxi_data->regs.drm;
	p_drm->phys = SMC_GPU_DRM_REG;
	p_drm->ioaddr = ioremap(p_drm->phys, 4);
}

static inline void iounmap_regs(void)
{
	iounmap(sunxi_data->regs.drm.ioaddr);
}

static void enable_clks_wrap(struct kbase_device *kbdev)
{
	mutex_lock(&sunxi_data->sunxi_lock);
	clk_prepare_enable(kbdev->clocks[1]);
	mutex_unlock(&sunxi_data->sunxi_lock);
}

static void disable_clks_wrap(struct kbase_device *kbdev)
{
	mutex_lock(&sunxi_data->sunxi_lock);
	clk_disable_unprepare(kbdev->clocks[1]);
	mutex_unlock(&sunxi_data->sunxi_lock);
}

int sunxi_chang_freq_safe(struct kbase_device *kbdev, unsigned long *freq, unsigned long u_volt)
{
	int ret = 0;

#if defined(CONFIG_ARCH_SUN55IW3)
	// increase frequency
	if (sunxi_data->current_freq < *freq) {
		// increase voltage first when up frequency
		if (sunxi_data->independent_power && u_volt != sunxi_data->current_u_volt) {
#ifdef CONFIG_REGULATOR
			ret = regulator_set_voltage(kbdev->regulators[0], u_volt, u_volt);
			if (ret < 0) {
				sunxi_err(kbdev->dev, "set gpu regulators err %d!\n", ret);
				return ret;
			}
#endif
			sunxi_data->current_u_volt = u_volt;
#ifdef CONFIG_MALI_DEVFREQ
			kbdev->current_voltages[0] = u_volt;
#endif
		}
		/* then set frequency. No need to dynamic change gpu-pll
		 * frequency, otherwise will cause the gpu-pll is not linear
		 * frequency tune. The kbdev->clocks[0] is gpu-pll.
		*/

		/*
		ret = clk_set_rate(kbdev->clocks[0], *freq);
		if (ret < 0) {
			sunxi_err(kbdev->dev, "set pll_gpu to %ld err %d!\n", *freq, ret);
			return ret;
		}
		*/

		ret = clk_set_rate(kbdev->clocks[1], *freq);
		if (ret < 0) {
			sunxi_err(kbdev->dev, "set gpu core clock to %ld err %d!\n", *freq, ret);
			return ret;
		}
		sunxi_data->current_freq = *freq;
	} else {
		/* decrease frequency first when down frequency, then set
		 * voltage. No need to dynamic change gpu-pll frequency,
		 * otherwise will cause the gpu-pll is not linear frequency
		 * tune. The kbdev->clocks[0] is gpu-pll.
		*/

		/*
		ret = clk_set_rate(kbdev->clocks[0], *freq);
		if (ret < 0) {
			sunxi_err(kbdev->dev, "set pll_gpu to %ld err %d!\n", *freq, ret);
			return ret;
		}
		*/

		ret = clk_set_rate(kbdev->clocks[1], *freq);
		if (ret < 0) {
			sunxi_err(kbdev->dev, "set gpu core clock to %ld err %d!\n", *freq, ret);
			return ret;
		}
		sunxi_data->current_freq = *freq;
		// decrease voltage
		if (sunxi_data->independent_power && u_volt != sunxi_data->current_u_volt) {
#ifdef CONFIG_REGULATOR
			ret = regulator_set_voltage(kbdev->regulators[0], u_volt, u_volt);
			if (ret < 0) {
				sunxi_err(kbdev->dev, "set gpu regulators err %d!\n", ret);
				return ret;
			}
#endif
			sunxi_data->current_u_volt = u_volt;
#ifdef CONFIG_MALI_DEVFREQ
			kbdev->current_voltages[0] = u_volt;
#endif
		}
	}
#endif

	return ret;
}

#ifdef CONFIG_MALI_DEVFREQ
int sunxi_dvfs_target(struct kbase_device *kbdev, unsigned long *freq, unsigned long u_volt)
{
	mutex_lock(&sunxi_data->sunxi_lock);
	if (sunxi_data->man_ctrl || !sunxi_data->dvfs_ctrl || sunxi_data->sence_ctrl) {
		*freq = kbdev->current_nominal_freq;
		mutex_unlock(&sunxi_data->sunxi_lock);
		return -ENODEV;
	}

	sunxi_chang_freq_safe(kbdev, freq, u_volt);
	mutex_unlock(&sunxi_data->sunxi_lock);

	return 0;
}
#endif

static int parse_dts_and_fex(struct kbase_device *kbdev, struct sunxi_data *sunxi_data)
{
#ifdef CONFIG_OF
	u32 val;
	int err;
	err = of_property_read_u32(kbdev->dev->of_node, "gpu_idle", &val);
	if (!err)
		sunxi_data->idle_ctrl = val ? true : false;
	err = of_property_read_u32(kbdev->dev->of_node, "independent_power", &val);
	if (!err)
		sunxi_data->independent_power = val ? true : false;
	err = of_property_read_u32(kbdev->dev->of_node, "dvfs_status", &val);
	if (!err)
		sunxi_data->dvfs_ctrl = val ? true : false;
#endif /* CONFIG_OF */

	return 0;
}

static ssize_t scene_ctrl_cmd_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", sunxi_data->sence_ctrl);
}

static ssize_t scene_ctrl_cmd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err;
	unsigned long val;
	unsigned long volt;
	enum scene_ctrl_cmd cmd;
#if defined(CONFIG_PM_OPP)
	struct dev_pm_opp *opp;
#endif
	err = kstrtoul(buf, 10, &val);
	if (err) {
		sunxi_err(dev, "scene_ctrl_cmd_store gets a invalid parameter!\n");
		return count;
	}

	cmd = (enum scene_ctrl_cmd)val;
	switch (cmd) {
	case SCENE_CTRL_NORMAL_MODE:
		sunxi_data->sence_ctrl = 0;
		break;

	case SCENE_CTRL_PERFORMANCE_MODE:
		sunxi_data->sence_ctrl = 1;
		val = sunxi_data->max_freq;
		volt = sunxi_data->max_u_volt;
#if defined(CONFIG_PM_OPP)
		/* dev_pm_opp_find_freq_floor will call dev_pm_opp_get(),
		 * whilh increment the reference count of OPP.
		 * So, The callers are required to call dev_pm_opp_put()
		 * for the returned OPP after use.
		 */
		opp = dev_pm_opp_find_freq_floor(dev, &val);
		if (!IS_ERR_OR_NULL(opp)) {
			volt = dev_pm_opp_get_voltage(opp);
			dev_pm_opp_put(opp);
		} else {
			val = sunxi_data->max_freq;
			volt = sunxi_data->max_u_volt;
		}
#endif
		mutex_lock(&sunxi_data->sunxi_lock);
		sunxi_chang_freq_safe(s_kbdev, &val, volt);
		mutex_unlock(&sunxi_data->sunxi_lock);
#ifdef CONFIG_MALI_DEVFREQ
		s_kbdev->current_nominal_freq = val;
#endif
		break;

	default:
		sunxi_err(dev, "invalid scene control command %d!\n", cmd);
		return count;
	}

	return count;
}

static DEVICE_ATTR(scene_ctrl, 0660,
		scene_ctrl_cmd_show, scene_ctrl_cmd_store);

/* read gpu interal registers's value */
static u32 sunxi_kbase_reg_read(struct kbase_device *kbdev, u32 offset)
{
	u32 val;

	val = readl(kbdev->reg + offset);

	return val;
}

/**
 * sunxi_gpu_show - Show callback for the sunxi_gpu_info sysfs entry.
 * @dev:  The device this sysfs file is for.
 * @attr: The attributes of the sysfs file.
 * @buf:  The output buffer to receive the GPU information.
 *
 * This function is called to show the current gpu info(dvfs\voltage\frequency).
 *
 * Return: The number of bytes output to @buf.
 */
static ssize_t sunxi_gpu_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t total = 0;
	size_t max_size = PAGE_SIZE;

	struct kbasep_pm_metrics diff;
	kbase_pm_get_dvfs_metrics(s_kbdev, &sunxi_data->sunxi_last, &diff);
#ifdef CONFIG_REGULATOR
	if (!IS_ERR_OR_NULL(s_kbdev->regulators[0])) {
		int vol = regulator_get_voltage(s_kbdev->regulators[0]);
		total += scnprintf(buf + total, max_size - total, "voltage:%dmV;\n", vol / 1000);
	}
#endif /* CONFIG_REGULATOR */

	total += scnprintf(buf + total, max_size - total, "idle:%s;\n",
			sunxi_data->idle_ctrl ? "on" : "off");
	total += scnprintf(buf + total, max_size - total, "scenectrl:%s;\n",
			sunxi_data->sence_ctrl ? "on" : "off");
	total += scnprintf(buf + total, max_size - total, "dvfs:%s;\n",
			sunxi_data->dvfs_ctrl ? "on" : "off");
	total += scnprintf(buf + total, max_size - total, "independent_power:%s;\n",
			sunxi_data->independent_power ? "yes" : "no");
	total += scnprintf(buf + total, max_size - total, "Frequency:%luMHz;\n",
			sunxi_data->current_freq/1000/1000);
	total += scnprintf(buf + total, max_size - total, "Utilisation from last show:%u%%;\n",
		diff.time_busy * 100 / (diff.time_busy + diff.time_idle));

	total += scnprintf(buf + total, max_size - total, "\nRegister state: \n");
	total += scnprintf(buf + total, max_size - total, "GPU_IRQ_RAWSTAT=0x%08x"
		"	GPU_STATUS=0x%08x\n",
		sunxi_kbase_reg_read(s_kbdev, GPU_CONTROL_REG(GPU_IRQ_RAWSTAT)),
		sunxi_kbase_reg_read(s_kbdev, GPU_CONTROL_REG(GPU_STATUS)));
	total += scnprintf(buf + total, max_size - total, "JOB_IRQ_RAWSTAT=0x%08x"
		"	JOB_IRQ_JS_STATE=0x%08x\n",
		sunxi_kbase_reg_read(s_kbdev, GPU_CONTROL_REG(JOB_IRQ_RAWSTAT)),
		sunxi_kbase_reg_read(s_kbdev, GPU_CONTROL_REG(JOB_IRQ_JS_STATE)));

	for (i = 0; i < 3; i++) {
		total += scnprintf(buf + total, max_size - total, "JS%d_STATUS=0x%08x"
				"	JS%d_HEAD_LO=0x%08x\n",
				i, sunxi_kbase_reg_read(s_kbdev, JOB_SLOT_REG(i, JS_STATUS)),
				i, sunxi_kbase_reg_read(s_kbdev, JOB_SLOT_REG(i, JS_HEAD_LO)));
	}

	total += scnprintf(buf + total, max_size - total, "MMU_IRQ_RAWSTAT=0x%08x"
		"	GPU_FAULTSTATUS=0x%08x\n",
		sunxi_kbase_reg_read(s_kbdev, MMU_REG(MMU_IRQ_RAWSTAT)),
		sunxi_kbase_reg_read(s_kbdev, GPU_CONTROL_REG(GPU_FAULTSTATUS)));
	total += scnprintf(buf + total, max_size - total, "GPU_IRQ_MASK=0x%08x"
		"	JOB_IRQ_MASK=0x%08x	MMU_IRQ_MASK=0x%08x\n",
		sunxi_kbase_reg_read(s_kbdev, GPU_CONTROL_REG(GPU_IRQ_MASK)),
		sunxi_kbase_reg_read(s_kbdev, JOB_CONTROL_REG(JOB_IRQ_MASK)),
		sunxi_kbase_reg_read(s_kbdev, MMU_REG(MMU_IRQ_MASK)));
	total += scnprintf(buf + total, max_size - total, "PWR_OVERRIDE0=0x%08x"
		"	PWR_OVERRIDE1=0x%08x\n",
		sunxi_kbase_reg_read(s_kbdev, GPU_CONTROL_REG(PWR_OVERRIDE0)),
		sunxi_kbase_reg_read(s_kbdev, GPU_CONTROL_REG(PWR_OVERRIDE1)));
	total += scnprintf(buf + total, max_size - total, "SHADER_CONFIG=0x%08x"
		"	L2_MMU_CONFIG=0x%08x\n",
		sunxi_kbase_reg_read(s_kbdev, GPU_CONTROL_REG(SHADER_CONFIG)),
		sunxi_kbase_reg_read(s_kbdev, GPU_CONTROL_REG(L2_MMU_CONFIG)));
	total += scnprintf(buf + total, max_size - total, "TILER_CONFIG=0x%08x"
		"	JM_CONFIG=0x%08x\n",
		sunxi_kbase_reg_read(s_kbdev, GPU_CONTROL_REG(TILER_CONFIG)),
		sunxi_kbase_reg_read(s_kbdev, GPU_CONTROL_REG(JM_CONFIG)));

	return total;
}
static DEVICE_ATTR_RO(sunxi_gpu_info);

/**
 * sunxi_gpu_freq_show - Show callback for the sunxi_gpu_freq sysfs entry.
 * @dev:  The device this sysfs file is for.
 * @attr: The attributes of the sysfs file.
 * @buf:  The output buffer to receive the GPU information.
 *
 * This function is called to show the current gpu frequency.
 *
 * Return: The number of bytes output to @buf.
 */
static ssize_t sunxi_gpu_freq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t total = 0;
	size_t max_size = PAGE_SIZE;
	struct kbasep_pm_metrics diff;

	kbase_pm_get_dvfs_metrics(s_kbdev, &sunxi_data->sunxi_last, &diff);
	total += scnprintf(buf + total, max_size - total, "Frequency:%luMHz;\n",
			sunxi_data->current_freq/1000/1000);
	total += scnprintf(buf + total, max_size - total, "Utilisation from last show:%u%%;\n",
		diff.time_busy * 100 / (diff.time_busy + diff.time_idle));

	return total;
}

/**
 * sunxi_gpu_freq_store - Store callback for the sunxi_gpu_freq sysfs file.
 * @dev:   The device with sysfs file is for
 * @attr:  The attributes of the sysfs file
 * @buf:   The value written to the sysfs file
 * @count: The number of bytes written to the sysfs file
 *
 * This function is called when the sunxi_gpu_info/freq sysfs file is written to.
 * It checks the data written, and if valid updates the gpu dvfs frequency variable
 *
 * Return: @count if the function succeeded. An error code on failure.
 */
static ssize_t sunxi_gpu_freq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long freq_MHz;
	struct kbase_device *kbdev;
	static unsigned long man_freq; /* in Hz */
	static unsigned long man_u_volt; /* in uV */
#if defined(CONFIG_PM_OPP)
	struct dev_pm_opp *opp;
#endif

	kbdev = s_kbdev;
	if (!kbdev)
		return -ENODEV;

	ret = kstrtoul(buf, 0, &freq_MHz);
	if (ret || freq_MHz <= 0) {
		sunxi_err(kbdev->dev, "Couldn't process sunxi_gpu_freq write operation.\n"
				"Use format <freq_MHz>\n");
		return -EINVAL;
	}

	man_freq = freq_MHz * 1000 * 1000;
	man_u_volt = sunxi_data->max_u_volt;

	/* If gpu use independent power-supply, driver can change gpu's voltage */
	if (sunxi_data->independent_power) {
#if defined(CONFIG_PM_OPP)
		opp = dev_pm_opp_find_freq_floor(dev, &man_freq);
		if (!IS_ERR_OR_NULL(opp)) {
			man_u_volt = dev_pm_opp_get_voltage(opp);
			dev_pm_opp_put(opp);
		}
#endif
	} else {
		man_u_volt = sunxi_data->current_u_volt;
	}

	/* If gpu doesn't use independent_power, sunxi_chang_freq_safe will not
	 * change the gpu's voltage; we don't care man_u_volt value.
	 */
	mutex_lock(&sunxi_data->sunxi_lock);
	sunxi_chang_freq_safe(kbdev, &man_freq, man_u_volt);
	mutex_unlock(&sunxi_data->sunxi_lock);

	/* If you manually change the GPU frequency, dvfs will be disabled */
	sunxi_data->dvfs_ctrl = 0;

	return count;
}
static DEVICE_ATTR_RW(sunxi_gpu_freq);

/**
 * sunxi_gpu_volt_show - Show callback for the sunxi_gpu_volt sysfs entry.
 * @dev:  The device this sysfs file is for.
 * @attr: The attributes of the sysfs file.
 * @buf:  The output buffer to receive the GPU information.
 *
 * This function is called to show the current gpu voltage.
 *
 * Return: The number of bytes output to @buf.
 */
static ssize_t sunxi_gpu_volt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%ldmv\n", sunxi_data->current_u_volt / 1000);
}

/**
 * sunxi_gpu_volt_store - Store callback for the sunxi_gpu_volt sysfs file.
 * @dev:   The device with sysfs file is for
 * @attr:  The attributes of the sysfs file
 * @buf:   The value written to the sysfs file
 * @count: The number of bytes written to the sysfs file
 *
 * This function is called when the sunxi_gpu_info/freq sysfs file is written to.
 * It checks the data written, and if valid updates the gpu dvfs frequency variable
 *
 * Return: @count if the function succeeded. An error code on failure.
 */
static ssize_t sunxi_gpu_volt_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long volt_mV;
	struct kbase_device *kbdev;
	static unsigned long man_u_volt; /* in uV */

	kbdev = s_kbdev;
	if (!kbdev)
		return -ENODEV;

	if (!sunxi_data->independent_power) {
		sunxi_err(kbdev->dev, "GPU not support change voltage!!!\n");
		return -EPERM;
	}

	ret = kstrtoul(buf, 0, &volt_mV);
	if (ret || volt_mV <= 0) {
		sunxi_err(kbdev->dev, "Couldn't process sunxi_gpu_volt write operation.\n"
				"Use format <volt_mV>\n");
		return -EINVAL;
	}

	man_u_volt = volt_mV * 1000;

#ifdef CONFIG_REGULATOR
	mutex_lock(&sunxi_data->sunxi_lock);
	ret = regulator_set_voltage(kbdev->regulators[0], man_u_volt, man_u_volt);
	if (ret < 0) {
		sunxi_err(kbdev->dev, "sunxi_gpu_volt set gpu voltage err %d!\n", ret);
		mutex_unlock(&sunxi_data->sunxi_lock);
		return ret;
	}
	mutex_unlock(&sunxi_data->sunxi_lock);
#endif

	sunxi_data->current_u_volt = man_u_volt;
#ifdef CONFIG_MALI_DEVFREQ
	kbdev->current_voltages[0] = man_u_volt;
#endif
	/* If you manually change the GPU voltage, dvfs will be disabled */
	sunxi_data->dvfs_ctrl = 0;

	return count;
}
static DEVICE_ATTR_RW(sunxi_gpu_volt);

/**
 * sunxi_gpu_dvfs_show - Show callback for the sunxi_gpu_dvfs sysfs entry.
 * @dev:  The device this sysfs file is for.
 * @attr: The attributes of the sysfs file.
 * @buf:  The output buffer to receive the GPU information.
 *
 * This function is called to show the current gpu dvfs_ctrl status.
 *
 * Return: The number of bytes output to @buf.
 */
static ssize_t sunxi_gpu_dvfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sunxi_data->dvfs_ctrl);
}

/**
 * sunxi_gpu_dvfs_store - Store callback for the sunxi_gpu_dvfs sysfs file.
 * @dev:   The device with sysfs file is for
 * @attr:  The attributes of the sysfs file
 * @buf:   The value written to the sysfs file
 * @count: The number of bytes written to the sysfs file
 *
 * This function is called when the sunxi_gpu_info/freq sysfs file is written to.
 * It checks the data written, and if valid updates the gpu dvfs_ctrl variable
 *
 * Return: @count if the function succeeded. An error code on failure.
 */
static ssize_t sunxi_gpu_dvfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int dvfs_ctrl;
	struct kbase_device *kbdev;

	kbdev = s_kbdev;
	if (!kbdev)
		return -ENODEV;

	ret = kstrtoint(buf, 0, &dvfs_ctrl);
	if (ret || dvfs_ctrl < 0) {
		sunxi_err(kbdev->dev, "Couldn't process sunxi_gpu_dvfs write operation.\n"
				"Use format <bool>\n");
		return -EINVAL;
	}

	sunxi_data->dvfs_ctrl = dvfs_ctrl ? true : false;

	return count;
}
static DEVICE_ATTR_RW(sunxi_gpu_dvfs);

static struct attribute *sunxi_gpu_attributes[] = {
	&dev_attr_sunxi_gpu_info.attr,
	&dev_attr_scene_ctrl.attr,
	&dev_attr_sunxi_gpu_freq.attr,
	&dev_attr_sunxi_gpu_volt.attr,
	&dev_attr_sunxi_gpu_dvfs.attr,
	NULL
};

static struct attribute_group sunxi_gpu_attribute_group = {
	.name = "sunxi_gpu",
	.attrs = sunxi_gpu_attributes,
};

int sunxi_platform_init(struct kbase_device *kbdev)
{
#if defined(CONFIG_PM_OPP)
	struct dev_pm_opp *opp;
#endif
	unsigned long freq = SUNXI_BAK_CLK_RATE;
	unsigned long u_volt = 950000;

	sunxi_data = (struct sunxi_data *)kzalloc(sizeof(struct sunxi_data), GFP_KERNEL);
	if (IS_ERR_OR_NULL(sunxi_data)) {
		sunxi_err(kbdev->dev, "sunxi init gpu Failed to malloc memory.\n");
		return -ENOMEM;
	}
	sunxi_data->dvfs_ctrl = true;
	sunxi_data->idle_ctrl = true;
	sunxi_data->independent_power = false;
	sunxi_data->power_on = false;
	sunxi_data->clk_on = false;
	parse_dts_and_fex(kbdev, sunxi_data);
#ifdef CONFIG_REGULATOR
	if (IS_ERR_OR_NULL(kbdev->regulators[0])) {
		sunxi_data->independent_power = 0;
	}
#else
	sunxi_data->independent_power = 0;
#endif

#if !defined(CONFIG_PM_OPP)
	clk_set_rate(kbdev->clocks[0], SUNXI_BAK_CLK_RATE);
	clk_set_rate(kbdev->clocks[1], SUNXI_BAK_CLK_RATE);
#else
	freq = ULONG_MAX;
	opp = dev_pm_opp_find_freq_floor(kbdev->dev, &freq);
	if (IS_ERR_OR_NULL(opp)) {
		sunxi_err(kbdev->dev, "sunxi init gpu Failed to get opp (%ld)\n", PTR_ERR(opp));
		freq = SUNXI_BAK_CLK_RATE;
	} else {
		u_volt = dev_pm_opp_get_voltage(opp);

		/* dev_pm_opp_find_freq_floor will call dev_pm_opp_get(),
		 * whilh increment the reference count of OPP.
		 * So, The callers are required to call dev_pm_opp_put()
		 * for the returned OPP after use.
		 */
		dev_pm_opp_put(opp);
	}

#ifdef CONFIG_REGULATOR
	if (sunxi_data->independent_power) {
		if (regulator_set_voltage(kbdev->regulators[0], u_volt, u_volt) < 0)
			sunxi_err(kbdev->dev, "sunxi init set gpu voltage err.\n");
	}
	if (kbdev->regulators[0])
		u_volt = regulator_get_voltage(kbdev->regulators[0]);
#endif
	clk_set_rate(kbdev->clocks[0], freq);
	clk_set_rate(kbdev->clocks[1], freq);
#endif

	sunxi_data->reset = devm_reset_control_get(kbdev->dev, NULL);
	if (IS_ERR_OR_NULL(sunxi_data->reset)) {
		sunxi_info(kbdev->dev, "sunxi init gpu Failed to get reset ctrl\n");
	}

	sunxi_data->bus_clk = of_clk_get(kbdev->dev->of_node, 2);
	if (!IS_ERR_OR_NULL(sunxi_data->bus_clk)) {
		clk_prepare_enable(sunxi_data->bus_clk);
	} else {
		sunxi_info(kbdev->dev, "sunxi init gpu Failed to get bus_clk \n");
	}

	sunxi_data->max_freq = freq;
	sunxi_data->max_u_volt = u_volt;
	ioremap_regs();
	mutex_init(&sunxi_data->sunxi_lock);

#if defined(CONFIG_ARCH_SUN55IW3)
	// 1890 need use pm_runtime framework to let
	// power domain control poweron or poweroff of gpu
	pm_runtime_enable(kbdev->dev);
	// When use 1890 power domain, you need enable clk and
	// deassert gpu reset in gpu initialization flow.
	// And then power domain will auto control gpu clk and reset,
	// when poweron or poweroff gpu.
	enable_clks_wrap(kbdev);
	reset_control_deassert(sunxi_data->reset);
#endif

#ifdef CONFIG_MALI_DEVFREQ
	kbdev->current_nominal_freq = freq;
	kbdev->current_voltages[0] = u_volt;
#endif
	sunxi_data->current_freq = freq;
	sunxi_data->current_u_volt = u_volt;

	clk_set_parent(kbdev->clocks[1], kbdev->clocks[0]);

	if (sysfs_create_group(&kbdev->dev->kobj, &sunxi_gpu_attribute_group)) {
		sunxi_err(kbdev->dev, "sunxi sysfs group creation failed!\n");
	}

	s_kbdev = kbdev;

#if defined(CONFIG_ARCH_SUN55IW3)
	sunxi_info(kbdev->dev, "[%ldmv-%ldMHz] inde_power:%d idle:%d dvfs:%d\n",
		u_volt/1000, freq/1000/1000,
		sunxi_data->independent_power, sunxi_data->idle_ctrl, sunxi_data->dvfs_ctrl);
#endif

	return 0;
}

void sunxi_platform_term(struct kbase_device *kbdev)
{
	sysfs_remove_group(&kbdev->dev->kobj, &sunxi_gpu_attribute_group);

#if defined(CONFIG_ARCH_SUN55IW3)
	pm_runtime_disable(kbdev->dev);
	disable_clks_wrap(kbdev);
	if (sunxi_data->bus_clk)
		clk_disable_unprepare(sunxi_data->bus_clk);
	reset_control_deassert(sunxi_data->reset);
#endif

	iounmap_regs();
	mutex_destroy(&sunxi_data->sunxi_lock);
	kfree(sunxi_data);

	sunxi_data = NULL;
}

struct kbase_platform_funcs_conf sunxi_platform_conf = {
	.platform_init_func = sunxi_platform_init,
	.platform_term_func = sunxi_platform_term,
	.platform_late_init_func = NULL,
	.platform_late_term_func = NULL,
#if !MALI_USE_CSF
	.platform_handler_context_init_func = NULL,
	.platform_handler_context_term_func = NULL,
	.platform_handler_atom_submit_func = NULL,
	.platform_handler_atom_complete_func = NULL
#endif
};

static int sunxi_protected_mode_enable(struct protected_mode_device *pdev)
{
	u32 val;
	val = readl(sunxi_data->regs.drm.ioaddr);
	val |= 1;
	writel(val, sunxi_data->regs.drm.ioaddr);

	return 0;
}

static int sunxi_protected_mode_disable(struct protected_mode_device *pdev)
{
	u32 val;
	val = readl(sunxi_data->regs.drm.ioaddr);
	val &= ~1;
	writel(val, sunxi_data->regs.drm.ioaddr);

	return 0;
}

struct protected_mode_ops sunxi_protected_ops = {
	.protected_mode_enable = sunxi_protected_mode_enable,
	.protected_mode_disable = sunxi_protected_mode_disable
};

static int sunxi_pm_callback_power_on(struct kbase_device *kbdev)
{
#if defined(CONFIG_ARCH_SUN55IW3)
	int error;
#endif

	if (sunxi_data->power_on)
		return 1;
#if defined(CONFIG_ARCH_SUN55IW3)
	error = pm_runtime_get_sync(kbdev->dev);
	if (error == 1)
		return 0;
#else
#error "Need enable regulator for GPU"
#endif
	sunxi_data->power_on = true;

	return 1;
}

static void sunxi_pm_callback_power_off(struct kbase_device *kbdev)
{
	if (!sunxi_data->power_on)
		return;
	sunxi_data->power_on = false;

#if defined(CONFIG_ARCH_SUN55IW3)
	pm_runtime_mark_last_busy(kbdev->dev);
	pm_runtime_put_autosuspend(kbdev->dev);
#else
#error "Need enable regulator for GPU"
#endif
}

struct kbase_pm_callback_conf sunxi_pm_callbacks = {
	.power_on_callback = sunxi_pm_callback_power_on,
	.power_off_callback = sunxi_pm_callback_power_off,
	.power_suspend_callback  = NULL,
	.power_resume_callback = NULL
};
