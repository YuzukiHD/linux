// SPDX-License-Identifier: GPL-2.0-or-later
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
#include "it6807.h"
#ifdef CONFIG_IT6807_HDMI_VBYONE_DEBUG
#include "dev_ite_debugfs.h"
#endif

#define IT6807_NAME ("it6807")
#define IT6807_MATCH_DTS_EN 1

struct regulator *hdmi_3v3;
struct regulator *it6807_3v3;
struct i2c_client *this_client;
IT6807_CONFIG_INFO it6807_config_info;

_iTE6805_DATA iTE6805_DATA;
_iTE6805_VTiming iTE6805_CurVTiming;
_iTE6805_PARSE3D_STR iTE6805_EDID_Parse3D;
extern iTE_u8 iTE6805_CurrentStage;

iTE_u1 i2c_write_byte(iTE_u8 address, iTE_u8 offset, iTE_u8 byteno,
					  iTE_u8 *p_data, iTE_u8 device)
{
	iTE_u8 i;
	int ret;
	struct i2c_adapter *adap = this_client->adapter;
	struct i2c_msg msg;
	unsigned char data[256];

	if (!adap) {
		printinfo("adap is null !");
		return 0;
	}
	data[0] = offset;
	if (byteno > 1) {
		for (i = 0; i < byteno - 1; i++) {
			data[i + 1] = p_data[i];
		}
	} else {
		data[1] = p_data[0];
	}

	msg.addr = address >> 1;
	msg.flags = 0;
	msg.len = byteno + 1;
	msg.buf = data;

	ret = i2c_transfer(adap, &msg, 1);

	if (ret >= 0) {
		return 1;
	} else {
		printinfo("error! slave = 0x%x, addr = 0x%2x\n ", address >> 1, offset);
	}

	return 0;
}

iTE_u1 i2c_read_byte(iTE_u8 address, iTE_u8 offset, iTE_u8 byteno,
					 iTE_u8 *p_data, iTE_u8 device)
{
	iTE_u8 i;
	int ret;
	struct i2c_adapter *adap = this_client->adapter;
	struct i2c_msg msg[2];
	unsigned char data[256];

	if (!adap) {
		printinfo("adap is null !");
		return 0;
	}
	data[0] = offset;

	/*
	 * Send out the register address...
	 */
	msg[0].addr = address >> 1;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &data[0];
	/*
	 * ...then read back the result.
	 */
	msg[1].addr = address >> 1;
	msg[1].flags = I2C_M_RD;
	msg[1].len = byteno;
	msg[1].buf = &data[1];

	ret = i2c_transfer(adap, msg, 2);
	if (ret >= 0) {
		if (byteno > 1) {
			for (i = 0; i < byteno - 1; i++) {
				p_data[i] = data[i + 1];
			}
		} else {
			p_data[0] = data[1];
		}
		return 1;
	} else {
		printinfo("error! slave = 0x%x, addr = 0x%2x\n ", address >> 1, offset);
	}
	return 0;
}

static const struct i2c_device_id it6807_id[] = {{"it6807_0", 0}, {}};
MODULE_DEVICE_TABLE(i2c, it6807_id);

static struct of_device_id it6807_dt_ids[] = {
	{
		.compatible = "allwinner,it6807_0",
	},
	{},
};
MODULE_DEVICE_TABLE(of, it6807_dt_ids);

static struct task_struct *it6807_fsm_task;

static int it6807_fsm_thread(void *data)
{
	while (!kthread_should_stop()) {
		/* 		printinfo_s("it6807_fsm_thread start ..\n"); */
		iTE6805_FSM();
		msleep(50);
		/* 		printinfo_s("it6807_fsm_thread end ..\n"); */
	}
	return 0;
}

static int script_it6807_gpio_init(const struct i2c_client *client)
{
	struct device_node *np = NULL;
	struct gpio_config config;

	np = of_find_node_by_name(NULL, "it6807");
	if (!np) {
		dev_err(&client->dev, "ERROR! get it6807_para failed\n");
		return -1;
	}

	if (!of_device_is_available(np)) {
		dev_err(&client->dev, "it6807 is not used\n");
		return -1;
	} else {
		it6807_config_info.it6807_used = 1;
	}

	it6807_config_info.reset_gpio = of_get_named_gpio_flags(
		np, "it6807_reset", 0, (enum of_gpio_flags *)&config);
	if (!gpio_is_valid(it6807_config_info.reset_gpio)) {
		dev_err(&client->dev, "it6807_reset_gpio is invalid.\n");
		return -1;
	} else {
		if (0 != gpio_request(it6807_config_info.reset_gpio, NULL)) {
			dev_err(&client->dev, "reset_gpio_request is failed\n");
			return -1;
		}
		if (0 != gpio_direction_output(it6807_config_info.reset_gpio, 1)) {
			dev_err(&client->dev, "it6807_reset_gpio set err!\n");
			return -1;
		}
	}

	it6807_config_info.backlight_gpio = of_get_named_gpio_flags(
		np, "it6807_backlight", 0, (enum of_gpio_flags *)&config);
	if (!gpio_is_valid(it6807_config_info.backlight_gpio)) {
		dev_info(&client->dev, "backlight_gpio is invalid.\n");
		/* return -1; */
	} else {
		if (0 != gpio_request(it6807_config_info.backlight_gpio, NULL)) {
			dev_err(&client->dev, "backlight_gpio_request is failed\n");
			return -1;
		}
		if (0 != gpio_direction_output(it6807_config_info.backlight_gpio, 0)) {
			dev_err(&client->dev, "backlight_gpio set err!\n");
			return -1;
		}
	}

	return 0;
}

/* ******************************************************
Function:
	I2c probe.
Input:
	client: i2c device struct.
	id: device id.
Output:
	Executive outcomes.
	0: succeed.
****************************************************** */
static int it6807_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int err;
	int ret = 0;

	dev_info(&client->dev, "%s\n", DRIVER_VERSION);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		return err;
	}

	this_client = client;
	dev_info(&client->dev, "probe ok====>0x%x\n", client->addr);

	ret = script_it6807_gpio_init(client);
	if (ret) {
		dev_err(&client->dev, "gpio init failed\n");
		return ret;
	}
	gpio_set_value(it6807_config_info.reset_gpio, 0);
	msleep(40);
	gpio_set_value(it6807_config_info.reset_gpio, 1);

	hdmi_3v3 = regulator_get(&client->dev, "it6807_hdmi_vcc_3v3");
	if (IS_ERR(hdmi_3v3)) {
		dev_err(&client->dev, "get it6807_hdmi_vcc_3v3 failed\n");
	} else {
		regulator_set_voltage(hdmi_3v3, 3300000, 3300000);
		ret = regulator_enable(hdmi_3v3);
		if (ret != 0) {
			dev_err(&client->dev, "it6807_hdmi_vcc enable failed!\n");
			regulator_disable(hdmi_3v3);
			regulator_put(hdmi_3v3);
		}
	}

	it6807_3v3 = regulator_get(&client->dev, "it6807_vcc_3v3");
	if (IS_ERR_OR_NULL(it6807_3v3)) {
		dev_err(&client->dev, "get it6807_vcc_3v3 failed\n");
	} else {
		regulator_set_voltage(it6807_3v3, 3300000, 3300000);
		ret = regulator_enable(it6807_3v3);
		if (ret != 0) {
			dev_err(&client->dev, "it6807_vcc enable failed!\n");
			regulator_disable(it6807_3v3);
			regulator_put(it6807_3v3);
			it6807_3v3 = NULL;
			return ret;
		}
	}

	it6807_fsm_task =
		kthread_create(it6807_fsm_thread, NULL, "it6807_fsm_thread");
	if (IS_ERR_OR_NULL(it6807_fsm_task)) {
		dev_err(&client->dev, "Unable to start kernel thread. ");
		ret = PTR_ERR(it6807_fsm_task);
		it6807_fsm_task = NULL;
		return ret;
	}
	wake_up_process(it6807_fsm_task);
	return 0;
}

#if !IT6807_MATCH_DTS_EN
static int it6807_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	if (!client || !client->adapter) {
		dev_err(&client->dev,
				"it6807 detect client or client->adapter is NULL\n");
		return -1;
	}
	dev_info(
		&client->dev,
		"enter it6807 detect==>the adapter number is %d,,,,client->addr=%x\n",
		adapter->nr, client->addr);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "it6807 i2c_check_functionality is fail\n");
		return -ENODEV;
	}

	if ((1 == adapter->nr) && (client->addr == 0x48)) {
		dev_info(&client->dev,
				 "Detected chip i2c1_it6807 at adapter %d, address 0x%02x\n",
				 i2c_adapter_id(adapter), client->addr);
		/* this_client = client; */
		strlcpy(info->type, "it6807_0", I2C_NAME_SIZE);
	} else {
		dev_err(&client->dev,
				"Detected chip i2c1_it6807 at adapter 1, address 0x48 fail\n");
		return -ENODEV;
	}
	return 0;
}
#endif

static int it6807_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "it6807_remove:**********************\n");
	if (gpio_is_valid(it6807_config_info.reset_gpio)) {
		gpio_set_value(it6807_config_info.reset_gpio, 0);
		gpio_free(it6807_config_info.reset_gpio);
	}

	if (gpio_is_valid(it6807_config_info.backlight_gpio)) {
		gpio_set_value(it6807_config_info.backlight_gpio, 0);
		gpio_free(it6807_config_info.backlight_gpio);
	}
	if (it6807_fsm_task) {
		dev_info(&client->dev, "it6807_fsm_task need exit now\n");
		kthread_stop(it6807_fsm_task);
		it6807_fsm_task = NULL;
	}
	i2c_set_clientdata(client, NULL);
	return 0;
}

static void it6807_shutdown(struct i2c_client *client)
{
	if (it6807_fsm_task) {
		dev_info(&client->dev, "it6807_fsm_task need exit now\n");
		kthread_stop(it6807_fsm_task);
		it6807_fsm_task = NULL;
	}
}

static int it6807_pm_suspend(struct device *dev)
{
	if (it6807_fsm_task) {
		dev_info(dev, "it6807_fsm_task need exit now\n");
		kthread_stop(it6807_fsm_task);
		it6807_fsm_task = NULL;
		iTE6805_CurrentStage = STAGE6805_INIT;
	}

	if (!IS_ERR_OR_NULL(it6807_3v3)) {
		if (regulator_is_enabled(it6807_3v3)) {
			regulator_disable(it6807_3v3);
		}
	}

	if (gpio_is_valid(it6807_config_info.reset_gpio)) {
		gpio_set_value(it6807_config_info.reset_gpio, 0);
		gpio_free(it6807_config_info.reset_gpio);
	}

	if (gpio_is_valid(it6807_config_info.backlight_gpio)) {
		gpio_set_value(it6807_config_info.backlight_gpio, 0);
		gpio_free(it6807_config_info.backlight_gpio);
	}
	return 0;
}

static int it6807_pm_resume(struct device *dev)
{
	int ret;

	if (!IS_ERR_OR_NULL(it6807_3v3)) {
		dev_info(dev, "resume func set it6807 vcc power.\n");
		regulator_set_voltage(it6807_3v3, 3300000, 3300000);
		if (regulator_enable(it6807_3v3) != 0) {
			dev_err(dev, "enable it6807 vcc power error!\n");
		}
	}

	if (gpio_is_valid(it6807_config_info.reset_gpio)) {
		ret = gpio_request(it6807_config_info.reset_gpio, NULL);
		if (!ret) {
			gpio_direction_output(it6807_config_info.reset_gpio, 0);
			gpio_set_value(it6807_config_info.reset_gpio, 0);
			msleep(40);
			gpio_set_value(it6807_config_info.reset_gpio, 1);
		}
	}

	if (gpio_is_valid(it6807_config_info.backlight_gpio)) {
		ret = gpio_request(it6807_config_info.backlight_gpio, NULL);
		if (!ret) {
			gpio_direction_output(it6807_config_info.backlight_gpio, 0);
		}
	}

	it6807_fsm_task =
		kthread_create(it6807_fsm_thread, NULL, "it6807_fsm_thread");
	if (IS_ERR(it6807_fsm_task)) {
		dev_err(dev, "Unable to start kernel thread. ");
		ret = PTR_ERR(it6807_fsm_task);
		it6807_fsm_task = NULL;
		return ret;
	}
	wake_up_process(it6807_fsm_task);
	return 0;
}

static const unsigned short normal_i2c[] = {0x48, I2C_CLIENT_END};

static struct dev_pm_ops it6807_pm_ops = {
	.suspend = it6807_pm_suspend,
	.resume = it6807_pm_resume,
};

static struct i2c_driver it6807_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
			.name = IT6807_NAME,
			.owner = THIS_MODULE,
#if IT6807_MATCH_DTS_EN
			.of_match_table = it6807_dt_ids,
#endif
#ifndef CONFIG_HAS_EARLYSUSPEND
#if defined(CONFIG_PM)
			.pm = &it6807_pm_ops,
#endif
#endif
		},
	.probe = it6807_probe,
	.remove = it6807_remove,
	.shutdown = it6807_shutdown,
	.id_table = it6807_id,
#if !IT6807_MATCH_DTS_EN
	.detect = it6807_detect,
	.address_list = normal_i2c,
#endif
};

/* ******************************************************
Function:
	Driver Install function.
Input:
	None.
Output:
	Executive Outcomes. 0---succeed.
******************************************************* */
static int __init it6807_init(void)
{
	int ret = 0;
	pr_info("[it6807] %s\n", __func__);

	ret = i2c_add_driver(&it6807_driver);
	if (ret != 0)
		pr_err("Failed to register it6807 i2c driver : %d \n", ret);
#ifdef CONFIG_IT6807_HDMI_VBYONE_DEBUG
	itedbg_init();
#endif
	return ret;
}

/* ******************************************************
Function:
	Driver uninstall function.
Input:
	None.
Output:
	Executive Outcomes. 0---succeed.
******************************************************* */
static void __exit it6807_exit(void)
{
	pr_info("[it6807] %s\n", __func__);
#ifdef CONFIG_IT6807_HDMI_VBYONE_DEBUG
	itedbg_exit();
#endif

	i2c_del_driver(&it6807_driver);
}

module_init(it6807_init);
module_exit(it6807_exit);

MODULE_AUTHOR("<wanpeng@allwinnertech.com>");
MODULE_DESCRIPTION("IT6807 Driver");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");