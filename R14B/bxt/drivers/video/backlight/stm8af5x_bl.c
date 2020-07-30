/*
 * Backlight driver for ST Microcontroller Stm8af5288
 *
 * Copyright (C) 2018 Harman International Ltd.
 *   Xingfeng Ye <Xingfeng.Ye@harman.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/mfd/stm8af5x.h>
#include <linux/slab.h>
#include <linux/module.h>

#define MAX_BRIGHTNESS      100
#define MIN_BRIGHTNESS      0

struct stm8af5x_backlight_data {
	struct stm8af5x_chip	*chip;
	int	current_brightness;
	int	reg_read;
	int	reg_write;
};

static int stm8af5x_backlight_set(struct backlight_device *bl, int brightness)
{
	struct stm8af5x_backlight_data *data = bl_get_data(bl);
	struct stm8af5x_chip *chip = data->chip;
	unsigned char value;
	int ret;

	if (brightness > MAX_BRIGHTNESS)
		value = MAX_BRIGHTNESS;
	else if ((brightness >= 0) && (brightness <= 100))
		value = brightness;
	else {
		dev_err(&bl->dev, "invalid brightness value %d\n", brightness);
		return brightness;
	}

	/*brightness value 0~100*/
	ret = write_stm8af5x_display_btness(chip->client, data->reg_write, value);
	if (ret < 0) {
		dev_err(&bl->dev, "write to command 0x%x error %d\n", data->reg_write, ret);
		goto out;
	}

	dev_err(&bl->dev, "set brightness %d\n", value);
	data->current_brightness = value;
	return 0;
out:
	dev_err(&bl->dev, "set brightness %d failure with return value:%d\n",
		value, ret);
	return ret;
}

static int stm8af5x_backlight_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.state & BL_CORE_SUSPENDED)
		brightness = 0;

	return stm8af5x_backlight_set(bl, brightness);
}

static int stm8af5x_backlight_get_brightness(struct backlight_device *bl)
{
	struct stm8af5x_backlight_data *data = bl_get_data(bl);
	struct stm8af5x_chip *chip = data->chip;
	int ret;

	ret = read_stm8af5x_display_btness(chip->client, data->reg_read, 3);
	if(ret < 0) {
		dev_err(&bl->dev, "get current backlight value failed %d\n", ret);
		return ret;
	}
	else {
		dev_err(&bl->dev, "get current backlight value %d%\n", ret);
	}

	data->current_brightness = ret;
	return ret;
}

static const struct backlight_ops stm8af5x_backlight_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.update_status	= stm8af5x_backlight_update_status,
	.get_brightness	= stm8af5x_backlight_get_brightness,
};

#if 0
static void stm8af5x_backlight_dt_init(struct platform_device *pdev)
{
	struct device_node *nproot = pdev->dev.parent->of_node, *np;
	struct max8925_backlight_pdata *pdata;
	u32 val;

	if (!nproot || !IS_ENABLED(CONFIG_OF))
		return;

	pdata = devm_kzalloc(&pdev->dev,
			     sizeof(struct max8925_backlight_pdata),
			     GFP_KERNEL);
	if (!pdata)
		return;

	np = of_find_node_by_name(nproot, "backlight");
	if (!np) {
		dev_err(&pdev->dev, "failed to find backlight node\n");
		return;
	}

	if (!of_property_read_u32(np, "maxim,max8925-dual-string", &val))
		pdata->dual_string = val;

	pdev->dev.platform_data = pdata;
}
#endif

static int stm8af5x_backlight_probe(struct platform_device *pdev)
{
	struct stm8af5x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct stm8af5x_backlight_data *data;
	struct backlight_device *bl;
	struct backlight_properties props;
	struct resource *res;
	unsigned char value;
	int ret = 0;

	data = devm_kzalloc(&pdev->dev, sizeof(struct stm8af5x_backlight_data),
			    GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_REG, 0);
	if (!res) {
		dev_err(&pdev->dev, "No REG resource for read reg!\n");
		return -ENXIO;
	}
	data->reg_read = res->start;

	res = platform_get_resource(pdev, IORESOURCE_REG, 1);
	if (!res) {
		dev_err(&pdev->dev, "No REG resource for write reg!\n");
		return -ENXIO;
	}
	data->reg_write = res->start;

	data->chip = chip;
	data->current_brightness = 0;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_BRIGHTNESS;

	bl = devm_backlight_device_register(&pdev->dev, "stm8af5x-backlight",
					&pdev->dev, data,
					&stm8af5x_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		return PTR_ERR(bl);
	}

	/* Get Current User requested brightness (0 - max_brightness) */
	bl->props.brightness = read_stm8af5x_display_btness(chip->client, data->reg_read, 3);
	if(bl->props.brightness < 0) {
		dev_err(&pdev->dev, "get current backlight value failed %d\n", bl->props.brightness);
		goto err;
	}
	else {
		dev_err(&pdev->dev, "get current backlight value %d%\n", bl->props.brightness);
	}

	bl->props.brightness = MAX_BRIGHTNESS;
	platform_set_drvdata(pdev, bl);
	backlight_update_status(bl);
	return 0;
err:
	devm_backlight_device_unregister(&pdev->dev, bl);
	return -1;

}

static struct platform_driver stm8af5x_backlight_driver = {
	.driver		= {
		.name	= "stm8af5x-bl",
	},
	.probe		= stm8af5x_backlight_probe,
};

module_platform_driver(stm8af5x_backlight_driver);

MODULE_DESCRIPTION("Backlight driver for ST Microcontroller Stm8af5288");
MODULE_AUTHOR("Xingfeng Ye <Xingfeng.Ye@harman.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:stm8af5x-backlight");

