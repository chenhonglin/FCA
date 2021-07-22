/*
 * cyttsp4_platform.c
 * Cypress TrueTouch(TM) Standard Product V4 Platform Module.
 * For use with Cypress touchscreen controllers.
 * Supported part families include:
 * GEN6 XL
 * GEN L
 *
 * Copyright (C) 2013-2015 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include "cyttsp4_regs.h"

#define CYTTSP4_FW_UPGRADE	\
	(defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_PLATFORM_FW_UPGRADE) && \
	defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_FW_UPGRADE_SOLO))

#define CYTTSP5_FW_UPGRADE	\
	(defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_PLATFORM_FW_UPGRADE) && \
	defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_FW_UPGRADE_GEN4))

#define CYTTSP6_FW_UPGRADE	\
	(defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_PLATFORM_FW_UPGRADE) && \
	defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_FW_UPGRADE_GEN6))

#define CYTTSP4_TTCONFIG_UPGRADE	\
	(defined \
	(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_PLATFORM_TTCONFIG_UPGRADE) && \
	defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_TTCONFIG_UPGRADE_SOLO))

#define CYTTSP5_TTCONFIG_UPGRADE	\
	(defined \
	(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_PLATFORM_TTCONFIG_UPGRADE) && \
	defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_TTCONFIG_UPGRADE_GEN4))

#define CYTTSP6_TTCONFIG_UPGRADE	\
	(defined \
	(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_PLATFORM_TTCONFIG_UPGRADE) && \
	defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_TTCONFIG_UPGRADE_GEN6))

#if CYTTSP4_FW_UPGRADE
#include "cyttsp4_img.h"
static struct cyttsp4_touch_firmware cyttsp4_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
static struct cyttsp4_touch_firmware cyttsp4_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#if CYTTSP5_FW_UPGRADE
#include "cyttsp5_img.h"
static struct cyttsp4_touch_firmware cyttsp5_firmware = {
	.img = cyttsp5_img,
	.size = ARRAY_SIZE(cyttsp5_img),
	.ver = cyttsp5_ver,
	.vsize = ARRAY_SIZE(cyttsp5_ver),
};
#else

static struct cyttsp4_touch_firmware cyttsp5_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#if CYTTSP6_FW_UPGRADE
#include "cyttsp6_img.h"
static struct cyttsp4_touch_firmware cyttsp6_firmware = {
	.img = cyttsp6_img,
	.size = ARRAY_SIZE(cyttsp6_img),
	.ver = cyttsp6_ver,
	.vsize = ARRAY_SIZE(cyttsp6_ver),
};
#else

static struct cyttsp4_touch_firmware cyttsp6_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#if CYTTSP4_TTCONFIG_UPGRADE
#include "cyttsp4_params.h"
static struct touch_settings cyttsp4_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp4_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};

static struct cyttsp4_touch_config cyttsp4_ttconfig = {
	.param_regs = &cyttsp4_sett_param_regs,
	.param_size = &cyttsp4_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};

#else
static struct cyttsp4_touch_config cyttsp4_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

#if CYTTSP5_TTCONFIG_UPGRADE
#include "cyttsp5_params.h"
static struct touch_settings cyttsp5_sett_param_regs = {
	.data = (uint8_t *)&cyttsp5_param_regs[0],
	.size = ARRAY_SIZE(cyttsp5_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size = {
	.data = (uint8_t *)&cyttsp5_param_size[0],
	.size = ARRAY_SIZE(cyttsp5_param_size),
	.tag = 0,
};

static struct cyttsp4_touch_config cyttsp5_ttconfig = {
	.param_regs = &cyttsp5_sett_param_regs,
	.param_size = &cyttsp5_sett_param_size,
	.fw_ver = ttconfig5_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig5_fw_ver),
};
#else
static struct cyttsp4_touch_config cyttsp5_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

#if CYTTSP6_TTCONFIG_UPGRADE
#include "cyttsp6_params.h"
static struct touch_settings cyttsp6_sett_param_regs = {
	.data = (uint8_t *)&cyttsp6_param_regs[0],
	.size = ARRAY_SIZE(cyttsp6_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp6_sett_param_size = {
	.data = (uint8_t *)&cyttsp6_param_size[0],
	.size = ARRAY_SIZE(cyttsp6_param_size),
	.tag = 0,
};

static struct cyttsp4_touch_config cyttsp6_ttconfig = {
	.param_regs = &cyttsp6_sett_param_regs,
	.param_size = &cyttsp6_sett_param_size,
	.fw_ver = ttconfig6_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig6_fw_ver),
};
#else
static struct cyttsp4_touch_config cyttsp6_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

struct cyttsp4_loader_platform_data _cyttsp4_loader_platform_data = {
	.fw = &cyttsp4_firmware,
	.ttconfig = &cyttsp4_ttconfig,
	.flags = CY_LOADER_FLAG_NONE,
};

struct cyttsp4_loader_platform_data _cyttsp5_loader_platform_data = {
	.fw = &cyttsp5_firmware,
	.ttconfig = &cyttsp5_ttconfig,
	.flags = CY_LOADER_FLAG_NONE,
};

struct cyttsp4_loader_platform_data _cyttsp6_loader_platform_data = {
	.fw = &cyttsp6_firmware,
	.ttconfig = &cyttsp6_ttconfig,
	.flags = CY_LOADER_FLAG_NONE,
};

int cyttsp4_get_loader_platform_data(
			struct device *dev, int dev_id)
{
	struct cyttsp4_platform_data *pdata = dev->platform_data;

	if (pdata) {
		if (dev_id == CY_DEVID_GEN3) {
			pdata->loader_pdata = &_cyttsp4_loader_platform_data;
		} else if (dev_id == CY_DEVID_GEN4) {
			pdata->loader_pdata = &_cyttsp5_loader_platform_data;
		} else if (dev_id & GEN6_DEVICE) {
			pdata->loader_pdata = &_cyttsp6_loader_platform_data;
		} else {
			dev_err(dev, "%s: Invalid dev Id %d\n", __func__,
				dev_id);
			return -EINVAL;
		}
	} else {
		dev_err(dev, "%s: No platform data\n", __func__);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cyttsp4_get_loader_platform_data);

int cyttsp4_get_tch_param_size(struct device *dev, int dev_id)
{
	struct touch_settings *param_size;
	struct cyttsp4_platform_data *pdata = dev->platform_data;

	if (pdata && pdata->core_pdata) {
		/* Required for Device Access Group 7 support */

		if (dev_id == CY_DEVID_GEN3) {
			param_size =
			_cyttsp4_loader_platform_data.ttconfig->param_size;
		} else if (dev_id == CY_DEVID_GEN4) {
			param_size =
			_cyttsp5_loader_platform_data.ttconfig->param_size;
		} else if (dev_id & GEN6_DEVICE) {
			param_size =
			_cyttsp6_loader_platform_data.ttconfig->param_size;
		} else {
			dev_err(dev, "%s: Invalid dev Id %d\n", __func__,
				dev_id);
			return -EINVAL;
		}

		pdata->core_pdata->sett[CY_IC_GRPNUM_TCH_PARM_SIZE] =
								param_size;
	} else {
		dev_err(dev, "%s: No platform data\n", __func__);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cyttsp4_get_tch_param_size);

static inline void cyttsp4_gpio_lock_as_irq(int irq_gpio, struct irq_data *d)
{
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 2, 0))
	return;
#else
	struct gpio_chip *chip = gpio_to_chip(irq_gpio);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0))
	gpiochip_lock_as_irq(chip, d->hwirq);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
	gpio_lock_as_irq(chip, d->hwirq);
#endif
#endif
}

static inline void cyttsp4_gpio_unlock_as_irq(int irq_gpio, struct irq_data *d)
{
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 2, 0))
	return;
#else
	struct gpio_chip *chip = gpio_to_chip(irq_gpio);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0))
	gpiochip_unlock_as_irq(chip, d->hwirq);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
	gpio_unlock_as_irq(chip, d->hwirq);
#endif
#endif
}

static inline int _cyttsp6_gpio_request(struct device *dev, int gpio)
{
	int rc;

	rc = gpio_request(gpio, NULL);
	if (rc < 0) {
		gpio_free(gpio);
		rc = gpio_request(gpio, NULL);
	}

	if (rc < 0)
		dev_err(dev, "%s: Fail request gpio=%d\n", __func__, gpio);
	return rc;
}

int cyttsp4_xres(struct cyttsp4_core_platform_data *pdata,
		 struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int rc = 0;

	gpio_set_value(rst_gpio, 1);
	msleep(20);
	gpio_set_value(rst_gpio, 0);
	msleep(40);
	gpio_set_value(rst_gpio, 1);
	msleep(20);
	dev_info(dev,
		 "%s: RESET CYTTSP gpio=%d r=%d\n", __func__,
		 pdata->rst_gpio, rc);

	return rc;
}

int cyttsp4_init(struct cyttsp4_core_platform_data *pdata,
		 int on, struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int err_gpio = pdata->err_gpio;
	int rc = 0;

	if (on) {
		if(rst_gpio) {
			rc = _cyttsp6_gpio_request(dev, rst_gpio);
			if (rc < 0) {
				dev_err(dev, "%s: Fail request rst_gpio\n", __func__);
				return rc;
			}

			rc = gpio_direction_output(rst_gpio, 1);
			if (rc < 0) {
				pr_err("%s: Fail set output gpio=%d\n", __func__,
				       rst_gpio);
				goto free_rst;
			}
		}
		rc = _cyttsp6_gpio_request(dev, irq_gpio);
		if (rc < 0) {
			dev_err(dev, "%s: Fail request irq_gpio\n", __func__);
			goto free_rst;
		}

		rc = gpio_direction_input(irq_gpio);
		if (rc < 0) {
			pr_err("%s: Fail set input gpio=%d\n", __func__,
			       irq_gpio);
			goto free_irq;
		}

		if (err_gpio) {
			pr_err("%s: set error gpio=%d\n", __func__, err_gpio);
			rc = _cyttsp6_gpio_request(dev, err_gpio);
			if (rc < 0) {
				dev_err(dev, "%s: Fail request irq_gpio\n",
					__func__);
				goto free_irq;
			}

			rc = gpio_direction_input(err_gpio);
			if (rc < 0) {
				pr_err("%s: Fail set error gpio=%d\n", __func__,
				       err_gpio);
				goto free_err;
			}
		}
		dev_info(dev, "%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d,ERR gpio=%d r=%d\n",
			 __func__, rst_gpio, irq_gpio, err_gpio, rc);

		return 0;
	}

free_err:
	if (err_gpio)
		gpio_free(err_gpio);
free_irq:
	gpio_free(irq_gpio);
free_rst:
	gpio_free(rst_gpio);

	return rc;
}

static int cyttsp4_wakeup(struct cyttsp4_core_platform_data *pdata,
			  struct device *dev, atomic_t *ignore_irq)
{
	int irq_gpio = pdata->irq_gpio;
	struct cyttsp4_core_data *cd = dev_get_drvdata(dev);
	struct irq_data *d = irq_get_irq_data(cd->irq);
	int rc = 0;

	if (ignore_irq)
		atomic_set(ignore_irq, 1);
	if (!d) {
		dev_err(dev, "%s:", __func__);
		return -EINVAL;
	}

	cyttsp4_gpio_unlock_as_irq(irq_gpio, d);
	rc = gpio_direction_output(irq_gpio, 0);
	if (rc < 0) {
		if (ignore_irq)
			atomic_set(ignore_irq, 0);
		dev_err(dev,
			"%s: Fail set output gpio=%d\n",
			__func__, irq_gpio);
	} else {
		usleep_range(1000, 2000);
		rc = gpio_direction_input(irq_gpio);
		if (ignore_irq)
			atomic_set(ignore_irq, 0);
		if (rc < 0) {
			dev_err(dev,
				"%s: Fail set input gpio=%d\n",
				__func__, irq_gpio);
		} else {
			cyttsp4_gpio_lock_as_irq(irq_gpio, d);
		}
	}

	dev_info(dev,
		 "%s: WAKEUP CYTTSP gpio=%d r=%d\n", __func__,
		 irq_gpio, rc);

	return rc;
}

static int cyttsp4_sleep(struct cyttsp4_core_platform_data *pdata,
			 struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

int cyttsp4_power(struct cyttsp4_core_platform_data *pdata,
		  int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp4_wakeup(pdata, dev, ignore_irq);

	return cyttsp4_sleep(pdata, dev, ignore_irq);
}

int cyttsp4_irq_stat(struct cyttsp4_core_platform_data *pdata,
		     struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}

int cyttsp4_error_stat(struct cyttsp4_core_platform_data *pdata,
		       struct device *dev)
{
	return gpio_get_value(pdata->err_gpio);
}

#ifdef CYTTSP4_DETECT_HW
int cyttsp4_detect(struct cyttsp4_core_platform_data *pdata,
		   struct device *dev, cyttsp4_platform_read read)
{
	int retry = 3;
	int rc;
	char buf[1];

	while (retry--) {
		/* Perform reset, wait for 100 ms and perform read */
		dev_vdbg(dev, "%s: Performing a reset\n", __func__);
		pdata->xres(pdata, dev);
		msleep(100);
		rc = read(dev, 0, buf, 1);
		if (!rc)
			return 0;

		dev_vdbg(dev, "%s: Read unsuccessful, try=%d\n",
			 __func__, 3 - retry);
	}

	return rc;
}
#endif
