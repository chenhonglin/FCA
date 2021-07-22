/***
 * Author: Brian McFarland <brian.mcfarland@harman.com>
 *  Kernel module to enforce proper power on/off behavior for SXM module(s)
 *  including on S3 + resume.
 *
 * (c) Copyright Harman International 2019
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

/************** PLATFORM DEV PARAMETERS *******************************/
/**
 * For more flexibility, This section could/should go in either a separate
 * module, in dts/acpi entries, or in board files. 
 *
 * However, directly instaiating platform DEVICE from the platform DRIVER
 * module is not unheard of. See trusty.c for example.
 */

/* Default pins. Should be in board file / dts / acpi */
static int mp_sxmio_rst  = 440;
static int mp_sxmio_shdn = 304;

/* AT POWER OFF: Minimum delay (in ms) between driving reset line low and
 * driver shutdown line low. */
static unsigned long mp_sxmio_rst_lo=250;

/* AT POWER OFF: Minimum delay (in ms) between driving shutdown line HIGH and
 * RESET line HIGH. */
static unsigned long mp_sxmio_shdn_hi=250;

/* Minimum power-off time in milliseconds */
static unsigned long mp_sxmio_min_off=2000; 

/* Delay threshold used for usleep_range(). Keeping this on the order of a
 * system tick will let the system coallesce wakeups more efficiently. But
 * using usleep_range() gives option for further precision if needed. */
#define SXM_DELAY_THRESH_US (10000u)
#define SXM_MAX_DELAYTIME_MS (3000u)


#define POWER_MODE_ON   (1)
#define POWER_MODE_OFF  (0)
#define POWER_MODE_INIT (-1)

/* Definition of platform device-specific data.*/
struct sxm_device_platform_data {
	int reset_gpio;
	int shdn_gpio;
};

/* Single instance of device-specific data. */
static struct sxm_device_platform_data sxm_pdata = {
	.reset_gpio = 0,
	.shdn_gpio = 0,
};

/* Release function is required in case we have to "unwind" from a failure in
 * probe/init and remove platform device. kernel will warn about this being a
 * bug if not here, even if the function has no work to do. */
static void sxm_pdev_release(struct device *dev) {
	dev_info(dev, "%s() was called\n", __func__);
	return;
}

/* Single instance of platform device. Register this device before calling
 * platform driver probe (Again: really belongs in boardfile/dts/acpi) */
static struct platform_device sxm_pdev = {
	.name = "sxmio-dev",
	.id   = PLATFORM_DEVID_NONE,
	.dev = {
		.platform_data = &sxm_pdata,
		.release       = sxm_pdev_release,
	}
};
/********** END PLATFORM DEV PARAMETERS *******************************/

/* Per-instance driver data/state */
struct sxm_driver_data {
	/* Current power on/off state */
	int		power_mode;

	/**
	 * Powerctl lock status. 
	 *
	 */
	struct mutex powerctl_lock;

	/* Shutdown & reset gpio descriptors, copied here out of convenience
	 * to avoid referring back to the platform data struct */
	struct gpio_desc *shdn;
	struct gpio_desc *rst;

	/* Last system shutdown time in jiffies. Must wait until this time +
	 * "min off time" before powering back on. */
	unsigned long last_shutdown_time;
};

/* Power OFF the device with proper sequence. powerctl lock should
 * be held PRIOR to calling this function. */
static int sxm_powerctl_shutdown(struct platform_device *pdev)
{
	struct sxm_driver_data  *ddata = platform_get_drvdata(pdev);
	unsigned long min_delay = mp_sxmio_rst_lo * 1000; 
	dev_dbg(&pdev->dev, "%s: pdev=%p, ddata=%p powerstate=%d\n", __func__,
	    pdev, ddata, ddata->power_mode);

        gpiod_direction_output(ddata->rst, 0);
	usleep_range(min_delay, min_delay + SXM_DELAY_THRESH_US);
        gpiod_direction_output(ddata->shdn, 0);

	ddata->power_mode = 0;

	ddata->last_shutdown_time = jiffies;
	dev_dbg(&pdev->dev, "%s: pdev=%p, ddata=%p powerstate=%d\n", __func__,
	    pdev, ddata, ddata->power_mode);
	return 0;
}

/* Power ON the device with proper sequence. powerctl lock should
 * be held PRIOR to calling this function. */
static int sxm_powerctl_poweron(struct platform_device *pdev)
{
	struct sxm_driver_data  *ddata = platform_get_drvdata(pdev);
	unsigned long min_delay = mp_sxmio_shdn_hi * 1000; 

	dev_dbg(&pdev->dev, "%s: pdev=%p, ddata=%p powerstate=%d\n", __func__,
	    pdev, ddata, ddata->power_mode);

	if (time_before(jiffies, 
		    ddata->last_shutdown_time +
		    msecs_to_jiffies(mp_sxmio_min_off)))
	{
		dev_dbg(&pdev->dev, "%s: waiting for min shutdown time\n", __func__);
		msleep( jiffies_to_msecs(ddata->last_shutdown_time +
			    msecs_to_jiffies(mp_sxmio_min_off) - jiffies));
	}

	gpiod_direction_output(ddata->shdn, 1);
	usleep_range(min_delay, min_delay + SXM_DELAY_THRESH_US);
	gpiod_direction_output(ddata->rst, 1);

	ddata->power_mode = 1;

	dev_dbg(&pdev->dev, "%s: pdev=%p, ddata=%p powerstate=%d\n", __func__,
	    pdev, ddata, ddata ? ddata->power_mode : -1);
	return 0;
}

static ssize_t sxmio_powerctl_show(struct device *dev, 
        struct device_attribute *attr, char *buf)
{
	struct platform_device  *pdev = to_platform_device(dev);
	struct sxm_driver_data  *ddata = platform_get_drvdata(pdev);
	ssize_t ret;

	if ( 0 == mutex_lock_interruptible(&ddata->powerctl_lock) ){
        if ( ddata->power_mode == POWER_MODE_ON ){
            ret = scnprintf(buf, PAGE_SIZE, "on\n");
        } else if ( ddata->power_mode == POWER_MODE_OFF ){
            ret = scnprintf(buf, PAGE_SIZE, "off\n");
        } else {
            ret = scnprintf(buf, PAGE_SIZE, "unknown\n");
        }
		mutex_unlock(&ddata->powerctl_lock);
	} else {
		ret = -EINTR;
	}
	return ret;
}

static ssize_t sxmio_powerctl_store(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device  *pdev = to_platform_device(dev);
	struct sxm_driver_data  *ddata = platform_get_drvdata(pdev);
	int mode;

	if ( sysfs_streq(buf, "on")) 
		mode = POWER_MODE_ON;
	else if (sysfs_streq(buf, "off"))
		mode = POWER_MODE_OFF;
	else 
		return -EINVAL;
    
	if ( 0 == mutex_lock_interruptible(&ddata->powerctl_lock)){
		if (ddata->power_mode != mode){
			if (mode == POWER_MODE_ON) sxm_powerctl_poweron(pdev);
			if (mode == POWER_MODE_OFF) sxm_powerctl_shutdown(pdev);
		}
		mutex_unlock(&ddata->powerctl_lock);
	} else {
		return -EINTR;
	}
	return count;
}

static int sxmio_suspend(struct platform_device  *pdev, pm_message_t message)
{
	struct sxm_driver_data  *ddata = platform_get_drvdata(pdev);
	int ret;
	dev_info(&pdev->dev, "%s - entry\n", __func__);
	mutex_lock(&ddata->powerctl_lock);
	ret = sxm_powerctl_shutdown(pdev);
	mutex_unlock(&ddata->powerctl_lock);
	dev_info(&pdev->dev, "%s - exit\n", __func__);
	return ret;
}

static int sxmio_resume(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s - exit\n", __func__);
	return 0;
}


static DEVICE_ATTR(powerctl, S_IWUSR | S_IWGRP | S_IRUGO,
        sxmio_powerctl_show,
        sxmio_powerctl_store);

static struct attribute *sxm_attrs[] = {
	&dev_attr_powerctl.attr, 
	NULL
};

static struct attribute_group sxmio_attr_group = {
	.attrs = sxm_attrs,
};


/* Setup one instance of SXMIO device + do any additional device <=> driver
 * binding steps */
static int sxm_probe(struct platform_device *pdev)
{
	struct sxm_device_platform_data * pdata; // platform supplied data
	struct sxm_driver_data          * ddata; // driver specific data
	int ret;

	pdata = dev_get_platdata(&pdev->dev);

	/* Request initial handles to GPIO lines. Does not actually change
	 * states of the pins. Using gpio_request guarantees module loading
	 * will fail if somebody else controls those pins.  devm_ variant
	 * automates cleanup. */
	if ( 0 != (ret = devm_gpio_request(&pdev->dev, pdata->reset_gpio, "sxmio-reset"))){
		dev_err(&pdev->dev, "%s: failed to request RESET gpio\n", __func__);
		return ret;
	}

	if ( 0 != (ret = devm_gpio_request(&pdev->dev, pdata->shdn_gpio, "sxmio-shdn"))) {
		dev_err(&pdev->dev, "%s: failed to request SHDN gpio\n", __func__);
		return ret;
	}


	/* Allocate + initialize remaining driver instance specific data */
	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if ( !ddata ){
		dev_err(&pdev->dev, "%s: out of memory!\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&ddata->powerctl_lock);
	ddata->power_mode = POWER_MODE_INIT; /* init to unknown state */
	ddata->shdn = gpio_to_desc(pdata->shdn_gpio);
	ddata->rst  = gpio_to_desc(pdata->reset_gpio);

	if ( !ddata->shdn || !ddata->rst ) {
		dev_err(&pdev->dev, "%s: failed to get gpio_desc: rst=%p, shdn=%p\n", 
                __func__, ddata->rst, ddata->shdn);
		ret = -EINVAL;
		goto cleanup_exit;
	} 

	platform_set_drvdata(pdev, ddata);

	/* Shutdown the device on module insertion. This should give a clean
	 * slate regardless of initial starting state of the pins. */
	if ( 0 != (ret = sxm_powerctl_shutdown(pdev))){
		dev_err(&pdev->dev, "%s: shutdown failed\n", __func__);
		goto cleanup_exit;
	}

	/* As final step, register user-space API (sysfs interface) */
	if ( 0 != (ret = sysfs_create_group(&pdev->dev.kobj, &sxmio_attr_group))) {
		dev_err(&pdev->dev, "%s: failed to make sysfs attributes\n", __func__);
		goto cleanup_exit;
	}

	pr_info("sxmio device found\n");
	return 0;

cleanup_exit:
	mutex_destroy(&ddata->powerctl_lock);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

/* This should just shutdown the sxm module. Resources (GPIOs) should be
 * automatically freed. */
static int sxm_remove(struct platform_device *pdev)
{
	struct sxm_driver_data          * ddata; // driver specific data
	ddata = platform_get_drvdata(pdev);
	sysfs_remove_group(&pdev->dev.kobj, &sxmio_attr_group);

	/* Shut down when removing the module just for a consistent state */
	mutex_lock(&ddata->powerctl_lock);
	if ( ddata->power_mode != POWER_MODE_OFF ){
		sxm_powerctl_shutdown(pdev);
	}
	mutex_unlock(&ddata->powerctl_lock);

	/* Don't do this until after removing sysfs node */
	mutex_destroy(&ddata->powerctl_lock);
	return 0;
}




/********** Module boiler-plate ***************************************/

/* Driver structure. */
static struct platform_driver sxm_driver = {
	.probe  = sxm_probe,
	.remove = sxm_remove,
	.suspend = sxmio_suspend,
	.resume  = sxmio_resume,
	.driver = {
		.name  = "sxmio-dev",
		.owner = THIS_MODULE
	},
};

static int __init sxmio_init(void)
{
	int ret = -1;

	/* Process / sanity check module params  */
	sxm_pdata.reset_gpio = mp_sxmio_rst;
	sxm_pdata.shdn_gpio  = mp_sxmio_shdn;


	if ( mp_sxmio_rst_lo > SXM_MAX_DELAYTIME_MS ){
		mp_sxmio_rst_lo = SXM_MAX_DELAYTIME_MS;
		pr_warn("limiting reset low time at %lu ms\n", mp_sxmio_rst_lo);
	}

	if ( mp_sxmio_shdn_hi > SXM_MAX_DELAYTIME_MS ){
		mp_sxmio_shdn_hi = SXM_MAX_DELAYTIME_MS;
		pr_warn("limiting shdn hi time at %lu ms\n", mp_sxmio_shdn_hi);
	}

	if ( mp_sxmio_min_off > SXM_MAX_DELAYTIME_MS ){
		mp_sxmio_min_off = SXM_MAX_DELAYTIME_MS;
		pr_warn("limiting min off time at %lu ms\n", mp_sxmio_min_off);
	}

	/* Register platform device */
	if ( 0 != (ret = platform_device_register(&sxm_pdev))) {
		pr_err("%s: failed to register platform device: %d\n", __func__, ret);
		return ret;
	} 

	if ( 0 != (ret = platform_driver_probe(&sxm_driver, sxm_probe))){
		pr_err("%s: probe failed: %d\n", __func__, ret);
		platform_device_unregister(&sxm_pdev);
		return ret;
	}

	return ret;
}

static void __exit sxmio_exit(void) {
	platform_driver_unregister(&sxm_driver);
	platform_device_unregister(&sxm_pdev);
}


/* Allow pin-number override on insmod, but set permissions to only allow
 * reading */
module_param_named(rst, mp_sxmio_rst, int, 0440);
module_param_named(shdn, mp_sxmio_shdn, int, 0440);
module_param_named(rst_lo, mp_sxmio_rst_lo, ulong, 0440);
module_param_named(shdn_hi, mp_sxmio_shdn_hi, ulong, 0440);
module_param_named(minoff, mp_sxmio_min_off, ulong, 0440);

module_init(sxmio_init);
module_exit(sxmio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Brian McFarland <brian.mcfarland@harman.com>");
MODULE_DESCRIPTION("Control proper SHDN + RST sequence for SXM mod with S2R handling");

