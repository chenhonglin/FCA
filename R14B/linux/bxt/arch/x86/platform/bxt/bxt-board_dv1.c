#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input/cy8ctmg110_pdata.h>
#include <linux/input.h>
#include <linux/input-event-codes.h>
#include <linux/i2c/r1_display.h>
#include <linux/i2c/dcsd_display.h>

#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/audioconf.h>


/* CYPRESS TTSP */
#include <linux/cyttsp6_core.h>
#include <linux/cyttsp6_platform.h>

#include "camera_init.h"

/* Cypress touch slave address */
#define CYPRESS_TS_ADDR 0x24

#ifdef CONFIG_GPIO_DV1_S2R
#define DV1_GPIO_S2R_DUMMY_I2C_ADDR		0x77
#endif

#ifdef CONFIG_GPIO_DV1_S2R
#define DV1_GPIO_S2R_DUMMY_I2C_ADDR		0x77
#endif

#define R1_SERIALIZER_ADDR 0x0C
#define DRVNAME            "bxt-board"
#define CYTTSP6_MOBIS_I2C_IRQ_GPIO (434 + 14)

#define ANDROIDBOOT_DISPLAYTYPE 30

static char androidboot_displaytype[ANDROIDBOOT_DISPLAYTYPE];
static char androidboot_audiosystemconf[ANDROIDBOOT_AUDIOSYSTEM_CONF];


char *get_displaytype(void)
{
	return &androidboot_displaytype[0];
}
EXPORT_SYMBOL(get_displaytype);

static int __init get_androidboot_displaytype(char *str)
{
	strlcpy(androidboot_displaytype, str, ANDROIDBOOT_DISPLAYTYPE);
	return 1;
}
__setup("androidboot.displaytype=", get_androidboot_displaytype);

char *get_info_audiosystem_conf(void)
{
       return &androidboot_audiosystemconf[0];
}
EXPORT_SYMBOL(get_info_audiosystem_conf);

static int __init get_androidboot_info_audiosystem_conf(char *str)
{
        strlcpy(androidboot_audiosystemconf, str, ANDROIDBOOT_AUDIOSYSTEM_CONF);
        return 1;
}
__setup("androidboot.audio.system.conf=", get_androidboot_info_audiosystem_conf);

static struct pxa2xx_spi_chip chip_data = {
        .gpio_cs = -EINVAL,
        .dma_burst_size = 1,
//        .pio_dma_threshold = 8,
};

/* KMRD Related */

#define DV2_BOARD 1
#define DV3_BOARD 2
#define RECOVERY_MODE 1
#define NORMAL_MODE 0

/* NMTP Related changes */
static int __initdata boardType = DV3_BOARD;
static int __initdata numTuners = 0;
static int __initdata bootmode = NORMAL_MODE;
static int __initdata Dab_Present = 0;

static int __init get_boot_hardware_id(char *str)
{
	if ( strncmp(str, "DV2", strlen("DV2"))==0 ) {
		boardType = DV2_BOARD;
	} else {
		boardType = DV3_BOARD;
	}
	return 1;
}
__setup("androidboot.mainboardid=", get_boot_hardware_id);

static int __init get_market_variant(char *str)
{
	pr_info(DRVNAME "get_market_variant.str is %s\n", str);
	if ( strncmp(str, "RW", strlen("RW"))==0 ) {
		if (Dab_Present)
			numTuners = 2;
		else
			numTuners = 1;
	}
	else if (( strncmp(str, "EU", strlen("EU"))==0 ) || (strncmp(str, "KR", strlen("KR"))==0))
		numTuners = 2;
	else
		numTuners = 1;
	return 1;
}
__setup("androidboot.variant.market=", get_market_variant);

static int __init get_tuner_variant(char *str)
{
	pr_info(DRVNAME "get_tuner_variant.str is %s\n", str);
	if ( strncmp(str, "TUNER_MERCURY_DAB", strlen("TUNER_MERCURY_DAB"))==0 ) {
		Dab_Present = 1;
		numTuners = 2;
	}
	return 1;
}
__setup("androidboot.variant.tuner=", get_tuner_variant);

/* Boot mode is decided based on command line
 "androidboot.target=recovery" -> Recovery mode
"androidboot.target=normal" -> Normal mode
 */
static int __init set_boot_mode(char *str)
{
	pr_info(DRVNAME " set_boot_mode.str is %s\n", str);
	if ( strncmp(str, "rec",3)==0 ) {
		bootmode = RECOVERY_MODE;
	} else if ( strncmp(str, "nor",3)==0 ) {
		bootmode = NORMAL_MODE;
	} else{
		pr_info(DRVNAME " set_boot_mode.str is not valid \n");
	}
	pr_info(DRVNAME " bootmode is %d\n", bootmode);
	return 1;
}
__setup("androidboot.target=", set_boot_mode);

struct nmtp_state {
	bool mode;
};

struct nmtp_state  nmtp_master_mode = {
	.mode 		= 0x0,
};

static struct nmtp_state nmtp_slave_mode = {
	.mode 		= 0x1,
};


static struct spi_board_info bxt_spi_slaves_recovery_mode[] = {
         {
                .modalias = "spidev",
                .max_speed_hz = 50000000,
                .bus_num = 1,
                .chip_select = 0,
                .controller_data = &chip_data,
                .mode = SPI_MODE_1,
        },
        {
                .modalias = "spidev",
                .max_speed_hz = 50000000,
                .bus_num = 1,
                .chip_select = 1,
                .controller_data = &chip_data,
                .mode = SPI_MODE_1,
        },
        {
                .modalias = "ioc-ipc",
                .max_speed_hz = 50000000,
                .bus_num = 2,
                .chip_select = 0,
                .controller_data = &chip_data,
                .mode = SPI_MODE_1,
        },
        {
                .modalias = "dual-spi-ipc",
                .max_speed_hz = 50000000,
                .bus_num = 3,
                .chip_select = 0,
                .controller_data = &chip_data,
                .mode = SPI_MODE_1,
        },
        {
                .modalias = "spidev",
                .max_speed_hz = 50000000,
                .bus_num = 3,
                .chip_select = 2,
                .controller_data = &chip_data,
                .mode = SPI_MODE_0,
        }

};


static struct spi_board_info bxt_spi_slaves[] = {
        {
                .modalias = "nmtp_spi",
                .max_speed_hz = 8000000,
                .bus_num = 1,
                .chip_select = 0,
                .controller_data = &chip_data,
                .mode = SPI_MODE_1,
		.platform_data = &nmtp_master_mode,
        },
        {
                .modalias = "ioc-ipc",
                .max_speed_hz = 50000000,
                .bus_num = 2,
                .chip_select = 0,
                .controller_data = &chip_data,
                .mode = SPI_MODE_1,
        },
        {
                .modalias = "dual-spi-ipc",
                .max_speed_hz = 50000000,
                .bus_num = 3,
                .chip_select = 0,
                .controller_data = &chip_data,
                .mode = SPI_MODE_1,
        },
        {
                .modalias = "spidev",
                .max_speed_hz = 50000000,
                .bus_num = 3,
                .chip_select = 2,
                .controller_data = &chip_data,
                .mode = SPI_MODE_0,
        }

};

static struct spi_board_info bxt_spi_slaves_tuner_sec[] = {
        {

                .modalias = "nmtp_spi",
                .max_speed_hz = 10000000,
                .bus_num = 1,
                .chip_select = 1,
                .controller_data = &chip_data,
                .mode = SPI_MODE_1,
		.platform_data = &nmtp_slave_mode,
        },

};

/* Cypress touch IRQ GPIO pin */
#define CYTTSP6_I2C_IRQ_GPIO (434 + 14)
#define CYTTSP6_I2C_RST_GPIO (434 + 19)
#define CYTTSP6_ACPI_IRQ_GPIO 0x71
#define CY_VKEYS_X 1024
#define CY_VKEYS_Y 768
#define CY_MAXX 800
#define CY_MAXY 480
#define CY_MINX 0
#define CY_MINY 0

#define CY_ABS_MIN_X CY_MINX
#define CY_ABS_MIN_Y CY_MINY
#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MIN_P 0
#define CY_ABS_MAX_P 255
#define CY_ABS_MIN_W 0
#define CY_ABS_MAX_W 255
#define CY_PROXIMITY_MIN_VAL    0
#define CY_PROXIMITY_MAX_VAL    1

#define CY_ABS_MIN_T 0

#define CY_ABS_MAX_T 15

#define CYTTSP6_I2C_MAX_XFER_LEN  0x100

static struct cyttsp6_core_platform_data _cyttsp6_core_platform_data = {
       // .irq_gpio = CYTTSP6_ACPI_IRQ_GPIO,
        .irq_gpio = CYTTSP6_MOBIS_I2C_IRQ_GPIO,
        .rst_gpio = CYTTSP6_I2C_RST_GPIO,
	.max_xfer_len = CYTTSP6_I2C_MAX_XFER_LEN,
        .xres = cyttsp6_xres,
        .init = cyttsp6_init,
        .power = cyttsp6_power,
        .detect = cyttsp6_detect,
        .irq_stat = cyttsp6_irq_stat,
        .error_stat = cyttsp6_error_stat,
        .sett = {
                NULL,   /* Reserved */
                NULL,   /* Command Registers */
                NULL,   /* Touch Report */
                NULL,   /* Cypress Data Record */
                NULL,   /* Test Record */
                NULL,   /* Panel Configuration Record */
                NULL,   /* &cyttsp6_sett_param_regs, */
                NULL,   /* &cyttsp6_sett_param_size, */
                NULL,   /* Reserved */
                NULL,   /* Reserved */
                NULL,   /* Operational Configuration Record */
                NULL, /* &cyttsp6_sett_ddata, *//* Design Data Record */
                NULL, /* &cyttsp6_sett_mdata, *//* Manufacturing Data Record */
                NULL,   /* Config and Test Registers */
                NULL, /* button-to-keycode table */
        },
//        .flags = CY_CORE_FLAG_WAKE_ON_GESTURE,
//       .easy_wakeup_gesture = CY_CORE_EWG_TAP_TAP
//                | CY_CORE_EWG_TWO_FINGER_SLIDE,
};

static const int16_t cyttsp6_abs[] = {
        ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
        ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
        ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
        CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
        ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
        ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0,
        ABS_MT_TOUCH_MINOR, 0, 255, 0, 0,
        ABS_MT_ORIENTATION, -127, 127, 0, 0,
        ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0,
        ABS_DISTANCE, 0, 255, 0, 0,     /* Used with hover */
};

struct touch_framework cyttsp6_framework = {
        .abs = cyttsp6_abs,
        .size = ARRAY_SIZE(cyttsp6_abs),
        .enable_vkeys = 0,
};

/* Multitouch platform data */
static struct cyttsp6_mt_platform_data _cyttsp6_mt_platform_data = {
        .frmwrk = &cyttsp6_framework,
//        .flags = CY_MT_FLAG_INV_Y,
        .inp_dev_name = CYTTSP6_MT_NAME,
        .vkeys_x = CY_VKEYS_X,
        .vkeys_y = CY_VKEYS_Y,
};

static struct cyttsp6_btn_platform_data _cyttsp6_btn_platform_data = {
        .inp_dev_name = CYTTSP6_BTN_NAME,
};

static const int16_t cyttsp6_prox_abs[] = {
        ABS_DISTANCE, CY_PROXIMITY_MIN_VAL, CY_PROXIMITY_MAX_VAL, 0, 0,
};

struct touch_framework cyttsp6_prox_framework = {
        .abs = cyttsp6_prox_abs,
        .size = ARRAY_SIZE(cyttsp6_prox_abs),
};

static struct cyttsp6_proximity_platform_data
                _cyttsp6_proximity_platform_data = {
        .frmwrk = &cyttsp6_prox_framework,
        .inp_dev_name = CYTTSP6_PROXIMITY_NAME,
};

static struct cyttsp6_platform_data r1_cypress_touch_pdata = {
    .core_pdata = &_cyttsp6_core_platform_data,
    .mt_pdata = &_cyttsp6_mt_platform_data,
    .btn_pdata = &_cyttsp6_btn_platform_data,
    .prox_pdata = &_cyttsp6_proximity_platform_data,
    .loader_pdata = &_cyttsp6_loader_platform_data,
};

static struct i2c_board_info integrated_i2c_devs7[] __initdata = {
#ifdef CONFIG_GPIO_DV1_S2R
    {
           /* Dummy I2C device for DV1 GPIO S2R module*/
           I2C_BOARD_INFO("dv1_gpio_s2r", DV1_GPIO_S2R_DUMMY_I2C_ADDR),
    },
#endif
    {
           I2C_BOARD_INFO("cyttsp6_i2c_adapter", CYPRESS_TS_ADDR),
           .platform_data = &r1_cypress_touch_pdata
    },
    {
	   I2C_BOARD_INFO("adv7613_serdes", 0x4c),
    },

};


static struct dcsd_display_platform_data dcsd_display_pdata = {
    .dcsd_ts_pdata = {
        .input_name = "dcsd_ts",
    }
};

static struct i2c_board_info dcsd_i2c_devs7[] __initdata = {
#ifdef CONFIG_GPIO_DV1_S2R
    {
          /* Dummy I2C device for DV1 GPIO S2R module*/
          I2C_BOARD_INFO("dv1_gpio_s2r", DV1_GPIO_S2R_DUMMY_I2C_ADDR),
    },
#endif
    {
          I2C_BOARD_INFO("dcsd_display", R1_SERIALIZER_ADDR),
          .platform_data = &dcsd_display_pdata
    },
};

#ifdef CONFIG_FCA_DV1_H119_DISPLAY

static struct r1_display_platform_data h119_display_pdata = {
	.mxt_platform_data = {
		.input_name = "atmel_mxt_ts",
		.irqflags = IRQF_TRIGGER_LOW,
		.t19_num_keys = 1,
		.t15_num_keys = 1,
		.t19_keymap = NULL,
		.t15_keymap = NULL,
		.gpio_int = CYTTSP6_MOBIS_I2C_IRQ_GPIO,
		.cfg_name = "maxtouch.cfg",
		.regulator_dis = 1,
	}
};

static struct i2c_board_info h119_i2c_devs7[] __initdata = {
#ifdef CONFIG_GPIO_DV1_S2R
	{
		/* Dummy I2C device for DV1 GPIO S2R module*/
		I2C_BOARD_INFO("dv1_gpio_s2r", DV1_GPIO_S2R_DUMMY_I2C_ADDR),
	},
#endif
	{
		I2C_BOARD_INFO("h119_display", R1_SERIALIZER_ADDR),
		.platform_data = &h119_display_pdata
	},

};
#endif

#ifdef CONFIG_FCA_MULTI_DISP_TOUCH_ENABLE
static struct r1_display_platform_data r1_multi_display_pdata = {
    .mxt_platform_data = {
        .irqflags = IRQF_TRIGGER_LOW,
        .t19_num_keys = 1,
    }
};

static struct i2c_board_info r1_i2c_devs4[] __initdata = {
    {
          I2C_BOARD_INFO("r1_multi_display", R1_SERIALIZER_ADDR),
          .platform_data = &r1_multi_display_pdata
    },
};
#endif

static int __init bxt_dv1_board_init(void)
{
	int ret;

        pr_info(DRVNAME ": >>> androidboot_cameratype = %s\n", androidboot_cameratype);
        if(!strcmp(androidboot_cameratype, "dg")){
          oem_camera_type = 1; // 1280x800
        }
        else if(!strcmp(androidboot_cameratype, "dg2")){
          oem_camera_type = 2; // 1024x768
        }

	pr_info(DRVNAME ": registering R1 i2c & spi devices...\n");

	camera_init();

#if defined(CONFIG_FCA_INT_DISP_TOUCH_ENABLE) && defined(CONFIG_ENABLE_DCSD_DISPLAY) && defined(CONFIG_FCA_DV1_H119_DISPLAY)
	static const char disp_720p[] = "720p";
	static const char disp_h119[]  = "h119";

	pr_info(DRVNAME ": androidboot_displaytype=%s\n", androidboot_displaytype);
	if ( strcmp(androidboot_displaytype, disp_720p)==0 ) {
		ret = i2c_register_board_info(7, integrated_i2c_devs7, ARRAY_SIZE(integrated_i2c_devs7));
		pr_info(DRVNAME ": displaytype(%s), Integrated_i2c_touch_devs7 cypress touch register ret value %d\n", androidboot_displaytype, ret);
	} else if (strcmp(androidboot_displaytype, disp_h119) == 0) {
		ret = i2c_register_board_info(7, h119_i2c_devs7, ARRAY_SIZE(h119_i2c_devs7));
		pr_info(DRVNAME ": displaytype(%s), gw/h119_i2c_devs7 register ret value %d\n", androidboot_displaytype, ret);
	} else { /* Default load the mobis/DCSD display */
		ret = i2c_register_board_info(7, dcsd_i2c_devs7, ARRAY_SIZE(dcsd_i2c_devs7));
		pr_info(DRVNAME ": displaytype(%s), dcsd_i2c_devs7 register ret value %d\n", androidboot_displaytype, ret);
	}
#endif

#if defined(CONFIG_FCA_MULTI_DISP_TOUCH_ENABLE)
		// Fall back to ED1 1080p display touch if neither Integrated nor Mobis touch configs are set
		ret = i2c_register_board_info(4, r1_i2c_devs4,
			ARRAY_SIZE(r1_i2c_devs4));
     pr_info(DRVNAME ": Multi_display_i2c_touch_devs4 atmel touch register ret value %d\n",ret);
#endif

	/* Register the SPI devices */
        if(bootmode == NORMAL_MODE){
	     if(numTuners == 2){
				/* Register the SPI devices for secondary tuner */
		ret = spi_register_board_info(bxt_spi_slaves_tuner_sec,
				ARRAY_SIZE(bxt_spi_slaves_tuner_sec));
	    if (ret)
		 pr_err(DRVNAME "failed to register the SPI slaves for secondary tuner ...(%d)\n", ret);
	    else
		 pr_info(DRVNAME "successfully registered the secondary tuner SPI slaves in Normal Mode...\n");	
	   }

		ret = spi_register_board_info(bxt_spi_slaves,
			ARRAY_SIZE(bxt_spi_slaves));
	if (ret)
		pr_err(DRVNAME "failed to register the SPI slaves Normal Mode...(%d)\n", ret);
	else
		pr_info(DRVNAME "successfully registered the SPI slaves in Normal Mode...\n");

	}
	else if (bootmode == RECOVERY_MODE){
			ret = spi_register_board_info(bxt_spi_slaves_recovery_mode,
			ARRAY_SIZE(bxt_spi_slaves_recovery_mode));
	  if (ret)
		  pr_err(DRVNAME "failed to register the SPI slaves in Recovery mode...(%d)\n", ret);
	  else
		 pr_info(DRVNAME "successfully registered the SPI slaves in Recovery mode...\n");

	}


exit:
	return ret;
}
arch_initcall(bxt_dv1_board_init);
MODULE_LICENSE(GPL);

