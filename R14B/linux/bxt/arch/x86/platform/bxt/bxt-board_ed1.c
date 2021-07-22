/* Place Holder, Contents will be populated as and when we have Board specific modules*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#ifdef FCA_DV1_MOBIS_DISPLAY
#include <linux/input/cy8ctmg110_pdata.h>
#include <linux/input.h>
#include <linux/input-event-codes.h>
#include <linux/i2c/mobis_ts.h>
#include <linux/i2c/mobis_display.h>
#else
#include <linux/platform_data/atmel_mxt_ts.h>
#include <linux/i2c/r1_display.h>
#endif
#include "camera_init.h"

#define R1_SERIALIZER_ADDR 0x0C
#define DRVNAME            "bxt-board"

#ifdef FCA_DV1_MOBIS_DISPLAY
/* Cypress touch slave address */
#define CYPRESS_TS_ADDR 0x24
#define CYTTSP6_I2C_IRQ_GPIO (434 + 14)
#define CYTTSP6_I2C_RST_GPIO (434 + 19)
#define CYTTSP6_ACPI_IRQ_GPIO 0x71

static struct mobis_display_platform_data mobis_display_pdata = {
    .mxt_platform_data = {
        .input_name = "mobis_ts",
        .irqflags = IRQF_TRIGGER_LOW,
        .t19_num_keys = 1,
        .t15_num_keys = 1,
        //.gpio_reset;
        .gpio_int = CYTTSP6_I2C_IRQ_GPIO,
        //.gpio_switch;
        .cfg_name="maxtouch.cfg",
        .regulator_dis = 1,
    }
};

static struct i2c_board_info mobis_i2c_devs7[] __initdata = {
    { I2C_BOARD_INFO("mobis_display", R1_SERIALIZER_ADDR),
    .platform_data = &mobis_display_pdata },
};

#else

static struct r1_display_platform_data r1_display_pdata = {
    .mxt_platform_data = {
  //     .input_name = "atmel_mxt_ts",
        .irqflags = IRQF_TRIGGER_LOW,
        .t19_num_keys = 1,
    }
};

static struct i2c_board_info r1_i2c_devs7[] __initdata = {
    { I2C_BOARD_INFO("r1_display", R1_SERIALIZER_ADDR),
    .platform_data = &r1_display_pdata },
};

#endif

static int __init bxt_board_init(void)
{
	int ret;

	camera_init();
	pr_info(DRVNAME ": >>> oem_camera_type = 0x%x\n", oem_camera_type);

        pr_info(DRVNAME ": registering R1 i2c devices...\n");

#if defined(CONFIG_ED1_BOARD) && defined(FCA_DV1_MOBIS_DISPLAY)
        ret = i2c_register_board_info(7, mobis_i2c_devs7,
        ARRAY_SIZE(mobis_i2c_devs7));
	pr_info(DRVNAME "mobis_i2c_devs7 register ret value %d\n",ret);
#elif defined(CONFIG_ED1_BOARD)
        ret = i2c_register_board_info(7, r1_i2c_devs7,
                      ARRAY_SIZE(r1_i2c_devs7));
        pr_info(DRVNAME ">>r1_i2c_devs7 7 register ret value %d\n",ret);
#else
        pr_info(DRVNAME ">>r1_i2c_devs7 skip register for DV1\n");

#endif

exit:
       return ret;
}
arch_initcall(bxt_board_init);
MODULE_LICENSE(GPL);
