/*
 * drivers/video/mobis_display.c
 * FPDLink Serializer/Deserializer for R1 display driver
 */
#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c/mobis_display.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/delay.h>


/* Touch device name */
#define TC_DEV_NAME_MOBIS              "mobis_ts"
#define TC_DEV_NAME                    TC_DEV_NAME_MOBIS

/* I2C Addresses used */
#define I2C_ADDR_SER_PORT0             (0x0C)
#define I2C_ADDR_DESER                 (0x2C)
#define I2C_ADDR_MOBIS_TC              (0x70)
#define I2C_ADDR_DCSD_TC                (0x12)

/* I2C Chip registers */
#define SER_REG_TRANS_PORT_SEL         (0x1E)
#define SER_REG_SLAVE_ADDR_0           (0x07)
#define SER_REG_SLAVE_ALIAS_0          (0x08)
#define SER_REG_GPIO0                  (0x0D)
#define SER_REG_GENERAL_CONFIGURATION  (0x03)
#define SER_REG_ICR                    (0xC6)
#define DESER_REG_GPIO0                (0x1D)
#define DESER_REG_GPIO7                (0x21)
#define DESER_REG_MAP_SELECT           (0x49)
#define DESER_REG_GENERAL_STATUS       (0x1C)


/* I2C Chip Values / Masks */
#define SER_VALUE_GPIO0                (0x1F)
#define SER_VALUE_LVDS_BYPASS          (0xDA)
#define SER_REG_ICR_ENABLE_RX_INT      (0x21)
#define DESER_VALUE_GPIO0              (0x23)
#define DESER_VALUE_GPIO7              (0x09)
#define DESER_VALUE_GPIO7_OFF          (0x01)
#define DESER_VALUE_MAP_SELECT         (0x40)
#define DESER_VALUE_LOCK_BIT           (0x01)

struct mobis_display_rec {
    struct mobis_display_platform_data *pdata;
    struct workqueue_struct         *work_queue;
    struct delayed_work              work_item;
    struct i2c_board_info            touch_device_board_info;
    struct i2c_client               *deser_client;
    struct i2c_client               *touch_client;
};

#ifdef FCA_BACKLIGHT_STATE
static int prev_disp_state = -1;
extern int DisplayOn;
#endif
static u8 touch_i2c_slave_addr = (u8)(I2C_ADDR_DCSD_TC);
static u8 ser_touch_i2c_slave_addr = (u8)(I2C_ADDR_DCSD_TC<<1);

extern char *get_displaytype(void);

static int ser_i2c_read(struct i2c_client *client, int reg, uint8_t *val)
{
    int ret;
    ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read dev:0x%.2hhX reg:0x%.2hhX\n",
            client->addr, reg);
        return ret;
    }
    *val = ret;
    return 0;
}

static int ser_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
    int ret = i2c_smbus_write_byte_data(client, reg, val);
    if (ret)
        dev_err(&client->dev,
            "Failed to write to dev:0x%.2hhX reg:0x%.2hhX val:0x%.2hhX\n",
            client->addr, reg, val );
    return ret;
}

static int is_deserializer_locked(struct i2c_client* client)
{
    uint8_t val;
    int     err;

    /* Read the lock status of the deserializer */
    err = ser_i2c_read(client, DESER_REG_GENERAL_STATUS, &val);
    if(err < 0) {
        return err;
    }

    return ((val & DESER_VALUE_LOCK_BIT) == DESER_VALUE_LOCK_BIT ) ? 1 : 0;
}

static void register_touch_controller(struct work_struct* work_arg){

    /* Dig out the delayed_work pointer */
    struct delayed_work *delayed_work =
        container_of(work_arg, struct delayed_work, work);

    /* Then the mobis_display_rec pointer */
    struct mobis_display_rec *data =
        container_of(delayed_work, struct mobis_display_rec, work_item);
    int err;

    pr_notice("mobis_display: delay complete - starting touch driver\n");

    /* Configure the deserializer's GPIO0 for back channel Interrupt from Touch controller */
    err = ser_i2c_write(data->deser_client, DESER_REG_GPIO0, DESER_VALUE_GPIO0);
    if (err < 0) {
        pr_err("Failed to write in de-serializer\n");
        return;
    }

    /* Now time to announce that the touch controller appeared */
    data->touch_client = i2c_new_device(data->deser_client->adapter,
        &data->touch_device_board_info);
    if (data->touch_client == NULL) {
        pr_err("Unable to create new touch client\n");
        return;
    }

    i2c_release_client(data->touch_client);
    data->touch_client = 0;
}

static void check_lock_status(struct work_struct *work_arg){

    /* Dig out the delayed_work pointer */
    struct delayed_work *delayed_work =
        container_of(work_arg, struct delayed_work, work);

    /* Then the mobis_display_rec pointer */
    struct mobis_display_rec *data =
        container_of(delayed_work, struct mobis_display_rec, work_item);
    int err = 0;

#ifdef FCA_BACKLIGHT_STATE
    if(prev_disp_state != DisplayOn)
        pr_err("check_lock_status: DisplayOn = %d\n", DisplayOn);

    if(DisplayOn == 1){
	    err = is_deserializer_locked(data->deser_client);
    }
    prev_disp_state = DisplayOn;
#else
	err = is_deserializer_locked(data->deser_client);
#endif

    if (err == 1)
    {
        /* Start the required 230ms delay */
        pr_notice("mobis_display: LVDS_LOCK detected, starting delay\n");
        INIT_DELAYED_WORK(&data->work_item, register_touch_controller);
        queue_delayed_work(data->work_queue, &data->work_item, HZ/4);
    }
    else if (err == 0)
    {
        /* TC still not ready...  Resubmit this function */
        msleep(300);
        queue_delayed_work(data->work_queue, &data->work_item, HZ/10);
    }
    else
    {
        pr_err("unable to even talk to the deserializer?\n");
        /* queue task again if LOCK detect failed */
        queue_delayed_work(data->work_queue, &data->work_item, msecs_to_jiffies(1000));
    }
}


static int mobis_display_enable(struct i2c_client *client)
{
    struct mobis_display_rec *data = i2c_get_clientdata(client);
    int err;

    memset(&data->touch_device_board_info, 0, sizeof(data->touch_device_board_info));
    strlcpy(data->touch_device_board_info.type, TC_DEV_NAME, I2C_NAME_SIZE);
    data->touch_device_board_info.addr          = touch_i2c_slave_addr;
    data->touch_device_board_info.platform_data = &data->pdata->mxt_platform_data;

    do {
        /* Configures the Serializer with Touch controller I2C slave address (remote side) */
        err = ser_i2c_write(client, SER_REG_SLAVE_ADDR_0, ser_touch_i2c_slave_addr);
        if (err < 0)
            break;

        /* Configures the Serializer with Touch controller I2C slave address (local side) */
        err = ser_i2c_write(client, SER_REG_SLAVE_ALIAS_0, ser_touch_i2c_slave_addr);
        if (err < 0)
            break;

        /* Configure the serializer's GPIO0 for back channel Interrupt from Touch controller */
        err = ser_i2c_write(client, SER_REG_GPIO0, SER_VALUE_GPIO0);
        if (err < 0)
            break;

        /* Enable the communication over I2C link (to deserializer and touch controller) */
        err = ser_i2c_write(client, SER_REG_GENERAL_CONFIGURATION, SER_VALUE_LVDS_BYPASS);
        if (err < 0)
            break;

        /* Enable Interrupt on Receiver interrupt and Global Interrupt */
        err = ser_i2c_write(client, SER_REG_ICR, SER_REG_ICR_ENABLE_RX_INT);
        if (err < 0)
            break;

        /* Create a new i2c_client for the deserializer address */
        data->deser_client = i2c_new_dummy(client->adapter, I2C_ADDR_DESER);
        if (data->deser_client == NULL){
            err=-ENODEV;
            break;
        }

#if 0
        /* Configure the Deserializer's GPIO7 to enable the power for the remote display */
        err = ser_i2c_write(data->deser_client, DESER_REG_GPIO7, DESER_VALUE_GPIO7);
        if (err < 0)
            break;

        /* Change the color packing order - Fixes psychodelic colors on 1080p display */
        err = ser_i2c_write(data->deser_client, DESER_REG_MAP_SELECT, DESER_VALUE_MAP_SELECT);
        if (err < 0)
            break;
#endif
        /* Create work queue */
        data->work_queue = create_singlethread_workqueue("mobis_display");
        if(data->work_queue == NULL)
        {
            dev_err(&client->dev, "Cannot create mobis_display work_queue\n");
            err=-ENOMEM;
            break;
        }

        /* Kick off work queue item to sample LVDS lock */
        INIT_DELAYED_WORK(&data->work_item, check_lock_status);
        queue_delayed_work(data->work_queue, &data->work_item, HZ/10);

    }while(0);

    if (err<0){
        // Free everything
        return err;
    }
    else
    {
        // Free only touch client (if present)
        if(data->touch_client){
            pr_notice("mobis_display:Releasing touch client\n");
            i2c_release_client(data->touch_client);
            data->touch_client = 0;
        }
        pr_notice("mobis_display:Report success\n");
        return 0;
    }

}

static release_all_resources(struct mobis_display_rec* data)
{
    if (data->work_queue){
        cancel_delayed_work_sync(&data->work_item);
        drain_workqueue(data->work_queue);
        destroy_workqueue(data->work_queue);
        data->work_queue = 0;
    }

    if (data->deser_client){
        i2c_unregister_device(data->deser_client);
        data->deser_client = 0;
    }

    if (data->touch_client){
        i2c_unregister_device(data->touch_client);
        data->touch_client = 0;
    }
}

static void mobis_display_disable(struct i2c_client *client)
{
    struct mobis_display_rec *data = i2c_get_clientdata(client);
    release_all_resources(data);
}

static void update_touch_i2c_slave_address(void)
{
    const char disp_mobis[] = "mobis";
    char *display_type = get_displaytype();

    if ( strcmp(display_type, disp_mobis)==0 ) {
        touch_i2c_slave_addr = (u8)(I2C_ADDR_MOBIS_TC);
    } else {
        touch_i2c_slave_addr = (u8)(I2C_ADDR_DCSD_TC);
    }
    ser_touch_i2c_slave_addr = (u8)(touch_i2c_slave_addr<<1);
}

static int mobis_display_probe(struct i2c_client *client,
                            const struct i2c_device_id *id)
{
    struct mobis_display_rec *data;
    struct mobis_display_platform_data *pdata =
        client->dev.platform_data;
    int err;
    if (!i2c_check_functionality(client->adapter,
                    I2C_FUNC_SMBUS_BYTE_DATA)) {
        dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
        return -EIO;
    }
    if (!pdata) {
        dev_err(&client->dev, "no platform data?\n");
        return -EINVAL;
    }
    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if (data == NULL)
        return -ENOMEM;
    data->pdata = pdata;

    update_touch_i2c_slave_address();

    i2c_set_clientdata(client, data);
    err = mobis_display_enable(client);
    if (err < 0){
        release_all_resources(data);
        i2c_set_clientdata(client, NULL);
        return err;
    }
    else{
        return 0;
    }
}

static int mobis_display_remove(struct i2c_client *client)
{
    struct mobis_display_rec *data = i2c_get_clientdata(client);
#ifdef FCA_BACKLIGHT_STATE
    prev_disp_state = -1;
#endif
    mobis_display_disable(client);
    i2c_set_clientdata(client, NULL);
    return 0;
}

static void mobis_display_shutdown(struct i2c_client *client)
{
    struct mobis_display_rec *data = i2c_get_clientdata(client);
#ifdef FCA_BACKLIGHT_STATE
    prev_disp_state = -1;
#endif
    mobis_display_disable(client);
    i2c_set_clientdata(client, NULL);
}

#ifdef CONFIG_PM_SLEEP
static int mobis_display_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
#ifdef FCA_BACKLIGHT_STATE
    prev_disp_state = -1;
#endif
    mobis_display_disable(client);
    return 0;
}

static int mobis_display_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    return mobis_display_enable(client);
}
#endif

static const struct i2c_device_id mobis_display_id[] = {
    { "mobis_display", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, mobis_display_id);

static SIMPLE_DEV_PM_OPS(mobis_display_pm_ops, mobis_display_suspend, mobis_display_resume);

static struct i2c_driver mobis_display_driver = {
    .driver = {
        .name  = "mobis_display",
        .owner = THIS_MODULE,
        .pm    = &mobis_display_pm_ops,
    },
    .probe    = mobis_display_probe,
    .remove   = mobis_display_remove,
    .shutdown = mobis_display_shutdown,
    .id_table = mobis_display_id,
};

module_i2c_driver(mobis_display_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("mobis_display FPDLink Serializer driver");
MODULE_ALIAS("i2c:mobis_display");

