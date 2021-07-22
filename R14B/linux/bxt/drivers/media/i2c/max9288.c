/*
 * Copyright (c) 2017 Harman International Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/max9288.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>
#include <media/max9288.h>

#include "max9288-reg.h"
#include "max96705-reg.h"

struct max9288 {
	struct v4l2_subdev v4l2_sd;   // add v4l2 sub dev support to i2c driver
	struct max9288_pdata *pdata;
	struct media_pad pad[NR_OF_MAX9288_PADS];
	unsigned char sensor_present;
	unsigned int nsources;
	unsigned int nsinks;
	unsigned int npads;
	unsigned int nstreams;
	const char *name;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_subdev *sub_devs[NR_OF_MAX9288_SINK_PADS];
	struct v4l2_mbus_framefmt *ffmts[NR_OF_MAX9288_PADS];
	struct rect *crop;
	struct rect *compose;
	struct {
		unsigned int *stream_id;
	} *stream; /* stream enable/disable status, indexed by pad */
	struct {
		unsigned int sink;
		unsigned int source;
		int flags;
	} *route; /* pad level info, indexed by stream */
	struct i2c_client *client_ser;
	struct regmap *regmap8_ser;
	struct regmap *regmap8_dser;
	struct mutex max_mutex;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *test_pattern;
};

struct max_reg_blk {
	unsigned int reg;
	unsigned int val;
};

/* Macro to get access to max9288 structure using the v4l2_subdev structure */
#define to_max_9286(_sd) container_of(_sd, struct max9288, v4l2_sd)

/*
 * Order matters.
 *
 * 1. Bits-per-pixel, descending.
 * 2. Bits-per-pixel compressed, descending.
 * 3. Pixel order, same as in pixel_order_str. Formats for all four pixel
 *    orders must be defined.
 */
static const struct max9288_csi_data_format max_csi_data_formats[] = {
	{MEDIA_BUS_FMT_YUYV8_1X16, 16, 16, PIXEL_ORDER_GBRG, 0x1e},
	{MEDIA_BUS_FMT_UYVY8_1X16, 16, 16, PIXEL_ORDER_GBRG, 0x1e},
};

static const uint32_t max9288_supported_codes_pad[] = {
	MEDIA_BUS_FMT_YUYV8_1X16,
	MEDIA_BUS_FMT_UYVY8_1X16,
	0,
};

static const uint32_t *max9288_supported_codes[] = {
	max9288_supported_codes_pad,
};

static struct regmap_config max9288_reg_config8 = {
	.reg_bits = 8,
	.val_bits = 8,
};

static struct regmap_config max96705_reg_config8 = {
	.reg_bits = 8,
	.val_bits = 8,
};

static struct max_reg_blk crossbar_settings[] = {
	{SER_CROSSBAR_0, 0x03},
	{SER_CROSSBAR_1, 0x04},
	{SER_CROSSBAR_2, 0x05},
	{SER_CROSSBAR_3, 0x06},
	{SER_CROSSBAR_4, 0x07},
	{SER_CROSSBAR_7, 0x0E},
	{SER_CROSSBAR_8, 0x0F},
	{SER_CROSSBAR_9, 0x0D},
	{SER_CROSSBAR_16, 0x10},
	{SER_CROSSBAR_17, 0x11},
	{SER_CROSSBAR_18, 0x12},
	{SER_CROSSBAR_19, 0x13},
	{SER_CROSSBAR_20, 0x14},
	{SER_CROSSBAR_21, 0x15},
	{SER_CROSSBAR_22, 0x16},
	{SER_CROSSBAR_23, 0x17},
	{SER_CROSSBAR_24, 0x00},
	{SER_CROSSBAR_25, 0x01},
	{SER_CROSSBAR_26, 0x02},
};

int max9288_entity_status = 0;

static int max_read_reg(struct regmap *regmap8,
						unsigned int reg,
						unsigned int *val)
{
	int rval, retry;
	int timeout = 20;

	for (retry = 0; retry < timeout; retry++) {
		rval = regmap_read(regmap8, reg, val);
		if (rval)
			msleep(5);
		else
			break;
	}

	if (rval)
		pr_err("%s: max reading 0x%x failed, rval: %d retried: %d \n",
		__func__, reg, rval, retry);

	return rval;
}

static int max_write_reg(struct regmap *regmap8,
						unsigned int reg,
						unsigned int val)
{
	int retry, rval;
	int timeout = 20;
	u8 read_val;
	for (retry = 0; retry < timeout; retry++) {
		rval = regmap_write(regmap8, reg, val);
		if (rval)
			msleep(5);
		else
			break;
	}

	if (rval)
		pr_err("%s: max writing 0x%x failed, rval: %d retried: %d \n",
		__func__, reg, rval, retry);

	return rval;
}

static int max_write_regs(struct regmap *regmap8,
							struct max_reg_blk *rblk, int len)
{
	int i;
	int rval;

	for (i = 0; i < len; i++) {
		rval = max_write_reg(regmap8, rblk[i].reg, rblk[i].val);
		if (rval)
			break;
	}

	if (rval)
		pr_err("%s: max writing 0x%x failed, rval: %d\n",
		__func__, rblk[i].reg, rval);

	return rval;
}

static int max9288_set_power(struct v4l2_subdev *subdev, int on)
{
	return 0;
}
/* Initialize image sensors and set stream on registers */
static int max9288_set_stream(struct v4l2_subdev *subdev, int enable)
{
	struct max9288 *max = to_max_9286(subdev);
	struct i2c_client *client = v4l2_get_subdevdata(&max->v4l2_sd);
	int i, rval;
	unsigned int val;

	dev_err(max->v4l2_sd.dev, "max9288 set stream. enable = %d\n", enable);

	/* Disable I2C ACK */
	max_write_reg(max->regmap8_dser, DSER_I2C_CTRL, 0x36);

	/* hardcode here. */
	max->sensor_present = 0x01;
	for (i = 0; i < NR_OF_MAX9288_SINK_PADS; i++) {
		if (((0x01 << (i)) &  max->sensor_present) == 0)
			continue;

		/* Find the pad at the remote end of the link */
		struct media_pad *remote_pad = media_entity_remote_pad(&max->pad[i]);
		struct v4l2_subdev *sd;
		u8 bpp;

		if (!remote_pad)
			continue;

		/* Calls sensor set stream */
		sd = media_entity_to_v4l2_subdev(remote_pad->entity);
		rval = v4l2_subdev_call(sd, video, s_stream, enable);
		if (rval) {
			dev_err(max->v4l2_sd.dev,
				"Failed to set stream for %s. enable = %d\n",
				sd->name, enable);
			return rval;
		}
	}

	/* Enable I2C ACK */
	max_write_reg(max->regmap8_dser, DSER_I2C_CTRL, 0xB6);


	if (!enable) {
		max_write_reg(max->regmap8_ser, SER_MAIN_CTL, 0x43);
		msleep(5);
		return 0;
	}


	/* TODO: Revisit this delays */
	max_write_reg(max->regmap8_ser, SER_DBL_ALIGN, 0xC4);
	msleep(2);
	max_write_reg(max->regmap8_dser, DSER_AUDIO, 0x0f);
	msleep(2);
	max_write_reg(max->regmap8_dser, DSER_CSI_DATA, 0x23);
	msleep(2);
	max_write_reg(max->regmap8_dser, DSER_CSI_LANE_SEL, 0x47);
	msleep(2);
	max_write_reg(max->regmap8_dser, DSER_CSI_LANE_MAP, 0x00);
	msleep(2);
	max_write_reg(max->regmap8_ser, SER_MAIN_CTL, 0x87);
	msleep(2);
	rval = max_read_reg(max->regmap8_dser, DSER_CHNL_CTRL, &val);
	if (rval)
		dev_err(max->v4l2_sd.dev, "Failed to read LOCK!\n");
	else
		dev_err(max->v4l2_sd.dev, "max9288 channel lock reg: 0x%x \
				(lock = %d)\n", val, ((val & 0x80) >> 7));
	msleep(2);
	max_write_reg(max->regmap8_dser, DSER_AUTOPPL, 0xC0);

	dev_err(max->v4l2_sd.dev, "max9288: set stream exit...\n");

	return 0;
}

/* Get the media bus format */
static struct v4l2_mbus_framefmt *__max9288_get_ffmt(struct v4l2_subdev *subdev,
			 struct v4l2_subdev_pad_config *cfg,
			 unsigned int pad, unsigned int which,
			 unsigned int stream)
{
	struct max9288 *max = to_max_9286(subdev);

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(subdev, cfg, pad);
	else
		return &max->ffmts[pad][stream];
}

/* callback for VIDIOC_SUBDEV_G_FMT ioctl handler code */
static int max9288_get_format(struct v4l2_subdev *subdev,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *fmt)
{
	struct max9288 *max = to_max_9286(subdev);

	if (fmt->stream > max->nstreams)
		return -EINVAL;

	mutex_lock(&max->max_mutex);
	fmt->format = *__max9288_get_ffmt(subdev, cfg, fmt->pad, fmt->which, fmt->stream);
	mutex_unlock(&max->max_mutex);

	dev_dbg(subdev->dev, "subdev_format: which: %s, pad: %d, stream: %d.\n",
		 fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE ?
		 "V4L2_SUBDEV_FORMAT_ACTIVE" : "V4L2_SUBDEV_FORMAT_TRY",
		 fmt->pad, fmt->stream);

	dev_dbg(subdev->dev, "framefmt: width: %d, height: %d, code: 0x%x.\n",
		fmt->format.width, fmt->format.height, fmt->format.code);

	return 0;
}

/* Validate csi_data_format */
static const struct max9288_csi_data_format *max9288_validate_csi_data_format(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(max_csi_data_formats); i++) {
		if (max_csi_data_formats[i].code == code)
			return &max_csi_data_formats[i];
	}

	return &max_csi_data_formats[0];
}

/* callback for VIDIOC_SUBDEV_S_FMT ioctl handler code */
static int max9288_set_format(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *fmt)
{
	struct max9288 *max = to_max_9286(subdev);
	const struct max9288_csi_data_format *csi_format;
	struct v4l2_mbus_framefmt *ffmt;

	if (fmt->stream > max->nstreams)
		return -EINVAL;

	csi_format = max9288_validate_csi_data_format(fmt->format.code);

	mutex_lock(&max->max_mutex);
	ffmt = __max9288_get_ffmt(subdev, cfg, fmt->pad, fmt->which, fmt->stream);
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ffmt->width = fmt->format.width;
		ffmt->height = fmt->format.height;
		ffmt->code = csi_format->code;
	}

	fmt->format = *ffmt;
	mutex_unlock(&max->max_mutex);

	dev_err(subdev->dev, "framefmt: width: %d, height: %d, code: 0x%x.\n",
	       ffmt->width, ffmt->height, ffmt->code);

	return 0;
}

/* get the current low level media bus frame parameters */
static int max9288_get_frame_desc(struct v4l2_subdev *sd,
	unsigned int pad, struct v4l2_mbus_frame_desc *desc)
{
	struct max9288 *max = to_max_9286(sd);
	struct v4l2_mbus_frame_desc_entry *entry = desc->entry;
	u8 vc = 0;
	int i;

	desc->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	for (i = 0; i < min(max->nstreams, desc->num_entries); i++) {
		struct v4l2_mbus_framefmt *ffmt =
			&max->ffmts[i][MAX9288_PAD_SOURCE];
		const struct max9288_csi_data_format *csi_format =
			max9288_validate_csi_data_format(ffmt->code);

		entry->size.two_dim.width = ffmt->width;
		entry->size.two_dim.height = ffmt->height;
		entry->pixelcode = ffmt->code;
		entry->bus.csi2.channel = vc++;    // virtual channels
		entry->bpp = csi_format->compressed;
		entry++;
	}

	return 0;
}

/* Enumerate media bus formats available at a given sub-device pad */
static int max9288_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	struct max9288 *max = to_max_9286(sd);
	const uint32_t *supported_code = max9288_supported_codes[code->pad];
	bool next_stream = false;
	int i;

	if (code->stream & V4L2_SUBDEV_FLAG_NEXT_STREAM) {
		next_stream = true;
		code->stream &= ~V4L2_SUBDEV_FLAG_NEXT_STREAM;
	}

	if (code->stream > max->nstreams)
		return -EINVAL;

	if (next_stream) {
		if (!(max->pad[code->pad].flags & MEDIA_PAD_FL_MULTIPLEX))
			return -EINVAL;
		if (code->stream < max->nstreams - 1) {
			code->stream++;
			return 0;
		} else {
			return -EINVAL;
		}
	}

	for (i = 0; supported_code[i]; i++) {
		if (i == code->index) {
			code->code = supported_code[i];
			return 0;
		}
	}

	return -EINVAL;
}

/* Configure Media Controller routing */
static int max9288_set_routing(struct v4l2_subdev *sd,
				   struct v4l2_subdev_routing *route)
{
	struct max9288 *max = to_max_9286(sd);
	int i, j, ret = 0;

	for (i = 0; i < min(route->num_routes, max->nstreams); ++i) {
		struct v4l2_subdev_route *t = &route->routes[i];
		unsigned int sink = t->sink_pad;
		unsigned int source = t->source_pad;

		if (t->sink_stream > max->nstreams - 1 ||
		    t->source_stream > max->nstreams - 1)
			continue;

		if (t->source_pad != MAX9288_PAD_SOURCE)
			continue;

		for (j = 0; j < max->nstreams; j++) {
			if (sink == max->route[j].sink &&
				source == max->route[j].source)
				break;
		}

		if (j == max->nstreams)
			continue;

		max->stream[sink].stream_id[0] = t->sink_stream;
		max->stream[source].stream_id[sink] = t->source_stream;

		if (t->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE)
			max->route[j].flags |=
				V4L2_SUBDEV_ROUTE_FL_ACTIVE;
		else if (!(t->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			max->route[j].flags &=
				(~V4L2_SUBDEV_ROUTE_FL_ACTIVE);
	}

	return ret;
}

/* Configure Media Controller routing */
static int max9288_get_routing(struct v4l2_subdev *sd,
				   struct v4l2_subdev_routing *route)
{
	struct max9288 *max = to_max_9286(sd);
	int i;

	for (i = 0; i < min(max->nstreams, route->num_routes); ++i) {
		unsigned int sink = max->route[i].sink;
		unsigned int source = max->route[i].source;

		route->routes[i].sink_pad = sink;
		route->routes[i].sink_stream =
			max->stream[sink].stream_id[0];
		route->routes[i].source_pad = source;
		route->routes[i].source_stream =
			max->stream[source].stream_id[sink];
		route->routes[i].flags = max->route[i].flags;
	}

	route->num_routes = i;

	return 0;
}

/* called when the subdev device node is opened by an application */
static int max9288_open(struct v4l2_subdev *subdev,
				struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(subdev, fh->pad, 0);

	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = MAX9288_PAD_SOURCE,
		.format = {
			.width = MAX9288_MAX_WIDTH,
			.height = MAX9288_MAX_HEIGHT,
			.code = MEDIA_BUS_FMT_YUYV8_1X16,
		},
		.stream = 0,
	};

	*try_fmt = fmt.format;

	return 0;
}

/*
 * called when this subdev is registered.
 * When called the v4l2_dev field is set to the correct v4l2_device
*/
static int max9288_registered(struct v4l2_subdev *subdev)
{
	struct max9288 *max = to_max_9286(subdev);
	int i, j, k, l, rval;
	printk(KERN_CRIT "%s: %d\r\n", __func__, __LINE__);
	for (i = 0, k = 0; i < max->pdata->subdev_num; i++) {
		struct max9288_subdev_i2c_info *info =
			&max->pdata->subdev_info[i];
		struct i2c_adapter *adapter;

		if (k >= max->nsinks)
			break;

		adapter = i2c_get_adapter(info->i2c_adapter_id);
		max->sub_devs[k] = v4l2_i2c_new_subdev_board(
			max->v4l2_sd.v4l2_dev, adapter,
			&info->board_info, 0);
		i2c_put_adapter(adapter);
		if (!max->sub_devs[k]) {
			dev_err(max->v4l2_sd.dev,
				"can't create new i2c subdev %d-%04x\n",
				info->i2c_adapter_id,
				info->board_info.addr);
			continue;
		}

		for (j = 0; j < max->sub_devs[k]->entity.num_pads; j++) {
			if (max->sub_devs[k]->entity.pads[j].flags &
				MEDIA_PAD_FL_SOURCE)
				break;
		}

		if (j == max->sub_devs[k]->entity.num_pads) {
			dev_warn(max->v4l2_sd.dev,
				"no source pad in subdev %d-%04x\n",
				info->i2c_adapter_id,
				info->board_info.addr);
			return -ENOENT;
		}

		for (l = 0; l < max->nsinks; l++) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
			rval = media_entity_create_link(
#else
			rval = media_create_pad_link(
#endif
				&max->sub_devs[k]->entity, j,
				&max->v4l2_sd.entity, l, 0);
			if (rval) {
				dev_err(max->v4l2_sd.dev,
					"can't create link to %d-%04x\n",
					info->i2c_adapter_id,
					info->board_info.addr);
				return -EINVAL;
			}
		}
		k++;
	}
	printk(KERN_CRIT "%s: %d\r\n", __func__, __LINE__);
   max9288_entity_status = 1;
   sysfs_notify((struct kobject *)&max->v4l2_sd.dev, NULL, "max9288_entity_created");

	return 0;
}

static const struct v4l2_subdev_core_ops max9288_core_subdev_ops = {
	.s_power = max9288_set_power,
};

static const struct v4l2_subdev_video_ops max9288_sd_video_ops = {
	.s_stream = max9288_set_stream,
};

static const struct v4l2_subdev_pad_ops max9288_sd_pad_ops = {
	.get_fmt = max9288_get_format,
	.set_fmt = max9288_set_format,
	.get_frame_desc = max9288_get_frame_desc,
	.enum_mbus_code = max9288_enum_mbus_code,
	.set_routing = max9288_set_routing,
	.get_routing = max9288_get_routing,
};

static struct v4l2_subdev_ops max9288_sd_ops = {
	.core = &max9288_core_subdev_ops,
	.video = &max9288_sd_video_ops,
	.pad = &max9288_sd_pad_ops,
};

static struct v4l2_subdev_internal_ops max9288_sd_internal_ops = {
	.open = max9288_open,
	.registered = max9288_registered,
};

static int max9288_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops max9288_ctrl_ops = {
	.s_ctrl = max9288_s_ctrl,
};

static const s64 max9288_op_sys_clock[] =  {418824000, };
static const struct v4l2_ctrl_config max9288_controls[] = {
	{
		.ops = &max9288_ctrl_ops,
		.id = V4L2_CID_LINK_FREQ,
		.name = "V4L2_CID_LINK_FREQ",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.max = ARRAY_SIZE(max9288_op_sys_clock) - 1,
		.min = 0,
		.step = 0,
		.def = 0,
		.qmenu_int = max9288_op_sys_clock,
	},
};

static int max9288_register_subdev(struct max9288 *max)
{
	int i, rval;
	struct i2c_client *client = v4l2_get_subdevdata(&max->v4l2_sd);

	v4l2_subdev_init(&max->v4l2_sd, &max9288_sd_ops);
	snprintf(max->v4l2_sd.name, sizeof(max->v4l2_sd.name), "max9288 %d-%4.4x",
		i2c_adapter_id(client->adapter), client->addr);
	max->v4l2_sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			V4L2_SUBDEV_FL_HAS_SUBSTREAMS;

	max->v4l2_sd.internal_ops = &max9288_sd_internal_ops;

	v4l2_set_subdevdata(&max->v4l2_sd, client);

	v4l2_ctrl_handler_init(&max->ctrl_handler,
				ARRAY_SIZE(max9288_controls));

	if (max->ctrl_handler.error) {
		dev_err(max->v4l2_sd.dev,
			"Failed to init max9288 controls. ERR: %d!\n",
			max->ctrl_handler.error);
		return max->ctrl_handler.error;
	}

	max->v4l2_sd.ctrl_handler = &max->ctrl_handler;

	for (i = 0; i < ARRAY_SIZE(max9288_controls); i++) {
		const struct v4l2_ctrl_config *cfg =
			&max9288_controls[i];
		struct v4l2_ctrl *ctrl;

		ctrl = v4l2_ctrl_new_custom(&max->ctrl_handler, cfg, NULL);
		if (!ctrl) {
			dev_err(max->v4l2_sd.dev,
				"Failed to create ctrl %s!\n", cfg->name);
			rval = max->ctrl_handler.error;
			goto failed_out;
		}
	}

	max->link_freq = v4l2_ctrl_find(&max->ctrl_handler, V4L2_CID_LINK_FREQ);
	max->test_pattern = v4l2_ctrl_find(&max->ctrl_handler,
					  V4L2_CID_TEST_PATTERN);

	for (i = 0; i < max->nsinks; i++)
		max->pad[i].flags = MEDIA_PAD_FL_SINK;
	max->pad[MAX9288_PAD_SOURCE].flags =
		MEDIA_PAD_FL_SOURCE | MEDIA_PAD_FL_MULTIPLEX;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	rval = media_entity_init(&max->v4l2_sd.entity, NR_OF_MAX9288_PADS, max->pad, 0);
#else
	rval = media_entity_pads_init(&max->v4l2_sd.entity, NR_OF_MAX9288_PADS, max->pad);
#endif

	if (rval) {
		dev_err(max->v4l2_sd.dev,
			"Failed to init media entity for max9288!\n");
		goto failed_out;
	}

	return 0;
failed_out:
	v4l2_ctrl_handler_free(&max->ctrl_handler);
	return rval;
}


static ssize_t max9288_entity_created_show(struct device *dev,
		                               struct device_attribute *attr,
		                               char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d", max9288_entity_status);

}

static DEVICE_ATTR(max9288_entity_created, S_IRUGO,
					                           max9288_entity_created_show, NULL);

static struct attribute *max9288_attributes[] = {
	   &dev_attr_max9288_entity_created.attr,
	   NULL
};


static const struct attribute_group max9288_attr_group = {
	   .attrs = max9288_attributes,
};



/* max9288 initialization */
static int max9288_init(struct max9288 *max, struct i2c_client *client)
{
	int rval;
	unsigned int val;
	unsigned int reset_gpio = max->pdata->reset_gpio;

	if (devm_gpio_request_one(max->v4l2_sd.dev, reset_gpio,
							 GPIOF_DIR_OUT | GPIOF_INIT_HIGH,
										"max9288 reset") != 0) {
	       dev_err(max->v4l2_sd.dev, "Unable to acquire gpio %d\n", reset_gpio);
	}

	gpio_set_value(reset_gpio, 1);
	usleep_range(5000, 5000);
	gpio_set_value(reset_gpio, 0);
	usleep_range(5000, 5000);
	gpio_set_value(reset_gpio, 1);
	usleep_range(5000, 5000);
	dev_err(max->v4l2_sd.dev, "Reset MAX9288 done.\n");

	/* Get the device ID of Deserializer */
	rval = max_read_reg(max->regmap8_dser, DSER_MAX9288_DEVID, &val);
	if (rval) {
		dev_err(max->v4l2_sd.dev, "Failed to read device ID of max9288!\n");
		return rval;
	}
	dev_err(max->v4l2_sd.dev, "max9288 device ID: 0x%x\n", val);

	/* TODO: Revisit delays */
	msleep(2);
	max_write_reg(max->regmap8_ser, SER_MAIN_CTL, 0x47);
	msleep(5);

	rval = max_write_reg(max->regmap8_dser, DSER_BWS, 0x1C);
	if (rval)
		dev_err(max->v4l2_sd.dev, "Failed to write max9288 reg: 0x17!\n");
	msleep(5);
	max_write_reg(max->regmap8_ser, SER_MAIN_CTL, 0x47);
	msleep(5);
	max_write_reg(max->regmap8_ser, SER_CONFIG, 0x80);
	msleep(2);
	max_write_reg(max->regmap8_ser, SER_GPIO_OUT, 0xbf);
	msleep(2);

	/* Setup crossbars.. */
	max_write_regs(max->regmap8_ser, crossbar_settings,
						ARRAY_SIZE(crossbar_settings));
	msleep(2);
	max_write_reg(max->regmap8_ser, SER_I2C_SOURCE_IS,
						(ADDR_AR0143_RVC_BASE + 1) << 1);
	msleep(2);
	max_write_reg(max->regmap8_ser, SER_I2C_DST_IS,
						ADDR_AR0143_SENSOR << 1);
	msleep(2);

	sysfs_create_group(&client->dev.kobj, &max9288_attr_group);
	return 0;
}

/* Unbind the max9288 device driver from the I2C client */
static int max9288_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct max9288 *max = to_max_9286(subdev);
	int i;

	mutex_destroy(&max->max_mutex);
	v4l2_ctrl_handler_free(&max->ctrl_handler);
	v4l2_device_unregister_subdev(&max->v4l2_sd);
	media_entity_cleanup(&max->v4l2_sd.entity);

	for (i = 0; i < NR_OF_MAX9288_SINK_PADS; i++) {
		if (max->sub_devs[i]) {
			struct i2c_client *sub_client =
				v4l2_get_subdevdata(max->sub_devs[i]);

			i2c_unregister_device(sub_client);
		}
		max->sub_devs[i] = NULL;
	}

	return 0;
}

static int max9288_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
	struct max9288 *max;
	int i, rval;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "No platform data.\n");
		return -ENODEV;
	}

	max = devm_kzalloc(&client->dev, sizeof(*max), GFP_KERNEL);
	if (!max) {
		dev_err(&client->dev, "Allocation failed.\n");
		return -ENODEV;
	}

	max->pdata = client->dev.platform_data;

	max->nsources = NR_OF_MAX9288_SOURCE_PADS;
	max->nsinks = NR_OF_MAX9288_SINK_PADS;
	max->npads = NR_OF_MAX9288_PADS;
	max->nstreams = NR_OF_MAX9288_STREAMS;

	max->crop = devm_kcalloc(&client->dev, max->npads, sizeof(struct v4l2_rect), GFP_KERNEL);
	max->compose = devm_kcalloc(&client->dev, max->npads, sizeof(struct v4l2_rect), GFP_KERNEL);
	max->route = devm_kcalloc(&client->dev, max->nstreams, sizeof(*max->route), GFP_KERNEL);
	max->stream = devm_kcalloc(&client->dev, max->npads, sizeof(*max->stream), GFP_KERNEL);

	if (!max->crop || !max->compose || !max->route || !max->stream)
		return -ENOMEM;

	for (i = 0; i < max->npads; i++) {
		max->ffmts[i] = devm_kcalloc(&client->dev, max->nstreams,
					sizeof(struct v4l2_mbus_framefmt), GFP_KERNEL);
		if (!max->ffmts[i])
			return -ENOMEM;

		max->stream[i].stream_id = devm_kcalloc(&client->dev, max->nsinks,
						sizeof(*max->stream[i].stream_id), GFP_KERNEL);
		if (!max->stream[i].stream_id)
			return -ENOMEM;
	}

	for (i = 0; i < max->nstreams; i++) {
		max->route[i].sink = i;
		max->route[i].source = MAX9288_PAD_SOURCE;
		max->route[i].flags = 0;
	}

	for (i = 0; i < max->nsinks; i++) {
		max->stream[i].stream_id[0] = i;
		max->stream[MAX9288_PAD_SOURCE].stream_id[i] = i;
	}

	max->regmap8_dser = devm_regmap_init_i2c(client,
					   &max9288_reg_config8);
	if (IS_ERR(max->regmap8_dser)) {
		dev_err(&client->dev, "Failed to init regmap8_dser!\n");
		return -EIO;
	}


	max->client_ser = i2c_new_dummy(client->adapter,
								  SER_ADDR_MAX96705);
	if (IS_ERR_OR_NULL(max->client_ser)) {
		dev_err(&client->dev, "failed to add serializer I2C device\n");
		return -ENODEV;
	}

	max->regmap8_ser = devm_regmap_init_i2c(max->client_ser,
										&max96705_reg_config8);
	if (IS_ERR(max->regmap8_ser)) {
		dev_err(&client->dev, "Failed to init regmap8_ser!\n");
		return -EIO;
	}

	mutex_init(&max->max_mutex);
	v4l2_i2c_subdev_init(&max->v4l2_sd, client, &max9288_sd_ops);

	rval = max9288_register_subdev(max);
	if (rval) {
		dev_err(&client->dev, "Failed to register max9288 subdevice!\n");
		return rval;
	}

	rval = max9288_init(max, client);
	if (rval) {
		dev_err(&client->dev, "Failed to initialise max9288!\n");
		return rval;
	}

	return 0;
}

static const struct i2c_device_id max9288_id_table[] = {
	{ MAX9288_NAME, 0 },
	{ },
};

static struct i2c_driver max9288_i2c_driver = {
	.driver = {
		.name = MAX9288_NAME,
	},
	.probe = max9288_probe,
	.remove = max9288_remove,
	.id_table = max9288_id_table,
};

module_i2c_driver(max9288_i2c_driver);

MODULE_LICENSE("GPL");
