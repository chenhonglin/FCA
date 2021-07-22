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
#include <media/max9296.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>

#include "max9296-reg.h"
#include "max96705-reg.h"

struct max9296 {
	struct v4l2_subdev v4l2_sd;   // add v4l2 sub dev support to i2c driver
	struct max9296_pdata *pdata;
	struct media_pad pad[NR_OF_MAX_PADS];
	unsigned char sensor_present;
	unsigned int total_sensor_num;
	unsigned int nsources;
	unsigned int nsinks;
	unsigned int npads;
	unsigned int nstreams;
	const char *name;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_subdev *sub_devs[NR_OF_MAX_SINK_PADS];
	struct v4l2_mbus_framefmt *ffmts[NR_OF_MAX_PADS];
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
	struct regmap *regmap16;
	struct mutex max_mutex;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *test_pattern;
};

struct max9296 *gMax = NULL;

struct max_reg_blk {
	unsigned int reg;
	unsigned int val;
};

/* Macro to get access to MAX9296 structure using the v4l2_subdev structure */
#define to_max_9296(_sd) container_of(_sd, struct max9296, v4l2_sd)
extern char *harman_get_board_revision(void);
atomic_t status = ATOMIC_INIT(0);

/*
 * Order matters.
 *
 * 1. Bits-per-pixel, descending.
 * 2. Bits-per-pixel compressed, descending.
 * 3. Pixel order, same as in pixel_order_str. Formats for all four pixel
 *    orders must be defined.
 */
static const struct max9296_csi_data_format max_csi_data_formats[] = {
	{MEDIA_BUS_FMT_YUYV8_1X16, 16, 16, PIXEL_ORDER_GBRG, 0x1e},
	{MEDIA_BUS_FMT_UYVY8_1X16, 16, 16, PIXEL_ORDER_GBRG, 0x1e},
};

static const uint32_t max9296_supported_codes_pad[] = {
	MEDIA_BUS_FMT_YUYV8_1X16,
	MEDIA_BUS_FMT_UYVY8_1X16,
	0,
};

static const uint32_t *max9296_supported_codes[] = {
	max9296_supported_codes_pad,
};

static struct regmap_config max9296_reg_config16 = {
	.reg_bits = 16,
	.val_bits = 8,
};


static struct max_reg_blk crossbar_settings[] = {
	{0x20, 0x13},
	{0x21, 0x14},
	{0x22, 0x15},
	{0x23, 0x16},
	{0x24, 0x17},
	{0x27, 0x0E},
	{0x28, 0x0F},
	{0x30, 0x00},
	{0x31, 0x01},
	{0x32, 0x02},
	{0x33, 0x03},
	{0x34, 0x04},
	{0x35, 0x05},
	{0x36, 0x06},
	{0x37, 0x07},
	{0x38, 0x10},
	{0x39, 0x11},
	{0x3A, 0x12},
};

int max9296_check_sync_status(void) {
        /*
         * Poll frame synchronization bit of deserializer
         * All the cameras should work in SYNC mode
         * MAX9296 sends a pulse to each camera, then each camera sends
         * out one frame.  The VSYNC for each camera should appear in
         * almost same time for the deserializer to lock FSYNC
         */

	unsigned int val;
	int rval;

	if (atomic_read(&status)) {
		rval = regmap_read(gMax->regmap16, DS_LINKA_LOCKED, &val);
		if (rval) {
			printk("max9296_check_sync_status : Read LinkA status error!\n");
		} else if (val & 0x01) {
			printk("max9296_check_sync_status : Deserializer LinkA locked (value = 0x%02x)\n", val);
			return 1;
		} else
			printk("max9296_check_sync_status : value @0x%x = 0x%x\n", DS_LINKA_LOCKED, val);
	} else
		return -EAGAIN;
	return 0;
}
EXPORT_SYMBOL(max9296_check_sync_status);

int  max96705_write_register(struct max9296 *max,
					unsigned int i, u8 reg, u8 val)
{
	int ret;
	int retry, timeout = 50;
	struct i2c_client *client = v4l2_get_subdevdata(&max->v4l2_sd);

	client->addr = S_ADDR_MAX96705 + i;
	for (retry = 0; retry < timeout; retry++) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (val < 0)
			usleep_range(10000, 20000);
		else
			break;
	}

	if (retry >= timeout) {
		dev_err(max->v4l2_sd.dev,
			"%s:write reg failed: reg=%2x\n", __func__, reg);
		return -1;
	}
	client->addr = DS_ADDR_MAX9296;

	return 0;
}

/* Serializer register read */
static inline int max96705_read_register(struct max9296 *max,
						unsigned int i, u8 reg)
{
	int val;
	int retry, timeout = 50;
	struct i2c_client *client = v4l2_get_subdevdata(&max->v4l2_sd);

	client->addr = S_ADDR_MAX96705 + i;
	for (retry = 0; retry < timeout; retry++) {
		val = i2c_smbus_read_byte_data(client, reg);
		if (val < 0)
			usleep_range(10000, 20000);
		else
			break;
	}

	if (retry >= timeout) {
		dev_err(max->v4l2_sd.dev,
			"%s:read reg failed: reg=%2x\n", __func__, reg);
		return -1;
	}
	client->addr = DS_ADDR_MAX9296;

	return val;
}

/* Initialize image sensors and set stream on registers */
static int max9296_set_stream(struct v4l2_subdev *subdev, int enable)
{
	struct max9296 *max = to_max_9296(subdev);
	struct i2c_client *client = v4l2_get_subdevdata(&max->v4l2_sd);
	int i, rval;
	unsigned int val, reg_val;
	unsigned int pix_count_reg, pix_count_low, pix_count_high;

	dev_err(max->v4l2_sd.dev, "MAX9296 set stream. enable = %d\n", enable);

	atomic_set(&status, 0);
	rval = regmap_write(max->regmap16, DS_I2CLOCACK_LINKA, 0x00);
	if (rval) {
		dev_err(max->v4l2_sd.dev, "Failed to disable I2C ACK!\n");
		atomic_set(&status, 1);
		return rval;
	}

	/* hardcode here. */
	max->sensor_present = 0x01;

	for (i = 0; i < NR_OF_MAX_SINK_PADS; i++) {
		if (((0x01 << (i)) &  max->sensor_present) == 0)
			continue;

		/* Find the pad at the remote end of the link */
		struct media_pad *remote_pad =
				media_entity_remote_pad(&max->pad[i]);
		struct v4l2_subdev *sd;
		u8 bpp;

		if (!remote_pad)
			continue;

		/* Calls sensor set stream */
		sd = media_entity_to_v4l2_subdev(remote_pad->entity);
		dev_err(max->v4l2_sd.dev, "Call sensor %s to set stream\n",
								sd->name);
		rval = v4l2_subdev_call(sd, video, s_stream, enable);
		if (rval) {
			dev_err(max->v4l2_sd.dev,
				"Failed to set stream for %s. enable = %d\n",
				sd->name, enable);
			atomic_set(&status, 1);
			return rval;
		}
	}

	/* Enable I2C ACK*/
	rval = regmap_write(max->regmap16, DS_I2CLOCACK_LINKA, 0x80);
	if (rval) {
		dev_err(max->v4l2_sd.dev, "Failed to enable I2C ACK!\n");
		atomic_set(&status, 1);
		return rval;
	}

	if (!enable) {

		max96705_write_register(max,
			S_ADDR_MAX96705_BROADCAST - S_ADDR_MAX96705,
				S_MAIN_CTL, 0x43);
		goto exit_stream;
	}

	max96705_write_register(max,
		S_ADDR_MAX96705_BROADCAST - S_ADDR_MAX96705, S_MAIN_CTL, 0x43);
	msleep(250);

	/* Check if valid PCLK is available for the links */
	for (i = 1; i <= NR_OF_MAX_SINK_PADS; i++) {
		if (((0x01 << (i-1)) &  max->sensor_present) == 0)
			continue;

		val = max96705_read_register(max, i, S_INPUT_STATUS);
		if (val != -1 && (val & 0x01)) {
			dev_dbg(max->v4l2_sd.dev,
				"Valid PCLK detected for link %d\n", i - 1);
		} else if (val == -1) {
			dev_err(max->v4l2_sd.dev,
				"Failed to read max96705 PCLK register 0x%02x for link %d\n",
				0x15, i - 1);
		} else {
			dev_err(max->v4l2_sd.dev, "No PCLK for link %d, value @0x%02x = 0x%02x\n",
				i - 1, S_INPUT_STATUS, val);
		}
	}

	/* Enable serial links */
	max96705_write_register(max,
		S_ADDR_MAX96705_BROADCAST - S_ADDR_MAX96705, S_MAIN_CTL, 0x83);
	/* Wait for more than 2 Frames time from each sensor */
	msleep(250); // I2C is unavailable when GMSL locks

	/*
	 * Poll frame synchronization bit of deserializer
	 * All the cameras should work in SYNC mode
	 * MAX9296 sends a pulse to each camera, then each camera sends
	 * out one frame.  The VSYNC for each camera should appear in
	 * almost same time for the deserializer to lock FSYNC
	 */

	rval = regmap_read(max->regmap16, DS_LINKA_LOCKED, &val);
	if (rval) {
		dev_err(max->v4l2_sd.dev, "Read LinkA status error!\n");
		atomic_set(&status, 1);
		return rval;
	} else if (val & 0x01) {
		dev_dbg(max->v4l2_sd.dev,
			"Deserializer LinkA locked (value = 0x%02x)\n", val);
	} else {
		dev_dbg(max->v4l2_sd.dev,
				"value @0x%x = 0x%x\n", DS_LINKA_LOCKED, val);
	}


	rval = regmap_read(max->regmap16, DS_PIPEX_LOCKED, &val);
	if (rval) {
		dev_err(max->v4l2_sd.dev, "Read Pipe X status error!\n");
		atomic_set(&status, 1);
		return rval;
	} else if (val & 0x01) {
		dev_dbg(max->v4l2_sd.dev,
				"Pipe X locked (value = 0x%02x)\n", val);
	} else {
		dev_dbg(max->v4l2_sd.dev,
				"Pipe X value @0x%x = 0x%x\n", 0x1dc, val);
	}

exit_stream:
	atomic_set(&status, 1);
	dev_err(max->v4l2_sd.dev, "%s: exiting. streaming %s\n",
			__func__, enable ? "enabled" : "disabled");

	return 0;
}

/* Get the media bus format */
static struct v4l2_mbus_framefmt *__max9296_get_ffmt(struct v4l2_subdev *subdev,
			 struct v4l2_subdev_pad_config *cfg,
			 unsigned int pad, unsigned int which,
			 unsigned int stream)
{
	struct max9296 *max = to_max_9296(subdev);

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(subdev, cfg, pad);
	else
		return &max->ffmts[pad][stream];
}

/* callback for VIDIOC_SUBDEV_G_FMT ioctl handler code */
static int max9296_get_format(struct v4l2_subdev *subdev,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *fmt)
{
	struct max9296 *max = to_max_9296(subdev);

	if (fmt->stream > max->nstreams)
		return -EINVAL;

	mutex_lock(&max->max_mutex);
	fmt->format = *__max9296_get_ffmt(subdev, cfg, fmt->pad,
						fmt->which, fmt->stream);
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
static const struct max9296_csi_data_format
			*max9296_validate_csi_data_format(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(max_csi_data_formats); i++) {
		if (max_csi_data_formats[i].code == code)
			return &max_csi_data_formats[i];
	}

	return &max_csi_data_formats[0];
}

/* callback for VIDIOC_SUBDEV_S_FMT ioctl handler code */
static int max9296_set_format(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *fmt)
{
	struct max9296 *max = to_max_9296(subdev);
	const struct max9296_csi_data_format *csi_format;
	struct v4l2_mbus_framefmt *ffmt;

	if (fmt->stream > max->nstreams)
		return -EINVAL;

	csi_format = max9296_validate_csi_data_format(fmt->format.code);

	mutex_lock(&max->max_mutex);
	ffmt = __max9296_get_ffmt(subdev, cfg, fmt->pad,
					fmt->which, fmt->stream);
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
static int max9296_get_frame_desc(struct v4l2_subdev *sd,
	unsigned int pad, struct v4l2_mbus_frame_desc *desc)
{
	struct max9296 *max = to_max_9296(sd);
	struct v4l2_mbus_frame_desc_entry *entry = desc->entry;
	u8 vc = 0;
	int i;

	desc->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	for (i = 0; i < min(max->nstreams, desc->num_entries); i++) {
		struct v4l2_mbus_framefmt *ffmt =
			&max->ffmts[i][MAX_PAD_SOURCE];
		const struct max9296_csi_data_format *csi_format =
			max9296_validate_csi_data_format(ffmt->code);

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
static int max9296_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	struct max9296 *max = to_max_9296(sd);
	const uint32_t *supported_code = max9296_supported_codes[code->pad];
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
static int max9296_set_routing(struct v4l2_subdev *sd,
				   struct v4l2_subdev_routing *route)
{
	struct max9296 *max = to_max_9296(sd);
	int i, j, ret = 0;

	for (i = 0; i < min(route->num_routes, max->nstreams); ++i) {
		struct v4l2_subdev_route *t = &route->routes[i];
		unsigned int sink = t->sink_pad;
		unsigned int source = t->source_pad;

		if (t->sink_stream > max->nstreams - 1 ||
		    t->source_stream > max->nstreams - 1)
			continue;

		if (t->source_pad != MAX_PAD_SOURCE)
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
static int max9296_get_routing(struct v4l2_subdev *sd,
				   struct v4l2_subdev_routing *route)
{
	struct max9296 *max = to_max_9296(sd);
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
static int max9296_open(struct v4l2_subdev *subdev,
				struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(subdev, fh->pad, 0);

	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = MAX_PAD_SOURCE,
		.format = {
			.width = MAX9296_MAX_WIDTH,
			.height = MAX9296_MAX_HEIGHT,
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
static int max9296_registered(struct v4l2_subdev *subdev)
{
	struct max9296 *max = to_max_9296(subdev);
	int i, j, k, l, rval, rvalue;

	for (i = 0, k = 0; i < max->pdata->subdev_num; i++) {
		struct max9296_subdev_i2c_info *info =
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

	return 0;
}

static int max9296_set_power(struct v4l2_subdev *subdev, int on)
{
	return 0;
}

static const struct v4l2_subdev_core_ops max9296_core_subdev_ops = {
	.s_power = max9296_set_power,
};

static const struct v4l2_subdev_video_ops max9296_sd_video_ops = {
	/* used to notify the driver that the video stream will
	 * start or has stopped
	 */
	.s_stream = max9296_set_stream,
};

static const struct v4l2_subdev_pad_ops max9296_sd_pad_ops = {
	.get_fmt = max9296_get_format,
	.set_fmt = max9296_set_format,
	.get_frame_desc = max9296_get_frame_desc,
	.enum_mbus_code = max9296_enum_mbus_code,
	.set_routing = max9296_set_routing,
	.get_routing = max9296_get_routing,
};

static struct v4l2_subdev_ops max9296_sd_ops = {
	.core = &max9296_core_subdev_ops,
	.video = &max9296_sd_video_ops,
	.pad = &max9296_sd_pad_ops,
};

static struct v4l2_subdev_internal_ops max9296_sd_internal_ops = {
	.open = max9296_open,
	.registered = max9296_registered,
};

static int max9296_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops max9296_ctrl_ops = {
	.s_ctrl = max9296_s_ctrl,
};

static const s64 max9296_op_sys_clock[] =  {200000000,};
static const struct v4l2_ctrl_config max9296_controls[] = {
	{
		.ops = &max9296_ctrl_ops,
		.id = V4L2_CID_LINK_FREQ,
		.name = "V4L2_CID_LINK_FREQ",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.max = ARRAY_SIZE(max9296_op_sys_clock) - 1,
		.min = 0,
		.step = 0,
		.def = 0,
		.qmenu_int = max9296_op_sys_clock,
	},
};

/* Registers MAX9296 sub-devices (Image sensors) */
static int max9296_register_subdev(struct max9296 *max)
{
	int i, rval;
	struct i2c_client *client = v4l2_get_subdevdata(&max->v4l2_sd);

	/* subdevice driver initializes v4l2 subdev */
	v4l2_subdev_init(&max->v4l2_sd, &max9296_sd_ops);
	snprintf(max->v4l2_sd.name, sizeof(max->v4l2_sd.name),
		"max9296 %d-%4.4x", i2c_adapter_id(client->adapter),
			client->addr);
	max->v4l2_sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			V4L2_SUBDEV_FL_HAS_SUBSTREAMS;

	max->v4l2_sd.internal_ops = &max9296_sd_internal_ops;

	v4l2_set_subdevdata(&max->v4l2_sd, client);

	v4l2_ctrl_handler_init(&max->ctrl_handler,
				ARRAY_SIZE(max9296_controls));

	if (max->ctrl_handler.error) {
		dev_err(max->v4l2_sd.dev,
			"Failed to init max9296 controls. ERR: %d!\n",
			max->ctrl_handler.error);
		return max->ctrl_handler.error;
	}

	max->v4l2_sd.ctrl_handler = &max->ctrl_handler;

	for (i = 0; i < ARRAY_SIZE(max9296_controls); i++) {
		const struct v4l2_ctrl_config *cfg =
			&max9296_controls[i];
		struct v4l2_ctrl *ctrl;

		ctrl = v4l2_ctrl_new_custom(&max->ctrl_handler, cfg, NULL);
		if (!ctrl) {
			dev_err(max->v4l2_sd.dev,
				"Failed to create ctrl %s!, err: %d\n",
					cfg->name, max->ctrl_handler.error);
			rval = max->ctrl_handler.error;
			goto failed_out;
		}
	}

	max->link_freq = v4l2_ctrl_find(&max->ctrl_handler, V4L2_CID_LINK_FREQ);
	max->test_pattern = v4l2_ctrl_find(&max->ctrl_handler,
					  V4L2_CID_TEST_PATTERN);

	for (i = 0; i < max->nsinks; i++)
		max->pad[i].flags = MEDIA_PAD_FL_SINK;
	max->pad[MAX_PAD_SOURCE].flags =
		MEDIA_PAD_FL_SOURCE | MEDIA_PAD_FL_MULTIPLEX;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	rval = media_entity_init(&max->v4l2_sd.entity,
					NR_OF_MAX_PADS, max->pad, 0);
#else
	rval = media_entity_pads_init(&max->v4l2_sd.entity,
					NR_OF_MAX_PADS, max->pad);
#endif
	if (rval) {
		dev_err(max->v4l2_sd.dev,
			"Failed to init media entity for max9296!\n");
		goto failed_out;
	}

	return 0;
failed_out:
	v4l2_ctrl_handler_free(&max->ctrl_handler);
	return rval;
}

/* MAX9296 initial setup and Reverse channel setup */
static int max9296_init(struct max9296 *max, struct i2c_client *client)
{
	int rval, i;
	unsigned int val;
	char *hw_id;

#ifndef CONFIG_BJEVN60_VIRTUAL
	unsigned int reset_gpio = max->pdata->reset_gpio;

	if (devm_gpio_request_one(max->v4l2_sd.dev, reset_gpio,
		GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "max9296 reset") != 0) {
		dev_err(max->v4l2_sd.dev, "Unable to acquire gpio %d\n",
								reset_gpio);
	}

	gpio_set_value(reset_gpio, 1);
	usleep_range(5000, 10000);
	gpio_set_value(reset_gpio, 0);
	usleep_range(5000, 10000);
	gpio_set_value(reset_gpio, 1);
	usleep_range(5000, 10000);
	dev_dbg(max->v4l2_sd.dev, "Reset max9296 done.\n");
#endif

	/* Get the device ID of Deserializer */
	rval = regmap_read(max->regmap16, 0x0D, &val);
	if (rval) {
		dev_err(max->v4l2_sd.dev, "Failed to read device ID of max9296!\n");
		return rval;
	}
	dev_err(max->v4l2_sd.dev, "max9296 device ID: 0x%x\n", val);

	hw_id = harman_get_board_revision();
	dev_err(max->v4l2_sd.dev, "camera hw_id:%s\n", hw_id);
	if ((!strncmp(hw_id, "ED2", 3)) || (!strncmp(hw_id, "ED3", 3))) {
		dev_err(max->v4l2_sd.dev, " camera change to GMS1");
		regmap_write(max->regmap16, 0x0006, 0x1F);
		msleep(5);
		regmap_write(max->regmap16, 0x0010, 0x31);
		msleep(200);
		regmap_write(max->regmap16, 0x0B06, 0xEF);
		msleep(5);
		regmap_write(max->regmap16, 0x0B07, 0x04);
		msleep(5);
	}

	/* Hardware configuration Change GMSL2 to GMSL1 */
	//regmap_write(max->regmap16, 0x06, 0x9f);

	/* Enable Auto ACK */
	regmap_write(max->regmap16, 0x0B0D, 0x80);
	usleep_range(2000, 5000);

	/* Enable configuration link, CLINKEN=1, SERNEN=0 */
	max96705_write_register(max, 0, S_MAIN_CTL, 0x43);
	msleep(100);

	/* Enable DBL, Falling Edge, Disable HS/VS Encoding */
	max96705_write_register(max, 0, S_CONFIG, 0x94);
	msleep(100);

	/* Disable Auto ACK when forward channel is not available */
	regmap_write(max->regmap16, 0x0B0D, 0x00);
	usleep_range(5000, 10000);

	/* Enable DBL Align */
	max96705_write_register(max, 0, S_DBL_ALIGN, 0xC4);
	usleep_range(2000, 5000);

	/* Enable GPIO OUT EN_SET_GP=1, SET_GPIO=1 */
	max96705_write_register(max, 0, 0x0f, 0xbf);
	usleep_range(5000, 10000);

	/* don't double on deserializer */
	regmap_write(max->regmap16, 0x0B07, 0x04);
	usleep_range(5000, 10000);

	/* Update VSYNC signal via VTG so that one line is not lost anymore */
	max96705_write_register(max, 0, 0x43, 0x25);
	max96705_write_register(max, 0, 0x47, 0x29);

	/* Mapping Control */
	/* Enable 3 Mappingsi SRC0-2, DST0-2*/
	regmap_write(max->regmap16, 0x040B, 0x07);

	usleep_range(5000, 10000);
	/* Destionation Controller = Controller 1.
	 * Controller 1 sends data to MIPI Port A
	 * SRC0-2 and DST0-2 map to DPHY1
	 */
	regmap_write(max->regmap16, 0x042D, 0x15);

	/* SRC DT = YUV422 */
	regmap_write(max->regmap16, 0x040D, 0x1E);
	/* DEST DT = YUV422 */
	regmap_write(max->regmap16, 0x040E, 0x1E);
	usleep_range(5000, 10000);

	/* SRC DT = Frame Start */
	regmap_write(max->regmap16, 0x040F, 0x00);
	/* DEST DT = Frame Start */
	regmap_write(max->regmap16, 0x0410, 0x00);
	/* SRC DT = Frame End */
	regmap_write(max->regmap16, 0x0411, 0x01);
	/* DEST DT = Frame End */
	regmap_write(max->regmap16, 0x0412, 0x01);

	/* set des in 2x4 mode, portA uses PHY01, portB uses PHY23 */
	regmap_write(max->regmap16, 0x0330, 0x04);
	usleep_range(5000, 10000);

	/* Four lane output from MIPI Port A */
	regmap_write(max->regmap16, 0x044A, 0xd0);

	/* Set DT, VC, BPP FOR PIPE X BPP = 16 DT = 1E
	 * Enable BPP DT VC YUV422 8bit
	 * Enable CSI output, BPP=0x10 x22,x1e,x2e
	 */
	regmap_write(max->regmap16, 0x0313, 0x82);

	/* soft_vc_x = 0, soft_vc_y = 0 */
	regmap_write(max->regmap16, 0x0316, 0x1E);

	/* MIPI clock output */
	regmap_write(max->regmap16, 0x031D, 0x6F);
	regmap_write(max->regmap16, 0x0320, 0x26);

	/* Disable Link B to enable LOCK output.
	 * LOCK will not assert w/o all active links having locked_g1
	 */
	regmap_write(max->regmap16, 0x0F00, 0x01);

	/* Disable processing HS and DE signals */
	regmap_write(max->regmap16, 0x0B0F, 0x01);

	/* Enable GMSL1 to GMSL2 color mapping YUV422 8 bit normal mode */
	regmap_write(max->regmap16, 0x0B96, 0x9B);

	/* Swap MSBs with LSBs and swap YU and YV positions.*/
	regmap_write(max->regmap16, 0x01C0, 0x08);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01C1, 0x09);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01C2, 0x0A);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01C3, 0x0B);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01C4, 0x0C);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01C5, 0x0D);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01C6, 0x0E);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01C7, 0x0F);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01C8, 0x00);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01C9, 0x01);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01CA, 0x02);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01CB, 0x03);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01CC, 0x04);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01CD, 0x05);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01CE, 0x06);
	usleep_range(5000, 10000);
	regmap_write(max->regmap16, 0x01CF, 0x07);

	/* Crossbar settings */
	for (i = 0; i < ARRAY_SIZE(crossbar_settings); i++) {

		rval = max96705_write_register(max, 0, crossbar_settings[i].reg,
						crossbar_settings[i].val);
		if (rval) {
			dev_err(max->v4l2_sd.dev, "Crossbar: Failed to write reg 0x%04x",
				crossbar_settings[i].reg);
			return rval;
		}
	}
	usleep_range(5000, 10000);

    /*
     * Setup each serializer individually and their respective I2C slave address
     * changed to a unique value by enabling one reverse channel at a time via
     * deserializer's DS_FWDCCEN_REVCCEN control register.
     * Also create broadcast slave address for MAX96705 serializer
     */
    /* hardcode here. */
	max->sensor_present = 0x01;

	/* Max96705 i2c address read */
	for (i = 1; i <= NR_OF_MAX_SINK_PADS; i++) {
	/* Setup the link only if the sensor is connected to the link */
		if (((0x1 << (i - 1)) &  max->sensor_present) == 0)
			continue;

		/* Change Serializer slave address */
		max96705_write_register(max, 0, S_SERADDR,
					(S_ADDR_MAX96705 + i) << 1);
		/* Unique link 'i' image sensor slave address */
		max96705_write_register(max, i, S_I2C_SOURCE_IS,
					(ADDR_AR1043_RVC_BASE + i) << 1);
		/* Link 'i' image sensor slave address */
		max96705_write_register(max, i, S_I2C_DST_IS,
					ADDR_AR1043_SENSOR << 1);
		/* Serializer broadcast address */
		max96705_write_register(max, i, S_I2C_SOURCE_SER,
					S_ADDR_MAX96705_BROADCAST << 1);
		/* Link 'i' serializer address */
		max96705_write_register(max, i, S_I2C_DST_SER,
					(S_ADDR_MAX96705 + i) << 1);
	}

	atomic_set(&status, 1);
	dev_err(max->v4l2_sd.dev, "max9296 init done.\n");

	return 0;
}

/* Unbind the MAX9296 device driver from the I2C client */
static int max9296_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct max9296 *max = to_max_9296(subdev);
	int i;

	mutex_destroy(&max->max_mutex);
	v4l2_ctrl_handler_free(&max->ctrl_handler);
	v4l2_device_unregister_subdev(&max->v4l2_sd);
	media_entity_cleanup(&max->v4l2_sd.entity);

	for (i = 0; i < NR_OF_MAX_SINK_PADS; i++) {
		if (max->sub_devs[i]) {
			struct i2c_client *sub_client =
				v4l2_get_subdevdata(max->sub_devs[i]);

			i2c_unregister_device(sub_client);
		}
		max->sub_devs[i] = NULL;
	}

	return 0;
}

/* Called by I2C probe */
static int max9296_probe(struct i2c_client *client,
				const struct i2c_device_id *devid)
{
	struct max9296 *max;
	int i, rval;

	if (client->dev.platform_data == NULL)
		return -ENODEV;

	max = devm_kzalloc(&client->dev, sizeof(*max), GFP_KERNEL);
	if (!max)
		return -ENODEV;
	else
		gMax = max;

	max->pdata = client->dev.platform_data;

	max->nsources = NR_OF_MAX_SOURCE_PADS;
	max->nsinks = NR_OF_MAX_SINK_PADS;
	max->npads = NR_OF_MAX_PADS;
	max->nstreams = NR_OF_MAX_STREAMS;

	max->crop = devm_kcalloc(&client->dev, max->npads,
				sizeof(struct v4l2_rect), GFP_KERNEL);
	max->compose = devm_kcalloc(&client->dev, max->npads,
				sizeof(struct v4l2_rect), GFP_KERNEL);
	max->route = devm_kcalloc(&client->dev, max->nstreams,
				sizeof(*max->route), GFP_KERNEL);
	max->stream = devm_kcalloc(&client->dev, max->npads,
				sizeof(*max->stream), GFP_KERNEL);

	if (!max->crop || !max->compose || !max->route || !max->stream)
		return -ENOMEM;

	for (i = 0; i < max->npads; i++) {
		max->ffmts[i] = devm_kcalloc(&client->dev, max->nstreams,
				sizeof(struct v4l2_mbus_framefmt), GFP_KERNEL);
		if (!max->ffmts[i])
			return -ENOMEM;

		max->stream[i].stream_id = devm_kcalloc(&client->dev,
		max->nsinks, sizeof(*max->stream[i].stream_id), GFP_KERNEL);
		if (!max->stream[i].stream_id)
			return -ENOMEM;
	}

	for (i = 0; i < max->nstreams; i++) {
		max->route[i].sink = i;
		max->route[i].source = MAX_PAD_SOURCE;
		max->route[i].flags = 0;
	}

	for (i = 0; i < max->nsinks; i++) {
		max->stream[i].stream_id[0] = i;
		max->stream[MAX_PAD_SOURCE].stream_id[i] = i;
	}

	max->regmap16 = devm_regmap_init_i2c(client,
					   &max9296_reg_config16);
	if (IS_ERR(max->regmap16)) {
		dev_err(&client->dev, "Failed to init regmap16!\n");
		return -EIO;
	}

	mutex_init(&max->max_mutex);

	/*
	 * Initialize the v4l2 subdev struct and also ensure v4l2 subdev
	 * and i2c_client both point to one another
	 */
	v4l2_i2c_subdev_init(&max->v4l2_sd, client, &max9296_sd_ops);

	rval = max9296_register_subdev(max);
	if (rval) {
		dev_err(&client->dev, "Failed to register MAX9296 subdevice!\n");
		return rval;
	}

	rval = max9296_init(max, client);
	if (rval) {
		dev_err(&client->dev, "Failed to initialise MAX9296!\n");
		return rval;
	}

	return 0;
}

static const struct i2c_device_id max9296_id_table[] = {
	{ MAX9296_NAME, 0 },
	{ },
};

static struct i2c_driver max9296_i2c_driver = {
	.driver = {
		.name = MAX9296_NAME,
	},
	.probe = max9296_probe,
	.remove = max9296_remove,
	.id_table = max9296_id_table,
};

static int __init max9296_driver_init(void)
{
	return i2c_add_driver(&max9296_i2c_driver);
}
arch_initcall(max9296_driver_init);

static void __exit max9296_driver_exit(void)
{
	i2c_del_driver(&max9296_i2c_driver);
}
module_exit(max9296_driver_exit);

MODULE_AUTHOR("Kiran Kumar <kiran2.kumar@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MAXIM 96705 Serializer and MAXIM 9296 CSI-2 Deserializer driver");

