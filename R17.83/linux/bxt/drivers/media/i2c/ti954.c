/*
 * Copyright (c) 2017 Intel Corporation.
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
#include <media/ti954.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>

#include "ti954-reg.h"

#define TI953_SENSOR_I2C_ADDRESS        0x10
#define TI953_SENSOR_A_I2C_ADDRESS      (TI953_SENSOR_I2C_ADDRESS + 1)
#define TI953_SENSOR_B_I2C_ADDRESS      (TI953_SENSOR_I2C_ADDRESS + 2)

struct ti954 {
	struct v4l2_subdev v4l2_sd;   // add v4l2 sub dev support to i2c driver
	struct ti954_pdata *pdata;
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
	struct regmap *regmap8;
	struct mutex ti954_mutex;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *test_pattern;
};

/* Macro to get access to ti954 structure using the v4l2_subdev structure */
#define to_ti954(_sd) container_of(_sd, struct ti954, v4l2_sd)

/*
 * Order matters.
 *
 * 1. Bits-per-pixel, descending.
 * 2. Bits-per-pixel compressed, descending.
 * 3. Pixel order, same as in pixel_order_str. Formats for all four pixel
 *    orders must be defined.
 */
static const struct ti954_csi_data_format ti_fpd_csi_data_format[] = {
	{MEDIA_BUS_FMT_YUYV8_1X16, 16, 16, PIXEL_ORDER_GBRG, 0x1e},
	{MEDIA_BUS_FMT_UYVY8_1X16, 16, 16, PIXEL_ORDER_GBRG, 0x1e},
};

static const uint32_t ti954_supported_codes_pad[] = {
	MEDIA_BUS_FMT_YUYV8_1X16,
	MEDIA_BUS_FMT_UYVY8_1X16,
	0,
};

static const uint32_t *ti954_supported_codes[] = {
	ti954_supported_codes_pad,
};

static struct regmap_config ti954_reg_config8 = {
	.reg_bits = 8,
	.val_bits = 8,
};


/* Serializer register write */
int  ti953_write_register(struct ti954 *va, unsigned int i, u8 reg, u8 val)
{
	int ret;
	int retry, timeout = 20;
	struct i2c_client *client = v4l2_get_subdevdata(&va->v4l2_sd);

	client->addr = TI953_SENSOR_I2C_ADDRESS + i;
	for (retry = 0; retry < timeout; retry++) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (val < 0)
			msleep(5);
		else
			break;
	}

	if (retry >= timeout) {
		dev_err(va->v4l2_sd.dev, "%s:write reg failed: reg=%2x\n", __func__, reg);
		return -1;
	}
	client->addr = DS_ADDR_TI954;

	return 0;
}

/* Serializer register read */
static inline int ti953_read_register(struct ti954 *va, unsigned int i, u8 reg)
{
	int val;
	int retry, timeout = 10;
	struct i2c_client *client = v4l2_get_subdevdata(&va->v4l2_sd);

	client->addr = TI953_SENSOR_I2C_ADDRESS + i;
	for (retry = 0; retry < timeout; retry++) {
		val = i2c_smbus_read_byte_data(client, reg);
		if (val < 0)
			msleep(5);
		else
			break;
	}

	if (retry >= timeout) {
		dev_err(va->v4l2_sd.dev, "%s:read reg failed: reg=%2x\n", __func__, reg);
		return -1;
	}
	client->addr = DS_ADDR_TI954;

	return val;
}

static int ti954_reg_set_bit(struct ti954 *va, unsigned char reg,
	unsigned char bit, unsigned char val)
{
	int ret;
	unsigned int reg_val;

	ret = regmap_read(va->regmap8, reg, &reg_val);
	if (ret)
		return ret;
	if (val)
		reg_val |= 1 << bit;
	else
		reg_val &= ~(1 << bit);

	return regmap_write(va->regmap8, reg, reg_val);
}

static int ti954_map_phy_i2c_addr(struct ti954 *va, unsigned short rx_port,
			      unsigned short addr)
{
	int rval;

	rval = regmap_write(va->regmap8, TI954_RX_PORT_SEL,(1 << rx_port));
	if (rval)
		return rval;

	return regmap_write(va->regmap8, TI954_SLAVE_ID0, addr);
}

static int ti954_map_alias_i2c_addr(struct ti954 *va, unsigned short rx_port,
			      unsigned short addr)
{
	int rval;

	rval = regmap_write(va->regmap8, TI954_RX_PORT_SEL,(1 << rx_port));
	if (rval)
		return rval;

	return regmap_write(va->regmap8, TI954_SLAVE_ALIAS_ID0, addr);
}

/* Initialize image sensors and set stream on registers */
static int ti954_set_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ti954 *va = to_ti954(subdev);
	struct i2c_client *client = v4l2_get_subdevdata(&va->v4l2_sd);
	int i, rval;
	unsigned int val, reg_val;
	unsigned int pix_count_reg, pix_count_low, pix_count_high;

	dev_err(va->v4l2_sd.dev, "ti954 set stream. enable = %d\n", enable);

	for (i = 0; i < NR_OF_MAX_SINK_PADS; i++) {
		if (((0x01 << (i)) &  va->sensor_present) == 0)
			continue;

		/* Find the pad at the remote end of the link */
		struct media_pad *remote_pad = media_entity_remote_pad(&va->pad[i]);
		struct v4l2_subdev *sd;
		u8 bpp;

		if (!remote_pad)
			continue;

		/* Calls sensor set stream */
		sd = media_entity_to_v4l2_subdev(remote_pad->entity);
		rval = v4l2_subdev_call(sd, video, s_stream, enable);
		if (rval) {
			dev_err(va->v4l2_sd.dev,
				"Failed to set stream for %s. enable = %d\n",
				sd->name, enable);
			return rval;
		}
	}

	rval = regmap_read(va->regmap8, TI954_CSI_CTL, &reg_val);
	if (rval) {
			dev_err(va->v4l2_sd.dev, "Failed to read register 0x%x!\n",TI954_CSI_CTL);
			return rval;
	}

	if (!enable) {
		reg_val = reg_val & (~TI954_CSI_ENABLE);
		/* Disable CSI output */
		rval = regmap_write(va->regmap8, TI954_CSI_CTL, reg_val);
		if (rval) {
			dev_err(va->v4l2_sd.dev, "Failed to disable CSI output!\n");
			return rval;
		}
		msleep(10);
		goto exit_stream;
	}
	else
	{
		reg_val = reg_val | TI954_CSI_ENABLE;
		/* Enable CSI output */
		rval = regmap_write(va->regmap8, TI954_CSI_CTL, reg_val);
		if (rval) {
			dev_err(va->v4l2_sd.dev, "Failed to enable CSI output!\n");
			return rval;
		}
        msleep(10);
	}

	/* Check if valid PCLK is available for the links */
	for (i = 0; i < NR_OF_MAX_SINK_PADS; i++) {
		if (((0x01 << (i)) &  va->sensor_present) == 0)
			continue;
		rval = regmap_write(va->regmap8, TI954_RX_PORT_SEL,((i==TI954_RX_PORT_1)?
							TI954_RX_PORT1_EN:TI954_RX_PORT0_EN) | (1 << i));
		if (rval) {
				dev_err(va->v4l2_sd.dev, "Failed to write register 0x%x!\n",TI954_RX_PORT_SEL);
				return rval;
		}
		/* Detect video links */
		rval = regmap_read(va->regmap8, TI954_RX_PORT_STS1, &reg_val);
		if (rval) {
				dev_err(va->v4l2_sd.dev, "Failed to read register 0x%x!\n",TI954_RX_PORT_STS1);
				return rval;
		}

		if(reg_val&TI954_RX_LOCK_STS) {
			dev_err(va->v4l2_sd.dev, "Valid PCLK detected for link %d\n", i);

		}
		else {
			dev_err(va->v4l2_sd.dev, "No PCLK for link %d\n",i);
		}
	}

exit_stream:
	dev_err(va->v4l2_sd.dev, "%s: exiting. streaming %s\n",
			__func__, enable ? "enabled" : "disabled");

	return 0;
}

/* Get the media bus format */
static struct v4l2_mbus_framefmt *__ti954_get_ffmt(struct v4l2_subdev *subdev,
			 struct v4l2_subdev_pad_config *cfg,
			 unsigned int pad, unsigned int which,
			 unsigned int stream)
{
	struct ti954 *va = to_ti954(subdev);

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(subdev, cfg, pad);
	else
		return &va->ffmts[pad][stream];
}

/* callback for VIDIOC_SUBDEV_G_FMT ioctl handler code */
static int ti954_get_format(struct v4l2_subdev *subdev,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *fmt)
{
	struct ti954 *va = to_ti954(subdev);

	if (fmt->stream > va->nstreams)
		return -EINVAL;

	mutex_lock(&va->ti954_mutex);
	fmt->format = *__ti954_get_ffmt(subdev, cfg, fmt->pad, fmt->which, fmt->stream);
	mutex_unlock(&va->ti954_mutex);

	dev_dbg(subdev->dev, "subdev_format: which: %s, pad: %d, stream: %d.\n",
		 fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE ?
		 "V4L2_SUBDEV_FORMAT_ACTIVE" : "V4L2_SUBDEV_FORMAT_TRY",
		 fmt->pad, fmt->stream);

	dev_dbg(subdev->dev, "framefmt: width: %d, height: %d, code: 0x%x.\n",
		fmt->format.width, fmt->format.height, fmt->format.code);

	return 0;
}

/* Validate csi_data_format */
static const struct ti954_csi_data_format *ti954_validate_csi_data_format(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ti_fpd_csi_data_format); i++) {
		if (ti_fpd_csi_data_format[i].code == code)
			return &ti_fpd_csi_data_format[i];
	}

	return &ti_fpd_csi_data_format[0];
}

/* callback for VIDIOC_SUBDEV_S_FMT ioctl handler code */
static int ti954_set_format(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *fmt)
{
	struct ti954 *va = to_ti954(subdev);
	const struct ti954_csi_data_format *csi_format;
	struct v4l2_mbus_framefmt *ffmt;

	if (fmt->stream > va->nstreams)
		return -EINVAL;

	csi_format = ti954_validate_csi_data_format(fmt->format.code);

	mutex_lock(&va->ti954_mutex);
	ffmt = __ti954_get_ffmt(subdev, cfg, fmt->pad, fmt->which, fmt->stream);
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ffmt->width = fmt->format.width;
		ffmt->height = fmt->format.height;
		ffmt->code = csi_format->code;
	}

	fmt->format = *ffmt;
	mutex_unlock(&va->ti954_mutex);

	dev_err(subdev->dev, "framefmt: width: %d, height: %d, code: 0x%x.\n",
	       ffmt->width, ffmt->height, ffmt->code);

	return 0;
}

/* get the current low level media bus frame parameters */
static int ti954_get_frame_desc(struct v4l2_subdev *sd,
	unsigned int pad, struct v4l2_mbus_frame_desc *desc)
{
	struct ti954 *va = to_ti954(sd);
	struct v4l2_mbus_frame_desc_entry *entry = desc->entry;
	u8 vc = 0;
	int i;

	desc->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	for (i = 0; i < min(va->nstreams, desc->num_entries); i++) {
		struct v4l2_mbus_framefmt *ffmt =
			&va->ffmts[i][MAX_PAD_SOURCE];
		const struct ti954_csi_data_format *csi_format =
			ti954_validate_csi_data_format(ffmt->code);

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
static int ti954_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	struct ti954 *va = to_ti954(sd);
	const uint32_t *supported_code = ti954_supported_codes[code->pad];
	bool next_stream = false;
	int i;

	if (code->stream & V4L2_SUBDEV_FLAG_NEXT_STREAM) {
		next_stream = true;
		code->stream &= ~V4L2_SUBDEV_FLAG_NEXT_STREAM;
	}

	if (code->stream > va->nstreams)
		return -EINVAL;

	if (next_stream) {
		if (!(va->pad[code->pad].flags & MEDIA_PAD_FL_MULTIPLEX))
			return -EINVAL;
		if (code->stream < va->nstreams - 1) {
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
static int ti954_set_routing(struct v4l2_subdev *sd,
				   struct v4l2_subdev_routing *route)
{
	struct ti954 *va = to_ti954(sd);
	int i, j, ret = 0;

	for (i = 0; i < min(route->num_routes, va->nstreams); ++i) {
		struct v4l2_subdev_route *t = &route->routes[i];
		unsigned int sink = t->sink_pad;
		unsigned int source = t->source_pad;

		if (t->sink_stream > va->nstreams - 1 ||
		    t->source_stream > va->nstreams - 1)
			continue;

		if (t->source_pad != MAX_PAD_SOURCE)
			continue;

		for (j = 0; j < va->nstreams; j++) {
			if (sink == va->route[j].sink &&
				source == va->route[j].source)
				break;
		}

		if (j == va->nstreams)
			continue;

		va->stream[sink].stream_id[0] = t->sink_stream;
		va->stream[source].stream_id[sink] = t->source_stream;

		if (t->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE)
			va->route[j].flags |=
				V4L2_SUBDEV_ROUTE_FL_ACTIVE;
		else if (!(t->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			va->route[j].flags &=
				(~V4L2_SUBDEV_ROUTE_FL_ACTIVE);
	}

	return ret;
}

/* Configure Media Controller routing */
static int ti954_get_routing(struct v4l2_subdev *sd,
				   struct v4l2_subdev_routing *route)
{
	struct ti954 *va = to_ti954(sd);
	int i;

	for (i = 0; i < min(va->nstreams, route->num_routes); ++i) {
		unsigned int sink = va->route[i].sink;
		unsigned int source = va->route[i].source;

		route->routes[i].sink_pad = sink;
		route->routes[i].sink_stream =
			va->stream[sink].stream_id[0];
		route->routes[i].source_pad = source;
		route->routes[i].source_stream =
			va->stream[source].stream_id[sink];
		route->routes[i].flags = va->route[i].flags;
	}

	route->num_routes = i;

	return 0;
}

/* called when the subdev device node is opened by an application */
static int ti954_open(struct v4l2_subdev *subdev,
				struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(subdev, fh->pad, 0);

	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = MAX_PAD_SOURCE,
		.format = {
			.width = TI_DS90UB954_MAX_WIDTH,
			.height = TI_DS90UB954_MAX_HEIGHT,
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
static int ti954_registered(struct v4l2_subdev *subdev)
{
	struct ti954 *va = to_ti954(subdev);
	int i, j, k, l, rval;

	for (i = 0, k = 0; i < va->pdata->subdev_num; i++) {
		struct ti954_subdev_i2c_info *info =
			&va->pdata->subdev_info[i];
		struct i2c_adapter *adapter;

		if (k >= va->nsinks)
			break;

		adapter = i2c_get_adapter(info->i2c_adapter_id);
		va->sub_devs[k] = v4l2_i2c_new_subdev_board(
			va->v4l2_sd.v4l2_dev, adapter,
			&info->board_info, 0);
		i2c_put_adapter(adapter);
		if (!va->sub_devs[k]) {
			dev_err(va->v4l2_sd.dev,
				"can't create new i2c subdev %d-%04x\n",
				info->i2c_adapter_id,
				info->board_info.addr);
			continue;
		}

		for (j = 0; j < va->sub_devs[k]->entity.num_pads; j++) {
			if (va->sub_devs[k]->entity.pads[j].flags &
				MEDIA_PAD_FL_SOURCE)
				break;
		}

		if (j == va->sub_devs[k]->entity.num_pads) {
			dev_warn(va->v4l2_sd.dev,
				"no source pad in subdev %d-%04x\n",
				info->i2c_adapter_id,
				info->board_info.addr);
			return -ENOENT;
		}

		for (l = 0; l < va->nsinks; l++) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
			rval = media_entity_create_link(
#else
			rval = media_create_pad_link(
#endif
				&va->sub_devs[k]->entity, j,
				&va->v4l2_sd.entity, l, 0);
			if (rval) {
				dev_err(va->v4l2_sd.dev,
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

static int ti954_set_power(struct v4l2_subdev *subdev, int on)
{
	return 0;
}

static const struct v4l2_subdev_core_ops ti954_core_subdev_ops = {
	.s_power = ti954_set_power,
};

static const struct v4l2_subdev_video_ops ti954_sd_video_ops = {
	/* used to notify the driver that the video stream will start or has stopped */
	.s_stream = ti954_set_stream,
};

static const struct v4l2_subdev_pad_ops ti954_sd_pad_ops = {
	.get_fmt = ti954_get_format,
	.set_fmt = ti954_set_format,
	.get_frame_desc = ti954_get_frame_desc,
	.enum_mbus_code = ti954_enum_mbus_code,
	.set_routing = ti954_set_routing,
	.get_routing = ti954_get_routing,
};

static struct v4l2_subdev_ops ti954_sd_ops = {
	.core = &ti954_core_subdev_ops,
	.video = &ti954_sd_video_ops,
	.pad = &ti954_sd_pad_ops,
};

static struct v4l2_subdev_internal_ops ti954_sd_internal_ops = {
	.open = ti954_open,
	.registered = ti954_registered,
};

static int ti954_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops ti954_ctrl_ops = {
	.s_ctrl = ti954_s_ctrl,
};

static const s64 ti954_op_sys_clock[] =  {400000000, 800000000};
static const struct v4l2_ctrl_config ti954_controls[] = {
	{
		.ops = &ti954_ctrl_ops,
		.id = V4L2_CID_LINK_FREQ,
		.name = "V4L2_CID_LINK_FREQ",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.max = ARRAY_SIZE(ti954_op_sys_clock) - 1,
		.min = 0,
		.step = 0,
		.def = 0,
		.qmenu_int = ti954_op_sys_clock,
	},
};

/* Registers ti954 sub-devices (Image sensors) */
static int ti954_register_subdev(struct ti954 *va)
{
	int i, rval;
	struct i2c_client *client = v4l2_get_subdevdata(&va->v4l2_sd);

	/* subdevice driver initializes v4l2 subdev */
	v4l2_subdev_init(&va->v4l2_sd, &ti954_sd_ops);
	snprintf(va->v4l2_sd.name, sizeof(va->v4l2_sd.name), "ti954 %d-%4.4x",
		i2c_adapter_id(client->adapter), client->addr);
	va->v4l2_sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			V4L2_SUBDEV_FL_HAS_SUBSTREAMS;

	va->v4l2_sd.internal_ops = &ti954_sd_internal_ops;

	v4l2_set_subdevdata(&va->v4l2_sd, client);

	v4l2_ctrl_handler_init(&va->ctrl_handler,
				ARRAY_SIZE(ti954_controls));

	if (va->ctrl_handler.error) {
		dev_err(va->v4l2_sd.dev,
			"Failed to init ti954 controls. ERR: %d!\n",
			va->ctrl_handler.error);
		return va->ctrl_handler.error;
	}

	va->v4l2_sd.ctrl_handler = &va->ctrl_handler;

	for (i = 0; i < ARRAY_SIZE(ti954_controls); i++) {
		const struct v4l2_ctrl_config *cfg =
			&ti954_controls[i];
		struct v4l2_ctrl *ctrl;

		ctrl = v4l2_ctrl_new_custom(&va->ctrl_handler, cfg, NULL);
		if (!ctrl) {
			dev_err(va->v4l2_sd.dev,
				"Failed to create ctrl %s!, err: %d\n", cfg->name, va->ctrl_handler.error);
			rval = va->ctrl_handler.error;
			goto failed_out;
		}
	}

	va->link_freq = v4l2_ctrl_find(&va->ctrl_handler, V4L2_CID_LINK_FREQ);
	va->test_pattern = v4l2_ctrl_find(&va->ctrl_handler,
					  V4L2_CID_TEST_PATTERN);

	for (i = 0; i < va->nsinks; i++)
		va->pad[i].flags = MEDIA_PAD_FL_SINK;
	va->pad[MAX_PAD_SOURCE].flags =
		MEDIA_PAD_FL_SOURCE | MEDIA_PAD_FL_MULTIPLEX;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	rval = media_entity_init(&va->v4l2_sd.entity, NR_OF_MAX_PADS, va->pad, 0);
#else
	rval = media_entity_pads_init(&va->v4l2_sd.entity, NR_OF_MAX_PADS, va->pad);
#endif
	if (rval) {
		dev_err(va->v4l2_sd.dev,
			"Failed to init media entity for ti954!\n");
		goto failed_out;
	}

	return 0;
failed_out:
	v4l2_ctrl_handler_free(&va->ctrl_handler);
	return rval;
}

/* ti954 initial setup and Reverse channel setup */
static int ti954_init(struct ti954 *va, struct i2c_client *client)
{
	int i,rval;
	unsigned int val,reg_val;

	/* Get the device ID of Deserializer */
	rval = regmap_read(va->regmap8, TI954_DEVID, &val);
	if (rval) {
		dev_err(va->v4l2_sd.dev, "Failed to read device ID of DS90UB954!\n");
		return rval;
	}

	dev_info(va->v4l2_sd.dev, "DS90UB954 device ID: 0x%X\n", val);
	/* UB954 Reset */
	regmap_write(va->regmap8, TI954_RESET, 0x02);
	msleep(500);

	va->sensor_present = 0;

	/* RX Port0 Setting */
	/* FPD3_PORT_SEL: RX_WRITE_PORT_0 set */
	regmap_write(va->regmap8, 0x4c, 0x01);
	/* BCC_CONFIG: I2C pass through, BC enable, BC CRC */
	regmap_write(va->regmap8, 0x58, 0x5e);
	/* SER_ALIAS_ID 0x11 */
	regmap_write(va->regmap8, 0x5c, 0x22);
	/* FPD3 mode - COAX CSI */
	regmap_write(va->regmap8, 0x6d, 0x7c);
	/* VC=0,DT=0x1e (YUV422 8bit) */
	regmap_write(va->regmap8, 0x70, 0x1e);
	/* VC=0,remapping VC0 */
	regmap_write(va->regmap8, 0x72, 0xe4);
	/* Enable continous clock in 953 */
	ti953_write_register(va, 1, 0x02, 0x72);
	msleep(100);

	/* RX Port1 Setting */
	/* FPD3_PORT_SEL: RX_WRITE_PORT_1 set */
	regmap_write(va->regmap8, 0x4c, 0x12);
	/* BCC_CONFIG: I2C pass through, BC enable, BC CRC */
	regmap_write(va->regmap8, 0x58, 0x5e);
	/* SER_ALIAS_ID 0x12 */
	regmap_write(va->regmap8, 0x5c, 0x24);
	/* FPD3 mode - COAX CSI */
	regmap_write(va->regmap8, 0x6d, 0x7c);
	/* VC=0,DT=0x1e (YUV422 8bit) */
	regmap_write(va->regmap8, 0x70, 0x1e);
	/* VC=0,remapping VC1 */
	regmap_write(va->regmap8, 0x72, 0xe1);
	/* Enable continous clock in 953 */
	ti953_write_register(va, 2, 0x02, 0x72);
	msleep(100);

	/*
	* Enable initial skew calibration sequence CSI_CTL: [5:4] CSI_LANE_COUNT = 4,
	* [0]CSI_ENABLE = 0
	*/
	regmap_write(va->regmap8, 0x33, 0x40);
	/* FWD_CTL1: rx port 0 and rx port1 forward enable */
	regmap_write(va->regmap8, 0x20, 0x00);
	msleep(100);

	for (i = 0; i < TI954_RX_PORT_NUM; i++) {
		rval = regmap_write(va->regmap8, TI954_RX_PORT_SEL,((i==TI954_RX_PORT_1)?
						TI954_RX_PORT1_EN:TI954_RX_PORT0_EN) | (1 << i));
		if (rval) {
				dev_err(va->v4l2_sd.dev, "Failed to write register 0x%x!\n",TI954_RX_PORT_SEL);
				return rval;
		}
		/* Detect video links */
		rval = regmap_read(va->regmap8, TI954_RX_PORT_STS1, &reg_val);
		if (rval) {
				dev_err(va->v4l2_sd.dev, "Failed to read register 0x%x!\n",TI954_RX_PORT_STS1);
				return rval;
		}

		/*
		 * Check on which links the sensors are connected
		 * And also check total number of sensors connected to the deserializer
		*/
		if(reg_val&TI954_RX_LOCK_STS) {
			va->sensor_present |= (1 << i);
		}

		dev_err(va->v4l2_sd.dev, "%s DS90UB954 va->sensor_present = %x\n",__func__, va->sensor_present);
	}

	return 0;
}

/* Unbind the ti954 device driver from the I2C client */
static int ti954_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ti954 *va = to_ti954(subdev);
	int i;

	mutex_destroy(&va->ti954_mutex);
	v4l2_ctrl_handler_free(&va->ctrl_handler);
	v4l2_device_unregister_subdev(&va->v4l2_sd);
	media_entity_cleanup(&va->v4l2_sd.entity);

	for (i = 0; i < NR_OF_MAX_SINK_PADS; i++) {
		if (va->sub_devs[i]) {
			struct i2c_client *sub_client =
				v4l2_get_subdevdata(va->sub_devs[i]);

			i2c_unregister_device(sub_client);
		}
		va->sub_devs[i] = NULL;
	}

	return 0;
}

/* Called by I2C probe */
static int ti954_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
	struct ti954 *va;
	int i, rval;

	if (client->dev.platform_data == NULL)
		return -ENODEV;

	va = devm_kzalloc(&client->dev, sizeof(*va), GFP_KERNEL);
	if (!va)
		return -ENODEV;

	va->pdata = client->dev.platform_data;

	va->nsources = NR_OF_MAX_SOURCE_PADS;
	va->nsinks = NR_OF_MAX_SINK_PADS;
	va->npads = NR_OF_MAX_PADS;
	va->nstreams = NR_OF_MAX_STREAMS;

	va->crop = devm_kcalloc(&client->dev, va->npads, sizeof(struct v4l2_rect), GFP_KERNEL);
	va->compose = devm_kcalloc(&client->dev, va->npads, sizeof(struct v4l2_rect), GFP_KERNEL);
	va->route = devm_kcalloc(&client->dev, va->nstreams, sizeof(*va->route), GFP_KERNEL);
	va->stream = devm_kcalloc(&client->dev, va->npads, sizeof(*va->stream), GFP_KERNEL);

	if (!va->crop || !va->compose || !va->route || !va->stream)
		return -ENOMEM;

	for (i = 0; i < va->npads; i++) {
		va->ffmts[i] = devm_kcalloc(&client->dev, va->nstreams,
					sizeof(struct v4l2_mbus_framefmt), GFP_KERNEL);
		if (!va->ffmts[i])
			return -ENOMEM;

		va->stream[i].stream_id = devm_kcalloc(&client->dev, va->nsinks,
						sizeof(*va->stream[i].stream_id), GFP_KERNEL);
		if (!va->stream[i].stream_id)
			return -ENOMEM;
	}

	for (i = 0; i < va->nstreams; i++) {
		va->route[i].sink = i;
		va->route[i].source = MAX_PAD_SOURCE;
		va->route[i].flags = 0;
	}

	for (i = 0; i < va->nsinks; i++) {
		va->stream[i].stream_id[0] = i;
		va->stream[MAX_PAD_SOURCE].stream_id[i] = i;
	}

	va->regmap8 = devm_regmap_init_i2c(client,
					   &ti954_reg_config8);
	if (IS_ERR(va->regmap8)) {
		dev_err(&client->dev, "Failed to init regmap8!\n");
		return -EIO;
	}

	mutex_init(&va->ti954_mutex);

	/*
	 * Intialize the v4l2 subdev struct and also ensure v4l2 subdev
	 * and i2c_client both point to one another
	*/
	v4l2_i2c_subdev_init(&va->v4l2_sd, client, &ti954_sd_ops);

	rval = ti954_register_subdev(va);
	if (rval) {
		dev_err(&client->dev, "Failed to register ti954 subdevice!\n");
		return rval;
	}

	rval = ti954_init(va, client);
	if (rval) {
		dev_err(&client->dev, "Failed to initialise ti954!\n");
		return rval;
	}

	return 0;
}

static const struct i2c_device_id ti954_id_table[] = {
	{ TI_DS90UB954_NAME, 0 },
	{ },
};

static struct i2c_driver ti954_i2c_driver = {
	.driver = {
		.name = TI_DS90UB954_NAME,
	},
	.probe = ti954_probe,
	.remove = ti954_remove,
	.id_table = ti954_id_table,
};

/*
 * Helper macro for registering a I2C driver
 * calls module_init and Module_exit
*/
module_i2c_driver(ti954_i2c_driver);

MODULE_AUTHOR("Weitao Zhu <Weitao.Zhu@harman.com>");
MODULE_AUTHOR("Yi Chen <yi.chen2@harman.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI953 Serializer and TI954 Deserializer driver");
