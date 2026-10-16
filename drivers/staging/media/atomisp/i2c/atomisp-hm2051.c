// SPDX-License-Identifier: GPL-2.0
/*
 * Support for Himax HM2051 2MP camera sensor.
 *
 * Copyright (c) 2025 Claude / Anthropic (driver skeleton)
 *
 * Based on ov5670_atomisp.c and CM-CHT unicam_hm2051.h register tables.
 * HM2051: 1600x1200, RAW10 RGGB, 1-lane MIPI CSI-2, 19.2MHz clock.
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/types.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "../include/linux/atomisp_gmin_platform.h"

#define HM2051_NATIVE_WIDTH		1296
#define HM2051_NATIVE_HEIGHT		736

#define HM2051_FPS			30
#define HM2051_SKIP_FRAMES		3

#define HM2051_REG_CHIP_ID		0x0000
#define HM2051_CHIP_ID			0x2051
#define HM2051_CHIP_ID_ALT		0x2355

#define HM2051_REG_MODE_SELECT		0x0005
#define HM2051_MODE_STREAMING		0x03
#define HM2051_MODE_STANDBY		0x02

#define HM2051_REG_EXPOSURE		0x0015
#define HM2051_REG_ANALOG_GAIN		0x0018

#define to_hm2051_sensor(x) container_of(x, struct hm2051_device, sd)

struct hm2051_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct mutex input_lock;
	bool is_streaming;

	int power_count;
	struct camera_sensor_platform_data *platform_data;
	struct camera_mipi_info csi;

	struct hm2051_mode {
		struct v4l2_mbus_framefmt fmt;
	} mode;

	struct hm2051_ctrls {
		struct v4l2_ctrl_handler handler;
		struct v4l2_ctrl *exposure;
		struct v4l2_ctrl *gain;
	} ctrls;
};

struct hm2051_reg {
	u16 reg;
	u8 val;
};

/*
 * I2C helpers for 16-bit register addresses, 8-bit values.
 */
static int hm2051_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf = 0;
	int ret;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = addr_buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = &data_buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2) {
		dev_err(&client->dev, "read reg 0x%04x failed: %d\n", reg, ret);
		return ret < 0 ? ret : -EIO;
	}

	*val = data_buf;
	return 0;
}

static int hm2051_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	int ret;

	ret = i2c_master_send(client, buf, 3);
	if (ret != 3) {
		dev_err(&client->dev, "write reg 0x%04x=0x%02x failed: %d\n",
			reg, val, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int hm2051_write_reg_array(struct i2c_client *client,
				  const struct hm2051_reg *reglist, int count)
{
	int i, ret;

	for (i = 0; i < count; i++) {
		ret = hm2051_write_reg(client, reglist[i].reg, reglist[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Register init sequence from CM-CHT unicam_hm2051.h (hm2051_regress0[]).
 * Configures PLL, analog, BLC, CFPN, digital window, and MIPI for
 * 1600x1200 RAW8 BGGR output at 30fps with 19.2MHz input clock.
 */
static const struct hm2051_reg hm2051_init_regs[] = {
	/* Enter standby */
	{0x0000, 0x00},
	{0x0100, 0x00},
	{0x0101, 0x00},
	{0x0005, 0x02},

	/* PLL / clock configuration */
	{0x0010, 0x02},
	{0x0011, 0x02},
	{0x0026, 0x08},
	{0x002A, 0x52},

	/* Timing / blanking */
	{0x0006, 0x04},
	{0x000F, 0x00},
	{0x0024, 0x40},
	{0x0027, 0x23},
	{0x0065, 0x01},
	{0x0074, 0x1B},

	/* BLC (Black Level Correction) */
	{0x002B, 0x00},
	{0x002C, 0x06},

	/* Analog configuration */
	{0x0040, 0x00},
	{0x0044, 0x03},
	{0x0045, 0x63},
	{0x0046, 0x5F},
	{0x0049, 0xC0},
	{0x004B, 0x03},

	/* Column / row control */
	{0x0070, 0x2F},
	{0x0072, 0xFB},
	{0x0073, 0x77},
	{0x0075, 0x40},
	{0x0078, 0x65},

	/* Sensor control */
	{0x0080, 0x98},
	{0x0082, 0x09},
	{0x0083, 0x3C},
	{0x0087, 0x41},
	{0x008D, 0x20},
	{0x008E, 0x30},

	/* CFPN (Column Fixed Pattern Noise) */
	{0x009D, 0x11},
	{0x009E, 0x12},

	/* Dark current */
	{0x0090, 0x00},
	{0x0091, 0x01},
	{0x0092, 0x02},
	{0x0093, 0x03},

	/* ISP / digital — restore original values.
	 * Note: 0x00C3-0x00C7 are read-only on this chip.
	 */
	{0x00C0, 0x64},
	{0x00C1, 0x15},
	{0x00C2, 0x00},
	{0x00C3, 0x02},
	{0x00C4, 0x0B},
	{0x00C6, 0x83},
	{0x00C7, 0x02},
	{0x00CC, 0x00},

	/* MIPI TX configuration */
	{0x4B3B, 0x12},
	{0x4B41, 0x10},
	{0x4B01, 0x2B},	/* MIPI data type: RAW10 (0x2B) — original CM-CHT value */
	{0x4B02, 0x05},	/* Try: bit0=10-bit packing? (default 0x04) */
	{0x0165, 0x03},
	{0x018C, 0x00},

	/* AE window */
	{0x0183, 0xF0},

	/* Digital window: H=1616 (fixed by sensor), V=1216 (original CM-CHT, H-1) */
	{0x0195, 0x06},
	{0x0196, 0x4F},
	{0x0197, 0x04},
	{0x0198, 0xBF},

	/* Lens shading / correction */
	{0x0144, 0x12},
	{0x0140, 0x20},
	{0x015A, 0x80},
	{0x015D, 0x20},
	{0x0160, 0x65},
	{0x0123, 0xD5},

	/* Crop window — X:160-1455 (1296 wide), Y:46-781 (736 tall, active area) */
	{0x0660, 0x00},
	{0x0661, 0xA0},
	{0x0662, 0x05},
	{0x0663, 0xAF},
	{0x0664, 0x00},
	{0x0665, 0x2E},
	{0x0666, 0x03},
	{0x0667, 0x0D},

	/* MIPI PHY / data lane */
	{0x4B50, 0x00},
	{0x4B51, 0xAF},
	{0x4B0A, 0x05},
	{0x4B0B, 0x10},
	{0x4B20, 0x9E},
	{0x4B07, 0xBD},

	/* Start streaming + exposure settle */
	{0x0005, 0x03},
	{0x0025, 0x00},
};

static const struct hm2051_reg hm2051_stream_on_regs[] = {
	{0x0000, 0x01},
	{0x0100, 0x01},
	{0x0101, 0x01},
	{0x4B20, 0x9E},
	{0x0005, 0x03},
};

static const struct hm2051_reg hm2051_stream_off_regs[] = {
	{0x0005, 0x02},
	{0x4B20, 0xDE},
};

/*
 * Exposure: 16-bit coarse integration time at register 0x0015.
 */
static int hm2051_exposure_set(struct hm2051_device *dev, u32 exp)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int ret;

	ret = hm2051_write_reg(client, HM2051_REG_EXPOSURE, (exp >> 8) & 0xff);
	if (ret)
		return ret;

	return hm2051_write_reg(client, HM2051_REG_EXPOSURE + 1, exp & 0xff);
}

/*
 * Analog gain: 2-bit register 0x0018.
 * 0=1x, 1=2x, 2=4x, 3=8x
 */
static int hm2051_gain_set(struct hm2051_device *dev, u32 gain)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);

	return hm2051_write_reg(client, HM2051_REG_ANALOG_GAIN, gain & 0x03);
}

static int hm2051_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hm2051_device *dev =
		container_of(ctrl->handler, struct hm2051_device, ctrls.handler);
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = hm2051_exposure_set(dev, ctrl->val);
		break;
	case V4L2_CID_GAIN:
		ret = hm2051_gain_set(dev, ctrl->val);
		break;
	default:
		return -EINVAL;
	}
	if (ret)
		return ret;

	/*
	 * Exposure and gain are double buffered; they only take effect after
	 * a CMU update (0x0100 = 1). Without it changes made while streaming
	 * are ignored until the next stream start.
	 */
	return hm2051_write_reg(client, 0x0100, 0x01);
}

static const struct v4l2_ctrl_ops hm2051_ctrl_ops = {
	.s_ctrl = hm2051_s_ctrl,
};

static struct v4l2_mbus_framefmt *
hm2051_get_pad_format(struct hm2051_device *dev,
		      struct v4l2_subdev_state *state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_state_get_format(state, pad);

	return &dev->mode.fmt;
}

static void hm2051_fill_format(struct v4l2_mbus_framefmt *fmt)
{
	memset(fmt, 0, sizeof(*fmt));
	fmt->width = HM2051_NATIVE_WIDTH;
	fmt->height = HM2051_NATIVE_HEIGHT;
	fmt->field = V4L2_FIELD_NONE;
	fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
}

static int hm2051_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct v4l2_mbus_framefmt *fmt;

	fmt = hm2051_get_pad_format(dev, sd_state, format->pad, format->which);
	hm2051_fill_format(fmt);

	format->format = *fmt;
	return 0;
}

static int hm2051_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct v4l2_mbus_framefmt *fmt;

	fmt = hm2051_get_pad_format(dev, sd_state, format->pad, format->which);
	format->format = *fmt;
	return 0;
}

static int hm2051_power_on(struct v4l2_subdev *sd)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_sensor_platform_data *pdata = dev->platform_data;
	int ret;

	if (!pdata) {
		dev_err(&client->dev, "no platform data for power on\n");
		return -ENODEV;
	}

	ret = pdata->v1p8_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "v1p8 power on failed: %d\n", ret);
		return ret;
	}

	ret = pdata->v2p8_ctrl(sd, 1);
	if (ret) {
		pdata->v1p8_ctrl(sd, 0);
		return ret;
	}

	ret = pdata->flisclk_ctrl(sd, 1);
	if (ret) {
		pdata->v2p8_ctrl(sd, 0);
		pdata->v1p8_ctrl(sd, 0);
		return ret;
	}

	if (pdata->gpio0_ctrl)
		pdata->gpio0_ctrl(sd, 1);
	if (pdata->gpio1_ctrl)
		pdata->gpio1_ctrl(sd, 1);

	msleep(20);
	return 0;
}

static void hm2051_power_off(struct v4l2_subdev *sd)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct camera_sensor_platform_data *pdata = dev->platform_data;

	if (!pdata)
		return;

	if (pdata->gpio1_ctrl)
		pdata->gpio1_ctrl(sd, 0);
	if (pdata->gpio0_ctrl)
		pdata->gpio0_ctrl(sd, 0);
	pdata->flisclk_ctrl(sd, 0);
	pdata->v2p8_ctrl(sd, 0);
	pdata->v1p8_ctrl(sd, 0);
}

static int hm2051_s_power(struct v4l2_subdev *sd, int on)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	dev_info(&client->dev, "s_power(%d) power_count=%d\n", on, dev->power_count);
	mutex_lock(&dev->input_lock);
	if (on) {
		if (dev->power_count++ > 0)
			goto out;
		ret = hm2051_power_on(sd);
		if (ret) {
			dev->power_count--;
			dev_err(&client->dev, "power_on failed: %d\n", ret);
		} else {
			dev_info(&client->dev, "power on successful\n");
		}
	} else {
		if (--dev->power_count > 0)
			goto out;
		dev->power_count = 0;
		hm2051_power_off(sd);
	}
out:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int hm2051_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	dev_info(&client->dev, "%s enable=%d\n", __func__, enable);
	mutex_lock(&dev->input_lock);

	if (enable) {
		u8 id_hi = 0, id_lo = 0;

		/* Ensure sensor is powered on */
		if (dev->power_count == 0) {
			mutex_unlock(&dev->input_lock);
			ret = hm2051_s_power(sd, 1);
			if (ret) {
				dev_err(&client->dev, "power on failed: %d\n", ret);
				return ret;
			}
			mutex_lock(&dev->input_lock);
		}

		/* Verify chip ID */
		ret = hm2051_read_reg(client, 0x0001, &id_hi);
		if (!ret)
			hm2051_read_reg(client, 0x0002, &id_lo);
		dev_info(&client->dev, "chip ID = 0x%02x%02x\n", id_hi, id_lo);

		/* Write full init sequence */
		ret = hm2051_write_reg_array(client, hm2051_init_regs,
					     ARRAY_SIZE(hm2051_init_regs));
		if (ret) {
			dev_err(&client->dev, "init regs write failed: %d\n", ret);
			goto out;
		}

		/* Restore V4L2 control values */
		ret = __v4l2_ctrl_handler_setup(&dev->ctrls.handler);
		if (ret)
			goto out;

		/* Stream on */
		ret = hm2051_write_reg_array(client, hm2051_stream_on_regs,
					     ARRAY_SIZE(hm2051_stream_on_regs));
		if (ret) {
			dev_err(&client->dev, "stream on failed: %d\n", ret);
			goto out;
		}
		dev_info(&client->dev, "streaming started\n");

		/* Dump key register ranges to find output format control */
		{
			u8 val;
			u16 reg;
			/* 0x0020-0x002F: timing/output */
			for (reg = 0x0020; reg <= 0x002F; reg++) {
				if (!hm2051_read_reg(client, reg, &val))
					dev_info(&client->dev, "REG[0x%04x]=0x%02x\n", reg, val);
			}
			/* 0x0070-0x007F: column/OMUX */
			for (reg = 0x0070; reg <= 0x007F; reg++) {
				if (!hm2051_read_reg(client, reg, &val))
					dev_info(&client->dev, "REG[0x%04x]=0x%02x\n", reg, val);
			}
			/* 0x00B0-0x00CF: ISP/digital */
			for (reg = 0x00B0; reg <= 0x00CF; reg++) {
				if (!hm2051_read_reg(client, reg, &val))
					dev_info(&client->dev, "REG[0x%04x]=0x%02x\n", reg, val);
			}
			/* 0x4B00-0x4B10: MIPI TX */
			for (reg = 0x4B00; reg <= 0x4B10; reg++) {
				if (!hm2051_read_reg(client, reg, &val))
					dev_info(&client->dev, "REG[0x%04x]=0x%02x\n", reg, val);
			}
		}

		dev->is_streaming = true;
	} else {
		hm2051_write_reg_array(client, hm2051_stream_off_regs,
				       ARRAY_SIZE(hm2051_stream_off_regs));
		dev->is_streaming = false;
	}

out:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int hm2051_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_frame_interval *interval)
{
	if (interval->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	interval->interval.numerator = 1;
	interval->interval.denominator = HM2051_FPS;

	return 0;
}

static int hm2051_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	return 0;
}

static int hm2051_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;

	fse->min_width = HM2051_NATIVE_WIDTH;
	fse->max_width = HM2051_NATIVE_WIDTH;
	fse->min_height = HM2051_NATIVE_HEIGHT;
	fse->max_height = HM2051_NATIVE_HEIGHT;

	return 0;
}

static int hm2051_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	*frames = HM2051_SKIP_FRAMES;
	return 0;
}

static const struct v4l2_subdev_sensor_ops hm2051_sensor_ops = {
	.g_skip_frames	= hm2051_g_skip_frames,
};

static const struct v4l2_subdev_core_ops hm2051_core_ops = {
	.s_power = hm2051_s_power,
};

static const struct v4l2_subdev_video_ops hm2051_video_ops = {
	.s_stream = hm2051_s_stream,
};

static const struct v4l2_subdev_pad_ops hm2051_pad_ops = {
	.enum_mbus_code = hm2051_enum_mbus_code,
	.enum_frame_size = hm2051_enum_frame_size,
	.get_fmt = hm2051_get_fmt,
	.set_fmt = hm2051_set_fmt,
	.get_frame_interval = hm2051_get_frame_interval,
};

static const struct v4l2_subdev_ops hm2051_ops = {
	.core = &hm2051_core_ops,
	.video = &hm2051_video_ops,
	.pad = &hm2051_pad_ops,
	.sensor = &hm2051_sensor_ops,
};

static int hm2051_init_controls(struct hm2051_device *dev)
{
	struct v4l2_ctrl_handler *hdl = &dev->ctrls.handler;

	v4l2_ctrl_handler_init(hdl, 2);

	hdl->lock = &dev->input_lock;
	dev->sd.ctrl_handler = hdl;

	/*
	 * Exposure: 16-bit coarse integration time in lines. The frame is
	 * ~1300 lines at 30 fps; longer exposures stretch the frame (2048
	 * gives ~19 fps), so default to just under one frame.
	 */
	dev->ctrls.exposure =
		v4l2_ctrl_new_std(hdl, &hm2051_ctrl_ops, V4L2_CID_EXPOSURE,
				  1, 0xFFFF, 1, 1280);

	/* Analog gain: 0=1x, 1=2x, 2=4x, 3=8x */
	dev->ctrls.gain =
		v4l2_ctrl_new_std(hdl, &hm2051_ctrl_ops, V4L2_CID_GAIN,
				  0, 3, 1, 3);

	return hdl->error;
}

static void hm2051_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hm2051_device *dev = to_hm2051_sensor(sd);

	dev_dbg(&client->dev, "hm2051_remove\n");

	/* Stop streaming if active */
	if (dev->is_streaming)
		hm2051_s_stream(sd, 0);

	if (dev->power_count > 0)
		hm2051_power_off(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&dev->sd.entity);
	v4l2_ctrl_handler_free(&dev->ctrls.handler);
	atomisp_gmin_remove_subdev(sd);
	mutex_destroy(&dev->input_lock);
}

static int hm2051_probe(struct i2c_client *client)
{
	struct device *dev_p = &client->dev;
	struct hm2051_device *dev;
	int ret;

	dev = devm_kzalloc(dev_p, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->input_lock);

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&dev->sd, client, &hm2051_ops);
	hm2051_fill_format(&dev->mode.fmt);

	/*
	 * Get GMIN platform data for PMIC power control.
	 * This sets up the power rails, clocks, and GPIOs through the
	 * Cherry Trail GMIN platform layer.
	 */
	dev->platform_data = gmin_camera_platform_data(&dev->sd,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr);
	if (!dev->platform_data)
		dev_warn(dev_p, "No GMIN platform data, power management may not work\n");

	/*
	 * Set up CSI-2 metadata as v4l2 subdev hostdata for ATOMISP.
	 * Front camera: SECONDARY port (port 0), 1 lane, RAW10 BGGR.
	 */
	dev->csi.port = ATOMISP_CAMERA_PORT_SECONDARY;
	dev->csi.num_lanes = 1;
	dev->csi.input_format = ATOMISP_INPUT_FORMAT_RAW_10;
	dev->csi.raw_bayer_order = atomisp_bayer_order_bggr;
	dev->csi.metadata_format = ATOMISP_INPUT_FORMAT_EMBEDDED;
	v4l2_set_subdev_hostdata(&dev->sd, &dev->csi);

	dev_info(dev_p, "HM2051 sensor matched via ACPI\n");

	/* Initialize controls */
	ret = hm2051_init_controls(dev);
	if (ret) {
		dev_err(dev_p, "Failed to init controls: %d\n", ret);
		return ret;
	}

	/* Initialize media entity */
	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&dev->sd.entity, 1, &dev->pad);
	if (ret) {
		dev_err(dev_p, "Failed to init media entity: %d\n", ret);
		goto ctrl_cleanup;
	}

	ret = v4l2_async_register_subdev(&dev->sd);
	if (ret) {
		dev_err(dev_p, "Failed to register async subdev: %d\n", ret);
		goto entity_cleanup;
	}

	dev_info(dev_p, "HM2051 registered successfully\n");
	return 0;

entity_cleanup:
	media_entity_cleanup(&dev->sd.entity);
ctrl_cleanup:
	v4l2_ctrl_handler_free(&dev->ctrls.handler);
	return ret;
}

static const struct acpi_device_id hm2051_acpi_match[] = {
	{"HIMX2051"},
	{},
};
MODULE_DEVICE_TABLE(acpi, hm2051_acpi_match);

static struct i2c_driver hm2051_driver = {
	.driver = {
		.name = "hm2051",
		.acpi_match_table = hm2051_acpi_match,
	},
	.probe = hm2051_probe,
	.remove = hm2051_remove,
};
module_i2c_driver(hm2051_driver);

MODULE_AUTHOR("Claude / Anthropic");
MODULE_DESCRIPTION("A low-level driver for Himax HM2051 2MP camera sensor");
MODULE_LICENSE("GPL");
