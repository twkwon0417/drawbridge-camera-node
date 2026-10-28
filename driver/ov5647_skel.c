// SPDX-License-Identifier: GPL-2.0
/*
 * Custom OV5647 V4L2 sensor driver for running the camera without the
 * Raspberry Pi camera firmware (start_x=0).
 *
 * Derived from drivers/media/i2c/ov5647.c in the Raspberry Pi Linux 4.19 tree
 * (Copyright (C) 2016, Synopsys, Inc.; author Ramiro Oliveira), which is
 * itself based on the Samsung S5K6AAFX and OmniVision OV7670 drivers.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/bitops.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define OV5647_REG_CHIPID_H	0x300A
#define OV5647_REG_CHIPID_L	0x300B
#define OV5647_CHIP_ID		0x5647

#define OV5647_DEFAULT_WIDTH	640
#define OV5647_DEFAULT_HEIGHT	480
#define OV5647_XCLK_FREQ	25000000U

struct regval_list {
	u16 addr;
	u8 data;
};

struct ov5647 {
	struct v4l2_subdev	sd;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_mbus_framefmt format;
	struct mutex		lock;
	struct clk		*xclk;
	struct gpio_desc	*pwdn;
	int			power_count;
	unsigned int		flags;
};

static inline struct ov5647 *to_sensor(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5647, sd);
}

static int reg_write(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	int ret;

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret < 0) {
		dev_err(&client->dev, "reg_write 실패 reg=0x%04x val=0x%02x ret=%d\n",
			reg, val, ret);
		return ret;
	}
	return 0;
}

static int reg_read(struct v4l2_subdev *sd, u16 reg, u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 addr[2] = { reg >> 8, reg & 0xff };
	int ret;

	ret = i2c_master_send(client, addr, sizeof(addr));
	if (ret < 0) {
		dev_err(&client->dev, "reg_read addr 전송 실패 reg=0x%04x ret=%d\n",
			reg, ret);
		return ret;
	}
	ret = i2c_master_recv(client, val, 1);
	if (ret < 0) {
		dev_err(&client->dev, "reg_read 수신 실패 reg=0x%04x ret=%d\n",
			reg, ret);
		return ret;
	}
	return 0;
}

static int reg_write_array(struct v4l2_subdev *sd,
			   const struct regval_list *regs, int count)
{
	int i, ret;

	for (i = 0; i < count; i++) {
		ret = reg_write(sd, regs[i].addr, regs[i].data);
		if (ret < 0)
			return ret;
	}
	return 0;
}

#include "ov5647_hw.h"
#include "ov5647_ctrls.h"

static int ov5647_detect_chip(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 hi, lo;
	u16 id;
	int ret;

	ret = reg_read(sd, OV5647_REG_CHIPID_H, &hi);
	if (ret < 0)
		return ret;
	ret = reg_read(sd, OV5647_REG_CHIPID_L, &lo);
	if (ret < 0)
		return ret;

	id = ((u16)hi << 8) | lo;
	if (id != OV5647_CHIP_ID) {
		dev_err(&client->dev,
			"칩 ID 불일치: 0x%04x (기대값 0x%04x)\n", id, OV5647_CHIP_ID);
		return -ENODEV;
	}
	dev_info(&client->dev, "OV5647 검출 성공 (chip id 0x%04x)\n", id);
	return 0;
}

static int ov5647_set_power(struct v4l2_subdev *sd, int on)
{
	struct ov5647 *sensor = to_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);

	if (on && sensor->power_count == 0) {

		if (sensor->pwdn) {
			gpiod_set_value_cansleep(sensor->pwdn, 0);
			msleep(PWDN_ACTIVE_DELAY_MS);
		}

		clk_set_rate(sensor->xclk, OV5647_XCLK_FREQ);
		ret = clk_prepare_enable(sensor->xclk);
		if (ret < 0) {
			dev_err(&client->dev, "xclk 활성화 실패: %d\n", ret);
			goto out;
		}
		ret = reg_write_array(sd, sensor_oe_enable_regs,
				      ARRAY_SIZE(sensor_oe_enable_regs));
		if (ret < 0) {
			clk_disable_unprepare(sensor->xclk);
			dev_err(&client->dev, "OE enable 실패: %d\n", ret);
			goto out;
		}

		ret = ov5647_hw_sensor_init(sd);
		if (ret < 0) {
			clk_disable_unprepare(sensor->xclk);
			dev_err(&client->dev,
				"센서 init 실패(전원/배선 확인): %d\n", ret);
			goto out;
		}
	} else if (!on && sensor->power_count == 1) {

		reg_write_array(sd, sensor_oe_disable_regs,
				ARRAY_SIZE(sensor_oe_disable_regs));
		ov5647_hw_set_sw_standby(sd, true);
		clk_disable_unprepare(sensor->xclk);
		if (sensor->pwdn)
			gpiod_set_value_cansleep(sensor->pwdn, 1);
	}

	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int ov5647_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{

	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR8_1X8;
	return 0;
}

static int ov5647_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct ov5647 *sensor = to_sensor(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		*fmt = *v4l2_subdev_get_try_format(sd, cfg, format->pad);
	else
		*fmt = sensor->format;

	return 0;
}

static int ov5647_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct ov5647 *sensor = to_sensor(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;

	fmt->code	= MEDIA_BUS_FMT_SBGGR8_1X8;
	fmt->width	= OV5647_DEFAULT_WIDTH;
	fmt->height	= OV5647_DEFAULT_HEIGHT;
	fmt->field	= V4L2_FIELD_NONE;
	fmt->colorspace	= V4L2_COLORSPACE_SRGB;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		*v4l2_subdev_get_try_format(sd, cfg, format->pad) = *fmt;
	else
		sensor->format = *fmt;

	return 0;
}

static int ov5647_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov5647 *sensor = to_sensor(sd);
	int ret;

	mutex_lock(&sensor->lock);

	if (enable)
		ret = ov5647_hw_stream_on(sd, sensor->flags);
	else
		ret = ov5647_hw_stream_off(sd);
	mutex_unlock(&sensor->lock);
	return ret;
}

static int ov5647_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5647 *sensor = container_of(ctrl->handler,
					     struct ov5647, ctrls);
	struct v4l2_subdev *sd = &sensor->sd;
	int ret;

	if (sensor->power_count == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = ov5647_hw_set_exposure(sd, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ov5647_hw_set_gain(sd, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = ov5647_hw_set_hflip(sd, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = ov5647_hw_set_vflip(sd, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov5647_hw_set_test_pattern(sd, ctrl->val);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static const struct v4l2_ctrl_ops ov5647_ctrl_ops = {
	.s_ctrl = ov5647_s_ctrl,
};

static int ov5647_init_controls(struct ov5647 *sensor)
{
	struct v4l2_ctrl_handler *hdl = &sensor->ctrls;
	int ret;

	v4l2_ctrl_handler_init(hdl, 8);

	v4l2_ctrl_new_std(hdl, &ov5647_ctrl_ops,
			  V4L2_CID_EXPOSURE, 4, 65535, 1, 1000);
	v4l2_ctrl_new_std(hdl, &ov5647_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 16, 1023, 1, 32);
	v4l2_ctrl_new_std(hdl, &ov5647_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(hdl, &ov5647_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std_menu_items(hdl, &ov5647_ctrl_ops,
			V4L2_CID_TEST_PATTERN,
			ARRAY_SIZE(ov5647_test_pattern_menu) - 1, 0, 0,
			ov5647_test_pattern_menu);

	if (hdl->error) {
		ret = hdl->error;
		dev_err(sensor->sd.dev, "control 등록 실패: %d\n", ret);
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	sensor->sd.ctrl_handler = hdl;
	return 0;
}

static const struct v4l2_subdev_core_ops ov5647_core_ops = {
	.s_power = ov5647_set_power,
};

static const struct v4l2_subdev_video_ops ov5647_video_ops = {
	.s_stream = ov5647_s_stream,
};

static const struct v4l2_subdev_pad_ops ov5647_pad_ops = {
	.enum_mbus_code = ov5647_enum_mbus_code,
	.get_fmt	= ov5647_get_fmt,
	.set_fmt	= ov5647_set_fmt,
};

static const struct v4l2_subdev_ops ov5647_subdev_ops = {
	.core	= &ov5647_core_ops,
	.video	= &ov5647_video_ops,
	.pad	= &ov5647_pad_ops,
};

static int ov5647_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);

	try_fmt->code		= MEDIA_BUS_FMT_SBGGR8_1X8;
	try_fmt->width		= OV5647_DEFAULT_WIDTH;
	try_fmt->height		= OV5647_DEFAULT_HEIGHT;
	try_fmt->field		= V4L2_FIELD_NONE;
	try_fmt->colorspace	= V4L2_COLORSPACE_SRGB;
	return 0;
}

static const struct v4l2_subdev_internal_ops ov5647_internal_ops = {
	.open = ov5647_open,
};

static int ov5647_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ov5647 *sensor;
	struct v4l2_subdev *sd;
	struct fwnode_handle *endpoint;

	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2,
	};
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "DT 에 endpoint 노드가 없음\n");
		return -EINVAL;
	}
	ret = v4l2_fwnode_endpoint_parse(endpoint, &bus_cfg);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "endpoint 파싱 실패: %d\n", ret);
		return ret;
	}

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != 2)
		dev_warn(dev, "data-lanes 가 2 가 아님(%u) — overlay 확인\n",
			 bus_cfg.bus.mipi_csi2.num_data_lanes);
	sensor->flags = bus_cfg.bus.mipi_csi2.flags;

	sensor->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "xclk 획득 실패\n");
		return PTR_ERR(sensor->xclk);
	}
	if (clk_get_rate(sensor->xclk) != OV5647_XCLK_FREQ)
		dev_warn(dev, "xclk 가 %lu Hz (기대값 %u Hz)\n",
			 clk_get_rate(sensor->xclk), OV5647_XCLK_FREQ);

	sensor->pwdn = devm_gpiod_get_optional(dev, "pwdn", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->pwdn)) {
		dev_err(dev, "pwdn GPIO 획득 실패\n");
		return PTR_ERR(sensor->pwdn);
	}

	mutex_init(&sensor->lock);

	sensor->format.code		= MEDIA_BUS_FMT_SBGGR8_1X8;
	sensor->format.width		= OV5647_DEFAULT_WIDTH;
	sensor->format.height		= OV5647_DEFAULT_HEIGHT;
	sensor->format.field		= V4L2_FIELD_NONE;
	sensor->format.colorspace	= V4L2_COLORSPACE_SRGB;

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &ov5647_subdev_ops);
	sd->internal_ops = &ov5647_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret < 0)
		goto err_mutex;

	ret = ov5647_init_controls(sensor);
	if (ret < 0)
		goto err_entity;

	ret = ov5647_set_power(sd, 1);
	if (ret < 0)
		goto err_ctrls;
	ret = ov5647_detect_chip(sd);
	ov5647_set_power(sd, 0);
	if (ret < 0)
		goto err_ctrls;

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(dev, "async subdev 등록 실패: %d\n", ret);
		goto err_ctrls;
	}

	dev_info(dev, "ov5647-skel 프로브 완료 (compatible: myorg,ov5647-skel)\n");
	return 0;

err_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls);
err_entity:
	media_entity_cleanup(&sd->entity);
err_mutex:
	mutex_destroy(&sensor->lock);
	return ret;
}

static int ov5647_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5647 *sensor = to_sensor(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&sensor->lock);
	return 0;
}

static const struct of_device_id ov5647_skel_of_match[] = {
	{ .compatible = "myorg,ov5647-skel" },
	{  }
};
MODULE_DEVICE_TABLE(of, ov5647_skel_of_match);

static const struct i2c_device_id ov5647_skel_id[] = {
	{ "ov5647-skel", 0 },
	{  }
};
MODULE_DEVICE_TABLE(i2c, ov5647_skel_id);

static struct i2c_driver ov5647_skel_driver = {
	.driver = {
		.name		= "ov5647-skel",
		.of_match_table	= of_match_ptr(ov5647_skel_of_match),
	},

	.probe		= ov5647_probe,
	.remove		= ov5647_remove,
	.id_table	= ov5647_skel_id,
};

module_i2c_driver(ov5647_skel_driver);

MODULE_LICENSE("GPL v2");
