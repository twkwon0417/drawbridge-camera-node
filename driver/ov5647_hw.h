// SPDX-License-Identifier: GPL-2.0
/*
 * Custom OV5647 V4L2 sensor driver for running the camera without the
 * Raspberry Pi camera firmware (start_x=0).
 *
 * Derived from drivers/media/i2c/ov5647.c in the Raspberry Pi Linux 4.19 tree
 * (Copyright (C) 2016, Synopsys, Inc.; author Ramiro Oliveira), which is
 * itself based on the Samsung S5K6AAFX and OmniVision OV7670 drivers.
 */

#ifndef OV5647_HW_H
#define OV5647_HW_H

#define OV5647_SW_STANDBY		0x0100
#define OV5647_SW_RESET			0x0103
#define OV5647_REG_FRAME_OFF_NUMBER	0x4202
#define OV5647_REG_MIPI_CTRL00		0x4800
#define OV5647_REG_MIPI_CTRL14		0x4814
#define OV5647_REG_PAD_OUT		0x300D
#define PWDN_ACTIVE_DELAY_MS		20

#define MIPI_CTRL00_CLOCK_LANE_GATE	BIT(5)
#define MIPI_CTRL00_LINE_SYNC_ENABLE	BIT(4)
#define MIPI_CTRL00_BUS_IDLE		BIT(2)
#define MIPI_CTRL00_CLOCK_LANE_DISABLE	BIT(0)

static struct regval_list sensor_oe_disable_regs[] = {
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
};

static struct regval_list sensor_oe_enable_regs[] = {
	{0x3000, 0x0f},
	{0x3001, 0xff},
	{0x3002, 0xe4},
};

static struct regval_list ov5647_640x480[] = {
	{0x0100, 0x00},
	{0x0103, 0x01},
	{0x3034, 0x08},
	{0x3035, 0x21},
	{0x3036, 0x46},
	{0x303c, 0x11},
	{0x3106, 0xf5},
	{0x3821, 0x07},
	{0x3820, 0x41},
	{0x3827, 0xec},
	{0x370c, 0x0f},
	{0x3612, 0x59},
	{0x3618, 0x00},
	{0x5000, 0x06},
	{0x5001, 0x01},
	{0x5002, 0x41},
	{0x5003, 0x08},
	{0x5a00, 0x08},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3016, 0x08},
	{0x3017, 0xe0},
	{0x3018, 0x44},
	{0x301c, 0xf8},
	{0x301d, 0xf0},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3c01, 0x80},
	{0x3b07, 0x0c},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x380e, 0x03},
	{0x380f, 0xd8},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3708, 0x64},
	{0x3709, 0x52},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xE0},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0xa1},
	{0x3811, 0x08},
	{0x3813, 0x02},
	{0x3630, 0x2e},
	{0x3632, 0xe2},
	{0x3633, 0x23},
	{0x3634, 0x44},
	{0x3636, 0x06},
	{0x3620, 0x64},
	{0x3621, 0xe0},
	{0x3600, 0x37},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x3731, 0x02},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3f05, 0x02},
	{0x3f06, 0x10},
	{0x3f01, 0x0a},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0d, 0x04},
	{0x3a0e, 0x03},
	{0x3a0f, 0x58},
	{0x3a10, 0x50},
	{0x3a1b, 0x58},
	{0x3a1e, 0x50},
	{0x3a11, 0x60},
	{0x3a1f, 0x28},
	{0x4001, 0x02},
	{0x4004, 0x02},
	{0x4000, 0x09},
	{0x4837, 0x24},
	{0x4050, 0x6e},
	{0x4051, 0x8f},
	{0x0100, 0x01},
};

static int ov5647_hw_set_virtual_channel(struct v4l2_subdev *sd, int channel)
{
	u8 channel_id;
	int ret;

	ret = reg_read(sd, OV5647_REG_MIPI_CTRL14, &channel_id);
	if (ret < 0)
		return ret;
	channel_id &= ~(3 << 6);
	return reg_write(sd, OV5647_REG_MIPI_CTRL14,
			 channel_id | (channel << 6));
}

static int ov5647_hw_stream_on(struct v4l2_subdev *sd, unsigned int flags)
{
	u8 val = MIPI_CTRL00_BUS_IDLE;
	int ret;

	if (flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK)
		val |= MIPI_CTRL00_CLOCK_LANE_GATE |
		       MIPI_CTRL00_LINE_SYNC_ENABLE;

	ret = reg_write(sd, OV5647_REG_MIPI_CTRL00, val);
	if (ret < 0)
		return ret;
	ret = reg_write(sd, OV5647_REG_FRAME_OFF_NUMBER, 0x00);
	if (ret < 0)
		return ret;
	return reg_write(sd, OV5647_REG_PAD_OUT, 0x00);
}

static int ov5647_hw_stream_off(struct v4l2_subdev *sd)
{
	int ret;

	ret = reg_write(sd, OV5647_REG_MIPI_CTRL00,
			MIPI_CTRL00_CLOCK_LANE_GATE |
			MIPI_CTRL00_BUS_IDLE |
			MIPI_CTRL00_CLOCK_LANE_DISABLE);
	if (ret < 0)
		return ret;
	ret = reg_write(sd, OV5647_REG_FRAME_OFF_NUMBER, 0x0f);
	if (ret < 0)
		return ret;
	return reg_write(sd, OV5647_REG_PAD_OUT, 0x01);
}

static int ov5647_hw_set_sw_standby(struct v4l2_subdev *sd, bool standby)
{
	u8 rdval;
	int ret;

	ret = reg_read(sd, OV5647_SW_STANDBY, &rdval);
	if (ret < 0)
		return ret;
	if (standby)
		rdval &= ~0x01;
	else
		rdval |= 0x01;
	return reg_write(sd, OV5647_SW_STANDBY, rdval);
}

static int ov5647_hw_sensor_init(struct v4l2_subdev *sd)
{
	u8 resetval, rdval;
	int ret;

	ret = reg_read(sd, OV5647_SW_STANDBY, &rdval);
	if (ret < 0)
		return ret;

	ret = reg_write_array(sd, ov5647_640x480, ARRAY_SIZE(ov5647_640x480));
	if (ret < 0)
		return ret;

	ret = ov5647_hw_set_virtual_channel(sd, 0);
	if (ret < 0)
		return ret;

	ret = reg_read(sd, OV5647_SW_STANDBY, &resetval);
	if (ret < 0)
		return ret;
	if (!(resetval & 0x01)) {
		ret = reg_write(sd, OV5647_SW_STANDBY, 0x01);
		if (ret < 0)
			return ret;
	}

	return ov5647_hw_stream_off(sd);
}

#endif
