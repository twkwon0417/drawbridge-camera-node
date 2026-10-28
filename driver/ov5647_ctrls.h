// SPDX-License-Identifier: GPL-2.0
/*
 * Custom OV5647 V4L2 sensor driver for running the camera without the
 * Raspberry Pi camera firmware (start_x=0).
 *
 * Derived from drivers/media/i2c/ov5647.c in the Raspberry Pi Linux 4.19 tree
 * (Copyright (C) 2016, Synopsys, Inc.; author Ramiro Oliveira), which is
 * itself based on the Samsung S5K6AAFX and OmniVision OV7670 drivers.
 */

#ifndef OV5647_CTRLS_H
#define OV5647_CTRLS_H

#define OV5647_REG_EXP_HI	0x3500
#define OV5647_REG_EXP_MID	0x3501
#define OV5647_REG_EXP_LO	0x3502

#define OV5647_REG_GAIN_HI	0x350A
#define OV5647_REG_GAIN_LO	0x350B

#define OV5647_REG_TIMING_TC_V	0x3820
#define OV5647_REG_TIMING_TC_H	0x3821

#define OV5647_REG_ISPCTRL3D	0x503D

static const char * const ov5647_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Color Squares",
	"Random Data",
};
static const u8 ov5647_test_pattern_val[] = {
	0x00,
	0x80,
	0x82,
	0x81,
};

static int ov5647_hw_set_exposure(struct v4l2_subdev *sd, u32 val)
{
	u32 reg = val << 4;
	int ret;

	ret = reg_write(sd, OV5647_REG_EXP_HI, (reg >> 16) & 0xff);
	if (ret < 0)
		return ret;
	ret = reg_write(sd, OV5647_REG_EXP_MID, (reg >> 8) & 0xff);
	if (ret < 0)
		return ret;
	return reg_write(sd, OV5647_REG_EXP_LO, reg & 0xff);
}

static int ov5647_hw_set_gain(struct v4l2_subdev *sd, u32 val)
{
	int ret;

	val &= 0x3ff;
	ret = reg_write(sd, OV5647_REG_GAIN_HI, (val >> 8) & 0x03);
	if (ret < 0)
		return ret;
	return reg_write(sd, OV5647_REG_GAIN_LO, val & 0xff);
}

static int ov5647_hw_set_hflip(struct v4l2_subdev *sd, bool on)
{
	u8 v;
	int ret = reg_read(sd, OV5647_REG_TIMING_TC_H, &v);

	if (ret < 0)
		return ret;
	if (on)
		v &= ~BIT(1);
	else
		v |= BIT(1);
	return reg_write(sd, OV5647_REG_TIMING_TC_H, v);
}

static int ov5647_hw_set_vflip(struct v4l2_subdev *sd, bool on)
{
	u8 v;
	int ret = reg_read(sd, OV5647_REG_TIMING_TC_V, &v);

	if (ret < 0)
		return ret;
	if (on)
		v |= BIT(1);
	else
		v &= ~BIT(1);
	return reg_write(sd, OV5647_REG_TIMING_TC_V, v);
}

static int ov5647_hw_set_test_pattern(struct v4l2_subdev *sd, int idx)
{
	if (idx < 0 || idx >= (int)ARRAY_SIZE(ov5647_test_pattern_val))
		return -EINVAL;
	return reg_write(sd, OV5647_REG_ISPCTRL3D, ov5647_test_pattern_val[idx]);
}

#endif
