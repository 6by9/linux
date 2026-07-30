// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX675 CMOS Image Sensor Driver
 *
 * The IMX675 is the successor of IMX290/327/462, 1920x1080 1/2.8 CMOS image sensors.
 *
 * Copyright (C) 2022 Soho Enterprise Ltd.
 * Author: Tetsuya Nomura <tetsuya.nomura@soho-enterprise.com>
 *
 * Based on IMX290 driver
 * Copyright (C) 2019 FRAMOS GmbH.
 * and
 * Copyright (C) 2019 Linaro Ltd.
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX675_STANDBY		0x3000
#define IMX675_REGHOLD		0x3001
#define IMX675_XMSTA		0x3002
#define IMX675_INCK_SEL		0x3014
	#define IMX675_INCK_SEL_74_25	0x00
	#define IMX675_INCK_SEL_37_125	0x01
	#define IMX675_INCK_SEL_72	0x02
	#define IMX675_INCK_SEL_27	0x03
	#define IMX675_INCK_SEL_24	0x04
#define IMX675_LANE_RATE	0x3015
	#define IMX675_LANE_RATE_2376	0x00
	#define IMX675_LANE_RATE_2079	0x01
	#define IMX675_LANE_RATE_1782	0x02
	#define IMX675_LANE_RATE_1440	0x03
	#define IMX675_LANE_RATE_1188	0x04
	#define IMX675_LANE_RATE_891	0x05
	#define IMX675_LANE_RATE_720	0x06
	#define IMX675_LANE_RATE_594	0x07
#define IMX675_FLIP_WINMODEH	0x3020
#define IMX675_FLIP_WINMODEV	0x3021
#define IMX675_ADBIT		0x3022
#define IMX675_MDBIT		0x3023
#define IMX675_VMAX		0x3028
	#define IMX675_VMAX_MAX		0x03ffff
#define IMX675_HMAX		0x302c
	#define IMX675_HMAX_MAX		0xffff
#define IMX675_FR_FDG_SEL0	0x3030
	#define IMX675_FDG_SEL0_LCG	0x00
	#define IMX675_FDG_SEL0_HCG	0x01
#define IMX675_FR_FDG_SEL1	0x3031
#define IMX675_FR_FDG_SEL2	0x3032
#define IMX675_CSI_LANE_MODE	0x3040
#define IMX675_EXPOSURE		0x3050
#define IMX675_GAIN		0x3070

#define IMX675_EXPOSURE_MIN	8
#define IMX675_EXPOSURE_STEP	1
/* Exposure must be this many lines less than VMAX */
#define IMX675_EXPOSURE_OFFSET  4

#define IMX675_NATIVE_WIDTH		2608U
#define IMX675_NATIVE_HEIGHT		1984U
#define IMX675_PIXEL_ARRAY_LEFT		8U
#define IMX675_PIXEL_ARRAY_TOP		20U
#define IMX675_PIXEL_ARRAY_WIDTH	2592U
#define IMX675_PIXEL_ARRAY_HEIGHT	1944U

static const char * const imx675_supply_name[] = {
	"vdda",
	"vddd",
	"vdddo",
};

#define IMX675_NUM_SUPPLIES ARRAY_SIZE(imx675_supply_name)

struct imx675_regval {
	u16 reg;
	u8 val;
};

struct imx675_mode {
	u32 width;
	u32 height;
	u32 hmax;
	u32 vmax;
	struct v4l2_rect crop;

	const struct imx675_regval *mode_data;
	u32 mode_data_size;
};

struct imx675 {
	struct device *dev;
	struct clk *xclk;
	u8 inck_sel;
	struct regmap *regmap;
	u8 nlanes;
	u8 bpp;

	const struct imx675_pixfmt *formats;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt current_format;
	const struct imx675_mode *current_mode;

	struct regulator_bulk_data supplies[IMX675_NUM_SUPPLIES];
	struct gpio_desc *rst_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *exposure;

	struct mutex lock;
};

struct imx675_pixfmt {
	u32 code;
	u8 bpp;
};

#define IMX675_NUM_FORMATS 2

static const struct imx675_pixfmt imx675_colour_formats[IMX675_NUM_FORMATS] = {
	{ MEDIA_BUS_FMT_SRGGB10_1X10, 10 },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, 12 },
};

static const struct imx675_pixfmt imx675_mono_formats[IMX675_NUM_FORMATS] = {
	{ MEDIA_BUS_FMT_Y10_1X10, 10 },
	{ MEDIA_BUS_FMT_Y12_1X12, 12 },
};

static const struct regmap_config imx675_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static const struct imx675_regval imx675_global_settings[] = {
	//{0x3002, 0x00}, //#Master mode operation start
	{0x301A, 0x08}, // HDR mode select (Normal) under debugging, set CHDR mode
	{0x301B, 0x00}, // Normal/binning
	{0x301C, 0x00}, // XVS sub sample
	{0x301E, 0x01}, // virtual channel
	{0x303C, 0x00}, // PIX HSTART
	{0x303D, 0x00}, // PIX HSTART
	{0x303E, 0x30}, // H WIDTH
	{0x303F, 0x0A}, // H WIDTH
	{0x3044, 0x00}, // PIX VSTART
	{0x3045, 0x00}, // PIX VSTART
	{0x3046, 0xAC}, // V WIDTH
	{0x3047, 0x07}, // V WIDTH

{0x3060, 0x95}, //    # RHS1[19:0]
{0x3061, 0x00}, //    # 
{0x3062, 0x00}, //    # 
{0x3064, 0x56}, //    # RHS2[19:0]
{0x3065, 0x00}, //    # 
{0x3066, 0x00}, //    # 

	//{0x3069, 0x00}, // Direct Gain Enable
	{0x3072, 0x00}, // GAIN SEF1
	{0x3073, 0x00}, // GAIN SEF1
	{0x3074, 0x00}, // GAIN SEF2
	{0x3075, 0x00}, // GAIN SEF2
	{0x3081, 0x00}, // EXP_GAIN
	//{0x308C, 0x00}, // Clear HDR DGAIN
	//{0x308D, 0x01}, // Clear HDR DGAIN
	//{0x3094, 0x00}, // CHDR AGAIN LG
	//{0x3095, 0x00}, // CHDR AGAIN LG
	//{0x3096, 0x00}, // CHDR AGAIN1
	//{0x3097, 0x00}, // CHDR AGAIN1
	//{0x309C, 0x00}, // CHDR AGAIN HG
	//{0x309D, 0x00}, // CHDR AGAIN HG
	{0x30A4, 0xAA}, // XVS/XHS OUT
	{0x30A6, 0x0F}, // XVS/XHS DRIVE HiZ
	{0x30CC, 0x00}, // XVS width
	{0x30CD, 0x00}, // XHS width
	{0x30CE, 0x02}, // Reserve
	{0x3400, 0x01}, // GAIN Adjust

{0x3400, 0x01}, //   # GAIN_PGC_FIDMD
{0x3460, 0x22}, //   # -
{0x3492, 0x08},//   # -
{0x3B1D, 0x17},//   # -
{0x3B44, 0x3F},//   # -
{0x3B60, 0x03},//   # -
{0x3C03, 0x04},//   # -
{0x3C04, 0x04},//   # -
{0x3C0A, 0x00},//   # -
{0x3C0B, 0x00},//    # -
{0x3C0C, 0x00},//    # -
{0x3C0D, 0x00},//    # -
{0x3C0E, 0x00},//    # -
{0x3C0F, 0x00},//    # -
{0x3C30, 0x73},//    # -
{0x3C3C, 0x20}, //   # -
{0x3C7C, 0xB9}, //   # -
{0x3C7D, 0x01}, //   # -
{0x3C7E, 0xB7}, //   # -
{0x3C7F, 0x01}, //   # -
{0x3CB0, 0x00}, //   # -
{0x3CB2, 0xFF}, //   # -
{0x3CB3, 0x03}, //   # -
{0x3CB4, 0xFF}, //   # -
{0x3CB5, 0x03}, //   # -
{0x3CBA, 0xFF}, //   # -
{0x3CBB, 0x03}, //   # -
{0x3CC0, 0xFF}, //   # -
{0x3CC1, 0x03}, //   # -
{0x3CC2, 0x00}, //   # -
{0x3CC6, 0xFF}, //   # -
{0x3CC7, 0x03}, //   # -
{0x3CC8, 0xFF}, //   # -
{0x3CC9, 0x03}, //   # -
{0x3E00, 0x1E}, //   # -
{0x3E02, 0x04}, //   # -
{0x3E03, 0x00}, //   # -
{0x3E20, 0x04}, //   # -
{0x3E21, 0x00}, //
{0x3E22, 0x1E}, //
{0x3E24, 0xBA}, //
{0x3E72, 0x85}, //
{0x3E76, 0x0C}, //

{0x3E77, 0x01}, //
{0x3E7A, 0x85}, //
{0x3E7E, 0x1F}, //
{0x3E82, 0xA6}, //
{0x3E86, 0x2D}, //
{0x3EE2, 0x33}, //
{0x3EE3, 0x03}, //
{0x4490, 0x07}, //
{0x4494, 0x19}, //
{0x4495, 0x00}, //
{0x4496, 0xBB}, //
{0x4497, 0x00}, //
{0x4498, 0x55}, //
{0x449A, 0x50}, //
{0x449C, 0x50}, //
{0x449E, 0x50}, //
{0x44A0, 0x3C}, //
{0x44A2, 0x19}, //
{0x44A4, 0x19}, //
{0x44A6, 0x19}, //
{0x44A8, 0x4B}, //
{0x44AA, 0x4B}, //
{0x44AC, 0x4B}, //
{0x44AE, 0x4B}, //
{0x44B0, 0x3C}, //
{0x44B2, 0x19}, //
{0x44B4, 0x19}, //
{0x44B6, 0x19}, //
{0x44B8, 0x4B}, //
{0x44BA, 0x4B}, //
{0x44BC, 0x4B}, //
{0x44BE, 0x4B}, //
{0x44C0, 0x3C}, //
{0x44C2, 0x19}, //
{0x44C4, 0x19}, //
{0x44C6, 0x19}, //
{0x44C8, 0xF0}, //
{0x44CA, 0xEB}, //
{0x44CC, 0xEB}, //
{0x44CE, 0xE6}, //
{0x44D0, 0xE6}, //
{0x44D2, 0xBB}, //

{0x44D4, 0xBB}, //
{0x44D6, 0xBB}, //
{0x44D8, 0xE6}, //
{0x44DA, 0xE6}, //
{0x44DC, 0xE6}, //
{0x44DE, 0xE6}, //
{0x44E0, 0xE6}, //
{0x44E2, 0xBB}, //
{0x44E4, 0xBB}, //
{0x44E6, 0xBB}, //
{0x44E8, 0xE6}, //
{0x44EA, 0xE6}, //
{0x44EC, 0xE6}, //
{0x44EE, 0xE6}, //
{0x44F0, 0xE6}, //
{0x44F2, 0xBB}, //
{0x44F4, 0xBB}, //
{0x44F6, 0xBB}, //
{0x4538, 0x15}, //
{0x4539, 0x15}, //
{0x453A, 0x15}, //
{0x4544, 0x15}, //
{0x4545, 0x15}, //
{0x4546, 0x15}, //
{0x4550, 0x11}, //
{0x4551, 0x11}, //
{0x4552, 0x11}, //
{0x4553, 0x11}, //
{0x4554, 0x11}, //
{0x4555, 0x11}, //
{0x4556, 0x11}, //
{0x4557, 0x11}, //
{0x4558, 0x11}, //
{0x455C, 0x11}, //
{0x455D, 0x11}, //
{0x455E, 0x11}, //
{0x455F, 0x11}, //
{0x4560, 0x11}, //
{0x4561, 0x11}, //
{0x4562, 0x11}, //
{0x4563, 0x11}, //
{0x4564, 0x11}, //

{0x4569, 0x01}, //
{0x456A, 0x01}, //
{0x456B, 0x06}, //
{0x456C, 0x06}, //
{0x456D, 0x06}, //
{0x456E, 0x06}, //
{0x456F, 0x06}, //
{0x4570, 0x06}, //

};

static const struct imx675_regval imx675_5mpix_common_settings[] = {
	/* mode settings */
	{0x3018, 0x00}, // WINMODE
	{ IMX675_FR_FDG_SEL1, 0x00 },
	{ IMX675_FR_FDG_SEL2, 0x00 },
};

/* supported link frequencies */
static const s64 imx675_link_freq_2lanes[] = {
	594000000,
};

static const s64 imx675_link_freq_4lanes[] = {
	297000000,
};

/*
 * In this function and in the similar ones below we rely on imx675_probe()
 * to ensure that nlanes is either 2 or 4.
 */
static inline const s64 *imx675_link_freqs_ptr(const struct imx675 *imx675)
{
	if (imx675->nlanes == 2)
		return imx675_link_freq_2lanes;
	else
		return imx675_link_freq_4lanes;
}

static inline int imx675_link_freqs_num(const struct imx675 *imx675)
{
	if (imx675->nlanes == 2)
		return ARRAY_SIZE(imx675_link_freq_2lanes);
	else
		return ARRAY_SIZE(imx675_link_freq_4lanes);
}

/* Mode configs */
static const struct imx675_mode imx675_modes[] = {
	{
		/*
		 * Note that this mode reads out the areas documented as
		 * "effective matrgin for color processing" and "effective pixel
		 * ignored area" in the datasheet.
		 */
		.width = 2608,
		.height = 1984,
		.hmax = 0x0465 * 3, // 
		.vmax = 0x1130, //
		.crop = {
			.left = IMX675_PIXEL_ARRAY_LEFT,
			.top = IMX675_PIXEL_ARRAY_TOP,
			.width = IMX675_NATIVE_WIDTH,
			.height = IMX675_NATIVE_HEIGHT,
		},
		.mode_data = imx675_5mpix_common_settings,
		.mode_data_size = ARRAY_SIZE(imx675_5mpix_common_settings),
	},
};

#define IMX675_NUM_MODES ARRAY_SIZE(imx675_modes)

static inline struct imx675 *to_imx675(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx675, sd);
}

static inline int imx675_read_reg(struct imx675 *imx675, u16 addr, u8 *value)
{
	unsigned int regval;
	int ret;

	ret = regmap_read(imx675->regmap, addr, &regval);
	if (ret) {
		dev_err(imx675->dev, "I2C read failed for addr: %x\n", addr);
		return ret;
	}

	*value = regval & 0xff;

	return 0;
}

static int imx675_write_reg(struct imx675 *imx675, u16 addr, u8 value)
{
	int ret;

	ret = regmap_write(imx675->regmap, addr, value);
	if (ret) {
		dev_err(imx675->dev, "I2C write failed for addr: %x\n", addr);
		return ret;
	}

	return ret;
}

static int imx675_set_register_array(struct imx675 *imx675,
				     const struct imx675_regval *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		ret = imx675_write_reg(imx675, settings->reg, settings->val);
		if (ret < 0)
			return ret;
	}

	/* Provide 10ms settle time */
	usleep_range(10000, 11000);

	return 0;
}

static int imx675_write_buffered_reg(struct imx675 *imx675, u16 address_low,
				     u8 nr_regs, u32 value)
{
	unsigned int i;
	int ret;

	ret = imx675_write_reg(imx675, IMX675_REGHOLD, 0x01);
	if (ret) {
		dev_err(imx675->dev, "Error setting hold register\n");
		return ret;
	}

	for (i = 0; i < nr_regs; i++) {
		ret = imx675_write_reg(imx675, address_low + i,
				       (u8)(value >> (i * 8)));
		if (ret) {
			dev_err(imx675->dev, "Error writing buffered registers\n");
			return ret;
		}
	}

	ret = imx675_write_reg(imx675, IMX675_REGHOLD, 0x00);
	if (ret) {
		dev_err(imx675->dev, "Error setting hold register\n");
		return ret;
	}

	return ret;
}

static int imx675_set_gain(struct imx675 *imx675, u32 value)
{
	int ret;

	ret = imx675_write_buffered_reg(imx675, IMX675_GAIN, 2, value);
	if (ret) {
		dev_err(imx675->dev, "Unable to write gain\n");
		return ret;
	}

	ret = imx675_write_reg(imx675, IMX675_FR_FDG_SEL0, value < 0x22 ?
			       IMX675_FDG_SEL0_LCG : IMX675_FDG_SEL0_LCG);
	if (ret)
		dev_err(imx675->dev, "Unable to write LCG/HCG mode\n");

	return ret;
}

static int imx675_set_exposure(struct imx675 *imx675, u32 value)
{
	u32 exposure = (imx675->current_mode->height + imx675->vblank->val) -
						value - 1;
	int ret;

	ret = imx675_write_buffered_reg(imx675, IMX675_EXPOSURE, 3,
					exposure);
					//0x000004);
	if (ret)
		dev_err(imx675->dev, "Unable to write exposure\n");

	return ret;
}

static int imx675_set_hmax(struct imx675 *imx675, u32 val)
{
	u32 hmax = (val + imx675->current_mode->width) >> 1;
	int ret;

	ret = imx675_write_buffered_reg(imx675, IMX675_HMAX, 2,
					hmax);
	if (ret)
		dev_err(imx675->dev, "Error setting HMAX register\n");

	return ret;
}

static int imx675_set_vmax(struct imx675 *imx675, u32 val)
{
	u32 vmax = val + imx675->current_mode->height;

	int ret;

	ret = imx675_write_buffered_reg(imx675, IMX675_VMAX, 3,
					vmax);
					//0x000465);
	if (ret)
		dev_err(imx675->dev, "Unable to write vmax\n");

	/*
	 * Changing vblank changes the allowed range for exposure.
	 * We don't supply the current exposure as default here as it
	 * may lie outside the new range. We will reset it just below.
	 */
	__v4l2_ctrl_modify_range(imx675->exposure,
				 IMX675_EXPOSURE_MIN,
				 vmax - IMX675_EXPOSURE_OFFSET,
				 IMX675_EXPOSURE_STEP,
				 vmax - IMX675_EXPOSURE_OFFSET);

	/*
	 * Becuse of the way exposure works for this sensor, updating
	 * vblank causes the effective exposure to change, so we must
	 * set it back to the "new" correct value.
	 */
	imx675_set_exposure(imx675, imx675->exposure->val);

	return ret;
}

/* Stop streaming */
static int imx675_stop_streaming(struct imx675 *imx675)
{
	int ret;

	ret = imx675_write_reg(imx675, IMX675_STANDBY, 0x01);
	if (ret < 0)
		return ret;

	msleep(30);

	return imx675_write_reg(imx675, IMX675_XMSTA, 0x00);
}

static int imx675_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx675 *imx675 = container_of(ctrl->handler,
					     struct imx675, ctrls);
	int ret = 0;

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(imx675->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx675_set_gain(imx675, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx675_set_exposure(imx675, ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		ret = imx675_set_hmax(imx675, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = imx675_set_vmax(imx675, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = imx675_write_reg(imx675, IMX675_FLIP_WINMODEH, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = imx675_write_reg(imx675, IMX675_FLIP_WINMODEV, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx675->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx675_ctrl_ops = {
	.s_ctrl = imx675_set_ctrl,
};

static int imx675_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	const struct imx675 *imx675 = to_imx675(sd);

	if (code->index >= IMX675_NUM_FORMATS)
		return -EINVAL;

	code->code = imx675->formats[code->index].code;

	return 0;
}

static int imx675_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct imx675 *imx675 = to_imx675(sd);

	if (fse->code != imx675->formats[0].code &&
	    fse->code != imx675->formats[1].code)
		return -EINVAL;

	if (fse->index >= IMX675_NUM_MODES)
		return -EINVAL;

	fse->min_width = imx675_modes[fse->index].width;
	fse->max_width = imx675_modes[fse->index].width;
	fse->min_height = imx675_modes[fse->index].height;
	fse->max_height = imx675_modes[fse->index].height;

	return 0;
}

static int imx675_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx675 *imx675 = to_imx675(sd);
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&imx675->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		framefmt = v4l2_subdev_state_get_format(sd_state,
						      fmt->pad);
	else
		framefmt = &imx675->current_format;

	fmt->format = *framefmt;

	mutex_unlock(&imx675->lock);

	return 0;
}

static u64 imx675_calc_pixel_rate(struct imx675 *imx675)
{
	return 148500000;
}

static int imx675_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx675 *imx675 = to_imx675(sd);
	const struct imx675_mode *mode;
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	mutex_lock(&imx675->lock);

	mode = v4l2_find_nearest_size(imx675_modes, IMX675_NUM_MODES,
				      width, height,
				      fmt->format.width, fmt->format.height);

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;

	for (i = 0; i < IMX675_NUM_FORMATS; i++)
		if (imx675->formats[i].code == fmt->format.code)
			break;

	if (i >= IMX675_NUM_FORMATS)
		i = 0;

	fmt->format.code = imx675->formats[i].code;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc =
			V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->format.colorspace);
	fmt->format.quantization =
		V4L2_MAP_QUANTIZATION_DEFAULT(true, fmt->format.colorspace,
					      fmt->format.ycbcr_enc);
	fmt->format.xfer_func =
		V4L2_MAP_XFER_FUNC_DEFAULT(fmt->format.colorspace);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		format = v4l2_subdev_state_get_format(sd_state, fmt->pad);
	} else {
		format = &imx675->current_format;
		imx675->current_mode = mode;
		imx675->bpp = imx675->formats[i].bpp;

		if (imx675->pixel_rate)
			__v4l2_ctrl_s_ctrl_int64(imx675->pixel_rate,
						 imx675_calc_pixel_rate(imx675));

		if (imx675->hblank) {
			__v4l2_ctrl_modify_range(imx675->hblank,
						 mode->hmax - mode->width,
						 IMX675_HMAX_MAX - mode->width,
						 1, mode->hmax - mode->width);
			__v4l2_ctrl_s_ctrl(imx675->hblank,
					   mode->hmax - mode->width);
		}
		if (imx675->vblank) {
			__v4l2_ctrl_modify_range(imx675->vblank,
						 mode->vmax - mode->height,
						 IMX675_VMAX_MAX - mode->height,
						 1,
						 mode->vmax - mode->height);
			__v4l2_ctrl_s_ctrl(imx675->vblank,
					   mode->vmax - mode->height);
		}
		if (imx675->exposure)
			__v4l2_ctrl_modify_range(imx675->exposure,
						 IMX675_EXPOSURE_MIN,
						 mode->vmax - 2,
						 IMX675_EXPOSURE_STEP,
						 mode->vmax - 2);
	}

	*format = fmt->format;

	mutex_unlock(&imx675->lock);

	return 0;
}

static int imx675_init_state(struct v4l2_subdev *subdev,
			     struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 1936;
	fmt.format.height = 1100;

	imx675_set_fmt(subdev, sd_state, &fmt);

	return 0;
}

static int imx675_write_current_format(struct imx675 *imx675)
{
	u8 ad_md_bit;
	int ret;

	switch (imx675->current_format.code) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_Y10_1X10:
		ad_md_bit = 0x00;
		break;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_Y12_1X12:
		ad_md_bit = 0x01;
		break;
	default:
		dev_err(imx675->dev, "Unknown pixel format\n");
		return -EINVAL;
	}

	ret = imx675_write_reg(imx675, IMX675_ADBIT, ad_md_bit);
	if (ret < 0)
		return ret;

	ret = imx675_write_reg(imx675, IMX675_MDBIT, ad_md_bit);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct v4l2_rect *
__imx675_get_pad_crop(struct imx675 *imx675,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx675->current_mode->crop;
	}

	return NULL;
}

static int imx675_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx675 *imx675 = to_imx675(sd);

		mutex_lock(&imx675->lock);
		sel->r = *__imx675_get_pad_crop(imx675, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx675->lock);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = IMX675_NATIVE_WIDTH;
		sel->r.height = IMX675_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = IMX675_PIXEL_ARRAY_TOP;
		sel->r.left = IMX675_PIXEL_ARRAY_LEFT;
		sel->r.width = IMX675_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX675_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/* Start streaming */
static int imx675_start_streaming(struct imx675 *imx675)
{
	int ret;

	/* Set init register settings */
	ret = imx675_set_register_array(imx675, imx675_global_settings,
					ARRAY_SIZE(imx675_global_settings));
	if (ret < 0) {
		dev_err(imx675->dev, "Could not set init registers\n");
		return ret;
	}
pr_err("write INCK_SEL with %02x\n", imx675->inck_sel);
	ret = imx675_write_reg(imx675, IMX675_INCK_SEL, imx675->inck_sel);
	if (ret < 0)
		return ret;

	/* Apply the register values related to current frame format */
	ret = imx675_write_current_format(imx675);
	if (ret < 0) {
		dev_err(imx675->dev, "Could not set frame format\n");
		return ret;
	}

	/* Apply default values of current mode */
	ret = imx675_set_register_array(imx675,
					imx675->current_mode->mode_data,
					imx675->current_mode->mode_data_size);
	if (ret < 0) {
		dev_err(imx675->dev, "Could not set current mode\n");
		return ret;
	}

	/* Apply lane config registers of current mode */
	ret = imx675_write_reg(imx675, IMX675_CSI_LANE_MODE,
			       imx675->nlanes == 2 ? 0x01 : 0x03);
	if (ret < 0)
		return ret;

	ret = imx675_write_reg(imx675, IMX675_LANE_RATE,
			       imx675->nlanes == 2 ? IMX675_LANE_RATE_1188 :
						     IMX675_LANE_RATE_594);
	if (ret < 0)
		return ret;

	/* Apply customized values from user */
	ret = v4l2_ctrl_handler_setup(imx675->sd.ctrl_handler);
	if (ret) {
		dev_err(imx675->dev, "Could not sync v4l2 controls\n");
		return ret;
	}

	ret = imx675_write_reg(imx675, IMX675_STANDBY, 0x00);
	if (ret < 0)
		return ret;

	msleep(30);

	/* Start streaming */
	return imx675_write_reg(imx675, IMX675_XMSTA, 0x00);
}

static int imx675_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx675 *imx675 = to_imx675(sd);
	int ret = 0;

	if (enable) {
		ret = pm_runtime_resume_and_get(imx675->dev);
		if (ret < 0)
			goto unlock_and_return;

		ret = imx675_start_streaming(imx675);
		if (ret) {
			dev_err(imx675->dev, "Start stream failed\n");
			pm_runtime_put(imx675->dev);
			goto unlock_and_return;
		}
	} else {
		imx675_stop_streaming(imx675);
		pm_runtime_put(imx675->dev);
	}
	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx675->vflip, enable);
	__v4l2_ctrl_grab(imx675->hflip, enable);

unlock_and_return:

	return ret;
}

static int imx675_get_regulators(struct device *dev, struct imx675 *imx675)
{
	unsigned int i;

	for (i = 0; i < IMX675_NUM_SUPPLIES; i++)
		imx675->supplies[i].supply = imx675_supply_name[i];

	return devm_regulator_bulk_get(dev, IMX675_NUM_SUPPLIES,
				       imx675->supplies);
}

static int imx675_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx675 *imx675 = to_imx675(sd);
	int ret;

	ret = clk_prepare_enable(imx675->xclk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	ret = regulator_bulk_enable(IMX675_NUM_SUPPLIES, imx675->supplies);
	if (ret) {
		dev_err(dev, "Failed to enable regulators\n");
		clk_disable_unprepare(imx675->xclk);
		return ret;
	}

	usleep_range(1, 2);
	gpiod_set_value_cansleep(imx675->rst_gpio, 0);
	usleep_range(30000, 31000);

	return 0;
}

static int imx675_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx675 *imx675 = to_imx675(sd);

	clk_disable_unprepare(imx675->xclk);
	gpiod_set_value_cansleep(imx675->rst_gpio, 1);
	regulator_bulk_disable(IMX675_NUM_SUPPLIES, imx675->supplies);

	return 0;
}

static const struct dev_pm_ops imx675_pm_ops = {
	SET_RUNTIME_PM_OPS(imx675_power_off, imx675_power_on, NULL)
};

static const struct v4l2_subdev_core_ops imx675_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx675_video_ops = {
	.s_stream = imx675_set_stream,
};

static const struct v4l2_subdev_pad_ops imx675_pad_ops = {
	.enum_mbus_code = imx675_enum_mbus_code,
	.enum_frame_size = imx675_enum_frame_size,
	.get_fmt = imx675_get_fmt,
	.set_fmt = imx675_set_fmt,
	.get_selection = imx675_get_selection,
};

static const struct v4l2_subdev_internal_ops ov9282_internal_ops = {
	.init_state = imx675_init_state,
};

static const struct v4l2_subdev_ops imx675_subdev_ops = {
	.core = &imx675_core_ops,
	.video = &imx675_video_ops,
	.pad = &imx675_pad_ops,
};

static const struct media_entity_operations imx675_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/*
 * Returns 0 if all link frequencies used by the driver for the given number
 * of MIPI data lanes are mentioned in the device tree, or the value of the
 * first missing frequency otherwise.
 */
static s64 imx675_check_link_freqs(const struct imx675 *imx675,
				   const struct v4l2_fwnode_endpoint *ep)
{
	int i, j;
	const s64 *freqs = imx675_link_freqs_ptr(imx675);
	int freqs_count = imx675_link_freqs_num(imx675);

	for (i = 0; i < freqs_count; i++) {
		for (j = 0; j < ep->nr_of_link_frequencies; j++)
			if (freqs[i] == ep->link_frequencies[j])
				break;
		if (j == ep->nr_of_link_frequencies)
			return freqs[i];
	}
	return 0;
}

static const struct of_device_id imx675_of_match[] = {
	{ .compatible = "sony,imx675", .data = imx675_colour_formats },
	{ .compatible = "sony,imx675-mono", .data = imx675_mono_formats },
	{ /* sentinel */ }
};

static int imx675_probe(struct i2c_client *client)
{
	struct v4l2_fwnode_device_properties props;
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	/* Only CSI2 is supported for now: */
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	const struct of_device_id *match;
	const struct imx675_mode *mode;
	struct v4l2_ctrl *ctrl;
	struct imx675 *imx675;
	u32 xclk_freq;
	s64 fq;
	int ret;

	imx675 = devm_kzalloc(dev, sizeof(*imx675), GFP_KERNEL);
	if (!imx675)
		return -ENOMEM;

	imx675->dev = dev;
	imx675->regmap = devm_regmap_init_i2c(client, &imx675_regmap_config);
	if (IS_ERR(imx675->regmap)) {
		dev_err(dev, "Unable to initialize I2C\n");
		return -ENODEV;
	}

	match = of_match_device(imx675_of_match, dev);
	if (!match)
		return -ENODEV;
	imx675->formats = (const struct imx675_pixfmt *)match->data;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "Endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret == -ENXIO) {
		dev_err(dev, "Unsupported bus type, should be CSI2\n");
		goto free_err;
	} else if (ret) {
		dev_err(dev, "Parsing endpoint node failed\n");
		goto free_err;
	}

	/* Get number of data lanes */
	imx675->nlanes = ep.bus.mipi_csi2.num_data_lanes;
	if (imx675->nlanes != 2 && imx675->nlanes != 4) {
		dev_err(dev, "Invalid data lanes: %d\n", imx675->nlanes);
		ret = -EINVAL;
		goto free_err;
	}

	dev_dbg(dev, "Using %u data lanes\n", imx675->nlanes);

	if (!ep.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		ret = -EINVAL;
		goto free_err;
	}

	/* Check that link frequences for all the modes are in device tree */
	fq = imx675_check_link_freqs(imx675, &ep);
	if (fq) {
		dev_err(dev, "Link frequency of %lld is not supported\n", fq);
		ret = -EINVAL;
		goto free_err;
	}

	/* get system clock (xclk) */
	imx675->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(imx675->xclk)) {
		dev_err(dev, "Could not get xclk");
		ret = PTR_ERR(imx675->xclk);
		goto free_err;
	}

	ret = fwnode_property_read_u32(dev_fwnode(dev), "clock-frequency",
				       &xclk_freq);
	if (ret) {
		dev_err(dev, "Could not get xclk frequency\n");
		goto free_err;
	}

	/* external clock can be one of a range of values - validate it */
	switch (xclk_freq) {
	case 74250000:
		imx675->inck_sel = IMX675_INCK_SEL_74_25;
		break;
	case 37125000:
		imx675->inck_sel = IMX675_INCK_SEL_37_125;
		break;
	case 72000000:
		imx675->inck_sel = IMX675_INCK_SEL_72;
		break;
	case 27000000:
		imx675->inck_sel = IMX675_INCK_SEL_27;
		break;
	case 24000000:
		imx675->inck_sel = IMX675_INCK_SEL_24;
		break;
	default:
		dev_err(dev, "External clock frequency %u is not supported\n",
			xclk_freq);
		ret = -EINVAL;
		goto free_err;
	}

	ret = clk_set_rate(imx675->xclk, xclk_freq);
	if (ret) {
		dev_err(dev, "Could not set xclk frequency\n");
		goto free_err;
	}

	ret = imx675_get_regulators(dev, imx675);
	if (ret < 0) {
		dev_err(dev, "Cannot get regulators\n");
		goto free_err;
	}

	imx675->rst_gpio = devm_gpiod_get_optional(dev, "reset",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(imx675->rst_gpio)) {
		dev_err(dev, "Cannot get reset gpio\n");
		ret = PTR_ERR(imx675->rst_gpio);
		goto free_err;
	}

	mutex_init(&imx675->lock);
pr_err("A\n");
	/*
	 * Initialize the frame format. In particular, imx675->current_mode
	 * and imx675->bpp are set to defaults: imx675_calc_pixel_rate() call
	 * below relies on these fields.
	 */
	imx675_init_state(&imx675->sd, NULL);

	v4l2_ctrl_handler_init(&imx675->ctrls, 11);

	v4l2_ctrl_new_std(&imx675->ctrls, &imx675_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 0, 100, 1, 0);

	mode = imx675->current_mode;
	imx675->hblank = v4l2_ctrl_new_std(&imx675->ctrls, &imx675_ctrl_ops,
					   V4L2_CID_HBLANK,
					   mode->hmax - mode->width,
					   IMX675_HMAX_MAX - mode->width, 1,
					   mode->hmax - mode->width);
pr_err("B-1 %d. hmax %u, width %u. Range %u - %u, def %u\n", imx675->ctrls.error,
		mode->hmax, mode->width,
					   mode->hmax - mode->width,
					   IMX675_HMAX_MAX - mode->width,
					   mode->hmax - mode->width
	);

	imx675->vblank = v4l2_ctrl_new_std(&imx675->ctrls, &imx675_ctrl_ops,
					   V4L2_CID_VBLANK,
					   mode->vmax - mode->height,
					   IMX675_VMAX_MAX - mode->height, 1,
					   mode->vmax - mode->height);
pr_err("B %d\n", imx675->ctrls.error);

	imx675->exposure = v4l2_ctrl_new_std(&imx675->ctrls, &imx675_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX675_EXPOSURE_MIN,
					     mode->vmax - 2,
					     IMX675_EXPOSURE_STEP,
					     mode->vmax - 2);
pr_err("B1 %d\n", imx675->ctrls.error);

	imx675->hflip = v4l2_ctrl_new_std(&imx675->ctrls, &imx675_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
pr_err("B2 %d\n", imx675->ctrls.error);
	imx675->vflip = v4l2_ctrl_new_std(&imx675->ctrls, &imx675_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
pr_err("B3 %d\n", imx675->ctrls.error);

	ctrl = v4l2_ctrl_new_int_menu(&imx675->ctrls, &imx675_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      imx675_link_freqs_num(imx675) - 1, 0,
				      imx675_link_freqs_ptr(imx675));
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
pr_err("B4 %d\n", imx675->ctrls.error);

	imx675->pixel_rate = v4l2_ctrl_new_std(&imx675->ctrls, &imx675_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       1, INT_MAX, 1,
					       imx675_calc_pixel_rate(imx675));

pr_err("C\n");
	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto free_ctrl;
pr_err("C1\n");

	ret = v4l2_ctrl_new_fwnode_properties(&imx675->ctrls, &imx675_ctrl_ops,
					      &props);
pr_err("C2\n");
	if (ret)
		goto free_ctrl;

	imx675->sd.ctrl_handler = &imx675->ctrls;
pr_err("C3\n");

	if (imx675->ctrls.error) {
		dev_err(dev, "Control initialization error %d\n",
			imx675->ctrls.error);
		ret = imx675->ctrls.error;
		goto free_ctrl;
	}
pr_err("D\n");

	v4l2_i2c_subdev_init(&imx675->sd, client, &imx675_subdev_ops);
	imx675->sd.internal_ops = &ov9282_internal_ops;

	imx675->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		V4L2_SUBDEV_FL_HAS_EVENTS;
	imx675->sd.dev = &client->dev;
	imx675->sd.entity.ops = &imx675_subdev_entity_ops;
	imx675->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	imx675->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx675->sd.entity, 1, &imx675->pad);
	if (ret < 0) {
		dev_err(dev, "Could not register media entity\n");
		goto free_ctrl;
	}
pr_err("E\n");

	/* Initialize the frame format (this also sets imx675->current_mode) */
	imx675_init_state(&imx675->sd, NULL);

	ret = v4l2_async_register_subdev(&imx675->sd);
	if (ret < 0) {
		dev_err(dev, "Could not register v4l2 device\n");
		goto free_entity;
	}
pr_err("F\n");

	/* Power on the device to match runtime PM state below */
	ret = imx675_power_on(dev);
	if (ret < 0) {
		dev_err(dev, "Could not power on the device\n");
		goto free_entity;
	}
pr_err("G\n");

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
pr_err("H\n");

	v4l2_fwnode_endpoint_free(&ep);

	return 0;

free_entity:
	media_entity_cleanup(&imx675->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&imx675->ctrls);
	mutex_destroy(&imx675->lock);
free_err:
	v4l2_fwnode_endpoint_free(&ep);

	return ret;
}

static void imx675_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx675 *imx675 = to_imx675(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	mutex_destroy(&imx675->lock);

	pm_runtime_disable(imx675->dev);
	if (!pm_runtime_status_suspended(imx675->dev))
		imx675_power_off(imx675->dev);
	pm_runtime_set_suspended(imx675->dev);
}

MODULE_DEVICE_TABLE(of, imx675_of_match);

static struct i2c_driver imx675_i2c_driver = {
	.probe  = imx675_probe,
	.remove = imx675_remove,
	.driver = {
		.name  = "imx675",
		.pm = &imx675_pm_ops,
		.of_match_table = of_match_ptr(imx675_of_match),
	},
};

module_i2c_driver(imx675_i2c_driver);

MODULE_DESCRIPTION("Sony IMX675 CMOS Image Sensor Driver");
MODULE_AUTHOR("Soho Enterprise Ltd.");
MODULE_AUTHOR("Tetsuya Nomura <tetsuya.nomura@soho-enterprise.com>");
MODULE_LICENSE("GPL v2");
