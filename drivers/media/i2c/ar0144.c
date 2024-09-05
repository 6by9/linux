// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Ideas on Board Oy
 * Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */

/*
 * Possible improvements:
 *
 * - Use grouped parameter hold to update controls atomically
 * - Expose debug information through debugfs (FRAME_COUNT, FRAME_STATUS, ...)
 * - Make MIPI D-PHY timings configurable
 * - Program additional parallel interface parameters (slew rates, polarities,
 *   ...)
 * - Expose additional controls (in particular the temperature sensor and the
 *   on-chip black level correction)
 * - Make binning configurable
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/v4l2-mediabus.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "ccs-pll.h"

#define AR0144_CHIP_VERSION_REG					CCI_REG16(0x3000)
#define		AR0144_CHIP_VERSION_CS					0x0356
#define		AR0144_CHIP_VERSION_AT					0x1356
#define AR0144_Y_ADDR_START					CCI_REG16(0x3002)
#define AR0144_X_ADDR_START					CCI_REG16(0x3004)
#define AR0144_Y_ADDR_END					CCI_REG16(0x3006)
#define AR0144_X_ADDR_END					CCI_REG16(0x3008)
#define AR0144_FRAME_LENGTH_LINES				CCI_REG16(0x300a)
#define		AR0144_MIN_FRAME_LENGTH_LINES				29
#define		AR0144_MAX_FRAME_LENGTH_LINES				65535
#define AR0144_LINE_LENGTH_PCK					CCI_REG16(0x300c)
#define		AR0144_MIN_LINE_LENGTH_PCK				1488
#define		AR0144_MAX_LINE_LENGTH_PCK				65534
#define AR0144_REVISION_NUMBER					CCI_REG16(0x300e)
#define		AR0144_REVISION_NUMBER_CREV(n)				(((n) >> 12) & 0xf)
#define		AR0144_REVISION_NUMBER_SILICON(n)			(((n) >> 4) & 0xf)
#define		AR0144_REVISION_NUMBER_OTPM(n)				(((n) >> 0) & 0xf)
#define AR0144_LOCK_CONTROL					CCI_REG16(0x3010)
#define		AR0144_LOCK_CONTROL_VALUE				0xbeef
#define AR0144_COARSE_INTEGRATION_TIME				CCI_REG16(0x3012)
#define AR0144_FINE_INTEGRATION_TIME				CCI_REG16(0x3014)
#define AR0144_COARSE_INTEGRATION_TIME_CB			CCI_REG16(0x3016)
#define AR0144_FINE_INTEGRATION_TIME_CB				CCI_REG16(0x3018)
#define AR0144_RESET_REGISTER					CCI_REG16(0x301a)
#define		AR0144_GROUPED_PARAMETER_HOLD				BIT(15)
#define		AR0144_SMIA_SERIALIZER_DIS				BIT(12)
#define		AR0144_FORCED_PLL_ON					BIT(11)
#define		AR0144_RESTART_BAD					BIT(10)
#define		AR0144_MASK_BAD						BIT(9)
#define		AR0144_GPI_EN						BIT(8)
#define		AR0144_PARALLEL_EN					BIT(7)
#define		AR0144_DRIVE_PINS					BIT(6)
#define		AR0144_LOCK_REG						BIT(3)
#define		AR0144_STREAM						BIT(2)
#define		AR0144_RESTART						BIT(1)
#define		AR0144_RESET						BIT(0)
#define AR0144_MODE_SELECT					CCI_REG8(0x301c)
#define		AR0144_MODE_STREAM					BIT(0)
#define AR0144_IMAGE_ORIENTATION				CCI_REG8(0x301d)
#define		AR0144_ORIENTATION_VERT_FLIP				BIT(1)
#define		AR0144_ORIENTATION_HORIZ_MIRROR				BIT(0)
#define AR0144_DATA_PEDESTAL					CCI_REG16(0x301e)
#define AR0144_SOFTWARE_RESET					CCI_REG8(0x3021)
#define AR0144_GROUPED_PARAMETER_HOLD_				CCI_REG8(0x3022)
#define AR0144_MASK_CORRUPTED_FRAMES				CCI_REG8(0x3023)
#define AR0144_PIXEL_ORDER					CCI_REG8(0x3024)
#define AR0144_GPI_STATUS					CCI_REG16(0x3026)
#define		AR0144_TRIGGER						BIT(2)
#define		AR0144_OUTPUT_ENABLE_N					BIT(1)
#define		AR0144_SADDR						BIT(0)
#define AR0144_ROW_SPEED					CCI_REG16(0x3028)
#define		AR0144_ROW_SPEED_HALF_CLK				BIT(4)
#define AR0144_VT_PIX_CLK_DIV					CCI_REG16(0x302a)
#define AR0144_VT_SYS_CLK_DIV					CCI_REG16(0x302c)
#define AR0144_PRE_PLL_CLK_DIV					CCI_REG16(0x302e)
#define AR0144_PLL_MULTIPLIER					CCI_REG16(0x3030)
#define AR0144_CTX_CONTROL_REG					CCI_REG16(0x3034)
#define AR0144_OP_PIX_CLK_DIV					CCI_REG16(0x3036)
#define AR0144_OP_SYS_CLK_DIV					CCI_REG16(0x3038)
#define AR0144_FRAME_COUNT					CCI_REG16(0x303a)
#define AR0144_FRAME_STATUS					CCI_REG16(0x303c)
#define		AR0144_PLL_LOCKED					BIT(3)
#define		AR0144_FRAME_START_DURING_GPH				BIT(2)
#define		AR0144_STANDBY_STATUS					BIT(1)
#define		AR0144_FRAMESYNC					BIT(0)
#define AR0144_LINE_LENGTH_PCK_CB				CCI_REG16(0x303e)
#define AR0144_READ_MODE					CCI_REG16(0x3040)
#define		AR0144_VERT_FLIP					BIT(15)
#define		AR0144_HORIZ_MIRROR					BIT(14)
#define		AR0144_READ_MODE_COL_BIN				BIT(13)
#define		AR0144_READ_MODE_ROW_BIN				BIT(12)
#define		AR0144_READ_MODE_COL_BIN_CB				BIT(11)
#define		AR0144_READ_MODE_ROW_BIN_CB				BIT(10)
#define		AR0144_READ_MODE_COL_SF_BIN_EN				BIT(9)
#define		AR0144_READ_MODE_COL_SF_BIN_EN_CB			BIT(8)
#define		AR0144_READ_MODE_COL_SF_BIN_MONO_EN			BIT(7)
#define		AR0144_READ_MODE_COL_SF_BIN_MONO_EN_CB			BIT(6)
#define		AR0144_READ_MODE_COL_SUM				BIT(5)
#define AR0144_EXTRA_DELAY					CCI_REG16(0x3042)
#define AR0144_GREEN1_GAIN					CCI_REG16(0x3056)
#define AR0144_BLUE_GAIN					CCI_REG16(0x3058)
#define AR0144_RED_GAIN						CCI_REG16(0x305a)
#define AR0144_GREEN2_GAIN					CCI_REG16(0x305c)
#define AR0144_GLOBAL_GAIN					CCI_REG16(0x305e)
#define AR0144_ANALOG_GAIN					CCI_REG16(0x3060)
#define		AR0144_COARSE_GAIN_CB(n)				((n) << 12)
#define		AR0144_FINE_GAIN_CB(n)					((n) << 8)
#define		AR0144_COARSE_GAIN(n)					((n) << 4)
#define		AR0144_FINE_GAIN(n)					((n) << 0)
#define AR0144_SMIA_TEST					CCI_REG16(0x3064)
#define		AR0144_EMBEDDED_DATA					BIT(8)
#define		AR0144_STATS_EN						BIT(7)
#define AR0144_CTX_WR_DATA_REG					CCI_REG16(0x3066)
#define AR0144_CTX_RD_DATA_REG					CCI_REG16(0x3068)
#define AR0144_DATAPATH_SELECT					CCI_REG16(0x306e)
#define		AR0144_SLEW_RATE_CTRL_PARALLEL(n)			(((n) & 7) << 13)
#define		AR0144_SLEW_RATE_CTRL_PARALLEL_MASK			GENMASK(15, 13)
#define		AR0144_SLEW_RATE_CTRL_PIXCLK(n)				(((n) & 7) << 10)
#define		AR0144_SLOW_RATE_CTRL_PIXCLK_MASK			GENMASK(12, 10)
#define		AR0144_SPECIAL_LINE_VALID_NORMAL			(0U << 0)
#define		AR0144_SPECIAL_LINE_VALID_CONTINUOUS			(1U << 0)
#define		AR0144_SPECIAL_LINE_VALID_CONTINUOUS_XOR		(2U << 0)
#define AR0144_TEST_PATTERN_MODE				CCI_REG16(0x3070)
#define		AR0144_TEST_PATTERN_NONE				0
#define		AR0144_TEST_PATTERN_SOLID				1
#define		AR0144_TEST_PATTERN_BARS				2
#define		AR0144_TEST_PATTERN_BARS_FADE				3
#define		AR0144_TEST_PATTERN_WALKING_1S				256
#define AR0144_TEST_DATA_RED					CCI_REG16(0x3072)
#define AR0144_TEST_DATA_GREENR					CCI_REG16(0x3074)
#define AR0144_TEST_DATA_BLUE					CCI_REG16(0x3076)
#define AR0144_TEST_DATA_GREENB					CCI_REG16(0x3078)
#define AR0144_OPERATION_MODE_CTRL				CCI_REG16(0x3082)
#define AR0144_OPERATION_MODE_CTRL_CB				CCI_REG16(0x3084)
#define AR0144_SEQ_DATA_PORT					CCI_REG16(0x3086)
#define AR0144_SEQ_CTRL_PORT					CCI_REG16(0x3088)
#define		AR0144_SEQUENCER_STOPPED				BIT(15)
#define		AR0144_AUTO_INC_ON_READ					BIT(14)
#define		AR0144_ACCESS_ADDRESS(n)				((n) & 0x3ff)
#define AR0144_X_ADDR_START_CB					CCI_REG16(0x308a)
#define AR0144_Y_ADDR_START_CB					CCI_REG16(0x308c)
#define AR0144_X_ADDR_END_CB					CCI_REG16(0x308e)
#define AR0144_Y_ADDR_END_CB					CCI_REG16(0x3090)
#define AR0144_X_EVEN_INC					CCI_REG16(0x30a0)
#define AR0144_X_ODD_INC					CCI_REG16(0x30a2)
#define		AR0144_X_ODD_INC_SKIP(n)				(((n) << 1) - 1)
#define AR0144_Y_EVEN_INC					CCI_REG16(0x30a4)
#define AR0144_Y_ODD_INC					CCI_REG16(0x30a6)
#define		AR0144_Y_ODD_INC_SKIP(n)				(((n) << 1) - 1)
#define AR0144_Y_ODD_INC_CB					CCI_REG16(0x30a8)
#define AR0144_FRAME_LINE_LENGTH_CB				CCI_REG16(0x30aa)
#define AR0144_X_ODD_INC_CB					CCI_REG16(0x30ae)
#define AR0144_DIGITAL_TEST					CCI_REG16(0x30b0)
#define		AR0144_PLL_COMPLETE_BYPASS				BIT(14)
#define		AR0144_PIXCLK_ON					BIT(8)
#define		AR0144_MONO_CHROME_OPERATION				BIT(7)
#define AR0144_TEMPSENS_DATA_REG				CCI_REG16(0x30b2)
#define AR0144_TEMPSENS_CTRL_REG				CCI_REG16(0x30b4)
#define		AR0144_RETRIGGER_THRESHOLD(n)				((n) << 6)
#define		AR0144_RETRIGGER_THRESHOLD_MASK				GENMASK(15, 6)
#define		AR0144_TEMP_CLEAR_VALUE					BIT(5)
#define		AR0144_TEMP_START_CONVERSION				BIT(4)
#define		AR0144_TEMPSENS_POWER_ON				BIT(0)
#define AR0144_GREEN1_GAIN_CB					CCI_REG16(0x30bc)
#define AR0144_BLUE_GAIN_CB					CCI_REG16(0x30be)
#define AR0144_RED_GAIN_CB					CCI_REG16(0x30c0)
#define AR0144_GREEN2_GAIN_CB					CCI_REG16(0x30c2)
#define AR0144_GLOBAL_GAIN_CB					CCI_REG16(0x30c4)
#define AR0144_TEMPSENS_CALIB1					CCI_REG16(0x30c6)
#define AR0144_TEMPSENS_CALIB2					CCI_REG16(0x30c8)
#define AR0144_GRR_CONTROL1					CCI_REG16(0x30ce)
#define AR0144_NOISE_PEDESTAL					CCI_REG16(0x30fe)
#define AR0144_AECTRLREG					CCI_REG16(0x3100)
#define		AR0144_MIN_ANA_GAIN(n)					((n) << 5)
#define		AR0144_MIN_ANA_GAIN_MASK				GENMASK(6, 5)
#define		AR0144_AUTO_DG_EN					BIT(4)
#define		AR0144_AUTO_AG_EN					BIT(1)
#define		AR0144_AE_ENABLE					BIT(0)
#define AR0144_AE_LUMA_TARGET_REG				CCI_REG16(0x3102)
#define AR0144_AE_MIN_EV_STEP_REG				CCI_REG16(0x3108)
#define AR0144_AE_MAX_EV_STEP_REG				CCI_REG16(0x310a)
#define AR0144_AE_DAMP_OFFSET_REG				CCI_REG16(0x310c)
#define AR0144_AE_DAMP_GAIN_REG					CCI_REG16(0x310e)
#define AR0144_AE_DAMP_MAX_REG					CCI_REG16(0x3110)
#define AR0144_AE_MAX_EXPOSURE_REG				CCI_REG16(0x311c)
#define AR0144_AE_MIN_EXPOSURE_REG				CCI_REG16(0x311e)
#define AR0144_AE_COARSE_INTEGRATION_TIME			CCI_REG16(0x3164)
#define AR0144_DELTA_DK_CONTROL					CCI_REG16(0x3180)
#define		AR0144_DELTA_DK_SUB_EN					BIT(15)
#define		AR0144_DELTA_DK_EVERY_FRAME				BIT(14)
#define		AR0144_DELTA_DK_RECALC					BIT(13)
#define		AR0144_DELTA_DK_GRADIENT_REMOVAL			BIT(10)
#define AR0144_DATA_FORMAT_BITS					CCI_REG16(0x31ac)
#define		AR0144_DATA_FORMAT_IN(n)				((n) << 8)
#define		AR0144_DATA_FORMAT_OUT(n)				((n) << 0)
#define AR0144_SERIAL_FORMAT					CCI_REG16(0x31ae)
#define		AR0144_NUM_LANES(n)					(n)
#define		AR0144_NUM_LANES_MASK					GENMASK(1, 0)
#define AR0144_FRAME_PREAMBLE					CCI_REG16(0x31b0)
#define AR0144_LINE_PREAMBLE					CCI_REG16(0x31b2)
#define AR0144_MIPI_TIMING_0					CCI_REG16(0x31b4)
#define		AR0144_T_HS_PREPARE(n)					((n) << 12)
#define		AR0144_T_HS_ZERO(n)					((n) << 8)
#define		AR0144_T_HS_TRAIL(n)					((n) << 4)
#define		AR0144_T_CLK_TRAIL(n)					((n) << 0)
#define AR0144_MIPI_TIMING_1					CCI_REG16(0x31b6)
#define		AR0144_T_CLK_PREPARE(n)					((n) << 12)
#define		AR0144_T_HS_EXIT(n)					((n) << 6)
#define		AR0144_T_CLK_ZERO(n)					((n) << 0)
#define AR0144_MIPI_TIMING_2					CCI_REG16(0x31b8)
#define		AR0144_T_BGAP(n)					((n) << 12)
#define		AR0144_T_CLK_PRE(n)					((n) << 6)
#define		AR0144_T_CLK_POST(n)					((n) << 0)
#define AR0144_MIPI_TIMING_3					CCI_REG16(0x31ba)
#define		AR0144_T_LPX(n)						((n) << 7)
#define		AR0144_T_WAKE_UP(n)					((n) << 0)
#define AR0144_MIPI_TIMING_4					CCI_REG16(0x31bc)
#define		AR0144_CONT_TX_CLK					BIT(15)
#define		AR0144_HEAVY_LP_LOAD					BIT(14)
#define		AR0144_T_INIT(n)					((n) << 0)
#define AR0144_SERIAL_CONFIG_STATUS				CCI_REG16(0x31be)
#define AR0144_SERIAL_CONTROL_STATUS				CCI_REG16(0x31c6)
#define		AR0144_FRAMER_TEST_MODE					BIT(7)
#define AR0144_SERIAL_CRC_0					CCI_REG16(0x31c8)
#define AR0144_COMPANDING					CCI_REG16(0x31d0)
#define		AR0144_COMPAND_EN					BIT(0)
#define AR0144_STAT_FRAME_ID					CCI_REG16(0x31d2)
#define AR0144_I2C_WRT_CHEKCSUM					CCI_REG16(0x31d6)
#define AR0144_SERIAL_TEST					CCI_REG16(0x31d8)
#define		AR0144_TEST_LANE_EN(n)					((n) << 8)
#define		AR0144_TEST_LANE_EN_MASK				GENMASK(11, 8)
#define		AR0144_TEST_MODE_LP00					(0 << 4)
#define		AR0144_TEST_MODE_LP11					(1 << 4)
#define		AR0144_TEST_MODE_HS0					(2 << 4)
#define		AR0144_TEST_MODE_HS1					(3 << 4)
#define		AR0144_TEST_MODE_SQUARE_HALF				(4 << 4)
#define		AR0144_TEST_MODE_SQUARE_FULL				(5 << 4)
#define		AR0144_TEST_MODE_SQUARE_LP				(6 << 4)
#define		AR0144_TEST_MODE_PRBS31					(7 << 4)
#define		AR0144_TEST_MODE_PRBS9					(8 << 4)
#define		AR0144_TEST_MODE_MASK					GENMASK(7, 0)
#define AR0144_PIX_DEF_1D					CCI_REG16(0x31e0)
#define		AR0144_PIX_DEF_1D_DDC_EN				BIT(3)
#define		AR0144_PIX_DEF_CORRECTION_MODE				BIT(1)
#define		AR0144_PIX_DEF_ENABLE					BIT(0)
#define AR0144_HORIZONTAL_CURSOR_POSITION			CCI_REG16(0x31e8)
#define AR0144_VERTICAL_CURSOR_POSITION				CCI_REG16(0x31ea)
#define AR0144_HORIZONTAL_CURSOR_WIDTH				CCI_REG16(0x31ec)
#define AR0144_VERTICAL_CURSOR_WIDTH				CCI_REG16(0x31ee)
#define AR0144_CCI_IDS						CCI_REG16(0x31fc)
#define AR0144_CUSTOMER_REV					CCI_REG16(0x31fe)
#define		AR0144_CUSTOMER_REV_CRA(n)				(((n) & GENMASK(9, 7)) >> 7)
#define		AR0144_CUSTOMER_REV_CFA(n)				(((n) & GENMASK(6, 4)) >> 4)
#define		AR0144_CUSTOMER_REV_CFA_COLOR				1
#define		AR0144_CUSTOMER_REV_CFA_MONO				2
#define AR0144_LED_FLASH_CONTROL				CCI_REG16(0x3270)
#define		AR0144_LED_FLASH_EN					BIT(8)
#define		AR0144_LED_DELAY(n)					((n) << 0)
#define		AR0144_LED_DELAY_MASK					GENMASK(7, 0)
#define AR0144_MIPI_TEST_CNTRL					CCI_REG16(0x3338)
#define AR0144_MIPI_COMPRESS_8_DATA_TYPE			CCI_REG16(0x333a)
#define AR0144_MIPI_COMPRESS_7_DATA_TYPE			CCI_REG16(0x333c)
#define AR0144_MIPI_COMPRESS_6_DATA_TYPE			CCI_REG16(0x333e)
#define AR0144_MIPI_JPEG_PN9_DATA_TYPE				CCI_REG16(0x3340)
#define AR0144_MIPI_CNTRL					CCI_REG16(0x3354)
#define		AR0144_CHAN_NUM(n)					((n) << 6)
#define		AR0144_DATA_TYPE_XMIT(n)				((n) << 0)
#define AR0144_MIPI_TEST_PATTERN_CNTRL				CCI_REG16(0x3356)
#define AR0144_MIPI_TEST_PATTERN_STATUS				CCI_REG16(0x3358)
#define AR0144_DIGITAL_CTRL_1					CCI_REG16(0x3786)
#define		AR0144_LSB_ALIGN_PARA_OUT				BIT(5)
#define		AR0144_USE_1FRAME_SYNCED				BIT(4)
#define		AR0144_PLL_TEST_MODE					BIT(3)
#define		AR0144_DRIVE_PIX_CLK					BIT(0)

#define AR0144_MIN_CSI2_LINK_FREQ		150000000U
#define AR0144_MAX_CSI2_LINK_FREQ		384000000U

#define AR0144_MAX_PARALLEL_LINK_FREQ		74250000U

/*
 * The pixel array contains 1300x820 optically transparent pixels, with 6 dummy
 * pixels on each side that can't be read out. The active size is 1288x808.
 */
#define AR0144_PIXEL_ARRAY_WIDTH		1300U
#define AR0144_PIXEL_ARRAY_HEIGHT		820U
#define AR0144_PIXEL_ARRAY_ACTIVE_LEFT		6U
#define AR0144_PIXEL_ARRAY_ACTIVE_TOP		6U
#define AR0144_PIXEL_ARRAY_ACTIVE_WIDTH		1288U
#define AR0144_PIXEL_ARRAY_ACTIVE_HEIGHT	808U

/* Embedded metadata stream height */
#define AR0144_EMBEDDED_DATA_HEIGHT		2U

/*
 * Documentation indicates minimum horizontal and vertical blanking of 208
 * pixels and 27 lines respectively, which matches the default values for the
 * LINE_LENGTH_PCK and FRAME_LENGTH_LINES registers. Out-of-tree drivers are
 * known to successfully use 22 or 25 lines of vertical blanking. This is
 * likely related to the fact that the total number of rows per frame is equal
 * to FRAME_LENGTH_LINES plus a 5 lines overhead, as indicated in at least one
 * place in the documentation.
 */
#define AR0144_MIN_HBLANK			208U
#define AR0144_MIN_VBLANK			27U

#define AR0144_FRAME_LENGTH_OVERHEAD		5U

/* The minimum values are not documented, pick reasonable minimums. */
#define AR0144_MIN_WIDTH			32U
#define AR0144_MIN_HEIGHT			32U
#define AR0144_DEF_WIDTH			1280U
#define AR0144_DEF_HEIGHT			800U

#define AR0144_NUM_SUPPLIES			3

enum ar0144_pad_ids {
	AR0144_PAD_SOURCE = 0,
	AR0144_PAD_IMAGE,
	AR0144_PAD_EDATA,
	AR0144_NUM_PADS,
};

enum ar0144_stream_ids {
	AR0144_STREAM_IMAGE,
	AR0144_STREAM_EDATA,
};

struct ar0144_model {
	bool mono;
};

struct ar0144_format_info {
	u32 colour;
	u32 mono;
	u32 edata;
	u16 bpp;
	u16 dt;
};

static const struct ar0144_model ar0144_model_color = {
	.mono = false,
};

static const struct ar0144_model ar0144_model_mono = {
	.mono = true,
};

/*
 * Keep the entries sorted by descending bit depth. The top entry is used as
 * the default media bus code, as well as the native code for the image pad.
 */
static const struct ar0144_format_info ar0144_formats[] = {
	{
		.colour = MEDIA_BUS_FMT_SGRBG12_1X12,
		.mono = MEDIA_BUS_FMT_Y12_1X12,
		.edata = MEDIA_BUS_FMT_META_12,
		.bpp = 12,
		.dt = MIPI_CSI2_DT_RAW12,
	}, {
		.colour = MEDIA_BUS_FMT_SGRBG10_1X10,
		.mono = MEDIA_BUS_FMT_Y10_1X10,
		.edata = MEDIA_BUS_FMT_META_10,
		.bpp = 10,
		.dt = MIPI_CSI2_DT_RAW10,
	}, {
		.colour = MEDIA_BUS_FMT_SGRBG8_1X8,
		.mono = MEDIA_BUS_FMT_Y8_1X8,
		.edata = MEDIA_BUS_FMT_META_8,
		.bpp = 8,
		.dt = MIPI_CSI2_DT_RAW8,
	},
};

struct ar0144 {
	struct device *dev;

	struct regmap *regmap;
	struct clk *clk;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[AR0144_NUM_SUPPLIES];

	ktime_t off_time;

	struct ccs_pll pll;

	struct v4l2_fwnode_endpoint bus_cfg;
	u64 valid_link_freqs[ARRAY_SIZE(ar0144_formats)];
	u32 valid_formats;

	struct v4l2_subdev sd;
	struct media_pad pads[AR0144_NUM_PADS];

	const struct ar0144_model *model;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *test_data[4];
	struct {
		struct v4l2_ctrl *hflip;
		struct v4l2_ctrl *vflip;
	};
};

static inline struct ar0144 *to_ar0144(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0144, sd);
}

static u32 ar0144_format_code(struct ar0144 *sensor,
			      const struct ar0144_format_info *info)
{
	return sensor->model->mono ? info->mono : info->colour;
}

static const struct ar0144_format_info *
ar0144_format_info(struct ar0144 *sensor, u32 code, bool use_def)
{
	const struct ar0144_format_info *def = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ar0144_formats); ++i) {
		const struct ar0144_format_info *info = &ar0144_formats[i];
		u32 info_code = ar0144_format_code(sensor, info);

		if (!(sensor->valid_formats & BIT(i)))
			continue;

		if (info_code == code)
			return info;

		if (!def)
			def = info;
	}

	return use_def ? def : NULL;
}

/* -----------------------------------------------------------------------------
 * Hardware configuration
 */

static int ar0144_configure_pll(struct ar0144 *sensor)
{
	int ret = 0;

	cci_write(sensor->regmap, AR0144_PRE_PLL_CLK_DIV,
		  sensor->pll.vt_fr.pre_pll_clk_div, &ret);
	cci_write(sensor->regmap, AR0144_PLL_MULTIPLIER,
		  sensor->pll.vt_fr.pll_multiplier, &ret);
	cci_write(sensor->regmap, AR0144_VT_PIX_CLK_DIV,
		  sensor->pll.vt_bk.pix_clk_div, &ret);
	cci_write(sensor->regmap, AR0144_VT_SYS_CLK_DIV,
		  sensor->pll.vt_bk.sys_clk_div, &ret);
	cci_write(sensor->regmap, AR0144_OP_PIX_CLK_DIV,
		  sensor->pll.op_bk.pix_clk_div, &ret);
	cci_write(sensor->regmap, AR0144_OP_SYS_CLK_DIV,
		  sensor->pll.op_bk.sys_clk_div, &ret);

	/* Wait 1ms for the PLL to lock. */
	fsleep(1000);

	return ret;
}

static int ar0144_configure_parallel(struct ar0144 *sensor,
				     const struct ar0144_format_info *info)
{
	return cci_write(sensor->regmap, AR0144_SERIAL_FORMAT, 0x0200, NULL);
}

static int ar0144_configure_mipi(struct ar0144 *sensor,
				 const struct ar0144_format_info *info)
{
	unsigned int num_lanes = sensor->bus_cfg.bus.mipi_csi2.num_data_lanes;
	bool cont_clk = !(sensor->bus_cfg.bus.mipi_csi2.flags &
			  V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK);
	int ret = 0;

	cci_write(sensor->regmap, AR0144_SERIAL_FORMAT, num_lanes | 0x0200,
		  &ret);

	cci_write(sensor->regmap, AR0144_MIPI_CNTRL,
		  AR0144_CHAN_NUM(0) | AR0144_DATA_TYPE_XMIT(info->dt), &ret);

	cci_write(sensor->regmap, AR0144_MIPI_TIMING_0,
		  AR0144_T_HS_PREPARE(2) | AR0144_T_HS_ZERO(6) |
		  AR0144_T_HS_TRAIL(6) | AR0144_T_CLK_TRAIL(5), &ret);
	cci_write(sensor->regmap, AR0144_MIPI_TIMING_1,
		  AR0144_T_CLK_PREPARE(1) | AR0144_T_HS_EXIT(4) |
		  AR0144_T_CLK_ZERO(14), &ret);
	cci_write(sensor->regmap, AR0144_MIPI_TIMING_2,
		  AR0144_T_BGAP(2) | AR0144_T_CLK_PRE(1) | AR0144_T_CLK_POST(7),
		  &ret);
	cci_write(sensor->regmap, AR0144_MIPI_TIMING_3,
		  AR0144_T_LPX(2) | AR0144_T_WAKE_UP(5), &ret);
	cci_write(sensor->regmap, AR0144_MIPI_TIMING_4,
		  AR0144_T_INIT(4) | (cont_clk ? AR0144_CONT_TX_CLK : 0),
		  &ret);

	return ret;
}

static int ar0144_start_streaming(struct ar0144 *sensor,
				  const struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format;
	const struct ar0144_format_info *info;
	const struct v4l2_rect *crop;
	const struct v4l2_rect *compose;
	unsigned int bin_x, bin_y;
	int ret = 0;
	u16 val;

	/*
	 * The link frequency cannot change during streaming. Grab it first, so
	 * that it won't be set by the __v4l2_ctrl_handler_setup() call below.
	 */
	__v4l2_ctrl_grab(sensor->link_freq, true);

	format = v4l2_subdev_state_get_format(state, AR0144_PAD_SOURCE,
					      AR0144_STREAM_IMAGE);
	crop = v4l2_subdev_state_get_crop(state, AR0144_PAD_IMAGE);
	compose = v4l2_subdev_state_get_compose(state, AR0144_PAD_IMAGE);
	info = ar0144_format_info(sensor, format->code, true);

	ret = ar0144_configure_pll(sensor);
	if (ret)
		goto error;

	cci_write(sensor->regmap, AR0144_DIGITAL_TEST,
		  (sensor->model->mono ? AR0144_MONO_CHROME_OPERATION : 0) |
		  0x0038, &ret);

	cci_write(sensor->regmap, AR0144_DATA_FORMAT_BITS,
		  AR0144_DATA_FORMAT_IN(info->bpp) |
		  AR0144_DATA_FORMAT_OUT(info->bpp), &ret);

	/* Analog crop rectangle, binning/skipping. */
	cci_write(sensor->regmap, AR0144_X_ADDR_START, crop->left, &ret);
	cci_write(sensor->regmap, AR0144_Y_ADDR_START, crop->top, &ret);
	cci_write(sensor->regmap, AR0144_X_ADDR_END,
		  crop->left + crop->width - 1, &ret);
	cci_write(sensor->regmap, AR0144_Y_ADDR_END,
		  crop->top + crop->height - 1, &ret);

	bin_x = crop->width / compose->width;
	bin_y = crop->height / compose->height;

	cci_write(sensor->regmap, AR0144_X_ODD_INC, (bin_x << 1) - 1, &ret);
	cci_write(sensor->regmap, AR0144_Y_ODD_INC, (bin_y << 1) - 1, &ret);

	/*
	 * Enable generation of embedded statistics, required for the on-chip
	 * auto-exposure. There is no downside in enabling it unconditionally.
	 */
	cci_write(sensor->regmap, AR0144_SMIA_TEST, AR0144_EMBEDDED_DATA |
		  AR0144_STATS_EN | 0x1802, &ret);

	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret)
		goto error;

	if (sensor->bus_cfg.bus_type == V4L2_MBUS_PARALLEL)
		ret = ar0144_configure_parallel(sensor, info);
	else
		ret = ar0144_configure_mipi(sensor, info);

	/*
	 * We're all set, start streaming. Mask bad frames and keep read-only
	 * registers locked.
	 */
	val = AR0144_MASK_BAD | AR0144_LOCK_REG | AR0144_STREAM | 0x2010;
	if (sensor->bus_cfg.bus_type == V4L2_MBUS_PARALLEL)
		val |= AR0144_SMIA_SERIALIZER_DIS | AR0144_PARALLEL_EN
		    |  AR0144_DRIVE_PINS;
	cci_write(sensor->regmap, AR0144_RESET_REGISTER, val, &ret);

	if (ret)
		goto error;

	return 0;

error:
	__v4l2_ctrl_grab(sensor->link_freq, false);
	return ret;
}

#define ar0144_read_poll_timeout(sensor, addr, val, cond, sleep_us, timeout_us) \
({ \
	int __ret, __err; \
	__ret = read_poll_timeout(cci_read, __err, __err || (cond), sleep_us, \
				  timeout_us, false, (sensor)->regmap, addr, \
				  &(val), NULL); \
	__ret ? : __err; \
})

static int ar0144_stop_streaming(struct ar0144 *sensor)
{
	u16 reset_bits = AR0144_MASK_BAD | AR0144_LOCK_REG | 0x2010;
	int ret;
	u64 val;

	__v4l2_ctrl_grab(sensor->link_freq, false);

	/*
	 * Initiate the transition to standby by clearing the STREAM bit. Don't
	 * clear the bits that affect the output interface yet.
	 */
	if (sensor->bus_cfg.bus_type == V4L2_MBUS_PARALLEL)
		reset_bits |= AR0144_SMIA_SERIALIZER_DIS | AR0144_PARALLEL_EN
			   |  AR0144_DRIVE_PINS;

	ret = cci_write(sensor->regmap, AR0144_RESET_REGISTER, reset_bits, NULL);
	if (ret)
		return ret;

	ret = ar0144_read_poll_timeout(sensor, AR0144_FRAME_STATUS, val,
				       val & AR0144_STANDBY_STATUS, 2000,
				       2000000);
	if (ret)
		dev_warn(sensor->dev, "%s while trying to enter standby (%d)\n",
			 ret == -ETIMEDOUT ? "timeout" : "error", ret);

	/* Standby state reached, disable the output interface. */
	reset_bits |= AR0144_SMIA_SERIALIZER_DIS;
	reset_bits &= ~(AR0144_PARALLEL_EN | AR0144_DRIVE_PINS);

	return cci_write(sensor->regmap, AR0144_RESET_REGISTER, reset_bits, NULL);
}

/* -----------------------------------------------------------------------------
 * PLL
 */

static int ar0144_pll_calculate(struct ar0144 *sensor, struct ccs_pll *pll,
				unsigned int link_freq, unsigned int bpp)
{
	struct ccs_pll_limits limits = {
		.min_ext_clk_freq_hz = 6000000,
		.max_ext_clk_freq_hz = 48000000,

		.vt_fr = {
			.min_pre_pll_clk_div = 1,
			.max_pre_pll_clk_div = 63,
			.min_pll_ip_clk_freq_hz = 1000000,	/* min_pll_op_clk_freq_hz / max_pll_multiplier */
			.max_pll_ip_clk_freq_hz = 24000000,	/* max_pll_op_clk_freq_hz / min_pll_multiplier */
			.min_pll_multiplier = 32,
			.max_pll_multiplier = 384,
			.min_pll_op_clk_freq_hz = 384000000,
			.max_pll_op_clk_freq_hz = 768000000,
		},
		.vt_bk = {
			.min_sys_clk_div = 1,
			.max_sys_clk_div = 16,
			.min_pix_clk_div = 4,
			.max_pix_clk_div = 16,
			.min_pix_clk_freq_hz = 6000000,		/* No documented limit */
			.max_pix_clk_freq_hz = 74250000,
		},
		.op_bk = {
			.min_sys_clk_div = 1,
			.max_sys_clk_div = 16,
			.min_pix_clk_div = 8,
			.max_pix_clk_div = 12,
			.min_pix_clk_freq_hz = 6000000,		/* No documented limit */
			.max_pix_clk_freq_hz = 74250000,
		},

		.min_line_length_pck_bin = 1200 + AR0144_MIN_HBLANK,	/* To be checked */
		.min_line_length_pck = 1200 + AR0144_MIN_HBLANK,
	};
	unsigned int num_lanes = sensor->bus_cfg.bus.mipi_csi2.num_data_lanes;
	int ret;

	/*
	 * The OP pixel clock limits depends on the number of lanes, which the
	 * PLL calculator doesn't take into account despite specifying the
	 * CCS_PLL_FLAG_LANE_SPEED_MODEL flag. Adjust them manually.
	 */
	limits.op_bk.min_pix_clk_freq_hz /= num_lanes;
	limits.op_bk.max_pix_clk_freq_hz /= num_lanes;

	/*
	 * There are no documented constraints on the sys clock frequency, for
	 * either branch. Recover them based on the PLL output clock frequency
	 * and sys_clk_div limits on one hand, and the pix clock frequency and
	 * the pix_clk_div limits on the other hand.
	 */
	limits.vt_bk.min_sys_clk_freq_hz =
		max(limits.vt_fr.min_pll_op_clk_freq_hz / limits.vt_bk.max_sys_clk_div,
		    limits.vt_bk.min_pix_clk_freq_hz * limits.vt_bk.min_pix_clk_div);
	limits.vt_bk.max_sys_clk_freq_hz =
		min(limits.vt_fr.max_pll_op_clk_freq_hz / limits.vt_bk.min_sys_clk_div,
		    limits.vt_bk.max_pix_clk_freq_hz * limits.vt_bk.max_pix_clk_div);

	limits.op_bk.min_sys_clk_freq_hz =
		max(limits.vt_fr.min_pll_op_clk_freq_hz / limits.op_bk.max_sys_clk_div,
		    limits.op_bk.min_pix_clk_freq_hz * limits.op_bk.min_pix_clk_div);
	limits.op_bk.max_sys_clk_freq_hz =
		min(limits.vt_fr.max_pll_op_clk_freq_hz / limits.op_bk.min_sys_clk_div,
		    limits.op_bk.max_pix_clk_freq_hz * limits.op_bk.max_pix_clk_div);

	memset(pll, 0, sizeof(*pll));

	pll->bus_type = CCS_PLL_BUS_TYPE_CSI2_DPHY;
	pll->op_lanes = num_lanes;
	pll->vt_lanes = 1;
	pll->csi2.lanes = num_lanes;
	/*
	 * As the sensor doesn't support FIFO derating, binning doesn't
	 * influence the PLL configuration. Hardcode the binning factors.
	 */
	pll->binning_horizontal = 1;
	pll->binning_vertical = 1;
	pll->scale_m = 1;
	pll->scale_n = 1;
	pll->bits_per_pixel = bpp;
	pll->flags = CCS_PLL_FLAG_LANE_SPEED_MODEL;
	pll->link_freq = link_freq;
	pll->ext_clk_freq_hz = clk_get_rate(sensor->clk);

	ret = ccs_pll_calculate(sensor->dev, &limits, pll);
	if (ret)
		return ret;

	/*
	 * The sensor ignores the LSB of the PLL multiplier. If the multiplier
	 * is an odd value, as a workaround to avoid precision loss, multiply
	 * both the pre-divider and the multiplier by 2 if this doesn't bring
	 * them beyond their maximum values. This doesn't necessarily guarantee
	 * optimum PLL parameters. Ideally the PLL calculator should handle
	 * this constraint.
	 */
	if ((pll->vt_fr.pll_multiplier & 1) &&
	    pll->vt_fr.pll_multiplier * 2 <= limits.vt_fr.max_pll_multiplier &&
	    pll->vt_fr.pre_pll_clk_div * 2 <= limits.vt_fr.max_pre_pll_clk_div) {
		pll->vt_fr.pll_multiplier *= 2;
		pll->vt_fr.pre_pll_clk_div *= 2;
	}

	if (pll->vt_fr.pll_multiplier & 1)
		dev_warn(sensor->dev,
			 "Odd PLL multiplier, link frequency %u will not be exact\n",
			 pll->link_freq);

	return 0;
}

static int ar0144_pll_update(struct ar0144 *sensor,
			     const struct ar0144_format_info *info)
{
	u64 link_freq;
	int ret;

	link_freq = sensor->bus_cfg.link_frequencies[sensor->link_freq->val];
	ret = ar0144_pll_calculate(sensor, &sensor->pll, link_freq, info->bpp);
	if (ret) {
		dev_err(sensor->dev, "PLL calculations failed: %d\n", ret);
		return ret;
	}

	__v4l2_ctrl_s_ctrl_int64(sensor->pixel_rate,
				 sensor->pll.pixel_rate_pixel_array);

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 controls
 */

static const char * const ar0144_test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"Full Color Bars",
	"Fade to Gray Color Bars",
	"Walking 1",
};

static const u32 ar0144_test_pattern_values[] = {
	AR0144_TEST_PATTERN_NONE,
	AR0144_TEST_PATTERN_SOLID,
	AR0144_TEST_PATTERN_BARS,
	AR0144_TEST_PATTERN_BARS_FADE,
	AR0144_TEST_PATTERN_WALKING_1S,
};

static void ar0144_update_link_freqs(struct ar0144 *sensor,
				     const struct ar0144_format_info *info)
{
	u64 valid_link_freqs;
	unsigned int index;
	unsigned int min;
	unsigned int max;

	index = info - ar0144_formats;
	valid_link_freqs = sensor->valid_link_freqs[index];

	min = __ffs(valid_link_freqs);
	max = __fls(valid_link_freqs);

	__v4l2_ctrl_modify_range(sensor->link_freq, min, max, ~valid_link_freqs,
				 min);
}

static int ar0144_update_exposure(struct ar0144 *sensor,
				  const struct v4l2_rect *crop)
{
	unsigned int max;

	/* The coarse integration time is limited to FRAME_LENGTH_LINES - 1. */
	max = crop->height + sensor->vblank->val - 1
	    - AR0144_FRAME_LENGTH_OVERHEAD;
	return __v4l2_ctrl_modify_range(sensor->exposure, 1, max, 1, max);
}

static void ar0144_update_blanking(struct ar0144 *sensor,
				   const struct v4l2_subdev_state *state)
{
	const struct v4l2_rect *crop;
	unsigned int min;
	unsigned int max;

	crop = v4l2_subdev_state_get_crop(state, AR0144_PAD_IMAGE);

	/*
	 * Horizontally, the line length (in pixel clocks), programmed in the
	 * LINE_LENGTH_PCK register, is equal to the sum of the crop width and
	 * the horizontal blanking. The sensor has lower and upper bounds for
	 * the LINE_LENGTH_PCK value, as well as a lower bound for the
	 * horizontal blanking.
	 */
	min = max_t(int, AR0144_MIN_LINE_LENGTH_PCK - crop->width,
		    AR0144_MIN_HBLANK);
	max = AR0144_MAX_LINE_LENGTH_PCK - crop->width;
	__v4l2_ctrl_modify_range(sensor->hblank, min, max, 2, min);

	/*
	 * Vertically, the situation is more complicated. The frame length (in
	 * lines) is equal to the FRAME_LENGTH_LINES register value plus 5
	 * lines of overhead. This needs to be taken into account for the
	 * VBLANK calculation.
	 */
	min = max_t(int, AR0144_MIN_FRAME_LENGTH_LINES - crop->height +
		    AR0144_FRAME_LENGTH_OVERHEAD, AR0144_MIN_VBLANK);
	max = AR0144_MAX_FRAME_LENGTH_LINES - crop->height
	    + AR0144_FRAME_LENGTH_OVERHEAD;
	__v4l2_ctrl_modify_range(sensor->vblank, min, max, 1, min);

	ar0144_update_exposure(sensor, crop);
}

static int ar0144_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0144 *sensor = container_of(ctrl->handler, struct ar0144, ctrls);
	const struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *format;
	const struct ar0144_format_info *info;
	const struct v4l2_rect *crop;
	int ret = 0;

	/*
	 * Return immediately for controls that don't need to be applied to the
	 * device.
	 */
	if (ctrl->flags & (V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_GRABBED))
		return 0;

	/*
	 * First process controls that update other parts of the device
	 * configuration.
	 */
	state = v4l2_subdev_get_locked_active_state(&sensor->sd);
	format = v4l2_subdev_state_get_format(state, AR0144_PAD_SOURCE,
					      AR0144_STREAM_IMAGE);
	info = ar0144_format_info(sensor, format->code, true);
	crop = v4l2_subdev_state_get_crop(state, AR0144_PAD_IMAGE);

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ret = ar0144_update_exposure(sensor, crop);
		break;

	case V4L2_CID_LINK_FREQ:
		ret = ar0144_pll_update(sensor, info);
		break;

	default:
		break;
	}

	if (ret)
		return ret;

	/*
	 * Apply controls to the hardware. If power is down, they controls will
	 * be applied when starting streaming.
	 */
	if (!pm_runtime_get_if_in_use(sensor->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(sensor->regmap, AR0144_ANALOG_GAIN, ctrl->val, &ret);
		break;

	case V4L2_CID_DIGITAL_GAIN:
		cci_write(sensor->regmap, AR0144_GLOBAL_GAIN, ctrl->val, &ret);
		break;

	case V4L2_CID_EXPOSURE:
		cci_write(sensor->regmap, AR0144_COARSE_INTEGRATION_TIME,
			  ctrl->val, &ret);
		break;

	case V4L2_CID_HBLANK:
		cci_write(sensor->regmap, AR0144_LINE_LENGTH_PCK,
			  crop->width + ctrl->val, &ret);
		break;

	case V4L2_CID_VBLANK:
		cci_write(sensor->regmap, AR0144_FRAME_LENGTH_LINES,
			  crop->height + ctrl->val - AR0144_FRAME_LENGTH_OVERHEAD,
			  &ret);
		break;

	case V4L2_CID_TEST_PATTERN:
		cci_write(sensor->regmap, AR0144_TEST_PATTERN_MODE,
			  ar0144_test_pattern_values[ctrl->val], &ret);

		/*
		 * Register 0x3044 is not documented, but mentioned in the test
		 * pattern configuration. Bits [5:4] should be set to 0 to
		 * avoid clipping pixel values to 0xf70.
		 */
		cci_write(sensor->regmap, CCI_REG16(0x3044),
			  ctrl->val ? 0x0400 : 0x0410, &ret);
		break;

	case V4L2_CID_TEST_PATTERN_RED:
	case V4L2_CID_TEST_PATTERN_GREENR:
	case V4L2_CID_TEST_PATTERN_BLUE:
	case V4L2_CID_TEST_PATTERN_GREENB:
		cci_write(sensor->regmap, AR0144_GROUPED_PARAMETER_HOLD_,
			  1, &ret);

		cci_write(sensor->regmap, AR0144_TEST_DATA_RED,
			  sensor->test_data[0]->val, &ret);
		cci_write(sensor->regmap, AR0144_TEST_DATA_GREENR,
			  sensor->test_data[1]->val, &ret);
		cci_write(sensor->regmap, AR0144_TEST_DATA_BLUE,
			  sensor->test_data[2]->val, &ret);
		cci_write(sensor->regmap, AR0144_TEST_DATA_GREENB,
			  sensor->test_data[3]->val, &ret);

		cci_write(sensor->regmap, AR0144_GROUPED_PARAMETER_HOLD_,
			  0, &ret);
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	{
		u16 reg = 0;

		if (sensor->hflip->val)
			reg |= AR0144_ORIENTATION_HORIZ_MIRROR;
		if (sensor->vflip->val)
			reg |= AR0144_ORIENTATION_VERT_FLIP;

		cci_write(sensor->regmap, AR0144_IMAGE_ORIENTATION, reg, &ret);
		break;
	}

	case V4L2_CID_EXPOSURE_AUTO:
		cci_write(sensor->regmap, AR0144_AECTRLREG,
			  ctrl->val == V4L2_EXPOSURE_AUTO ?
			  AR0144_AUTO_AG_EN | AR0144_AE_ENABLE : 0, &ret);
		break;

	case V4L2_CID_COMPANDING:
		cci_write(sensor->regmap, AR0144_COMPANDING,
			  ctrl->val == V4L2_COMPANDING_COMPRESS ?
			  AR0144_COMPAND_EN : 0, &ret);
		break;

	case V4L2_CID_LINK_FREQ:
		break;

	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(sensor->dev);
	pm_runtime_put_autosuspend(sensor->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar0144_ctrl_ops = {
	.s_ctrl = ar0144_s_ctrl,
};

static int ar0144_init_ctrls(struct ar0144 *sensor)
{
	static const u32 test_pattern_ctrls[] = {
		V4L2_CID_TEST_PATTERN_RED,
		V4L2_CID_TEST_PATTERN_GREENR,
		V4L2_CID_TEST_PATTERN_BLUE,
		V4L2_CID_TEST_PATTERN_GREENB,
	};
	struct v4l2_fwnode_device_properties props;
	unsigned int i;
	int ret;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 18);

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &ar0144_ctrl_ops,
					&props);

	/*
	 * Set the link frequency, pixel rate, horizontal blanking and vertical
	 * blanking and exposure to hardcoded values. They will be updated by
	 * ar0144_update_link_freqs(), ar0144_pll_update() and
	 * ar0144_update_blanking().
	 */
	sensor->pixel_rate =
		v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
				  V4L2_CID_PIXEL_RATE, 1, INT_MAX, 1, 1);
	sensor->link_freq =
		v4l2_ctrl_new_int_menu(&sensor->ctrls, &ar0144_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       sensor->bus_cfg.nr_of_link_frequencies - 1, 0,
				       sensor->bus_cfg.link_frequencies);

	sensor->hblank = v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
					   V4L2_CID_HBLANK, AR0144_MIN_HBLANK,
					   AR0144_MIN_HBLANK, 2, AR0144_MIN_HBLANK);
	sensor->vblank = v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
					   V4L2_CID_VBLANK, AR0144_MIN_VBLANK,
					   AR0144_MIN_VBLANK, 1, AR0144_MIN_VBLANK);

	sensor->exposure = v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
					     V4L2_CID_EXPOSURE, 1,
					     AR0144_MAX_FRAME_LENGTH_LINES - 1,
					     1, 16);

	/*
	 * The sensor analogue gain is split in an exponential coarse gain and
	 * a fine gain. The minimum recommended gain is 1.6842, which maps to a
	 * gain code of 13. Set the minimum to 0 to expose the whole range of
	 * possible values, and the default to the recommended minimum.
	 */
	v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 0, 79, 1, 13);

	v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
			  V4L2_CID_DIGITAL_GAIN, 0, 0x7ff, 1, 0x080);

	v4l2_ctrl_new_std_menu(&sensor->ctrls, &ar0144_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,
			       ~(BIT(V4L2_EXPOSURE_AUTO) | BIT(V4L2_EXPOSURE_MANUAL)),
			       V4L2_EXPOSURE_MANUAL);

	v4l2_ctrl_new_std_menu_items(&sensor->ctrls, &ar0144_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ar0144_test_pattern_menu) - 1,
				     0, 0, ar0144_test_pattern_menu);

	for (i = 0; i < ARRAY_SIZE(test_pattern_ctrls); ++i)
		sensor->test_data[i] =
			v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
					  test_pattern_ctrls[i], 0, 4095, 1, 0);

	sensor->hflip = v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	sensor->vflip = v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu(&sensor->ctrls, &ar0144_ctrl_ops,
			       V4L2_CID_COMPANDING, V4L2_COMPANDING_COMPRESS, 0,
			       V4L2_COMPANDING_LINEAR);

	if (sensor->ctrls.error) {
		ret = sensor->ctrls.error;
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return ret;
	}

	v4l2_ctrl_cluster(4, sensor->test_data);
	v4l2_ctrl_cluster(2, &sensor->hflip);

	sensor->sd.ctrl_handler = &sensor->ctrls;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

static int ar0144_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0144 *sensor = to_ar0144(sd);

	switch (code->pad) {
	case AR0144_PAD_IMAGE: {
		const struct ar0144_format_info *info;

		/* The internal image pad is hardwired to the native format. */
		if (code->index > 0)
			return -EINVAL;

		info = &ar0144_formats[0];
		code->code = ar0144_format_code(sensor, info);
		return 0;
	}

	case AR0144_PAD_EDATA: {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_CCS_EMBEDDED;
		return 0;
	}

	case AR0144_PAD_SOURCE:
	default:
		break;
	}

	/*
	 * On the source pad, the sensor supports multiple image raw formats
	 * with different bit depths. The embedded data format bit depth
	 * follows the image stream.
	 */
	if (code->stream == AR0144_STREAM_IMAGE) {
		unsigned int index = 0;
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(ar0144_formats); ++i) {
			const struct ar0144_format_info *info = &ar0144_formats[i];

			if (!(sensor->valid_formats & BIT(i)))
				continue;

			if (code->index == index) {
				code->code = ar0144_format_code(sensor, info);
				return 0;
			}

			index++;
		}

		return -EINVAL;
	} else {
		struct v4l2_mbus_framefmt *fmt;

		if (code->index > 0)
			return -EINVAL;

		fmt = v4l2_subdev_state_get_format(state, AR0144_PAD_SOURCE,
						   AR0144_STREAM_EDATA);
		code->code = fmt->code;
	}

	return 0;
}

static int ar0144_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar0144 *sensor = to_ar0144(sd);
	const struct ar0144_format_info *info;
	const struct v4l2_mbus_framefmt *fmt;

	if (fse->index > 0)
		return -EINVAL;

	fmt = v4l2_subdev_state_get_format(state, fse->pad, fse->stream);

	/*
	 * Verify the media bus code. On the source image stream multiple
	 * options are supported, while on all other streams the requested code
	 * must match the current format.
	 */
	if (fse->pad == AR0144_PAD_SOURCE &&
	    fse->stream == AR0144_STREAM_IMAGE) {
		info = ar0144_format_info(sensor, fse->code, false);
		if (!info)
			return -EINVAL;
	} else {
		if (fse->code != fmt->code)
			return -EINVAL;
	}

	fse->min_width = fmt->width;
	fse->max_width = fmt->width;
	fse->min_height = fmt->height;
	fse->max_height = fmt->height;

	return 0;
}

static int ar0144_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *format)
{
	struct ar0144 *sensor = to_ar0144(sd);
	const struct ar0144_format_info *info;
	struct v4l2_mbus_framefmt *fmt;

	if (v4l2_subdev_is_streaming(sd) &&
	    format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EBUSY;

	/* The format can only be set for the image stream on the source pad. */
	if (format->pad != AR0144_PAD_SOURCE ||
	    format->stream != AR0144_STREAM_IMAGE)
		return v4l2_subdev_get_fmt(sd, state, format);

	/*
	 * Only the media bus code can be updated on the source pad, dimensions
	 * are set by the compose on the image pad rectangle.
	 */
	fmt = v4l2_subdev_state_get_format(state, AR0144_PAD_SOURCE,
					   AR0144_STREAM_IMAGE);
	info = ar0144_format_info(sensor, format->format.code, true);
	fmt->code = ar0144_format_code(sensor, info);

	format->format = *fmt;

	/* Update the format on the source side of the embedded data stream. */
	fmt = v4l2_subdev_state_get_format(state, AR0144_PAD_SOURCE,
					   AR0144_STREAM_EDATA);
	fmt->code = info->edata;

	if (format->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return 0;

	ar0144_update_link_freqs(sensor, info);
	ar0144_pll_update(sensor, info);

	return 0;
}

static int ar0144_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	/*
	 * The sensor doesn't support output crop, selection rectangles are
	 * supported on the internal image pad only.
	 */
	if (sel->pad != AR0144_PAD_IMAGE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = (AR0144_PIXEL_ARRAY_ACTIVE_WIDTH - AR0144_DEF_WIDTH) / 2;
		sel->r.top = (AR0144_PIXEL_ARRAY_ACTIVE_HEIGHT - AR0144_DEF_HEIGHT) / 2;
		sel->r.width = AR0144_DEF_WIDTH;
		sel->r.height = AR0144_DEF_HEIGHT;
		break;

	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = AR0144_PIXEL_ARRAY_ACTIVE_WIDTH;
		sel->r.height = AR0144_PIXEL_ARRAY_ACTIVE_HEIGHT;
		break;

	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, AR0144_PAD_IMAGE);
		break;

	case V4L2_SEL_TGT_COMPOSE:
		sel->r = *v4l2_subdev_state_get_compose(state, AR0144_PAD_IMAGE);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int ar0144_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *compose;
	struct v4l2_rect *crop;
	unsigned int bin_x, bin_y;

	/*
	 * The sensor doesn't support output crop, selection rectangles are
	 * supported on the internal image pad only.
	 */
	if (sel->pad != AR0144_PAD_IMAGE)
		return -EINVAL;

	if (v4l2_subdev_is_streaming(sd) &&
	    sel->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EBUSY;

	crop = v4l2_subdev_state_get_crop(state, AR0144_PAD_IMAGE);
	compose = v4l2_subdev_state_get_compose(state, AR0144_PAD_IMAGE);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		crop->left = min_t(unsigned int, ALIGN(sel->r.left, 2),
				   AR0144_PIXEL_ARRAY_ACTIVE_WIDTH -
				   AR0144_MIN_WIDTH);
		crop->top = min_t(unsigned int, ALIGN(sel->r.top, 2),
				  AR0144_PIXEL_ARRAY_ACTIVE_HEIGHT -
				  AR0144_MIN_HEIGHT);
		crop->width = clamp(sel->r.width, AR0144_MIN_WIDTH,
				    AR0144_PIXEL_ARRAY_ACTIVE_WIDTH -
				    crop->left);
		crop->height = clamp(sel->r.height, AR0144_MIN_HEIGHT,
				     AR0144_PIXEL_ARRAY_ACTIVE_HEIGHT -
				     crop->top);

		sel->r = *crop;

		compose->width = crop->width;
		compose->height = crop->height;
		break;

	case V4L2_SEL_TGT_COMPOSE:
		/*
		 * The output size results from the binning/skipping applied to
		 * the crop rectangle. The x/y increments must be powers of 2.
		 * Clamp the width/height first, to avoid both divisions by 0
		 * and the undefined behaviour of roundup_pow_of_two(0).
		 */
		compose->width = clamp(sel->r.width, 1U, crop->width);
		compose->height = clamp(sel->r.height, 1U, crop->height);
		bin_x = clamp(roundup_pow_of_two(crop->width / compose->width),
			      1, 16);
		bin_y = clamp(roundup_pow_of_two(crop->height / compose->height),
			      1, 16);
		compose->width = crop->width / bin_x;
		compose->height = crop->height / bin_y;

		sel->r = *compose;
		break;

	default:
		return -EINVAL;
	}

	/* Propagate the compose rectangle to the output format. */
	fmt = v4l2_subdev_state_get_format(state, AR0144_PAD_SOURCE,
					   AR0144_STREAM_IMAGE);
	fmt->width = compose->width;
	fmt->height = compose->height;

	/* Update the embedded data stream width. */
	fmt = v4l2_subdev_state_get_format(state, AR0144_PAD_EDATA);
	fmt->width = compose->width;

	fmt = v4l2_subdev_state_get_format(state, AR0144_PAD_SOURCE,
					   AR0144_STREAM_EDATA);
	fmt->width = compose->width;

	return 0;
}

static int ar0144_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct ar0144 *sensor = to_ar0144(sd);
	const struct ar0144_format_info *info;
	struct v4l2_subdev_state *state;
	u32 img_code;
	u32 ed_code;

	if (pad != AR0144_PAD_SOURCE)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	img_code = v4l2_subdev_state_get_format(state, AR0144_PAD_SOURCE,
						AR0144_STREAM_IMAGE)->code;
	ed_code = v4l2_subdev_state_get_format(state, AR0144_PAD_SOURCE,
					       AR0144_STREAM_EDATA)->code;
	v4l2_subdev_unlock_state(state);

	info = ar0144_format_info(sensor, img_code, true);

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 2;

	fd->entry[0].pixelcode = img_code;
	fd->entry[0].stream = AR0144_STREAM_IMAGE;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = info->dt;

	fd->entry[1].pixelcode = ed_code;
	fd->entry[1].stream = AR0144_STREAM_EDATA;
	fd->entry[1].bus.csi2.vc = 0;
	fd->entry[1].bus.csi2.dt = MIPI_CSI2_DT_EMBEDDED_8B;

	return 0;
}

static int ar0144_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *cfg)
{
	struct ar0144 *sensor = to_ar0144(sd);

	cfg->type = sensor->bus_cfg.bus_type;

	switch (sensor->bus_cfg.bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
	default:
		cfg->bus.mipi_csi2 = sensor->bus_cfg.bus.mipi_csi2;
		break;

	case V4L2_MBUS_PARALLEL:
		cfg->bus.parallel = sensor->bus_cfg.bus.parallel;
		break;
	}

	return 0;
}

static int ar0144_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct ar0144 *sensor = to_ar0144(sd);
	int ret;

	/*
	 * The image stream controls sensor streaming, as embedded data isn't
	 * controllable independently.
	 */
	if (!(streams_mask & BIT(AR0144_STREAM_IMAGE)))
		return 0;

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		return ret;

	ret = ar0144_start_streaming(sensor, state);
	if (ret) {
		dev_err(sensor->dev, "Failed to start streaming: %d\n",
			ret);
		pm_runtime_put_sync(sensor->dev);
		return ret;
	}

	return 0;
}

static int ar0144_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct ar0144 *sensor = to_ar0144(sd);

	/*
	 * The image stream controls sensor streaming, as embedded data isn't
	 * controllable independently.
	 */
	if (!(streams_mask & BIT(AR0144_STREAM_IMAGE)))
		return 0;

	ar0144_stop_streaming(sensor);
	pm_runtime_mark_last_busy(sensor->dev);
	pm_runtime_put_autosuspend(sensor->dev);

	return 0;
}

static int ar0144_entity_init_state(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[] = {
		{
			.sink_pad = AR0144_PAD_IMAGE,
			.sink_stream = 0,
			.source_pad = AR0144_PAD_SOURCE,
			.source_stream = AR0144_STREAM_IMAGE,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE |
				 V4L2_SUBDEV_ROUTE_FL_IMMUTABLE,
		}, {
			.sink_pad = AR0144_PAD_EDATA,
			.sink_stream = 0,
			.source_pad = AR0144_PAD_SOURCE,
			.source_stream = AR0144_STREAM_EDATA,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE |
				 V4L2_SUBDEV_ROUTE_FL_IMMUTABLE,
		},
	};
	struct v4l2_subdev_krouting routing = {
		.len_routes = ARRAY_SIZE(routes),
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};
	const struct ar0144_format_info *info;
	struct ar0144 *sensor = to_ar0144(sd);
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;
	struct v4l2_rect *compose;
	int ret;

	ret = v4l2_subdev_set_routing(sd, state, &routing);
	if (ret)
		return ret;

	/* Image pad. */
	info = &ar0144_formats[0];

	fmt = v4l2_subdev_state_get_format(state, AR0144_PAD_IMAGE);
	fmt->width = AR0144_PIXEL_ARRAY_ACTIVE_WIDTH;
	fmt->height = AR0144_PIXEL_ARRAY_ACTIVE_HEIGHT;
	fmt->code = ar0144_format_code(sensor, info);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;

	crop = v4l2_subdev_state_get_crop(state, AR0144_PAD_IMAGE);
	crop->left = 4;
	crop->top = 4;
	crop->width = AR0144_DEF_WIDTH;
	crop->height = AR0144_DEF_HEIGHT;

	compose = v4l2_subdev_state_get_compose(state, AR0144_PAD_IMAGE);
	compose->left = 0;
	compose->top = 0;
	compose->width = AR0144_DEF_WIDTH;
	compose->height = AR0144_DEF_HEIGHT;

	/* Embedded data pad. */
	fmt = v4l2_subdev_state_get_format(state, AR0144_PAD_EDATA);
	fmt->width = AR0144_DEF_WIDTH;
	fmt->height = AR0144_EMBEDDED_DATA_HEIGHT;
	fmt->code = MEDIA_BUS_FMT_CCS_EMBEDDED;
	fmt->field = V4L2_FIELD_NONE;

	/* Source pad, image stream. */
	info = ar0144_format_info(sensor, 0, true);

	fmt = v4l2_subdev_state_get_format(state, AR0144_PAD_SOURCE,
					   AR0144_STREAM_IMAGE);
	fmt->width = AR0144_DEF_WIDTH;
	fmt->height = AR0144_DEF_HEIGHT;
	fmt->code = ar0144_format_code(sensor, info);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;

	/* Source pad, embedded data stream. */
	fmt = v4l2_subdev_state_get_format(state, AR0144_PAD_SOURCE,
					   AR0144_STREAM_EDATA);
	fmt->width = AR0144_DEF_WIDTH;
	fmt->height = AR0144_EMBEDDED_DATA_HEIGHT;
	fmt->code = info->edata;
	fmt->field = V4L2_FIELD_NONE;

	return 0;
}

static const struct v4l2_subdev_pad_ops ar0144_subdev_pad_ops = {
	.enum_mbus_code = ar0144_enum_mbus_code,
	.enum_frame_size = ar0144_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ar0144_set_fmt,
	.get_selection = ar0144_get_selection,
	.set_selection = ar0144_set_selection,
	.get_frame_desc = ar0144_get_frame_desc,
	.get_mbus_config = ar0144_get_mbus_config,
	.enable_streams = ar0144_enable_streams,
	.disable_streams = ar0144_disable_streams,
};

static const struct v4l2_subdev_ops ar0144_subdev_ops = {
	.pad = &ar0144_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar0144_subdev_internal_ops = {
	.init_state = ar0144_entity_init_state,
};

static const struct media_entity_operations ar0144_entity_ops = {
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
};

static int ar0144_init_subdev(struct ar0144 *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	struct v4l2_subdev *sd = &sensor->sd;
	const struct v4l2_mbus_framefmt *format;
	const struct ar0144_format_info *info;
	struct v4l2_subdev_state *state;
	int ret;

	v4l2_i2c_subdev_init(sd, client, &ar0144_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_STREAMS;
	sd->internal_ops = &ar0144_subdev_internal_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &ar0144_entity_ops;

	sensor->pads[AR0144_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	sensor->pads[AR0144_PAD_IMAGE].flags = MEDIA_PAD_FL_SINK
					     | MEDIA_PAD_FL_INTERNAL;
	sensor->pads[AR0144_PAD_EDATA].flags = MEDIA_PAD_FL_SINK
					     | MEDIA_PAD_FL_INTERNAL;

	ret = media_entity_pads_init(&sd->entity, ARRAY_SIZE(sensor->pads),
				     sensor->pads);
	if (ret)
		return ret;

	ret = ar0144_init_ctrls(sensor);
	if (ret)
		goto err_entity;

	sensor->sd.state_lock = sensor->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto err_ctrls;

	/*
	 * Update the link frequency, PLL configuration (including the pixel
	 * rate) and blanking controls.
	 */
	state = v4l2_subdev_lock_and_get_active_state(sd);
	format = v4l2_subdev_state_get_format(state, AR0144_PAD_SOURCE,
					      AR0144_STREAM_IMAGE);
	info = ar0144_format_info(sensor, format->code, true);

	ar0144_update_link_freqs(sensor, info);
	ar0144_pll_update(sensor, info);
	ar0144_update_blanking(sensor, state);

	v4l2_subdev_unlock_state(state);

	return 0;

err_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls);
err_entity:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static void ar0144_cleanup_subdev(struct ar0144 *sensor)
{
	v4l2_subdev_cleanup(&sensor->sd);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	media_entity_cleanup(&sensor->sd.entity);
}

/* -----------------------------------------------------------------------------
 * Power management
 */

static int ar0144_power_on(struct ar0144 *sensor)
{
	u64 reset_delay;
	long rate;
	int ret;

	/*
	 * The sensor must be powered off for at least 100ms before being
	 * powered on again.
	 */
	if (sensor->off_time) {
		u64 off_duration;

		off_duration = ktime_us_delta(ktime_get_boottime(),
					      sensor->off_time);
		if (off_duration < 100000)
			fsleep(100000 - off_duration);
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies),
				    sensor->supplies);
	if (ret) {
		dev_err(sensor->dev, "Failed to enable regulators\n");
		return ret;
	}

	ret = clk_prepare_enable(sensor->clk);
	if (ret) {
		regulator_bulk_disable(ARRAY_SIZE(sensor->supplies),
				       sensor->supplies);
		dev_err(sensor->dev, "Failed to enable clock\n");
		return ret;
	}

	/*
	 * The internal initialization time after hard reset is 160000 EXTCLK
	 * cycles.
	 */
	rate = clk_get_rate(sensor->clk);
	reset_delay = DIV_ROUND_UP_ULL(160000ULL * USEC_PER_SEC, rate);

	if (sensor->reset) {
		gpiod_set_value_cansleep(sensor->reset, 1);
		fsleep(1000);
		gpiod_set_value_cansleep(sensor->reset, 0);
	} else {
		cci_write(sensor->regmap, AR0144_RESET_REGISTER, AR0144_RESET,
			  NULL);
	}

	fsleep(reset_delay);

	return 0;
}

static void ar0144_power_off(struct ar0144 *sensor)
{
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
	sensor->off_time = ktime_get_boottime();

	clk_disable_unprepare(sensor->clk);
	gpiod_set_value_cansleep(sensor->reset, 1);
}

static int ar0144_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar0144 *sensor = to_ar0144(sd);

	return ar0144_power_on(sensor);
}

static int ar0144_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar0144 *sensor = to_ar0144(sd);

	ar0144_power_off(sensor);

	return 0;
}

static const struct dev_pm_ops ar0144_pm_ops = {
	RUNTIME_PM_OPS(ar0144_runtime_suspend, ar0144_runtime_resume, NULL)
};

/* -----------------------------------------------------------------------------
 * Probe & remove
 */

static int ar0144_identify_device(struct ar0144 *sensor)
{
	const struct ar0144_model *model;
	u64 customer_rev;
	u64 chip_id;
	int ret = 0;

	cci_read(sensor->regmap, AR0144_CHIP_VERSION_REG, &chip_id, &ret);
	cci_read(sensor->regmap, AR0144_CUSTOMER_REV, &customer_rev, &ret);

	if (ret) {
		dev_err(sensor->dev, "Failed to read chip ID: %d\n", ret);
		return ret;
	}

	if (chip_id != AR0144_CHIP_VERSION_CS &&
	    chip_id != AR0144_CHIP_VERSION_AT) {
		dev_err(sensor->dev,
			"Wrong chip version 0x%04x (expected 0x%04x or 0x%04x)\n",
			(u32)chip_id, AR0144_CHIP_VERSION_CS,
			AR0144_CHIP_VERSION_AT);
		return -ENODEV;
	}

	switch (AR0144_CUSTOMER_REV_CFA(customer_rev)) {
	default:
		dev_warn(sensor->dev,
			 "Unknown CFA %u, defaulting to color\n",
			 (u32)AR0144_CUSTOMER_REV_CFA(customer_rev));
		fallthrough;
	case AR0144_CUSTOMER_REV_CFA_COLOR:
		model = &ar0144_model_color;
		break;
	case AR0144_CUSTOMER_REV_CFA_MONO:
		model = &ar0144_model_mono;
		break;
	}

	if (sensor->model != model)
		dev_warn(sensor->dev,
			 "Customer rev CFA doesn't match compatible string\n");

	sensor->model = model;

	if (IS_ENABLED(CONFIG_DYNAMIC_DEBUG) || IS_ENABLED(DEBUG)) {
		cci_read(sensor->regmap, AR0144_REVISION_NUMBER, &chip_id, NULL);
		dev_dbg(sensor->dev,
			"Sensor detected, CFA %s, CRA %u, OTPM r%u, silicon r%u, CREV r%u\n",
			sensor->model->mono ? "mono" : "color",
			(u32)AR0144_CUSTOMER_REV_CRA(customer_rev),
			(u32)AR0144_REVISION_NUMBER_OTPM(chip_id),
			(u32)AR0144_REVISION_NUMBER_SILICON(chip_id),
			(u32)AR0144_REVISION_NUMBER_CREV(chip_id));
	}

	return 0;
}

static int ar0144_parse_dt(struct ar0144 *sensor)
{
	struct v4l2_fwnode_endpoint *ep = &sensor->bus_cfg;
	struct fwnode_handle *endpoint;
	u64 valid_link_freqs = 0;
	unsigned int nlanes;
	unsigned int i, j;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!endpoint) {
		dev_err(sensor->dev, "Endpoint node not found\n");
		return -EINVAL;
	}

	ep->bus_type = V4L2_MBUS_UNKNOWN;
	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(sensor->dev, "Parsing endpoint node failed\n");
		goto error;
	}

	switch (ep->bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
		/* Validate the number of data lanes. */
		nlanes = ep->bus.mipi_csi2.num_data_lanes;
		if (nlanes != 1 && nlanes != 2) {
			dev_err(sensor->dev, "Invalid data lanes: %d\n", nlanes);
			ret = -EINVAL;
			goto error;
		}
		break;

	case V4L2_MBUS_PARALLEL:
		nlanes = ep->bus.parallel.bus_width ? : 12;
		break;

	default:
		dev_err(sensor->dev, "Unsupported bus type %u\n", ep->bus_type);
		goto error;
	}

	/* Get and validate the link frequencies. */
	if (!ep->nr_of_link_frequencies) {
		dev_err(sensor->dev, "link-frequency property not found in DT\n");
		ret = -EINVAL;
		goto error;
	}

	if (ep->nr_of_link_frequencies > 64) {
		dev_err(sensor->dev, "Too many link-frequency entries\n");
		ret = -EINVAL;
		goto error;
	}

	for (i = 0; i < ARRAY_SIZE(ar0144_formats); ++i) {
		const struct ar0144_format_info *info = &ar0144_formats[i];

		for (j = 0; j < ep->nr_of_link_frequencies; ++j) {
			u64 freq = ep->link_frequencies[j];
			struct ccs_pll pll;

			ret = ar0144_pll_calculate(sensor, &pll, freq,
						   info->bpp);
			if (ret)
				continue;

			sensor->valid_link_freqs[i] |= BIT(j);
			valid_link_freqs |= BIT(j);
		}

		if (!sensor->valid_link_freqs[i]) {
			dev_warn(sensor->dev,
				 "No valid link frequency for %u bpp\n",
				 info->bpp);
			continue;
		}

		sensor->valid_formats |= BIT(i);
	}

	if (!sensor->valid_formats) {
		dev_err(sensor->dev,
			"No valid link frequency found for any format\n");
		ret = -EINVAL;
		goto error;
	}

	for (i = 0; i < ep->nr_of_link_frequencies; ++i) {
		if (!(valid_link_freqs & BIT(i)))
			dev_warn(sensor->dev,
				 "Link frequency %llu not valid for any format\n",
				 ep->link_frequencies[i]);
	}

	dev_dbg(sensor->dev, "Using %u data lanes and %u link frequencies\n",
		nlanes, ep->nr_of_link_frequencies);

	return 0;

error:
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
	return ret;
}

static const char * const ar0144_supply_name[AR0144_NUM_SUPPLIES] = {
	"vaa",
	"vddio",
	"vdd",
};

static int ar0144_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ar0144 *sensor;
	unsigned int i;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = dev;
	sensor->model = device_get_match_data(sensor->dev);

	/* Acquire resources. */
	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "Unable to initialize I2C\n");

	sensor->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->clk))
		return dev_err_probe(dev, PTR_ERR(sensor->clk),
				     "Cannot get clock\n");

	sensor->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(dev, PTR_ERR(sensor->reset),
				     "Cannot get reset gpio\n");

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); i++)
		sensor->supplies[i].supply = ar0144_supply_name[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot get supplies\n");

	/* Parse the DT properties. */
	ret = ar0144_parse_dt(sensor);
	if (ret)
		return ret;

	/*
	 * Enable power management. The driver supports runtime PM, but needs to
	 * work when runtime PM is disabled in the kernel. To that end, power
	 * the sensor on manually here, identify it, and fully initialize it.
	 */
	ret = ar0144_power_on(sensor);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Could not power on the device\n");
		goto err_dt;
	}

	ret = ar0144_identify_device(sensor);
	if (ret)
		goto err_power;

	/*
	 * Enable runtime PM with autosuspend. As the device has been powered
	 * manually, mark it as active, and increase the usage count without
	 * resuming the device.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	/* Initialize and register the subdev. */
	ret = ar0144_init_subdev(sensor);
	if (ret) {
		dev_err(dev, "Subdev initialization error %d\n", ret);
		goto err_pm;
	}

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret) {
		dev_err(dev, "Could not register V4L2 subdevice\n");
		goto err_subdev;
	}

	/*
	 * Decrease the PM usage count. The device will get suspended after the
	 * autosuspend delay, turning the power off.
	 */
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_subdev:
	ar0144_cleanup_subdev(sensor);
err_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
err_power:
	ar0144_power_off(sensor);
err_dt:
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
	return ret;
}

static void ar0144_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0144 *sensor = to_ar0144(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	ar0144_cleanup_subdev(sensor);
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		ar0144_power_off(sensor);
	pm_runtime_set_suspended(sensor->dev);
}

static const struct of_device_id ar0144_of_match[] = {
	{ .compatible = "onnn,ar0144c", &ar0144_model_color },
	{ .compatible = "onnn,ar0144m", &ar0144_model_mono },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0144_of_match);

static struct i2c_driver ar0144_i2c_driver = {
	.driver = {
		.name = "ar0144",
		.of_match_table = ar0144_of_match,
		.pm = pm_ptr(&ar0144_pm_ops),
	},
	.probe = ar0144_probe,
	.remove = ar0144_remove,
};

module_i2c_driver(ar0144_i2c_driver);

MODULE_DESCRIPTION("onsemi AR0144 Camera Sensor Driver");
MODULE_LICENSE("GPL");
