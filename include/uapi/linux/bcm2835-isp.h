/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * bcm2835-isp.h
 *
 * BCM2835 ISP driver - user space header file.
 *
 * Copyright © 2019-2026 Raspberry Pi (Trading) Ltd.
 *
 * Author: Naushir Patuck (naush@raspberrypi.com)
 *
 */

#ifndef __BCM2835_ISP_H_
#define __BCM2835_ISP_H_

#include <linux/media/v4l2-isp.h>

/*
 * All structs below are directly mapped onto the equivalent structs in
 * drivers/staging/vc04_services/vchiq-mmal/mmal-parameters.h
 * for convenience.
 */

/**
 * struct bcm2835_isp_rational - Rational value type.
 *
 * @num:	Numerator.
 * @den:	Denominator.
 */
struct bcm2835_isp_rational {
	__s32 num;
	__u32 den;
};

/**
 * struct bcm2835_isp_ccm - Colour correction matrix.
 *
 * @ccm:	3x3 correction matrix coefficients.
 * @offsets:	1x3 correction offsets.
 */
struct bcm2835_isp_ccm {
	struct bcm2835_isp_rational ccm[3][3];
	__s32 offsets[3];
};

/**
 * struct bcm2835_isp_custom_ccm - Custom CCM configuration.
 *
 * @enabled:	Enable custom CCM.
 * @ccm:	Custom CCM coefficients and offsets.
 */
struct bcm2835_isp_custom_ccm {
	__u32 enabled;
	struct bcm2835_isp_ccm ccm;
};

/**
 * enum bcm2835_isp_gain_format - format of the gains in the lens shading
 *				  tables.
 *
 * @GAIN_FORMAT_U0P8_1:		Gains are u0.8 format, starting at 1.0
 * @GAIN_FORMAT_U1P7_0:		Gains are u1.7 format, starting at 0.0
 * @GAIN_FORMAT_U1P7_1:		Gains are u1.7 format, starting at 1.0
 * @GAIN_FORMAT_U2P6_0:		Gains are u2.6 format, starting at 0.0
 * @GAIN_FORMAT_U2P6_1:		Gains are u2.6 format, starting at 1.0
 * @GAIN_FORMAT_U3P5_0:		Gains are u3.5 format, starting at 0.0
 * @GAIN_FORMAT_U3P5_1:		Gains are u3.5 format, starting at 1.0
 * @GAIN_FORMAT_U4P10:		Gains are u4.10 format, starting at 0.0
 */
enum bcm2835_isp_gain_format {
	GAIN_FORMAT_U0P8_1 = 0,
	GAIN_FORMAT_U1P7_0 = 1,
	GAIN_FORMAT_U1P7_1 = 2,
	GAIN_FORMAT_U2P6_0 = 3,
	GAIN_FORMAT_U2P6_1 = 4,
	GAIN_FORMAT_U3P5_0 = 5,
	GAIN_FORMAT_U3P5_1 = 6,
	GAIN_FORMAT_U4P10  = 7,
};

/**
 * struct bcm2835_isp_lens_shading - Lens shading tables.
 *
 * @enabled:		Enable lens shading.
 * @grid_cell_size:	Size of grid cells in samples (16, 32, 64, 128 or 256).
 * @grid_width:		Width of lens shading tables in grid cells.
 * @grid_stride:	Row to row distance (in grid cells) between grid cells
 *			in the same horizontal location.
 * @grid_height:	Height of lens shading tables in grid cells.
 * @dmabuf:		dmabuf file handle containing the table.
 * @ref_transform:	Reference transform - unsupported, please pass zero.
 * @corner_sampled:	Whether the gains are sampled at the corner points
 *			of the grid cells or in the cell centres.
 * @gain_format:	Format of the gains (see enum &bcm2835_isp_gain_format).
 */
struct bcm2835_isp_lens_shading {
	__u32 enabled;
	__u32 grid_cell_size;
	__u32 grid_width;
	__u32 grid_stride;
	__u32 grid_height;
	__s32 dmabuf;
	__u32 ref_transform;
	__u32 corner_sampled;
	__u32 gain_format;
};

/**
 * struct bcm2835_isp_black_level - Sensor black level configuration.
 *
 * @enabled:		Enable black level.
 * @black_level_r:	Black level for red channel.
 * @black_level_g:	Black level for green channels.
 * @black_level_b:	Black level for blue channel.
 * @padding:		Unused padding.
 */
struct bcm2835_isp_black_level {
	__u32 enabled;
	__u16 black_level_r;
	__u16 black_level_g;
	__u16 black_level_b;
	__u8 padding[2]; /* Unused */
};

/**
 * struct bcm2835_isp_geq - Green equalisation parameters.
 *
 * @enabled:	Enable green equalisation.
 * @offset:	Fixed offset of the green equalisation threshold.
 * @slope:	Slope of the green equalisation threshold.
 */
struct bcm2835_isp_geq {
	__u32 enabled;
	__u32 offset;
	struct bcm2835_isp_rational slope;
};

#define BCM2835_NUM_GAMMA_PTS 33

/**
 * struct bcm2835_isp_gamma - Gamma parameters.
 *
 * @enabled:	Enable gamma adjustment.
 * @x:		X values of the points defining the gamma curve.
 *		Values should be scaled to 16 bits.
 * @y:		Y values of the points defining the gamma curve.
 *		Values should be scaled to 16 bits.
 */
struct bcm2835_isp_gamma {
	__u32 enabled;
	__u16 x[BCM2835_NUM_GAMMA_PTS];
	__u16 y[BCM2835_NUM_GAMMA_PTS];
};

/**
 * enum bcm2835_isp_cdn_mode - Mode of operation for colour denoise.
 *
 * @CDN_MODE_FAST:		Fast (but lower quality) colour denoise
 *				algorithm, typically used for video recording.
 * @CDN_MODE_HIGH_QUALITY:	High quality (but slower) colour denoise
 *				algorithm, typically used for stills capture.
 */
enum bcm2835_isp_cdn_mode {
	CDN_MODE_FAST = 0,
	CDN_MODE_HIGH_QUALITY = 1,
};

/**
 * struct bcm2835_isp_cdn - Colour denoise parameters.
 *
 * @enabled:	Enable colour denoise.
 * @mode:	Colour denoise operating mode (see enum &bcm2835_isp_cdn_mode)
 */
struct bcm2835_isp_cdn {
	__u32 enabled;
	__u32 mode;
};

/**
 * struct bcm2835_isp_denoise - Denoise parameters.
 *
 * @enabled:	Enable denoise.
 * @constant:	Fixed offset of the noise threshold.
 * @slope:	Slope of the noise threshold.
 * @strength:	Denoise strength between 0.0 (off) and 1.0 (maximum).
 */
struct bcm2835_isp_denoise {
	__u32 enabled;
	__u32 constant;
	struct bcm2835_isp_rational slope;
	struct bcm2835_isp_rational strength;
};

/**
 * struct bcm2835_isp_sharpen - Sharpen parameters.
 *
 * @enabled:	Enable sharpening.
 * @threshold:	Threshold at which to start sharpening pixels.
 * @strength:	Strength with which pixel sharpening increases.
 * @limit:	Limit to the amount of sharpening applied.
 */
struct bcm2835_isp_sharpen {
	__u32 enabled;
	struct bcm2835_isp_rational threshold;
	struct bcm2835_isp_rational strength;
	struct bcm2835_isp_rational limit;
};

/**
 * enum bcm2835_isp_dpc_mode - defective pixel correction (DPC) strength.
 *
 * @DPC_MODE_OFF:		No DPC.
 * @DPC_MODE_NORMAL:		Normal DPC.
 * @DPC_MODE_STRONG:		Strong DPC.
 */
enum bcm2835_isp_dpc_mode {
	DPC_MODE_OFF = 0,
	DPC_MODE_NORMAL = 1,
	DPC_MODE_STRONG = 2,
};

/**
 * struct bcm2835_isp_dpc - Defective pixel correction (DPC) parameters.
 *
 * @enabled:	Enable DPC.
 * @strength:	DPC strength (see enum &bcm2835_isp_dpc_mode).
 */
struct bcm2835_isp_dpc {
	__u32 enabled;
	__u32 strength;
};

/**
 * struct bcm2835_isp_awb_gains - AWB gains configuration.
 *
 * @r_gain:	Red channel AWB gain.
 * @b_gain:	Blue channel AWB gain.
 */
struct bcm2835_isp_awb_gains {
	struct bcm2835_isp_rational r_gain;
	struct bcm2835_isp_rational b_gain;
};

/**
 * struct bcm2835_isp_digital_gain - Digital gain configuration.
 *
 * @gain:	Digital gain value.
 */
struct bcm2835_isp_digital_gain {
	struct bcm2835_isp_rational gain;
};

/*
 * BCM2835 ISP extensible parameters buffer definitions.
 *
 * The extensible parameters mechanism allows userspace to submit ISP
 * configuration parameters as a buffer containing a series of tagged
 * blocks rather than individual V4L2 controls. This enables atomic
 * application of multiple parameters in a single operation.
 */

/**
 * enum bcm2835_isp_param_buffer_version - BCM2835 ISP parameters buffer version
 *
 * @BCM2835_ISP_PARAM_BUFFER_V1: First version of parameters buffer format
 */
enum bcm2835_isp_param_buffer_version {
	BCM2835_ISP_PARAM_BUFFER_V1 = V4L2_ISP_PARAMS_VERSION_V1,
};

/**
 * enum bcm2835_isp_param_block_type - BCM2835 ISP parameter block types
 *
 * This enumeration defines the types of parameters blocks that can be
 * included in the extensible parameters buffer. Each block type corresponds
 * to a specific ISP processing block configuration.
 *
 * @BCM2835_ISP_PARAM_BLOCK_BLACK_LEVEL: Black level configuration
 * @BCM2835_ISP_PARAM_BLOCK_GEQ: Green equalisation configuration
 * @BCM2835_ISP_PARAM_BLOCK_GAMMA: Gamma curve configuration
 * @BCM2835_ISP_PARAM_BLOCK_DENOISE: Denoise configuration
 * @BCM2835_ISP_PARAM_BLOCK_SHARPEN: Sharpening configuration
 * @BCM2835_ISP_PARAM_BLOCK_DPC: Defective pixel correction configuration
 * @BCM2835_ISP_PARAM_BLOCK_CDN: Colour denoise configuration
 * @BCM2835_ISP_PARAM_BLOCK_CC_MATRIX: Colour correction matrix configuration
 * @BCM2835_ISP_PARAM_BLOCK_LENS_SHADING: Lens shading table configuration
 * @BCM2835_ISP_PARAM_BLOCK_AWB_GAINS: AWB gains configuration
 * @BCM2835_ISP_PARAM_BLOCK_DIGITAL_GAIN: Digital gain configuration
 */
enum bcm2835_isp_param_block_type {
	BCM2835_ISP_PARAM_BLOCK_BLACK_LEVEL,
	BCM2835_ISP_PARAM_BLOCK_GEQ,
	BCM2835_ISP_PARAM_BLOCK_GAMMA,
	BCM2835_ISP_PARAM_BLOCK_DENOISE,
	BCM2835_ISP_PARAM_BLOCK_SHARPEN,
	BCM2835_ISP_PARAM_BLOCK_DPC,
	BCM2835_ISP_PARAM_BLOCK_CDN,
	BCM2835_ISP_PARAM_BLOCK_CC_MATRIX,
	BCM2835_ISP_PARAM_BLOCK_LENS_SHADING,
	BCM2835_ISP_PARAM_BLOCK_AWB_GAINS,
	BCM2835_ISP_PARAM_BLOCK_DIGITAL_GAIN,
};

/**
 * struct bcm2835_isp_params_black_level - Black level parameters block
 *
 * @header: Block header (type = BCM2835_ISP_PARAM_BLOCK_BLACK_LEVEL)
 * @black_level: Black level configuration
 */
struct bcm2835_isp_params_black_level {
	struct v4l2_isp_params_block_header header;
	struct bcm2835_isp_black_level black_level;
} __attribute__((aligned(8)));

/**
 * struct bcm2835_isp_params_geq - Green equalisation parameters block
 *
 * @header: Block header (type = BCM2835_ISP_PARAM_BLOCK_GEQ)
 * @geq: Green equalisation configuration
 */
struct bcm2835_isp_params_geq {
	struct v4l2_isp_params_block_header header;
	struct bcm2835_isp_geq geq;
} __attribute__((aligned(8)));

/**
 * struct bcm2835_isp_params_gamma - Gamma parameters block
 *
 * @header: Block header (type = BCM2835_ISP_PARAM_BLOCK_GAMMA)
 * @gamma: Gamma curve configuration
 */
struct bcm2835_isp_params_gamma {
	struct v4l2_isp_params_block_header header;
	struct bcm2835_isp_gamma gamma;
} __attribute__((aligned(8)));

/**
 * struct bcm2835_isp_params_denoise - Denoise parameters block
 *
 * @header: Block header (type = BCM2835_ISP_PARAM_BLOCK_DENOISE)
 * @denoise: Denoise configuration
 */
struct bcm2835_isp_params_denoise {
	struct v4l2_isp_params_block_header header;
	struct bcm2835_isp_denoise denoise;
} __attribute__((aligned(8)));

/**
 * struct bcm2835_isp_params_sharpen - Sharpen parameters block
 *
 * @header: Block header (type = BCM2835_ISP_PARAM_BLOCK_SHARPEN)
 * @sharpen: Sharpening configuration
 */
struct bcm2835_isp_params_sharpen {
	struct v4l2_isp_params_block_header header;
	struct bcm2835_isp_sharpen sharpen;
} __attribute__((aligned(8)));

/**
 * struct bcm2835_isp_params_dpc - Defective pixel correction parameters block
 *
 * @header: Block header (type = BCM2835_ISP_PARAM_BLOCK_DPC)
 * @dpc: DPC configuration
 */
struct bcm2835_isp_params_dpc {
	struct v4l2_isp_params_block_header header;
	struct bcm2835_isp_dpc dpc;
} __attribute__((aligned(8)));

/**
 * struct bcm2835_isp_params_cdn - Colour denoise parameters block
 *
 * @header: Block header (type = BCM2835_ISP_PARAM_BLOCK_CDN)
 * @cdn: Colour denoise configuration
 */
struct bcm2835_isp_params_cdn {
	struct v4l2_isp_params_block_header header;
	struct bcm2835_isp_cdn cdn;
} __attribute__((aligned(8)));

/**
 * struct bcm2835_isp_params_cc_matrix - Colour correction matrix parameters block
 *
 * @header: Block header (type = BCM2835_ISP_PARAM_BLOCK_CC_MATRIX)
 * @ccm: Colour correction matrix configuration
 */
struct bcm2835_isp_params_cc_matrix {
	struct v4l2_isp_params_block_header header;
	struct bcm2835_isp_custom_ccm ccm;
} __attribute__((aligned(8)));

/**
 * struct bcm2835_isp_params_lens_shading - Lens shading parameters block
 *
 * @header: Block header (type = BCM2835_ISP_PARAM_BLOCK_LENS_SHADING)
 * @ls: Lens shading configuration (includes dmabuf fd for table data)
 */
struct bcm2835_isp_params_lens_shading {
	struct v4l2_isp_params_block_header header;
	struct bcm2835_isp_lens_shading ls;
} __attribute__((aligned(8)));

/**
 * struct bcm2835_isp_params_awb_gains - AWB gains parameters block
 *
 * @header: Block header (type = BCM2835_ISP_PARAM_BLOCK_AWB_GAINS)
 * @awb_gains: AWB gains configuration
 */
struct bcm2835_isp_params_awb_gains {
	struct v4l2_isp_params_block_header header;
	struct bcm2835_isp_awb_gains awb_gains;
} __attribute__((aligned(8)));

/**
 * struct bcm2835_isp_params_digital_gain - Digital gain parameters block
 *
 * @header: Block header (type = BCM2835_ISP_PARAM_BLOCK_DIGITAL_GAIN)
 * @digital_gain: Digital gain configuration
 */
struct bcm2835_isp_params_digital_gain {
	struct v4l2_isp_params_block_header header;
	struct bcm2835_isp_digital_gain digital_gain;
} __attribute__((aligned(8)));

/**
 * define BCM2835_ISP_PARAMS_MAX_SIZE - Maximum size of all ISP parameters
 *
 * This defines the maximum size needed to accommodate all possible parameter
 * blocks in a single buffer. Drivers use this to allocate appropriately
 * sized buffers.
 */
#define BCM2835_ISP_PARAMS_MAX_SIZE					\
	(sizeof(struct bcm2835_isp_params_black_level) +		\
	 sizeof(struct bcm2835_isp_params_geq) +			\
	 sizeof(struct bcm2835_isp_params_gamma) +			\
	 sizeof(struct bcm2835_isp_params_denoise) +			\
	 sizeof(struct bcm2835_isp_params_sharpen) +			\
	 sizeof(struct bcm2835_isp_params_dpc) +			\
	 sizeof(struct bcm2835_isp_params_cdn) +			\
	 sizeof(struct bcm2835_isp_params_cc_matrix) +			\
	 sizeof(struct bcm2835_isp_params_lens_shading) +		\
	 sizeof(struct bcm2835_isp_params_awb_gains) +			\
	 sizeof(struct bcm2835_isp_params_digital_gain))

/*
 * ISP statistics structures.
 *
 * The bcm2835_isp_stats structure is generated at the output of the
 * statistics node.  Note that this does not directly map onto the statistics
 * output of the ISP HW.  Instead, the MMAL firmware code maps the HW statistics
 * to the bcm2835_isp_stats structure.
 */
#define DEFAULT_AWB_REGIONS_X 16
#define DEFAULT_AWB_REGIONS_Y 12

#define NUM_HISTOGRAMS 2
#define NUM_HISTOGRAM_BINS 128
#define AWB_REGIONS (DEFAULT_AWB_REGIONS_X * DEFAULT_AWB_REGIONS_Y)
#define FLOATING_REGIONS 16
#define AGC_REGIONS 16
#define FOCUS_REGIONS 12

/**
 * struct bcm2835_isp_stats_hist - Histogram statistics
 *
 * @r_hist:	Red channel histogram.
 * @g_hist:	Combined green channel histogram.
 * @b_hist:	Blue channel histogram.
 */
struct bcm2835_isp_stats_hist {
	__u32 r_hist[NUM_HISTOGRAM_BINS];
	__u32 g_hist[NUM_HISTOGRAM_BINS];
	__u32 b_hist[NUM_HISTOGRAM_BINS];
};

/**
 * struct bcm2835_isp_stats_region - Region sums.
 *
 * @counted:	The number of 2x2 bayer tiles accumulated.
 * @notcounted:	The number of 2x2 bayer tiles not accumulated.
 * @r_sum:	Total sum of counted pixels in the red channel for a region.
 * @g_sum:	Total sum of counted pixels in the green channel for a region.
 * @b_sum:	Total sum of counted pixels in the blue channel for a region.
 */
struct bcm2835_isp_stats_region {
	__u32 counted;
	__u32 notcounted;
	__u64 r_sum;
	__u64 g_sum;
	__u64 b_sum;
};

/**
 * struct bcm2835_isp_stats_focus - Focus statistics.
 *
 * @contrast_val:	Focus measure - accumulated output of the focus filter.
 *			In the first dimension, index [0] counts pixels below a
 *			preset threshold, and index [1] counts pixels above the
 *			threshold.  In the second dimension, index [0] uses the
 *			first predefined filter, and index [1] uses the second
 *			predefined filter.
 * @contrast_val_num:	The number of counted pixels in the above accumulation.
 */
struct bcm2835_isp_stats_focus {
	__u64 contrast_val[2][2];
	__u32 contrast_val_num[2][2];
};

/**
 * struct bcm2835_isp_stats - ISP statistics.
 *
 * @version:		Version of the bcm2835_isp_stats structure.
 * @size:		Size of the bcm2835_isp_stats structure.
 * @hist:		Histogram statistics for the entire image.
 * @awb_stats:		Statistics for the regions defined for AWB calculations.
 * @floating_stats:	Statistics for arbitrarily placed (floating) regions.
 * @agc_stats:		Statistics for the regions defined for AGC calculations.
 * @focus_stats:	Focus filter statistics for the focus regions.
 */
struct bcm2835_isp_stats {
	__u32 version;
	__u32 size;
	struct bcm2835_isp_stats_hist hist[NUM_HISTOGRAMS];
	struct bcm2835_isp_stats_region awb_stats[AWB_REGIONS];
	struct bcm2835_isp_stats_region floating_stats[FLOATING_REGIONS];
	struct bcm2835_isp_stats_region agc_stats[AGC_REGIONS];
	struct bcm2835_isp_stats_focus focus_stats[FOCUS_REGIONS];
};

#endif /* __BCM2835_ISP_H_ */
