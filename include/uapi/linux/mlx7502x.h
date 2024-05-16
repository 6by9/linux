/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Melexis 7502x ToF cameras driver.
 *
 * Copyright (C) 2021 Melexis N.V.
 *
 */

#ifndef __UAPI_MLX7502X_H_
#define __UAPI_MLX7502X_H_

#include <linux/v4l2-controls.h>

/*
 * this is related to the taps in ToF cameras,
 * usually A minus B is the best option
 */
enum v4l2_mlx7502x_output_mode {
	V4L2_MLX7502X_OUTPUT_MODE_A_MINUS_B = 0,
	V4L2_MLX7502X_OUTPUT_MODE_A_PLUS_B,
	V4L2_MLX7502X_OUTPUT_MODE_A,
	V4L2_MLX7502X_OUTPUT_MODE_B,
	V4L2_MLX7502X_OUTPUT_MODE_A_AND_B, /* output frame size doubles */
};

#define V4L2_CID_MLX7502X_OUTPUT_MODE	(V4L2_CID_USER_MLX7502X_BASE + 0)

#endif /* __UAPI_MLX7502X_H_ */
