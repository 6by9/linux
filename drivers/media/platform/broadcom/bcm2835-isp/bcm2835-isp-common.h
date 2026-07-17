/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Broadcom BCM2835 ISP driver - common header
 *
 * Copyright (c) 2026 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2026 Ideas On Board Oy
 */

#ifndef BCM2835_ISP_COMMON_H
#define BCM2835_ISP_COMMON_H

#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/raspberrypi/mmal-parameters.h>
#include <linux/raspberrypi/mmal-vchiq.h>

#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>

struct dma_buf;

struct bcm2835_isp_params {
	struct device *dev;
	struct v4l2_device *v4l2_dev;
	struct vchiq_mmal_instance *mmal_instance;
	struct vchiq_mmal_port *port;

	struct video_device vdev;
	struct media_pad pad;
	struct vb2_queue queue;
	struct mutex lock; /* params node vdev lock */

	/* Lens shading state */
	struct dma_buf *last_ls_dmabuf;
	struct mmal_parameter_lens_shading_v2 ls;
};

struct bcm2835_isp_params *
bcm2835_isp_params_register(struct v4l2_device *v4l2_dev, struct device *dev,
			    struct vchiq_mmal_instance *mmal_instance,
			    struct vchiq_mmal_port *port, int video_nr);

void bcm2835_isp_params_unregister(struct bcm2835_isp_params *params);

void bcm2835_isp_params_drop_ls_ref(struct bcm2835_isp_params *params);

int bcm2835_isp_node_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap);

#endif /* BCM2835_ISP_COMMON_H */
