// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom BCM2835 ISP driver - extensible parameters node
 *
 * Copyright (c) 2026 Raspberry Pi Ltd.
 * Copyright (c) 2026 Ideas On Board Oy
 *
 * Author: Jai Luthra <jai.luthra@ideasonboard.com>
 */

#include <linux/dma-buf.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/raspberrypi/mmal-parameters.h>
#include <linux/raspberrypi/vc_sm_knl.h>

#include <media/v4l2-ioctl.h>
#include <media/v4l2-isp.h>
#include <media/videobuf2-dma-contig.h>

#include <uapi/linux/bcm2835-isp.h>

#include "bcm2835-isp-common.h"

MODULE_IMPORT_NS("DMA_BUF");

#define BCM2835_ISP_PARAMS_NAME "bcm2835-isp-params"

#define BCM2835_ISP_PARAMS_BUF_SIZE \
	v4l2_isp_params_buffer_size(BCM2835_ISP_PARAMS_MAX_SIZE)

/**
 * union bcm2835_isp_params_block - Generalisation of a parameter block
 *
 * @header:	Block header pointer for type checking
 * @black_level: Black level configuration block
 * @geq:	Green equalisation configuration block
 * @gamma:	Gamma curve configuration block
 * @denoise:	Denoise configuration block
 * @sharpen:	Sharpen configuration block
 * @dpc:	Defective pixel correction configuration block
 * @cdn:	Colour denoise configuration block
 * @ccm:	Colour correction matrix configuration block
 * @ls:		Lens shading configuration block
 * @awb_gains:	AWB gains configuration block
 * @digital_gain: Digital gain configuration block
 * @data:	Raw pointer for block iteration
 */
union bcm2835_isp_params_block {
	const struct v4l2_isp_params_block_header *header;
	const struct bcm2835_isp_params_black_level *black_level;
	const struct bcm2835_isp_params_geq *geq;
	const struct bcm2835_isp_params_gamma *gamma;
	const struct bcm2835_isp_params_denoise *denoise;
	const struct bcm2835_isp_params_sharpen *sharpen;
	const struct bcm2835_isp_params_dpc *dpc;
	const struct bcm2835_isp_params_cdn *cdn;
	const struct bcm2835_isp_params_cc_matrix *ccm;
	const struct bcm2835_isp_params_lens_shading *ls;
	const struct bcm2835_isp_params_awb_gains *awb_gains;
	const struct bcm2835_isp_params_digital_gain *digital_gain;
	const __u8 *data;
};

typedef void (*bcm2835_isp_params_handler)(struct mmal_parameter_isp_parameters *mmal_param,
					   struct bcm2835_isp_params *params,
					   union bcm2835_isp_params_block block);

struct bcm2835_isp_params_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	void *config;
};

#define to_bcm2835_isp_params_buf(vbuf) \
	container_of(vbuf, struct bcm2835_isp_params_buffer, vb)

static int isp_set_param(struct bcm2835_isp_params *params, u32 parameter, void
			 *value, u32 value_size)
{
	return vchiq_mmal_port_parameter_set(params->mmal_instance, params->port,
					     parameter, value, value_size);
}

static int map_ls_table(struct bcm2835_isp_params *params,
			struct dma_buf *dmabuf,
			const struct bcm2835_isp_lens_shading *v4l2_ls)
{
	void *vcsm_handle;
	int ret;

	if (IS_ERR_OR_NULL(dmabuf))
		return -EINVAL;

	/*
	 * struct bcm2835_isp_lens_shading and struct
	 * mmal_parameter_lens_shading_v2 match so that we can do a
	 * simple memcpy here.
	 * Only the dmabuf to the actual table needs any manipulation.
	 */
	memcpy(&params->ls, v4l2_ls, sizeof(params->ls));
	ret = vc_sm_cma_import_dmabuf(dmabuf, &vcsm_handle);
	if (ret) {
		dma_buf_put(dmabuf);
		return ret;
	}

	params->ls.mem_handle_table = vc_sm_cma_int_handle(vcsm_handle);
	params->last_ls_dmabuf = dmabuf;

	vc_sm_cma_free(vcsm_handle);

	return 0;
}

/* Block handlers */

#define BCM2835_ISP_PARAMS_HANDLER(_name, _param_field, _mmal_field)		\
static void bcm2835_isp_params_##_name(struct mmal_parameter_isp_parameters *mmal_param, \
				       struct bcm2835_isp_params *params,	\
				       union bcm2835_isp_params_block block)	\
{										\
	memcpy(&mmal_param->_mmal_field,					\
		      (void *)&block._param_field->_param_field,		\
		      sizeof(block._param_field->_param_field));		\
	u32 *ptr = (u32 *)&mmal_param->_mmal_field;				\
	ptr--;									\
	*ptr = 1;								\
}

BCM2835_ISP_PARAMS_HANDLER(black_level, black_level, black_level.enable)
BCM2835_ISP_PARAMS_HANDLER(geq, geq, green_eq.enable)
BCM2835_ISP_PARAMS_HANDLER(gamma, gamma, gamma.enable)
BCM2835_ISP_PARAMS_HANDLER(denoise, denoise, denoise.enable)
BCM2835_ISP_PARAMS_HANDLER(sharpen, sharpen, sharpen.enable)
BCM2835_ISP_PARAMS_HANDLER(dpc, dpc, dpc.enable)
BCM2835_ISP_PARAMS_HANDLER(cdn, cdn, colour_denoise.enable)
BCM2835_ISP_PARAMS_HANDLER(cc_matrix, ccm, ccm.enable)
BCM2835_ISP_PARAMS_HANDLER(awb_gains, awb_gains, awb_gains.r_gain)
BCM2835_ISP_PARAMS_HANDLER(digital_gain, digital_gain, digital_gain.value)

static void bcm2835_isp_params_lens_shading(struct mmal_parameter_isp_parameters *mmal_param,
					    struct bcm2835_isp_params *params,
					    union bcm2835_isp_params_block block)
{
	struct dma_buf *dmabuf;
	int ret = 0;

	dmabuf = dma_buf_get(block.ls->ls.dmabuf);
	if (IS_ERR(dmabuf))
		return;
	if (dmabuf != params->last_ls_dmabuf)
		ret = map_ls_table(params, dmabuf, &block.ls->ls);

	if (!ret && mmal_param->lens_shading.mem_handle_table) {
		memcpy(&mmal_param->lens_shading.enabled, &params->ls,
		       sizeof(params->ls));
		mmal_param->lens_shading.update = 1;
	}
	dma_buf_put(dmabuf);
}

static const bcm2835_isp_params_handler bcm2835_isp_params_handlers[] = {
	[BCM2835_ISP_PARAM_BLOCK_BLACK_LEVEL] = bcm2835_isp_params_black_level,
	[BCM2835_ISP_PARAM_BLOCK_GEQ] = bcm2835_isp_params_geq,
	[BCM2835_ISP_PARAM_BLOCK_GAMMA] = bcm2835_isp_params_gamma,
	[BCM2835_ISP_PARAM_BLOCK_DENOISE] = bcm2835_isp_params_denoise,
	[BCM2835_ISP_PARAM_BLOCK_SHARPEN] = bcm2835_isp_params_sharpen,
	[BCM2835_ISP_PARAM_BLOCK_DPC] = bcm2835_isp_params_dpc,
	[BCM2835_ISP_PARAM_BLOCK_CDN] = bcm2835_isp_params_cdn,
	[BCM2835_ISP_PARAM_BLOCK_CC_MATRIX] = bcm2835_isp_params_cc_matrix,
	[BCM2835_ISP_PARAM_BLOCK_LENS_SHADING] = bcm2835_isp_params_lens_shading,
	[BCM2835_ISP_PARAM_BLOCK_AWB_GAINS] = bcm2835_isp_params_awb_gains,
	[BCM2835_ISP_PARAM_BLOCK_DIGITAL_GAIN] = bcm2835_isp_params_digital_gain,
};

static const struct v4l2_isp_params_block_type_info
bcm2835_isp_params_block_types_info[] = {
	[BCM2835_ISP_PARAM_BLOCK_BLACK_LEVEL] = {
		.size = sizeof(struct bcm2835_isp_params_black_level),
	},
	[BCM2835_ISP_PARAM_BLOCK_GEQ] = {
		.size = sizeof(struct bcm2835_isp_params_geq),
	},
	[BCM2835_ISP_PARAM_BLOCK_GAMMA] = {
		.size = sizeof(struct bcm2835_isp_params_gamma),
	},
	[BCM2835_ISP_PARAM_BLOCK_DENOISE] = {
		.size = sizeof(struct bcm2835_isp_params_denoise),
	},
	[BCM2835_ISP_PARAM_BLOCK_SHARPEN] = {
		.size = sizeof(struct bcm2835_isp_params_sharpen),
	},
	[BCM2835_ISP_PARAM_BLOCK_DPC] = {
		.size = sizeof(struct bcm2835_isp_params_dpc),
	},
	[BCM2835_ISP_PARAM_BLOCK_CDN] = {
		.size = sizeof(struct bcm2835_isp_params_cdn),
	},
	[BCM2835_ISP_PARAM_BLOCK_CC_MATRIX] = {
		.size = sizeof(struct bcm2835_isp_params_cc_matrix),
	},
	[BCM2835_ISP_PARAM_BLOCK_LENS_SHADING] = {
		.size = sizeof(struct bcm2835_isp_params_lens_shading),
	},
	[BCM2835_ISP_PARAM_BLOCK_AWB_GAINS] = {
		.size = sizeof(struct bcm2835_isp_params_awb_gains),
	},
	[BCM2835_ISP_PARAM_BLOCK_DIGITAL_GAIN] = {
		.size = sizeof(struct bcm2835_isp_params_digital_gain),
	},
};

static_assert(ARRAY_SIZE(bcm2835_isp_params_handlers) ==
	      ARRAY_SIZE(bcm2835_isp_params_block_types_info));

static void bcm2835_isp_params_apply(struct bcm2835_isp_params *params,
				     struct bcm2835_isp_params_buffer *buf)
{
	const struct v4l2_isp_params_buffer *config = buf->config;
	size_t block_offset = 0;
	size_t max_offset = config->data_size;
	struct mmal_parameter_isp_parameters mmal_param = { 0 };

	while (block_offset < max_offset) {
		union bcm2835_isp_params_block block;
		bcm2835_isp_params_handler handler;

		block.data = &config->data[block_offset];
		handler = bcm2835_isp_params_handlers[block.header->type];
		handler(&mmal_param, params, block);

		block_offset += block.header->size;
	}

	isp_set_param(params, MMAL_PARAMETER_ISP_SETTINGS, &mmal_param,
		      sizeof(mmal_param));

	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

/* vb2 operations */

static int bcm2835_isp_params_queue_setup(struct vb2_queue *q,
					  unsigned int *num_buffers,
					  unsigned int *num_planes,
					  unsigned int sizes[],
					  struct device *alloc_devs[])
{
	if (*num_planes && *num_planes > 1)
		return -EINVAL;

	if (sizes[0] && sizes[0] < BCM2835_ISP_PARAMS_BUF_SIZE)
		return -EINVAL;

	*num_planes = 1;

	if (!sizes[0])
		sizes[0] = BCM2835_ISP_PARAMS_BUF_SIZE;

	return 0;
}

static int bcm2835_isp_params_buf_init(struct vb2_buffer *vb)
{
	struct bcm2835_isp_params_buffer *buf =
		to_bcm2835_isp_params_buf(to_vb2_v4l2_buffer(vb));

	buf->config = kvmalloc(BCM2835_ISP_PARAMS_BUF_SIZE, GFP_KERNEL);
	if (!buf->config)
		return -ENOMEM;

	return 0;
}

static void bcm2835_isp_params_buf_cleanup(struct vb2_buffer *vb)
{
	struct bcm2835_isp_params_buffer *buf =
		to_bcm2835_isp_params_buf(to_vb2_v4l2_buffer(vb));

	kvfree(buf->config);
	buf->config = NULL;
}

static int bcm2835_isp_params_buf_prepare(struct vb2_buffer *vb)
{
	struct bcm2835_isp_params *params = vb2_get_drv_priv(vb->vb2_queue);
	struct bcm2835_isp_params_buffer *buf =
		to_bcm2835_isp_params_buf(to_vb2_v4l2_buffer(vb));
	const struct v4l2_isp_params_buffer *config;
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(params->dev, vb,
						   BCM2835_ISP_PARAMS_BUF_SIZE);
	if (ret)
		return ret;

	config = vb2_plane_vaddr(vb, 0);
	if (config->version != BCM2835_ISP_PARAM_BUFFER_V1)
		return -EINVAL;

	/* Copy into scratch buffer */
	memcpy(buf->config, config, BCM2835_ISP_PARAMS_BUF_SIZE);

	return v4l2_isp_params_validate_buffer(params->dev, vb, buf->config,
					       bcm2835_isp_params_block_types_info,
					       ARRAY_SIZE(bcm2835_isp_params_block_types_info));
}

static void bcm2835_isp_params_buf_queue(struct vb2_buffer *vb)
{
	struct bcm2835_isp_params *params = vb2_get_drv_priv(vb->vb2_queue);
	struct bcm2835_isp_params_buffer *buf =
		to_bcm2835_isp_params_buf(to_vb2_v4l2_buffer(vb));

	/*
	 * Apply params immediately - the firmware will apply them on the
	 * next frame boundary.
	 */
	bcm2835_isp_params_apply(params, buf);
}

static const struct vb2_ops bcm2835_isp_params_vb2_ops = {
	.queue_setup = bcm2835_isp_params_queue_setup,
	.buf_init = bcm2835_isp_params_buf_init,
	.buf_cleanup = bcm2835_isp_params_buf_cleanup,
	.buf_prepare = bcm2835_isp_params_buf_prepare,
	.buf_queue = bcm2835_isp_params_buf_queue,
};

/* V4L2 ioctls */

static int bcm2835_isp_params_enum_fmt(struct file *file, void *fh,
				       struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_META_FMT_BCM2835_ISP_PARAMS;

	return 0;
}

static int bcm2835_isp_params_g_fmt(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	f->fmt.meta.dataformat = V4L2_META_FMT_BCM2835_ISP_PARAMS;
	f->fmt.meta.buffersize = BCM2835_ISP_PARAMS_BUF_SIZE;

	return 0;
}

static const struct v4l2_ioctl_ops bcm2835_isp_params_ioctl_ops = {
	.vidioc_querycap = bcm2835_isp_node_querycap,
	.vidioc_enum_fmt_meta_out = bcm2835_isp_params_enum_fmt,
	.vidioc_g_fmt_meta_out = bcm2835_isp_params_g_fmt,
	.vidioc_s_fmt_meta_out = bcm2835_isp_params_g_fmt,
	.vidioc_try_fmt_meta_out = bcm2835_isp_params_g_fmt,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,

	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations bcm2835_isp_params_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

struct bcm2835_isp_params *
bcm2835_isp_params_register(struct v4l2_device *v4l2_dev, struct device *dev,
			    struct vchiq_mmal_instance *mmal_instance,
			    struct vchiq_mmal_port *port, int video_nr)
{
	struct bcm2835_isp_params *params;
	struct video_device *vdev;
	struct vb2_queue *q;
	int ret;

	params = devm_kzalloc(dev, sizeof(*params), GFP_KERNEL);
	if (!params)
		return ERR_PTR(-ENOMEM);

	params->dev = dev;
	params->v4l2_dev = v4l2_dev;
	params->mmal_instance = mmal_instance;
	params->port = port;

	mutex_init(&params->lock);

	/* Initialize vb2 queue */
	q = &params->queue;
	q->type = V4L2_BUF_TYPE_META_OUTPUT;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = params;
	q->ops = &bcm2835_isp_params_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct bcm2835_isp_params_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	q->dev = dev;
	q->lock = &params->lock;

	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(dev, "Failed to init params vb2 queue\n");
		return ERR_PTR(ret);
	}

	/* Initialize video device */
	vdev = &params->vdev;
	vdev->device_caps = V4L2_CAP_META_OUTPUT | V4L2_CAP_STREAMING;
	vdev->fops = &bcm2835_isp_params_fops;
	vdev->ioctl_ops = &bcm2835_isp_params_ioctl_ops;
	vdev->minor = -1;
	vdev->release = video_device_release_empty;
	vdev->queue = q;
	vdev->lock = &params->lock;
	vdev->v4l2_dev = v4l2_dev;
	vdev->vfl_dir = VFL_DIR_TX;
	snprintf(vdev->name, sizeof(vdev->name), "%s", BCM2835_ISP_PARAMS_NAME);

	params->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&vdev->entity, 1, &params->pad);
	if (ret) {
		vb2_queue_release(q);
		return ERR_PTR(ret);
	}

	video_set_drvdata(vdev, params);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, video_nr);
	if (ret) {
		dev_err(dev, "Failed to register params device node\n");
		media_entity_cleanup(&vdev->entity);
		vb2_queue_release(q);
		return ERR_PTR(ret);
	}

	v4l2_info(v4l2_dev, "Params device node registered as /dev/video%d\n",
		  vdev->num);

	return params;
}

void bcm2835_isp_params_unregister(struct bcm2835_isp_params *params)
{
	if (!video_is_registered(&params->vdev))
		return;

	v4l2_info(params->v4l2_dev, "Unregistering params device node\n");

	vb2_video_unregister_device(&params->vdev);
	media_entity_cleanup(&params->vdev.entity);
	mutex_destroy(&params->lock);
}

/*
 * The ISP component on the firmware has a reference to the dmabuf handle for
 * the lens shading table.
 * Pass a null handle to remove that reference.
 */
void bcm2835_isp_params_drop_ls_ref(struct bcm2835_isp_params *params)
{
	memset(&params->ls, 0, sizeof(params->ls));
	/* Must set a valid grid size for the FW */
	params->ls.grid_cell_size = 16;
	isp_set_param(params, MMAL_PARAMETER_LENS_SHADING_OVERRIDE,
		      &params->ls, sizeof(params->ls));
	params->last_ls_dmabuf = NULL;
}
