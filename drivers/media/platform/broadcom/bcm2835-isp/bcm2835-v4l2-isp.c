// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom BCM2835 ISP driver
 *
 * Copyright (c) 2019-2026 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2026 Ideas On Board Oy
 *
 * Author: Naushir Patuck (naush@raspberrypi.com)
 *
 */

#include <linux/module.h>
#include <linux/raspberrypi/mmal-msg.h>
#include <linux/raspberrypi/mmal-parameters.h>
#include <linux/raspberrypi/mmal-vchiq.h>
#include <linux/raspberrypi/vchiq_bus.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "bcm2835-isp-fmts.h"
#include "bcm2835-isp-common.h"

/*
 * We want to instantiate 2 independent instances allowing 2 simultaneous users
 * of the ISP hardware.
 */
#define BCM2835_ISP_NUM_INSTANCES 2

MODULE_IMPORT_NS("DMA_BUF");

static unsigned int debug;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "activates debug info");

static unsigned int video_nr[BCM2835_ISP_NUM_INSTANCES] = { 13, 20 };
module_param_array(video_nr, uint, NULL, 0644);
MODULE_PARM_DESC(video_nr, "base video device numbers");

#define BCM2835_ISP_NAME "bcm2835-isp"
#define BCM2835_ISP_ENTITY_NAME_LEN 32

#define BCM2835_ISP_NUM_OUTPUTS 1
#define BCM2835_ISP_NUM_CAPTURES 2
#define BCM2835_ISP_NUM_METADATA 1
#define BCM2835_ISP_NUM_PARAMS 1

#define BCM2835_ISP_NUM_NODES						\
		(BCM2835_ISP_NUM_OUTPUTS + BCM2835_ISP_NUM_CAPTURES +	\
		 BCM2835_ISP_NUM_METADATA)
#define BCM2835_ISP_PARAMS_PAD BCM2835_ISP_NUM_NODES
#define BCM2835_ISP_NUM_ENTITY_PADS					\
		(BCM2835_ISP_NUM_NODES + BCM2835_ISP_NUM_PARAMS)

/* Default frame dimension of 1280 pixels. */
#define DEFAULT_DIM 1280U
/*
 * Maximum frame dimension of 16384 pixels.  Even though the ISP runs in tiles,
 * have a sensible limit so that we do not create an excessive number of tiles
 * to process.
 */
#define MAX_DIM 16384U
/*
 * Minimum frame dimension of 64 pixels.  Anything lower, and the tiling
 * algorithm may not be able to cope when applying filter context.
 */
#define MIN_DIM 64U

/* Timeout for stop_streaming to allow all buffers to return */
#define COMPLETE_TIMEOUT (2 * HZ)

/* Per-queue, driver-specific private data */
struct bcm2835_isp_q_data {
	/*
	 * These parameters should be treated as gospel, with everything else
	 * being determined from them.
	 */
	unsigned int bytesperline;
	unsigned int width;
	unsigned int height;
	unsigned int sizeimage;
	enum v4l2_colorspace colorspace;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_xfer_func xfer_func;
	enum v4l2_quantization quantization;
	const struct bcm2835_isp_fmt *fmt;
};

/*
 * Structure to describe a single node /dev/video<N> which represents a single
 * input or output queue to the ISP device.
 */
struct bcm2835_isp_node {
	int vfl_dir;
	unsigned int id;
	const char *name;
	struct vchiq_mmal_port *port;
	struct video_device vfd;
	struct media_pad pad;
	struct mutex lock; /* top level device node lock */
	struct mutex queue_lock;

	struct vb2_queue queue;
	unsigned int sequence;

	/* The list of formats supported on the node. */
	struct bcm2835_isp_fmt const **supported_fmts;
	unsigned int num_supported_fmts;

	struct bcm2835_isp_q_data q_data;

	/* Parent device structure */
	struct bcm2835_isp_dev *dev;

	bool registered;
};

/*
 * Structure representing the entire ISP device, comprising several input and
 * output nodes /dev/video<N>.
 */
struct bcm2835_isp_dev {
	struct v4l2_device v4l2_dev;
	struct device *dev;
	struct media_device mdev;
	struct media_entity entity;
	bool media_device_registered;
	bool media_entity_registered;
	struct vchiq_mmal_instance *mmal_instance;
	struct vchiq_mmal_component *component;
	struct completion frame_cmplt;

	struct bcm2835_isp_node node[BCM2835_ISP_NUM_NODES];
	struct media_pad pad[BCM2835_ISP_NUM_ENTITY_PADS];
	atomic_t num_streaming;

	/* Extensible params node */
	struct bcm2835_isp_params *params;
};

struct bcm2835_isp_buffer {
	struct vb2_v4l2_buffer vb;
	struct mmal_buffer mmal;
};

#define to_bcm2835_isp_buffer(vbuf) \
	container_of(vbuf, struct bcm2835_isp_buffer, vb)

static
inline struct bcm2835_isp_dev *node_get_dev(struct bcm2835_isp_node *node)
{
	return node->dev;
}

static inline bool node_is_output(struct bcm2835_isp_node *node)
{
	return node->queue.type == V4L2_BUF_TYPE_VIDEO_OUTPUT;
}

static inline bool node_is_capture(struct bcm2835_isp_node *node)
{
	return node->queue.type == V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

static inline bool node_is_stats(struct bcm2835_isp_node *node)
{
	return node->queue.type == V4L2_BUF_TYPE_META_CAPTURE;
}

static inline enum v4l2_buf_type index_to_queue_type(int index)
{
	if (index < BCM2835_ISP_NUM_OUTPUTS)
		return V4L2_BUF_TYPE_VIDEO_OUTPUT;
	else if (index < BCM2835_ISP_NUM_OUTPUTS + BCM2835_ISP_NUM_CAPTURES)
		return V4L2_BUF_TYPE_VIDEO_CAPTURE;
	else
		return V4L2_BUF_TYPE_META_CAPTURE;
}

static const struct bcm2835_isp_fmt *get_fmt(u32 mmal_fmt)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_formats); i++) {
		if (supported_formats[i].mmal_fmt == mmal_fmt)
			return &supported_formats[i];
	}
	return NULL;
}

static const
struct bcm2835_isp_fmt *find_format_by_fourcc(unsigned int fourcc,
					      struct bcm2835_isp_node *node)
{
	const struct bcm2835_isp_fmt *fmt;
	unsigned int i;

	for (i = 0; i < node->num_supported_fmts; i++) {
		fmt = node->supported_fmts[i];
		if (fmt->fourcc == fourcc)
			return fmt;
	}

	return NULL;
}

static const
struct bcm2835_isp_fmt *find_format(struct v4l2_format *f,
				    struct bcm2835_isp_node *node)
{
	return find_format_by_fourcc(node_is_stats(node) ?
				     f->fmt.meta.dataformat :
				     f->fmt.pix.pixelformat,
				     node);
}

/* vb2_to_mmal_buffer() - converts vb2 buffer header to MMAL
 *
 * Copies all the required fields from a VB2 buffer to the MMAL buffer header,
 * ready for sending to the VPU.
 */
static void vb2_to_mmal_buffer(struct mmal_buffer *buf,
			       struct vb2_v4l2_buffer *vb2)
{
	u64 pts;

	buf->mmal_flags = 0;
	if (vb2->flags & V4L2_BUF_FLAG_KEYFRAME)
		buf->mmal_flags |= MMAL_BUFFER_HEADER_FLAG_KEYFRAME;

	/* Data must be framed correctly as one frame per buffer. */
	buf->mmal_flags |= MMAL_BUFFER_HEADER_FLAG_FRAME_END;

	buf->length = vb2->vb2_buf.planes[0].bytesused;
	/*
	 * Minor ambiguity in the V4L2 spec as to whether passing in a 0 length
	 * buffer, or one with V4L2_BUF_FLAG_LAST set denotes end of stream.
	 * Handle either.
	 */
	if (!buf->length || (vb2->flags & V4L2_BUF_FLAG_LAST))
		buf->mmal_flags |= MMAL_BUFFER_HEADER_FLAG_EOS;

	/* vb2 timestamps in nsecs, mmal in usecs */
	pts = vb2->vb2_buf.timestamp;
	do_div(pts, 1000);
	buf->pts = pts;
	buf->dts = MMAL_TIME_UNKNOWN;
}

static void mmal_buffer_cb(struct vchiq_mmal_instance *instance,
			   struct vchiq_mmal_port *port, int status,
			   struct mmal_buffer *mmal_buf)
{
	struct bcm2835_isp_buffer *q_buf;
	struct bcm2835_isp_node *node = port->cb_ctx;
	struct bcm2835_isp_dev *dev = node_get_dev(node);
	struct vb2_v4l2_buffer *vb2;

	q_buf = container_of(mmal_buf, struct bcm2835_isp_buffer, mmal);
	vb2 = &q_buf->vb;
	v4l2_dbg(2, debug, &dev->v4l2_dev,
		 "%s: port:%s[%d], status:%d, buf:%p, dmabuf:%p, length:%lu, flags %u, pts %lld\n",
		 __func__, node_is_output(node) ? "input" : "output", node->id,
		 status, mmal_buf, mmal_buf->dma_buf, mmal_buf->length,
		 mmal_buf->mmal_flags, mmal_buf->pts);

	if (status) {
		/* error in transfer */
		if (vb2) {
			/* there was a buffer with the error so return it */
			vb2_buffer_done(&vb2->vb2_buf, VB2_BUF_STATE_ERROR);
		}
		return;
	}

	/* vb2 timestamps in nsecs, mmal in usecs */
	vb2->vb2_buf.timestamp = mmal_buf->pts * 1000;
	vb2->sequence = node->sequence++;
	vb2_set_plane_payload(&vb2->vb2_buf, 0, mmal_buf->length);
	vb2_buffer_done(&vb2->vb2_buf, VB2_BUF_STATE_DONE);

	if (!port->enabled)
		complete(&dev->frame_cmplt);
}

struct colorspace_translation {
	enum v4l2_colorspace v4l2_value;
	u32 mmal_value;
};

static u32 translate_color_space(enum v4l2_colorspace color_space)
{
	static const struct colorspace_translation translations[] = {
		{ V4L2_COLORSPACE_DEFAULT, MMAL_COLOR_SPACE_UNKNOWN },
		{ V4L2_COLORSPACE_SMPTE170M, MMAL_COLOR_SPACE_ITUR_BT601 },
		{ V4L2_COLORSPACE_SMPTE240M, MMAL_COLOR_SPACE_SMPTE240M },
		{ V4L2_COLORSPACE_REC709, MMAL_COLOR_SPACE_ITUR_BT709 },
		/* V4L2_COLORSPACE_BT878 unavailable */
		{ V4L2_COLORSPACE_470_SYSTEM_M, MMAL_COLOR_SPACE_BT470_2_M },
		{ V4L2_COLORSPACE_470_SYSTEM_BG, MMAL_COLOR_SPACE_BT470_2_BG },
		{ V4L2_COLORSPACE_JPEG, MMAL_COLOR_SPACE_JPEG_JFIF },
		/*
		 * We don't have an encoding for SRGB as such, but VideoCore
		 * will do the right thing if it gets "unknown".
		 */
		{ V4L2_COLORSPACE_SRGB, MMAL_COLOR_SPACE_UNKNOWN },
		/* V4L2_COLORSPACE_OPRGB unavailable */
		/* V4L2_COLORSPACE_BT2020 unavailable */
		/* V4L2_COLORSPACE_RAW unavailable */
		/* V4L2_COLORSPACE_DCI_P3 unavailable */
	};

	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(translations); i++) {
		if (color_space == translations[i].v4l2_value)
			return translations[i].mmal_value;
	}

	return MMAL_COLOR_SPACE_UNKNOWN;
}

static void setup_mmal_port_format(struct bcm2835_isp_node *node,
				   struct vchiq_mmal_port *port)
{
	struct bcm2835_isp_q_data *q_data = &node->q_data;

	port->format.encoding = q_data->fmt->mmal_fmt;
	/* Raw image format - set width/height */
	port->es.video.width = (q_data->bytesperline << 3) / q_data->fmt->depth;
	port->es.video.height = q_data->height;
	port->es.video.crop.width = q_data->width;
	port->es.video.crop.height = q_data->height;
	port->es.video.crop.x = 0;
	port->es.video.crop.y = 0;
	port->es.video.color_space = translate_color_space(q_data->colorspace);
};

static int setup_mmal_port(struct bcm2835_isp_node *node)
{
	struct bcm2835_isp_dev *dev = node_get_dev(node);
	unsigned int enable = 1;
	int ret;

	v4l2_dbg(2, debug, &dev->v4l2_dev, "%s: setup %s[%d]\n", __func__,
		 node->name, node->id);

	vchiq_mmal_port_parameter_set(dev->mmal_instance, node->port,
				      MMAL_PARAMETER_ZERO_COPY, &enable,
				      sizeof(enable));
	setup_mmal_port_format(node, node->port);
	ret = vchiq_mmal_port_set_format(dev->mmal_instance, node->port);
	if (ret < 0) {
		v4l2_dbg(1, debug, &dev->v4l2_dev,
			 "%s: vchiq_mmal_port_set_format failed\n",
			 __func__);
		return ret;
	}

	if (node->q_data.sizeimage < node->port->minimum_buffer.size) {
		v4l2_err(&dev->v4l2_dev,
			 "buffer size mismatch sizeimage %u < min size %u\n",
			 node->q_data.sizeimage,
			 node->port->minimum_buffer.size);
		return -EINVAL;
	}

	return 0;
}

static int bcm2835_isp_mmal_buf_cleanup(struct mmal_buffer *mmal_buf)
{
	mmal_vchi_buffer_cleanup(mmal_buf);

	if (mmal_buf->dma_buf) {
		dma_buf_put(mmal_buf->dma_buf);
		mmal_buf->dma_buf = NULL;
	}

	return 0;
}

static int bcm2835_isp_node_queue_setup(struct vb2_queue *q,
					unsigned int *nbuffers,
					unsigned int *nplanes,
					unsigned int sizes[],
					struct device *alloc_devs[])
{
	struct bcm2835_isp_node *node = vb2_get_drv_priv(q);
	unsigned int size;

	if (setup_mmal_port(node))
		return -EINVAL;

	size = node->q_data.sizeimage;
	if (size == 0) {
		v4l2_info(&node_get_dev(node)->v4l2_dev,
			  "%s: Image size unset in queue_setup for node %s[%d]\n",
			  __func__, node->name, node->id);
		return -EINVAL;
	}

	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = size;

	node->port->current_buffer.size = size;

	if (*nbuffers < node->port->minimum_buffer.num)
		*nbuffers = node->port->minimum_buffer.num;

	node->port->current_buffer.num = *nbuffers;

	v4l2_dbg(2, debug, &node_get_dev(node)->v4l2_dev,
		 "%s: Image size %u, nbuffers %u for node %s[%d]\n",
		 __func__, sizes[0], *nbuffers, node->name, node->id);
	return 0;
}

static int bcm2835_isp_buf_init(struct vb2_buffer *vb)
{
	struct bcm2835_isp_node *node = vb2_get_drv_priv(vb->vb2_queue);
	struct bcm2835_isp_dev *dev = node_get_dev(node);
	struct bcm2835_isp_buffer *buf =
		to_bcm2835_isp_buffer(to_vb2_v4l2_buffer(vb));

	v4l2_dbg(3, debug, &dev->v4l2_dev, "%s: vb %p\n", __func__, vb);

	buf->mmal.buffer = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
	buf->mmal.buffer_size = vb2_plane_size(&buf->vb.vb2_buf, 0);
	mmal_vchi_buffer_init(dev->mmal_instance, &buf->mmal);
	return 0;
}

static int bcm2835_isp_buf_prepare(struct vb2_buffer *vb)
{
	struct bcm2835_isp_node *node = vb2_get_drv_priv(vb->vb2_queue);
	struct bcm2835_isp_dev *dev = node_get_dev(node);
	struct vb2_v4l2_buffer *vb2 = to_vb2_v4l2_buffer(vb);
	struct bcm2835_isp_buffer *buf = to_bcm2835_isp_buffer(vb2);
	struct dma_buf *dma_buf;
	int ret;

	v4l2_dbg(3, debug, &dev->v4l2_dev, "%s: type: %d ptr %p\n",
		 __func__, vb->vb2_queue->type, vb);

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (vb2->field == V4L2_FIELD_ANY)
			vb2->field = V4L2_FIELD_NONE;
		if (vb2->field != V4L2_FIELD_NONE) {
			v4l2_err(&dev->v4l2_dev,
				 "%s field isn't supported\n", __func__);
			return -EINVAL;
		}
	}

	if (vb2_plane_size(vb, 0) < node->q_data.sizeimage) {
		v4l2_err(&dev->v4l2_dev,
			 "%s data will not fit into plane (%lu < %u)\n",
			 __func__, vb2_plane_size(vb, 0),
			 node->q_data.sizeimage);
		return -EINVAL;
	}

	if (!V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		vb2_set_plane_payload(vb, 0, node->q_data.sizeimage);

	switch (vb->memory) {
	case VB2_MEMORY_DMABUF:
		dma_buf = dma_buf_get(vb->planes[0].m.fd);

		if (dma_buf != buf->mmal.dma_buf) {
			/*
			 * dmabuf either hasn't already been mapped, or it has
			 * changed.
			 */
			if (buf->mmal.dma_buf) {
				v4l2_err(&dev->v4l2_dev,
					 "%s Buffer changed - why did the core not call cleanup?\n",
					 __func__);
				bcm2835_isp_mmal_buf_cleanup(&buf->mmal);
			}

			buf->mmal.dma_buf = dma_buf;
		} else {
			/*
			 * Already have a reference to the buffer, so release it
			 * here.
			 */
			dma_buf_put(dma_buf);
		}
		ret = 0;
		break;
	case VB2_MEMORY_MMAP:
		/*
		 * We want to do this at init, but vb2_core_expbuf checks that
		 * the index < q->num_buffers, and q->num_buffers only gets
		 * updated once all the buffers are allocated.
		 */
		if (!buf->mmal.dma_buf) {
			ret = vb2_core_expbuf_dmabuf(vb->vb2_queue,
						     vb->vb2_queue->type,
						     vb, 0, O_CLOEXEC,
						     &buf->mmal.dma_buf);
			v4l2_dbg(3, debug, &dev->v4l2_dev,
				 "%s: exporting ptr %p to dmabuf %p\n",
				 __func__, vb, buf->mmal.dma_buf);
			if (ret)
				v4l2_err(&dev->v4l2_dev,
					 "%s: Failed to expbuf idx %d, ret %d\n",
					 __func__, vb->index, ret);
		} else {
			ret = 0;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void bcm2835_isp_node_buffer_queue(struct vb2_buffer *buf)
{
	struct bcm2835_isp_node *node = vb2_get_drv_priv(buf->vb2_queue);
	struct bcm2835_isp_buffer *buffer =
		to_bcm2835_isp_buffer(to_vb2_v4l2_buffer(buf));
	struct bcm2835_isp_dev *dev = node_get_dev(node);

	v4l2_dbg(3, debug, &dev->v4l2_dev, "%s: node %s[%d], buffer %p\n",
		 __func__, node->name, node->id, buffer);

	vb2_to_mmal_buffer(&buffer->mmal, &buffer->vb);
	v4l2_dbg(3, debug, &dev->v4l2_dev,
		 "%s: node %s[%d] - submitting  mmal dmabuf %p\n", __func__,
		 node->name, node->id, buffer->mmal.dma_buf);
	vchiq_mmal_submit_buffer(dev->mmal_instance, node->port, &buffer->mmal);
}

static void bcm2835_isp_buffer_cleanup(struct vb2_buffer *vb)
{
	struct bcm2835_isp_buffer *buffer =
		to_bcm2835_isp_buffer(to_vb2_v4l2_buffer(vb));

	bcm2835_isp_mmal_buf_cleanup(&buffer->mmal);
}

static int bcm2835_isp_node_start_streaming(struct vb2_queue *q,
					    unsigned int count)
{
	struct bcm2835_isp_node *node = vb2_get_drv_priv(q);
	struct bcm2835_isp_dev *dev = node_get_dev(node);
	int ret;

	v4l2_dbg(1, debug, &dev->v4l2_dev, "%s: node %s[%d] (count %u)\n",
		 __func__, node->name, node->id, count);

	ret = vchiq_mmal_component_enable(dev->mmal_instance, dev->component);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "%s: Failed enabling component, ret %d\n",
			 __func__, ret);
		return -EIO;
	}

	node->sequence = 0;
	node->port->cb_ctx = node;
	ret = vchiq_mmal_port_enable(dev->mmal_instance, node->port,
				     mmal_buffer_cb);
	if (!ret)
		atomic_inc(&dev->num_streaming);
	else
		v4l2_err(&dev->v4l2_dev,
			 "%s: Failed enabling port, ret %d\n", __func__, ret);

	return ret;
}

static void bcm2835_isp_node_stop_streaming(struct vb2_queue *q)
{
	struct bcm2835_isp_node *node = vb2_get_drv_priv(q);
	struct bcm2835_isp_dev *dev = node_get_dev(node);
	int ret;

	v4l2_dbg(1, debug, &dev->v4l2_dev, "%s: node %s[%d], mmal port %p\n",
		 __func__, node->name, node->id, node->port);

	init_completion(&dev->frame_cmplt);

	/* Disable MMAL port - this will flush buffers back */
	ret = vchiq_mmal_port_disable(dev->mmal_instance, node->port);
	if (ret)
		v4l2_err(&dev->v4l2_dev,
			 "%s: Failed disabling %s port, ret %d\n", __func__,
			 node_is_output(node) ? "i/p" : "o/p",
			 ret);

	while (atomic_read(&node->port->buffers_with_vpu)) {
		v4l2_dbg(1, debug, &dev->v4l2_dev,
			 "%s: Waiting for buffers to be returned - %d outstanding\n",
			 __func__, atomic_read(&node->port->buffers_with_vpu));
		ret = wait_for_completion_timeout(&dev->frame_cmplt,
						  COMPLETE_TIMEOUT);
		if (ret <= 0) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: Timeout waiting for buffers to be returned - %d outstanding\n",
				 __func__,
				 atomic_read(&node->port->buffers_with_vpu));
			break;
		}
	}

	atomic_dec(&dev->num_streaming);
	/* If all ports disabled, then disable the component */
	if (atomic_read(&dev->num_streaming) == 0) {
		bcm2835_isp_params_drop_ls_ref(dev->params);
		ret = vchiq_mmal_component_disable(dev->mmal_instance,
						   dev->component);
		if (ret) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: Failed disabling component, ret %d\n",
				 __func__, ret);
		}
	}

	/*
	 * Simply wait for any vb2 buffers to finish. We could take steps to
	 * make them complete more quickly if we care, or even return them
	 * ourselves.
	 */
	vb2_wait_for_all_buffers(&node->queue);
}

static const struct vb2_ops bcm2835_isp_node_queue_ops = {
	.queue_setup		= bcm2835_isp_node_queue_setup,
	.buf_init		= bcm2835_isp_buf_init,
	.buf_prepare		= bcm2835_isp_buf_prepare,
	.buf_queue		= bcm2835_isp_node_buffer_queue,
	.buf_cleanup		= bcm2835_isp_buffer_cleanup,
	.start_streaming	= bcm2835_isp_node_start_streaming,
	.stop_streaming		= bcm2835_isp_node_stop_streaming,
};

static const
struct bcm2835_isp_fmt *get_default_format(struct bcm2835_isp_node *node)
{
	return node->supported_fmts[0];
}

static inline unsigned int get_bytesperline(int width,
					    const struct bcm2835_isp_fmt *fmt)
{
	/* GPU aligns 24bpp images to a multiple of 32 pixels (not bytes). */
	if (fmt->depth == 24)
		return ALIGN(width, 32) * 3;
	else
		return ALIGN((width * fmt->depth) >> 3, fmt->bytesperline_align);
}

static inline unsigned int get_sizeimage(int bpl, int width, int height,
					 const struct bcm2835_isp_fmt *fmt)
{
	return (bpl * height * fmt->size_multiplier_x2) >> 1;
}

static const struct v4l2_file_operations bcm2835_isp_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap		= vb2_fop_mmap
};

static int populate_qdata_fmt(struct v4l2_format *f,
			      struct bcm2835_isp_node *node)
{
	struct bcm2835_isp_dev *dev = node_get_dev(node);
	struct bcm2835_isp_q_data *q_data = &node->q_data;
	int ret;

	if (!node_is_stats(node)) {
		v4l2_dbg(1, debug, &dev->v4l2_dev,
			 "%s: Setting pix format for type %d, wxh: %ux%u, fmt: %08x, size %u\n",
			 __func__, f->type, f->fmt.pix.width, f->fmt.pix.height,
			 f->fmt.pix.pixelformat, f->fmt.pix.sizeimage);

		q_data->fmt = find_format(f, node);
		q_data->width = f->fmt.pix.width;
		q_data->height = f->fmt.pix.height;

		/* All parameters should have been set correctly by try_fmt */
		q_data->bytesperline = f->fmt.pix.bytesperline;
		q_data->sizeimage = f->fmt.pix.sizeimage;
		q_data->ycbcr_enc = f->fmt.pix.ycbcr_enc;
		q_data->xfer_func = f->fmt.pix.xfer_func;
		q_data->quantization = f->fmt.pix.quantization;

		/* We must indicate which of the allowed colour spaces we have. */
		q_data->colorspace = f->fmt.pix.colorspace;
	} else {
		v4l2_dbg(1, debug, &dev->v4l2_dev,
			 "%s: Setting meta format for fmt: %08x, size %u\n",
			 __func__, f->fmt.meta.dataformat,
			 f->fmt.meta.buffersize);

		q_data->fmt = find_format(f, node);
		q_data->width = 0;
		q_data->height = 0;
		q_data->bytesperline = 0;
		q_data->sizeimage = f->fmt.meta.buffersize;

		/* This won't mean anything for metadata, but may as well fill it in. */
		q_data->colorspace = V4L2_COLORSPACE_DEFAULT;
	}

	v4l2_dbg(1, debug, &dev->v4l2_dev,
		 "%s: Calculated bpl as %u, size %u\n", __func__,
		 q_data->bytesperline, q_data->sizeimage);

	setup_mmal_port_format(node, node->port);
	ret = vchiq_mmal_port_set_format(dev->mmal_instance, node->port);
	if (ret) {
		v4l2_err(&dev->v4l2_dev,
			 "%s: Failed vchiq_mmal_port_set_format on port, ret %d\n",
			 __func__, ret);
		ret = -EINVAL;
	}

	if (q_data->sizeimage < node->port->minimum_buffer.size) {
		v4l2_err(&dev->v4l2_dev,
			 "%s: Current buffer size of %u < min buf size %u - driver mismatch to MMAL\n",
			 __func__,
			 q_data->sizeimage,
			 node->port->minimum_buffer.size);
	}

	v4l2_dbg(1, debug, &dev->v4l2_dev,
		 "%s: Set format for type %d, wxh: %dx%d, fmt: %08x, size %u\n",
		 __func__, f->type, q_data->width, q_data->height,
		 q_data->fmt->fourcc, q_data->sizeimage);

	return ret;
}

int bcm2835_isp_node_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap)
{
	strscpy(cap->driver, BCM2835_ISP_NAME, sizeof(cap->driver));
	strscpy(cap->card, BCM2835_ISP_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 BCM2835_ISP_NAME);

	return 0;
}

static int bcm2835_isp_node_g_fmt(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct bcm2835_isp_node *node = video_drvdata(file);

	if (f->type != node->queue.type)
		return -EINVAL;

	if (node_is_stats(node)) {
		f->fmt.meta.dataformat = V4L2_META_FMT_BCM2835_ISP_STATS;
		f->fmt.meta.buffersize =
			node->port->minimum_buffer.size;
	} else {
		struct bcm2835_isp_q_data *q_data = &node->q_data;

		f->fmt.pix.width = q_data->width;
		f->fmt.pix.height = q_data->height;
		f->fmt.pix.field = V4L2_FIELD_NONE;
		f->fmt.pix.pixelformat = q_data->fmt->fourcc;
		f->fmt.pix.bytesperline = q_data->bytesperline;
		f->fmt.pix.sizeimage = q_data->sizeimage;
		f->fmt.pix.colorspace = q_data->colorspace;
		f->fmt.pix.ycbcr_enc = q_data->ycbcr_enc;
		f->fmt.pix.xfer_func = q_data->xfer_func;
		f->fmt.pix.quantization = q_data->quantization;
	}

	return 0;
}

static int bcm2835_isp_node_enum_fmt(struct file *file, void  *priv,
				     struct v4l2_fmtdesc *f)
{
	struct bcm2835_isp_node *node = video_drvdata(file);

	if (f->type != node->queue.type)
		return -EINVAL;

	if (f->index < node->num_supported_fmts) {
		/* Format found */
		f->pixelformat = node->supported_fmts[f->index]->fourcc;
		f->flags = 0;
		return 0;
	}

	return -EINVAL;
}

static int bcm2835_isp_enum_framesizes(struct file *file, void *priv,
				       struct v4l2_frmsizeenum *fsize)
{
	struct bcm2835_isp_node *node = video_drvdata(file);
	struct bcm2835_isp_dev *dev = node_get_dev(node);
	const struct bcm2835_isp_fmt *fmt;

	if (node_is_stats(node) || fsize->index)
		return -EINVAL;

	fmt = find_format_by_fourcc(fsize->pixel_format, node);
	if (!fmt) {
		v4l2_err(&dev->v4l2_dev, "Invalid pixel code: %x\n",
			 fsize->pixel_format);
		return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = MIN_DIM;
	fsize->stepwise.max_width = MAX_DIM;
	fsize->stepwise.step_width = fmt->step_size;

	fsize->stepwise.min_height = MIN_DIM;
	fsize->stepwise.max_height = MAX_DIM;
	fsize->stepwise.step_height = fmt->step_size;

	return 0;
}

static int bcm2835_isp_node_try_fmt(struct file *file, void *priv,
				    struct v4l2_format *f)
{
	struct bcm2835_isp_node *node = video_drvdata(file);
	const struct bcm2835_isp_fmt *fmt;

	if (f->type != node->queue.type)
		return -EINVAL;

	fmt = find_format(f, node);
	if (!fmt)
		fmt = get_default_format(node);

	if (!node_is_stats(node)) {
		bool is_rgb;

		f->fmt.pix.width = max(min(f->fmt.pix.width, MAX_DIM),
				       MIN_DIM);
		f->fmt.pix.height = max(min(f->fmt.pix.height, MAX_DIM),
					MIN_DIM);

		f->fmt.pix.pixelformat = fmt->fourcc;

		/*
		 * Fill in the actual colour space when the requested one was
		 * not supported. This also catches the case when the "default"
		 * colour space was requested (as that's never in the mask).
		 */
		if (!(V4L2_COLORSPACE_MASK(f->fmt.pix.colorspace) & fmt->colorspace_mask))
			f->fmt.pix.colorspace = fmt->colorspace_default;
		/* In all cases, we only support the defaults for these: */
		f->fmt.pix.ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(f->fmt.pix.colorspace);
		f->fmt.pix.xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(f->fmt.pix.colorspace);
		/* RAW counts as sRGB here so that we get full range. */
		is_rgb = f->fmt.pix.colorspace == V4L2_COLORSPACE_SRGB ||
			f->fmt.pix.colorspace == V4L2_COLORSPACE_RAW;
		f->fmt.pix.quantization = V4L2_MAP_QUANTIZATION_DEFAULT(is_rgb,
									f->fmt.pix.colorspace,
									f->fmt.pix.ycbcr_enc);

		/* Respect any stride value (suitably aligned) that was requested. */
		f->fmt.pix.bytesperline = max(get_bytesperline(f->fmt.pix.width, fmt),
					      ALIGN(f->fmt.pix.bytesperline,
						    fmt->bytesperline_align));
		f->fmt.pix.field = V4L2_FIELD_NONE;
		f->fmt.pix.sizeimage =
			get_sizeimage(f->fmt.pix.bytesperline, f->fmt.pix.width,
				      f->fmt.pix.height, fmt);
	} else {
		f->fmt.meta.dataformat = fmt->fourcc;
		f->fmt.meta.buffersize = node->port->minimum_buffer.size;
	}

	return 0;
}

static int bcm2835_isp_node_s_fmt(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct bcm2835_isp_node *node = video_drvdata(file);
	int ret;

	if (f->type != node->queue.type)
		return -EINVAL;

	ret = bcm2835_isp_node_try_fmt(file, priv, f);
	if (ret)
		return ret;

	v4l2_dbg(1, debug, &node_get_dev(node)->v4l2_dev,
		 "%s: Set format for node %s[%d]\n",
		 __func__, node->name, node->id);

	return populate_qdata_fmt(f, node);
}

static int bcm2835_isp_node_s_selection(struct file *file, void *fh,
					struct v4l2_selection *s)
{
	struct mmal_parameter_crop crop;
	struct bcm2835_isp_node *node = video_drvdata(file);
	struct bcm2835_isp_dev *dev = node_get_dev(node);

	if (!node_is_output(node))
		return -EINVAL;

	if (!s->r.width || !s->r.height)
		return -EINVAL;

	/* We can only set crop on the input. */
	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		/*
		 * Adjust the crop window if it goes outside of the frame
		 * dimensions.
		 */
		s->r.left = clamp_t(unsigned int, s->r.left, 0,
				    node->q_data.width - MIN_DIM);
		s->r.top = clamp_t(unsigned int, s->r.top, 0,
				   node->q_data.height - MIN_DIM);
		s->r.width = clamp_t(unsigned int, s->r.width, MIN_DIM,
				     node->q_data.width - s->r.left);
		s->r.height = clamp_t(unsigned int, s->r.height, MIN_DIM,
				      node->q_data.height - s->r.top);
		break;
	default:
		return -EINVAL;
	}

	crop.rect.x = s->r.left;
	crop.rect.y = s->r.top;
	crop.rect.width = s->r.width;
	crop.rect.height = s->r.height;

	return vchiq_mmal_port_parameter_set(dev->mmal_instance, node->port,
					     MMAL_PARAMETER_CROP,
					     &crop, sizeof(crop));
}

static int bcm2835_isp_node_g_selection(struct file *file, void *fh,
					struct v4l2_selection *s)
{
	struct mmal_parameter_crop crop;
	struct bcm2835_isp_node *node = video_drvdata(file);
	struct bcm2835_isp_dev *dev = node_get_dev(node);
	u32 crop_size = sizeof(crop);
	int ret;

	if (!node_is_output(node))
		return -EINVAL;

	/* We can only return out an input crop. */
	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		ret = vchiq_mmal_port_parameter_get(dev->mmal_instance,
						    node->port,
						    MMAL_PARAMETER_CROP,
						    &crop, &crop_size);
		if (!ret) {
			s->r.left = crop.rect.x;
			s->r.top = crop.rect.y;
			s->r.width = crop.rect.width;
			s->r.height = crop.rect.height;
		}
		break;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		/* Default (i.e. no) crop window. */
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = node->q_data.width;
		s->r.height = node->q_data.height;
		ret = 0;
		break;
	default:
		ret =  -EINVAL;
	}

	return ret;
}

static const struct v4l2_ioctl_ops bcm2835_isp_node_ioctl_ops = {
	.vidioc_querycap		= bcm2835_isp_node_querycap,
	.vidioc_g_fmt_vid_cap		= bcm2835_isp_node_g_fmt,
	.vidioc_g_fmt_vid_out		= bcm2835_isp_node_g_fmt,
	.vidioc_g_fmt_meta_cap		= bcm2835_isp_node_g_fmt,
	.vidioc_s_fmt_vid_cap		= bcm2835_isp_node_s_fmt,
	.vidioc_s_fmt_vid_out		= bcm2835_isp_node_s_fmt,
	.vidioc_s_fmt_meta_cap		= bcm2835_isp_node_s_fmt,
	.vidioc_try_fmt_vid_cap		= bcm2835_isp_node_try_fmt,
	.vidioc_try_fmt_vid_out		= bcm2835_isp_node_try_fmt,
	.vidioc_try_fmt_meta_cap	= bcm2835_isp_node_try_fmt,
	.vidioc_s_selection		= bcm2835_isp_node_s_selection,
	.vidioc_g_selection		= bcm2835_isp_node_g_selection,

	.vidioc_enum_fmt_vid_cap	= bcm2835_isp_node_enum_fmt,
	.vidioc_enum_fmt_vid_out	= bcm2835_isp_node_enum_fmt,
	.vidioc_enum_fmt_meta_cap	= bcm2835_isp_node_enum_fmt,
	.vidioc_enum_framesizes		= bcm2835_isp_enum_framesizes,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,

	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
};

/*
 * Size of the array to provide to the VPU when asking for the list of supported
 * formats.
 *
 * The ISP component currently advertises 62 input formats, so add a small
 * overhead on that. Should the component advertise more formats then the excess
 * will be dropped and a warning logged.
 */
#define MAX_SUPPORTED_ENCODINGS 70

/* Populate node->supported_fmts with the formats supported by those ports. */
static int bcm2835_isp_get_supported_fmts(struct bcm2835_isp_node *node)
{
	struct bcm2835_isp_dev *dev = node_get_dev(node);
	struct bcm2835_isp_fmt const **list;
	unsigned int i, j, num_encodings;
	u32 fourccs[MAX_SUPPORTED_ENCODINGS];
	u32 param_size = sizeof(fourccs);
	int ret;

	ret = vchiq_mmal_port_parameter_get(dev->mmal_instance, node->port,
					    MMAL_PARAMETER_SUPPORTED_ENCODINGS,
					    &fourccs, &param_size);

	if (ret) {
		if (ret == MMAL_MSG_STATUS_ENOSPC) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: port has more encodings than we provided space for. Some are dropped (%zu vs %u).\n",
				 __func__, param_size / sizeof(u32),
				 MAX_SUPPORTED_ENCODINGS);
			num_encodings = MAX_SUPPORTED_ENCODINGS;
		} else {
			v4l2_err(&dev->v4l2_dev, "%s: get_param ret %u.\n",
				 __func__, ret);
			return -EINVAL;
		}
	} else {
		num_encodings = param_size / sizeof(u32);
	}

	/*
	 * Assume at this stage that all encodings will be supported in V4L2.
	 * Any that aren't supported will waste a very small amount of memory.
	 */
	list = devm_kzalloc(dev->dev,
			    sizeof(struct bcm2835_isp_fmt *) * num_encodings,
			    GFP_KERNEL);
	if (!list)
		return -ENOMEM;
	node->supported_fmts = list;

	for (i = 0, j = 0; i < num_encodings; i++) {
		const struct bcm2835_isp_fmt *fmt = get_fmt(fourccs[i]);

		if (fmt) {
			list[j] = fmt;
			j++;
		}
	}
	node->num_supported_fmts = j;

	return 0;
}

/*
 * Register a device node /dev/video<N> to go along with one of the ISP's input
 * or output nodes.
 */
static int bcm2835_isp_register_node(struct bcm2835_isp_dev *dev,
				     unsigned int instance,
				     struct bcm2835_isp_node *node, int index)
{
	struct video_device *vfd;
	struct vb2_queue *queue;
	bool is_rgb;
	int ret;

	mutex_init(&node->lock);
	mutex_init(&node->queue_lock);

	node->dev = dev;
	vfd = &node->vfd;
	queue = &node->queue;
	queue->type = index_to_queue_type(index);
	/*
	 * Setup the node type-specific params.
	 */
	switch (queue->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		vfd->device_caps = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;
		node->id = index;
		node->vfl_dir = VFL_DIR_TX;
		node->name = "output";
		node->port = &dev->component->input[node->id];
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		vfd->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
		/* First Capture node starts at id 0, etc. */
		node->id = index - BCM2835_ISP_NUM_OUTPUTS;
		node->vfl_dir = VFL_DIR_RX;
		node->name = "capture";
		node->port = &dev->component->output[node->id];
		break;
	case V4L2_BUF_TYPE_META_CAPTURE:
		vfd->device_caps = V4L2_CAP_META_CAPTURE | V4L2_CAP_STREAMING;
		node->id = index - BCM2835_ISP_NUM_OUTPUTS;
		node->vfl_dir = VFL_DIR_RX;
		node->name = "stats";
		node->port = &dev->component->output[node->id];
		break;
	}

	ret = bcm2835_isp_get_supported_fmts(node);
	if (ret)
		return ret;

	/* Initialise the video node. */
	vfd->vfl_type	= VFL_TYPE_VIDEO;
	vfd->fops	= &bcm2835_isp_fops;
	vfd->ioctl_ops	= &bcm2835_isp_node_ioctl_ops;
	vfd->minor	= -1;
	vfd->release	= video_device_release_empty;
	vfd->queue	= &node->queue;
	vfd->lock	= &node->lock;
	vfd->v4l2_dev	= &dev->v4l2_dev;
	vfd->vfl_dir	= node->vfl_dir;

	node->q_data.fmt = get_default_format(node);
	node->q_data.width = DEFAULT_DIM;
	node->q_data.height = DEFAULT_DIM;
	node->q_data.bytesperline =
		get_bytesperline(DEFAULT_DIM, node->q_data.fmt);
	node->q_data.sizeimage = node_is_stats(node) ?
				 node->port->recommended_buffer.size :
				 get_sizeimage(node->q_data.bytesperline,
					       node->q_data.width,
					       node->q_data.height,
					       node->q_data.fmt);
	node->q_data.colorspace = node->q_data.fmt->colorspace_default;
	node->q_data.ycbcr_enc =
		V4L2_MAP_YCBCR_ENC_DEFAULT(node->q_data.colorspace);
	node->q_data.xfer_func =
		V4L2_MAP_XFER_FUNC_DEFAULT(node->q_data.colorspace);
	is_rgb = node->q_data.colorspace == V4L2_COLORSPACE_SRGB ||
		node->q_data.colorspace == V4L2_COLORSPACE_RAW;
	node->q_data.quantization = V4L2_MAP_QUANTIZATION_DEFAULT(is_rgb,
								  node->q_data.colorspace,
								  node->q_data.ycbcr_enc);

	queue->io_modes = VB2_MMAP | VB2_DMABUF;
	queue->drv_priv = node;
	queue->ops = &bcm2835_isp_node_queue_ops;
	queue->mem_ops = &vb2_dma_contig_memops;
	queue->buf_struct_size = sizeof(struct bcm2835_isp_buffer);
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	queue->dev = dev->dev;
	queue->lock = &node->queue_lock;

	ret = vb2_queue_init(queue);
	if (ret < 0) {
		v4l2_info(&dev->v4l2_dev, "vb2_queue_init failed\n");
		return ret;
	}

	/* Define the device names */
	snprintf(vfd->name, sizeof(node->vfd.name), "%s-%s%d", BCM2835_ISP_NAME,
		 node->name, node->id);

	node->pad.flags = node_is_output(node) ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&node->vfd.entity, 1, &node->pad);
	if (ret)
		goto queue_cleanup;

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, video_nr[instance]);
	if (ret) {
		v4l2_err(&dev->v4l2_dev,
			 "Failed to register video %s[%d] device node\n",
			 node->name, node->id);
		goto queue_cleanup;
	}

	node->registered = true;
	video_set_drvdata(vfd, node);

	v4l2_info(&dev->v4l2_dev,
		  "Device node %s[%d] registered as /dev/video%d\n",
		  node->name, node->id, vfd->num);

	return 0;

queue_cleanup:
	vb2_queue_release(&node->queue);
	return ret;
}

/* Unregister one of the /dev/video<N> nodes associated with the ISP. */
static void bcm2835_unregister_node(struct bcm2835_isp_node *node)
{
	struct bcm2835_isp_dev *dev = node_get_dev(node);

	v4l2_info(&dev->v4l2_dev,
		  "Unregistering node %s[%d] device node /dev/video%d\n",
		  node->name, node->id, node->vfd.num);

	if (node->registered) {
		video_unregister_device(&node->vfd);
		vb2_queue_release(&node->queue);
	}

	/*
	 * node->supported_fmts.list is free'd automatically
	 * as a managed resource.
	 */
	node->supported_fmts = NULL;
	node->num_supported_fmts = 0;
	node->registered = false;
}

static void media_controller_unregister(struct bcm2835_isp_dev *dev)
{
	v4l2_info(&dev->v4l2_dev, "Unregister from media controller\n");

	if (dev->media_device_registered) {
		media_device_unregister(&dev->mdev);
		media_device_cleanup(&dev->mdev);
		dev->media_device_registered = false;
	}

	kfree(dev->entity.name);
	dev->entity.name = NULL;

	if (dev->media_entity_registered) {
		media_device_unregister_entity(&dev->entity);
		dev->media_entity_registered = false;
	}

	dev->v4l2_dev.mdev = NULL;
}

static int media_controller_register(struct bcm2835_isp_dev *dev)
{
	struct media_entity *entity;
	char *name;
	unsigned int i;
	int ret;

	v4l2_dbg(2, debug, &dev->v4l2_dev, "Registering with media controller\n");

	name = kmalloc(BCM2835_ISP_ENTITY_NAME_LEN, GFP_KERNEL);
	if (!name) {
		ret = -ENOMEM;
		goto done;
	}
	snprintf(name, BCM2835_ISP_ENTITY_NAME_LEN, "bcm2835-isp");
	dev->entity.name = name;
	dev->entity.obj_type = MEDIA_ENTITY_TYPE_BASE;
	dev->entity.function = MEDIA_ENT_F_PROC_VIDEO_ISP;

	for (i = 0; i < BCM2835_ISP_NUM_NODES; i++) {
		dev->pad[i].flags = node_is_output(&dev->node[i]) ?
					MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;
	}
	dev->pad[BCM2835_ISP_PARAMS_PAD].flags = MEDIA_PAD_FL_SINK;

	ret = media_entity_pads_init(&dev->entity, BCM2835_ISP_NUM_ENTITY_PADS,
				     dev->pad);
	if (ret)
		goto done;

	ret = media_device_register_entity(&dev->mdev, &dev->entity);
	if (ret)
		goto done;

	dev->media_entity_registered = true;

	for (i = 0; i < BCM2835_ISP_NUM_NODES; i++) {
		entity = &dev->node[i].vfd.entity;
		int output = node_is_output(&dev->node[i]);

		if (output)
			ret = media_create_pad_link(entity, 0,
						    &dev->entity, i,
						    MEDIA_LNK_FL_IMMUTABLE |
						    MEDIA_LNK_FL_ENABLED);
		else
			ret = media_create_pad_link(&dev->entity, i,
						    entity, 0,
						    MEDIA_LNK_FL_IMMUTABLE |
						    MEDIA_LNK_FL_ENABLED);
		if (ret)
			goto done;
	}

	entity = &dev->params->vdev.entity;
	ret = media_create_pad_link(entity, 0, &dev->entity, BCM2835_ISP_PARAMS_PAD,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
	if (ret)
		goto done;

	ret = media_device_register(&dev->mdev);
	if (!ret)
		dev->media_device_registered = true;
done:
	return ret;
}

static void bcm2835_isp_remove_instance(struct bcm2835_isp_dev *dev)
{
	unsigned int i;

	bcm2835_isp_params_unregister(dev->params);

	for (i = 0; i < BCM2835_ISP_NUM_NODES; i++)
		bcm2835_unregister_node(&dev->node[i]);

	v4l2_device_unregister(&dev->v4l2_dev);
	media_controller_unregister(dev);

	if (dev->component)
		vchiq_mmal_component_finalise(dev->mmal_instance,
					      dev->component);

	vchiq_mmal_finalise(dev->mmal_instance);
}

static int bcm2835_isp_probe_instance(struct vchiq_device *device,
				      struct bcm2835_isp_dev **dev_int,
				      unsigned int instance)
{
	struct bcm2835_isp_dev *dev;
	unsigned int i;
	int ret;

	dev = devm_kzalloc(&device->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	*dev_int = dev;
	dev->dev = &device->dev;
	dev->mdev.dev = &device->dev;

	strscpy(dev->mdev.model, BCM2835_ISP_NAME, sizeof(dev->mdev.model));
	snprintf(dev->mdev.bus_info, sizeof(dev->mdev.bus_info), "platform:%s",
		 BCM2835_ISP_NAME);
	media_device_init(&dev->mdev);

	dev->v4l2_dev.mdev = &dev->mdev;

	ret = v4l2_device_register(&device->dev, &dev->v4l2_dev);
	if (ret)
		return ret;

	ret = vchiq_mmal_init(&device->dev, &dev->mmal_instance);
	if (ret) {
		v4l2_device_unregister(&dev->v4l2_dev);
		return ret;
	}

	ret = vchiq_mmal_component_init(dev->mmal_instance, "ril.isp",
					&dev->component);
	if (ret) {
		v4l2_err(&dev->v4l2_dev,
			 "%s: failed to create ril.isp component\n", __func__);
		return ret;
	}

	if (dev->component->inputs < BCM2835_ISP_NUM_OUTPUTS ||
	    dev->component->outputs < BCM2835_ISP_NUM_CAPTURES +
					BCM2835_ISP_NUM_METADATA) {
		v4l2_err(&dev->v4l2_dev,
			 "%s: ril.isp returned %d i/p (%d expected), %d o/p (%d expected) ports\n",
			  __func__, dev->component->inputs,
			  BCM2835_ISP_NUM_OUTPUTS,
			  dev->component->outputs,
			  BCM2835_ISP_NUM_CAPTURES + BCM2835_ISP_NUM_METADATA);
		return -EINVAL;
	}

	atomic_set(&dev->num_streaming, 0);

	for (i = 0; i < BCM2835_ISP_NUM_NODES; i++) {
		struct bcm2835_isp_node *node = &dev->node[i];

		ret = bcm2835_isp_register_node(dev, instance, node, i);
		if (ret)
			return ret;
	}

	/* Register extensible params node */
	dev->params = bcm2835_isp_params_register(&dev->v4l2_dev, dev->dev,
						  dev->mmal_instance,
						  &dev->component->input[0],
						  video_nr[instance] + BCM2835_ISP_NUM_NODES);
	if (IS_ERR(dev->params))
		return PTR_ERR(dev->params);

	ret = media_controller_register(dev);
	if (ret)
		return ret;

	return 0;
}

static void bcm2835_isp_remove(struct vchiq_device *device)
{
	struct bcm2835_isp_dev **bcm2835_isp_instances;
	unsigned int i;

	bcm2835_isp_instances = vchiq_get_drvdata(device);
	for (i = 0; i < BCM2835_ISP_NUM_INSTANCES; i++) {
		if (bcm2835_isp_instances[i])
			bcm2835_isp_remove_instance(bcm2835_isp_instances[i]);
	}
}

static int bcm2835_isp_probe(struct vchiq_device *device)
{
	struct bcm2835_isp_dev **bcm2835_isp_instances;
	unsigned int i;
	int ret;

	ret = dma_set_mask_and_coherent(&device->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&device->dev, "dma_set_mask_and_coherent failed: %d\n",
			ret);
		return ret;
	}

	bcm2835_isp_instances = devm_kzalloc(&device->dev,
					     sizeof(bcm2835_isp_instances) *
						      BCM2835_ISP_NUM_INSTANCES,
					     GFP_KERNEL);
	if (!bcm2835_isp_instances)
		return -ENOMEM;

	vchiq_set_drvdata(device, bcm2835_isp_instances);

	for (i = 0; i < BCM2835_ISP_NUM_INSTANCES; i++) {
		ret = bcm2835_isp_probe_instance(device,
						 &bcm2835_isp_instances[i], i);
		if (ret)
			goto error;
	}

	dev_info(&device->dev, "Loaded V4L2 %s\n", BCM2835_ISP_NAME);
	return 0;

error:
	bcm2835_isp_remove(device);

	return ret;
}

static struct vchiq_device_id device_id_table[] = {
	{ .name = "bcm2835-isp" },
	{}
};
MODULE_DEVICE_TABLE(vchiq, device_id_table);

static struct vchiq_driver bcm2835_isp_drv = {
	.probe = bcm2835_isp_probe,
	.remove = bcm2835_isp_remove,
	.id_table = device_id_table,
	.driver = {
		.name = BCM2835_ISP_NAME,
	},
};

module_vchiq_driver(bcm2835_isp_drv);

MODULE_DESCRIPTION("BCM2835 ISP driver");
MODULE_AUTHOR("Naushir Patuck <naush@raspberrypi.com>");
MODULE_LICENSE("GPL");
