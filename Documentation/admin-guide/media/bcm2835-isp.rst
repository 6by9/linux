.. SPDX-License-Identifier: GPL-2.0

=============================================
Broadcom BCM2835 Image Signal Processor (ISP)
=============================================

The BCM2835 ISP
===============

The BCM2835 Image Signal Processor (ISP) is a fixed function hardware pipeline
that performs image processing on frames stored in memory. Frames can be Bayer,
RGB, or YUV and are submitted to the ISP through a V4L2 output node. The ISP
produces two processed image outputs at different resolutions and can generate
statistics for Bayer inputs.

The bcm2835-isp driver
======================

The bcm2835-isp driver lives under
`drivers/media/platform/broadcom/bcm2835-isp` and registers a set of V4L2 video
nodes connected through a media graph. The pipeline is configured through the
V4L2 extensible parameters framework using a dedicated params node.

The media topology registered by the driver is represented below:

.. _bcm2835-isp-topology:

.. kernel-figure:: bcm2835-isp.dot
    :alt:   Diagram of the default media pipeline topology
    :align: center

The media graph registers the following video device nodes:

- bcm2835-isp-output0: output device that queues frames to the ISP input.
- bcm2835-isp-capture0: capture device for the main processed output.
- bcm2835-isp-capture1: capture device for the secondary processed output.
- bcm2835-isp-stats2: metadata capture device for ISP statistics.
- bcm2835-isp-params: metadata output device for ISP configuration parameters.

bcm2835-isp-output0
-------------------

Frames to be processed by the ISP are queued to `bcm2835-isp-output0`. Supported
input formats include Bayer, RGB, and YUV.

bcm2835-isp-capture0, bcm2835-isp-capture1
------------------------------------------

The two capture devices return processed images in YUV or RGB formats. The
secondary output is typically used for a lower-resolution stream.

bcm2835-isp-stats2
------------------

The `bcm2835-isp-stats2` node provides per-frame statistics for Bayer inputs as
metadata buffers. The metadata format is described at
:ref:`v4l2-meta-fmt-bcm2835-isp-stats`.

bcm2835-isp-params
------------------

The `bcm2835-isp-params` node accepts configuration buffers that define the ISP
processing parameters to apply on the next frame boundary. The metadata format
is described at :ref:`v4l2-meta-fmt-bcm2835-isp-params`.

ISP configuration
=================

The ISP configuration is described solely by the contents of the parameters
buffer queued to `bcm2835-isp-params`. Each buffer uses the V4L2 extensible
parameters format described in :ref:`v4l2-isp`, with block types defined in
``include/uapi/linux/bcm2835-isp.h``.

Userspace must populate a :c:type:`v4l2_isp_params_buffer` and append one or
more block structs, each of which embeds a
:c:type:`v4l2_isp_params_block_header` as its first member. The driver applies
those parameters on a frame boundary once the buffer is queued. Parameter
structure is defined at :ref:`v4l2-meta-fmt-bcm2835-isp-params`.
