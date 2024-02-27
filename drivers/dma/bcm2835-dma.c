// SPDX-License-Identifier: GPL-2.0+
/*
 * BCM2835 DMA engine support
 *
 * Author:      Florian Meier <florian.meier@koalo.de>
 *              Copyright 2013
 *
 * Based on
 *	OMAP DMAengine support by Russell King
 *
 *	BCM2708 DMA Driver
 *	Copyright (C) 2010 Broadcom
 *
 *	Raspberry Pi PCM I2S ALSA Driver
 *	Copyright (c) by Phil Poole 2013
 *
 *	MARVELL MMP Peripheral DMA Driver
 *	Copyright 2012 Marvell International Ltd.
 */
#include <linux/dmaengine.h>
#include <linux/dma-direct.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_dma.h>

#include "virt-dma.h"

#define BCM2835_DMA_MAX_DMA_CHAN_SUPPORTED 14
#define BCM2835_DMA_CHAN_NAME_SIZE 8
#define BCM2711_DMA40_PHYS_ADDR 0x400000000ULL

/**
 * struct bcm2835_dmadev - BCM2835 DMA controller
 * @ddev: DMA device
 * @base: base address of register map
 * @zero_page: bus address of zero page (to detect transactions copying from
 *	zero page and avoid accessing memory if so)
 */
struct bcm2835_dmadev {
	struct dma_device ddev;
	void __iomem *base;
	dma_addr_t zero_page;
	const struct bcm2835_dma_cfg *cfg;
};

struct bcm_dma_cb {
	u32 rsvd[8];
};

struct bcm2835_dma_cb {
	u32 info;
	u32 src;
	u32 dst;
	u32 length;
	u32 stride;
	u32 next;
	u32 pad[2];
};

struct bcm2711_dma40_scb {
	u32 ti;
	u32 src;
	u32 srci;
	u32 dst;
	u32 dsti;
	u32 len;
	u32 next_cb;
	u32 pad;
};

struct bcm2835_cb_entry {
	struct bcm_dma_cb *cb;
	dma_addr_t paddr;
};

struct bcm2835_chan {
	struct virt_dma_chan vc;

	struct dma_slave_config	cfg;
	unsigned int dreq;

	int ch;
	struct bcm2835_desc *desc;
	struct dma_pool *cb_pool;

	void __iomem *chan_base;
	int irq_number;
	unsigned int irq_flags;

	bool is_lite_channel;
};

struct bcm2835_dma_cfg {
	dma_addr_t addr_offset;
	u32 cs_reg;
	u32 cb_reg;

	u32 wait_mask;
	u32 reset_mask;
	u32 int_mask;
	u32 active_mask;

	u32 (*cb_get_length)(void *data);
	dma_addr_t (*cb_get_addr)(void *data, enum dma_transfer_direction);

	void (*cb_init)(void *data, struct bcm2835_chan *c,
			enum dma_transfer_direction, dma_addr_t src,
			dma_addr_t dst, bool zero_page);
	void (*cb_set_src)(void *data, enum dma_transfer_direction,
			   dma_addr_t src);
	void (*cb_set_dst)(void *data, enum dma_transfer_direction,
			   dma_addr_t dst);
	void (*cb_set_next)(void *data, dma_addr_t next);
	void (*cb_set_length)(void *data, u32 length);
	void (*cb_append_extra)(void *data,
				struct bcm2835_chan *c,
				enum dma_transfer_direction direction,
				bool cyclic, bool final, unsigned long flags);
	void (*cb_dma_abort)(struct dma_chan *c);

	dma_addr_t (*to_cb_addr)(dma_addr_t addr);

	void (*chan_plat_init)(struct bcm2835_chan *c);
	dma_addr_t (*read_addr)(struct bcm2835_chan *c,
				enum dma_transfer_direction);

	u64 dma_mask;
};

struct bcm2835_desc {
	struct bcm2835_chan *c;
	struct virt_dma_desc vd;
	enum dma_transfer_direction dir;

	unsigned int frames;
	size_t size;

	bool cyclic;

	struct bcm2835_cb_entry cb_list[];
};

#define BCM2835_DMA_CS		0x00
#define BCM2835_DMA_ADDR	0x04
#define BCM2835_DMA_TI		0x08
#define BCM2835_DMA_SOURCE_AD	0x0c
#define BCM2835_DMA_DEST_AD	0x10
#define BCM2835_DMA_LEN		0x14
#define BCM2835_DMA_STRIDE	0x18
#define BCM2835_DMA_NEXTCB	0x1c
#define BCM2835_DMA_DEBUG	0x20

/* DMA CS Control and Status bits */
#define BCM2835_DMA_ACTIVE	BIT(0)  /* activate the DMA */
#define BCM2835_DMA_END		BIT(1)  /* current CB has ended */
#define BCM2835_DMA_INT		BIT(2)  /* interrupt status */
#define BCM2835_DMA_DREQ	BIT(3)  /* DREQ state */
#define BCM2835_DMA_ISPAUSED	BIT(4)  /* Pause requested or not active */
#define BCM2835_DMA_ISHELD	BIT(5)  /* Is held by DREQ flow control */
#define BCM2835_DMA_WAITING_FOR_WRITES BIT(6) /* waiting for last
					       * AXI-write to ack
					       */
#define BCM2835_DMA_ERR		BIT(8)
#define BCM2835_DMA_PRIORITY(x) ((x & 15) << 16) /* AXI priority */
#define BCM2835_DMA_PANIC_PRIORITY(x) ((x & 15) << 20) /* panic priority */
/* current value of TI.BCM2835_DMA_WAIT_RESP */
#define BCM2835_DMA_WAIT_FOR_WRITES BIT(28)
#define BCM2835_DMA_DIS_DEBUG	BIT(29) /* disable debug pause signal */
#define BCM2835_DMA_ABORT	BIT(30) /* Stop current CB, go to next, WO */
#define BCM2835_DMA_RESET	BIT(31) /* WO, self clearing */

/* Transfer information bits - also bcm2835_cb.info field */
#define BCM2835_DMA_INT_EN	BIT(0)
#define BCM2835_DMA_TDMODE	BIT(1) /* 2D-Mode */
#define BCM2835_DMA_WAIT_RESP	BIT(3) /* wait for AXI-write to be acked */
#define BCM2835_DMA_D_INC	BIT(4)
#define BCM2835_DMA_D_WIDTH	BIT(5) /* 128bit writes if set */
#define BCM2835_DMA_D_DREQ	BIT(6) /* enable DREQ for destination */
#define BCM2835_DMA_D_IGNORE	BIT(7) /* ignore destination writes */
#define BCM2835_DMA_S_INC	BIT(8)
#define BCM2835_DMA_S_WIDTH	BIT(9) /* 128bit writes if set */
#define BCM2835_DMA_S_DREQ	BIT(10) /* enable SREQ for source */
#define BCM2835_DMA_S_IGNORE	BIT(11) /* ignore source reads - read 0 */
#define BCM2835_DMA_BURST_LENGTH(x) ((x & 15) << 12)
#define BCM2835_DMA_PER_MAP(x)	((x & 31) << 16) /* REQ source */
#define BCM2835_DMA_WAIT(x)	((x & 31) << 21) /* add DMA-wait cycles */
#define BCM2835_DMA_NO_WIDE_BURSTS BIT(26) /* no 2 beat write bursts */

/* debug register bits */
#define BCM2835_DMA_DEBUG_LAST_NOT_SET_ERR	BIT(0)
#define BCM2835_DMA_DEBUG_FIFO_ERR		BIT(1)
#define BCM2835_DMA_DEBUG_READ_ERR		BIT(2)
#define BCM2835_DMA_DEBUG_OUTSTANDING_WRITES_SHIFT 4
#define BCM2835_DMA_DEBUG_OUTSTANDING_WRITES_BITS 4
#define BCM2835_DMA_DEBUG_ID_SHIFT		16
#define BCM2835_DMA_DEBUG_ID_BITS		9
#define BCM2835_DMA_DEBUG_STATE_SHIFT		16
#define BCM2835_DMA_DEBUG_STATE_BITS		9
#define BCM2835_DMA_DEBUG_VERSION_SHIFT		25
#define BCM2835_DMA_DEBUG_VERSION_BITS		3
#define BCM2835_DMA_DEBUG_LITE			BIT(28)

/* shared registers for all dma channels */
#define BCM2835_DMA_INT_STATUS         0xfe0
#define BCM2835_DMA_ENABLE             0xff0

#define BCM2835_DMA_DATA_TYPE_S8	1
#define BCM2835_DMA_DATA_TYPE_S16	2
#define BCM2835_DMA_DATA_TYPE_S32	4
#define BCM2835_DMA_DATA_TYPE_S128	16

/* Valid only for channels 0 - 14, 15 has its own base address */
#define BCM2835_DMA_CHAN(n)	((n) << 8) /* Base address */
#define BCM2835_DMA_CHANIO(base, n) ((base) + BCM2835_DMA_CHAN(n))

/* 40-bit DMA support */
#define BCM2711_DMA40_CS	0x00
#define BCM2711_DMA40_CB	0x04
#define BCM2711_DMA40_DEBUG	0x0c
#define BCM2711_DMA40_TI	0x10
#define BCM2711_DMA40_SRC	0x14
#define BCM2711_DMA40_SRCI	0x18
#define BCM2711_DMA40_DEST	0x1c
#define BCM2711_DMA40_DESTI	0x20
#define BCM2711_DMA40_LEN	0x24
#define BCM2711_DMA40_NEXT_CB	0x28
#define BCM2711_DMA40_DEBUG2	0x2c

#define BCM2711_DMA40_ACTIVE		BIT(0)
#define BCM2711_DMA40_END		BIT(1)
#define BCM2711_DMA40_INT		BIT(2)
#define BCM2711_DMA40_DREQ		BIT(3)  /* DREQ state */
#define BCM2711_DMA40_RD_PAUSED		BIT(4)  /* Reading is paused */
#define BCM2711_DMA40_WR_PAUSED		BIT(5)  /* Writing is paused */
#define BCM2711_DMA40_DREQ_PAUSED	BIT(6)  /* Is paused by DREQ flow control */
#define BCM2711_DMA40_WAITING_FOR_WRITES BIT(7)  /* Waiting for last write */
#define BCM2711_DMA40_PROT		(BIT(8) | BIT(9))
#define BCM2711_DMA40_ERR		BIT(10)
#define BCM2711_DMA40_QOS(x)		(((x) & 0x1f) << 16)
#define BCM2711_DMA40_PANIC_QOS(x)	(((x) & 0x1f) << 20)
#define BCM2711_DMA40_TRANSACTIONS	BIT(25)
#define BCM2711_DMA40_WAIT_FOR_WRITES	BIT(28)
#define BCM2711_DMA40_DISDEBUG		BIT(29)
#define BCM2711_DMA40_ABORT		BIT(30)
#define BCM2711_DMA40_HALT		BIT(31)

/* Transfer information bits */
#define BCM2711_DMA40_INTEN		BIT(0)
#define BCM2711_DMA40_TDMODE		BIT(1) /* 2D-Mode */
#define BCM2711_DMA40_WAIT_RESP		BIT(2) /* wait for AXI write to be acked */
#define BCM2711_DMA40_WAIT_RD_RESP	BIT(3) /* wait for AXI read to complete */
#define BCM2711_DMA40_PER_MAP(x)	(((x) & 31) << 9) /* REQ source */
#define BCM2711_DMA40_S_DREQ		BIT(14) /* enable SREQ for source */
#define BCM2711_DMA40_D_DREQ		BIT(15) /* enable DREQ for destination */
#define BCM2711_DMA40_S_WAIT(x)		(((x) & 0xff) << 16) /* add DMA read-wait cycles */
#define BCM2711_DMA40_D_WAIT(x)		(((x) & 0xff) << 24) /* add DMA write-wait cycles */

/* debug register bits */
#define BCM2711_DMA40_DEBUG_RESET		BIT(23)

#define BCM2711_DMA40_INC		BIT(12)
#define BCM2711_DMA40_IGNORE		BIT(15)

/* the max dma length for different channels */
#define MAX_DMA_LEN SZ_1G
#define MAX_LITE_DMA_LEN (SZ_64K - 4)

static inline size_t bcm2835_dma_max_frame_length(struct bcm2835_chan *c)
{
	/* lite and normal channels have different max frame length */
	return c->is_lite_channel ? MAX_LITE_DMA_LEN : MAX_DMA_LEN;
}

/* how many frames of max_len size do we need to transfer len bytes */
static inline size_t bcm2835_dma_frames_for_length(size_t len,
						   size_t max_len)
{
	return DIV_ROUND_UP(len, max_len);
}

static inline struct bcm2835_dmadev *to_bcm2835_dma_dev(struct dma_device *d)
{
	return container_of(d, struct bcm2835_dmadev, ddev);
}

static inline const struct bcm2835_dma_cfg *to_bcm2835_cfg(struct dma_device *d)
{
	struct bcm2835_dmadev *od = container_of(d, struct bcm2835_dmadev, ddev);

	return od->cfg;
}

static inline struct bcm2835_chan *to_bcm2835_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct bcm2835_chan, vc.chan);
}

static inline struct bcm2835_desc *to_bcm2835_dma_desc(
		struct dma_async_tx_descriptor *t)
{
	return container_of(t, struct bcm2835_desc, vd.tx);
}

static u32 bcm2835_dma_prepare_cb_info(struct bcm2835_chan *c,
				       enum dma_transfer_direction direction,
				       bool zero_page)
{
	u32 result;

	if (direction == DMA_MEM_TO_MEM)
		return BCM2835_DMA_D_INC | BCM2835_DMA_S_INC;

	result = BCM2835_DMA_WAIT_RESP;

	/* Setup DREQ channel */
	if (c->dreq != 0)
		result |= BCM2835_DMA_PER_MAP(c->dreq);

	if (direction == DMA_DEV_TO_MEM) {
		result |= BCM2835_DMA_S_DREQ | BCM2835_DMA_D_INC;
	} else {
		result |= BCM2835_DMA_D_DREQ | BCM2835_DMA_S_INC;

		/* non-lite channels can write zeroes w/o accessing memory */
		if (zero_page && !c->is_lite_channel)
			result |= BCM2835_DMA_S_IGNORE;
	}

	return result;
}

static u32 bcm2835_dma_prepare_cb_extra(struct bcm2835_chan *c,
					enum dma_transfer_direction direction,
					bool cyclic, bool final,
					unsigned long flags)
{
	u32 result = 0;

	if (cyclic) {
		if (flags & DMA_PREP_INTERRUPT)
			result |= BCM2835_DMA_INT_EN;
	} else {
		if (!final)
			return 0;

		result |= BCM2835_DMA_INT_EN;

		if (direction == DMA_MEM_TO_MEM)
			result |= BCM2835_DMA_WAIT_RESP;
	}

	return result;
}

static u32 bcm2711_dma_prepare_cb_info(struct bcm2835_chan *c,
				       enum dma_transfer_direction direction,
				       bool zero_page)
{
	u32 result;

	if (direction == DMA_MEM_TO_MEM)
		return 0;

	result = BCM2711_DMA40_WAIT_RESP;

	/* Setup DREQ channel */
	if (c->dreq != 0)
		result |= BCM2711_DMA40_PER_MAP(c->dreq);

	if (direction == DMA_DEV_TO_MEM)
		result |= BCM2711_DMA40_S_DREQ | BCM2711_DMA40_WAIT_RD_RESP;
	else
		result |= BCM2711_DMA40_D_DREQ;

	return result;
}

static u32 bcm2711_dma_prepare_cb_extra(struct bcm2835_chan *c,
					enum dma_transfer_direction direction,
					bool cyclic, bool final,
					unsigned long flags)
{
	u32 result = 0;

	if (cyclic) {
		if (flags & DMA_PREP_INTERRUPT)
			result |= BCM2711_DMA40_INTEN;
	} else {
		if (!final)
			return 0;

		result |= BCM2711_DMA40_INTEN;

		if (direction == DMA_MEM_TO_MEM)
			result |= BCM2711_DMA40_WAIT_RESP;
	}

	return result;
}

static inline bool need_src_incr(enum dma_transfer_direction direction)
{
	return direction != DMA_DEV_TO_MEM;
}

static inline bool need_dst_incr(enum dma_transfer_direction direction)
{
	switch (direction) {
	case DMA_MEM_TO_MEM:
	case DMA_DEV_TO_MEM:
		return true;
	default:
		break;
	}

	return false;
}

static u32 bcm2835_dma_cb_get_length(void *data)
{
	struct bcm2835_dma_cb *cb = data;

	return cb->length;
}

static dma_addr_t
bcm2835_dma_cb_get_addr(void *data, enum dma_transfer_direction direction)
{
	struct bcm2835_dma_cb *cb = data;

	if (direction == DMA_DEV_TO_MEM)
		return cb->dst;

	return cb->src;
}

static void
bcm2835_dma_cb_init(void *data, struct bcm2835_chan *c,
		    enum dma_transfer_direction direction, dma_addr_t src,
		    dma_addr_t dst, bool zero_page)
{
	struct bcm2835_dma_cb *cb = data;

	cb->info = bcm2835_dma_prepare_cb_info(c, direction, zero_page);
	cb->src = lower_32_bits(src);
	cb->dst = lower_32_bits(dst);
	cb->stride = 0;
	cb->next = 0;
}

static void
bcm2835_dma_cb_set_src(void *data, enum dma_transfer_direction direction,
		       dma_addr_t src)
{
	struct bcm2835_dma_cb *cb = data;

	cb->src = src;
}

static void
bcm2835_dma_cb_set_dst(void *data, enum dma_transfer_direction direction,
		       dma_addr_t dst)
{
	struct bcm2835_dma_cb *cb = data;

	cb->dst = dst;
}

static void bcm2835_dma_cb_set_next(void *data, dma_addr_t next)
{
	struct bcm2835_dma_cb *cb = data;

	cb->next = next;
}

static void bcm2835_dma_cb_set_length(void *data, u32 length)
{
	struct bcm2835_dma_cb *cb = data;

	cb->length = length;
}

static void
bcm2835_dma_cb_append_extra(void *data, struct bcm2835_chan *c,
			    enum dma_transfer_direction direction,
			    bool cyclic, bool final, unsigned long flags)
{
	struct bcm2835_dma_cb *cb = data;

	cb->info |= bcm2835_dma_prepare_cb_extra(c, direction, cyclic, final,
						 flags);
}

static dma_addr_t bcm2835_dma_to_cb_addr(dma_addr_t addr)
{
	return addr;
}

static void bcm2835_dma_chan_plat_init(struct bcm2835_chan *c)
{
	/* check in DEBUG register if this is a LITE channel */
	if (readl(c->chan_base + BCM2835_DMA_DEBUG) & BCM2835_DMA_DEBUG_LITE)
		c->is_lite_channel = true;
}

static dma_addr_t bcm2835_dma_read_addr(struct bcm2835_chan *c,
					enum dma_transfer_direction direction)
{
	if (direction == DMA_MEM_TO_DEV)
		return readl(c->chan_base + BCM2835_DMA_SOURCE_AD);
	else if (direction == DMA_DEV_TO_MEM)
		return readl(c->chan_base + BCM2835_DMA_DEST_AD);

	return 0;
}

/* BCM2711 and BCM2712 DMA4 channels */

static inline u32 bcm2711_dma_cb_get_length(void *data)
{
	struct bcm2711_dma40_scb *scb = data;

	return scb->len;
}

static inline dma_addr_t
bcm2711_dma_cb_get_addr(void *data, enum dma_transfer_direction direction)
{
	struct bcm2711_dma40_scb *scb = data;

	if (direction == DMA_DEV_TO_MEM)
		return (((dma_addr_t)scb->dsti & 0xff) << 32) + scb->dst;

	return (((dma_addr_t)scb->srci & 0xff) << 32) + scb->src;
}

static inline void
bcm2711_dma_cb_init(void *data, struct bcm2835_chan *c,
		    enum dma_transfer_direction direction, dma_addr_t src,
		    dma_addr_t dst, bool zero_page)
{
	struct bcm2711_dma40_scb *scb = data;

	scb->ti = bcm2711_dma_prepare_cb_info(c, direction, zero_page);
	scb->src = lower_32_bits(src);
	scb->srci = upper_32_bits(src);

	if (need_src_incr(direction))
		scb->srci |= BCM2711_DMA40_INC;

	scb->dst = lower_32_bits(dst);
	scb->dsti = upper_32_bits(dst);

	if (need_dst_incr(direction))
		scb->dsti |= BCM2711_DMA40_INC;

	scb->next_cb = 0;
}

static inline void
bcm2711_dma_cb_set_src(void *data, enum dma_transfer_direction direction,
		       dma_addr_t src)
{
	struct bcm2711_dma40_scb *scb = data;

	scb->src = lower_32_bits(src);
	scb->srci = upper_32_bits(src);

	if (need_src_incr(direction))
		scb->srci |= BCM2711_DMA40_INC;
}

static inline void
bcm2711_dma_cb_set_dst(void *data, enum dma_transfer_direction direction,
		       dma_addr_t dst)
{
	struct bcm2711_dma40_scb *scb = data;

	scb->dst = lower_32_bits(dst);
	scb->dsti = upper_32_bits(dst);

	if (need_dst_incr(direction))
		scb->dsti |= BCM2711_DMA40_INC;
}

static inline void bcm2711_dma_cb_set_next(void *data, dma_addr_t next)
{
	struct bcm2711_dma40_scb *scb = data;

	scb->next_cb = next;
}

static inline void bcm2711_dma_cb_set_length(void *data, u32 length)
{
	struct bcm2711_dma40_scb *scb = data;

	scb->len = length;
}

static inline void
bcm2711_dma_cb_append_extra(void *data, struct bcm2835_chan *c,
			    enum dma_transfer_direction direction,
			    bool cyclic, bool final, unsigned long flags)
{
	struct bcm2711_dma40_scb *scb = data;

	scb->ti |= bcm2711_dma_prepare_cb_extra(c, direction, cyclic, final,
						flags);
}

static inline dma_addr_t bcm2711_dma_to_cb_addr(dma_addr_t addr)
{
	return (addr >> 5);
}

static void bcm2711_dma_chan_plat_init(struct bcm2835_chan *c)
{
}

static dma_addr_t bcm2711_dma_read_addr(struct bcm2835_chan *c,
					enum dma_transfer_direction direction)
{
	if (direction == DMA_MEM_TO_DEV)
		return readl(c->chan_base + BCM2711_DMA40_SRC) +
			((readl(c->chan_base + BCM2711_DMA40_SRCI) & 0xff) << 8);
	else if (direction == DMA_DEV_TO_MEM)
		return readl(c->chan_base + BCM2711_DMA40_DEST) +
			((readl(c->chan_base + BCM2711_DMA40_DESTI) & 0xff) << 8);

	return 0;
}

static void bcm2711_dma_cb_abort(struct dma_chan *chan)
{
	const struct bcm2835_dma_cfg *cfg = to_bcm2835_cfg(chan->device);
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	void __iomem *chan_base = c->chan_base;
	int timeout = 10000;

	/*
	 * A zero control block address means the channel is idle.
	 * (The ACTIVE flag in the CS register is not a reliable indicator.)
	 */
	if (!readl(chan_base + cfg->cb_reg))
		return;

	/* Pause the current DMA */
	writel(readl(chan_base + cfg->cs_reg) & ~BCM2711_DMA40_ACTIVE,
	       chan_base + cfg->cs_reg);

	/* wait for outstanding transactions to complete */
	while ((readl(chan_base + cfg->cs_reg) & BCM2711_DMA40_TRANSACTIONS) &&
	       --timeout)
		cpu_relax();

	/* Peripheral might be stuck and fail to signal AXI write responses */
	if (!timeout)
		dev_err(c->vc.chan.device->dev,
			"failed to complete outstanding writes\n");

	writel(BCM2711_DMA40_PROT, chan_base + cfg->cs_reg);

	/* Reset the DMA */
	writel(readl(chan_base + BCM2711_DMA40_DEBUG) | BCM2711_DMA40_DEBUG_RESET,
	       chan_base + BCM2711_DMA40_DEBUG);
}

static void bcm2835_dma_free_cb_chain(struct bcm2835_desc *desc)
{
	size_t i;

	for (i = 0; i < desc->frames; i++)
		dma_pool_free(desc->c->cb_pool, desc->cb_list[i].cb,
			      desc->cb_list[i].paddr);

	kfree(desc);
}

static void bcm2835_dma_desc_free(struct virt_dma_desc *vd)
{
	bcm2835_dma_free_cb_chain(
		container_of(vd, struct bcm2835_desc, vd));
}

static bool
bcm2835_dma_create_cb_set_length(struct dma_chan *chan, void *data, size_t len,
				 size_t period_len, size_t *total_len)
{
	const struct bcm2835_dma_cfg *cfg = to_bcm2835_cfg(chan->device);
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	size_t max_len = bcm2835_dma_max_frame_length(c);

	/* set the length taking lite-channel limitations into account */
	u32 length = min_t(u32, len, max_len);

	cfg->cb_set_length(data, length);

	/* finished if we have no period_length */
	if (!period_len)
		return false;

	/*
	 * period_len means: that we need to generate
	 * transfers that are terminating at every
	 * multiple of period_len - this is typically
	 * used to set the interrupt flag in info
	 * which is required during cyclic transfers
	 */

	/* have we filled in period_length yet? */
	if (*total_len + length < period_len) {
		/* update number of bytes in this period so far */
		*total_len += length;
		return false;
	}

	/* calculate the length that remains to reach period_length */
	cfg->cb_set_length(data, period_len - *total_len);

	/* reset total_length for next period */
	*total_len = 0;

	return true;
}

static inline size_t bcm2835_dma_count_frames_for_sg(
	struct bcm2835_chan *c,
	struct scatterlist *sgl,
	unsigned int sg_len)
{
	size_t frames = 0;
	struct scatterlist *sgent;
	unsigned int i;
	size_t plength = bcm2835_dma_max_frame_length(c);

	for_each_sg(sgl, sgent, sg_len, i)
		frames += bcm2835_dma_frames_for_length(
			sg_dma_len(sgent), plength);

	return frames;
}

/**
 * bcm2835_dma_create_cb_chain - create a control block and fills data in
 *
 * @chan:           the @dma_chan for which we run this
 * @direction:      the direction in which we transfer
 * @cyclic:         it is a cyclic transfer
 * @frames:         number of controlblocks to allocate
 * @src:            the src address to assign
 * @dst:            the dst address to assign
 * @buf_len:        the full buffer length (may also be 0)
 * @period_len:     the period length when to apply @finalextrainfo
 *                  in addition to the last transfer
 *                  this will also break some control-blocks early
 * @gfp:            the GFP flag to use for allocation
 * @flags
 */
static struct bcm2835_desc *bcm2835_dma_create_cb_chain(
	struct dma_chan *chan, enum dma_transfer_direction direction,
	bool cyclic, size_t frames, dma_addr_t src, dma_addr_t dst,
	size_t buf_len,	size_t period_len, gfp_t gfp, unsigned long flags)
{
	const struct bcm2835_dma_cfg *cfg = to_bcm2835_cfg(chan->device);
	struct bcm2835_dmadev *od = to_bcm2835_dma_dev(chan->device);
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	size_t len = buf_len, total_len;
	size_t frame;
	struct bcm2835_desc *d;
	struct bcm2835_cb_entry *cb_entry;
	struct bcm_dma_cb *control_block;
	bool zero_page = false;

	if (!frames)
		return NULL;

	/* allocate and setup the descriptor. */
	d = kzalloc(struct_size(d, cb_list, frames), gfp);
	if (!d)
		return NULL;

	d->c = c;
	d->dir = direction;
	d->cyclic = cyclic;

	switch (direction) {
	case DMA_MEM_TO_MEM:
	case DMA_DEV_TO_MEM:
		break;
	default:
		zero_page = src == od->zero_page;
	}

	/*
	 * Iterate over all frames, create a control block
	 * for each frame and link them together.
	 */
	for (frame = 0, total_len = 0; frame < frames; d->frames++, frame++) {
		cb_entry = &d->cb_list[frame];
		cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
					      &cb_entry->paddr);
		if (!cb_entry->cb)
			goto error_cb;

		/* fill in the control block */
		control_block = cb_entry->cb;

		cfg->cb_init(control_block, c, direction, src, dst, zero_page);
		/* set up length in control_block if requested */
		if (buf_len) {
			/* calculate length honoring period_length */
			if (bcm2835_dma_create_cb_set_length(chan, control_block,
							     len, period_len,
							     &total_len)) {
				/* add extrainfo bits in info */
				bcm2835_dma_cb_append_extra(control_block, c,
							    direction, cyclic,
							    false, flags);
			}

			/* calculate new remaining length */
			len -= cfg->cb_get_length(control_block);
		}

		/* link this the last controlblock */
		if (frame)
			cfg->cb_set_next(d->cb_list[frame - 1].cb,
					 cfg->to_cb_addr(cb_entry->paddr));

		/* update src and dst and length */
		if (src && need_src_incr(direction))
			src += cfg->cb_get_length(control_block);
		if (dst && need_dst_incr(direction))
			dst += cfg->cb_get_length(control_block);

		/* Length of total transfer */
		d->size += cfg->cb_get_length(control_block);
	}

	/* the last frame requires extra flags */
	cfg->cb_append_extra(d->cb_list[d->frames - 1].cb, c, direction, cyclic,
			     true, flags);

	/* detect a size missmatch */
	if (buf_len && (d->size != buf_len))
		goto error_cb;

	return d;
error_cb:
	bcm2835_dma_free_cb_chain(d);

	return NULL;
}

static void bcm2835_dma_fill_cb_chain_with_sg(
	struct dma_chan *chan,
	enum dma_transfer_direction direction,
	struct bcm2835_cb_entry *cb,
	struct scatterlist *sgl,
	unsigned int sg_len)
{
	const struct bcm2835_dma_cfg *cfg = to_bcm2835_cfg(chan->device);
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	size_t len, max_len;
	unsigned int i;
	dma_addr_t addr;
	struct scatterlist *sgent;

	max_len = bcm2835_dma_max_frame_length(c);
	for_each_sg(sgl, sgent, sg_len, i) {
		for (addr = sg_dma_address(sgent), len = sg_dma_len(sgent);
		     len > 0;
		     addr += cfg->cb_get_length(cb->cb), len -= cfg->cb_get_length(cb->cb), cb++) {
			if (direction == DMA_DEV_TO_MEM)
				cfg->cb_set_dst(cb->cb, direction, addr);
			else
				cfg->cb_set_src(cb->cb, direction, addr);
			cfg->cb_set_length(cb->cb, min(len, max_len));
		}
	}
}

static void bcm2835_dma_cb_abort(struct dma_chan *chan)
{
	const struct bcm2835_dma_cfg *cfg = to_bcm2835_cfg(chan->device);
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	void __iomem *chan_base = c->chan_base;
	long int timeout = 10000;

	/*
	 * A zero control block address means the channel is idle.
	 * (The ACTIVE flag in the CS register is not a reliable indicator.)
	 */
	if (!readl(chan_base + cfg->cb_reg))
		return;

	/* Write 0 to the active bit - Pause the DMA */
	writel(0, chan_base + cfg->cs_reg);

	/* Wait for any current AXI transfer to complete */
	while ((readl(chan_base + cfg->cs_reg) & cfg->wait_mask) &&
	       --timeout)
		cpu_relax();

	/* Peripheral might be stuck and fail to signal AXI write responses */
	if (!timeout)
		dev_err(c->vc.chan.device->dev,
			"failed to complete outstanding writes\n");

	writel(cfg->reset_mask, chan_base + cfg->cs_reg);
}

static void bcm2835_dma_start_desc(struct dma_chan *chan)
{
	const struct bcm2835_dma_cfg *cfg = to_bcm2835_cfg(chan->device);
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	struct virt_dma_desc *vd = vchan_next_desc(&c->vc);
	struct bcm2835_desc *d;

	if (!vd) {
		c->desc = NULL;
		return;
	}

	list_del(&vd->node);

	c->desc = d = to_bcm2835_dma_desc(&vd->tx);

	writel(cfg->to_cb_addr(d->cb_list[0].paddr), c->chan_base + cfg->cb_reg);
	writel(cfg->active_mask, c->chan_base + cfg->cs_reg);
}

static irqreturn_t bcm2835_dma_callback(int irq, void *data)
{
	struct dma_chan *chan = data;
	const struct bcm2835_dma_cfg *cfg = to_bcm2835_cfg(chan->device);
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	struct bcm2835_desc *d;
	unsigned long flags;

	/* check the shared interrupt */
	if (c->irq_flags & IRQF_SHARED) {
		/* check if the interrupt is enabled */
		flags = readl(c->chan_base + cfg->cs_reg);
		/* if not set then we are not the reason for the irq */
		if (!(flags & cfg->int_mask))
			return IRQ_NONE;
	}

	spin_lock_irqsave(&c->vc.lock, flags);

	/*
	 * Clear the INT flag to receive further interrupts. Keep the channel
	 * active in case the descriptor is cyclic or in case the client has
	 * already terminated the descriptor and issued a new one. (May happen
	 * if this IRQ handler is threaded.) If the channel is finished, it
	 * will remain idle despite the ACTIVE flag being set.
	 */
	writel(cfg->int_mask | cfg->active_mask, c->chan_base + cfg->cs_reg);

	d = c->desc;

	if (d) {
		if (d->cyclic) {
			/* call the cyclic callback */
			vchan_cyclic_callback(&d->vd);
		} else if (!readl(c->chan_base + cfg->cb_reg)) {
			vchan_cookie_complete(&c->desc->vd);
			bcm2835_dma_start_desc(chan);
		}
	}

	spin_unlock_irqrestore(&c->vc.lock, flags);

	return IRQ_HANDLED;
}

static int bcm2835_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	struct device *dev = c->vc.chan.device->dev;

	dev_dbg(dev, "Allocating DMA channel %d\n", c->ch);

	/*
	 * Control blocks are 256 bit in length and must start at a 256 bit
	 * (32 byte) aligned address (BCM2835 ARM Peripherals, sec. 4.2.1.1).
	 */
	c->cb_pool = dma_pool_create(dev_name(dev), dev,
				     sizeof(struct bcm_dma_cb), 32, 0);
	if (!c->cb_pool) {
		dev_err(dev, "unable to allocate descriptor pool\n");
		return -ENOMEM;
	}

	return request_irq(c->irq_number, bcm2835_dma_callback,
			   c->irq_flags, "DMA IRQ", chan);
}

static void bcm2835_dma_free_chan_resources(struct dma_chan *chan)
{
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);

	vchan_free_chan_resources(&c->vc);
	free_irq(c->irq_number, c);
	dma_pool_destroy(c->cb_pool);

	dev_dbg(c->vc.chan.device->dev, "Freeing DMA channel %u\n", c->ch);
}

static size_t bcm2835_dma_desc_size(struct bcm2835_desc *d)
{
	return d->size;
}

static size_t bcm2835_dma_desc_size_pos(const struct bcm2835_dma_cfg *cfg,
					struct bcm2835_desc *d, dma_addr_t addr)
{
	unsigned int i;
	size_t size;

	for (size = i = 0; i < d->frames; i++) {
		struct bcm_dma_cb *control_block = d->cb_list[i].cb;
		size_t this_size = cfg->cb_get_length(control_block);
		dma_addr_t dma = cfg->cb_get_addr(control_block, d->dir);

		if (size)
			size += this_size;
		else if (addr >= dma && addr < dma + this_size)
			size += dma + this_size - addr;
	}

	return size;
}

static enum dma_status bcm2835_dma_tx_status(struct dma_chan *chan,
	dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	const struct bcm2835_dma_cfg *cfg = to_bcm2835_cfg(chan->device);
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	struct virt_dma_desc *vd;
	enum dma_status ret;
	unsigned long flags;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE || !txstate)
		return ret;

	spin_lock_irqsave(&c->vc.lock, flags);
	vd = vchan_find_desc(&c->vc, cookie);
	if (vd) {
		txstate->residue =
			bcm2835_dma_desc_size(to_bcm2835_dma_desc(&vd->tx));
	} else if (c->desc && c->desc->vd.tx.cookie == cookie) {
		struct bcm2835_desc *d = c->desc;
		dma_addr_t pos;

		pos = cfg->read_addr(c, d->dir);
		txstate->residue = bcm2835_dma_desc_size_pos(cfg, d, pos);
	} else {
		txstate->residue = 0;
	}

	spin_unlock_irqrestore(&c->vc.lock, flags);

	return ret;
}

static void bcm2835_dma_issue_pending(struct dma_chan *chan)
{
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&c->vc.lock, flags);
	if (vchan_issue_pending(&c->vc) && !c->desc)
		bcm2835_dma_start_desc(chan);

	spin_unlock_irqrestore(&c->vc.lock, flags);
}

static struct dma_async_tx_descriptor *bcm2835_dma_prep_dma_memcpy(
	struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
	size_t len, unsigned long flags)
{
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	struct bcm2835_desc *d;
	size_t max_len = bcm2835_dma_max_frame_length(c);
	size_t frames;

	/* if src, dst or len is not given return with an error */
	if (!src || !dst || !len)
		return NULL;

	/* calculate number of frames */
	frames = bcm2835_dma_frames_for_length(len, max_len);

	/* allocate the CB chain - this also fills in the pointers */
	d = bcm2835_dma_create_cb_chain(chan, DMA_MEM_TO_MEM, false, frames,
					src, dst, len, 0, GFP_KERNEL, 0);
	if (!d)
		return NULL;

	return vchan_tx_prep(&c->vc, &d->vd, flags);
}

static struct dma_async_tx_descriptor *bcm2835_dma_prep_slave_sg(
	struct dma_chan *chan,
	struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	const struct bcm2835_dma_cfg *cfg = to_bcm2835_cfg(chan->device);
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	struct bcm2835_desc *d;
	dma_addr_t src = 0, dst = 0;
	size_t frames;

	if (!is_slave_direction(direction)) {
		dev_err(chan->device->dev,
			"%s: bad direction?\n", __func__);
		return NULL;
	}

	if (direction == DMA_DEV_TO_MEM) {
		if (c->cfg.src_addr_width != DMA_SLAVE_BUSWIDTH_4_BYTES)
			return NULL;
		src = phys_to_dma(chan->device->dev, c->cfg.src_addr);
	} else {
		if (c->cfg.dst_addr_width != DMA_SLAVE_BUSWIDTH_4_BYTES)
			return NULL;
		dst = phys_to_dma(chan->device->dev, c->cfg.dst_addr);
	}

	/* count frames in sg list */
	frames = bcm2835_dma_count_frames_for_sg(c, sgl, sg_len);

	/* allocate the CB chain */
	d = bcm2835_dma_create_cb_chain(chan, direction, false, frames, src,
					dst, 0, 0, GFP_NOWAIT, 0);
	if (!d)
		return NULL;

	/* fill in frames with scatterlist pointers */
	bcm2835_dma_fill_cb_chain_with_sg(chan, direction, d->cb_list,
					  sgl, sg_len);

	return vchan_tx_prep(&c->vc, &d->vd, flags);
}

static struct dma_async_tx_descriptor *bcm2835_dma_prep_dma_cyclic(
	struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction,
	unsigned long flags)
{
	const struct bcm2835_dma_cfg *cfg = to_bcm2835_cfg(chan->device);
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	struct bcm2835_desc *d;
	dma_addr_t src, dst;
	size_t max_len = bcm2835_dma_max_frame_length(c);
	size_t frames;

	/* Grab configuration */
	if (!is_slave_direction(direction)) {
		dev_err(chan->device->dev, "%s: bad direction?\n", __func__);
		return NULL;
	}

	if (!buf_len) {
		dev_err(chan->device->dev,
			"%s: bad buffer length (= 0)\n", __func__);
		return NULL;
	}

	if (!(flags & DMA_PREP_INTERRUPT))
		period_len = buf_len;

	/*
	 * warn if buf_len is not a multiple of period_len - this may leed
	 * to unexpected latencies for interrupts and thus audiable clicks
	 */
	if (buf_len % period_len)
		dev_warn_once(chan->device->dev,
			      "%s: buffer_length (%zd) is not a multiple of period_len (%zd)\n",
			      __func__, buf_len, period_len);

	if (direction == DMA_DEV_TO_MEM) {
		if (c->cfg.src_addr_width != DMA_SLAVE_BUSWIDTH_4_BYTES)
			return NULL;
		src = phys_to_dma(chan->device->dev, c->cfg.src_addr);
		dst = buf_addr;
	} else {
		if (c->cfg.dst_addr_width != DMA_SLAVE_BUSWIDTH_4_BYTES)
			return NULL;
		dst = phys_to_dma(chan->device->dev, c->cfg.dst_addr);
		src = buf_addr;
	}

	/* calculate number of frames */
	frames = /* number of periods */
		 DIV_ROUND_UP(buf_len, period_len) *
		 /* number of frames per period */
		 bcm2835_dma_frames_for_length(period_len, max_len);

	/*
	 * allocate the CB chain
	 * note that we need to use GFP_NOWAIT, as the ALSA i2s dmaengine
	 * implementation calls prep_dma_cyclic with interrupts disabled.
	 */
	d = bcm2835_dma_create_cb_chain(chan, direction, true, frames, src, dst,
					buf_len, period_len, GFP_NOWAIT, flags);
	if (!d)
		return NULL;

	/* wrap around into a loop */
	cfg->cb_set_next(d->cb_list[d->frames - 1].cb,
			 cfg->to_cb_addr(d->cb_list[0].paddr));

	return vchan_tx_prep(&c->vc, &d->vd, flags);
}

static int bcm2835_dma_slave_config(struct dma_chan *chan,
				    struct dma_slave_config *cfg)
{
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);

	c->cfg = *cfg;

	return 0;
}

static int bcm2835_dma_terminate_all(struct dma_chan *chan)
{
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	const struct bcm2835_dma_cfg *cfg = to_bcm2835_cfg(chan->device);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&c->vc.lock, flags);

	/* stop DMA activity */
	if (c->desc) {
		vchan_terminate_vdesc(&c->desc->vd);
		c->desc = NULL;
		cfg->cb_dma_abort(chan);
	}

	vchan_get_all_descriptors(&c->vc, &head);
	spin_unlock_irqrestore(&c->vc.lock, flags);
	vchan_dma_desc_free_list(&c->vc, &head);

	return 0;
}

static void bcm2835_dma_synchronize(struct dma_chan *chan)
{
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);

	vchan_synchronize(&c->vc);
}

static int bcm2835_dma_chan_init(struct bcm2835_dmadev *d,
				 unsigned int chan_start, int chan_id, int irq,
				 unsigned int irq_flags)
{
	struct bcm2835_chan *c;

	c = devm_kzalloc(d->ddev.dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	c->vc.desc_free = bcm2835_dma_desc_free;
	vchan_init(&c->vc, &d->ddev);

	c->chan_base = BCM2835_DMA_CHANIO(d->base, chan_id - chan_start);
	c->ch = chan_id;
	c->irq_number = irq;
	c->irq_flags = irq_flags;

	d->cfg->chan_plat_init(c);

	return 0;
}

static void bcm2835_dma_free(struct bcm2835_dmadev *od)
{
	struct bcm2835_chan *c, *next;

	list_for_each_entry_safe(c, next, &od->ddev.channels,
				 vc.chan.device_node) {
		list_del(&c->vc.chan.device_node);
		tasklet_kill(&c->vc.task);
	}

	dma_unmap_page_attrs(od->ddev.dev, od->zero_page, PAGE_SIZE,
			     DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
}

static const struct bcm2835_dma_cfg bcm2835_data = {
	.addr_offset = 0,

	.cs_reg = BCM2835_DMA_CS,
	.cb_reg = BCM2835_DMA_ADDR,

	.wait_mask = BCM2835_DMA_WAITING_FOR_WRITES,
	.reset_mask = BCM2835_DMA_RESET,
	.int_mask = BCM2835_DMA_INT,
	.active_mask = BCM2835_DMA_ACTIVE,

	.cb_get_length = bcm2835_dma_cb_get_length,
	.cb_get_addr = bcm2835_dma_cb_get_addr,
	.cb_init = bcm2835_dma_cb_init,
	.cb_set_src = bcm2835_dma_cb_set_src,
	.cb_set_dst = bcm2835_dma_cb_set_dst,
	.cb_set_next = bcm2835_dma_cb_set_next,
	.cb_set_length = bcm2835_dma_cb_set_length,
	.cb_append_extra = bcm2835_dma_cb_append_extra,
	.cb_dma_abort = bcm2835_dma_cb_abort,

	.to_cb_addr = bcm2835_dma_to_cb_addr,

	.chan_plat_init = bcm2835_dma_chan_plat_init,
	.read_addr = bcm2835_dma_read_addr,

	.dma_mask = DMA_BIT_MASK(32),
};

static const struct bcm2835_dma_cfg bcm2711_data = {
	.addr_offset = BCM2711_DMA40_PHYS_ADDR,

	.cs_reg = BCM2711_DMA40_CS,
	.cb_reg = BCM2711_DMA40_CB,

	.wait_mask = BCM2711_DMA40_WAITING_FOR_WRITES,
	.reset_mask = BCM2711_DMA40_HALT,
	.int_mask = BCM2711_DMA40_INTEN,
	.active_mask = BCM2711_DMA40_ACTIVE | BCM2711_DMA40_PROT,

	.cb_get_length = bcm2711_dma_cb_get_length,
	.cb_get_addr = bcm2711_dma_cb_get_addr,
	.cb_init = bcm2711_dma_cb_init,
	.cb_set_src = bcm2711_dma_cb_set_src,
	.cb_set_dst = bcm2711_dma_cb_set_dst,
	.cb_set_next = bcm2711_dma_cb_set_next,
	.cb_set_length = bcm2711_dma_cb_set_length,
	.cb_append_extra = bcm2711_dma_cb_append_extra,
	.cb_dma_abort = bcm2711_dma_cb_abort,

	.to_cb_addr = bcm2711_dma_to_cb_addr,

	.chan_plat_init = bcm2711_dma_chan_plat_init,
	.read_addr = bcm2711_dma_read_addr,

	.dma_mask = DMA_BIT_MASK(40),
};

static const struct of_device_id bcm2835_dma_of_match[] = {
	{ .compatible = "brcm,bcm2835-dma", .data = &bcm2835_data },
	{ .compatible = "brcm,bcm2711-dma", .data = &bcm2711_data },
	{},
};
MODULE_DEVICE_TABLE(of, bcm2835_dma_of_match);

static struct dma_chan *bcm2835_dma_xlate(struct of_phandle_args *spec,
					   struct of_dma *ofdma)
{
	struct bcm2835_dmadev *d = ofdma->of_dma_data;
	struct dma_chan *chan;

	chan = dma_get_any_slave_channel(&d->ddev);
	if (!chan)
		return NULL;

	/* Set DREQ from param */
	to_bcm2835_dma_chan(chan)->dreq = spec->args[0];

	return chan;
}

static int bcm2835_dma_probe(struct platform_device *pdev)
{
	struct bcm2835_dmadev *od;
	void __iomem *base;
	int rc;
	int i, j;
	int interrupt;
	int irq[BCM2835_DMA_MAX_DMA_CHAN_SUPPORTED + 1];
	int irq_flags;
	int chan_start = -1;
	uint32_t chans_available;
	char chan_name[BCM2835_DMA_CHAN_NAME_SIZE];

	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	od = devm_kzalloc(&pdev->dev, sizeof(*od), GFP_KERNEL);
	if (!od)
		return -ENOMEM;

	od->cfg = of_device_get_match_data(&pdev->dev);
	if (!od->cfg)
		return -EINVAL;

	dma_set_max_seg_size(&pdev->dev, 0x3FFFFFFF);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	od->base = base;

	rc = dma_set_mask_and_coherent(&pdev->dev, od->cfg->dma_mask);
	if (rc) {
		dev_err(&pdev->dev, "Unable to set DMA mask\n");
		return rc;
	}

	dma_cap_set(DMA_SLAVE, od->ddev.cap_mask);
	dma_cap_set(DMA_PRIVATE, od->ddev.cap_mask);
	dma_cap_set(DMA_CYCLIC, od->ddev.cap_mask);
	dma_cap_set(DMA_MEMCPY, od->ddev.cap_mask);
	od->ddev.device_alloc_chan_resources = bcm2835_dma_alloc_chan_resources;
	od->ddev.device_free_chan_resources = bcm2835_dma_free_chan_resources;
	od->ddev.device_tx_status = bcm2835_dma_tx_status;
	od->ddev.device_issue_pending = bcm2835_dma_issue_pending;
	od->ddev.device_prep_dma_cyclic = bcm2835_dma_prep_dma_cyclic;
	od->ddev.device_prep_slave_sg = bcm2835_dma_prep_slave_sg;
	od->ddev.device_prep_dma_memcpy = bcm2835_dma_prep_dma_memcpy;
	od->ddev.device_config = bcm2835_dma_slave_config;
	od->ddev.device_terminate_all = bcm2835_dma_terminate_all;
	od->ddev.device_synchronize = bcm2835_dma_synchronize;
	od->ddev.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	od->ddev.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	od->ddev.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV) |
			      BIT(DMA_MEM_TO_MEM);
	od->ddev.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	od->ddev.descriptor_reuse = true;
	od->ddev.dev = &pdev->dev;
	INIT_LIST_HEAD(&od->ddev.channels);

	platform_set_drvdata(pdev, od);

	od->zero_page = dma_map_page_attrs(od->ddev.dev, ZERO_PAGE(0), 0,
					   PAGE_SIZE, DMA_TO_DEVICE,
					   DMA_ATTR_SKIP_CPU_SYNC);
	if (dma_mapping_error(od->ddev.dev, od->zero_page)) {
		dev_err(&pdev->dev, "Failed to map zero page\n");
		return -ENOMEM;
	}

	/* Request DMA channel mask from device tree */
	rc = of_property_read_u32(pdev->dev.of_node, "dma-channel-mask",
				  &chans_available);

	if (rc) {
		/* Try deprecated property */
		if (of_property_read_u32(pdev->dev.of_node,
					 "brcm,dma-channel-mask",
					 &chans_available)) {
			dev_err(&pdev->dev, "Failed to get channel mask\n");
			goto err_no_dma;
		}

		dev_warn(&pdev->dev, "brcm,dma-channel-mask deprecated - please update DT\n");
	}

	/* get irqs for each channel that we support */
	for (i = 0; i <= BCM2835_DMA_MAX_DMA_CHAN_SUPPORTED; i++) {
		/* get the named irq */
		snprintf(chan_name, sizeof(chan_name), "dma%i", i);
		interrupt = platform_get_irq_byname_optional(pdev, chan_name);

		/* Find first channel with a defined interrupt, even if masked
		 * out. The base address is expected to match that channel.
		 *
		 * Not supporting offset dma with the legacy missing
		 * interrupt-names property.
		 */
		if (interrupt >= 0 && chan_start == -1)
			chan_start = i;

		/* skip masked out channels */
		if (!(chans_available & (1 << i))) {
			irq[i] = -1;
			continue;
		}

		if (interrupt >= 0) {
			irq[i] = interrupt;
			continue;
		}

		/* legacy device tree case handling */
		dev_warn_once(&pdev->dev,
			      "missing interrupt-names property in device tree - legacy interpretation is used\n");
		/*
		 * in case of channel >= 11
		 * use the 11th interrupt and that is shared
		 */
		irq[i] = platform_get_irq(pdev, i < 11 ? i : 11);
	}

	if (chan_start < 0) {
		dev_warn_once(&pdev->dev, "Not interrupts defined for DMA channels\n");
		chan_start = 0;
	}

	/* get irqs for each channel */
	for (i = chan_start; i <= BCM2835_DMA_MAX_DMA_CHAN_SUPPORTED; i++) {
		/* skip channels without irq */
		if (irq[i] < 0)
			continue;

		/* check if there are other channels that also use this irq */
		irq_flags = 0;
		for (j = chan_start; j <= BCM2835_DMA_MAX_DMA_CHAN_SUPPORTED; j++)
			if ((i != j) && (irq[j] == irq[i])) {
				irq_flags = IRQF_SHARED;
				break;
			}

		/* initialize the channel */
		rc = bcm2835_dma_chan_init(od, chan_start, i, irq[i], irq_flags);
		if (rc)
			goto err_no_dma;
	}

	dev_dbg(&pdev->dev, "Initialized %i DMA channels\n", i);

	/* Device-tree DMA controller registration */
	rc = of_dma_controller_register(pdev->dev.of_node,
			bcm2835_dma_xlate, od);
	if (rc) {
		dev_err(&pdev->dev, "Failed to register DMA controller\n");
		goto err_no_dma;
	}

	rc = dma_async_device_register(&od->ddev);
	if (rc) {
		dev_err(&pdev->dev,
			"Failed to register slave DMA engine device: %d\n", rc);
		goto err_no_dma;
	}

	dev_dbg(&pdev->dev, "Load BCM2835 DMA engine driver\n");

	return 0;

err_no_dma:
	bcm2835_dma_free(od);
	return rc;
}

static void bcm2835_dma_remove(struct platform_device *pdev)
{
	struct bcm2835_dmadev *od = platform_get_drvdata(pdev);

	dma_async_device_unregister(&od->ddev);
	bcm2835_dma_free(od);
}

static struct platform_driver bcm2835_dma_driver = {
	.probe	= bcm2835_dma_probe,
	.remove_new = bcm2835_dma_remove,
	.driver = {
		.name = "bcm2835-dma",
		.of_match_table = of_match_ptr(bcm2835_dma_of_match),
	},
};

module_platform_driver(bcm2835_dma_driver);

MODULE_ALIAS("platform:bcm2835-dma");
MODULE_DESCRIPTION("BCM2835 DMA engine driver");
MODULE_AUTHOR("Florian Meier <florian.meier@koalo.de>");
MODULE_LICENSE("GPL");
