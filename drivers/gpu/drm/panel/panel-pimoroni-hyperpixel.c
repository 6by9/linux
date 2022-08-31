// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pinmoroni Hyperpixel TFT LCD drm_panel driver.
 *
 * Copyright (C) 2022 Raspberry Pi Ltd
 *
 * Derived from drivers/drm/gpu/panel/panel-ilitek-hyperpixel.c
 */

#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

struct hyperpixel {
	struct drm_panel panel;
	struct spi_device *spi;
	struct gpio_desc *reset;
	struct regulator *power;
	u32 bus_format;
};

#define HYPERPIXEL_DATA		BIT(8)

#define HYPERPIXEL_MAX_MSG_LEN	17

struct hyperpixel_msg {
	unsigned int len;
	u16 msg[HYPERPIXEL_MAX_MSG_LEN];
};

#define HYPERPIXEL_SET_REG_PARAM(reg, data)	\
	{					\
		.len = 2,			\
		.msg = {			\
			(reg),			\
			HYPERPIXEL_DATA | (data),	\
		},				\
	}

#define HYPERPIXEL_SET_REG_PARAM2(reg, data0, data1)	\
	{					\
		.len = 3,			\
		.msg = {			\
			(reg),			\
			HYPERPIXEL_DATA | (data0),	\
			HYPERPIXEL_DATA | (data1),	\
		},				\
	}

#define HYPERPIXEL_SET_REG_PARAM4(reg, data0, data1, data2, data3)	\
	{					\
		.len = 4,			\
		.msg = {			\
			(reg),			\
			HYPERPIXEL_DATA | (data0),	\
			HYPERPIXEL_DATA | (data1),	\
			HYPERPIXEL_DATA | (data2),	\
			HYPERPIXEL_DATA | (data3)	\
		},				\
	}

#define HYPERPIXEL_SET_REG(reg)	\
	{				\
		.len = 1,		\
		.msg = { (reg) },		\
	}

#define HYPERPIXEL_DELAY(ms)	\
	{				\
		.len = 1,		\
		.msg = { (ms) },		\
	}

static const struct hyperpixel_msg panel_init[] = {
	HYPERPIXEL_SET_REG(MIPI_DCS_EXIT_SLEEP_MODE),
	HYPERPIXEL_DELAY(120),
	HYPERPIXEL_SET_REG(MIPI_DCS_SOFT_RESET),
	HYPERPIXEL_DELAY(120),
	HYPERPIXEL_SET_REG_PARAM4(0xc1, 0xa8, 0xb1, 0x45, 0x04),
	HYPERPIXEL_SET_REG_PARAM2(0xc5, 0x80, 0x6c),
	HYPERPIXEL_SET_REG_PARAM2(0xc6, 0xbd, 0x84),
	HYPERPIXEL_SET_REG_PARAM2(0xc7, 0xbd, 0x84),
	HYPERPIXEL_SET_REG_PARAM(0xbd, 0x02),
	HYPERPIXEL_SET_REG(MIPI_DCS_EXIT_SLEEP_MODE),
	HYPERPIXEL_DELAY(120),
	HYPERPIXEL_SET_REG(HYPERPIXEL_DATA | 0x00),
	HYPERPIXEL_SET_REG(HYPERPIXEL_DATA | 0x00),
	HYPERPIXEL_SET_REG(HYPERPIXEL_DATA | 0x82),
	HYPERPIXEL_SET_REG_PARAM(MIPI_DCS_SET_GAMMA_CURVE, 0x08),
	{
		.len = 16,
		.msg = { 0xe0, 0x00, 0x04, 0x08, 0x0b, 0x0c, 0x0d, 0x0e,
			 0x00, 0x04, 0x08, 0x13, 0x14, 0x2f, 0x29, 0x24 }
	}, {
		.len = 17,
		.msg = { 0xe1, 0x00, 0x04, 0x08, 0x0b, 0x0c, 0x11, 0x0d,
			 0x0e, 0x00, 0x04, 0x08, 0x13, 0x14, 0x2f, 0x29, 0x24 }
	},
	HYPERPIXEL_SET_REG_PARAM(MIPI_DCS_SET_GAMMA_CURVE, 0x08),
	HYPERPIXEL_SET_REG_PARAM2(0xfd, 0x00, 0x08),
};

#define NUM_INIT_REGS ARRAY_SIZE(panel_init)

static inline struct hyperpixel *panel_to_hyperpixel(struct drm_panel *panel)
{
	return container_of(panel, struct hyperpixel, panel);
}

static int hyperpixel_write_msg(struct hyperpixel *ctx, const struct hyperpixel_msg *msg)
{
	struct spi_transfer xfer = { };
	struct spi_message spi;

	spi_message_init(&spi);

	xfer.tx_buf = msg->msg;
	xfer.bits_per_word = 9;
	xfer.len = sizeof(u16) * msg->len;

	spi_message_add_tail(&xfer, &spi);
	return spi_sync(ctx->spi, &spi);
}

static int hyperpixel_write_msg_list(struct hyperpixel *ctx,
				     const struct hyperpixel_msg msgs[],
				     unsigned int num_msgs)
{
	int ret, i;

	for (i = 0; i < num_msgs; i++) {
		if (!msgs[i].len) {
			msleep(msgs[i].msg[0]);
			continue;
		}

		ret = hyperpixel_write_msg(ctx, &msgs[i]);
		if (ret)
			break;
	}

	return ret;
}

static const struct drm_display_mode hyperpixel_480x800_mode = {
	.clock = 32000,
	.hdisplay = 800,
	.hsync_start = 800 + 50,
	.hsync_end = 800 + 50 + 20,
	.htotal = 800 + 50 + 20 + 50,
	.vdisplay = 480,
	.vsync_start = 480 + 3,
	.vsync_end = 480 + 3 + 2,
	.vtotal = 480 + 3 + 2 + 3,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static int hyperpixel_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct hyperpixel *ctx = panel_to_hyperpixel(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &hyperpixel_480x800_mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%ux@%u\n",
			hyperpixel_480x800_mode.hdisplay,
			hyperpixel_480x800_mode.vdisplay,
			drm_mode_vrefresh(&hyperpixel_480x800_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = 61;
	connector->display_info.height_mm = 103;
	drm_display_info_set_bus_formats(&connector->display_info,
					 &ctx->bus_format, 1);
	connector->display_info.bus_flags =
					DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE;

	return 1;
}

static int hyperpixel_prepare(struct drm_panel *panel)
{
	struct hyperpixel *ctx = panel_to_hyperpixel(panel);
	int ret;

	ret = regulator_enable(ctx->power);
	if (ret)
		return ret;

	ret = hyperpixel_write_msg_list(ctx, panel_init, NUM_INIT_REGS);

	return ret;
}

static int hyperpixel_enable(struct drm_panel *panel)
{
	struct hyperpixel *ctx = panel_to_hyperpixel(panel);
	const struct hyperpixel_msg msg = HYPERPIXEL_SET_REG(MIPI_DCS_SET_DISPLAY_ON);
	int ret;

	ret = hyperpixel_write_msg(ctx, &msg);

	return ret;
}

static int hyperpixel_disable(struct drm_panel *panel)
{
	struct hyperpixel *ctx = panel_to_hyperpixel(panel);
	const struct hyperpixel_msg msg = HYPERPIXEL_SET_REG(MIPI_DCS_SET_DISPLAY_OFF);
	int ret;

	ret = hyperpixel_write_msg(ctx, &msg);

	return ret;
}

static int hyperpixel_unprepare(struct drm_panel *panel)
{
	struct hyperpixel *ctx = panel_to_hyperpixel(panel);
	const struct hyperpixel_msg msg = HYPERPIXEL_SET_REG(MIPI_DCS_ENTER_SLEEP_MODE);
	int ret;

	ret = hyperpixel_write_msg(ctx, &msg);

	return ret;
}

static const struct drm_panel_funcs hyperpixel_drm_funcs = {
	.disable	= hyperpixel_disable,
	.enable		= hyperpixel_enable,
	.get_modes	= hyperpixel_get_modes,
	.prepare	= hyperpixel_prepare,
	.unprepare	= hyperpixel_unprepare,
};

static const struct of_device_id hyperpixel_of_match[] = {
	{
		.compatible = "pimoroni,hyperpixel35",
		.data = (void *)MEDIA_BUS_FMT_RGB666_1X24_CPADHI,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, hyperpixel_of_match);

static int hyperpixel_probe(struct spi_device *spi)
{
	const struct of_device_id *id;
	struct hyperpixel *ctx;
	int ret;

	ctx = devm_kzalloc(&spi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	id = of_match_node(hyperpixel_of_match, spi->dev.of_node);
	if (!id)
		return -ENODEV;

	ctx->bus_format = (u32)(uintptr_t)id->data;

	spi_set_drvdata(spi, ctx);
	ctx->spi = spi;

	drm_panel_init(&ctx->panel, &spi->dev, &hyperpixel_drm_funcs,
		       DRM_MODE_CONNECTOR_DPI);

	ctx->power = devm_regulator_get(&spi->dev, "power");
	if (IS_ERR(ctx->power))
		return PTR_ERR(ctx->power);

	ctx->reset = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset)) {
		dev_err(&spi->dev, "Couldn't get our reset line\n");
		return PTR_ERR(ctx->reset);
	}

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	return 0;
}

static int hyperpixel_remove(struct spi_device *spi)
{
	struct hyperpixel *ctx = spi_get_drvdata(spi);

	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct spi_device_id hyperpixel_ids[] = {
	{ "hyperpixel35", 0 },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(spi, hyperpixel_ids);

static struct spi_driver hyperpixel_driver = {
	.probe = hyperpixel_probe,
	.remove = hyperpixel_remove,
	.driver = {
		.name = "hyperpixel",
		.of_match_table = hyperpixel_of_match,
	},
	.id_table = hyperpixel_ids,
};
module_spi_driver(hyperpixel_driver);

MODULE_AUTHOR("Dave Stevenson <dave.stevenson@raspberrypi.com>");
MODULE_DESCRIPTION("Pimoroni Hyperpixel 3.5 LCD panel driver");
MODULE_LICENSE("GPL v2");
