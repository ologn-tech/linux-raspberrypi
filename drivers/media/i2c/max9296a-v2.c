
/*
 * Maxim MAX9296a Quad GMSL2 Deserializer Driver
 *
 * Copyright (C) 2021 Renesas Electronics Corporation
 * Copyright (C) 2021 Niklas Söderlund
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define MAX9296A_ID 0x94

#define MAX9296A_DPLL_FREQ 1000

enum max9296a_pattern {
	max9296a_PATTERN_CHECKERBOARD = 0,
	max9296a_PATTERN_GRADIENT,
};

struct max9296a_priv {
	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *gpiod_pwdn;

	bool cphy;
	struct v4l2_mbus_config_mipi_csi2 mipi;

	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrl_handler;
	struct media_pad pads[1];

	enum max9296a_pattern pattern;
};

static int max9296a_read(struct max9296a_priv *priv, int reg)
{
	/* Đọc giá trị từ 1 thanh ghi */
	int ret, val;

	ret = regmap_read(priv->regmap, reg, &val);
	if (ret) {
		dev_err(&priv->client->dev, "read 0x%04x failed\n", reg);
		return ret;
	}

	return val;
}

static int max9296a_write(struct max9296a_priv *priv, unsigned int reg, u8 val)
{
	/* Ghi giá trị vào 1 thanh ghi */
	int ret;

	ret = regmap_write(priv->regmap, reg, val);
	if (ret)
		dev_err(&priv->client->dev, "write 0x%04x failed\n", reg);

	return ret;
}

static int max9296a_update_bits(struct max9296a_priv *priv, unsigned int reg,
				u8 mask, u8 val)
{
	/* Cập nhật các bit trong 1 thanh ghi */
	int ret;

	ret = regmap_update_bits(priv->regmap, reg, mask, val);
	if (ret)
		dev_err(&priv->client->dev, "update 0x%04x failed\n", reg);

	return ret;
}

static int max9296a_write_bulk(struct max9296a_priv *priv, unsigned int reg,
			       const void *val, size_t val_count)
{
	/* Dùng khi cần ghi nhiều byte dữ liệu, ví dụ: cấu hình một bảng thông số hoặc gửi dữ liệu lớn vào chip. */
	int ret;

	ret = regmap_bulk_write(priv->regmap, reg, val, val_count);
	if (ret)
		dev_err(&priv->client->dev, "bulk write 0x%04x failed\n", reg);

	return ret;
}

static int max9296a_write_bulk_value(struct max9296a_priv *priv,
				     unsigned int reg, unsigned int val,
				     size_t val_count)
{
	/* Dùng để ghi các giá trị lớn (như tần số hoặc cấu hình đa byte) vào các thanh ghi liên tiếp. */
	unsigned int i;
	u8 values[4];

	for (i = 1; i <= val_count; i++)
		values[i - 1] = (val >> ((val_count - i) * 8)) & 0xff;

	return max9296a_write_bulk(priv, reg, &values, val_count);
}

static void max9296a_reset(struct max9296a_priv *priv)
{
	/* Đã sửa theo 96792A */
	max9296a_update_bits(priv, 0x10, 0x80, 0x80); 
	msleep(20); 
}

static void max9296a_mipi_enable(struct max9296a_priv *priv, bool enable)
{
    if (enable) {
        max9296a_update_bits(priv, 0x313, BIT(1), BIT(1));
        max9296a_update_bits(priv, 0x330, BIT(4) | BIT(3), BIT(4) | BIT(3)); 
    } else {
        max9296a_update_bits(priv, 0x313, BIT(1), 0x00);
        max9296a_update_bits(priv, 0x330, BIT(4) | BIT(3), 0x00);
    }
}

static void max9296a_mipi_configure(struct max9296a_priv *priv)
{
    unsigned int i;
    u8 phy5 = 0;

    /* Disable MIPI output before configuration */
    max9296a_mipi_enable(priv, false);

    /* Configure MIPI PHY mode and lane count */
    if (priv->cphy) {
        /* C-PHY: 3 lanes, 2x4 aggregation for both ports */
        max9296a_write(priv, 0x333, 0xC8); 
        /* Configure C-PHY timing */
        max9296a_write(priv, 0x336, 0x40); 
        max9296a_write(priv, 0x337, 0x80); 
    } else {
        /* D-PHY: 4 lanes, 2x4 aggregation for both ports */
        max9296a_write(priv, 0x333, 0xF0); 
    }

    /* Configure default lane mapping */
    max9296a_write(priv, 0x334, 0xE4); /* Default lane map for 2x4 */

    /* Configure lane polarities */
    unsigned int max_lanes = priv->cphy ? 3 : 4; /* C-PHY: 3 lanes, D-PHY: 4 lanes */
    for (i = 0; i < max_lanes; i++) {
        if (priv->mipi.lane_polarities[i]) {
				if (!priv->cphy && i == 0)
                phy5 |= BIT(5); /* Clock lane polarity */
            else
                phy5 |= BIT(i - (priv->cphy ? 0 : 1)); /* Data lanes */
        }
    }
    max9296a_write(priv, 0x335, phy5);

    /* Configure DPLL frequency for both ports (Registers 0x31D, 0x320) */
    u8 dpll_val = ((MAX9296A_DPLL_FREQ / 100) & 0x1F) | BIT(5); /* Enable DPLL */
    max9296a_update_bits(priv, 0x31D, 0x3F, dpll_val);
    max9296a_update_bits(priv, 0x320, 0x3F, dpll_val);

    /* Enable MIPI output for both ports (Register 0x330) */
    max9296a_update_bits(priv, 0x330, BIT(4) | BIT(3), BIT(4) | BIT(3));
}

static void max9296a_pattern_enable(struct max9296a_priv *priv, bool enable)
{
    const u32 h_active = 1920; 
    const u32 h_fp = 88;       
    const u32 h_sw = 44;       
    const u32 h_bp = 148;      
    const u32 h_tot = h_active + h_fp + h_sw + h_bp; 

    const u32 v_active = 1080; 
    const u32 v_fp = 4;        
    const u32 v_sw = 5;        
    const u32 v_bp = 36;       
    const u32 v_tot = v_active + v_fp + v_sw + v_bp; 

    if (!enable) {
        /* Disable video pattern generator (PATGEN_MODE = 0b00) */
        max9296a_write(priv, 0x241, 0x00);
        return;
    }

    /* Set PCLK to 75 MHz (DPLL configuration, adjust register as needed) */
    max9296a_write(priv, 0x031, 0x12); /* Example: DPLL for 75 MHz, see datasheet page 186 */

    /* Configure Video Timing Generator for 1920x1080@30fps */
    max9296a_write_bulk_value(priv, 0x242, 0, 3); /* HSYNC start */
    max9296a_write_bulk_value(priv, 0x245, v_sw * h_tot, 3); /* VSYNC width: 5 * 2200 */
    max9296a_write_bulk_value(priv, 0x248, (v_active + v_fp + v_bp) * h_tot, 3); /* VSYNC start: (1080+4+36) * 2200 */

    max9296a_write_bulk_value(priv, 0x24B, 0, 3); /* DE start (horizontal) */
    max9296a_write_bulk_value(priv, 0x24E, h_sw, 2); /* HSYNC width: 44 */
    max9296a_write_bulk_value(priv, 0x250, h_active + h_fp + h_bp, 2); /* DE start: 1920+88+148 */
    max9296a_write_bulk_value(priv, 0x252, v_tot, 3); /* Total lines: 1125 (fixed to 3 bytes) */
    max9296a_write_bulk_value(priv, 0x254, h_tot * (v_sw + v_bp) + (h_sw + h_bp), 3); /* DE start (vertical): 2200 * (5+36) + (44+148) */
    max9296a_write_bulk_value(priv, 0x257, h_active, 2); /* Active width: 1920 */
    max9296a_write_bulk_value(priv, 0x259, h_fp + h_sw + h_bp, 2); /* Blank width: 88+44+148 */
    max9296a_write_bulk_value(priv, 0x25B, v_active, 2); /* Active height: 1080 */

    /* Enable VS, HS, and DE in free-running mode */
    max9296a_write(priv, 0x240, 0xFB); /* VTG free-running, all signals enabled */

    /* Configure Video Pattern Generator */
    if (priv->pattern == max9296a_PATTERN_CHECKERBOARD) {
        /* Set checkerboard pattern size (60x60 pixels) */
        max9296a_write(priv, 0x264, 0x3C); /* Checker width */
        max9296a_write(priv, 0x265, 0x3C); /* Checker height */
        max9296a_write(priv, 0x266, 0x3C); /* Checker step */

        /* Set checkerboard colors (RGB: yellow and cyan) */
        max9296a_write_bulk_value(priv, 0x25E, 0xFECC00, 3); /* Color 1: Yellow */
        max9296a_write_bulk_value(priv, 0x261, 0x006AA7, 3); /* Color 2: Cyan */

        /* Enable checkerboard pattern (PATGEN_MODE = 0b10) */
        max9296a_write(priv, 0x241, 0x10);
    } else {
        /* Set gradient increment */
        max9296a_write(priv, 0x25D, 0x10); /* Gradient step size */

        /* Enable gradient pattern (PATGEN_MODE = 0b01) */
        max9296a_write(priv, 0x241, 0x20);
    }
}

static int max9296a_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct max9296a_priv *priv = v4l2_get_subdevdata(sd);

	if (enable) {
		max9296a_pattern_enable(priv, true);
		max9296a_mipi_enable(priv, true);
	} else {
		max9296a_mipi_enable(priv, false);
		max9296a_pattern_enable(priv, false);
	}
	
	return 0;
}

static const struct v4l2_subdev_video_ops max9296a_video_ops = {
	.s_stream = max9296a_s_stream,
};

static int max9296a_init_state(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	static const struct v4l2_mbus_framefmt default_fmt = {
		.width          = 1920,
		.height         = 1080,
		.code           = MEDIA_BUS_FMT_RGB888_1X24,
		.colorspace     = V4L2_COLORSPACE_SRGB,
		.field          = V4L2_FIELD_NONE,
		.ycbcr_enc      = V4L2_YCBCR_ENC_DEFAULT,
		.quantization   = V4L2_QUANTIZATION_DEFAULT,
		.xfer_func      = V4L2_XFER_FUNC_DEFAULT,
	};
	struct v4l2_mbus_framefmt *fmt;

	fmt = v4l2_subdev_state_get_format(state, 0);
	*fmt = default_fmt;

	return 0;
}

static const struct v4l2_subdev_internal_ops max9296a_internal_ops = {
	.init_state = max9296a_init_state,
};

static const struct v4l2_subdev_pad_ops max9296a_pad_ops = {
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = v4l2_subdev_get_fmt,
};

static const struct v4l2_subdev_ops max9296a_subdev_ops = {
	.video = &max9296a_video_ops,
	.pad = &max9296a_pad_ops,
};

static const char * const max9296a_test_pattern[] = {
	"Checkerboard",
	"Gradient",
};

static int max9296a_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct max9296a_priv *priv =
		container_of(ctrl->handler, struct max9296a_priv, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		priv->pattern = ctrl->val ?
			max9296a_PATTERN_GRADIENT :
			max9296a_PATTERN_CHECKERBOARD;
		break;
	}
	return 0;
}

static const struct v4l2_ctrl_ops max9296a_ctrl_ops = {
	.s_ctrl = max9296a_s_ctrl,
};

static int max9296a_v4l2_register(struct max9296a_priv *priv)
{
	long pixel_rate;
	int ret;

	priv->sd.internal_ops = &max9296a_internal_ops;
	v4l2_i2c_subdev_init(&priv->sd, priv->client, &max9296a_subdev_ops);
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;

	v4l2_ctrl_handler_init(&priv->ctrl_handler, 2);

	/*
	 * TODO: Once V4L2_CID_LINK_FREQ is changed from a menu control to an
	 * INT64 control it should be used here instead of V4L2_CID_PIXEL_RATE.
	 */
	pixel_rate = max9296a_DPLL_FREQ / priv->mipi.num_data_lanes * 1000000;
	v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_PIXEL_RATE,
			  pixel_rate, pixel_rate, 1, pixel_rate);

	v4l2_ctrl_new_std_menu_items(&priv->ctrl_handler, &max9296a_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(max9296a_test_pattern) - 1,
				     0, 0, max9296a_test_pattern);

	priv->sd.ctrl_handler = &priv->ctrl_handler;
	ret = priv->ctrl_handler.error;
	if (ret)
		goto error;

	priv->pads[0].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&priv->sd.entity, 1, priv->pads);
	if (ret)
		goto error;

	v4l2_set_subdevdata(&priv->sd, priv);

	priv->sd.state_lock = priv->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&priv->sd);
	if (ret)
		goto error;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret < 0) {
		dev_err(&priv->client->dev, "Unable to register subdevice\n");
		goto error;
	}

	return 0;
error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return ret;
}

static int max9296a_parse_dt(struct max9296a_priv *priv)
{
	struct fwnode_handle *ep;
	struct v4l2_fwnode_endpoint v4l2_ep = {
		.bus_type = V4L2_MBUS_UNKNOWN,
	};
	unsigned int supported_lanes;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(&priv->client->dev), 4,
					     0, 0);
	if (!ep) {
		dev_err(&priv->client->dev, "Not connected to subdevice\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(ep, &v4l2_ep);
	fwnode_handle_put(ep);
	if (ret) {
		dev_err(&priv->client->dev, "Could not parse v4l2 endpoint\n");
		return -EINVAL;
	}

	switch (v4l2_ep.bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
		supported_lanes = 4;
		priv->cphy = false;
		break;
	case V4L2_MBUS_CSI2_CPHY:
		supported_lanes = 3;
		priv->cphy = true;
		break;
	default:
		dev_err(&priv->client->dev, "Unsupported bus-type %u\n",
			v4l2_ep.bus_type);
		return -EINVAL;
	}

	if (v4l2_ep.bus.mipi_csi2.num_data_lanes != supported_lanes) {
		dev_err(&priv->client->dev, "Only %u data lanes supported\n",
			supported_lanes);
		return -EINVAL;
	}

	priv->mipi = v4l2_ep.bus.mipi_csi2;

	return 0;
}

static const struct regmap_config max9296a_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x52D6,
};

static int max9296a_probe(struct i2c_client *client)
{
	struct max9296a_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	priv->regmap = devm_regmap_init_i2c(client, &max9296a_i2c_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->gpiod_pwdn = devm_gpiod_get_optional(&client->dev, "enable",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(priv->gpiod_pwdn))
		return PTR_ERR(priv->gpiod_pwdn);

	gpiod_set_consumer_name(priv->gpiod_pwdn, "max9296a-pwdn");
	gpiod_set_value_cansleep(priv->gpiod_pwdn, 1);

	if (priv->gpiod_pwdn)
		usleep_range(4000, 5000);

	if (max9296a_read(priv, 0x0D) != max9296a_ID)              /* Này đã sửa theo datasheet */
		return -ENODEV;

	max9296a_reset(priv);

	ret = max9296a_parse_dt(priv);
	if (ret)
		return ret;

	max9296a_mipi_configure(priv);

	return max9296a_v4l2_register(priv);
}

static void max9296a_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct max9296a_priv *priv = container_of(sd, struct max9296a_priv, sd);

	v4l2_async_unregister_subdev(&priv->sd);

	gpiod_set_value_cansleep(priv->gpiod_pwdn, 0);
}

static const struct of_device_id max9296a_of_table[] = {
	{ .compatible = "maxim,max9296a,v2" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, max9296a_of_table);

static struct i2c_driver max9296a_i2c_driver = {
	.driver	= {
		.name = "max9296a",
		.of_match_table	= of_match_ptr(max9296a_of_table),
	},
	.probe = max9296a_probe,
	.remove = max9296a_remove,
};

module_i2c_driver(max9296a_i2c_driver);

MODULE_DESCRIPTION("Maxim max9296aA Quad GMSL2 Deserializer Driver");
MODULE_AUTHOR("Cuong Le <cuong.le@ologn.tech>");
MODULE_LICENSE("GPL");
