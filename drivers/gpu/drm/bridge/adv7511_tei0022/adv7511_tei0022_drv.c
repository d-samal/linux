/*
 * Analog Devices ADV7511 HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>

#include "adv7511.h"

/* ADI recommended values for proper operation. */
static const struct reg_sequence adv7511_fixed_registers[] = {
	{ 0x98, 0x03 },
	{ 0x9a, 0xe0 },
	{ 0x9c, 0x30 },
	{ 0x9d, 0x61 },
	{ 0xa2, 0xa4 },
	{ 0xa3, 0xa4 },
	{ 0xe0, 0xd0 },
	{ 0xf9, 0x00 },
	{ 0x55, 0x02 },
};

/* -----------------------------------------------------------------------------
 * Register access
 */

static const uint8_t adv7511_register_defaults[] = {
	0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 00 */
	0x00, 0x00, 0x01, 0x0e, 0xbc, 0x18, 0x01, 0x13,
	0x25, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 10 */
	0x46, 0x62, 0x04, 0xa8, 0x00, 0x00, 0x1c, 0x84,
	0x1c, 0xbf, 0x04, 0xa8, 0x1e, 0x70, 0x02, 0x1e, /* 20 */
	0x00, 0x00, 0x04, 0xa8, 0x08, 0x12, 0x1b, 0xac,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 30 */
	0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0xb0,
	0x00, 0x50, 0x90, 0x7e, 0x79, 0x70, 0x00, 0x00, /* 40 */
	0x00, 0xa8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x02, 0x0d, 0x00, 0x00, 0x00, 0x00, /* 50 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 60 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 70 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 80 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, /* 90 */
	0x0b, 0x02, 0x00, 0x18, 0x5a, 0x60, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x80, 0x08, 0x04, 0x00, 0x00, /* a0 */
	0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x14,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* b0 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* c0 */
	0x00, 0x03, 0x00, 0x00, 0x02, 0x00, 0x01, 0x04,
	0x30, 0xff, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, /* d0 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x01,
	0x80, 0x75, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, /* e0 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0x11, 0x00, /* f0 */
	0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static bool adv7511_register_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ADV7511_REG_CHIP_REVISION:
	case ADV7511_REG_SPDIF_FREQ:
	case ADV7511_REG_CTS_AUTOMATIC1:
	case ADV7511_REG_CTS_AUTOMATIC2:
	case ADV7511_REG_VIC_DETECTED:
	case ADV7511_REG_VIC_SEND:
	case ADV7511_REG_AUX_VIC_DETECTED:
	case ADV7511_REG_STATUS:
	case ADV7511_REG_GC(1):
	case ADV7511_REG_INT(0):
	case ADV7511_REG_INT(1):
	case ADV7511_REG_PLL_STATUS:
	case ADV7511_REG_AN(0):
	case ADV7511_REG_AN(1):
	case ADV7511_REG_AN(2):
	case ADV7511_REG_AN(3):
	case ADV7511_REG_AN(4):
	case ADV7511_REG_AN(5):
	case ADV7511_REG_AN(6):
	case ADV7511_REG_AN(7):
	case ADV7511_REG_HDCP_STATUS:
	case ADV7511_REG_BCAPS:
	case ADV7511_REG_BKSV(0):
	case ADV7511_REG_BKSV(1):
	case ADV7511_REG_BKSV(2):
	case ADV7511_REG_BKSV(3):
	case ADV7511_REG_BKSV(4):
	case ADV7511_REG_DDC_STATUS:
	case ADV7511_REG_EDID_READ_CTRL:
	case ADV7511_REG_BSTATUS(0):
	case ADV7511_REG_BSTATUS(1):
	case ADV7511_REG_CHIP_ID_HIGH:
	case ADV7511_REG_CHIP_ID_LOW:
		return true;
	}

	return false;
}

static const struct regmap_config adv7511_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0xff,
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults_raw = adv7511_register_defaults,
	.num_reg_defaults_raw = ARRAY_SIZE(adv7511_register_defaults),

	.volatile_reg = adv7511_register_volatile,
};

/* -----------------------------------------------------------------------------
 * Hardware configuration
 */

static void adv7511_set_colormap(struct adv7511 *adv7511, bool enable,
				 const uint16_t *coeff,
				 unsigned int scaling_factor)
{
	unsigned int i;

	regmap_update_bits(adv7511->regmap, ADV7511_REG_CSC_UPPER(1),
			   ADV7511_CSC_UPDATE_MODE, ADV7511_CSC_UPDATE_MODE);

	if (enable) {
		for (i = 0; i < 12; i) {
			regmap_update_bits(adv7511->regmap,
					   ADV7511_REG_CSC_UPPER(i),
					   0x1f, coeff[i] >> 8);
			regmap_write(adv7511->regmap,
				     ADV7511_REG_CSC_LOWER(i),
				     coeff[i] & 0xff);
		}
	}

	if (enable)
		regmap_update_bits(adv7511->regmap, ADV7511_REG_CSC_UPPER(0),
				   0xe0, 0x80 | (scaling_factor << 5));
	else
		regmap_update_bits(adv7511->regmap, ADV7511_REG_CSC_UPPER(0),
				   0x80, 0x00);

	regmap_update_bits(adv7511->regmap, ADV7511_REG_CSC_UPPER(1),
			   ADV7511_CSC_UPDATE_MODE, 0);
}

static int adv7511_packet_enable(struct adv7511 *adv7511, unsigned int packet)
{
	if (packet & 0xff)
		regmap_update_bits(adv7511->regmap, ADV7511_REG_PACKET_ENABLE0,
				   packet, 0xff);

	if (packet & 0xff00) {
		packet >>= 8;
		regmap_update_bits(adv7511->regmap, ADV7511_REG_PACKET_ENABLE1,
				   packet, 0xff);
	}

	return 0;
}

static int adv7511_packet_disable(struct adv7511 *adv7511, unsigned int packet)
{
	if (packet & 0xff)
		regmap_update_bits(adv7511->regmap, ADV7511_REG_PACKET_ENABLE0,
				   packet, 0x00);

	if (packet & 0xff00) {
		packet >>= 8;
		regmap_update_bits(adv7511->regmap, ADV7511_REG_PACKET_ENABLE1,
				   packet, 0x00);
	}

	return 0;
}

/* Coefficients for adv7511 color space conversion */
static const uint16_t adv7511_csc_ycbcr_to_rgb[] = {
	0x0734, 0x04ad, 0x0000, 0x1c1b,
	0x1ddc, 0x04ad, 0x1f24, 0x0135,
	0x0000, 0x04ad, 0x087c, 0x1b77,
};

static void adv7511_set_config_csc(struct adv7511 *adv7511,
                                   bool rgb)
{
        struct adv7511_video_config config;
        bool output_format_422, output_format_ycbcr;
        unsigned int mode;
        uint8_t infoframe[17];

	config.hdmi_mode = true;

        hdmi_avi_infoframe_init(&config.avi_infoframe);

        config.avi_infoframe.scan_mode = HDMI_SCAN_MODE_UNDERSCAN;

        if (rgb) {
                config.csc_enable = false;
                config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
        } else {
                config.csc_scaling_factor = ADV7511_CSC_SCALING_4;
                config.csc_coefficents = adv7511_csc_ycbcr_to_rgb;
        }

        if (config.hdmi_mode) {
                mode = ADV7511_HDMI_CFG_MODE_HDMI;

                switch (config.avi_infoframe.colorspace) {
                case HDMI_COLORSPACE_YUV444:
                        output_format_422 = false;
                        output_format_ycbcr = true;
                        config.csc_enable = true;
                        config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
                        break;
                case HDMI_COLORSPACE_YUV422:
                        output_format_422 = true;
                        output_format_ycbcr = true;
                        config.csc_enable = false;
                        config.avi_infoframe.colorspace =
                                HDMI_COLORSPACE_YUV422;
                        break;
                default:
                        output_format_422 = false;
                        output_format_ycbcr = false;
                        break;
                }
        } else {
                mode = ADV7511_HDMI_CFG_MODE_DVI;
                output_format_422 = false;
                output_format_ycbcr = false;
        }

        adv7511_packet_disable(adv7511, ADV7511_PACKET_ENABLE_AVI_INFOFRAME);

        adv7511_set_colormap(adv7511, config.csc_enable,
                             config.csc_coefficents,
                             config.csc_scaling_factor);

        regmap_update_bits(adv7511->regmap, ADV7511_REG_VIDEO_INPUT_CFG1, 0x81,
                           (output_format_422 << 7) | output_format_ycbcr);

        regmap_update_bits(adv7511->regmap, ADV7511_REG_HDCP_HDMI_CFG,
                           ADV7511_HDMI_CFG_MODE_MASK, mode);

        hdmi_avi_infoframe_pack(&config.avi_infoframe, infoframe,
                                sizeof(infoframe));

        /* The AVI infoframe id is not configurable */
        regmap_bulk_write(adv7511->regmap, ADV7511_REG_AVI_INFOFRAME_VERSION,
                          infoframe + 1, sizeof(infoframe) - 1);

        adv7511_packet_enable(adv7511, ADV7511_PACKET_ENABLE_AVI_INFOFRAME);
}


static void adv7511_set_link_config(struct adv7511 *adv7511,
				    const struct adv7511_link_config *config)
{
	/*
	 * The input style values documented in the datasheet don't match the
	 * hardware register field values :-(
	 */
	static const unsigned int input_styles[4] = { 0, 2, 1, 3 };

	unsigned int clock_delay;
	unsigned int color_depth;
	unsigned int input_id;

	clock_delay = (config->clock_delay + 1200) / 400;
	color_depth = config->input_color_depth == 8 ? 3
		    : (config->input_color_depth == 10 ? 1 : 2);

	/* TODO Support input ID 6 */
	if (config->input_colorspace != HDMI_COLORSPACE_YUV422)
		input_id = config->input_clock == ADV7511_INPUT_CLOCK_DDR
			 ? 5 : 0;
	else if (config->input_clock == ADV7511_INPUT_CLOCK_DDR)
		input_id = config->embedded_sync ? 8 : 7;
	else if (config->input_clock == ADV7511_INPUT_CLOCK_2X)
		input_id = config->embedded_sync ? 4 : 3;
	else
		input_id = config->embedded_sync ? 2 : 1;

	regmap_update_bits(adv7511->regmap, ADV7511_REG_I2C_FREQ_ID_CFG, 0xf,
			   input_id);
	regmap_update_bits(adv7511->regmap, ADV7511_REG_VIDEO_INPUT_CFG1, 0x7e,
			   (color_depth << 4) |
			   (input_styles[config->input_style] << 2));
	regmap_write(adv7511->regmap, ADV7511_REG_VIDEO_INPUT_CFG2,
		     config->input_justification << 3);
	regmap_write(adv7511->regmap, ADV7511_REG_TIMING_GEN_SEQ,
		     config->sync_pulse << 2);

	regmap_write(adv7511->regmap, 0xba, clock_delay << 5);

	adv7511->embedded_sync = config->embedded_sync;
	adv7511->hsync_polarity = config->hsync_polarity;
	adv7511->vsync_polarity = config->vsync_polarity;
	adv7511->rgb = config->input_colorspace == HDMI_COLORSPACE_RGB;

	adv7511_set_config_csc(adv7511, adv7511->rgb);
}

static void __adv7511_power_on(struct adv7511 *adv7511)
{
	regmap_update_bits(adv7511->regmap, ADV7511_REG_POWER,
			   ADV7511_POWER_POWER_DOWN, 0);
	if (adv7511->i2c_main->irq) {
		/*
		 * Documentation says the INT_ENABLE registers are reset in
		 * POWER_DOWN mode. My 7511w preserved the bits, however.
		 * Still, let's be safe and stick to the documentation.
		 */
		regmap_write(adv7511->regmap, ADV7511_REG_INT_ENABLE(0),
			     ADV7511_INT0_HPD);
		regmap_write(adv7511->regmap, ADV7511_REG_INT_ENABLE(1),
			     ADV7511_INT1_DDC_ERROR);
	}

	/*
	 * Per spec it is allowed to pulse the HPD signal to indicate that the
	 * EDID information has changed. Some monitors do this when they wakeup
	 * from standby or are enabled. When the HPD goes low the adv7511 is
	 * reset and the outputs are disabled which might cause the monitor to
	 * go to standby again. To avoid this we ignore the HPD pin for the
	 * first few seconds after enabling the output.
	 */
	regmap_update_bits(adv7511->regmap, ADV7511_REG_POWER2,
			   ADV7511_REG_POWER2_HPD_SRC_MASK,
			   ADV7511_REG_POWER2_HPD_SRC_NONE);
}

static void adv7511_power_on(struct adv7511 *adv7511)
{
	__adv7511_power_on(adv7511);

	/*
	 * Most of the registers are reset during power down or when HPD is low.
	 */
	regcache_sync(adv7511->regmap);

	adv7511->powered = true;
}

static void __adv7511_power_off(struct adv7511 *adv7511)
{
	/* TODO: setup additional power down modes */
	regmap_update_bits(adv7511->regmap, ADV7511_REG_POWER,
			   ADV7511_POWER_POWER_DOWN,
			   ADV7511_POWER_POWER_DOWN);
	regcache_mark_dirty(adv7511->regmap);
}

static void adv7511_power_off(struct adv7511 *adv7511)
{
	__adv7511_power_off(adv7511);
	adv7511->powered = false;
}

/* -----------------------------------------------------------------------------
 * Interrupt and hotplug detection
 */

static void adv7511_hpd_work(struct work_struct *work)
{
	struct adv7511 *adv7511 = container_of(work, struct adv7511, hpd_work);
	unsigned int val;
	int ret;

	ret = regmap_read(adv7511->regmap, ADV7511_REG_STATUS, &val);
	if (ret < 0)
 		adv7511_power_off(adv7511);
	else if (val & ADV7511_STATUS_HPD)
 		adv7511_power_on(adv7511);
	else
 		adv7511_power_off(adv7511);
}


/* -----------------------------------------------------------------------------
 * Probe & remove
 */

static const char * const adv7511_supply_names[] = {
	"avdd",
	"dvdd",
	"pvdd",
	"bgvdd",
	"dvdd-3v",
};

static int adv7511_init_regulators(struct adv7511 *adv)
{
	struct device *dev = &adv->i2c_main->dev;
	const char * const *supply_names;
	unsigned int i;
	int ret;

	supply_names = adv7511_supply_names;
	adv->num_supplies = ARRAY_SIZE(adv7511_supply_names);

	adv->supplies = devm_kcalloc(dev, adv->num_supplies,
				     sizeof(*adv->supplies), GFP_KERNEL);
	if (!adv->supplies)
		return -ENOMEM;

	for (i = 0; i < adv->num_supplies; i)
		adv->supplies[i].supply = supply_names[i];

	ret = devm_regulator_bulk_get(dev, adv->num_supplies, adv->supplies);
	if (ret)
		return ret;

	printk("Here I am probing the adv7511_altvip driver 3: \n");  //sjk

	return regulator_bulk_enable(adv->num_supplies, adv->supplies);
}

static void adv7511_uninit_regulators(struct adv7511 *adv)
{
	regulator_bulk_disable(adv->num_supplies, adv->supplies);
}

static int adv7511_parse_dt(struct device_node *np,
			    struct adv7511_link_config *config)
{
	const char *str;
	int ret;

	of_property_read_u32(np, "adi,input-depth", &config->input_color_depth);
	if (config->input_color_depth != 8 && config->input_color_depth != 10 &&
	    config->input_color_depth != 12)
		return -EINVAL;

	ret = of_property_read_string(np, "adi,input-colorspace", &str);
	if (ret < 0)
		return ret;

	if (!strcmp(str, "rgb"))
		config->input_colorspace = HDMI_COLORSPACE_RGB;
	else if (!strcmp(str, "yuv422"))
		config->input_colorspace = HDMI_COLORSPACE_YUV422;
	else if (!strcmp(str, "yuv444"))
		config->input_colorspace = HDMI_COLORSPACE_YUV444;
	else
		return -EINVAL;

	ret = of_property_read_string(np, "adi,input-clock", &str);
	if (ret < 0)
		return ret;

	if (!strcmp(str, "1x"))
		config->input_clock = ADV7511_INPUT_CLOCK_1X;
	else if (!strcmp(str, "2x"))
		config->input_clock = ADV7511_INPUT_CLOCK_2X;
	else if (!strcmp(str, "ddr"))
		config->input_clock = ADV7511_INPUT_CLOCK_DDR;
	else
		return -EINVAL;

	if (config->input_colorspace == HDMI_COLORSPACE_YUV422 ||
	    config->input_clock != ADV7511_INPUT_CLOCK_1X) {
		ret = of_property_read_u32(np, "adi,input-style",
					   &config->input_style);
		if (ret)
			return ret;

		if (config->input_style < 1 || config->input_style > 3)
			return -EINVAL;

		ret = of_property_read_string(np, "adi,input-justification",
					      &str);
		if (ret < 0)
			return ret;

		if (!strcmp(str, "left"))
			config->input_justification =
				ADV7511_INPUT_JUSTIFICATION_LEFT;
		else if (!strcmp(str, "evenly"))
			config->input_justification =
				ADV7511_INPUT_JUSTIFICATION_EVENLY;
		else if (!strcmp(str, "right"))
			config->input_justification =
				ADV7511_INPUT_JUSTIFICATION_RIGHT;
		else
			return -EINVAL;

	} else {
		config->input_style = 1;
		config->input_justification = ADV7511_INPUT_JUSTIFICATION_LEFT;
	}

	of_property_read_u32(np, "adi,clock-delay", &config->clock_delay);
	if (config->clock_delay < -1200 || config->clock_delay > 1600)
		return -EINVAL;

	config->embedded_sync = of_property_read_bool(np, "adi,embedded-sync");

	/* Hardcode the sync pulse configurations for now. */
	config->sync_pulse = ADV7511_INPUT_SYNC_PULSE_NONE;
	config->vsync_polarity = ADV7511_SYNC_POLARITY_PASSTHROUGH;
	config->hsync_polarity = ADV7511_SYNC_POLARITY_PASSTHROUGH;

	return 0;
}

static int adv7511_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct adv7511_link_config link_config;
	struct adv7511 *adv7511;
	struct device *dev = &i2c->dev;
	unsigned int main_i2c_addr = i2c->addr << 1;
	unsigned int edid_i2c_addr = main_i2c_addr + 4;
	unsigned int val;
	int ret;

	printk("Here I am probing the adv7511_altvip driver: \n");  //sjk

	if (!dev->of_node)
		return -EINVAL;

	adv7511 = devm_kzalloc(dev, sizeof(*adv7511), GFP_KERNEL);
	if (!adv7511)
		return -ENOMEM;

	adv7511->i2c_main = i2c;
	adv7511->powered = false;

	memset(&link_config, 0, sizeof(link_config));

	ret = adv7511_parse_dt(dev->of_node, &link_config);
	if (ret)
		return ret;
	printk("Here I am probing the adv7511_altvip driver 2: \n");  //sjk


	/*ret = adv7511_init_regulators(adv7511);
	if (ret) {
		dev_err(dev, "failed to init regulators\n");
		return ret;
	}*/ //sjk

	/*
	 * The power down GPIO is optional. If present, toggle it from active to
	 * inactive to wake up the encoder.
	 */
	/*adv7511->gpio_pd = devm_gpiod_get_optional(dev, "pd", GPIOD_OUT_HIGH);
	if (IS_ERR(adv7511->gpio_pd)) {
		ret = PTR_ERR(adv7511->gpio_pd);
		goto uninit_regulators;
	}

	if (adv7511->gpio_pd) {
		mdelay(5);
		gpiod_set_value_cansleep(adv7511->gpio_pd, 0);
	} */ //sjk

	adv7511->regmap = devm_regmap_init_i2c(i2c, &adv7511_regmap_config);

	//printk("Here I am probing the adv7511_altvip driver 2: \n");  //sjk

	if (IS_ERR(adv7511->regmap)) {
		ret = PTR_ERR(adv7511->regmap);
		goto uninit_regulators;
	}

	ret = regmap_read(adv7511->regmap, ADV7511_REG_CHIP_REVISION, &val);

	/*if (ret)
		goto uninit_regulators;
	dev_dbg(dev, "Rev. %d\n", val); */ //sjk

	ret = regmap_register_patch(adv7511->regmap,
			adv7511_fixed_registers,
			ARRAY_SIZE(adv7511_fixed_registers));
	if (ret)
		goto uninit_regulators;

	regmap_write(adv7511->regmap, ADV7511_REG_EDID_I2C_ADDR, edid_i2c_addr);
	regmap_write(adv7511->regmap, ADV7511_REG_PACKET_I2C_ADDR,
		     main_i2c_addr - 0xa);
	regmap_write(adv7511->regmap, ADV7511_REG_CEC_I2C_ADDR,
		     main_i2c_addr - 2);

	adv7511_packet_disable(adv7511, 0xffff);

	INIT_WORK(&adv7511->hpd_work, adv7511_hpd_work);

	/* CEC is unused for now */
	regmap_write(adv7511->regmap, ADV7511_REG_CEC_CTRL,
		     ADV7511_CEC_CTRL_POWER_DOWN);

	i2c_set_clientdata(i2c, adv7511);

	adv7511_set_link_config(adv7511, &link_config);

	adv7511_packet_enable(adv7511, ADV7511_PACKET_ENABLE_AVI_INFOFRAME);

	adv7511_power_on(adv7511);

	return 0;

uninit_regulators:
	adv7511_uninit_regulators(adv7511);

	return ret;
}

static int adv7511_remove(struct i2c_client *i2c)
{
	struct adv7511 *adv7511 = i2c_get_clientdata(i2c);

	adv7511_uninit_regulators(adv7511);

	return 0;
}

/**
 * I2C Device Table -
 *
 * name - Name of the actual device/chip.
 * driver_data - Driver data
 */
static const struct i2c_device_id adv7511_i2c_ids[] = {
	{ "adv7511", 0},
	{ "adv7511w", 0},
	{ "adv7513", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, adv7511_i2c_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id adv7511_of_ids[] = {
	{ .compatible = "adi,adv7511"},
	{ .compatible = "adi,adv7511w"},
	{ .compatible = "adi,adv7513"},
	{ }
};
MODULE_DEVICE_TABLE(of, adv7511_of_ids);
#endif

static struct i2c_driver adv7511_driver = {
	.driver = {
		.name = "adv7511",
		.of_match_table = adv7511_of_ids,
	},
	.id_table = adv7511_i2c_ids,
	.probe = adv7511_probe,
	.remove = adv7511_remove,
};

module_i2c_driver(adv7511_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("ADV7511 HDMI transmitter driver");
MODULE_LICENSE("GPL");
