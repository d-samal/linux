/*
 * Analog Devices AD738x Differential Input, Simultaneous Sampling,
 * 16/14/12-BIT, SAR ADC driver
 *
 * Copyright 2017 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/gpio.h>


#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/spi/spi-engine.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#include "cf_axi_adc.h"

#define SUCCESS 0
#define FAILURE 1

#define AD469x_OUTPUT_MODE_TWOS_COMPLEMENT	0x01

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/* AD469x registers */
#define AD469x_REG_IF_CONFIG_A		0x000
#define AD469x_REG_IF_CONFIG_B		0x001
#define AD469x_REG_DEVICE_TYPE		0x003
#define AD469x_REG_DEVICE_ID_L		0x004
#define AD469x_REG_DEVICE_ID_H		0x005
#define AD469x_REG_SCRATCH_PAD		0x00A
#define AD469x_REG_VENDOR_L		0x00C
#define AD469x_REG_VENDOR_H		0x00D
#define AD469x_REG_LOOP_MODE		0x00E
#define AD469x_REG_IF_CONFIG_C		0x010
#define AD469x_REG_IF_STATUS		0x011
#define AD469x_REG_STATUS		0x014
#define AD469x_REG_ALERT_STATUS1	0x015
#define AD469x_REG_ALERT_STATUS2	0x016
#define AD469x_REG_ALERT_STATUS3	0x017
#define AD469x_REG_ALERT_STATUS4	0x018
#define AD469x_REG_CLAMP_STATUS1	0x01A
#define AD469x_REG_CLAMP_STATUS2	0x01B
#define AD469x_REG_SETUP		0x020
#define AD469x_REG_REF_CTRL		0x021
#define AD469x_REG_SEQ_CTRL		0x022
#define AD469x_REG_AC_CTRL		0x023
#define AD469x_REG_STD_SEQ_CONFIG	0x024
#define AD469x_REG_GPIO_CTRL		0x026
#define AD469x_REG_GP_MODE		0x027
#define AD469x_REG_GPIO_STATE		0x028
#define AD469x_REG_TEMP_CTRL		0x029
#define AD469x_REG_CONFIG_IN(x)		((x & 0x0F) | 0x30)
#define AD469x_REG_AS_SLOT(x)		((x & 0x7F) | 0x100)

/* 5-bit SDI Conversion Mode Commands */
#define AD469x_CMD_REG_CONFIG_MODE		(0x0A << 3)
#define AD469x_CMD_SEL_TEMP_SNSOR_CH		(0x0F << 3)
#define AD469x_CMD_CONFIG_CH_SEL(x)		((0x10 | (0x0F & x)) << 3)

/* AD469x_REG_SETUP */
#define AD469x_SETUP_IF_MODE_MASK		(0x01 << 2)
#define AD469x_SETUP_IF_MODE_CONV		(0x01 << 2)
#define AD469x_SETUP_CYC_CTRL_MASK		(0x01 << 1)
#define AD469x_SETUP_CYC_CTRL_SINGLE(x)		((x & 0x01) << 1)



/* AD469x_REG_GP_MODE */
#define AD469x_GP_MODE_BUSY_GP_EN_MASK		(0x01 << 1)
#define AD469x_GP_MODE_BUSY_GP_EN(x)		((x & 0x01) << 1)
#define AD469x_GP_MODE_BUSY_GP_SEL_MASK		(0x01 << 4)
#define AD469x_GP_MODE_BUSY_GP_SEL(x)		((x & 0x01) << 4)

/* AD469x_REG_SEQ_CTRL */
#define AD469x_SEQ_CTRL_STD_SEQ_EN_MASK		(0x01 << 7)
#define AD469x_SEQ_CTRL_STD_SEQ_EN(x)		((x & 0x01) << 7)
#define AD469x_SEQ_CTRL_NUM_SLOTS_AS_MASK	(0x7f << 0)
#define AD469x_SEQ_CTRL_NUM_SLOTS_AS(x)		((x & 0x7f) << 0)

/* AD469x_REG_TEMP_CTRL */
#define AD469x_REG_TEMP_CTRL_TEMP_EN_MASK	(0x01 << 0)
#define AD469x_REG_TEMP_CTRL_TEMP_EN(x)		((x & 0x01) << 0)

/* AD469x_REG_AS_SLOT */
#define AD469x_REG_AS_SLOT_INX(x)		((x & 0x0f) << 0)

/* AD469x_REG_IF_CONFIG_C */
#define AD469x_REG_IF_CONFIG_C_MB_STRICT_MASK	(0x01 << 5)
#define AD469x_REG_IF_CONFIG_C_MB_STRICT(x)	((x & 0x01) << 5)

/* AD469x_REG_CONFIG_INn */
#define AD469x_REG_CONFIG_IN_OSR_MASK		(0x03 << 0)
#define AD469x_REG_CONFIG_IN_OSR(x)		((x & 0x03) << 0)
#define AD469x_REG_CONFIG_IN_HIZ_EN_MASK	(0x01 << 3)
#define AD469x_REG_CONFIG_IN_HIZ_EN(x)		((x & 0x01) << 3)
#define AD469x_REG_CONFIG_IN_PAIR_MASK		(0x03 << 4)
#define AD469x_REG_CONFIG_IN_PAIR(x)		((x & 0x03) << 4)
#define AD469x_REG_CONFIG_IN_MODE_MASK		(0x01 << 6)
#define AD469x_REG_CONFIG_IN_MODE(x)		((x & 0x01) << 6)
#define AD469x_REG_CONFIG_IN_TD_EN_MASK		(0x01 << 7)
#define AD469x_REG_CONFIG_IN_TD_EN(x)		((x & 0x01) << 7)

#define AD469x_CHANNEL(x)			(BIT(x) & 0xFFFF)
#define AD469x_CHANNEL_NO			16
#define AD469x_SLOTS_NO				0x80
#define AD469x_CHANNEL_TEMP			16

/* AXI_AD469x_IF_REG_CONFIG_INn */
#define	CNV_CONFIG_REG				(0X103)
#define	UP_DATA_SEQ				(0X101)
#define	CNV_RATE_CONFIG				(0X102)
#define	UP_DATA_COUNT				(0X106)
#define	CNV_PULSE_WIDTH				(0X107)
#define	TEST_REG_READ_ONLY			(0X105)
#define	UP_SCRATCH_REG  			(0X104)	

#define CNV_ENABLE(x)				(x & 0x01)	
#define UP_RESETN(x)				((x & 0x01) << 1)	

enum ad469x_ids {
	ID_AD4695,
	ID_AD4696,
};

static enum ad469x_reg_read {
	AD469x_STOP,
	AD469x_START
};

static enum ad469x_reg_read adc_mode;

struct ad469x_chip_info {
	struct iio_chan_spec	*channels;
	unsigned int 		num_channels;
	int 			freq_samp;
	u16			cnv_width;
	u16			cnv_rate;
};

#define AD469X_CHAN(index, bits)					\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.indexed = 1,						\
		.channel = index,					\
		.scan_index = index,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = bits,					\
			.storagebits =bits,				\
			.endianness = IIO_CPU,				\
		},							\
	}
	
#define DECLARE_AD469X_CHANNELS(name, bits,bits_set) 	\
static struct iio_chan_spec name[] = { 		\
		AD469X_CHAN(0, bits), 		\
		AD469X_CHAN(1, bits), 		\
}


static const struct iio_chan_spec ad469x_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(16),
	AD469X_CHAN(0, 16),
	AD469X_CHAN(1, 16),
	AD469X_CHAN(2, 16),
	AD469X_CHAN(3, 16),
	AD469X_CHAN(4, 16),
	AD469X_CHAN(5, 16),
	AD469X_CHAN(6, 16),
	AD469X_CHAN(7, 16),
	AD469X_CHAN(8, 16),
	AD469X_CHAN(9, 16),
	AD469X_CHAN(10, 16),
	AD469X_CHAN(11, 16),
	AD469X_CHAN(12, 16),
	AD469X_CHAN(13, 16),
	AD469X_CHAN(14, 16),
	AD469X_CHAN(15, 16),
};

DECLARE_AD469X_CHANNELS(ad4696_channels, 16,16);
DECLARE_AD469X_CHANNELS(ad4695_channels, 16,16);



static struct ad469x_chip_info ad469x_chip_info_tbl[] = {
	[ID_AD4695] = {
		.channels = ad469x_channels,
		.num_channels = ARRAY_SIZE(ad4695_channels),
		.freq_samp = 500000,
		.cnv_width = 0x30,
		.cnv_rate =  0x140,
	},
	[ID_AD4696] = {
		.channels = ad469x_channels,
		.num_channels = ARRAY_SIZE(ad4696_channels),
		.freq_samp = 1000000,
		.cnv_width = 0x30,
		.cnv_rate =  0xA0,
	},

};

static const struct axiadc_chip_info conv_chip_info = {
	.name = "ad469x_axi_adc",
	.max_rate = 1000000UL,
	.num_channels = 16,
	.channel[0] = AD469X_CHAN(0, 16),
	.channel[1] = AD469X_CHAN(1, 16),
	.channel[2] = AD469X_CHAN(2, 16),
	.channel[3] = AD469X_CHAN(3, 16),
	.channel[4] = AD469X_CHAN(4, 16),
	.channel[5] = AD469X_CHAN(5, 16),
	.channel[6] = AD469X_CHAN(6, 16),
	.channel[7] = AD469X_CHAN(7, 16),
	.channel[8] = AD469X_CHAN(8, 16),
	.channel[9] = AD469X_CHAN(9, 16),
	.channel[10] = AD469X_CHAN(10, 16),
	.channel[11] = AD469X_CHAN(11, 16),
	.channel[12] = AD469X_CHAN(12, 16),
	.channel[13] = AD469X_CHAN(13, 16),
	.channel[14] = AD469X_CHAN(14, 16),
	.channel[15] = AD469X_CHAN(15, 16),

};



struct ad469x_state {
	struct device   *dev;
	struct spi_message spi_msg;
	struct spi_transfer spi_transfer;
	struct spi_device		*spi;
	struct spi_transfer		t[8];
	struct regulator		*vref;
	const struct ad469x_chip_info 	*chip_info;
	struct mutex lock;
	u8 data_mode;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 * Make the buffer large enough for one 64 bit sample and one 64 bit
	 * aligned 64 bit timestamp.
	 */

	u16 data[10] ____cacheline_aligned;
	unsigned int num_bits;
	bool bus_locked;
};

static bool ad469x_has_axi_adc(struct device *dev)
{
	return device_property_present(dev, "spibus-connected");
}

static struct ad469x_state *ad469x_get_data(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv;

	if (ad469x_has_axi_adc(&indio_dev->dev)) {
		/* AXI ADC*/
		conv = iio_device_get_drvdata(indio_dev);
		return conv->phy;
	} else {
		return iio_priv(indio_dev);
	}
}

static int ad469x_spi_reg_write(struct ad469x_state *st,
			     u16 reg_addr,
			     u16 reg_data)
{
	int ret;
	u8 buf[4];

	buf[0] = ((reg_addr >> 8) & 0x7F);
	buf[1] = 0xFF & reg_addr;

	if (reg_addr == 0x25 | ( reg_addr >= 0x40 & reg_addr < 0x100))
	{
		buf[3] = reg_data ;
		buf[2] = reg_data >> 8;
	}
	else 
	{
		buf[2] = reg_data ;
		buf[3] = reg_data >> 8;
	}

	struct spi_transfer t [] = {
		{
		.tx_buf = buf,
		.len = 4,
		.bits_per_word = 32,
		}
	};

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	return ret;
}

static int  ad469x_spi_reg_read(struct ad469x_state *st,
			    u16 reg_addr,
			    u8 *reg_data)
{
	int ret;
	u8 buf[4];
	u8 rx_buf[4];

	buf[0] = (1 << 7) | ((reg_addr >> 8) & 0x7F);
	buf[1] = 0xFF & reg_addr;
	buf[2] = 0xFF;
	buf[3] = 0xFF;

	struct spi_transfer xfer [] = {
	{
		.tx_buf = buf,
		.rx_buf = rx_buf,
		.len = 4,
		.bits_per_word = 32,
	}
	};

	ret =spi_sync_transfer(st->spi, xfer, ARRAY_SIZE(xfer));
	if (reg_addr == 0x25 | ( reg_addr >= 0x40 & reg_addr < 0x100))
	{
		reg_data[0] = rx_buf[1];
		reg_data[1] = rx_buf[0];
	}
	else 
	{
		reg_data[0] = rx_buf[0]; 
		reg_data[1] = rx_buf[1];
	}

	return ret;
}

static int ad469x_spi_write_mask(struct iio_dev *indio_dev,
			         u16 reg_addr,
			         u16 mask,
			         u16 data)
{
	struct ad469x_state *st = ad469x_get_data(indio_dev);
	u8 reg_data[2];
	u16 read_val;
	int ret;
	ad469x_spi_reg_read(st, reg_addr, reg_data);
	read_val = (reg_data[0] << 8) | reg_data[1];
	read_val &= ~mask;
	read_val |= data;
	ret |= ad469x_spi_reg_write(st, reg_addr, read_val);
	return ret;
}

static u8 no_of_ones(u16 data){
	int i=0;
	u8 count,temp;
	for (i=0  ; i<16 ; i++){
		temp=(data >> i) & 0x01;
		if(temp==0x01){
		count++;}
	}
	return count;
}

static int ad469x_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad469x_state *st = ad469x_get_data(indio_dev);
	u8 rx_buf[3];       

	u8 read_val[2];
	u16 data;
	int ret;
	struct spi_message msg;

	if (st->data_mode)
	{
		ad469x_spi_write_mask(indio_dev,AD469x_REG_SEQ_CTRL, 
				AD469x_SEQ_CTRL_STD_SEQ_EN_MASK, AD469x_SEQ_CTRL_STD_SEQ_EN(1));
		ad469x_spi_reg_read(st, AD469x_REG_STD_SEQ_CONFIG+1, read_val);
		data = (read_val[0] << 8) | read_val[1];
		spi_engine_write_reg(st->spi,UP_DATA_SEQ,data);
		spi_engine_write_reg(st->spi,UP_DATA_COUNT,no_of_ones(data));
	}
			
	ad469x_spi_write_mask(indio_dev,AD469x_REG_SETUP, 
				AD469x_SETUP_IF_MODE_MASK, AD469x_SETUP_IF_MODE_CONV);

	struct spi_transfer xfer [] = { 
	{
		.rx_buf = rx_buf,
		.len = 4,
		.bits_per_word = 24,
	},
	};
	spi_bus_lock(st->spi->master);
	spi_message_init_with_transfers(&msg, xfer, ARRAY_SIZE(xfer));
	ret = spi_engine_offload_load_msg(st->spi, &msg);
	if (ret < 0)
		return ret;
	spi_engine_offload_enable(st->spi, true);  
	spi_engine_write_reg(st->spi,CNV_CONFIG_REG,CNV_ENABLE(1) |  UP_RESETN(1) | (st->data_mode <<2)); 
	return 0;

}

static int ad469x_buffer_predisable(struct iio_dev *indio_dev)
{	
	int ret,i;
	struct ad469x_state *st = ad469x_get_data(indio_dev);
	spi_engine_offload_enable(st->spi, false);
	adc_mode=AD469x_STOP;
	st->bus_locked = false;
	
	spi_bus_unlock(st->spi->master);

	for(i=0 ; i<=4 ; i++){
	ret = ad469x_spi_reg_write(st, 0x5000, 0x00); // to exit into conversion mode, must be before configuration
	}
	return 0;
}

static int ad469x_reg_access(struct iio_dev *indio_dev,
			     u32 reg, u32 writeval,
			     u32 *readval)
{	
	struct ad469x_state *st = ad469x_get_data(indio_dev);
	u8 reg_data[2];
	int ret;

	if(adc_mode=AD469x_START)
	{	
		ad469x_buffer_predisable(indio_dev);
	}
	mutex_lock(&st->lock);

	if (reg == 0x0400 ){
		if (readval == NULL)
	 	{
			st->data_mode = (0x01 & writeval);
		} 
		else 
		{	
			*readval = st->data_mode;
		}
	}
	else
	{
		if (readval == NULL)
	 	{
			ret = ad469x_spi_reg_write(st, reg, writeval);
		} 
		else 
		{
			ad469x_spi_reg_read(st, reg, reg_data);
			*readval = (reg_data[0] << 8) | reg_data[1];
			ret = 0;
		} 
	}
	mutex_unlock(&st->lock);
	return ret;
}

static int ad469x_read_samples(struct ad469x_state *st, unsigned int ch)
{
	int ret=0;
	u8 buf[3];
	u8 rx_buf[3];

	buf[0] = (1 << 7) | (ch << 4);
	buf[1] = 0;
	buf[2] = 0;
	struct spi_transfer xfer [] = {
	{
		.tx_buf = buf,
		.rx_buf = rx_buf,
		.len = 4,
		.bits_per_word = 24,
	}
	};
	
	struct spi_transfer xfer1 [] = {
	{
		.tx_buf = buf,
		.rx_buf = rx_buf,
		.len = 4,
		.bits_per_word = 24,
	}
	};
	
	ret =spi_sync_transfer(st->spi, xfer, ARRAY_SIZE(xfer));
	ret =spi_sync_transfer(st->spi, xfer1, ARRAY_SIZE(xfer1));
	ret = (rx_buf[1] << 8) | rx_buf[2];
	return ret;
}

static int ad469x_scan_direct(struct iio_dev *indio_dev,unsigned int ch)
{	
	struct ad469x_state *st =ad469x_get_data(indio_dev); 
	int ret=0;
	int ret1=0;
	int i;

	ret = ad469x_spi_reg_write(st, AD469x_REG_SEQ_CTRL, 0x00);
	ad469x_spi_write_mask(indio_dev,AD469x_REG_SETUP, 
				AD469x_SETUP_IF_MODE_MASK, AD469x_SETUP_IF_MODE_CONV);
	ret = ad469x_read_samples(st,ch);
	
	for(i=0 ; i<=4 ; i++){
	ret1 = ad469x_spi_reg_write(st, 0x5000, 0x00); // to exit into conversion mode, must be 
	}
	ret1 = ad469x_spi_reg_write(st, AD469x_REG_SEQ_CTRL, 0x80);

	return ret;
}

static int ad469x_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long info)
{
	struct ad469x_state *st =ad469x_get_data(indio_dev); 
	u8 buf[8];
	int scale_uv;
	int i;
	int ret;
	int *data;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = 	ad469x_scan_direct(indio_dev, chan->scan_index);
		iio_device_release_direct_mode(indio_dev);

		if (ret < 0)
			return ret;
		*val = (short)ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		scale_uv = regulator_get_voltage(st->vref);
		if (scale_uv < 0)
			return scale_uv;

		*val = scale_uv * 2 / 1000;
		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->chip_info->freq_samp;
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int hw_submit_block(struct iio_dma_buffer_queue *queue,
			   struct iio_dma_buffer_block *block)
{	
	block->block.bytes_used = block->block.size;

	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static const struct iio_dma_buffer_ops dma_buffer_ops = {
	.submit = hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static void ad469x_reg_disable(void *data)
{
	struct regulator *reg = data;

	regulator_disable(reg);
}

static const struct iio_buffer_setup_ops ad469x_buffer_ops = {
	.postenable = &ad469x_buffer_postenable,
	.predisable = &ad469x_buffer_predisable,
};

static const struct iio_info ad469x_info = {
	.read_raw = &ad469x_read_raw,
	.debugfs_reg_access = &ad469x_reg_access,AD469x_REG_SETUP
};


static int ad469x_post_setup(struct iio_dev *indio_dev){
	indio_dev->setup_ops = &ad469x_buffer_ops;
	return 0;
}

static int ad469x_Devic_Setup(struct iio_dev *indio_dev)
{
	int ret;
	struct ad469x_state *st = ad469x_get_data(indio_dev);

	ret = ad469x_spi_reg_write(st, AD469x_REG_IF_CONFIG_A, 0x10);
	ret = ad469x_spi_reg_write(st, AD469x_REG_IF_CONFIG_B, 0x00);
	ret = ad469x_spi_reg_write(st, AD469x_REG_IF_CONFIG_C, 0x23); 
	
	ret = ad469x_spi_reg_write(st, AD469x_REG_SEQ_CTRL, 0x80);
	ret = ad469x_spi_reg_write(st, AD469x_REG_STD_SEQ_CONFIG+1, 0x03);
	ret = ad469x_spi_reg_write(st, AD469x_REG_GPIO_CTRL, 0x00);
	ret = ad469x_spi_reg_write(st, AD469x_REG_GP_MODE, 0x8E);

	spi_engine_write_reg(st->spi,CNV_CONFIG_REG,0x00);// to disable up_resetn and up_cnv pulse

	spi_engine_write_reg(st->spi,CNV_RATE_CONFIG,st->chip_info->cnv_rate);// value to calculate as 1000ns*spi_clk_frequency **
	spi_engine_write_reg(st->spi,CNV_PULSE_WIDTH,st->chip_info->cnv_width);// value to calculate as 250ns*spi_clk_frequency  **
	ret = ad469x_spi_reg_write(st, AD469x_REG_SETUP, 0x30);
	return ret;
}


static int ad469x_register_axi_adc(struct ad469x_state *st)
{
	struct axiadc_converter	*conv;
	struct spi_device *spi = to_spi_device(st->dev);

	conv = devm_kzalloc(&st->spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->spi = spi;
	conv->clk = NULL;
	conv->chip_info = &conv_chip_info;
	conv->adc_output_mode = AD469x_OUTPUT_MODE_TWOS_COMPLEMENT;
	conv->reg_access = &ad469x_reg_access;
	conv->read_raw = &ad469x_read_raw;	
	conv->post_setup = &ad469x_post_setup;
	conv->phy = st;
	/* Without this, the axi_adc won't find the converter data */
	spi_set_drvdata(st->spi, conv);

	return 0;
}

static int ad469x_register(struct ad469x_state *st, struct iio_dev *indio_dev)
{	
	struct iio_buffer *buffer;
	indio_dev->dev.parent = &st->spi->dev;
	indio_dev->name = "ad4696";
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->channels = ad4696_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad4696_channels);
	indio_dev->info = &ad469x_info;
	indio_dev->setup_ops = &ad469x_buffer_ops;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "rx",
						&dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	return devm_iio_device_register(&st->spi->dev, indio_dev);
}


static int ad469x_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	struct ad469x_state *st;
	int ret ,dev_id;
	
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	dev_id = spi_get_device_id(spi)->driver_data;
	st = iio_priv(indio_dev);

	st->spi = spi;
	st->chip_info =&ad469x_chip_info_tbl[spi_get_device_id(spi)->driver_data];
	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);
	ret = regulator_enable(st->vref);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&spi->dev, ad469x_reg_disable, st->vref);
	if (ret)
		return ret;
	spi_set_drvdata(spi, indio_dev);
	ad469x_Devic_Setup(indio_dev);

	if (device_property_present(&spi->dev, "dmas"))
		ret = ad469x_register(st, indio_dev);
	else
		ret = ad469x_register_axi_adc(st);

	return ret;
error_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);
error_disable_vref:
	regulator_disable(st->vref);
error:
	iio_dmaengine_buffer_free(indio_dev->buffer);
	regulator_disable(st->vref);

	return ret;
}

static int ad469x_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad469x_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	regulator_disable(st->vref);

	return 0;
}

static const struct spi_device_id ad469x_id[] = {
	{ "ad4695", ID_AD4695 },
	{ "ad4696", ID_AD4696 },
	{},
};

static const struct of_device_id ad469x_of_match[] = {
        { .compatible = "adi,ad4695" },
        { .compatible = "adi,ad4696" },
        {},
};

MODULE_DEVICE_TABLE(of, ad469x_id);

static struct spi_driver ad469x_driver = {
	.driver = {
		.name = "ad4696", 
		.of_match_table = ad469x_of_match,
	},
	.probe = ad469x_probe,
	.remove = ad469x_remove,
	.id_table = ad469x_id,
};


module_spi_driver(ad469x_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD469x ADC driver");
MODULE_LICENSE("GPL v2");

