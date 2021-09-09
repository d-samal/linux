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
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

/*
 * AD738X registers definition
 */
#define AD738X_REG_NOP			0x00
#define AD738X_REG_CONFIG1		0x01
#define AD738X_REG_CONFIG2		0x02
#define AD738X_REG_ALERT		0x03
#define AD738X_REG_ALERT_LOW_TH		0x04
#define AD738X_REG_ALERT_HIGH_TH	0x05

/*
 * AD738X_REG_CONFIG1
 */
#define AD738X_CONFIG1_OS_MODE_MSK	BIT(9)
#define AD738X_CONFIG1_OS_MODE(x)	(((x) & 0x1) << 9)
#define AD738X_CONFIG1_OSR_MSK		GENMASK(8, 6)
#define AD738X_CONFIG1_OSR(x)		(((x) & 0x7) << 6)
#define AD738X_CONFIG1_CRC_W_MSK	BIT(5)
#define AD738X_CONFIG1_CRC_W(x)		(((x) & 0x1) << 5)
#define AD738X_CONFIG1_CRC_R_MSK	BIT(4)
#define AD738X_CONFIG1_CRC_R(x)		(((x) & 0x1) << 4)
#define AD738X_CONFIG1_ALERTEN_MSK	BIT(3)
#define AD738X_CONFIG1_ALERTEN(x)	(((x) & 0x1) << 3)
#define AD738X_CONFIG1_RES_MSK		BIT(2)
#define AD738X_CONFIG1_RES(x)		(((x) & 0x1) << 2)
#define AD738X_CONFIG1_REFSEL_MSK	BIT(1)
#define AD738X_CONFIG1_REFSEL(x)	(((x) & 0x1) << 1)
#define AD738X_CONFIG1_PMODE_MSK	BIT(0)
#define AD738X_CONFIG1_PMODE(x)		(((x) & 0x1) << 0)
#define AD738X_CONFIG1_RESET_MSK	GENMASK(15, 0)
#define AD738X_CONFIG1_RESET(x)		(((x) & 0xFFFF) << 0)

/*
 * AD738X_REG_CONFIG2
 */
#define AD738X_CONFIG2_SDO2_MSK		BIT(8)
#define AD738X_CONFIG2_SDO2(x)		(((x) & 0x1) << 8)
#define AD738X_CONFIG2_SDO4_MSK		GENMASK(9, 8)
#define AD738X_CONFIG2_SDO4(x)		(((x) & 0x3) << 8)
#define AD738X_CONFIG2_RESET_MSK	GENMASK(7, 0)
#define AD738X_CONFIG2_RESET(x)      	(((x) & 0xFF) << 0)

/*
 * AD738X_REG_ALERT_LOW_TH
 */
#define AD738X_ALERT_LOW_MSK		GENMASK(11, 0)
#define AD738X_ALERT_LOW(x) 		(((x) & 0xFFF) << 0)

/*
 * AD738X_REG_ALERT_HIGH_TH
 */
#define AD738X_ALERT_HIGH_MSK		GENMASK(11, 0)
#define AD738X_ALERT_HIGH(x) 		(((x) & 0xFFF) << 0)

/* Write to register x */
#define AD738X_REG_WRITE(x)		((1 << 7) | ((x & 0x07) << 4))

enum ad738x_ids {
	ID_7380,
	ID_7381,
	ID_7382,
	ID_7383,
	ID_7384,
	ID_7385,
	ID_7386,
	ID_7387,
	ID_7388,
	ID_7380_4,
	ID_7381_4,
	ID_7382_4,
	ID_7383_4,
	ID_7384_4,
	ID_7385_4,
	ID_7386_4,
	ID_7387_4,
	ID_7388_4
};

enum ad738x_reg_read {
	AD738x_STOP,
	AD738x_START
};

enum ad738x_reg_read adc_mode;

static int ad738x_buffer_predisable(struct iio_dev *indio_dev);



struct ad738x_chip_info {
	struct iio_chan_spec	*channels;
	unsigned int 		num_channels;
};


#define AD738X_CHAN(index, bits,bits_set)					\
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
			.storagebits =bits_set,				\
			.endianness = IIO_CPU,				\
			.shift = (bits_set/2) * index,				\
		},							\
	}

	
#define DECLARE_AD738X_CHANNELS(name, bits,bits_set) 	\
static struct iio_chan_spec name[] = { 		\
		AD738X_CHAN(0, bits,bits_set), 		\
		AD738X_CHAN(1, bits,bits_set), 		\
}

#define DECLARE_AD738X_4_CHANNELS(name, bits,bits_set) \
static struct iio_chan_spec name[] = { \
		AD738X_CHAN(0, bits,bits_set), \
		AD738X_CHAN(1, bits,bits_set), \
		AD738X_CHAN(2, bits,bits_set), \
		AD738X_CHAN(3, bits,bits_set), \
}

DECLARE_AD738X_CHANNELS(ad7380_channels, 18,64);
DECLARE_AD738X_CHANNELS(ad7381_channels, 16 ,32);
DECLARE_AD738X_CHANNELS(ad7382_channels, 14 ,32);
DECLARE_AD738X_CHANNELS(ad7383_channels, 18 ,64);
DECLARE_AD738X_CHANNELS(ad7384_channels, 16 ,32);
DECLARE_AD738X_CHANNELS(ad7385_channels, 14 ,32);
DECLARE_AD738X_CHANNELS(ad7386_channels, 18 ,64);
DECLARE_AD738X_CHANNELS(ad7387_channels, 16 ,32);
DECLARE_AD738X_CHANNELS(ad7388_channels, 14 ,32);
DECLARE_AD738X_4_CHANNELS(ad7380_4_channels, 16,64);
DECLARE_AD738X_4_CHANNELS(ad7381_4_channels, 14,32);
DECLARE_AD738X_4_CHANNELS(ad7382_4_channels, 12,32);
DECLARE_AD738X_4_CHANNELS(ad7383_4_channels, 16,64);
DECLARE_AD738X_4_CHANNELS(ad7384_4_channels, 14,32);
DECLARE_AD738X_4_CHANNELS(ad7385_4_channels, 12,32);
DECLARE_AD738X_4_CHANNELS(ad7386_4_channels, 16,64);
DECLARE_AD738X_4_CHANNELS(ad7387_4_channels, 14,32);
DECLARE_AD738X_4_CHANNELS(ad7388_4_channels, 12,32);

static struct ad738x_chip_info ad738x_chip_info_tbl[] = {
	[ID_7380] = {
		.channels = ad7380_channels,
		.num_channels = ARRAY_SIZE(ad7380_channels),
	},
	[ID_7381] = {
		.channels = ad7381_channels,
		.num_channels = ARRAY_SIZE(ad7381_channels),
	},
	[ID_7382] = {
		.channels = ad7382_channels,
		.num_channels = ARRAY_SIZE(ad7382_channels),
	},
	[ID_7383] = {
		.channels = ad7383_channels,
		.num_channels = ARRAY_SIZE(ad7383_channels),
	},
	[ID_7384] = {
		.channels = ad7384_channels,
		.num_channels = ARRAY_SIZE(ad7384_channels),
	},
	[ID_7385] = {
		.channels = ad7385_channels,
		.num_channels = ARRAY_SIZE(ad7385_channels),
	},
	[ID_7386] = {
		.channels = ad7386_channels,
		.num_channels = ARRAY_SIZE(ad7386_channels),
	},
	[ID_7387] = {
		.channels = ad7387_channels,
		.num_channels = ARRAY_SIZE(ad7387_channels),
	},
	[ID_7388] = {
		.channels = ad7388_channels,
		.num_channels = ARRAY_SIZE(ad7388_channels),
	},
	[ID_7380_4] = {
		.channels = ad7380_4_channels,
		.num_channels = ARRAY_SIZE(ad7380_4_channels),
	},
	[ID_7381_4] = {
		.channels = ad7381_4_channels,
		.num_channels = ARRAY_SIZE(ad7381_4_channels),
	},
	[ID_7382_4] = {
		.channels = ad7382_4_channels,
		.num_channels = ARRAY_SIZE(ad7382_4_channels),
	},
	[ID_7383_4] = {
		.channels = ad7383_4_channels,
		.num_channels = ARRAY_SIZE(ad7383_4_channels),
	},
	[ID_7384_4] = {
		.channels = ad7384_4_channels,
		.num_channels = ARRAY_SIZE(ad7384_4_channels),
	},
	[ID_7385_4] = {
		.channels = ad7385_4_channels,
		.num_channels = ARRAY_SIZE(ad7385_4_channels),
	},
	[ID_7386_4] = {
		.channels = ad7386_4_channels,
		.num_channels = ARRAY_SIZE(ad7386_4_channels),
	},
	[ID_7387_4] = {
		.channels = ad7387_4_channels,
		.num_channels = ARRAY_SIZE(ad7387_4_channels),
	},
	[ID_7388_4] = {
		.channels = ad7388_4_channels,
		.num_channels = ARRAY_SIZE(ad7388_4_channels),
	},
};

struct ad738x_state {
	struct spi_message spi_msg;
	struct spi_transfer spi_transfer;
	struct spi_device		*spi;
	struct spi_transfer		t[8];
	struct regulator		*vref;
	const struct ad738x_chip_info 	*chip_info;
	struct gpio_desc		*gpio_shift;
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


static int ad738x_spi_reg_write(struct ad738x_state *st,
			        u8 reg_addr,
			        u16 reg_data)
{
	struct spi_transfer t[] = {
		{
		.tx_buf = &st->data[0],
		.len = 2,
		.bits_per_word = 16,
		}
	};
	int ret;
	st->data[0] = ((reg_addr & 0x07) << 4) | 0x80 ;
	st->data[0] = st->data[0] | be16_to_cpu(reg_data & 0x0FFF);
	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;
	return ret;
}

static int ad738x_spi_reg_read(struct ad738x_state *st, u8 reg_addr,
			       u8 *data)
{
	u8 tx_buf[2];
	u8 rx_buf[4];
	u8 n_tx = 4;
	u8 n_rx = 2;
	int ret;

	tx_buf[0] = (reg_addr & 0x07) << 4;
	tx_buf[1] = 0x00;

	struct spi_transfer xfer [] = {
	{
		.tx_buf = tx_buf,
		.len = 2,
		.cs_change=1,
		.bits_per_word = 16,
	},
	{
		.rx_buf = rx_buf,
		.len = 2,
		.bits_per_word = 16,
	},
	};
	ret =spi_sync_transfer(st->spi, xfer, ARRAY_SIZE(xfer));

	if (ret < 0)
		return ret;

	data[0] = rx_buf[1];
	data[1] = rx_buf[0];
	return ret;
}

static int ad738x_spi_write_mask(struct ad738x_state *st,
			         u8 reg_addr,
			         u16 mask,
			         u16 data)
{
	u8 spi_buf[2];
	u16 reg_data;
	int ret;
	reg_data = (spi_buf[0] << 8) | spi_buf[1];
	reg_data &= ~mask;
	reg_data |= data;
	ret |= ad738x_spi_reg_write(st, reg_addr, reg_data);

	return ret;
}

static int ad738x_reg_access(struct iio_dev *indio_dev,
			     u32 reg, u32 writeval,
			     u32 *readval)
{	
	struct ad738x_state *st = iio_priv(indio_dev);
	u8 reg_data[2];
	int ret;

	if(adc_mode=AD738x_START)
	{	
		ad738x_buffer_predisable(indio_dev);
	}


	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = ad738x_spi_reg_write(st, reg, writeval);
	} else {
		ad738x_spi_reg_read(st, reg, reg_data);
		*readval = (reg_data[0] << 8) | reg_data[1];
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static int ad738x_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *active_scan_mask)
{
	struct ad738x_state *st = iio_priv(indio_dev);
	u8 num_bytes = st->chip_info->num_channels * 2;
	int i;
	int j = 0;

	st->t[0].tx_buf = &st->data[0];
	st->t[0].len = num_bytes;
	st->t[0].cs_change = 1;

	/* Find which channels are active and overwrite the data
	   for the disabled channels*/
	for (i = 0; i < st->chip_info->num_channels; i++) {
		st->t[i+1].rx_buf = &st->data[j];
		st->t[i+1].len = 2;
		if(test_bit(i, active_scan_mask))
			j+=2;
	}

	return 0;
}

static irqreturn_t ad738x_trigger_handler(int irq, void  *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad738x_state *st = iio_priv(indio_dev);
	u8 num_bytes = st->chip_info->num_channels * 2;
	int i;
	int ret;

	/* Send NOP */
	for (i = 0; i < num_bytes; i++)
		st->data[i] = cpu_to_be16(AD738X_REG_NOP);

	ret = spi_sync_transfer(st->spi, st->t, ARRAY_SIZE(st->t));

	if (ret == 0) {
		iio_push_to_buffers_with_timestamp(indio_dev, st->data,
						   iio_get_time_ns(indio_dev));
	}
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ad738x_scan_direct(struct ad738x_state *st ,int *data)
{
	u8 num_bytes = 2;
	int ret=0;
	u8 reg_data[2];
	adc_mode=AD738x_START;
	
 	ad738x_spi_reg_write(st, AD738X_REG_CONFIG2, AD738X_CONFIG2_SDO2(1));
	struct spi_transfer t[] = {
		{
			.rx_buf = &st->data[0],
			.len = 2,
			.cs_change = 0,
			.bits_per_word=st->num_bits,
		},
		{
			.rx_buf = &st->data[1],
			.len = 2,
			.bits_per_word=st->num_bits,
		},
	};
	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));

	if (ret < 0)
		return ret;
	return ret;
}

static int ad738x_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long info)
{
	struct ad738x_state *st = iio_priv(indio_dev);
	u8 index = 1 * chan->scan_index;
	u8 buf[8];
	int scale_uv;
	int i;
	int ret;
	int *data;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		if (iio_buffer_enabled(indio_dev))
			ret = -EBUSY;
		else
			ret = ad738x_scan_direct(st,data);
		mutex_unlock(&indio_dev->mlock);
		if (ret < 0)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		scale_uv = regulator_get_voltage(st->vref);
		if (scale_uv < 0)
			return scale_uv;

		*val = scale_uv * 2 / 1000;
		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = 4000000;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ad738x_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad738x_state *st = iio_priv(indio_dev);
	int ret;
	u8 reg_data[2];
	adc_mode=AD738x_START;
	u8 tmp=0;	
	
	ad738x_spi_reg_write(st, AD738X_REG_CONFIG2, AD738X_CONFIG2_SDO2(0));
	ad738x_spi_reg_read(st,AD738X_REG_CONFIG1, reg_data);
	
	tmp=((reg_data[1] & 0x0004) >>2);

	if(tmp==1)
	{
		gpiod_set_value(st->gpio_shift, 1);
		
		struct spi_transfer xfer [] = {
		{
		.rx_buf = (void *)-1,
		.len = 2,
		.cs_change=1,
		.bits_per_word = st->num_bits ,
		},{
		.rx_buf = (void *)-1,
		.len = 2,
		.cs_change=1,
		.bits_per_word = st->num_bits,
		},{
		.rx_buf = (void *)-1,
		.len = 2,
		.cs_change=0,
		.bits_per_word = st->num_bits,
		}	
		};

		spi_message_init_with_transfers(&st->spi_msg, xfer,ARRAY_SIZE(xfer));
		spi_bus_lock(st->spi->master);
		st->bus_locked = true;
		ret = spi_engine_offload_load_msg(st->spi, &st->spi_msg);
		if (ret < 0)
			return ret;
		spi_engine_offload_enable(st->spi, true);
	}
	else
	{		
		gpiod_set_value(st->gpio_shift, 0);

		struct spi_transfer xfer [] = {
		{
		.rx_buf = (void *)-1,
		.len = 1,
		.cs_change=1,
		.bits_per_word = st->num_bits-2 ,
		},{
		.rx_buf = (void *)-1,
		.len = 1,
		.cs_change=1,
		.bits_per_word = st->num_bits-2,
		},{
		.rx_buf = (void *)-1,
		.len = 1,
		.cs_change=1,
		.bits_per_word = st->num_bits-2,
		},{
		.rx_buf = (void *)-1,
		.len = 1,
		.cs_change=0,
		.bits_per_word = st->num_bits-2,
		}	
		};

		spi_message_init_with_transfers(&st->spi_msg, xfer,ARRAY_SIZE(xfer));

		spi_bus_lock(st->spi->master);
		st->bus_locked = true;

		ret = spi_engine_offload_load_msg(st->spi, &st->spi_msg);
		if (ret < 0)
			return ret;

		spi_engine_offload_enable(st->spi, true);
	}
	return 0;
}
static int ad738x_buffer_predisable(struct iio_dev *indio_dev)
{	
	struct ad738x_state *st = iio_priv(indio_dev);

	spi_engine_offload_enable(st->spi, false);
	adc_mode=AD738x_STOP;

	st->bus_locked = false;
	return spi_bus_unlock(st->spi->master);
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

static const struct iio_buffer_setup_ops ad738x_buffer_setup_ops = {
	.postenable = &ad738x_buffer_postenable,
	.predisable = &ad738x_buffer_predisable,
};

static const struct iio_info ad738x_info = {
	.read_raw = &ad738x_read_raw,
	.debugfs_reg_access = &ad738x_reg_access,
	.update_scan_mode = ad738x_update_scan_mode,
};

static int ad738x_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	struct ad738x_state *st;
	int ret ,dev_id;
	
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	dev_id = spi_get_device_id(spi)->driver_data;
	st = iio_priv(indio_dev);
	st->chip_info =
		&ad738x_chip_info_tbl[spi_get_device_id(spi)->driver_data];
	
	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);
	ret = regulator_enable(st->vref);
	if (ret)
		return ret;
	
	spi_set_drvdata(spi, indio_dev);
	struct spi_controller *ctlr = spi->controller;

	ctlr->max_speed_hz=80000000;
	spi->controller = ctlr;
	st->spi=spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE |INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad738x_buffer_setup_ops;
	indio_dev->info = &ad738x_info;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	st->num_bits = indio_dev->channels->scan_type.realbits;

	st->gpio_shift = devm_gpiod_get(&spi->dev, "shift",GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_shift))
		return PTR_ERR(st->gpio_shift);
	
	/*2-wire mode */
	ret = ad738x_spi_write_mask(st,
				    AD738X_REG_CONFIG2,
				    AD738X_CONFIG2_SDO2_MSK,
				    AD738X_CONFIG2_SDO2(0));
	
	if (ret < 0)
		goto error_buffer_cleanup;

	buffer = iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "rx",
					    &dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret < 0)
		goto error;

	return 0;

error_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);
error_disable_vref:
	regulator_disable(st->vref);
error:
	iio_dmaengine_buffer_free(indio_dev->buffer);
	regulator_disable(st->vref);

	return ret;
}

static int ad738x_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad738x_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	regulator_disable(st->vref);

	return 0;
}

static const struct spi_device_id ad738x_id[] = {
	{ "ad7380", ID_7380 },
	{ "ad7381", ID_7381 },
	{ "ad7382", ID_7382 },
	{ "ad7383", ID_7383 },
	{ "ad7384", ID_7384 },
	{ "ad7385", ID_7385 },
	{ "ad7386", ID_7386 },
	{ "ad7387", ID_7387 },
	{ "ad7388", ID_7388 },
	{ "ad7380-4", ID_7380_4 },
	{ "ad7381-4", ID_7381_4 },
	{ "ad7382-4", ID_7382_4 },
	{ "ad7383-4", ID_7383_4 },
	{ "ad7384-4", ID_7384_4 },
	{ "ad7385-4", ID_7385_4 },
	{ "ad7386-4", ID_7386_4 },
	{ "ad7387-4", ID_7387_4 },
	{ "ad7388-4", ID_7388_4 },
	{}
};

static const struct of_device_id ad738x_of_match[] = {
        { .compatible = "adi,ad7380" },
        { .compatible = "adi,ad7381" },
        { },
};

MODULE_DEVICE_TABLE(spi, ad738x_id);

static struct spi_driver ad738x_driver = {
	.driver = {
		.name = "ad738x",
	},
	.probe = ad738x_probe,
	.remove = ad738x_remove,
	.id_table = ad738x_id,
};
module_spi_driver(ad738x_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD738x ADC driver");
MODULE_LICENSE("GPL v2");
