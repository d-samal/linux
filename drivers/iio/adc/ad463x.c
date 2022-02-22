/*
 * Analog Devices AD463x Differential Input, Simultaneous Sampling,
 * 24-BIT, SAR ADC driver
 *
 * Copyright 2021 Analog Devices Inc.
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
#include <linux/pwm.h>

/* Register addresses */
#define AD463X_REG_INTERFACE_CONFIG_A	0x00
#define AD463X_REG_INTERFACE_CONFIG_B	0x01
#define AD463X_REG_DEVICE_CONFIG	0x02
#define AD463X_REG_CHIP_TYPE		0x03
#define AD463X_REG_PRODUCT_ID_L		0x04
#define AD463X_REG_PRODUCT_ID_H		0x05
#define AD463X_REG_CHIP_GRADE		0x06
#define AD463X_REG_SCRATCH_PAD		0x0A
#define AD463X_REG_SPI_REVISION		0x0B
#define AD463X_REG_VENDOR_L		0x0C
#define AD463X_REG_VENDOR_H		0x0D
#define AD463X_REG_STREAM_MODE		0x0E
#define AD463X_REG_EXIT_CFG_MODE	0x14
#define AD463X_REG_AVG			0x15
#define AD463X_REG_OFFSET_BASE		0x16
#define AD463X_REG_OFFSET_X0_0		0x16
#define AD463X_REG_OFFSET_X0_1		0x17
#define AD463X_REG_OFFSET_X0_2		0x18
#define AD463X_REG_OFFSET_X1_0		0x19
#define AD463X_REG_OFFSET_X1_1		0x1A
#define AD463X_REG_OFFSET_X1_2		0x1B
#define AD463X_REG_GAIN_BASE		0x1C
#define AD463X_REG_GAIN_X0_LSB		0x1C
#define AD463X_REG_GAIN_X0_MSB		0x1D
#define AD463X_REG_GAIN_X1_LSB		0x1E
#define AD463X_REG_GAIN_X1_MSB		0x1F
#define AD463X_REG_MODES		0x20
#define AD463X_REG_OSCILATOR		0x21
#define AD463X_REG_IO			0x22
#define AD463X_REG_PAT0			0x23
#define AD463X_REG_PAT1			0x24
#define AD463X_REG_PAT2			0x25
#define AD463X_REG_PAT3			0x26
#define AD463X_REG_DIG_DIAG		0x34
#define AD463X_REG_DIG_ERR		0x35
/* MODES */
#define AD463X_LANE_MODE_MSK		GENMASK(7, 6)
#define AD463X_CLK_MODE_MSK		GENMASK(5, 4)
#define AD463X_DATA_RATE_MODE_MSK	BIT(3)
#define AD463X_OUT_DATA_MODE_MSK	GENMASK(2, 0)
/* EXIT_CFG_MD */
#define AD463X_EXIT_CFG_MODE		BIT(0)
/* AVG */
#define AD463X_AVG_FILTER_RESET		BIT(7)
#define AD463X_AVG_LEN_DEFAULT		0x06

#define AD463X_REG_CHAN_OFFSET(ch, pos)	(AD463X_REG_OFFSET_BASE + (3*ch) + pos)
#define AD463X_REG_CHAN_GAIN(ch, pos)	(AD463X_REG_GAIN_BASE + (2 * ch) + pos)

#define AD463X_CONFIG_TIMING		0x2000
#define AD463X_REG_READ_DUMMY		0x00
#define AD463X_REG_WRITE_MASK(x)	(x & 0x7FFF)
#define AD463X_REG_READ_MASK(x)		(x | BIT(15))

#define AD463X_SPI_REG_ACCESS_SPEED	20000000UL
#define AD463X_SPI_SAMPLING_SPEED	80000000UL
#define AD463X_SPI_WIDTH(mode, width)	(width >> (mode >> 6))

#define AD463X_FREQ_TO_PERIOD(f)	DIV_ROUND_UP(USEC_PER_SEC, \
						     (f / NSEC_PER_USEC))
#define AD463X_TRIGGER_PULSE_WIDTH_NS	10

#define AD463X_T_CONV_HI		10
#define AD463X_T_CONV			300

#define	CNV_CONFIG_REG				(0X103)
#define	UP_CONFIG_REG				(0X105)
#define	CNV_RATE_CONFIG				(0X101)
#define	CNV_PULSE_WIDTH				(0X102)
#define	TEST_REG_READ_ONLY			(0X108)
#define	UP_SCRATCH_REG  			(0X104)
#define	VCOM0_REG  					(0X106)
#define	VCOM1_REG  					(0X107)

#define CNV_ENABLE(x)				(x & 0x01)
#define UP_RESETN(x)				((x & 0x01) << 1)

enum ad463x_id {
	ID_AD4630_24,
	ID_AD4630_20,
	ID_AD4630_16,
	ID_AD4631_24,
	ID_AD4631_20,
	ID_AD4631_16,
	ID_AD4632_24,
	ID_AD4632_20,
	ID_AD4632_16,
};

enum ad463x_reg_read {
	AD463x_DATA_MODE,
	AD463x_REG_MODE
};

enum ad463x_reg_read adc_mode;

static int ad463x_buffer_predisable(struct iio_dev *indio_dev);

struct ad463x_chip_info {
	struct iio_chan_spec	*channels;
	unsigned int 		num_channels;
	int 			freq_samp;
	u16			cnv_width;
	u16			cnv_rate;
	u16			cnv_rate_1_lane_sdr;
};

#define AD463x_CHAN(index, bits,bits_set)					\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.ext_info = &ad463x_ext_info,				\
		.indexed = 1,						\
		.channel = index,					\
		.scan_index = index,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = bits,					\
			.storagebits =bits_set,				\
			.endianness = IIO_CPU,				\
			.shift = (bits_set/2) * index + 8,				\
		},							\
	}

#define DECLARE_AD463x_CHANNELS(name, bits,bits_set) 	\
static struct iio_chan_spec name[] = { 		\
		AD463x_CHAN(0, bits,bits_set), 		\
		AD463x_CHAN(1, bits,bits_set), 		\
}

enum ad463x_lane_mode {
	AD463X_ONE_LANE_PER_CH	= 0x00,
	AD463X_TWO_LANES_PER_CH	= BIT(6),
	AD463X_FOUR_LANES_PER_CH = BIT(7),
	AD463X_SHARED_TWO_CH = (BIT(6) | BIT(7)),
};

enum ad463x_clock_mode {
	AD463X_SPI_COMPATIBLE_MODE = 0x00,
	AD463X_ECHO_CLOCK_MODE = BIT(4),
	AD463X_CLOCK_MASTER_MODE = BIT(5),
};

enum ad463x_data_rate_mode {
	AD463X_SINGLE_DATA_RATE = 0x00,
	AD463X_DUAL_DATA_RATE = BIT(3),
};

enum ad463x_out_data_mode {
	AD463X_24_DIFF = 0x00,
	AD463X_16_DIFF_8_COM = 0x01,
	AD463X_24_DIFF_8_COM = 0x02,
	AD463X_30_AVERAGED_DIFF = 0x03,
	AD463X_32_PATTERN = 0x04
};

enum ad463x_power_mode {
	AD463X_NORMAL_OPERATING_MODE = 0,
	AD463X_LOW_POWER_MODE = (BIT(0) | BIT(1)),
};

struct ad463x_state {
	struct spi_message spi_msg;
	struct spi_transfer spi_transfer;
	struct spi_device		*spi;
	struct spi_transfer		t[8];
	struct regulator		*vref;
	const struct ad463x_chip_info 	*chip_info;
	struct gpio_desc		*gpio_reset;
	unsigned int num_bits;
	bool bus_locked;
	bool spi_is_dma_mapped;
};

struct mode_info{
	u8 lane_mode;
	u8 clock_mode;
	u8 ddr_mode;
	u8 out_data_mode;
};

static int ad463x_spi_reg_write(struct ad463x_state *st,
			     u16 reg_addr,
			     u16 reg_data)
{
	int ret;
	u8 buf[3];

	buf[0] = ((reg_addr >> 8) & 0x7F);
	buf[1] = 0xFF & reg_addr;
	buf[2] = reg_data;
	struct spi_transfer xfer [] = {
	{
		.tx_buf = &buf[0],
		.len = 1,
		.cs_change=0,
		.bits_per_word = 8,
	},
	{
		.tx_buf = &buf[1],
		.len = 1,
		.cs_change=0,
		.bits_per_word = 8,
	},
	{
		.tx_buf = &buf[2],
		.len = 1,
		.bits_per_word = 8,
	},
	};

	ret = spi_sync_transfer(st->spi, xfer, ARRAY_SIZE(xfer));
	if (ret < 0)
		return ret;

	return ret;
}

static int  ad463x_spi_reg_read(struct ad463x_state *st,
			    u16 reg_addr,
			    u8 *reg_data)
{
	int ret;
	u8 buf[3];
	u8 rx_buf;

	buf[0] = (1 << 7) | ((reg_addr >> 8) & 0x7F);
	buf[1] = 0xFF & reg_addr;
	buf[2] = AD463X_REG_READ_DUMMY;

	struct spi_transfer xfer [] = {
	{
		.tx_buf = &buf[0],
		.len = 1,
		.bits_per_word = 8,
		.cs_change=0,
	},
	{
		.tx_buf = &buf[1],
		.len = 1,
		.cs_change=0,
		.bits_per_word = 8,
	},
	{
		.tx_buf = &buf[2],
		.rx_buf = &rx_buf,
		.len = 1,
		.bits_per_word = 8,
	},
	};

	ret =spi_sync_transfer(st->spi, xfer, ARRAY_SIZE(xfer));

	reg_data[0] = rx_buf;
	return ret;
}

static int get_mode_info(struct ad463x_state *st, struct mode_info *m_info)
{
	u8 modes;

	ad463x_spi_reg_read(st, AD463X_REG_MODES, &modes);

	/* Lane Mode */
	m_info->lane_mode = (modes & 0xC0);
	/* Clock Mode */
	m_info->clock_mode = (modes & 0x30);
	/* DDR Mode */
	m_info->ddr_mode = (modes & 0x08);
	/* Out Data Mode */
	m_info->out_data_mode = (modes & 0x07);
	return 0;
}

static int ad463x_set_reg_access(struct ad463x_state *st, bool state)
{
	u8 dummy;

	if (state)
		/* Send a sequence starting with "1 0 1" to the SPI bus*/
		return ad463x_spi_reg_read(st, AD463X_CONFIG_TIMING, &dummy);
	else
		return ad463x_spi_reg_write(st, AD463X_REG_EXIT_CFG_MODE,
					AD463X_EXIT_CFG_MODE);
}

static int ad463x_reg_access_in_data_read(struct ad463x_state *st)
{
	int ret;
	if(adc_mode == AD463x_DATA_MODE)
		{
			if(st->spi_is_dma_mapped)
				{
					spi_engine_offload_enable(st->spi, false);
					spi_bus_unlock(st->spi->master);
				}

			ret = ad463x_set_reg_access(st, true);
			if (ret < 0)
				return ret;
			adc_mode = AD463x_REG_MODE;
		}
	return ret;
}

static int ad463x_reg_access(struct iio_dev *indio_dev,
			     u32 reg, u32 writeval,
			     u32 *readval)
{
	struct ad463x_state *st = iio_priv(indio_dev);
	u8 reg_data[2];
	int ret;

	mutex_lock(&indio_dev->mlock);
	ad463x_reg_access_in_data_read(st);

	if (readval == NULL) {
		ret = ad463x_spi_reg_write(st, reg, writeval);
	} else {
		ad463x_spi_reg_read(st, reg, reg_data);
		*readval = reg_data[0];
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int ad463x_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long info)
{
	struct ad463x_state *st = iio_priv(indio_dev);
	int scale_uv;

	switch (info) {
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

static ssize_t ad463x_read_vcm(struct iio_dev *dev, uintptr_t private,
				const struct iio_chan_spec *chan, char *buf)
{
	struct ad463x_state *st = iio_priv(dev);
	unsigned int vcom_vtg;

	if (chan->channel == 0)
		spi_engine_read_reg(st->spi, VCOM0_REG, &vcom_vtg);
	else
		spi_engine_read_reg(st->spi, VCOM1_REG, &vcom_vtg);

	return sprintf(buf, "%d\n", (unsigned char) vcom_vtg);
}

const struct iio_chan_spec_ext_info ad463x_ext_info = {
	.name = "common_mode_voltage",
	.shared = IIO_SEPARATE,
	.read = ad463x_read_vcm,
};

DECLARE_AD463x_CHANNELS(ad4630_channels, 24, 64);

static struct ad463x_chip_info ad463x_chip_info_tbl[] = {
	[ID_AD4630_24] = {
		.channels = ad4630_channels,
		.num_channels = ARRAY_SIZE(ad4630_channels),
		.freq_samp = 2000000,
		.cnv_width = 0x25,
		.cnv_rate = 0x50,
		.cnv_rate_1_lane_sdr = 0x60,
	},
};

static int ad463x_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad463x_state *st = iio_priv(indio_dev);
	u8 data[4];
	u8 hdl_reg;
	u8 num_clock = 8;
	u16 dataLane2[2];
	int ret;
	struct mode_info m_info;

	ret = get_mode_info(st, &m_info);

    hdl_reg = 1 << (4 + (m_info.lane_mode >> 6));
    hdl_reg |= (m_info.clock_mode >> 2);
    hdl_reg |= (m_info.ddr_mode >> 2);
    hdl_reg |= (m_info.out_data_mode >> 2);

    if(m_info.ddr_mode == AD463X_DUAL_DATA_RATE)
    	num_clock /= 2;

	ret = ad463x_set_reg_access(st, false);
		if (ret < 0)
			return ret;

	adc_mode = AD463x_DATA_MODE;
	spi_engine_write_reg(st->spi,CNV_RATE_CONFIG,st->chip_info->cnv_rate);
	spi_engine_write_reg(st->spi,UP_CONFIG_REG,hdl_reg); /* Update mode configuration in HDL */

	if( m_info.lane_mode == AD463X_FOUR_LANES_PER_CH){

		struct spi_transfer xfer[] = {
			{
			.tx_buf = NULL,
			.rx_buf = data,
			.len = 1,
			.bits_per_word = num_clock,
			}
		};

		spi_bus_lock(st->spi->master);
		spi_message_init_with_transfers(&st->spi_msg, xfer,ARRAY_SIZE(xfer));
		ret = spi_engine_offload_load_msg(st->spi, &st->spi_msg);
		if (ret < 0)
			return ret;
		spi_engine_offload_enable(st->spi, true);
		spi_engine_write_reg(st->spi,CNV_CONFIG_REG,CNV_ENABLE(1) |  UP_RESETN(1));
	}
	else if( m_info.lane_mode == AD463X_TWO_LANES_PER_CH){

		struct spi_transfer xfer[] = {
			{
			.tx_buf = NULL,
			.rx_buf = &dataLane2[0],
			.len = 1,
			.bits_per_word = num_clock,
			},
			{
			.tx_buf = NULL,
			.rx_buf = &dataLane2[1],
			.len = 1,
			.bits_per_word = num_clock,
			}
		};

		spi_bus_lock(st->spi->master);
		spi_message_init_with_transfers(&st->spi_msg, xfer,ARRAY_SIZE(xfer));
		ret = spi_engine_offload_load_msg(st->spi, &st->spi_msg);
		if (ret < 0)
			return ret;
		spi_engine_offload_enable(st->spi, true);
		spi_engine_write_reg(st->spi,CNV_CONFIG_REG,CNV_ENABLE(1) |  UP_RESETN(1));
	}
	else if( m_info.lane_mode == AD463X_ONE_LANE_PER_CH){

		struct spi_transfer xfer[] = {
			{
			.tx_buf = NULL,
			.rx_buf = &data[0],
			.len = 1,
			.bits_per_word = num_clock,
			},
			{
			.tx_buf = NULL,
			.rx_buf = &data[1],
			.len = 1,
			.bits_per_word = num_clock,
			},
			{
			.tx_buf = NULL,
			.rx_buf = &data[2],
			.len = 1,
			.bits_per_word = num_clock,
			},
			{
			.tx_buf = NULL,
			.rx_buf = &data[3],
			.len = 1,
			.bits_per_word = num_clock,
			}
		};

		if(m_info.ddr_mode == AD463X_SINGLE_DATA_RATE)
			spi_engine_write_reg(st->spi,CNV_RATE_CONFIG,st->chip_info->cnv_rate_1_lane_sdr);

		spi_bus_lock(st->spi->master);
		spi_message_init_with_transfers(&st->spi_msg, xfer,ARRAY_SIZE(xfer));
		ret = spi_engine_offload_load_msg(st->spi, &st->spi_msg);
		if (ret < 0)
			return ret;
		spi_engine_offload_enable(st->spi, true);
		spi_engine_write_reg(st->spi,CNV_CONFIG_REG,CNV_ENABLE(1) |  UP_RESETN(1));
	}
	return 0;
}

static int ad463x_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad463x_state *st = iio_priv(indio_dev);
	int ret;

	spi_engine_offload_enable(st->spi, false);
	spi_bus_unlock(st->spi->master);

	ret = ad463x_set_reg_access(st, true);
	if (ret < 0)
		return ret;
	adc_mode = AD463x_REG_MODE;
	return ret;
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

static const struct iio_buffer_setup_ops ad463x_buffer_setup_ops = {
	.postenable = &ad463x_buffer_postenable,
	.predisable = &ad463x_buffer_predisable,
};

static const struct iio_info ad463x_info = {
	.read_raw = &ad463x_read_raw,
	.debugfs_reg_access = &ad463x_reg_access,
};

static int ad463x_setup(struct ad463x_state *st)
{
	int ret;

	st->gpio_reset = devm_gpiod_get(&st->spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	gpiod_direction_output(st->gpio_reset, 1);
	gpiod_set_value(st->gpio_reset, 0);

	spi_engine_write_reg(st->spi,CNV_CONFIG_REG,0x00);// to disable up_resetn and up_cnv pulse

	ret = ad463x_set_reg_access(st, true);
		if (ret < 0)
			return ret;
	adc_mode = AD463x_REG_MODE;

	ad463x_spi_reg_write(st, AD463X_REG_INTERFACE_CONFIG_A, 0x10);
	ad463x_spi_reg_write(st, AD463X_REG_INTERFACE_CONFIG_B, 0x80);
	ad463x_spi_reg_write(st, AD463X_REG_DEVICE_CONFIG, 0x00);
	ad463x_spi_reg_write(st, AD463X_REG_AVG, 0x00);
	ad463x_spi_reg_write(st, AD463X_REG_MODES, 0x82);
	ad463x_spi_reg_write(st, AD463X_REG_OSCILATOR, 0x00);
	ad463x_spi_reg_write(st, AD463X_REG_IO, 0x01);
	ad463x_spi_reg_write(st, AD463X_REG_PAT0, 0x00);
	ad463x_spi_reg_write(st, AD463X_REG_PAT1, 0x30);
	ad463x_spi_reg_write(st, AD463X_REG_PAT2, 0x46);
	ad463x_spi_reg_write(st, AD463X_REG_PAT3, 0xAD);

	spi_engine_write_reg(st->spi,CNV_RATE_CONFIG,st->chip_info->cnv_rate);// value to calculate as 1000ns*spi_clk_frequency **
	spi_engine_write_reg(st->spi,CNV_PULSE_WIDTH,st->chip_info->cnv_width);// value to calculate as 250ns*spi_clk_frequency  **

	return 0;
}

static int ad463x_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	struct ad463x_state *st;
	int ret ,dev_id;
	
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	dev_id = spi_get_device_id(spi)->driver_data;
	st = iio_priv(indio_dev);
	st->chip_info = &ad463x_chip_info_tbl[spi_get_device_id(spi)->driver_data];
	
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
	st->spi_is_dma_mapped = spi_engine_offload_supported(spi);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE |INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad463x_buffer_setup_ops;
	indio_dev->info = &ad463x_info;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	st->num_bits = indio_dev->channels->scan_type.realbits;

	ret = ad463x_setup(st);
	if (ret < 0) {
		dev_err(&spi->dev, "%s setup failed\n", indio_dev->name);
		return -ENOEXEC;
	}

	buffer = iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "rx",
					    &dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret < 0)
		goto error;

	return 0;

error:
	iio_dmaengine_buffer_free(indio_dev->buffer);
	regulator_disable(st->vref);

	return ret;
}

static int ad463x_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad463x_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	regulator_disable(st->vref);

	return 0;
}

static const struct spi_device_id ad463x_id[] = {
	{ "ad4630", ID_AD4630_24 },
	{}
};

static const struct of_device_id ad463x_of_match[] = {
        { .compatible = "adi,ad4630" },
        { },
};

MODULE_DEVICE_TABLE(spi, ad463x_id);

static struct spi_driver ad463x_driver = {
	.driver = {
		.name = "ad463x",
	},
	.probe = ad463x_probe,
	.remove = ad463x_remove,
	.id_table = ad463x_id,
};
module_spi_driver(ad463x_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD463x ADC driver");
MODULE_LICENSE("GPL v2");
