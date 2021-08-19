// SPDX-License-Identifier: GPL-2.0
/*
 * AD7606 Parallel Interface ADC driver
 *
 * Copyright 2011 Analog Devices Inc.
 */
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/io.h>

#include <linux/iio/iio.h>
#include <linux/spi/spi.h>
#include "ad7606.h"

#define burst_length_reg	((0x112)<<2)	
#define data_write_reg  	((0x114)<<2)
#define data_read_reg   	((0x113)<<2)
#define cnvst_en_reg    	((0x110)<<2)
#define up_cnv_rate_reg   	((0x111)<<2)
#define PCORE_REG_VERSION	((0x100)<<2)
#define PCORE_VERSION(x)	((x >> 12) & 0xf)

#define AD7606_CONFIGURATION_REGISTER	0x02
#define AD7606_SINGLE_DOUT		0x0
#define AD7606_QUAD_DOUT		0x10

/* AD7606_RANGE_CH_X_Y */
#define AD7606_RANGE_CH_MSK(ch)		(GENMASK(3, 0) << (4 * ((ch) % 2)))
#define AD7606_RANGE_CH_MODE(ch, mode)	\
	((GENMASK(3, 0) & mode) << (4 * ((ch) % 2)))
#define AD7606_RANGE_CH_ADDR(ch)	(0x03 + ((ch) >> 1))
#define AD7606_OS_MODE			0x08

struct spi_engine_program {
	unsigned int length;
	uint16_t instructions[];
};

struct spi_engine {
	struct clk *clk;
	struct clk *ref_clk;

	struct spi_master *master;

	spinlock_t lock;

	void __iomem *base;

	struct spi_message *msg;
	struct spi_engine_program *p;
	unsigned cmd_length;
	const uint16_t *cmd_buf;

	struct spi_transfer *tx_xfer;
	unsigned int tx_length;
	const uint8_t *tx_buf;

	struct spi_transfer *rx_xfer;
	unsigned int rx_length;
	uint8_t *rx_buf;

	unsigned int sync_id;
	unsigned int completed_id;

	unsigned int int_enable;

	struct timer_list watchdog_timer;
	unsigned int word_length;
};

static const unsigned int ad7606B_oversampling_avail[9] = {
	1, 2, 4, 8, 16, 32, 64, 128, 256
};

static const struct iio_chan_spec ad7606B_soft_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(8),
	AD7606B_CHANNEL(0),
	AD7606B_CHANNEL(1),
	AD7606B_CHANNEL(2),
	AD7606B_CHANNEL(3),
	AD7606B_CHANNEL(4),
	AD7606B_CHANNEL(5),
	AD7606B_CHANNEL(6),
	AD7606B_CHANNEL(7),
};

static u16 ad7606B_par_rd_wr_cmd(int addr, char isWriteOp)
{
	return (addr & 0x3F) | (((~isWriteOp) & 0x1) << 6);
}

static void ad7606_par_burst_length(struct ad7606_state *st,
				   int count)
{
	writel(count , (unsigned long)st->base_address + burst_length_reg);
}

void switch_register_ADC_mode(struct ad7606_state *st, char mode)
{
	ad7606_par_burst_length(st,0);
	if (mode == reg_mode){
	writel( 0xAF , (unsigned long)st->base_address + data_write_reg);//to switch to register mode
	readl((unsigned long)st->base_address + data_read_reg);//to switch to register mode}
	}else {
	writel( 0x00 , (unsigned long)st->base_address + data_write_reg);//to switch to ADC mode
	}
}

static int ad7606_par16_read_block(struct ad7606_state *st,
				   int count, void *buf)
{
	int i;
	unsigned short *data = buf;
	switch_register_ADC_mode(st,ADC_mode);
	writel(0xd0,(unsigned long)st->base_address + up_cnv_rate_reg);// to configure convst rate
	writel(0x03,(unsigned long)st->base_address + cnvst_en_reg);// to enable convst signal
	ndelay(800);
	writel(0x00,(unsigned long)st->base_address + cnvst_en_reg);// to disable convst signal
	for(i=0; i<count; i++){
		 st->data[i+2]=(u16)(readl((unsigned long)st->base_address + data_read_reg));}
	switch_register_ADC_mode(st,reg_mode);
	return 0;
}

static int ad7606_par_reg_write(struct ad7606_state *st,
				  unsigned int addr, unsigned int val)
{
	uint16_t temp;
	ad7606_par_burst_length(st,0);
	temp= (u16)(((addr & 0x007f)<<8) + (val & 0x00ff));
	writel( temp , (unsigned long)st->base_address + data_write_reg);
	return 0;
}

static int ad7606_par_reg_read(struct ad7606_state *st,
				  unsigned int addr)
{
	uint16_t temp;
	ad7606_par_burst_length(st,0);
	temp= (u16)((0x80 | (addr & 0x007f))<<8);
	writel( temp , (unsigned long)st->base_address + data_write_reg);
	return (uint8_t)(readl((unsigned long)st->base_address + data_read_reg));;
}

static int ad7606_par_data_read(struct ad7606_state *st)
{
	unsigned int reg;
	switch_register_ADC_mode(st,ADC_mode);
	writel(0xd0,(unsigned long)st->base_address + up_cnv_rate_reg);// to configure convst rate
	writel(0x03,(unsigned long)st->base_address + cnvst_en_reg);// to enable convst signal
	ad7606_par_burst_length(st,3);
	reg=readl((unsigned long)st->base_address + data_read_reg);

	return 0;
}


static int ad7606_par_data_write(struct ad7606_state *st)
{
	unsigned int reg;
	writel(0x00,(unsigned long)st->base_address + cnvst_en_reg);// to disable convst signal
	switch_register_ADC_mode(st,reg_mode);
	return 0;
}

static int ad7606_par_write_mask(struct ad7606_state *st,
				 unsigned int addr,
				 unsigned long mask,
				 unsigned int val)
{
	int readval=0;

	readval = ad7606_par_reg_read(st, addr);
	if (readval < 0)
		return readval;

	readval &= ~mask;
	readval |= val;

	return ad7606_par_reg_write(st, addr, readval);
}

static int ad7606_write_scale_sw(struct iio_dev *indio_dev, int ch, int val)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	return ad7606_par_write_mask(st,
				     AD7606_RANGE_CH_ADDR(ch),
				     AD7606_RANGE_CH_MSK(ch),
				     AD7606_RANGE_CH_MODE(ch, val));
}

static int ad7606_write_os_sw(struct iio_dev *indio_dev, int val)
{	
	struct ad7606_state *st = iio_priv(indio_dev);
	return ad7606_par_reg_write(st, AD7606_OS_MODE, val);
}


static int ad7606B_sw_mode_config(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	unsigned int buf[3];

	/*
	 * Software mode is enabled when all three oversampling
	 * pins are set to high. If oversampling gpios are defined
	 * in the device tree, then they need to be set to high,
	 * otherwise, they must be hardwired to VDD
	 */
	if (st->gpio_os) {
		memset32(buf, 1, ARRAY_SIZE(buf));
		gpiod_set_array_value(ARRAY_SIZE(buf),
				      st->gpio_os->desc, buf);
	}
	/* OS of 128 and 256 are available only in softwar(spi, false);*/
	st->oversampling_avail = ad7606B_oversampling_avail;
	st->num_os_ratios = ARRAY_SIZE(ad7606B_oversampling_avail);

	st->write_scale = ad7606_write_scale_sw;
	st->write_os = &ad7606_write_os_sw;

	/* Configure device spi to output on a single channel */
	st->bops->reg_write(st,
			    AD7606_CONFIGURATION_REGISTER,
			   AD7606_QUAD_DOUT); //converted to the quad Dout by user
	/*
	 * Scale can be configured individually for each channel
	 * in software mode.
	 */
	indio_dev->channels = ad7606B_soft_channels;

	return 0;
}

static const struct ad7606_bus_ops ad7606_par16_bops = {
	.read_block = ad7606_par16_read_block,
};

static const struct ad7606_bus_ops ad7606B_par_bops = {
	.read_block = ad7606_par16_read_block,
	.offload_enable = ad7606_par_data_read,
	.offload_disable = ad7606_par_data_write,
	.reg_read = ad7606_par_reg_read,
	.reg_write = ad7606_par_reg_write,
	.write_mask = ad7606_par_write_mask,
	.rd_wr_cmd = ad7606B_par_rd_wr_cmd,
	.sw_mode_config = ad7606B_sw_mode_config,
};


static int spi_engine_transfer_one_message(struct spi_master *master,
	struct spi_message *msg)
{	
	printk("dummy function for probing");
}

static int ad7606_par_probe(struct spi_device *spi)
{
	void __iomem *addr;
	const struct spi_device_id *id = spi_get_device_id(spi);
	const struct ad7606_bus_ops *bops;
	
	switch (id->driver_data) {
	case ID_AD7616:
		bops = &ad7606B_par_bops;
		break;
	case ID_AD7606B:
		bops = &ad7606B_par_bops;
		break;
	default:
		bops = &ad7606B_par_bops;
		break;
	}
		
	struct spi_engine *spi_engine = spi_master_get_devdata(spi->master);
	addr = spi_engine->base;
	if (IS_ERR(addr))
		return PTR_ERR(addr);
	
	return ad7606_probe(&spi->dev, 1,addr,
			    id->name,id->driver_data,
			    bops);
}



static const struct of_device_id ad7606_of_match[] = {
	{ .compatible = "adi,ad7605-4" },
	{ .compatible = "adi,ad7606-4" },
	{ .compatible = "adi,ad7606-6" },
	{ .compatible = "adi,ad7606-8" },
	{ .compatible = "adi,ad7606b" },
	{ },
};
MODULE_DEVICE_TABLE(of, ad7606_of_match);

static const struct spi_device_id ad7606_driver_ids[] = {
	{ "ad7605-4", ID_AD7605_4 },
	{ "ad7606-4", ID_AD7606_4 },
	{ "ad7606-6", ID_AD7606_6 },
	{ "ad7606-8", ID_AD7606_8 },
	{ "ad7606b",  ID_AD7606B },
	{ "ad7616",   ID_AD7616 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad7606_driver_ids);

static struct spi_driver ad7606_driver = {
	.driver = {
		.name = "ad7606",
		.of_match_table = ad7606_of_match,
		.pm = AD7606_PM_OPS,
	},
	.probe = ad7606_par_probe,
	.id_table = ad7606_driver_ids,
};
module_spi_driver(ad7606_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices AD7606 ADC");
MODULE_LICENSE("GPL v2");
