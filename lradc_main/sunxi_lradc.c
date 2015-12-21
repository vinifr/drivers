/*
 *  Support for Allwinner LRADC in A1X SoCs
 *
 *  based on linux/drivers/iio/adc/at91_adc.c
 *
 *  Copyright 2011 Free Electrons
 *
 *  Copyright 2013 Vinicius Freire <viniciusfre@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/bitmap.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

/* LRADC registers definitions */

/* LRADC Control Register */
#define SUNXI_LRADC_CTRL		(0x00)
/* LRADC Interrupt Control Register */
#define SUNXI_LRADC_INTC		(0x04)
/* LRADC Interrupt Status Register */
#define SUNXI_LRADC_INTS		(0x08)
/* LRADC Data 0 Register */
#define SUNXI_LRADC_DATA0		(0x0C)
/* LRADC Data 1 Register */
#define SUNXI_LRADC_DATA1		(0x10)

/* Bit definitions for LRADC_CTRL */
#define SUNXI_LRADC_CTRL_FST_DLY(x)	((x & 0x7F) << 24)
#define SUNXI_LRADC_CTRL_CHAN_SEL(x)	((x & 0x03) << 22)
#define SUNXI_LRADC_CTRL_TIME_SEL(x)	((x & 0x0F) << 16)
#define SUNXI_LRADC_CTRL_KEY_MODE(x)	((x & 0x03) << 12)
#define SUNXI_LRADC_CTRL_LVLA_B(x)	((x & 0x0F) << 8)
#define SUNXI_LRADC_CTRL_HOLD_EN(x)	((x) << 6)
#define SUNXI_LRADC_CTRL_LVLB(x)	((x & 0x03) << 4)
#define SUNXI_LRADC_CTRL_S_RATE(x)	((x & 0x03) << 2)
#define SUNXI_LRADC_CTRL_EN(x)		((x) << 0)

/* Bit definitions for LRADC_INTC */
#define SUNXI_LRADC_INTC_KEYUP(x)	BIT((x * 8) + 4)
#define SUNXI_LRADC_INTC_ALR_HOLD(x)	BIT((x * 8) + 3)
#define SUNXI_LRADC_INTC_HOLD(x)	BIT((x * 8) + 2)
#define SUNXI_LRADC_INTC_KEYDOWN(x)	BIT((x * 8) + 1)
#define SUNXI_LRADC_INTC_DATA(x)	BIT((x * 8) + 0)

/* Bit definitions for LRADC_INTS */
#define SUNXI_LRADC_INTS_KEYUP(x)	((x * 8) + 4)
#define SUNXI_LRADC_INTS_ALR_HOLD(x)	((x * 8) + 3)
#define SUNXI_LRADC_INTS_HOLD(x)	((x * 8) + 2)
#define SUNXI_LRADC_INTS_KEYDOWN(x)	((x * 8) + 1)
#define SUNXI_LRADC_INTS_DATA(x)	((x * 8) + 0)

#define SUNXI_LRADC_SAMPLE_RATE		0x0C

/* Allwinner LRADC A13, A10, A10S and A20 have 2 channels */
#define LRADC_SUNXI_NUM_CHANNELS	2
#define LRADC_SUNXI_CHANNEL_MASK	0x03
#define LRADC_SUNXI_RESOLUTION		6
#define LRADC_SUNXI_MAX_SAMPLES		4
#define LRADC_SUNXI_FIRST_DLY		(2<<24)

#define  LRADC_SUNXI_SAMPLE_32HZ	(3)
#define  LRADC_SUNXI_SAMPLE_62HZ	(2)
#define  LRADC_SUNXI_SAMPLE_125HZ	(1)
#define  LRADC_SUNXI_SAMPLE_250HZ	(0)

struct sunxi_lradc_state {
	unsigned long		channels_mask;
	u8			channel_cur;
	bool			done;
	int			irq;
	u16			last_value;
	struct mutex		lock;
	u8			num_channels;
	void __iomem		*reg_base;
	u16			vref_mv;
	struct regulator	*reg;
	wait_queue_head_t	wq_data_avail;
	u32			inter_mask;
};

static int sunxi_lradc_readl(struct sunxi_lradc_state *info, u32 reg)
{
	return readl(info->reg_base + reg);
}

static void sunxi_lradc_writel(struct sunxi_lradc_state *info, u32 reg, u32 val)
{
	writel(val, info->reg_base + reg);
}

static irqreturn_t sunxi_lradc_irq(int irq, void *private)
{
	struct iio_dev *idev = private;
	struct sunxi_lradc_state *st = iio_priv(idev);
	
	u32 status = sunxi_lradc_readl(st, SUNXI_LRADC_INTS);
	if (!(status & st->inter_mask))
		return IRQ_NONE;
	st->last_value = sunxi_lradc_readl(st, st->channel_cur);
	st->done = true;
	wake_up_interruptible(&st->wq_data_avail);

	return IRQ_HANDLED;
}

static int sunxi_lradc_channel_init(struct iio_dev *idev)
{
	struct sunxi_lradc_state *st = iio_priv(idev);
	struct iio_chan_spec *chan_array;
	int bit, idx = 0;

	st->channels_mask = LRADC_SUNXI_CHANNEL_MASK;
	st->num_channels = LRADC_SUNXI_NUM_CHANNELS;
	idev->num_channels = bitmap_weight(&st->channels_mask,
					   st->num_channels) + 1;

	chan_array = devm_kzalloc(&idev->dev,
				  ((idev->num_channels + 1) *
					sizeof(struct iio_chan_spec)),
				  GFP_KERNEL);

	if (!chan_array)
		return -ENOMEM;

	for_each_set_bit(bit, &st->channels_mask, st->num_channels) {
		struct iio_chan_spec *chan = chan_array + idx;

		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = bit;
		chan->scan_index = idx;
		chan->scan_type.sign = 'u';
		chan->scan_type.realbits = LRADC_SUNXI_RESOLUTION;
		chan->scan_type.storagebits = 32;
		chan->info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE);
		chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		idx++;
	}

	idev->channels = chan_array;
	return idev->num_channels;
}

static int sunxi_lradc_read_raw(struct iio_dev *idev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct sunxi_lradc_state *st = iio_priv(idev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);

		sunxi_lradc_writel(st, SUNXI_LRADC_CTRL,
				SUNXI_LRADC_CTRL_CHAN_SEL(chan->channel));
		st->channel_cur = chan->channel;
		/* Enables interruption for correspondent channel only */
		if(st->channel_cur == 0)
			st->inter_mask = SUNXI_LRADC_INTC_DATA(0);
		else
			st->inter_mask = SUNXI_LRADC_INTC_DATA(1);
		sunxi_lradc_writel(st, SUNXI_LRADC_INTC, st->inter_mask);
		sunxi_lradc_writel(st, SUNXI_LRADC_CTRL, SUNXI_LRADC_CTRL_EN(1));

		ret = wait_event_interruptible_timeout(st->wq_data_avail,
						       st->done,
						       msecs_to_jiffies(1000));
		if (ret == 0)
			ret = -ETIMEDOUT;
		if (ret < 0) {
			mutex_unlock(&st->lock);
			return ret;
		}

		*val = st->last_value;

		sunxi_lradc_writel(st, SUNXI_LRADC_CTRL, SUNXI_LRADC_CTRL_EN(0));
		sunxi_lradc_writel(st, SUNXI_LRADC_INTC, 0x00);

		st->last_value = 0;
		st->done = false;
		mutex_unlock(&st->lock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = (st->vref_mv * 1000) >> chan->scan_type.realbits;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		break;
	}
	return -EINVAL;
}

static int sunxi_lradc_check_sampling_freq(const char *buf)
{
	int i;
	const char * const samples[] = { "0", "1", "2", "3",
	};
	
	for (i = 0; i < LRADC_SUNXI_MAX_SAMPLES; i++) {
		if (sysfs_streq(buf, samples[i]))
			return i;
	}
	return -EINVAL;
}

static ssize_t sunxi_lradc_set_sampling_freq(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct iio_dev *idev = dev_to_iio_dev(dev);
	struct platform_device *pdev = to_platform_device(idev->dev.parent);
	struct sunxi_lradc_state *st = iio_priv(idev);
	u32 rate;
	
	rate = sunxi_lradc_check_sampling_freq(buf);
	if (rate < 0) {
		dev_err(&pdev->dev,
			"sampling frequency is not supported\n");
		return rate;
	}
	sunxi_lradc_writel(st, SUNXI_LRADC_CTRL, 
			   SUNXI_LRADC_CTRL_S_RATE(rate));
	return 0;
}

static ssize_t sunxi_lradc_show_sampling_freq(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *idev = dev_to_iio_dev(dev);
	//struct platform_device *pdev = to_platform_device(idev->dev.parent);
	//struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct sunxi_lradc_state *st = iio_priv(idev);
	u32 rate;

	rate = sunxi_lradc_readl(st, SUNXI_LRADC_CTRL) &
				SUNXI_LRADC_SAMPLE_RATE;

	return rate;
}

static IIO_DEVICE_ATTR(sampling_frequency,
			S_IWUSR | S_IRUGO,
			sunxi_lradc_show_sampling_freq,
			sunxi_lradc_set_sampling_freq,
			0);

static struct attribute *sunxi_lradc_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	NULL
};

static const struct attribute_group sunxi_lradc_attribute_group = {
	.attrs = sunxi_lradc_attributes,
};

static const struct iio_info sunxi_lradc_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &sunxi_lradc_read_raw,
	.attrs = &sunxi_lradc_attribute_group,
};

static int sunxi_lradc_probe(struct platform_device *pdev)
{
	int ret;
	struct iio_dev *idev;
	struct sunxi_lradc_state *st;
	struct resource *res;
	u32 reg;

	idev = iio_device_alloc(sizeof(struct sunxi_lradc_state));
	//idev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (idev == NULL) {
		return -ENOMEM;
	}

	st = iio_priv(idev);

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "Could not find valid DT data.\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, idev);

	idev->dev.parent = &pdev->dev;
	idev->name = dev_name(&pdev->dev);
	idev->modes = INDIO_DIRECT_MODE;
	idev->info = &sunxi_lradc_info;

	st->irq = platform_get_irq(pdev, 0);
	if (st->irq < 0) {
		dev_err(&pdev->dev, "No IRQ ID is designated\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	st->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(st->reg_base)) {
		ret = PTR_ERR(st->reg_base);
		return ret;
	}

	/* Disable all IRQs */
	sunxi_lradc_writel(st, SUNXI_LRADC_INTC, 0x00);
	ret = devm_request_irq(&pdev->dev,
		               st->irq,
			       sunxi_lradc_irq,
			       0,
			       pdev->dev.driver->name,
			       idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to allocate IRQ.\n");
		return ret;
	}
	
	st->reg = devm_regulator_get(&pdev->dev, "vref");	
	if (IS_ERR(st->reg)) {
		dev_err(&pdev->dev, "failed getting regulator");
		return PTR_ERR(st->reg);
	}
	ret = regulator_enable(st->reg);
	if (ret)
		return ret;
	st->vref_mv = regulator_get_voltage(st->reg);
	st->vref_mv /= 1000;

	reg = SUNXI_LRADC_CTRL_FST_DLY(LRADC_SUNXI_FIRST_DLY);
	reg |= SUNXI_LRADC_CTRL_S_RATE(LRADC_SUNXI_SAMPLE_250HZ);
	reg |= SUNXI_LRADC_CTRL_HOLD_EN(0);
		
	sunxi_lradc_writel(st, SUNXI_LRADC_CTRL, reg);

	/* Setup the ADC channels available on the board */
	ret = sunxi_lradc_channel_init(idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't initialize the channels.\n");
		goto error_reg;
	}

	init_waitqueue_head(&st->wq_data_avail);
	mutex_init(&st->lock);

	ret = iio_device_register(idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't register the device.\n");
		goto error_reg;
	}

	return 0;
error_reg:
	regulator_disable(st->reg);
	return ret;
}

static int sunxi_lradc_remove(struct platform_device *pdev)
{
	struct iio_dev *idev = platform_get_drvdata(pdev);
	//struct sunxi_lradc_state *st = iio_priv(idev);

	iio_device_unregister(idev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sunxi_lradc_dt_ids[] = {
	{.compatible = "allwinner,sun4i-lradc"},
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_lradc_dt_ids);
#endif

static struct platform_driver sunxi_lradc_driver = {
	.probe = sunxi_lradc_probe,
	.remove = sunxi_lradc_remove,
	.driver = {
		.name = "sunxi_lradc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sunxi_lradc_dt_ids),
	},
};

module_platform_driver(sunxi_lradc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Sunxi A1X LRADC Driver");
MODULE_AUTHOR("Vinicius Freire <viniciusfre@gmail.com>");
