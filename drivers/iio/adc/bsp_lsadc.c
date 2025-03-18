/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/iio.h>

#define VENDOR_LSADC_CONFIG		0x00
#define VENDOR_CONFIG_DEGLITCH	BIT(17)
#define VENDOR_CONFIG_RESET		BIT(15)
#define VENDOR_CONFIG_MODE		BIT(13)
#define VENDOR_CONFIG_EQU_MODE	BIT(12)
#define VENDOR_CONFIG_CHN3		BIT(11)
#define VENDOR_CONFIG_CHN2		BIT(10)
#define VENDOR_CONFIG_CHN1		BIT(9)
#define VENDOR_CONFIG_CHN0		BIT(8)

#define VENDOR_CONFIG_EQU_CHN3	BIT(3)
#define VENDOR_CONFIG_EQU_CHN2	BIT(2)
#define VENDOR_CONFIG_EQU_CHN1	BIT(1)
#define VENDOR_CONFIG_EQU_CHN0	BIT(0)

#define VENDOR_LSADC_TIMESCAN	0x08
#define VENDOR_LSADC_INTEN		0x10
#define VENDOR_LSADC_INTSTATUS	0x14
#define VENDOR_LSADC_INTCLR		0x18
#define VENDOR_LSADC_START		0x1C
#define VENDOR_LSADC_STOP		0x20
#define VENDOR_LSADC_ACTBIT		0x24
#define VENDOR_LSADC_CHNDATA		0x2C
#define VENDOR_LSADC_CHN_EQU_DATA	0x50

#define VENDOR_LSADC_CON_EN		(1u << 0)
#define VENDOR_LSADC_CON_DEN		(0u << 0)

#define VENDOR_LSADC_NUM_BITS		10
#define VENDOR_LSADC_CHN_MASK		0xF

/* fix clk:3000000, default tscan set 10ms */
#define VENDOR_LSADC_TSCAN_MS		(10*3000)

#define VENDOR_LSADC_TIMEOUT		msecs_to_jiffies(100)

/* default voltage scale for every channel <mv> */
static int g_vendor_lsadc_voltage[] = {
	1800, 1800, 1800, 1800
};

struct vendor_lsadc {
	void __iomem		*regs;
	struct clk *lsadc_clk;
	int irq;
	struct completion	completion;
	struct reset_control	*reset;
	const struct vendor_lsadc_data	*data;
	unsigned int		cur_chn;
	unsigned int		value;
	unsigned int		average_value;
};

struct vendor_lsadc_data {
	int				num_bits;
	const struct iio_chan_spec	*channels;
	int				num_channels;

	void (*clear_irq)(struct vendor_lsadc *info, int mask);
	void (*start_conv)(struct vendor_lsadc *info);
	void (*stop_conv)(struct vendor_lsadc *info);
};

static int vendor_lsadc_read_raw(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan,
					int *val, int *val2, long mask)
{
	struct vendor_lsadc *info = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		reinit_completion(&info->completion);

		/* Select the channel to be used */
		info->cur_chn = chan->channel;

		if (info->data->start_conv)
			info->data->start_conv(info);

		if (!wait_for_completion_timeout(&info->completion,
							VENDOR_LSADC_TIMEOUT)) {
			if (info->data->stop_conv)
				info->data->stop_conv(info);
			mutex_unlock(&indio_dev->mlock);
			return -ETIMEDOUT;
		}

		*val = info->value;
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_AVERAGE_RAW:
		mutex_lock(&indio_dev->mlock);

		reinit_completion(&info->completion);

		/* Select the channel to be used */
		info->cur_chn = chan->channel;

		if (info->data->start_conv)
			info->data->start_conv(info);

		if (!wait_for_completion_timeout(&info->completion,
							VENDOR_LSADC_TIMEOUT)) {
			if (info->data->stop_conv)
				info->data->stop_conv(info);
			mutex_unlock(&indio_dev->mlock);
			return -ETIMEDOUT;
		}

		*val = info->average_value;
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = g_vendor_lsadc_voltage[chan->channel];
		*val2 = info->data->num_bits;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static irqreturn_t vendor_lsadc_isr(int irq, void *dev_id)
{
	struct vendor_lsadc *info = (struct vendor_lsadc *)dev_id;
	int mask;

	mask = readl(info->regs + VENDOR_LSADC_INTSTATUS);
	if ((mask & VENDOR_LSADC_CHN_MASK) == 0)
		return IRQ_NONE;

	/* Clear irq */
	mask &= VENDOR_LSADC_CHN_MASK;
	if (info->data->clear_irq)
		info->data->clear_irq(info, mask);

	/* Read value */
	info->value = readl(info->regs +
		VENDOR_LSADC_CHNDATA + (info->cur_chn << 2)); /* 2: bit 2 */
	info->value &= GENMASK(info->data->num_bits - 1, 0);

	/* Read average value */
	info->average_value = readl(info->regs +
		VENDOR_LSADC_CHN_EQU_DATA + (info->cur_chn << 2)); /* 2: bit 2 */
	info->average_value &= GENMASK(info->data->num_bits - 1, 0);

	/* stop adc */
	if (info->data->stop_conv)
		info->data->stop_conv(info);

	complete(&info->completion);

	return IRQ_HANDLED;
}

static const struct iio_info vendor_lsadc_iio_info = {
	.read_raw = vendor_lsadc_read_raw,
};

#define vendor_lsadc_channel(_index, _id) {		\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,									\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | 	\
			BIT(IIO_CHAN_INFO_SCALE) | 				\
			BIT(IIO_CHAN_INFO_AVERAGE_RAW),			\
	.datasheet_name = (_id),						\
}

static const struct iio_chan_spec vendor_lsadc_iio_channels[] = {
	vendor_lsadc_channel(0, "adc0"),
	vendor_lsadc_channel(1, "adc1"),
	vendor_lsadc_channel(2, "adc2"),
	vendor_lsadc_channel(3, "adc3"),
};

static void vendor_lsadc_clear_irq(struct vendor_lsadc *info, int mask)
{
	writel(mask, info->regs + VENDOR_LSADC_INTCLR);
}

static void vendor_lsadc_start_conv(struct vendor_lsadc *info)
{
	unsigned int con;

	/* set number bit */
	con = GENMASK(info->data->num_bits - 1, 0);
	writel(con, (info->regs + VENDOR_LSADC_ACTBIT));

	/* config */
	con = readl(info->regs + VENDOR_LSADC_CONFIG);
	con &= ~VENDOR_CONFIG_RESET; /* set to 0 */
	con &= ~VENDOR_CONFIG_EQU_MODE; /* set to 0 */
	con |= (VENDOR_CONFIG_DEGLITCH | VENDOR_CONFIG_MODE); /* set to 1 */
	con &= ~(VENDOR_CONFIG_CHN0 | VENDOR_CONFIG_CHN1 |
		VENDOR_CONFIG_CHN2 | VENDOR_CONFIG_CHN3 |
		VENDOR_CONFIG_EQU_CHN0 | VENDOR_CONFIG_EQU_CHN1 |
		VENDOR_CONFIG_EQU_CHN2 | VENDOR_CONFIG_EQU_CHN3); /* set to 0 */
	con |= (VENDOR_CONFIG_CHN0 << info->cur_chn); /* set to 1 */
	con |= (VENDOR_CONFIG_EQU_CHN0 << info->cur_chn); /* set to 1 */
	writel(con, (info->regs + VENDOR_LSADC_CONFIG));

	/* set timescan */
	writel(VENDOR_LSADC_TSCAN_MS, (info->regs + VENDOR_LSADC_TIMESCAN));

	/* clear interrupt */
	writel(VENDOR_LSADC_CHN_MASK, info->regs + VENDOR_LSADC_INTCLR);

	/* enable interrupt */
	writel(VENDOR_LSADC_CON_EN, (info->regs + VENDOR_LSADC_INTEN));

	/* start scan */
	writel(VENDOR_LSADC_CON_EN, (info->regs + VENDOR_LSADC_START));
}

static void vendor_lsadc_stop_conv(struct vendor_lsadc *info)
{
	/* reset the timescan */
	writel(VENDOR_LSADC_CON_DEN, (info->regs + VENDOR_LSADC_TIMESCAN));

	/* disable interrupt */
	writel(VENDOR_LSADC_CON_DEN, (info->regs + VENDOR_LSADC_INTEN));

	/* stop scan */
	writel(VENDOR_LSADC_CON_EN, (info->regs + VENDOR_LSADC_STOP));
}

static const struct vendor_lsadc_data lsadc_data = {
	.num_bits = VENDOR_LSADC_NUM_BITS,
	.channels = vendor_lsadc_iio_channels,
	.num_channels = ARRAY_SIZE(vendor_lsadc_iio_channels),

	.clear_irq = vendor_lsadc_clear_irq,
	.start_conv = vendor_lsadc_start_conv,
	.stop_conv = vendor_lsadc_stop_conv,
};

static const struct of_device_id vendor_lsadc_match[] = {
	{
		.compatible = "vendor,lsadc",
		.data = &lsadc_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, vendor_lsadc_match);

/* Reset LSADC Controller */
static void vendor_lsadc_reset_controller(struct reset_control *reset)
{
	reset_control_assert(reset);
	usleep_range(10, 20); /* 10, 20: us*/
	reset_control_deassert(reset);
}

static int vendor_lsadc_device_alloc(struct platform_device *pdev, struct iio_dev **indio_dev)
{
	struct vendor_lsadc *info = NULL;
	struct device_node *np = pdev->dev.of_node;

	if (np == NULL)
		return -ENODEV;

	(*indio_dev) = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if ((*indio_dev) == NULL) {
		dev_err(&pdev->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}
	return 0;
}

static void vendor_lsadc_device_free(struct platform_device *pdev, struct iio_dev *indio_dev)
{
	devm_iio_device_free(&pdev->dev, indio_dev);
}

static int vendor_lsadc_clk_prepare_enable(struct platform_device *pdev, struct iio_dev *indio_dev,
	struct vendor_lsadc **info)
{
	struct resource	*mem = NULL;
	const struct of_device_id *match = NULL;
	int ret;

	*info = iio_priv(indio_dev);
	(*info)->lsadc_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR((*info)->lsadc_clk)) {
		dev_err(&pdev->dev, "getting clock failed with %ld\n",
				PTR_ERR((*info)->lsadc_clk));
		return PTR_ERR((*info)->lsadc_clk);
	}

	match = of_match_device(vendor_lsadc_match, &pdev->dev);
	(*info)->data = match->data;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	(*info)->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR((*info)->regs))
		return PTR_ERR((*info)->regs);

	ret = clk_prepare_enable((*info)->lsadc_clk);
	if (ret < 0)
		return ret;
	return 0;
}

static void vendor_lsadc_clk_disable_unprepare(struct vendor_lsadc *info)
{
	clk_disable_unprepare(info->lsadc_clk);
}

static int vendor_lsadc_requst_irq(struct platform_device *pdev, struct vendor_lsadc *info)
{
	int ret;

	/*
	 * The reset should be an optional property, as it should work
	 * with old devicetrees as well
	 */
	info->reset = devm_reset_control_get(&pdev->dev, "lsadc-crg");
	if (IS_ERR(info->reset)) {
		ret = PTR_ERR(info->reset);
		if (ret != -ENOENT)
			return ret;

		dev_dbg(&pdev->dev, "no reset control found\n");
		info->reset = NULL;
	}

	init_completion(&info->completion);

	info->irq = platform_get_irq(pdev, 0);
	if (info->irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		ret = info->irq;
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, info->irq, vendor_lsadc_isr,
			       IRQF_SHARED, dev_name(&pdev->dev), info);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed requesting irq %d\n", info->irq);
		return ret;
	}

	if (info->reset != NULL)
		vendor_lsadc_reset_controller(info->reset);

	return 0;
}

static void vendor_lsadc_free_irq(struct platform_device *pdev, struct vendor_lsadc *info)
{
	devm_free_irq(&pdev->dev, info->irq, info);
}

static int vendor_lsadc_device_register(struct platform_device *pdev, struct iio_dev *indio_dev,
	struct vendor_lsadc *info)
{
	int ret;

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &vendor_lsadc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->driver_module = THIS_MODULE;

	indio_dev->channels = info->data->channels;
	indio_dev->num_channels = info->data->num_channels;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed register iio device\n");
		return ret;
	}

	return 0;
}

static int vendor_lsadc_probe(struct platform_device *pdev)
{
	struct vendor_lsadc *info = NULL;
	struct iio_dev *indio_dev = NULL;
	int ret;

	ret = vendor_lsadc_device_alloc(pdev, &indio_dev);
	if (ret != 0)
		return ret;

	ret = vendor_lsadc_clk_prepare_enable(pdev, indio_dev, &info);
	if (ret != 0)
		goto vendor_lsadc_out0;

	ret = vendor_lsadc_requst_irq(pdev, info);
	if (ret != 0)
		goto vendor_lsadc_out1;

	vendor_lsadc_device_register(pdev, indio_dev, info);
	if (ret != 0)
		goto vendor_lsadc_out2;

	return 0;

vendor_lsadc_out2:
	vendor_lsadc_free_irq(pdev, info);
vendor_lsadc_out1:
	vendor_lsadc_clk_disable_unprepare(info);
vendor_lsadc_out0:
	vendor_lsadc_device_free(pdev, indio_dev);

	return ret;
}

static int vendor_lsadc_remove(struct platform_device *pdev)
{
	struct vendor_lsadc *info = NULL;
	struct iio_dev *indio_dev = NULL;

	indio_dev = platform_get_drvdata(pdev);
	info = iio_priv(indio_dev);

	devm_iio_device_unregister(&pdev->dev, indio_dev);
	devm_free_irq(&pdev->dev, info->irq, info);
	clk_disable_unprepare(info->lsadc_clk);
	devm_iio_device_free(&pdev->dev, indio_dev);

	return 0;
}

static struct platform_driver vendor_lsadc_driver = {
	.probe		= vendor_lsadc_probe,
	.remove		= vendor_lsadc_remove,
	.driver		= {
		.name	= "vendor-lsadc",
		.of_match_table = vendor_lsadc_match,
	},
};

module_platform_driver(vendor_lsadc_driver);

MODULE_AUTHOR("Vendor multimedia software group");
MODULE_DESCRIPTION("Vendor multimedia software group lsadc driver");
MODULE_LICENSE("GPL v2");
