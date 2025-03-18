/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/of_platform.h>

#include "fmc100_nand_os.h"
#include "fmc100_nand.h"
#include <linux/mfd/bsp_fmc.h>
#include <linux/securec.h>

static int nand_host_parm_init(struct platform_device *pltdev,
				    struct device *dev,
				    struct fmc_host **host,
				    struct nand_chip **chip,
				    struct mtd_info **mtd)
{
	int len;
	struct bsp_fmc *fmc = dev_get_drvdata(dev->parent);

	if (!fmc) {
		dev_err(dev, "get mfd fmc devices failed\n");
		return -ENXIO;
	}

	len = sizeof(struct fmc_host) + sizeof(struct nand_chip)
	      + sizeof(struct mtd_info);
	*host = devm_kzalloc(dev, len, GFP_KERNEL);
	if (!(*host))
		return -ENOMEM;

	(void)memset_s((char *)(*host), len, 0, len);

	platform_set_drvdata(pltdev, host);

	(*host)->dev = &pltdev->dev;
	(*host)->chip = *chip = (struct nand_chip *)&(*host)[1];
	(*host)->mtd = *mtd = nand_to_mtd(*chip);
	(*host)->regbase = fmc->regbase;
	(*host)->iobase = fmc->iobase;
	(*host)->clk = fmc->clk;
	(*chip)->legacy.IO_ADDR_R = (*chip)->legacy.IO_ADDR_W = (*host)->iobase;
	(*host)->buffer = fmc->buffer;
	(*host)->dma_buffer = fmc->dma_buffer;
	(*host)->dma_len = fmc->dma_len;

	return 0;
}

static int bsp_nand_os_probe(struct platform_device *pltdev)
{
	int result;
	struct fmc_host *host = NULL;
	struct nand_chip *chip = NULL;
	struct mtd_info *mtd = NULL;
	int nr_parts = 0;
	struct mtd_partition *parts = NULL;
	struct device *dev = &pltdev->dev;
	struct device_node *np = NULL;

	result = nand_host_parm_init(pltdev, dev, &host, &chip, &mtd);
	if (result)
		return result;

	/* fmc Nand host init */
	chip->priv = host;
	result = fmc100_nand_init(chip);
	if (result) {
		db_msg("Error: host init failed! result: %d\n", result);
		goto fail;
	}

	np = of_get_next_available_child(dev->of_node, NULL);
	mtd->name = np->name;
	mtd->type = MTD_NANDFLASH;
	mtd->priv = chip;
	mtd->flags = MTD_CAP_NANDFLASH;
	mtd->owner = THIS_MODULE;

	if (nand_scan(chip, CONFIG_FMC100_MAX_NAND_CHIP)) {
		result = -ENXIO;
		goto fail;
	}

	result = mtd_device_register(host->mtd, parts, nr_parts);
	if (result) {
		kfree(parts);
		parts = NULL;
	}

	return (result == 1) ? -ENODEV : 0;

fail:
	clk_disable_unprepare(host->clk);

	mtd_device_unregister(mtd);
	nand_cleanup(mtd_to_nand(mtd));

	return result;
}

static int bsp_nand_os_remove(struct platform_device *pltdev)
{
	struct fmc_host *host = platform_get_drvdata(pltdev);

	clk_disable_unprepare(host->clk);

	mtd_device_unregister(host->mtd);
	nand_cleanup(mtd_to_nand(host->mtd));

	return 0;
}

#ifdef CONFIG_PM
static int fmc100_nand_os_suspend(struct platform_device *pltdev,
				    pm_message_t state)
{
	struct fmc_host *host = platform_get_drvdata(pltdev);
	struct device *dev = &pltdev->dev;
	if (!host || !host->clk) {
		dev_err(dev, "host or host->clk is null err\n");
		return 0;
	}

	while ((fmc_readl(host, FMC_OP) & FMC_OP_REG_OP_START))
		_cond_resched();

	while ((fmc_readl(host, FMC_OP_CTRL) & OP_CTRL_DMA_OP_READY))
		_cond_resched();

	clk_disable_unprepare(host->clk);
	fmc_pr(PM_DBG, "\t|-disable system clock\n");
	return 0;
}

static int fmc100_nand_os_resume(struct platform_device *pltdev)
{
	int cs;
	struct fmc_host *host = platform_get_drvdata(pltdev);
	struct nand_chip *chip = NULL;
	struct nand_memory_organization *memorg;

	if (!host)
		return 0;

	chip = host->chip;
	memorg = nanddev_get_memorg(&chip->base);

	for (cs = 0; cs < memorg->ntargets; cs++)
		host->send_cmd_reset(host);

	fmc100_nand_config(host);
	return 0;
}
#endif /* CONFIG_PM */

static const struct of_device_id bsp_nand_dt_ids[] = {
	{ .compatible = "vendor,bsp_nand" },
	{ } /* sentinel */
};
MODULE_DEVICE_TABLE(of, bsp_nand_dt_ids);

static struct platform_driver bsp_nand_driver = {
	.driver = {
		.name   = "bsp_nand",
		.of_match_table = bsp_nand_dt_ids,
	},
	.probe  = bsp_nand_os_probe,
	.remove = bsp_nand_os_remove,
#ifdef CONFIG_PM
	.suspend    = fmc100_nand_os_suspend,
	.resume     = fmc100_nand_os_resume,
#endif
};
module_platform_driver(bsp_nand_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Flash Memory Controller V100 Nand Driver");
