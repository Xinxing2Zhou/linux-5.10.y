/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/partitions.h>
#include <linux/of_platform.h>
#include <linux/mfd/bsp_fmc.h>

#include <asm/setup.h>

#include <linux/securec.h>
#include "fmc100.h"

static int fmc100_spi_nand_pre_probe(struct nand_chip *chip)
{
	uint8_t nand_maf_id;
	struct fmc_host *host = chip->priv;

	/* Reset the chip first */
	host->send_cmd_reset(host);
	udelay(1000);/* 1000us */

	/* Check the ID */
	host->offset = 0;
	/* dest fmcbuf just need init ID lenth */
	if (memset_s((unsigned char *)(chip->legacy.IO_ADDR_R), FMC_MEM_LEN, 0, 0x10)) {
		return 1;
	}

	host->send_cmd_readid(host);
	nand_maf_id = fmc_readb(chip->legacy.IO_ADDR_R);
	if (nand_maf_id == 0x00 || nand_maf_id == 0xff) {
		printk("Cannot found a valid SPI Nand Device\n");
		return 1;
	}

	return 0;
}

static int fmc_nand_scan(struct mtd_info *mtd)
{
	int result;
	unsigned char cs;
	unsigned char chip_num = CONFIG_SPI_NAND_MAX_CHIP_NUM;
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct fmc_host *host = chip->priv;

	for (cs = 0; (chip_num != 0) && (cs < FMC_MAX_CHIP_NUM); cs++) {
		if (fmc_cs_user[cs]) {
			fmc_pr(BT_DBG, "\t\t*-Current CS(%d) is occupied.\n",
			       cs);
			continue;
		}

		host->cmd_op.cs = cs;

		if (fmc100_spi_nand_pre_probe(chip))
			return -ENODEV;

		fmc_pr(BT_DBG, "\t\t*-Scan SPI nand flash on CS: %d\n", cs);
		if (nand_scan_with_ids(chip, chip_num, NULL))
			continue;

		chip_num--;
	}

	if (chip_num == CONFIG_SPI_NAND_MAX_CHIP_NUM)
		result = -ENXIO;
	else
		result = 0;

	return result;
}

static int spi_nand_host_parm_init(struct platform_device *pltdev,
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

	len = sizeof(struct fmc_host) + sizeof(struct nand_chip) +
	      sizeof(struct mtd_info);
	*host = devm_kzalloc(dev, len, GFP_KERNEL);
	if (!(*host)) {
		dev_err(dev, "get host failed\n");
		return -ENOMEM;
	}
	(void)memset_s((char *)(*host), len, 0, len);

	platform_set_drvdata(pltdev, *host);
	(*host)->dev = &pltdev->dev;
	(*host)->chip = *chip = (struct nand_chip *)&(*host)[1];
	(*host)->mtd  = *mtd  = nand_to_mtd(*chip);
	(*host)->regbase = fmc->regbase;
	(*host)->iobase = fmc->iobase;
	(*host)->clk = fmc->clk;
	(*host)->lock = &fmc->lock;
	(*host)->buffer = fmc->buffer;
	(*host)->dma_buffer = fmc->dma_buffer;
	/* dest fmcbuf just need init dma_len lenth */
	if (memset_s((char *)(*host)->iobase, FMC_MEM_LEN, 0xff, fmc->dma_len)) {
		dev_err(dev, "memset_s failed\n");
		return -1;
	}
	(*chip)->legacy.IO_ADDR_R = (*chip)->legacy.IO_ADDR_W = (*host)->iobase;
	(*chip)->priv = (*host);

	return 0;
}

static int bsp_spi_nand_probe(struct platform_device *pltdev)
{
	int result;
	struct fmc_host *host = NULL;
	struct nand_chip *chip = NULL;
	struct mtd_info *mtd = NULL;
	struct device *dev = &pltdev->dev;
	struct device_node *np = NULL;

	fmc_pr(BT_DBG, "\t*-Start SPI Nand flash driver probe\n");
	result = spi_nand_host_parm_init(pltdev, dev, &host, &chip, &mtd);
	if (result) {
		fmc_pr(BT_DBG, "\t*-test0\n");
		return result;
	}
	/* Set system clock */
	result = clk_prepare_enable(host->clk);
	if (result) {
		printk("\nclk prepare enable failed!");
		goto fail;
	}

	fmc100_spi_nand_init(chip);

	np = of_get_next_available_child(dev->of_node, NULL);
	if (np == NULL) {
		printk("\nof_get_next_available_child failed!");
		goto fail;
	}
	mtd->name = np->name;
	mtd->type = MTD_NANDFLASH;
	mtd->priv = chip;
	mtd->owner = THIS_MODULE;

	result = of_property_read_u32(np, "spi-max-frequency", &host->clkrate);
	if (result) {
		printk("\nget fmc clkrate failed");
		goto fail;
	}

	result = fmc_nand_scan(mtd);
	if (result) {
		fmc_pr(BT_DBG, "\t|-Scan SPI Nand failed.\n");
		goto fail;
	}

	result = mtd_device_register(mtd, NULL, 0);
	if (result == 0) {
		fmc_pr(BT_DBG, "\t*-End driver probe !!\n");
		return 0;
	}

	result = -ENODEV;

	mtd_device_unregister(mtd);
	nand_cleanup(mtd_to_nand(mtd));

fail:
	clk_disable_unprepare(host->clk);
	db_msg("Error: driver probe, result: %d\n", result);
	return result;
}

static int bsp_spi_nand_remove(struct platform_device *pltdev)
{
	struct fmc_host *host = platform_get_drvdata(pltdev);
	unsigned char cs;

	for (cs = 0; cs < FMC_MAX_CHIP_NUM; cs++)
		fmc_cs_user[cs] = 0;

	if (host) {
		if (host->clk)
			clk_disable_unprepare(host->clk);
		if (host->mtd) {
			mtd_device_unregister(host->mtd);
			nand_cleanup(mtd_to_nand(host->mtd));
		}
	}

	return 0;
}

#ifdef CONFIG_PM
static int fmc100_os_suspend(struct platform_device *pltdev,
			       pm_message_t state)
{
	struct fmc_host *host = platform_get_drvdata(pltdev);

	if (host && host->suspend)
		return (host->suspend)(pltdev, state);

	return 0;
}

static int fmc100_os_resume(struct platform_device *pltdev)
{
	struct fmc_host *host = platform_get_drvdata(pltdev);

	if (host && host->resume)
		return (host->resume)(pltdev);

	return 0;
}
#endif /* End of CONFIG_PM */

static const struct of_device_id bsp_spi_nand_dt_ids[] = {
	{ .compatible = "vendor,spi-nand" },
	{ } /* sentinel */
};
MODULE_DEVICE_TABLE(of, bsp_spi_nand_dt_ids);

static struct platform_driver bsp_spi_nand_driver = {
	.driver = {
		.name   = "bsp_spi_nand",
		.of_match_table = bsp_spi_nand_dt_ids,
	},
	.probe  = bsp_spi_nand_probe,
	.remove = bsp_spi_nand_remove,
#ifdef CONFIG_PM
	.suspend    = fmc100_os_suspend,
	.resume     = fmc100_os_resume,
#endif
};
module_platform_driver(bsp_spi_nand_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vendor Flash Memory Controller V100 SPI Nand Driver");
