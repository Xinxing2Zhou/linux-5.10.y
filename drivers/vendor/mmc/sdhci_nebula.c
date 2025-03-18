/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>

#include "dfx/mci_proc.h"
#include "sdhci_nebula.h"

static unsigned int g_slot_index;
struct mmc_host *g_mci_host[MCI_SLOT_NUM] = {NULL};
struct mmc_host *g_sdio_mmc_host = NULL;

static const struct of_device_id g_sdhci_nebula_match[] = {
	{ .compatible = "nebula,sdhci" },
	{ .compatible = "huanglong,sdhci" },
	{},
};

MODULE_DEVICE_TABLE(of, g_sdhci_nebula_match);

static struct sdhci_ops g_sdhci_nebula_ops = {
	.platform_execute_tuning = sdhci_nebula_execute_tuning,
	.reset = sdhci_nebula_reset,
	.set_clock = sdhci_nebula_set_clock,
	.irq = sdhci_nebula_irq,
	.set_bus_width = sdhci_nebula_set_bus_width,
	.set_uhs_signaling = sdhci_nebula_set_uhs_signaling,
	.hw_reset = sdhci_nebula_hw_reset,
	.signal_voltage_switch = sdhci_nebula_voltage_switch,
	.init = sdhci_nebula_extra_init,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	.dump_vendor_regs = sdhci_nebula_dump_vendor_regs,
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	.adma_write_desc = sdhci_nebula_adma_write_desc,
#endif

};

static const struct sdhci_pltfm_data g_sdhci_nebula_pdata = {
	.ops = &g_sdhci_nebula_ops,
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
};

static int sdhci_nebula_probe(struct platform_device *pdev)
{
	int ret;
	struct sdhci_host *host;
	struct sdhci_nebula *nebula = NULL;

	host = sdhci_pltfm_init(pdev, &g_sdhci_nebula_pdata,
				sizeof(struct sdhci_nebula));
	if (IS_ERR(host))
		return (int)PTR_ERR(host);

	ret = sdhci_nebula_pltfm_init(pdev, host);
	if (ret)
		goto pltfm_free;

	nebula = nebula_priv(host);
	if (nebula->priv_cap & NEBULA_CAP_PM_RUNTIME) {
		pm_runtime_get_noresume(&pdev->dev);
		pm_runtime_set_autosuspend_delay(&pdev->dev,
			MMC_AUTOSUSPEND_DELAY_MS);
		pm_runtime_use_autosuspend(&pdev->dev);
		pm_runtime_set_active(&pdev->dev);
		pm_runtime_enable(&pdev->dev);
	}

	ret = sdhci_nebula_add_host(host);
	if (ret)
		goto pm_runtime_disable;

	if (nebula->priv_cap & NEBULA_CAP_PM_RUNTIME) {
		pm_runtime_mark_last_busy(&pdev->dev);
		pm_runtime_put_autosuspend(&pdev->dev);
	}

	ret = sdhci_nebula_proc_init(host);
	if (ret)
		goto pm_runtime_disable;

	g_mci_host[g_slot_index++] = host->mmc;

	if (nebula->devid == MMC_DEV_TYPE_SDIO_0) {
		g_sdio_mmc_host = host->mmc;
	}
	return ERET_SUCCESS;

pm_runtime_disable:
	if (nebula->priv_cap & NEBULA_CAP_PM_RUNTIME) {
		pm_runtime_disable(&pdev->dev);
		pm_runtime_set_suspended(&pdev->dev);
		pm_runtime_put_noidle(&pdev->dev);
	}

pltfm_free:
	sdhci_pltfm_free(pdev);
	return ret;
}

static int sdhci_nebula_remove(struct platform_device *pdev)
{
	int ret;
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->priv_cap & NEBULA_CAP_PM_RUNTIME) {
		pm_runtime_get_sync(&pdev->dev);
		pm_runtime_disable(&pdev->dev);
		pm_runtime_put_noidle(&pdev->dev);
	}

	sdhci_remove_host(host, true);

	ret = sdhci_nebula_proc_shutdown(host);
	if (ret != ERET_SUCCESS) {
		pr_err("failed to shutdown proc.\n");
		return ret;
	}

	if (!IS_ERR_OR_NULL(nebula->hclk))
		clk_disable_unprepare(nebula->hclk);

	clk_disable_unprepare(pltfm_host->clk);

	sdhci_pltfm_free(pdev);

	return ret;
}

static const struct dev_pm_ops g_sdhci_nebula_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sdhci_nebula_pltfm_suspend,
				sdhci_nebula_pltfm_resume)
	SET_RUNTIME_PM_OPS(sdhci_nebula_runtime_suspend,
			   sdhci_nebula_runtime_resume,
			   NULL)
};

static struct platform_driver g_sdhci_nebula_driver = {
	.probe = sdhci_nebula_probe,
	.remove = sdhci_nebula_remove,
	.driver = {
			.name = "sdhci_nebula",
			.of_match_table = g_sdhci_nebula_match,
			.pm = &g_sdhci_nebula_pm_ops,
	},
};

static int __init sdhci_bsp_init(void)
{
	int ret;

	ret = platform_driver_register(&g_sdhci_nebula_driver);
	if (ret) {
		pr_err("failed to register sdhci drv.\n");
		return ret;
	}

	ret = mci_proc_init();
	if (ret)
		platform_driver_unregister(&g_sdhci_nebula_driver);

	return ret;
}

static void __exit sdhci_bsp_exit(void)
{
	mci_proc_shutdown();

	platform_driver_unregister(&g_sdhci_nebula_driver);
}

module_init(sdhci_bsp_init);
module_exit(sdhci_bsp_exit);

MODULE_DESCRIPTION("SDHCI driver for vendor");
MODULE_AUTHOR("CompanyNameMagicTag.");
MODULE_LICENSE("GPL v2");
