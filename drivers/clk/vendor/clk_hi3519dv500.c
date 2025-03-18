/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <dt-bindings/clock/hi3519dv500_clock.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "clk.h"
#include "crg.h"
#include "reset.h"

/* soc clk config */
static const struct bsp_fixed_rate_clock hi3519dv500_fixed_rate_clks_crg[] = {
	{ HI3519DV500_FIXED_2400M, "2400m",   NULL, 0, 2400000000, },
	{ HI3519DV500_FIXED_1200M, "1200m",   NULL, 0, 1200000000, },
	{ HI3519DV500_FIXED_1188M, "1188m",   NULL, 0, 1188000000, },
	{ HI3519DV500_FIXED_896M, "896m",    NULL, 0, 896000000, },
	{ HI3519DV500_FIXED_800M, "800m",    NULL, 0, 800000000, },
	{ HI3519DV500_FIXED_792M, "792m",    NULL, 0, 792000000, },
	{ HI3519DV500_FIXED_786M, "786m",    NULL, 0, 786000000, },
	{ HI3519DV500_FIXED_750M, "750m",    NULL, 0, 750000000, },
	{ HI3519DV500_FIXED_700M, "700m",    NULL, 0, 700000000, },
	{ HI3519DV500_FIXED_672M, "672m",    NULL, 0, 672000000, },
	{ HI3519DV500_FIXED_600M, "600m",    NULL, 0, 600000000, },
	{ HI3519DV500_FIXED_594M, "594m",    NULL, 0, 594000000, },
	{ HI3519DV500_FIXED_560M, "560m",    NULL, 0, 560000000, },
	{ HI3519DV500_FIXED_500M, "500m",    NULL, 0, 500000000, },
	{ HI3519DV500_FIXED_475M, "475m",    NULL, 0, 475000000, },
	{ HI3519DV500_FIXED_396M, "396m",    NULL, 0, 396000000, },
	{ HI3519DV500_FIXED_300M, "300m",    NULL, 0, 300000000, },
	{ HI3519DV500_FIXED_297M, "297m",    NULL, 0, 297000000, },
	{ HI3519DV500_FIXED_257M, "257m",    NULL, 0, 257000000, },
	{ HI3519DV500_FIXED_250M, "250m",    NULL, 0, 250000000, },
	{ HI3519DV500_FIXED_200M, "200m",    NULL, 0, 200000000, },
	{ HI3519DV500_FIXED_198M, "198m",    NULL, 0, 198000000, },
	{ HI3519DV500_FIXED_187P_5M, "187p5m",   NULL, 0, 187500000, },
	{ HI3519DV500_FIXED_150M, "150m",    NULL, 0, 150000000, },
	{ HI3519DV500_FIXED_148P_5M, "148p5m",  NULL, 0, 148500000, },
	{ HI3519DV500_FIXED_134M, "134m",    NULL, 0, 134000000, },
	{ HI3519DV500_FIXED_108M, "108m",    NULL, 0, 108000000, },
	{ HI3519DV500_FIXED_100M, "100m",    NULL, 0, 100000000, },
	{ HI3519DV500_FIXED_99M, "99m",     NULL, 0, 99000000, },
	{ HI3519DV500_FIXED_74P_25M, "74p25m",  NULL, 0, 74250000, },
	{ HI3519DV500_FIXED_72M, "72m",     NULL, 0, 72000000, },
	{ HI3519DV500_FIXED_64M, "64m",     NULL, 0, 64000000, },
	{ HI3519DV500_FIXED_60M, "60m",     NULL, 0, 60000000, },
	{ HI3519DV500_FIXED_54M, "54m",     NULL, 0, 54000000, },
	{ HI3519DV500_FIXED_50M, "50m",     NULL, 0, 50000000, },
	{ HI3519DV500_FIXED_49P_5M, "49p5m",   NULL, 0, 49500000, },
	{ HI3519DV500_FIXED_37P_125M, "37p125m", NULL, 0, 37125000, },
	{ HI3519DV500_FIXED_36M, "36m",     NULL, 0, 36000000, },
	{ HI3519DV500_FIXED_27M, "27m",     NULL, 0, 27000000, },
	{ HI3519DV500_FIXED_25M, "25m",     NULL, 0, 25000000, },
	{ HI3519DV500_FIXED_24M, "24m",     NULL, 0, 24000000, },
	{ HI3519DV500_FIXED_12M, "12m",     NULL, 0, 12000000, },
	{ HI3519DV500_FIXED_12P_288M, "12p288m", NULL, 0, 12288000, },
	{ HI3519DV500_FIXED_6M, "6m",      NULL, 0, 6000000, },
	{ HI3519DV500_FIXED_3M, "3m",      NULL, 0, 3000000, },
	{ HI3519DV500_FIXED_1P_6M, "1p6m",    NULL, 0, 1600000, },
	{ HI3519DV500_FIXED_400K, "400k",    NULL, 0, 400000, },
	{ HI3519DV500_FIXED_100K, "100k",    NULL, 0, 100000, },
};


static const char *fmc_mux_p[] __initdata = {
	"24m", "100m", "150m", "198m", "237m", "300m", "396m"
};
static u32 fmc_mux_table[] = {0, 1, 2, 3, 4, 5, 6};

static const char *mmc_mux_p[] __initdata = {
	"400k", "25m", "50m", "100m", "150m", "187p5m", "200m"
};
static u32 mmc_mux_table[] = {0, 1, 2, 3, 4, 5, 6, 7};

static const char *sdio0_mux_p[] __initdata = {
	"400k", "25m", "50m", "100m", "150m", "187p5m", "200m"
};
static u32 sdio0_mux_table[] = {0, 1, 2, 3, 4, 5, 6, 7};

static const char *sdio1_mux_p[] __initdata = {
	"400k", "25m", "50m", "100m", "150m", "187p5m", "200m"
};
static u32 sdio1_mux_table[] = {0, 1, 2, 3, 4, 5, 6, 7};

static const char *uart_mux_p[] __initdata = {"100m", "50m", "24m", "3m"};
static u32 uart_mux_table[] = {0, 1, 2, 3};

static const char *i2c_mux_p[] __initdata = {
	"50m", "100m"
};
static u32 i2c_mux_table[] = {0, 1};

static const char * pwm0_mux_p[] __initdata = {"198m"};
static u32 pwm0_mux_table[] = {0};

static const char * pwm1_mux_p[] __initdata = {"198m"};
static u32 pwm1_mux_table[] = {0};

static const char * pwm2_mux_p[] __initdata = {"198m"};
static u32 pwm2_mux_table[] = {0};

static struct bsp_mux_clock hi3519dv500_mux_clks_crg[] __initdata = {
	{
		HI3519DV500_FMC_MUX, "fmc_mux",
		fmc_mux_p, ARRAY_SIZE(fmc_mux_p),
		CLK_SET_RATE_PARENT, 0x3f40, 12, 3, 0, fmc_mux_table,
	},
	{
		HI3519DV500_MMC0_MUX, "mmc0_mux",
		mmc_mux_p, ARRAY_SIZE(mmc_mux_p),
		CLK_SET_RATE_PARENT, 0x34c0, 24, 3, 0, mmc_mux_table,
	},
	{
		HI3519DV500_MMC1_MUX, "mmc1_mux",
		sdio0_mux_p, ARRAY_SIZE(sdio0_mux_p),
		CLK_SET_RATE_PARENT, 0x35c0, 24, 3, 0, sdio0_mux_table,
	},
	{
		HI3519DV500_MMC2_MUX, "mmc2_mux",
		sdio1_mux_p, ARRAY_SIZE(sdio1_mux_p),
		CLK_SET_RATE_PARENT, 0x36c0, 24, 3, 0, sdio1_mux_table,
	},
	{
		HI3519DV500_UART0_MUX, "uart0_mux",
		uart_mux_p, ARRAY_SIZE(uart_mux_p),
		CLK_SET_RATE_PARENT, 0x4180, 12, 2, 0, uart_mux_table
	},
	{
		HI3519DV500_UART1_MUX, "uart1_mux",
		uart_mux_p, ARRAY_SIZE(uart_mux_p),
		CLK_SET_RATE_PARENT, 0x4188, 12, 2, 0, uart_mux_table
	},
	{
		HI3519DV500_UART2_MUX, "uart2_mux",
		uart_mux_p, ARRAY_SIZE(uart_mux_p),
		CLK_SET_RATE_PARENT, 0x4190, 12, 2, 0, uart_mux_table
	},
	{
		HI3519DV500_UART3_MUX, "uart3_mux",
		uart_mux_p, ARRAY_SIZE(uart_mux_p),
		CLK_SET_RATE_PARENT, 0x4198, 12, 2, 0, uart_mux_table
	},
	{
		HI3519DV500_UART4_MUX, "uart4_mux",
		uart_mux_p, ARRAY_SIZE(uart_mux_p),
		CLK_SET_RATE_PARENT, 0x41a0, 12, 2, 0, uart_mux_table
	},
	{
		HI3519DV500_UART5_MUX, "uart5_mux",
		uart_mux_p, ARRAY_SIZE(uart_mux_p),
		CLK_SET_RATE_PARENT, 0x41a8, 12, 2, 0, uart_mux_table
	},
	{
		HI3519DV500_I2C0_MUX, "i2c0_mux",
		i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x4280, 12, 1, 0, i2c_mux_table
	},
	{
		HI3519DV500_I2C1_MUX, "i2c1_mux",
		i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x4288, 12, 1, 0, i2c_mux_table
	},
	{
		HI3519DV500_I2C2_MUX, "i2c2_mux",
		i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x4290, 12, 1, 0, i2c_mux_table
	},
	{
		HI3519DV500_I2C3_MUX, "i2c3_mux",
		i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x4298, 12, 1, 0, i2c_mux_table
	},
	{
		HI3519DV500_I2C4_MUX, "i2c4_mux",
		i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x42a0, 12, 1, 0, i2c_mux_table
	},
	{
		HI3519DV500_I2C5_MUX, "i2c5_mux",
		i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x42a8, 12, 1, 0, i2c_mux_table
	},
	{
		HI3519DV500_I2C6_MUX, "i2c6_mux",
		i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x42b0, 12, 1, 0, i2c_mux_table
	},
	{
		HI3519DV500_I2C7_MUX, "i2c7_mux",
		i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x42b8, 12, 1, 0, i2c_mux_table
	},
	{
		HI3519DV500_PWM0_MUX, "pwm0_mux",
		pwm0_mux_p, ARRAY_SIZE(pwm0_mux_p),
		CLK_SET_RATE_PARENT, 0x4588, 12, 2, 0, pwm0_mux_table
	},
	{
		HI3519DV500_PWM1_MUX, "pwm1_mux",
		pwm1_mux_p, ARRAY_SIZE(pwm1_mux_p),
		CLK_SET_RATE_PARENT, 0x4590, 12, 2, 0, pwm1_mux_table
	},
	{
		HI3519DV500_PWM2_MUX, "pwm2_mux",
		pwm2_mux_p, ARRAY_SIZE(pwm2_mux_p),
		CLK_SET_RATE_PARENT, 0x4598, 12, 2, 0, pwm2_mux_table
	},
};

static struct bsp_fixed_factor_clock
	hi3519dv500_fixed_factor_clks[] __initdata = {
};

static struct bsp_gate_clock hi3519dv500_gate_clks[] __initdata = {
	{
		HI3519DV500_FMC_CLK, "clk_fmc", "fmc_mux",
		CLK_SET_RATE_PARENT, 0x3f40, 4, 0,
	},
	{
		HI3519DV500_MMC0_CLK, "clk_mmc0", "mmc0_mux",
		CLK_SET_RATE_PARENT, 0x34c0, 0, 0,
	},
	{
		HI3519DV500_MMC0_HCLK, "hclk_mmc0", NULL,
		CLK_SET_RATE_PARENT, 0x34c0, 1, 0,
	},
	{
		HI3519DV500_MMC1_CLK, "clk_mmc1", "mmc1_mux",
		CLK_SET_RATE_PARENT, 0x35c0, 0, 0,
	},
	{
		HI3519DV500_MMC1_HCLK, "hclk_mmc1", NULL,
		CLK_SET_RATE_PARENT, 0x35c0, 1, 0,
	},
	{
		HI3519DV500_MMC2_CLK, "clk_mmc2", "mmc2_mux",
		CLK_SET_RATE_PARENT, 0x36c0, 0, 0,
	},
	{
		HI3519DV500_MMC2_HCLK, "hclk_mmc2", NULL,
		CLK_SET_RATE_PARENT, 0x36c0, 1, 0,
	},
	{
		HI3519DV500_UART0_CLK, "clk_uart0", "uart0_mux",
		CLK_SET_RATE_PARENT, 0x4180, 4, 0,
	},
	{
		HI3519DV500_UART1_CLK, "clk_uart1", "uart1_mux",
		CLK_SET_RATE_PARENT, 0x4188, 4, 0,
	},
	{
		HI3519DV500_UART2_CLK, "clk_uart2", "uart2_mux",
		CLK_SET_RATE_PARENT, 0x4190, 4, 0,
	},
	{
		HI3519DV500_UART3_CLK, "clk_uart3", "uart3_mux",
		CLK_SET_RATE_PARENT, 0x4198, 4, 0,
	},
	{
		HI3519DV500_UART4_CLK, "clk_uart4", "uart4_mux",
		CLK_SET_RATE_PARENT, 0x41A0, 4, 0,
	},
	{
	    HI3519DV500_UART5_CLK, "clk_uart5", "uart5_mux",
		CLK_SET_RATE_PARENT, 0x41a8, 4, 0,
	},
	/* ethernet mac */
	{
		HI3519DV500_ETH_CLK, "clk_eth", NULL,
		CLK_SET_RATE_PARENT, 0x37c4, 4, 0,
	},
	{
		HI3519DV500_ETH_MACIF_CLK, "clk_eth_macif", NULL,
		CLK_SET_RATE_PARENT, 0x37c0, 4, 0,
	},
	{
		HI3519DV500_ETH1_CLK, "clk_eth1", NULL,
		CLK_SET_RATE_PARENT, 0x3804, 4, 0,
	},
	{
		HI3519DV500_ETH1_MACIF_CLK, "clk_eth1_macif", NULL,
		CLK_SET_RATE_PARENT, 0x3800, 4, 0,
	},
	{
		HI3519DV500_I2C0_CLK, "clk_i2c0", "i2c0_mux",
		CLK_SET_RATE_PARENT, 0x4280, 4, 0,
	},
	{
		HI3519DV500_I2C1_CLK, "clk_i2c1", "i2c1_mux",
		CLK_SET_RATE_PARENT, 0x4288, 4, 0,
	},
	{
		HI3519DV500_I2C2_CLK, "clk_i2c2", "i2c2_mux",
		CLK_SET_RATE_PARENT, 0x4290, 4, 0,
	},
	{
		HI3519DV500_I2C3_CLK, "clk_i2c3", "i2c3_mux",
		CLK_SET_RATE_PARENT, 0x4298, 4, 0,
	},
	{
		HI3519DV500_I2C4_CLK, "clk_i2c4", "i2c4_mux",
		CLK_SET_RATE_PARENT, 0x42a0, 4, 0,
	},
	{ 	HI3519DV500_I2C5_CLK, "clk_i2c5", "i2c5_mux",
		CLK_SET_RATE_PARENT, 0x42a8, 4, 0,
	},
	{ 	HI3519DV500_I2C6_CLK, "clk_i2c6", "i2c6_mux",
		CLK_SET_RATE_PARENT, 0x42b0, 4, 0,
	},
	{ 	HI3519DV500_I2C7_CLK, "clk_i2c7", "i2c7_mux",
		CLK_SET_RATE_PARENT, 0x42b8, 4, 0,
	},
	/* spi */
	{
		HI3519DV500_SPI0_CLK, "clk_spi0", "100m",
		CLK_SET_RATE_PARENT, 0x4480, 4, 0,
	},
	{
		HI3519DV500_SPI1_CLK, "clk_spi1", "100m",
		CLK_SET_RATE_PARENT, 0x4488, 4, 0,
	},
	{
		HI3519DV500_SPI2_CLK, "clk_spi2", "100m",
		CLK_SET_RATE_PARENT, 0x4490, 4, 0,
	},
	{
		HI3519DV500_SPI3_CLK, "clk_spi3", "100m",
		CLK_SET_RATE_PARENT, 0x4498, 4, 0,
	},
	{
		HI3519DV500_EDMAC_AXICLK, "axi_clk_edmac", NULL,
		CLK_SET_RATE_PARENT, 0x2a80, 5, 0,
	},
	{
		HI3519DV500_EDMAC_CLK, "clk_edmac", NULL,
		CLK_SET_RATE_PARENT, 0x2a80, 4, 0,
	},
	/* lsadc */
	{
		HI3519DV500_LSADC_CLK, "clk_lsadc", NULL,
		CLK_SET_RATE_PARENT, 0x46c0, 4, 0,
	},
	/* pwm0 */
	{
		HI3519DV500_PWM0_CLK, "clk_pwm0", "pwm0_mux",
		CLK_SET_RATE_PARENT, 0x4588, 4, 0,
	},
	/* pwm1 */
	{
		HI3519DV500_PWM1_CLK, "clk_pwm1", "pwm1_mux",
		CLK_SET_RATE_PARENT, 0x4590, 4, 0,
	},
	/* pwm2 */
	{
		HI3519DV500_PWM2_CLK, "clk_pwm2", "pwm2_mux",
		CLK_SET_RATE_PARENT, 0x4598, 4, 0,
	},
};

static __init struct bsp_clock_data *hi3519dv500_clk_register(
		struct platform_device *pdev)
{
	struct bsp_clock_data *clk_data = NULL;
	int ret;

	clk_data = bsp_clk_alloc(pdev, HI3519DV500_CRG_NR_CLKS);
	if (clk_data == NULL)
		return ERR_PTR(-ENOMEM);

	ret = bsp_clk_register_fixed_rate(hi3519dv500_fixed_rate_clks_crg,
			ARRAY_SIZE(hi3519dv500_fixed_rate_clks_crg), clk_data);
	if (ret)
		return ERR_PTR(ret);

	ret = bsp_clk_register_mux(hi3519dv500_mux_clks_crg,
			ARRAY_SIZE(hi3519dv500_mux_clks_crg),
			clk_data);
	if (ret)
		goto unregister_fixed_rate;

	ret = bsp_clk_register_fixed_factor(hi3519dv500_fixed_factor_clks,
			ARRAY_SIZE(hi3519dv500_fixed_factor_clks), clk_data);
	if (ret)
		goto unregister_mux;

	ret = bsp_clk_register_gate(hi3519dv500_gate_clks,
			ARRAY_SIZE(hi3519dv500_gate_clks),
			clk_data);
	if (ret)
		goto unregister_factor;

	ret = of_clk_add_provider(pdev->dev.of_node,
			of_clk_src_onecell_get, &clk_data->clk_data);
	if (ret)
		goto unregister_gate;

	return clk_data;

unregister_gate:
	bsp_clk_unregister_gate(hi3519dv500_gate_clks,
			ARRAY_SIZE(hi3519dv500_gate_clks), clk_data);
unregister_factor:
	bsp_clk_unregister_fixed_factor(hi3519dv500_fixed_factor_clks,
			ARRAY_SIZE(hi3519dv500_fixed_factor_clks), clk_data);
unregister_mux:
	bsp_clk_unregister_mux(hi3519dv500_mux_clks_crg,
			ARRAY_SIZE(hi3519dv500_mux_clks_crg),
			clk_data);
unregister_fixed_rate:
	bsp_clk_unregister_fixed_rate(hi3519dv500_fixed_rate_clks_crg,
			ARRAY_SIZE(hi3519dv500_fixed_rate_clks_crg), clk_data);
	return ERR_PTR(ret);
}

static __init void hi3519dv500_clk_unregister(const struct platform_device *pdev)
{
	struct bsp_crg_dev *crg = platform_get_drvdata(pdev);

	of_clk_del_provider(pdev->dev.of_node);

	bsp_clk_unregister_gate(hi3519dv500_gate_clks,
			ARRAY_SIZE(hi3519dv500_gate_clks), crg->clk_data);
	bsp_clk_unregister_mux(hi3519dv500_mux_clks_crg,
			ARRAY_SIZE(hi3519dv500_mux_clks_crg), crg->clk_data);
	bsp_clk_unregister_fixed_factor(hi3519dv500_fixed_factor_clks,
			ARRAY_SIZE(hi3519dv500_fixed_factor_clks), crg->clk_data);
	bsp_clk_unregister_fixed_rate(hi3519dv500_fixed_rate_clks_crg,
			ARRAY_SIZE(hi3519dv500_fixed_rate_clks_crg), crg->clk_data);
}

static const struct bsp_crg_funcs hi3519dv500_crg_funcs = {
	.register_clks = hi3519dv500_clk_register,
	.unregister_clks = hi3519dv500_clk_unregister,
};


static const struct of_device_id hi3519dv500_crg_match_table[] = {
	{
		.compatible = "vendor,hi3519dv500_clock",
		.data = &hi3519dv500_crg_funcs
	},
	{ }
};
MODULE_DEVICE_TABLE(of, hi3519dv500_crg_match_table);

static int hi3519dv500_crg_probe(struct platform_device *pdev)
{
	struct bsp_crg_dev *crg = NULL;
	int ret = -1;

	crg = devm_kmalloc(&pdev->dev, sizeof(*crg), GFP_KERNEL);
	if (crg == NULL)
		return -ENOMEM;

	crg->funcs = of_device_get_match_data(&pdev->dev);
	if (crg->funcs == NULL) {
		ret = -ENOENT;
		goto crg_free;
	}

	crg->rstc = vendor_reset_init(pdev);
	if (crg->rstc == NULL) {
		ret = -ENOENT;
		goto crg_free;
	}

	crg->clk_data = crg->funcs->register_clks(pdev);
	if (IS_ERR(crg->clk_data)) {
		bsp_reset_exit(crg->rstc);
		ret = PTR_ERR(crg->clk_data);
		goto crg_free;
	}

	platform_set_drvdata(pdev, crg);
	return 0;

crg_free:
	if (crg != NULL) {
		devm_kfree(&pdev->dev, crg);
		crg = NULL;
	}
	return ret;
}

static int hi3519dv500_crg_remove(struct platform_device *pdev)
{
	struct bsp_crg_dev *crg = platform_get_drvdata(pdev);

	bsp_reset_exit(crg->rstc);
	crg->funcs->unregister_clks(pdev);
	return 0;
}

static struct platform_driver hi3519dv500_crg_driver = {
	.probe          = hi3519dv500_crg_probe,
	.remove     = hi3519dv500_crg_remove,
	.driver         = {
		.name   = "hi3519dv500_clock",
		.of_match_table = hi3519dv500_crg_match_table,
	},
};

static int __init hi3519dv500_crg_init(void)
{
	return platform_driver_register(&hi3519dv500_crg_driver);
}
core_initcall(hi3519dv500_crg_init);

static void __exit hi3519dv500_crg_exit(void)
{
	platform_driver_unregister(&hi3519dv500_crg_driver);
}
module_exit(hi3519dv500_crg_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("HiSilicon HI3519DV500 CRG Driver");
