/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/reset.h>

#ifdef CONFIG_ARCH_HI3519DV500_FAMILY

/* reg addr of the xth chn. */
#define pwm_period_cfg_addr(x)     (0x0000 + (0x100 * (x)))
#define pwm_duty0_cfg_addr(x)      (0x0004 + (0x100 * (x)))
#define pwm_duty1_cfg_addr(x)      (0x0008 + (0x100 * (x)))
#define pwm_duty2_cfg_addr(x)      (0x000C + (0x100 * (x)))
#define pwm_num_cfg_addr(x)        (0x0010 + (0x100 * (x)))
#define pwm_ctrl_addr(x)           (0x0014 + (0x100 * (x)))
#define pwm_dt_value_cfg_addr(x)   (0x0020 + (0x100 * (x)))
#define pwm_dt_ctrl_cfg_addr(x)    (0x0024 + (0x100 * (x)))
#define pwm_sync_cfg_addr(x)       (0x0030 + (0x100 * (x)))
#define pwm_sync_delay_cfg_addr(x) (0x0034 + (0x100 * (x)))
#define pwm_period_addr(x)         (0x0040 + (0x100 * (x)))
#define pwm_duty0_addr(x)          (0x0044 + (0x100 * (x)))
#define pwm_duty1_addr(x)          (0x0048 + (0x100 * (x)))
#define pwm_duty2_addr(x)          (0x004C + (0x100 * (x)))
#define pwm_num_addr(x)            (0x0050 + (0x100 * (x)))
#define pwm_ctrl_st_addr(x)        (0x0054 + (0x100 * (x)))
#define pwm_dt_value_addr(x)       (0x0060 + (0x100 * (x)))
#define pwm_dt_ctrl_addr(x)        (0x0064 + (0x100 * (x)))
#define pwm_sync_delay_addr(x)     (0x0074 + (0x100 * (x)))

#define PWM_SYNC_START_ADDR        0x0ff0

#define PWM_ALIGN_MODE_SHIFT      4
#define PWM_ALIGN_MODE_MASK       GENMASK(5, 4)

#define PWM_PRE_DIV_SEL_SHIFT     8
#define PWM_PRE_DIV_SEL_MASK      GENMASK(11, 8)

/* pwm dt value */
#define PWM_DT_A_SHIFT      0
#define PWM_DT_A_MASK       GENMASK(31, 16)

#define PWM_DT_B_SHIFT      16
#define PWM_DT_B_MASK       GENMASK(15, 0)

/* pwm dt ctrl */
#define PWM_DTS_OUT_0P_SHIFT      0
#define PWM_DTS_OUT_0P_MASK       BIT(0)

#define PWM_DTS_OUT_0N_SHIFT      1
#define PWM_DTS_OUT_0N_MASK       BIT(1)

#define PWM_DTS_OUT_1P_SHIFT      2
#define PWM_DTS_OUT_1P_MASK       BIT(2)

#define PWM_DTS_OUT_1N_SHIFT      3
#define PWM_DTS_OUT_1N_MASK       BIT(3)

#define PWM_DTS_OUT_2P_SHIFT      4
#define PWM_DTS_OUT_2P_MASK       BIT(4)

#define PWM_DTS_OUT_2N_SHIFT      5
#define PWM_DTS_OUT_2N_MASK       BIT(5)

#else

#define pwm_period_cfg_addr(x)    (((x) * 0x20) + 0x0)
#define pwm_duty0_cfg_addr(x)    (((x) * 0x20) + 0x4)
#define pwm_cfg2_addr(x)    (((x) * 0x20) + 0x8)
#define pwm_ctrl_addr(x)    (((x) * 0x20) + 0xC)

#endif

/* pwm ctrl */
#define PWM_ENABLE_SHIFT    0
#define PWM_ENABLE_MASK     BIT(0)

#define PWM_POLARITY_SHIFT  1
#define PWM_POLARITY_MASK   BIT(1)

#define PWM_KEEP_SHIFT      2
#define PWM_KEEP_MASK       BIT(2)

/* pwm period */
#define PWM_PERIOD_MASK     GENMASK(31, 0)

/* pwm duty */
#define PWM_DUTY_MASK       GENMASK(31, 0)

#define PWM_PERIOD_HZ      1000

enum pwm_pre_div {
	PWM_PRE_DIV_1 = 0,
	PWM_PRE_DIV_2,
	PWM_PRE_DIV_4,
	PWM_PRE_DIV_8,
	PWM_PRE_DIV_16,
	PWM_PRE_DIV_32,
	PWM_PRE_DIV_64,
	PWM_PRE_DIV_128,
	PWM_PRE_DIV_256,
};

enum pwm_align {
	PWM_ALIGN_RIGHT = 0,
	PWM_ALIGN_LEFT,
	PWM_ALIGN_MIDDLE,
};

typedef enum {
	PWM_CONTROLLER_0 = 0,
	PWM_CONTROLLER_1,
	PWM_CONTROLLER_2,
} pwm_controller_index;

typedef enum {
	PWM_CHN_0 = 0,
	PWM_CHN_1,
	PWM_CHN_2,
	PWM_CHN_3,
	PWM_CHN_4,
	PWM_CHN_5,
} pwm_chn_index;

struct bsp_pwm_chip {
	pwm_controller_index controller_index;
	struct pwm_chip	chip;
	struct clk *clk;
	void __iomem *base;
	struct reset_control *rstc;
};

struct bsp_pwm_soc {
	u32 num_pwms;
	const char *pwm_name;
};

#ifdef CONFIG_ARCH_HI3519DV500_FAMILY
#define CHIP_PWM_NUM 3
#define CHIP_PWM_CONTROLLER_0_NAME "pwm0"
#define CHIP_PWM_CONTROLLER_1_NAME "pwm1"
#define CHIP_PWM_CONTROLLER_2_NAME "pwm2"

static const struct bsp_pwm_soc pwm_soc[CHIP_PWM_NUM] = {
	{ .num_pwms = 6, .pwm_name = CHIP_PWM_CONTROLLER_0_NAME },
	{ .num_pwms = 6, .pwm_name = CHIP_PWM_CONTROLLER_1_NAME },
	{ .num_pwms = 5, .pwm_name = CHIP_PWM_CONTROLLER_2_NAME },
};
#endif

static inline struct bsp_pwm_chip *to_bsp_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct bsp_pwm_chip, chip);
}

static void bsp_pwm_set_bits(void __iomem *base, u32 offset,
					u32 mask, u32 data)
{
	void __iomem *address = base + offset;
	u32 value;

	value = readl(address);
	value &= ~mask;
	value |= (data & mask);
	writel(value, address);
}

static void bsp_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct bsp_pwm_chip *bsp_pwm_chip = to_bsp_pwm_chip(chip);

	bsp_pwm_set_bits(bsp_pwm_chip->base, pwm_ctrl_addr(pwm->hwpwm),
			PWM_ENABLE_MASK, 0x1);
}

static void bsp_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct bsp_pwm_chip *bsp_pwm_chip = to_bsp_pwm_chip(chip);

	bsp_pwm_set_bits(bsp_pwm_chip->base, pwm_ctrl_addr(pwm->hwpwm),
			PWM_ENABLE_MASK, 0x0);
}

static bool bsp_pwm_is_complementary_chn(pwm_controller_index controller_index, pwm_chn_index chn_index)
{
#ifdef CONFIG_ARCH_HI3519DV500_FAMILY
	if (((controller_index == PWM_CONTROLLER_0) &&
		(chn_index == PWM_CHN_0 || chn_index == PWM_CHN_3 ||
		chn_index == PWM_CHN_4 || chn_index == PWM_CHN_5)) ||
		((controller_index == PWM_CONTROLLER_1) &&
		(chn_index == PWM_CHN_0 || chn_index == PWM_CHN_4 || chn_index == PWM_CHN_5))) {
		return 1;
	}
#endif
	return 0;
}

static void bsp_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
					const struct pwm_state *state)
{
	struct bsp_pwm_chip *bsp_pwm_chip = to_bsp_pwm_chip(chip);
	u64 freq, period, duty, duty1, duty2;

	freq = div_u64(clk_get_rate(bsp_pwm_chip->clk), 1000000);

	period = div_u64(freq * state->period, PWM_PERIOD_HZ);
	duty = div_u64(period * state->duty_cycle, state->period);
	duty1 = div_u64(period * state->duty_cycle1, state->period);
	duty2 = div_u64(period * state->duty_cycle2, state->period);

#ifdef CONFIG_ARCH_HI3519DV500_FAMILY
	bsp_pwm_set_bits(bsp_pwm_chip->base, pwm_ctrl_addr(pwm->hwpwm),
		PWM_PRE_DIV_SEL_MASK, (PWM_PRE_DIV_1 << PWM_PRE_DIV_SEL_SHIFT));
#endif

	bsp_pwm_set_bits(bsp_pwm_chip->base, pwm_period_cfg_addr(pwm->hwpwm),
			PWM_PERIOD_MASK, period);

	bsp_pwm_set_bits(bsp_pwm_chip->base, pwm_duty0_cfg_addr(pwm->hwpwm),
			PWM_DUTY_MASK, duty);

	if (bsp_pwm_is_complementary_chn(bsp_pwm_chip->controller_index, pwm->hwpwm) == 1) {
		bsp_pwm_set_bits(bsp_pwm_chip->base, pwm_duty1_cfg_addr(pwm->hwpwm),
				PWM_DUTY_MASK, duty1);

		bsp_pwm_set_bits(bsp_pwm_chip->base, pwm_duty2_cfg_addr(pwm->hwpwm),
				PWM_DUTY_MASK, duty2);
	}
}

static void bsp_pwm_set_polarity(struct pwm_chip *chip,
					struct pwm_device *pwm,
					enum pwm_polarity polarity)
{
	struct bsp_pwm_chip *bsp_pwm_chip = to_bsp_pwm_chip(chip);

	if (polarity == PWM_POLARITY_INVERSED)
		bsp_pwm_set_bits(bsp_pwm_chip->base, pwm_ctrl_addr(pwm->hwpwm),
				PWM_POLARITY_MASK, (0x1 << PWM_POLARITY_SHIFT));
	else
		bsp_pwm_set_bits(bsp_pwm_chip->base, pwm_ctrl_addr(pwm->hwpwm),
				PWM_POLARITY_MASK, (0x0 << PWM_POLARITY_SHIFT));
}

static void bsp_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct bsp_pwm_chip *bsp_pwm_chip = to_bsp_pwm_chip(chip);
	void __iomem *base;
	u32 freq, value;

	freq = div_u64(clk_get_rate(bsp_pwm_chip->clk), 1000000);
	base = bsp_pwm_chip->base;
	value = readl(base + pwm_period_cfg_addr(pwm->hwpwm));
	state->period = div_u64(value * PWM_PERIOD_HZ, freq);

	value = readl(base + pwm_duty0_cfg_addr(pwm->hwpwm));
	state->duty_cycle = div_u64(value * PWM_PERIOD_HZ, freq);

	if (bsp_pwm_is_complementary_chn(bsp_pwm_chip->controller_index, pwm->hwpwm) == 1) {
		value = readl(base + pwm_duty1_cfg_addr(pwm->hwpwm));
		state->duty_cycle1 = div_u64(value * PWM_PERIOD_HZ, freq);

		value = readl(base + pwm_duty2_cfg_addr(pwm->hwpwm));
		state->duty_cycle2 = div_u64(value * PWM_PERIOD_HZ, freq);
	}

	value = readl(base + pwm_ctrl_addr(pwm->hwpwm));
	state->enabled = (PWM_ENABLE_MASK & value);
}

static int bsp_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
				const struct pwm_state *state)
{
	if (state->polarity != pwm->state.polarity)
		bsp_pwm_set_polarity(chip, pwm, state->polarity);

	if (state->period != pwm->state.period ||
		state->duty_cycle != pwm->state.duty_cycle ||
		state->duty_cycle1 != pwm->state.duty_cycle1 ||
		state->duty_cycle2 != pwm->state.duty_cycle2)
		bsp_pwm_config(chip, pwm, state);

	if (state->enabled != pwm->state.enabled) {
		if (state->enabled)
			bsp_pwm_enable(chip, pwm);
		else
			bsp_pwm_disable(chip, pwm);
	}

	return 0;
}

static const struct pwm_ops bsp_pwm_ops = {
	.get_state = bsp_pwm_get_state,
	.apply = bsp_pwm_apply,

	.owner = THIS_MODULE,
};

static void bsp_pwm_probe_set_chip_ops(struct platform_device *pdev, struct bsp_pwm_chip *pwm_chip, int chip_loop)
{
	pwm_chip->chip.ops = &bsp_pwm_ops;
	pwm_chip->chip.dev = &pdev->dev;
	pwm_chip->chip.base = -1;
	pwm_chip->chip.npwm = pwm_soc[chip_loop].num_pwms;
	pwm_chip->chip.of_xlate = of_pwm_xlate_with_flags;
	pwm_chip->chip.of_pwm_n_cells = 3;
}

static int bsp_pwm_probe(struct platform_device *pdev)
{
	struct bsp_pwm_chip *pwm_chip, *pwm_chip_tmp;
	struct resource *res;
	int ret;
	int i;
	int chip_loop;
	const char *pwm_name = NULL;

	pwm_chip_tmp = devm_kzalloc(&pdev->dev, sizeof(*pwm_chip_tmp) * CHIP_PWM_NUM, GFP_KERNEL);
	if (pwm_chip_tmp == NULL)
		return -ENOMEM;

	for (chip_loop = 0; chip_loop < CHIP_PWM_NUM; chip_loop++) {
		pwm_chip = pwm_chip_tmp + chip_loop;
		pwm_name = pwm_soc[chip_loop].pwm_name;
		pwm_chip->controller_index = chip_loop;
		pwm_chip->clk = devm_clk_get(&pdev->dev, pwm_name);
		if (IS_ERR(pwm_chip->clk)) {
			dev_err(&pdev->dev, "getting clock failed with %ld\n",
					PTR_ERR(pwm_chip->clk));
			return PTR_ERR(pwm_chip->clk);
		}

		bsp_pwm_probe_set_chip_ops(pdev, pwm_chip, chip_loop);

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, pwm_name);
		pwm_chip->base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(pwm_chip->base))
			return PTR_ERR(pwm_chip->base);

		ret = clk_prepare_enable(pwm_chip->clk);
		if (ret < 0)
			return ret;

		pwm_chip->rstc = devm_reset_control_get_exclusive(&pdev->dev, pwm_name);
		if (IS_ERR(pwm_chip->rstc)) {
			clk_disable_unprepare(pwm_chip->clk);
			return PTR_ERR(pwm_chip->rstc);
		}

		reset_control_assert(pwm_chip->rstc);
		msleep(30);
		reset_control_deassert(pwm_chip->rstc);

		ret = pwmchip_add(&pwm_chip->chip);
		if (ret < 0) {
			clk_disable_unprepare(pwm_chip->clk);
			return ret;
		}

		for (i = 0; i < pwm_chip->chip.npwm; i++) {
			bsp_pwm_set_bits(pwm_chip->base, pwm_ctrl_addr(i), PWM_KEEP_MASK, (0x1 << PWM_KEEP_SHIFT));
		}
	}

	platform_set_drvdata(pdev, pwm_chip_tmp);

	return 0;
}

static int bsp_pwm_remove(struct platform_device *pdev)
{
	int ret = 0;
	int chip_loop;
	struct bsp_pwm_chip *pwm_chip;
	struct bsp_pwm_chip *pwm_chip_tmp;

	pwm_chip_tmp = platform_get_drvdata(pdev);

	for (chip_loop = 0; chip_loop < CHIP_PWM_NUM; chip_loop++) {
		pwm_chip = pwm_chip_tmp + chip_loop;
		reset_control_assert(pwm_chip->rstc);
		msleep(30);
		reset_control_deassert(pwm_chip->rstc);

		clk_disable_unprepare(pwm_chip->clk);

		ret |= pwmchip_remove(&pwm_chip->chip);
	}

	return ret;
}

static const struct of_device_id bsp_pwm_of_match[] = {
	{ .compatible = "vendor,pwm", .data = &pwm_soc[0] },
	{  }
};
MODULE_DEVICE_TABLE(of, bsp_pwm_of_match);

static struct platform_driver bsp_pwm_driver = {
	.driver = {
		.name = "bsp-pwm",
		.of_match_table = bsp_pwm_of_match,
	},
	.probe = bsp_pwm_probe,
	.remove	= bsp_pwm_remove,
};
module_platform_driver(bsp_pwm_driver);

MODULE_LICENSE("GPL");
