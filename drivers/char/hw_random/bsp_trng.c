/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/securec.h>

#define CA_MISC_REG_BASE_ADDR          0x101e8000
#define CA_MISC_REG_SIZE               0x1000
#define CPU_ID_STAT                    0x0018
#define CRYPTO_CPU_ID_SCPU             0xa5
#define CRYPTO_CPU_ID_ACPU             0xaa
#define TRNG_TIMEOUT_IN_US             1000000
/*! \Define the offset of TRNG reg */
#define HISC_COM_TRNG_FIFO_DATA        0x100
#define HISC_COM_TRNG_FIFO_READY       0x104
#define HISC_COM_TRNG_DATA_ST          0x108

#define TRNG_DONE                      1
#define TRNG_DATA_READY                1

#define HWRNG_READ_BYTE                4

void *g_trng_base_addr;
void *g_ca_misc_base_addr;
resource_size_t g_trng_reg_addr;
phys_addr_t g_trng_reg_size;
struct hwrng *g_rng;

/* Define the union hisc_com_trng_fifo_ready */
typedef union {
	/* Define the struct bits */
	struct {
		u32	trng_data_ready  :  1;  /* [0]  */
		u32	trng_done        :  1;  /* [1]  */
		u32	reserved_0       :  30; /* [31..2]  */
	} bits;

	/* Define an unsigned member */
	u32	all;
} hisc_com_trng_fifo_ready;

typedef enum {
	CRYPTO_CPU_TYPE_SCPU,
	CRYPTO_CPU_TYPE_ACPU,
	CRYPTO_CPU_TYPE_INVALID
} crypto_cpu_type;

static u32 crypto_reg_read(volatile void *reg_addr)
{
	return ioread32(reg_addr);
}

static u32 trng_reg_read(u32 offset)
{
	return crypto_reg_read((volatile void *)(g_trng_base_addr + offset));
}

static u32 ca_misc_reg_read(u32 offset)
{
	return crypto_reg_read((volatile void *)(g_ca_misc_base_addr + offset));
}

static crypto_cpu_type get_cpu_type(void)
{
	u32 cpu_id = ca_misc_reg_read(CPU_ID_STAT) & 0x00ff;
	if (cpu_id == CRYPTO_CPU_ID_SCPU) {
		return CRYPTO_CPU_TYPE_SCPU;
	} else if (cpu_id == CRYPTO_CPU_ID_ACPU) {
		return CRYPTO_CPU_TYPE_ACPU;
	}
	return CRYPTO_CPU_TYPE_INVALID;
}

static bool is_trng_ready(void)
{
	hisc_com_trng_fifo_ready trng_ready;

	if (get_cpu_type() != CRYPTO_CPU_TYPE_SCPU) { /* Not check status on non-secure Core */
		return true;
	}

	trng_ready.all = trng_reg_read(HISC_COM_TRNG_FIFO_READY);

	if ((trng_ready.bits).trng_done != TRNG_DONE ||
		(trng_ready.bits).trng_data_ready != TRNG_DATA_READY) {
			return false;
	}
	return true;
}

static int read_from_trng(void *buf, size_t max)
{
	u32 data;
	u32 chk_randnum;
	data = trng_reg_read(HISC_COM_TRNG_FIFO_DATA);
	chk_randnum = trng_reg_read(HISC_COM_TRNG_FIFO_DATA);
	if ((data != 0x00000000) && (data != 0xffffffff) && (data != chk_randnum)) {
		if (max >= HWRNG_READ_BYTE) {
			memcpy_s(buf, max, &data, HWRNG_READ_BYTE);
			return HWRNG_READ_BYTE;
		} else {
			memcpy_s(buf, max, &data, max);
			return max;
		}
	}
	return 0;
}

static int trng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	u32 times = 0;

	while (times < TRNG_TIMEOUT_IN_US) {
		times++;
		if (is_trng_ready() == false) {
			continue;
		}

		if (is_trng_ready() == true) {
			return read_from_trng(buf, max);
		}
	}

	return 0;
}

static int trng_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		g_trng_reg_addr = res->start;
		g_trng_reg_size = res->end - res->start + 1;
	} else {
		return -ENODEV;
	}

	g_rng = devm_kzalloc(&pdev->dev, sizeof(*g_rng), GFP_KERNEL);
	if (!g_rng)
		return -ENOMEM;

	platform_set_drvdata(pdev, g_rng);

	g_rng->name = pdev->name;
	g_rng->read = trng_read;

	g_trng_base_addr = ioremap(g_trng_reg_addr, g_trng_reg_size);
	if (g_trng_base_addr == NULL) {
		devm_kfree(&pdev->dev, g_rng);
		return -EINVAL;
	}
	g_ca_misc_base_addr = ioremap(CA_MISC_REG_BASE_ADDR, CA_MISC_REG_SIZE);
	if (g_ca_misc_base_addr == NULL) {
		devm_kfree(&pdev->dev, g_rng);
		iounmap(g_trng_base_addr);
		return -EINVAL;
	}

	ret = devm_hwrng_register(&pdev->dev, g_rng);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register hwrng\n");
		devm_kfree(&pdev->dev, g_rng);
		iounmap(g_ca_misc_base_addr);
		iounmap(g_trng_base_addr);
		return ret;
	}

	printk("Succeed to load trng.\n");
	return 0;
}

static int trng_remove(struct platform_device *pdev)
{
	devm_hwrng_unregister(&pdev->dev, g_rng);
	iounmap(g_ca_misc_base_addr);
	iounmap(g_trng_base_addr);
	devm_kfree(&pdev->dev, g_rng);
	printk("Succeed to remove trng.\n");
	return 0;
}

static const struct of_device_id trng_match[] = {
	{ .compatible = "vendor,trng" },
	{ },
};
MODULE_DEVICE_TABLE(of, trng_match);

static struct platform_driver trng_driver = {
	.probe		= trng_probe,
	.remove		= trng_remove,
	.driver		= {
		.name	= "vendor,trng",
		.of_match_table = trng_match,
	},
};

module_platform_driver(trng_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Hardware random number generator driver");
