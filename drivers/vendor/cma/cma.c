/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#include <linux/bsp_cma.h>
#include <linux/securec.h>

static u32 num_zones;
static struct cma_zone bsp_zone[ZONE_MAX];
static int use_bootargs;

unsigned int get_cma_size(void)
{
	u32 i;
	u64 total = 0;

	for (i = 0; i < num_zones; i++)
		total += bsp_zone[i].nbytes;

	return (unsigned int)(total >> 20); /* unit M: shift right 20 bits */
}

int is_cma_address(phys_addr_t phys, unsigned long size)
{
	phys_addr_t start, end;
	u32 i;

	for (i = 0; i < num_zones; i++) {
		start = bsp_zone[i].phys_start;
		end = bsp_zone[i].phys_start + bsp_zone[i].nbytes;

		if ((phys >= start) && ((phys + size) <= end))
			return 1; /* Yes, found! */
	}

	return 0;
}
EXPORT_SYMBOL(is_cma_address);

static int __init bsp_mmz_parse_cmdline(char *s)
{
	char *line = NULL, *tmp = NULL;
	char tmpline[256];
	int ret;

	if (s == NULL) {
		pr_info("There is no cma zone!\n");
		return 0;
	}
	ret = strncpy_s(tmpline, sizeof(tmpline), s, sizeof(tmpline) - 1);
	if (ret) {
		printk("%s:strncpy_s failed\n", __func__);
		return ret;
	}

	tmpline[sizeof(tmpline) - 1] = '\0';
	tmp = tmpline;

	while ((line = strsep(&tmp, ":")) != NULL) {
		int i;
		char *argv[6];

		for (i = 0; (argv[i] = strsep(&line, ",")) != NULL;)
			if (++i == ARRAY_SIZE(argv))
				break;

		if (num_zones >= ZONE_MAX)
			return 0;
		bsp_zone[num_zones].pdev.coherent_dma_mask = DMA_BIT_MASK(64);
		if (i == 4) {
			strlcpy(bsp_zone[num_zones].name, argv[0], NAME_LEN_MAX);
			bsp_zone[num_zones].gfp = (uintptr_t)memparse(argv[1], NULL);
			bsp_zone[num_zones].phys_start = (uintptr_t)memparse(argv[2], NULL);
			bsp_zone[num_zones].nbytes = (uintptr_t)memparse(argv[3], NULL);
		}

		else if (i == 6) {
			strlcpy(bsp_zone[num_zones].name, argv[0], NAME_LEN_MAX);
			bsp_zone[num_zones].gfp = (uintptr_t)memparse(argv[1], NULL);
			bsp_zone[num_zones].phys_start = (uintptr_t)memparse(argv[2], NULL);
			bsp_zone[num_zones].nbytes = (uintptr_t)memparse(argv[3], NULL);
			bsp_zone[num_zones].alloc_type = (uintptr_t)memparse(argv[4], NULL);
			bsp_zone[num_zones].block_align = (uintptr_t)memparse(argv[5], NULL);
		} else {
			pr_err("ion parameter is not correct\n");
			continue;
		}

		num_zones++;
	}
	if (num_zones != 0)
		use_bootargs = 1;

	return 0;
}
early_param("mmz", bsp_mmz_parse_cmdline);

phys_addr_t get_zones_start(void)
{
	u32 i;
	phys_addr_t lowest_zone_base = memblock_end_of_DRAM();

	for (i = 0; i < num_zones; i++) {
		if (lowest_zone_base > bsp_zone[i].phys_start)
			lowest_zone_base = bsp_zone[i].phys_start;
	}

	return lowest_zone_base;
}
EXPORT_SYMBOL(get_zones_start);

struct cma_zone *get_cma_zone(const char *name)
{
	u32 i;

	if (name == NULL)
		return NULL;
	for (i = 0; i < num_zones; i++)
		if (strcmp(bsp_zone[i].name, name) == 0)
			break;

	if (i == num_zones)
		return NULL;

	return &bsp_zone[i];
}
EXPORT_SYMBOL(get_cma_zone);

struct device *get_cma_device(const char *name)
{
	u32 i;

	if (name == NULL)
		return NULL;

	for (i = 0; i < num_zones; i++)
		if (strcmp(bsp_zone[i].name, name) == 0)
			break;

	if (i == num_zones)
		return NULL;

	return &bsp_zone[i].pdev;
}
EXPORT_SYMBOL(get_cma_device);

int __init declare_heap_memory(void)
{
	u32 i;
	int ret = 0;

	if (use_bootargs == 0) {
		pr_info("cma zone is not set!\n");
		return ret;
	}

	for (i = 0; i < num_zones; i++) {
		ret = dma_contiguous_reserve_area(bsp_zone[i].nbytes,
						  bsp_zone[i].phys_start, 0, &bsp_zone[i].pdev.cma_area, true);
		if (ret)
			panic("declare cma zone %s base: 0x%lx size:%lu MB failed. ret:%d",
			      bsp_zone[i].name, (unsigned long)bsp_zone[i].phys_start,
			      (unsigned long)bsp_zone[i].nbytes >> 20, ret);

		bsp_zone[i].phys_start = cma_get_base(bsp_zone[i].pdev.cma_area);
		bsp_zone[i].nbytes = cma_get_size(bsp_zone[i].pdev.cma_area);
	}

	return ret;
}

static int bsp_mmz_setup(struct reserved_mem *rmem)
{
	return 0;
}
RESERVEDMEM_OF_DECLARE(cma, "bsp-mmz", bsp_mmz_setup);

