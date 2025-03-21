/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/mfd/bsp_fmc.h>
#include "match_table.h"
#include "nfc_gen.h"

struct nand_flash_dev *(*get_spi_nand_flash_type_hook)(struct mtd_info *mtd,
		unsigned char *id) = NULL;
EXPORT_SYMBOL_GPL(get_spi_nand_flash_type_hook);

static struct match_t match_ecc[] = {
	MATCH_SET_TYPE_DATA(NAND_ECC_NONE_0, "none"),
	MATCH_SET_TYPE_DATA(NAND_ECC_0BIT, "none"),
	MATCH_SET_TYPE_DATA(NAND_ECC_1BIT_512, "1bit/512"),
	MATCH_SET_TYPE_DATA(NAND_ECC_4BIT, "4bit/512"),
	MATCH_SET_TYPE_DATA(NAND_ECC_4BIT_512, "4bit/512"),
	MATCH_SET_TYPE_DATA(NAND_ECC_4BYTE, "4byte/1k"),
	MATCH_SET_TYPE_DATA(NAND_ECC_8BIT, "4bit/512"),
	MATCH_SET_TYPE_DATA(NAND_ECC_8BIT_512, "8bit/512"),
	MATCH_SET_TYPE_DATA(NAND_ECC_8BYTE, "8byte/1k"),
	MATCH_SET_TYPE_DATA(NAND_ECC_13BIT, "13bit/1k"),
	MATCH_SET_TYPE_DATA(NAND_ECC_16BIT, "8bit/512"),
	MATCH_SET_TYPE_DATA(NAND_ECC_18BIT, "18bit/1k"),
	MATCH_SET_TYPE_DATA(NAND_ECC_24BIT, "24bit/1k"),
	MATCH_SET_TYPE_DATA(NAND_ECC_27BIT, "27bit/1k"),
	MATCH_SET_TYPE_DATA(NAND_ECC_32BIT, "32bit/1k"),
	MATCH_SET_TYPE_DATA(NAND_ECC_40BIT, "40bit/1k"),
	MATCH_SET_TYPE_DATA(NAND_ECC_41BIT, "41bit/1k"),
	MATCH_SET_TYPE_DATA(NAND_ECC_48BIT, "48bit/1k"),
	MATCH_SET_TYPE_DATA(NAND_ECC_60BIT, "60bit/1k"),
	MATCH_SET_TYPE_DATA(NAND_ECC_72BIT, "72bit/1k"),
	MATCH_SET_TYPE_DATA(NAND_ECC_80BIT, "80bit/1k"),
};

const char *nand_ecc_name(int type)
{
	return (char *)match_type_to_data(match_ecc, ARRAY_SIZE(match_ecc),
					  type, "unknown");
}
EXPORT_SYMBOL_GPL(nand_ecc_name);

char *get_ecctype_str(enum ecc_type ecctype)
{
	static char *ecctype_string[] = {
		"None", "1bit/512Byte", "4bits/512Byte", "8bits/512Byte",
		"24bits/1K", "40bits/1K", "unknown", "unknown"
	};
	return ecctype_string[(ecctype & 0x07)];
}

static struct match_type_str page2name[] = {
	{ NAND_PAGE_512B, "512" },
	{ NAND_PAGE_2K,   "2K" },
	{ NAND_PAGE_4K,   "4K" },
	{ NAND_PAGE_8K,   "8K" },
	{ NAND_PAGE_16K,  "16K" },
	{ NAND_PAGE_32K,  "32K" },
};

const char *nand_page_name(int type)
{
	return type2str(page2name, ARRAY_SIZE(page2name), type, "unknown");
}
EXPORT_SYMBOL_GPL(nand_page_name);

char *get_pagesize_str(enum page_type pagetype)
{
	static char *pagesize_str[] = {
		"512", "2K", "4K", "8K", "16K", "unknown",
		"unknown", "unknown"
	};
	return pagesize_str[(pagetype & 0x07)];
}

static struct match_reg_type page2size[] = {
	{ _512B, NAND_PAGE_512B },
	{ _2K, NAND_PAGE_2K },
	{ _4K, NAND_PAGE_4K },
	{ _8K, NAND_PAGE_8K },
	{ _16K, NAND_PAGE_16K },
	{ _32K, NAND_PAGE_32K },
};

unsigned int get_pagesize(enum page_type pagetype)
{
	unsigned int pagesize[] = {
		_512B, _2K, _4K, _8K, _16K, 0, 0, 0
	};
	return pagesize[(pagetype & 0x07)];
}

int nandpage_size2type(int size)
{
	return reg2type(page2size, ARRAY_SIZE(page2size), size, NAND_PAGE_2K);
}

int nandpage_type2size(int size)
{
	return type2reg(page2size, ARRAY_SIZE(page2size), size, NAND_PAGE_2K);
}

char *nand_dbgfs_options;

static int __init dbgfs_options_setup(char *s)
{
	nand_dbgfs_options = s;
	return 1;
}
__setup("nanddbgfs=", dbgfs_options_setup);

int get_bits(unsigned int n)
{
	int loop;
	int ret = 0;

	if (!n)
		return 0;

	if (n > 0xFFFF)
		loop = n > 0xFFFFFF ? 32 : 24;
	else
		loop = n > 0xFF ? 16 : 8;

	while (loop-- > 0 && n) {
		if (n & 1)
			ret++;
		n >>= 1;
	}
	return ret;
}

#define et_ecc_none	0x00
#define et_ecc_4bit	0x02
#define et_ecc_8bit	0x03
#define et_ecc_24bit1k	0x04
#define et_ecc_40bit1k	0x05
#define et_ecc_64bit1k	0x06

static struct match_reg_type ecc_yaffs_type_t[] = {
	{et_ecc_none,		NAND_ECC_0BIT},
	{et_ecc_4bit,		NAND_ECC_8BIT},
	{et_ecc_8bit,		NAND_ECC_16BIT},
	{et_ecc_24bit1k,	NAND_ECC_24BIT},
	{et_ecc_40bit1k,	NAND_ECC_40BIT},
	{et_ecc_64bit1k,	NAND_ECC_64BIT}
};

unsigned char match_ecc_type_to_yaffs(unsigned char type)
{
	return type2reg(ecc_yaffs_type_t, ARRAY_SIZE(ecc_yaffs_type_t), type,
			et_ecc_4bit);
}

static struct match_t page_table[] = {
	{NAND_PAGE_2K,	PAGE_SIZE_2KB,	"2K"},
	{NAND_PAGE_4K,	PAGE_SIZE_4KB,	"4K"},
	{NAND_PAGE_8K,	PAGE_SIZE_8KB,	"8K"},
	{NAND_PAGE_16K,	PAGE_SIZE_16KB,	"16K"},
};

unsigned char match_page_reg_to_type(unsigned char reg)
{
	return match_reg_to_type(page_table, ARRAY_SIZE(page_table), reg,
				 NAND_PAGE_2K);
}

unsigned char match_page_type_to_reg(unsigned char type)
{
	return match_type_to_reg(page_table, ARRAY_SIZE(page_table), type,
				 PAGE_SIZE_2KB);
}
EXPORT_SYMBOL_GPL(match_page_type_to_reg);

const char *match_page_type_to_str(unsigned char type)
{
	return match_type_to_data(page_table, ARRAY_SIZE(page_table), type,
				  "unknown");
}

static struct match_t ecc_table[] = {
	{NAND_ECC_0BIT,		ECC_TYPE_0BIT,	"none"},
	{NAND_ECC_8BIT,		ECC_TYPE_8BIT,	"4bit/512"},
	{NAND_ECC_16BIT,	ECC_TYPE_16BIT,	"8bit/512"},
	{NAND_ECC_24BIT,	ECC_TYPE_24BIT,	"24bit/1K"},
	{NAND_ECC_28BIT,	ECC_TYPE_28BIT,	"28bit/1K"},
	{NAND_ECC_40BIT,	ECC_TYPE_40BIT,	"40bit/1K"},
	{NAND_ECC_64BIT,	ECC_TYPE_64BIT,	"64bit/1K"},
};

unsigned char match_ecc_reg_to_type(unsigned char reg)
{
	return match_reg_to_type(ecc_table, ARRAY_SIZE(ecc_table), reg,
				 NAND_ECC_8BIT);
}

unsigned char match_ecc_type_to_reg(unsigned char type)
{
	return match_type_to_reg(ecc_table, ARRAY_SIZE(ecc_table), type,
				 ECC_TYPE_8BIT);
}
EXPORT_SYMBOL_GPL(match_ecc_type_to_reg);

const char *match_ecc_type_to_str(unsigned char type)
{
	return match_type_to_data(ecc_table, ARRAY_SIZE(ecc_table), type,
				  "unknown");
}

static struct match_t page_type_size_table[] = {
	{NAND_PAGE_2K,	_2K,	NULL},
	{NAND_PAGE_4K,	_4K,	NULL},
	{NAND_PAGE_8K,	_8K,	NULL},
	{NAND_PAGE_16K,	_16K,	NULL},
};

unsigned char match_page_size_to_type(unsigned int size)
{
	return match_reg_to_type(page_type_size_table,
				 ARRAY_SIZE(page_type_size_table), size, NAND_PAGE_2K);
}

unsigned int match_page_type_to_size(unsigned char type)
{
	return match_type_to_reg(page_type_size_table,
				 ARRAY_SIZE(page_type_size_table), type, _2K);
}
EXPORT_SYMBOL_GPL(match_page_type_to_size);
