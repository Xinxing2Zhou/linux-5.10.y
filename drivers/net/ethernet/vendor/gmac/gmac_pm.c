/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/crc16.h>
#include <linux/securec.h>
#include "gmac_pm.h"

struct pm_reg_config pm_reg_config_backup;

static void init_crc_table(void);
static unsigned short compute_crc(const char *message, int nbytes);
static unsigned short calculate_crc16(const char *buf, unsigned int mask)
{
	char data[N];
	int i;
	int len = 0;

	if (memset_s(data, sizeof(data), 0, sizeof(data)) != EOK)
		printk("memset_s  err : %s %d.\n", __func__, __LINE__);

	for (i = 0; i < N; i++) {
		if (mask & 0x1)
			data[len++] = buf[i];

		mask >>= 1;
	}

	return compute_crc(data, len);
}

/* use this func in config pm func */
static void _pmt_reg_backup(struct gmac_netdev_local const *ld)
{
	if (ld == NULL)
		return;
	pm_reg_config_backup.pmt_ctrl = readl(ld->gmac_iobase + PMT_CTRL);
	pm_reg_config_backup.pmt_mask0 = readl(ld->gmac_iobase + PMT_MASK0);
	pm_reg_config_backup.pmt_mask1 = readl(ld->gmac_iobase + PMT_MASK1);
	pm_reg_config_backup.pmt_mask2 = readl(ld->gmac_iobase + PMT_MASK2);
	pm_reg_config_backup.pmt_mask3 = readl(ld->gmac_iobase + PMT_MASK3);
	pm_reg_config_backup.pmt_cmd = readl(ld->gmac_iobase + PMT_CMD);
	pm_reg_config_backup.pmt_offset = readl(ld->gmac_iobase + PMT_OFFSET);
	pm_reg_config_backup.pmt_crc1_0 = readl(ld->gmac_iobase + PMT_CRC1_0);
	pm_reg_config_backup.pmt_crc3_2 = readl(ld->gmac_iobase + PMT_CRC3_2);
}

#define	PM_SET			1
#define PM_CLEAR		0

static void pmt_config_filter(struct pm_config const *config,
			      struct gmac_netdev_local const *ld)
{
	unsigned int v;
	unsigned int cmd = 0;
	unsigned int offset = 0;
	unsigned short crc[FILTERS] = { 0 };
	int reg_mask;
	unsigned int i;

	/*
	 * filter.valid		mask.valid	mask_bytes	effect
	 *	0		*		*		no use the filter
	 *	1		0		*	all pkts can wake-up(non-exist)
	 *	1		1		0		all pkts can wake-up
	 *	1		1		!0		normal filter
	 */
	/* setup filter */
	for (i = 0; i < FILTERS; i++) {
		if (config->filter[i].valid) {
			if (config->filter[i].offset < PM_FILTER_OFFSET_MIN)
				continue;
			/* high 8 bits offset and low 8 bits valid bit */
			offset |= config->filter[i].offset << (i * 8);
			cmd |= BIT(i * 8); /* valid bit8 */
			/* mask offset 4i */
			reg_mask = PMT_MASK0 + (i * 4);

			/*
			 * for logic, mask valid bit(bit31) must set to 0,
			 * 0 is enable
			 */
			v = config->filter[i].mask_bytes;
			v &= ~BIT(31); /* bit31 */
			writel(v, ld->gmac_iobase + reg_mask);

			/* crc */
			crc[i] = calculate_crc16(config->filter[i].value, v);
			if (i <= 1) { /* for filter0 and filter 1 */
				v = readl(ld->gmac_iobase + PMT_CRC1_0);
				v &= ~(0xFFFF << (16 * i)); /* 16 bits mask */
				v |= crc[i] << (16 * i); /* 16 bits mask */
				writel(v, ld->gmac_iobase + PMT_CRC1_0);
			} else { /* filter2 and filter3 */
				v = readl(ld->gmac_iobase + PMT_CRC3_2);
				v &= ~(0xFFFF << (16 * (i - 2))); /* filer 2 3, 16 bits mask */
				v |= crc[i] << (16 * (i - 2)); /* filer 2 3, 16 bits mask */
				writel(v, ld->gmac_iobase + PMT_CRC3_2);
			}
		}
	}

	if (cmd) {
		writel(offset, ld->gmac_iobase + PMT_OFFSET);
		writel(cmd, ld->gmac_iobase + PMT_CMD);
	}
}

static int pmt_config_gmac(struct pm_config const *config, struct gmac_netdev_local *ld)
{
	unsigned int v;
	unsigned long flags;

	if (ld == NULL || config == NULL)
		return -EINVAL;

	spin_lock_irqsave(&ld->pmtlock, flags);
	if (config->wakeup_pkts_enable) {
		/* disable wakeup_pkts_enable before reconfig? */
		v = readl(ld->gmac_iobase + PMT_CTRL);
		v &= ~BIT(2); /* bit2 */
		writel(v, ld->gmac_iobase + PMT_CTRL); /* any side effect? */
	} else {
		goto config_ctrl;
	}

	pmt_config_filter(config, ld);

config_ctrl:
	v = 0;
	if (config->uc_pkts_enable)
		v |= BIT(9); /* bit9 uc pkts wakeup */
	if (config->wakeup_pkts_enable)
		v |= BIT(2); /* bit2 use filter framework */
	if (config->magic_pkts_enable)
		v |= BIT(1); /* magic pkts wakeup */

	v |= 0x3 << 5; /* set bit5 bit6, clear irq status */
	writel(v, ld->gmac_iobase + PMT_CTRL);

	_pmt_reg_backup(ld);

	spin_unlock_irqrestore(&ld->pmtlock, flags);

	return 0;
}

/* pmt_config will overwrite pre-config */
int pmt_config(struct net_device const *ndev, struct pm_config const *config)
{
	static int init;
	int ret;
	struct gmac_netdev_local *priv = netdev_priv(ndev);

	if (!init)
		init_crc_table();

	ret = pmt_config_gmac(config, priv);
	if (ret)
		return ret;

	priv->pm_state = PM_SET;
	priv->wol_enable = true;
	device_set_wakeup_enable(priv->dev, 1);

	return 0;
}

bool pmt_enter(struct gmac_netdev_local *ld)
{
	int pm = false;
	unsigned long flags;
	if (ld == NULL)
		return -EINVAL;
	spin_lock_irqsave(&ld->pmtlock, flags);
	if (ld->pm_state == PM_SET) {
		unsigned int v;

		v = readl(ld->gmac_iobase + PMT_CTRL);
		v |= BIT(0); /* enter power down */
		v |= BIT(3); /* bit3, enable wakeup irq */
		v |= 0x3 << 5; /* set bit5 bit6, clear irq status */
		writel(v, ld->gmac_iobase + PMT_CTRL);

		ld->pm_state = PM_CLEAR;
		pm = true;
	}
	spin_unlock_irqrestore(&ld->pmtlock, flags);
	return pm;
}

void pmt_exit(struct gmac_netdev_local *ld)
{
	unsigned int v;
	unsigned long flags;
	if (ld == NULL)
		return;
	/* logic auto exit power down mode */
	spin_lock_irqsave(&ld->pmtlock, flags);

	v = readl(ld->gmac_iobase + PMT_CTRL);
	v &= ~BIT(0); /* enter power down */
	v &= ~BIT(3); /* bit3, enable wakeup irq */

	v |= 0x3 << 5; /* set bit5 bit6, clear irq status */
	writel(v, ld->gmac_iobase + PMT_CTRL);

	spin_unlock_irqrestore(&ld->pmtlock, flags);

	ld->wol_enable = false;
}

void pmt_reg_restore(struct gmac_netdev_local *ld)
{
	unsigned int v;
	unsigned long flags;
	if (ld == NULL)
		return;
	spin_lock_irqsave(&ld->pmtlock, flags);
	v = pm_reg_config_backup.pmt_mask0;
	writel(v, ld->gmac_iobase + PMT_MASK0);

	v = pm_reg_config_backup.pmt_mask1;
	writel(v, ld->gmac_iobase + PMT_MASK1);

	v = pm_reg_config_backup.pmt_mask2;
	writel(v, ld->gmac_iobase + PMT_MASK2);

	v = pm_reg_config_backup.pmt_mask3;
	writel(v, ld->gmac_iobase + PMT_MASK3);

	v = pm_reg_config_backup.pmt_cmd;
	writel(v, ld->gmac_iobase + PMT_CMD);

	v = pm_reg_config_backup.pmt_offset;
	writel(v, ld->gmac_iobase + PMT_OFFSET);

	v = pm_reg_config_backup.pmt_crc1_0;
	writel(v, ld->gmac_iobase + PMT_CRC1_0);

	v = pm_reg_config_backup.pmt_crc3_2;
	writel(v, ld->gmac_iobase + PMT_CRC3_2);

	v = pm_reg_config_backup.pmt_ctrl;
	writel(v, ld->gmac_iobase + PMT_CTRL);
	spin_unlock_irqrestore(&ld->pmtlock, flags);
}

/* ========the following code copy from Synopsys DWC_gmac_crc_example.c====== */
#define CRC16 /* Change it to CRC16 for CRC16 Computation */

#if defined(CRC16)
#define CRC_NAME		"CRC-16"
#define POLYNOMIAL		0x8005
#define INITIAL_REMAINDER	0xFFFF
#define FINAL_XOR_VALUE		0x0000
#define REVERSE_DATA
#undef REVERSE_REMAINDER
#endif

#define WIDTH    (8 * sizeof(unsigned short))
#define TOPBIT   BIT(WIDTH - 1)

#ifdef REVERSE_DATA
#undef  REVERSE_DATA
#define reverse_data(X)		((unsigned char)reverse((X), 8))
#else
#undef  REVERSE_DATA
#define reverse_data(X)		(X)
#endif

#ifdef REVERSE_REMAINDER
#undef  REVERSE_REMAINDER
#define reverse_remainder(X)	((unsigned short)reverse((X), WIDTH))
#else
#undef  REVERSE_REMAINDER
#define reverse_remainder(X)	(X)
#endif

#define CRC_TABLE_LEN	256
static unsigned short crc_table[CRC_TABLE_LEN];

static unsigned int reverse(unsigned int data, unsigned char nbits)
{
	unsigned int reversed = 0x00000000;
	unsigned char bit;

	/* Reverse the data about the center bit. */
	for (bit = 0; bit < nbits; ++bit) {
		/* If the LSB bit is set, set the reflection of it. */
		if (data & 0x01)
			reversed |= BIT((nbits - 1) - bit);

		data = (data >> 1);
	}
	return reversed;
}

/* This Initializes the partial CRC look up table */
static void init_crc_table(void)
{
	unsigned short remainder;
	unsigned int dividend;
	unsigned char bit;

	/* Compute the remainder of each possible dividend. */
	for (dividend = 0; dividend < CRC_TABLE_LEN; ++dividend) {
		/* Start with the dividend followed by zeros, WIDTH - 8. */
		remainder = (unsigned short)(dividend << (WIDTH - 8));

		/* Perform modulo-2 division, a bit at a time for 8 times. */
		for (bit = 8; bit > 0; --bit) {
			/* Try to divide the current data bit. */
			if (remainder & TOPBIT)
				remainder = (remainder << 1) ^ POLYNOMIAL;
			else
				remainder = (remainder << 1);
		}

		/* Store the result into the table. */
		crc_table[dividend] = remainder;
	}
}

static unsigned short compute_crc(const char *message, int nbytes)
{
	unsigned short remainder = INITIAL_REMAINDER;
	int byte;
	unsigned char data;

	/* Divide the message by the polynomial, a byte at a time. */
	for (byte = 0; byte < nbytes; ++byte) {
		/* high 8 bits */
		data = reverse_data(message[byte]) ^ (remainder >> (WIDTH - 8));
		remainder = crc_table[data] ^ (remainder << 8); /* shift left 8 bits */
	}

	/* The final remainder is the CRC. */
	return (reverse_remainder(remainder) ^ FINAL_XOR_VALUE);
}
