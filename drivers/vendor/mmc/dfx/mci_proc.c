/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#include <linux/version.h>
#include <linux/seq_file.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include "card.h"
#include "core.h"
#include "sdhci_nebula.h"
#include "mci_proc.h"

#define MCI_PARENT       "mci"
#define MCI_STATS_PROC   "mci_info"
#define MAX_CLOCK_SCALE	 4

static struct proc_dir_entry *g_proc_mci_dir;

static char *g_card_type[MAX_CARD_TYPE + 1] = {
	"MMC card",
	"SD card",
	"SDIO card",
	"SD combo (IO+mem) card",
	"unknown"
};
static char *g_clock_unit[MAX_CLOCK_SCALE] = {
	"Hz",
	"KHz",
	"MHz",
	"GHz"
};

#define BIT_WIDTH	32
static unsigned int unstuff_bits(const u32 *resp, u32 start, u32 size)
{
	const u32 mask = ((size < BIT_WIDTH) ? 1 << size : 0) - 1;
	const u32 off = 0x3 - ((start) / BIT_WIDTH);
	const u32 shft = (start) & 31; /* max shift value 31 */
	u32 res;

	res = resp[off] >> shft;
	if (size + shft > BIT_WIDTH)
		res |= resp[off - 1] << ((BIT_WIDTH - shft) % BIT_WIDTH);
	res = res & mask;

	return res;
}

static char *mci_get_card_type(unsigned int sd_type)
{
	if (sd_type >= MAX_CARD_TYPE)
		return g_card_type[MAX_CARD_TYPE];
	else
		return g_card_type[sd_type];
}

static unsigned int analyze_clock_scale(unsigned int clock,
	unsigned int *clock_val)
{
	unsigned int scale = 0;
	unsigned int tmp = clock;

	while (1) {
		tmp = tmp / 1000; /* Cal freq by dividing 1000 */
		if (tmp > 0) {
			*clock_val = tmp;
			scale++;
		} else {
			break;
		}
	}
	return scale;
}

static inline int is_card_uhs(unsigned char timing)
{
	return timing >= MMC_TIMING_UHS_SDR12 &&
	       timing <= MMC_TIMING_UHS_DDR50;
};

static inline int is_card_hs(unsigned char timing)
{
	return timing == MMC_TIMING_SD_HS || timing == MMC_TIMING_MMC_HS;
};

static void mci_stats_seq_speed(struct seq_file *s, struct mmc_host *mmc)
{
	unsigned int speed_class, grade_speed_uhs;
	struct mmc_card *card = mmc->card;
	const char *uhs_bus_speed_mode = "";
	static const char * const uhs_speeds[] = {
		[UHS_SDR12_BUS_SPEED] = "SDR12 ",
		[UHS_SDR25_BUS_SPEED] = "SDR25 ",
		[UHS_SDR50_BUS_SPEED] = "SDR50 ",
		[UHS_SDR104_BUS_SPEED] = "SDR104 ",
		[UHS_DDR50_BUS_SPEED] = "DDR50 ",
	};

	if (is_card_uhs(mmc->ios.timing) &&
			card->sd_bus_speed < ARRAY_SIZE(uhs_speeds))
		uhs_bus_speed_mode =
			uhs_speeds[card->sd_bus_speed];

	seq_printf(s, "\tMode: %s %s\n",
		   is_card_uhs(mmc->ios.timing) ? "UHS" :
		   is_card_hs(mmc->ios.timing) ? "HS" :
		   (mmc->ios.enhanced_strobe == true) ? "HS400ES" :
		   (mmc->ios.timing == MMC_TIMING_MMC_HS400) ? "HS400" :
		   (mmc->ios.timing == MMC_TIMING_MMC_HS200) ? "HS200" :
		   (mmc->ios.timing == MMC_TIMING_MMC_DDR52) ? "DDR" :
		   "DS", uhs_bus_speed_mode);

	speed_class = unstuff_bits(card->raw_ssr, 56, 8); /* 56 = 440 - 384 */
	grade_speed_uhs = unstuff_bits(card->raw_ssr, 12, 4); /* 12 = 396 - 384 */
	seq_printf(s, "\tSpeed Class: Class %s\n",
		   (speed_class == 0x00) ? "0" :
		   (speed_class == 0x01) ? "2" :
		   (speed_class == 0x02) ? "4" :
		   (speed_class == 0x03) ? "6" :
		   (speed_class == 0x04) ? "10" :
		   "Reserved");
	seq_printf(s, "\tUhs Speed Grade: %s\n",
		   (grade_speed_uhs == 0x00) ?
		   "Less than 10MB/sec(0h)" :
		   (grade_speed_uhs == 0x01) ?
		   "10MB/sec and above(1h)" :
		   "Reserved");
}

static void mci_stats_seq_clock(struct seq_file *s, struct mmc_host *mmc)
{
	unsigned int clock, clock_scale;
	unsigned int clock_value = 0;

	clock = mmc->ios.clock;
	clock_scale = analyze_clock_scale(clock, &clock_value);
	seq_printf(s, "\tHost work clock: %d%s\n",
		   clock_value, g_clock_unit[clock_scale]);

	clock = mmc->ios.clock;
	clock_scale = analyze_clock_scale(clock, &clock_value);
	seq_printf(s, "\tCard support clock: %d%s\n",
		   clock_value, g_clock_unit[clock_scale]);

	clock = mmc->actual_clock;
	clock_scale = analyze_clock_scale(clock, &clock_value);
	seq_printf(s, "\tCard work clock: %d%s\n",
		   clock_value, g_clock_unit[clock_scale]);
}

static void mci_stats_seq_printout(struct seq_file *s)
{
	unsigned int index_mci;
	const char *type = NULL;
	struct mmc_host *mmc = NULL;
	struct mmc_card *card = NULL;
	int present;
	struct sdhci_host *host = NULL;
	struct sdhci_nebula *priv = NULL;

	for (index_mci = 0; index_mci < MCI_SLOT_NUM; index_mci++) {
		mmc = g_mci_host[index_mci];

		if (mmc == NULL) {
			seq_printf(s, "MCI%d: invalid\n", index_mci);
			continue;
		} else {
			seq_printf(s, "MCI%d", index_mci);
		}

		mmc_claim_host(mmc);
		host = mmc_priv(mmc);
		priv = nebula_priv(host);

		present = host->mmc->ops->get_cd(host->mmc);
		if (present != 0)
			seq_puts(s, ": pluged");
		else
			seq_puts(s, ": unplugged");

		card = host->mmc->card;
		if (present == 0) {
			seq_puts(s, "_disconnected\n");
		} else if ((present != 0) && (card == NULL)) {
			seq_puts(s, "_init_failed\n");
		} else if (card != NULL) {
			seq_puts(s, "_connected\n");

			seq_printf(s, "\tType: %s",
				   mci_get_card_type(card->type));

			if (card->state & MMC_STATE_BLOCKADDR) {
				type = ((card->state & MMC_CARD_SDXC) != 0) ?
				       "SDXC" : "SDHC";
				seq_printf(s, "(%s)\n", type);
			}

			mci_stats_seq_speed(s, mmc);
			mci_stats_seq_clock(s, mmc);

			/* add card read/write error count */
			seq_printf(s, "\tCard error count: %d\n", host->error_count);
		}
		mmc_release_host(mmc);
	}
}

/* proc interface setup */
static void *mci_seq_start(struct seq_file *s, loff_t *pos)
{
	/*  counter is used to tracking multi proc interfaces
	 *  We have only one interface so return zero
	 *  pointer to start the sequence.
	 */
	static unsigned long counter;

	if (*pos == 0)
		return &counter;

	return NULL;
}

/* proc interface next */
static void *mci_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	(*pos)++;

	return mci_seq_start(s, pos);
}

/* define parameters where showed in proc file */
static int mci_stats_seq_show(struct seq_file *s, void *v)
{
	mci_stats_seq_printout(s);
	return 0;
}

/* proc interface stop */
static void mci_seq_stop(struct seq_file *s, void *v)
{
}

/* proc interface operation */
static const struct seq_operations mci_stats_seq_ops = {
	.start = mci_seq_start,
	.next = mci_seq_next,
	.stop = mci_seq_stop,
	.show = mci_stats_seq_show
};

/* proc file open */
static int mci_stats_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &mci_stats_seq_ops);
};

/* proc file operation */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations  mci_stats_proc_ops = {
	.open = mci_stats_proc_open,
	.read = seq_read,
	.release = seq_release
};
#else
static const struct proc_ops  mci_stats_proc_ops = {
	.proc_open = mci_stats_proc_open,
	.proc_read = seq_read,
	.proc_release = seq_release
};
#endif

int mci_proc_init(void)
{
	struct proc_dir_entry *proc_stats_entry = NULL;

	g_proc_mci_dir = proc_mkdir(MCI_PARENT, NULL);
	if (g_proc_mci_dir == NULL) {
		pr_err("%s: failed to create proc file %s\n",
		       __func__, MCI_PARENT);
		return 1;
	}

	proc_stats_entry = proc_create(MCI_STATS_PROC,
				       0, g_proc_mci_dir, &mci_stats_proc_ops);
	if (proc_stats_entry == NULL) {
		pr_err("%s: failed to create proc file %s\n",
		       __func__, MCI_STATS_PROC);
		remove_proc_entry(MCI_PARENT, NULL);
		return 1;
	}

	return 0;
}

int mci_proc_shutdown(void)
{
	if (g_proc_mci_dir != NULL) {
		remove_proc_entry(MCI_STATS_PROC, g_proc_mci_dir);
		remove_proc_entry(MCI_PARENT, NULL);
		g_proc_mci_dir = NULL;
	}

	return 0;
}
