/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>

#include "card.h"
#include "core.h"
#include "sdhci_nebula.h"
#include "nebula_dfx.h"

#ifdef CONFIG_SDHCI_NEBULA_DFX

#define BITS_PER_U32	32
#define BYTES_PER_U32   4

#define SD_SSR_BITS_OFFSET              384
#define SD_SSR_SPEED_CLASS_WIDTH        8
#define SD_SSR_SPEED_CLASS_OFS          (440 - SD_SSR_BITS_OFFSET)
#define SD_SSR_UHS_SPEED_GRADE_WIDTH    4
#define SD_SSR_UHS_SPEED_GRADE_OFS      (396 - SD_SSR_BITS_OFFSET)

/**
 * R1_OUT_OF_RANGE - Command argument out of range
 * R1_ADDRESS_ERROR - Misaligned address
 * R1_BLOCK_LEN_ERROR - Transferred block length incorrect
 * R1_WP_VIOLATION - Tried to write to protected block
 * R1_CC_ERROR - Card controller error
 * R1_ERROR - General/unknown error
 */
#define CMD_R1_ERRORS  \
	(R1_OUT_OF_RANGE | R1_ADDRESS_ERROR |   \
	 R1_BLOCK_LEN_ERROR | R1_WP_VIOLATION | \
	 R1_CC_ERROR | R1_ERROR)

static char *g_card_type_string[] = {
	"MMC card",
	"SD card",
	"SDIO card",
	"SD combo (IO+mem) card",
	"unknown"
};

static void nebula_show_str(struct seq_file *m, const char *key, const char *val)
{
	seq_printf(m, "%-25s:%-35s|\n", key, val);
}

static void nebula_show_u32(struct seq_file *m, const char *key, u32 val)
{
	seq_printf(m, "%-25s:%-35u|\n", key, val);
}

static void nebula_show_hex(struct seq_file *m, const char *key, u32 val)
{
	seq_printf(m, "%-25s:0x%-33x|\n", key, val);
}

static u32 unstuff_bits(const u32 *resp, u32 start, u32 size)
{
	const u32 mask = ((size < BITS_PER_U32) ? (1 << size) : 0) - 1;
	const u32 off = (BYTES_PER_U32 - 1) - ((start) / BITS_PER_U32);
	const u32 shft = (start) & (BITS_PER_U32 - 1);
	u32 res;

	res = resp[off] >> shft;
	if ((size + shft) > BITS_PER_U32)
		res |= resp[off - 1] << ((BITS_PER_U32 - shft) % BITS_PER_U32);
	res = res & mask;

	return res;
}

static char *nebula_get_card_type(u32 card_type)
{
	if (card_type >= ARRAY_SIZE(g_card_type_string))
		return g_card_type_string[ARRAY_SIZE(g_card_type_string) - 1];
	else
		return g_card_type_string[card_type];
}

static void nebula_help_show(struct seq_file *m)
{
	seq_puts(m, "#\n"
		"# echo \"help <on | off>\" > /proc/mmc[x]/status\n"
		"# echo \"rescan <mode>\" > /proc/mmc[x]/status\n"
		"#  mode format: <speed>,<bus_width>; example: \"hs200,4\"\n"
		"#  speed: ds, hs, hs200, hs400, hs400es, sdr12, sdr25, sdr50, sdr104\n"
		"#  bus_width: 1, 4, 8\n"
		"# echo \"lvl=<log_level>\" > /proc/mmc[x]/status\n"
		"#  log_level: 0, 1, 2\n"
		"#\n");
}

static inline bool is_card_uhs(unsigned char timing)
{
	return (timing >= MMC_TIMING_UHS_SDR12) &&
	       (timing <= MMC_TIMING_UHS_DDR50);
};

static inline bool is_card_hs(unsigned char timing)
{
	return (timing == MMC_TIMING_SD_HS) || (timing == MMC_TIMING_MMC_HS);
};

static void nebula_seq_sd_info(struct seq_file *m)
{
	struct sdhci_host *host = (struct sdhci_host *)m->private;
	struct card_info *info = &host->c_info;
	const char *uhs_bus_speed_mode = "DS";
	u32 speed_class, grade_speed_uhs;
	static const char * const uhs_speeds[] = {
		[UHS_SDR12_BUS_SPEED] = "UHS SDR12",
		[UHS_SDR25_BUS_SPEED] = "UHS SDR25",
		[UHS_SDR50_BUS_SPEED] = "UHS SDR50",
		[UHS_SDR104_BUS_SPEED] = "UHS SDR104",
		[UHS_DDR50_BUS_SPEED] = "UHS DDR50",
	};

	if (is_card_uhs(info->timing) && info->sd_bus_speed < ARRAY_SIZE(uhs_speeds)) {
		uhs_bus_speed_mode = uhs_speeds[info->sd_bus_speed];
	}

	nebula_show_str(m, "  work_mode", (is_card_hs(info->timing)) ? "HS" : uhs_bus_speed_mode);

	nebula_show_str(m, "  capacity_type",
		((info->card_state & MMC_STATE_BLOCKADDR) == 0) ? "SD (<=2GB)" :
		((info->card_state & MMC_CARD_SDXC) != 0) ? "SDXC (<=2TB)" : "SDHC (<=32GB)");

	speed_class = unstuff_bits(info->ssr, SD_SSR_SPEED_CLASS_OFS,
					SD_SSR_SPEED_CLASS_WIDTH);
	nebula_show_str(m, "  speed_class",
		(speed_class == SD_SPEED_CLASS0) ? "0" :
		(speed_class == SD_SPEED_CLASS1) ? "2" :
		(speed_class == SD_SPEED_CLASS2) ? "4" :
		(speed_class == SD_SPEED_CLASS3) ? "6" :
		(speed_class == SD_SPEED_CLASS4) ? "10" :
			"Reserved");

	grade_speed_uhs = unstuff_bits(info->ssr, SD_SSR_UHS_SPEED_GRADE_OFS,
					SD_SSR_UHS_SPEED_GRADE_WIDTH);
	nebula_show_str(m, "  uhs_speed_grade",
		(grade_speed_uhs == SD_SPEED_GRADE0) ?
		"Less than 10MB/sec(0h)" :
		(grade_speed_uhs == SD_SPEED_GRADE1) ?
		"10MB/sec and above(1h)" :
		"Reserved");
}

static void nebula_seq_emmc_info(struct seq_file *m)
{
	struct sdhci_host *host = (struct sdhci_host *)m->private;
	struct mmc_host *mmc = host->mmc;

	nebula_show_str(m, "  work_mode",
		(mmc->ios.enhanced_strobe) ? "HS400ES" :
		(mmc->ios.timing == MMC_TIMING_MMC_HS400) ? "HS400" :
		(mmc->ios.timing == MMC_TIMING_MMC_HS200) ? "HS200" :
		(mmc->ios.timing == MMC_TIMING_MMC_DDR52) ? "DDR" :
		(mmc->ios.timing == MMC_TIMING_MMC_HS) ? "HS" : "DS");
}

static void nebula_cmd_backtrace_show(struct seq_file *m)
{
	int idx, sp;
	struct sdhci_host *host = (struct sdhci_host *)m->private;
	struct sdhci_nebula *nebula = nebula_priv(host);

	nebula_show_str(m, "command_trace", "idx command");
	sp = (nebula->cmd_bt.sp % NEBULA_DFX_BT_MAX_NUM) + 1;
	for (idx = 0; idx < NEBULA_DFX_BT_MAX_NUM; idx++) {
		seq_printf(m, "%-25s: %d   CMD%-27u|\n", "", idx,
			nebula->cmd_bt.opcode[sp++]);
		sp %= NEBULA_DFX_BT_MAX_NUM;
	}
}

static int nebula_stats_show(struct seq_file *m, void *v)
{
	bool card_insert = true;
	struct sdhci_host *host = (struct sdhci_host *)m->private;
	struct card_info *info = &host->c_info;
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->dfx_cap.help) {
		nebula_help_show(m);
		nebula->dfx_cap.help = false;
		return 0;
	}

	seq_printf(m, "========================%s=================================\n",
		mmc_hostname(host->mmc));
	nebula_show_str(m, "version", SDHCI_NEBULA_KERNEL_VERSION);
	nebula_show_str(m, "mmc_device", host->hw_name);

	if ((mmc_card_is_removable(host->mmc) != 0) && host->mmc->ops->get_cd != NULL)
		card_insert = host->mmc->ops->get_cd(host->mmc);

	if (card_insert && (info->card_connect != CARD_CONNECT) && (host->mmc->card_status == MMC_CARD_INIT_FAIL)) {
		nebula_show_str(m, "card_status", "pluged_init_failed");
		nebula_cmd_backtrace_show(m);
	} else if (card_insert && (info->card_connect == CARD_CONNECT)) {
		nebula_show_str(m, "card_status", "plugged");
	} else {
		nebula_show_str(m, "card_status", "unplugged");
	}

	if (card_insert && (info->card_connect == CARD_CONNECT)) {
		nebula_show_str(m, "  card_type", nebula_get_card_type(info->card_type));
		if (info->card_type == MMC_TYPE_MMC)
			nebula_seq_emmc_info(m);
		else
			nebula_seq_sd_info(m);

		nebula_show_str(m, "  cmdq_enable", (host->cqe_on) ? "true" : "false");
		nebula_show_u32(m, "  bus_width", (1 << host->mmc->ios.bus_width));
		nebula_show_u32(m, "  host_work_clock", info->card_support_clock);
		nebula_show_u32(m, "  card_work_clock", host->mmc->actual_clock);
	}

	nebula_show_hex(m, "host_caps1", host->mmc->caps);
	nebula_show_hex(m, "host_caps2", host->mmc->caps2);
	nebula_show_u32(m, "error_count", host->error_count);

	seq_puts(m, "=============================================================\n");

	return 0;
}

/* proc file open */
static int nebula_stats_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nebula_stats_show, PDE_DATA(inode));
};

static void nebula_parse_bus_width(struct sdhci_host *host, const char *mode)
{
	host->mmc->caps &= ~(MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA);

	if (strstr(mode, ",4") != NULL) {
		host->mmc->caps |= MMC_CAP_4_BIT_DATA;
	} else if (strstr(mode, ",8") != NULL) {
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;
	}
}

static void nebula_parse_mmc_mode(struct sdhci_host *host, const char *mode)
{
	u32 avail_type = 0;
	struct mmc_card *card = host->mmc->card;

	if (mmc_card_is_removable(card->host)) {
		return;
	}

	nebula_parse_bus_width(host, mode);

	if (strstr(mode, "hs400es") != NULL) {
		avail_type |= EXT_CSD_CARD_TYPE_HS400ES | EXT_CSD_CARD_TYPE_HS400 | \
					EXT_CSD_CARD_TYPE_HS200 | EXT_CSD_CARD_TYPE_HS;
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;
	} else if (strstr(mode, "hs400") != NULL) {
		avail_type |= EXT_CSD_CARD_TYPE_HS400 | EXT_CSD_CARD_TYPE_HS200 | \
					EXT_CSD_CARD_TYPE_HS;
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;
	} else if (strstr(mode, "hs200") != NULL) {
		avail_type |= EXT_CSD_CARD_TYPE_HS200 | EXT_CSD_CARD_TYPE_HS;
		host->mmc->caps |= MMC_CAP_4_BIT_DATA;
	} else if (strstr(mode, "hs") != NULL) {
		avail_type |= EXT_CSD_CARD_TYPE_HS;
	}

	card->mmc_avail_type = avail_type;
}

static void nebula_parse_sd_mode(struct sdhci_host *host, const char *mode)
{
	u32 sd_bus_speed = 0;

	host->mmc->caps &= ~(MMC_CAP_UHS | MMC_CAP_SD_HIGHSPEED);

	nebula_parse_bus_width(host, mode);

	if (strstr(mode, "sdr12")) {
		sd_bus_speed |= MMC_CAP_UHS_SDR12;
		host->mmc->caps |= MMC_CAP_4_BIT_DATA;
	} else if (strstr(mode, "sdr25") != NULL) {
		sd_bus_speed |= MMC_CAP_UHS_SDR25;
		host->mmc->caps |= MMC_CAP_4_BIT_DATA;
	} else if (strstr(mode, "sdr50") != NULL) {
		sd_bus_speed |= MMC_CAP_UHS_SDR50;
		host->mmc->caps |= MMC_CAP_4_BIT_DATA;
	} else if (strstr(mode, "sdr104") != NULL) {
		sd_bus_speed |= MMC_CAP_UHS_SDR104;
		host->mmc->caps |= MMC_CAP_4_BIT_DATA;
	} else if (strstr(mode, "hs") != NULL) {
		sd_bus_speed |= MMC_CAP_SD_HIGHSPEED;
	}

	host->mmc->caps |= sd_bus_speed;
}

static void nebula_trigger_rescan(struct sdhci_host *host, const char *mode)
{
	int ret;
	struct mmc_host *mmc = host->mmc;
	struct mmc_card *card = mmc->card;

	if (card != NULL) {
		mmc_claim_host(mmc);
		nebula_parse_mmc_mode(host, mode);
		nebula_parse_sd_mode(host, mode);
		mmc->card_status = MMC_CARD_UNINIT;
		ret = mmc_hw_reset(mmc);
		mmc->card_status = (ret == 0) ? MMC_CARD_INIT : MMC_CARD_INIT_FAIL;
		if (mmc->ops->card_info_save)
			mmc->ops->card_info_save(mmc);
		mmc_release_host(mmc);
	}
}

static void nebula_trigger_detect(struct sdhci_host *host, const char *mode)
{
	u8 val;
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->devid != MMC_DEV_TYPE_MMC_0) {
		val = sdhci_readb(host, SDHCI_WAKE_UP_CONTROL);
		val |= SDHCI_DETECT_POLARITY;
		sdhci_writeb(host, val, SDHCI_WAKE_UP_CONTROL);
	}
}

static void nebula_trigger_help(struct sdhci_host *host, const char *mode)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (strstr(mode, "on") != NULL) {
		nebula->dfx_cap.help = true;
	} else if (strstr(mode, "off") != NULL) {
		nebula->dfx_cap.help = false;
	}
}

static void nebula_trigger_log_level(struct sdhci_host *host, const char *mode)
{
	u32 lvl = DEBUG_LVL_INFO;
	char *ptr = NULL;
	struct sdhci_nebula *nebula = nebula_priv(host);

	ptr = strstr(mode, "=");
	if (ptr == NULL) {
		return;
	}

	ptr++;

	if (get_option(&ptr, &lvl) != 0) {
		return;
	}

	if (lvl >= DEBUG_LVL_INFO && lvl < DEBUG_LVL_MAX) {
		nebula->dfx_cap.log_level = lvl;
	}
}

static ssize_t nebula_proc_write(struct file *file, const char __user *buf,
	size_t count, loff_t *pos)
{
	char kbuf[MAX_STR_LEN] = {0};
	struct sdhci_host *host = (struct sdhci_host *)PDE_DATA(file_inode(file));

	if (count == 0)
		return -EINVAL;

	if (count > sizeof(kbuf))
		count = sizeof(kbuf) - 1;

	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;

	/* Strip trailing '\n' and terminate string */
	kbuf[count - 1] = 0;

	if (strstr(kbuf, "rescan") != NULL) {
		nebula_trigger_rescan(host, kbuf);
	} else if (strstr(kbuf, "help") != NULL) {
		nebula_trigger_help(host, kbuf);
	} else if (strstr(kbuf, "lvl") != NULL) {
		nebula_trigger_log_level(host, kbuf);
	} else if (strstr(kbuf, "detect") != NULL) {
		nebula_trigger_detect(host, kbuf);
	}

	return (ssize_t)count;
}
/* proc file operation */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops nebula_stats_proc_ops = {
	.proc_open = nebula_stats_proc_open,
	.proc_read = seq_read,
	.proc_write = nebula_proc_write,
	.proc_release = single_release,
};
#else
static const struct file_operations nebula_stats_proc_ops = {
	.owner = THIS_MODULE,
	.open    = nebula_stats_proc_open,
	.read    = seq_read,
	.release = single_release,
	.write   = nebula_proc_write,
};
#endif

void sdhci_nebula_dfx_irq(struct sdhci_host *host, u32 intmask)
{
	struct mmc_command *cmd = host->cmd;
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (cmd == NULL || ((intmask & SDHCI_INT_CMD_MASK) == 0)) {
		return;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	if (host->mmc->doing_init_tune != 0) {
		return;
	}
#endif

	nebula->cmd_bt.sp++;
	nebula->cmd_bt.sp %= NEBULA_DFX_BT_MAX_NUM;
	nebula->cmd_bt.opcode[nebula->cmd_bt.sp] = cmd->opcode;
}

int sdhci_nebula_proc_init(struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	nebula->proc_root = proc_mkdir(mmc_hostname(host->mmc), NULL);
	if (nebula->proc_root == NULL) {
		pr_err("%s: create proc file failed\n", mmc_hostname(host->mmc));
		return -ENOMEM;
	}

	nebula->proc_stat = proc_create_data("status", 0, nebula->proc_root,
							&nebula_stats_proc_ops, (void *)host);
	if (nebula->proc_stat == NULL) {
		pr_err("%s: create status file failed\n", mmc_hostname(host->mmc));
		remove_proc_entry(mmc_hostname(host->mmc), NULL);
		nebula->proc_root = NULL;
		return -ENOMEM;
	}

	nebula->dfx_cap.log_level = DEBUG_LVL_INFO;
	nebula->dfx_cap.help = false;

	return ERET_SUCCESS;
}

int sdhci_nebula_proc_shutdown(struct sdhci_host *host)
{
	struct sdhci_nebula *nebula = nebula_priv(host);

	if (nebula->proc_root != NULL) {
		if (nebula->proc_stat != NULL) {
			remove_proc_entry("status", nebula->proc_root);
			nebula->proc_stat = NULL;
		}
		remove_proc_entry(mmc_hostname(host->mmc), NULL);
		nebula->proc_root = NULL;
	}

	return ERET_SUCCESS;
}

#else
int sdhci_nebula_proc_init(struct sdhci_host *host)
{
	return ERET_SUCCESS;
}

int sdhci_nebula_proc_shutdown(struct sdhci_host *host)
{
	return ERET_SUCCESS;
};
#endif
