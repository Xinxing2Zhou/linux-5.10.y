/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef _DRIVERS_MMC_NEBULA_QUIRK_IDS_H
#define _DRIVERS_MMC_NEBULA_QUIRK_IDS_H

#define MMC_CMDQ_FORCE_OFF        0x1
#define MMC_CMDQ_DIS_WHITELIST    0x2

#ifdef CONFIG_MMC_CQHCI
#include "card.h"
#include "host.h"
#include "quirks.h"

#define CID_MANFID_SANDISK_F      0x45
/*
 * Quirk cmdq for MMC products.
 */
static inline void __maybe_unused nebula_cmdq_quirk_mmc(struct mmc_card *card, int data)
{
	struct mmc_host *host = card->host;

	if (host != NULL) {
		host->caps2 |= (MMC_CAP2_CQE | MMC_CAP2_CQE_DCMD);
		pr_debug("Whitelist: match device %s\n", card->cid.prod_name);
	}
}

static const struct mmc_fixup mmc_cmdq_whitelist[] = {
	/* Toshiba */
	MMC_FIXUP("008GB0", CID_MANFID_TOSHIBA, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	MMC_FIXUP("016G30", CID_MANFID_TOSHIBA, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	MMC_FIXUP("016GB0", CID_MANFID_TOSHIBA, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	MMC_FIXUP("008G30", CID_MANFID_TOSHIBA, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	/* Samsung */
	MMC_FIXUP("BJTD4R", CID_MANFID_SAMSUNG, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	MMC_FIXUP("CKTA42", CID_MANFID_SAMSUNG, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	MMC_FIXUP("8GTF4R", CID_MANFID_SAMSUNG, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	MMC_FIXUP("AJTD4R", CID_MANFID_SAMSUNG, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	/* Sandisk */
	MMC_FIXUP("DF4032", CID_MANFID_SANDISK_F, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	MMC_FIXUP("DG4016", CID_MANFID_SANDISK_F, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	MMC_FIXUP("DF4128", CID_MANFID_SANDISK_F, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	MMC_FIXUP("DG4008", CID_MANFID_SANDISK_F, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	/* Kingston */
	MMC_FIXUP("TB2816", CID_MANFID_KINGSTON, CID_OEMID_ANY, nebula_cmdq_quirk_mmc, 0),
	/* null, no remove */
	END_FIXUP
};

#endif /* CONFIG_MMC_CQHCI */

#endif /* _DRIVERS_MMC_BSP_QUIRK_IDS_H */
