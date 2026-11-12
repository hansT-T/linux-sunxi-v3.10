/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_MMC_SUNXI_MMC_V1_PANIC_H
#define _LINUX_MMC_SUNXI_MMC_V1_PANIC_H

#include <linux/types.h>

struct gendisk;
struct mmc_card;
struct mmc_host;

/* T3's reserved eMMC user-area partition, in 512-byte sectors. */
#define SUNXI_MMC_V1_LOG_PARTNO		4
#define SUNXI_PSTORE_START_SECTOR	15237120U
#define SUNXI_PSTORE_NR_SECTORS		32768U
#define SUNXI_PSTORE_SECTOR_SIZE	512U

bool sunxi_mmc_v1_panic_target(struct mmc_host *mmc);

/*
 * Fatal panic only, after local IRQs are disabled and other CPUs stopped.
 * sector is relative to the fixed log area. bytes is the nonzero payload
 * length. The last sector is zero-padded, and buf need not be word-aligned.
 * Success is 0. Failure may leave earlier sectors written.
 * Sequential successful calls share one takeover.
 */
int sunxi_mmc_panic_write(u32 sector, const void *buf, size_t bytes);

#ifdef CONFIG_MMC_SUNXI_PANIC_LOG
void sunxi_mmc_panic_log_add(struct mmc_card *card, struct gendisk *disk);
void sunxi_mmc_panic_log_remove(struct mmc_card *card);
#else
static inline void sunxi_mmc_panic_log_add(struct mmc_card *card,
					  struct gendisk *disk)
{
}

static inline void sunxi_mmc_panic_log_remove(struct mmc_card *card)
{
}
#endif

#endif
