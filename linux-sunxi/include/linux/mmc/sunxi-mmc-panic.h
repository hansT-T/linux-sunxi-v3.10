/* SPDX-License-Identifier: GPL-2.0 */
/*
 * T3/sdc2 v4p5x panic writer. See Documentation/mmc/sunxi-mmc-panic.md.
 * Licensed under the GNU General Public License, version 2.
 */
#ifndef _LINUX_MMC_SUNXI_MMC_PANIC_H
#define _LINUX_MMC_SUNXI_MMC_PANIC_H

#include <linux/types.h>

struct mmc_card;
struct mmc_host;

#define SUNXI_MMC_PANIC_SECTOR_SIZE	512U
#define SUNXI_MMC_PANIC_MAX_BYTES		4096U

/*
 * Normal process context, with a live card reference and no MMC host claim.
 * The caller reserves [start_sector, start_sector + nr_sectors) exclusively
 * for logs in the eMMC user area; both arguments use whole-disk sectors.
 * Keeps a card reference and a clock hold until unprepare. Returns 0/errno.
 * After set_ios, CMD6, card reset, voltage switch or suspend, unprepare and prepare
 * again in a stable running state. Serialize setup with power management too.
 */
int sunxi_mmc_panic_prepare(struct mmc_card *card, u32 start_sector,
			    u32 nr_sectors);

/*
 * Fatal panic only: local IRQs disabled and other CPUs stopped. Never resume
 * normal MMC I/O afterwards. sector is relative to the prepared log area.
 * bytes must be 512-aligned, in [512, 4096]; buf need not be word-aligned.
 * Returns 0 only after all sectors and card-ready checks succeed, else errno.
 * Multiple sequential calls are allowed. An I/O failure disables later writes
 * until reboot; a failed call may already have written some of its sectors.
 */
int sunxi_mmc_panic_write(u32 sector, const void *buf, size_t bytes);

/*
 * Normal process context, no MMC host claim. Stop the backend's callbacks
 * before calling; serialize preparation with card/driver removal.
 * Waits for existing readers before releasing the card/clock/context.
 */
void sunxi_mmc_panic_unprepare(struct mmc_host *mmc);

#endif
