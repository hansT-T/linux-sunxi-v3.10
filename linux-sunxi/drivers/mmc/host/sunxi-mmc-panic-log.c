/* SPDX-License-Identifier: GPL-2.0 */
/* Single-record printk dumper for the T3 eMMC user area. */

#include <linux/blkdev.h>
#include <linux/cpumask.h>
#include <linux/crc32.h>
#include <linux/irqflags.h>
#include <linux/kmsg_dump.h>
#include <linux/string.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sunxi-mmc-v1-panic.h>

#include <asm/byteorder.h>

#define SUNXI_PANIC_LOG_MAGIC		0x434d4d50U /* "PMMC" on disk */
#define SUNXI_PANIC_LOG_VERSION		2U
#define SUNXI_PANIC_LOG_WRITING		1U
#define SUNXI_PANIC_LOG_COMMITTED	2U
#define SUNXI_PANIC_LOG_PAYLOAD_MAX	4096U
#define SUNXI_PANIC_LOG_OPEN_MODE	(FMODE_READ | FMODE_EXCL)

/* On-disk layout shared with tools/mmc-panic/read-log.py. */
struct sunxi_panic_log_header {
	__le32 magic;
	__le32 version;
	__le32 header_len;
	__le32 state;
	__le32 payload_len;
	__le32 payload_crc;
	__le32 header_crc;
	__le32 reserved;
};

/* One sdc2 target. The dump callback uses only the two static buffers. */
static struct mmc_card *sunxi_panic_log_card;
static struct block_device *sunxi_panic_log_partition;
static struct sunxi_panic_log_header sunxi_panic_log_header;
static char sunxi_panic_log_payload[SUNXI_PANIC_LOG_PAYLOAD_MAX];

static u32 sunxi_panic_log_crc(const void *data, size_t len)
{
	return crc32_le(~0U, data, len) ^ ~0U;
}

static int sunxi_panic_log_write_header(u32 state)
{
	struct sunxi_panic_log_header *header = &sunxi_panic_log_header;
	u32 crc;

	header->state = cpu_to_le32(state);
	header->header_crc = 0;
	crc = sunxi_panic_log_crc(header, sizeof(*header));
	header->header_crc = cpu_to_le32(crc);
	return sunxi_mmc_panic_write(0, header, sizeof(*header));
}

static void sunxi_panic_log_dump(struct kmsg_dumper *dumper,
				enum kmsg_dump_reason reason)
{
	struct sunxi_panic_log_header *header = &sunxi_panic_log_header;
	size_t len;

	if (reason != KMSG_DUMP_PANIC || !irqs_disabled() ||
	    num_online_cpus() != 1)
		return;
	if (!kmsg_dump_get_buffer(dumper, true, sunxi_panic_log_payload,
				  sizeof(sunxi_panic_log_payload), &len) || !len)
		return;

	memset(header, 0, sizeof(*header));
	header->magic = cpu_to_le32(SUNXI_PANIC_LOG_MAGIC);
	header->version = cpu_to_le32(SUNXI_PANIC_LOG_VERSION);
	header->header_len = cpu_to_le32(sizeof(*header));
	header->payload_len = cpu_to_le32(len);
	header->payload_crc = cpu_to_le32(
		sunxi_panic_log_crc(sunxi_panic_log_payload, len));

	if (sunxi_panic_log_write_header(SUNXI_PANIC_LOG_WRITING))
		return;
	if (sunxi_mmc_panic_write(1, sunxi_panic_log_payload, len))
		return;
	sunxi_panic_log_write_header(SUNXI_PANIC_LOG_COMMITTED);
}

static struct kmsg_dumper sunxi_panic_log_dumper = {
	.dump = sunxi_panic_log_dump,
	.max_reason = KMSG_DUMP_PANIC,
};

void sunxi_mmc_panic_log_add(struct mmc_card *card, struct gendisk *disk)
{
	struct block_device *partition;
	int ret;

	if (!mmc_card_mmc(card) || !sunxi_mmc_v1_panic_target(card->host))
		return;

	partition = bdget_disk(disk, SUNXI_MMC_V1_LOG_PARTNO);
	if (!partition) {
		dev_warn(&card->dev, "panic log partition %d missing\n",
			 SUNXI_MMC_V1_LOG_PARTNO);
		return;
	}
	ret = blkdev_get(partition, SUNXI_PANIC_LOG_OPEN_MODE,
			 &sunxi_panic_log_dumper);
	if (ret) {
		dev_warn(&card->dev, "cannot claim panic log partition: %d\n", ret);
		return;
	}
	if (partition->bd_disk != disk ||
	    get_start_sect(partition) != SUNXI_PSTORE_START_SECTOR ||
	    part_nr_sects_read(partition->bd_part) != SUNXI_PSTORE_NR_SECTORS) {
		dev_warn(&card->dev, "panic log partition has unexpected geometry\n");
		goto release;
	}

	ret = kmsg_dump_register(&sunxi_panic_log_dumper);
	if (ret)
		goto release;
	sunxi_panic_log_card = card;
	sunxi_panic_log_partition = partition;
	dev_info(&card->dev, "panic logger registered for partition %d\n",
		 SUNXI_MMC_V1_LOG_PARTNO);
	return;

release:
	blkdev_put(partition, SUNXI_PANIC_LOG_OPEN_MODE);
}

void sunxi_mmc_panic_log_remove(struct mmc_card *card)
{
	if (card != sunxi_panic_log_card)
		return;

	kmsg_dump_unregister(&sunxi_panic_log_dumper);
	blkdev_put(sunxi_panic_log_partition, SUNXI_PANIC_LOG_OPEN_MODE);
	sunxi_panic_log_card = NULL;
}
