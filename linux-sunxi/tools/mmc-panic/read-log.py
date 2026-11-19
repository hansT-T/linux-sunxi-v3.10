#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0

import argparse
import struct
import sys
import zlib


SECTOR_SIZE = 512
SLOT_SECTORS = 512
SLOT_COUNT = 64
MAX_PAYLOAD_SIZE = 128 * 1024
MAGIC = 0x434D4D50
VERSION_WITH_LEGACY_CRC = 1
VERSION = 2
COMMITTED = 2
HEADER = struct.Struct("<8I")
HEADER_CRC_OFFSET = 24


def read_log(path, slot=0):
    if not 0 <= slot < SLOT_COUNT:
        raise ValueError("slot must be between 0 and {}".format(SLOT_COUNT - 1))

    with open(path, "rb") as partition:
        partition.seek(slot * SLOT_SECTORS * SECTOR_SIZE)
        header = bytearray(partition.read(SECTOR_SIZE))
        if len(header) != SECTOR_SIZE:
            raise ValueError("short panic log header")

        magic, version, header_len, state, length, payload_crc, header_crc, _ = (
            HEADER.unpack_from(header)
        )
        if (magic, header_len) != (MAGIC, HEADER.size) or version not in (
            VERSION_WITH_LEGACY_CRC, VERSION
        ):
            raise ValueError("unrecognized panic log header")
        if state != COMMITTED:
            raise ValueError("no committed panic log")
        if not 0 < length <= MAX_PAYLOAD_SIZE:
            raise ValueError("invalid panic log length")

        struct.pack_into("<I", header, HEADER_CRC_OFFSET, 0)
        if zlib.crc32(header[:header_len]) != header_crc:
            if version != VERSION_WITH_LEGACY_CRC or zlib.crc32(header) != header_crc:
                raise ValueError("panic log header checksum mismatch")

        payload = partition.read(length)
        if len(payload) != length or zlib.crc32(payload) != payload_crc:
            raise ValueError("incomplete or corrupt panic log")
        return payload


def main():
    parser = argparse.ArgumentParser(
        description="Read a committed Sunxi eMMC panic log and write it to stdout."
    )
    parser.add_argument("partition", help="dedicated log partition or its raw image")
    parser.add_argument("slot", nargs="?", type=int, default=0,
                        help="slot number (0-{}, default: 0)".format(SLOT_COUNT - 1))
    args = parser.parse_args()

    try:
        payload = read_log(args.partition, args.slot)
        sys.stdout.buffer.write(payload)
    except (OSError, ValueError) as error:
        parser.exit(1, "{}: {}\n".format(parser.prog, error))


if __name__ == "__main__":
    main()
