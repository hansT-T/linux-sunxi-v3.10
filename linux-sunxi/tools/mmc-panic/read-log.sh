#!/bin/sh
# SPDX-License-Identifier: GPL-2.0

fail()
{
	printf '%s: %s\n' "${0##*/}" "$*" >&2
	exit 1
}

[ "$#" -eq 1 ] || fail "usage: $0 <log-partition-or-image>"
partition=$1
command -v gzip >/dev/null 2>&1 || fail "gzip is required for CRC32"
tmp=$(mktemp -d "${TMPDIR:-/tmp}/sunxi-panic.XXXXXX") || exit 1
trap 'rm -rf "$tmp"' 0
trap 'exit 1' HUP INT TERM

header_word()
{
	od -An -v -tx1 -j "$1" -N 4 "$tmp/header" | tr -d '[:space:]'
}

crc32()
{
	# A gzip trailer contains the IEEE CRC32 followed by the input size.
	gzip -c < "$1" > "$tmp/crc.gz" || return 1
	tail -c 8 "$tmp/crc.gz" | od -An -v -tx1 -N 4 | tr -d '[:space:]'
}

dd if="$partition" of="$tmp/header" bs=32 count=1 2>/dev/null ||
	fail "cannot read header from $partition"
[ "$(wc -c < "$tmp/header")" -eq 32 ] || fail "short header"
[ "$(header_word 0)" = 504d4d43 ] || fail "invalid header magic"
[ "$(header_word 4)" = 01000000 ] || fail "unsupported format version"
[ "$(header_word 8)" = 20000000 ] || fail "invalid header length"
[ "$(header_word 12)" = 02000000 ] || fail "no committed log"

set -- $(od -An -v -tu1 -j 16 -N 4 "$tmp/header")
[ "$#" -eq 4 ] && [ "$3" -eq 0 ] && [ "$4" -eq 0 ] ||
	fail "invalid payload length"
length=$(($1 + 256 * $2 + 65536 * $3))
[ "$length" -ge 1 ] && [ "$length" -le 131072 ] || fail "invalid payload length"

payload_crc=$(header_word 20)
header_crc=$(header_word 24)
dd if=/dev/zero of="$tmp/header" bs=1 seek=24 count=4 conv=notrunc 2>/dev/null ||
	fail "cannot prepare header CRC input"
actual_crc=$(crc32 "$tmp/header") || fail "cannot compute header CRC32"
[ "$actual_crc" = "$header_crc" ] || fail "header CRC mismatch"

dd if="$partition" of="$tmp/payload" bs=1 skip=512 count="$length" 2>/dev/null ||
	fail "cannot read payload from $partition"
[ "$(wc -c < "$tmp/payload")" -eq "$length" ] || fail "short payload"
actual_crc=$(crc32 "$tmp/payload") || fail "cannot compute payload CRC32"
[ "$actual_crc" = "$payload_crc" ] || fail "payload CRC mismatch"

cat "$tmp/payload"
