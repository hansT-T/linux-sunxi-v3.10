#!/usr/bin/env python3
"""Exercise the real sunxi_mmc_panic_write() orchestration with fake hardware.

Usage:
  python3 tools/testing/sunxi-mmc-panic-interface.py --source /path/to/sunxi-mmc.c

The C function is extracted from the supplied source, never reimplemented here.
This checks its public contract and failure handling, not register sequencing,
DMA completion, timing, electrical behavior, or persistence on a real board.
A native C compiler is sufficient; no Linux cross compiler is needed.
"""

import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


def extract_function(source, name):
    match = re.search(r"^int\s+" + re.escape(name) + r"\s*\(", source, re.M)
    if not match:
        raise ValueError(f"cannot find definition of {name}")
    start = source.index("{", match.end())
    # Ignore braces within comments and quoted text when counting C blocks.
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.S)
    depth = 0
    for token in tokens.finditer(source, start):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():token.end()]
    raise ValueError(f"unterminated function {name}")


PREAMBLE = r'''
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

typedef uint8_t u8;
typedef uint32_t u32;
/* PUBLIC_CONSTANTS */
#define ACCESS_ONCE(x) (x)
#define REQUIRE(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #expr); exit(1); \
} } while (0)

#ifdef CONFIG_ARM_ARCH_TIMER
typedef struct { int value; } atomic_t;
struct sunxi_mmc_host {
    int lock;
    void *mrq_busy, *mrq_retry, *manual_stop_mrq, *mrq;
    bool wait_dma;
};
struct sunxi_mmc_panic_context {
    struct sunxi_mmc_host *host;
    u32 start_sector, nr_sectors, status_arg;
    atomic_t writing;
    bool taken_over;
    int error;
};
static struct sunxi_mmc_host host;
static struct sunxi_mmc_panic_context context;
static struct sunxi_mmc_panic_context *sunxi_mmc_panic_ctx;
static int rcu_depth, irq_disabled = 1, card_ret, takeover_ret;
static int check_calls, takeover_calls, sector_calls;
static int fail_sector_call, nested_ret;
static bool inject_nested;
static u32 written_lba[32];
static const void *written_buf[32];
static unsigned char payload[8193];
static int sunxi_mmc_panic_write_sector(struct sunxi_mmc_host *, u32, u32, const void *);
int sunxi_mmc_panic_write(u32, const void *, size_t);

static int irqs_disabled(void) { return irq_disabled; }
static void rcu_read_lock(void) { ++rcu_depth; }
static void rcu_read_unlock(void) { REQUIRE(rcu_depth > 0); --rcu_depth; }
#define rcu_dereference(p) (p)
static int atomic_cmpxchg(atomic_t *p, int old, int value)
{
    int previous = p->value;
    if (previous == old) p->value = value;
    return previous;
}
static void atomic_set(atomic_t *p, int value) { p->value = value; }
static void smp_mb(void) {}
static int spin_is_locked(const int *lock) { return *lock; }
static int sunxi_mmc_panic_check_card(struct sunxi_mmc_panic_context *ctx)
{
    REQUIRE(ctx == &context);
    REQUIRE(ctx->writing.value == 1 && rcu_depth > 0);
    ++check_calls;
    return card_ret;
}
static int sunxi_mmc_panic_takeover(struct sunxi_mmc_host *h, u32 arg)
{
    REQUIRE(h == &host && arg == context.status_arg);
    ++takeover_calls;
    return takeover_ret;
}
static int sunxi_mmc_panic_write_sector(struct sunxi_mmc_host *h, u32 arg,
                                      u32 lba, const void *data)
{
    REQUIRE(h == &host && arg == context.status_arg);
    REQUIRE(context.writing.value == 1 && rcu_depth > 0);
    REQUIRE(sector_calls < 32);
    written_lba[sector_calls] = lba;
    written_buf[sector_calls] = data;
    ++sector_calls;
    if (inject_nested) {
        inject_nested = false;
        nested_ret = sunxi_mmc_panic_write(0, payload, 512);
        REQUIRE(context.writing.value == 1);
    }
    return sector_calls == fail_sector_call ? -EIO : 0;
}
#endif
'''

TESTS = r'''
#ifdef CONFIG_ARM_ARCH_TIMER
static void reset_case(void)
{
    REQUIRE(rcu_depth == 0);
    memset(&host, 0, sizeof(host));
    memset(&context, 0, sizeof(context));
    memset(written_lba, 0, sizeof(written_lba));
    memset(written_buf, 0, sizeof(written_buf));
    context.host = &host;
    context.start_sector = 1000;
    context.nr_sectors = 32;
    context.status_arg = 1U << 16;
    sunxi_mmc_panic_ctx = &context;
    irq_disabled = 1;
    card_ret = takeover_ret = check_calls = takeover_calls = sector_calls = 0;
    fail_sector_call = nested_ret = 0;
    inject_nested = false;
}
static int write_checked(u32 sector, const void *buf, size_t bytes)
{
    int ret = sunxi_mmc_panic_write(sector, buf, bytes);
    REQUIRE(rcu_depth == 0);
    REQUIRE(context.writing.value == 0);
    return ret;
}
static void no_hardware(void)
{
    REQUIRE(check_calls == 0 && takeover_calls == 0);
    REQUIRE(sector_calls == 0);
}
static void test_input_contract(void)
{
    reset_case();
    REQUIRE(write_checked(0, NULL, 512) == -EINVAL);
    REQUIRE(write_checked(0, payload, 0) == -EINVAL);
    REQUIRE(write_checked(0, payload, 511) == -EINVAL);
    REQUIRE(write_checked(0, payload, 513) == -EINVAL);
    REQUIRE(write_checked(0, payload, 4097) == -EINVAL);
    REQUIRE(write_checked(0, payload, 4608) == -EINVAL);
    REQUIRE(write_checked(0, payload, SIZE_MAX) == -EINVAL);
    no_hardware();
    irq_disabled = 0;
    REQUIRE(write_checked(0, payload, 512) == -EPERM);
    no_hardware();
    irq_disabled = 1;
    sunxi_mmc_panic_ctx = NULL;
    REQUIRE(write_checked(0, payload, 512) == -ENODEV);
    no_hardware();
    puts("PASS invalid sizes, IRQ requirement, missing preparation");
}
static void test_range(void)
{
    reset_case();
    REQUIRE(write_checked(32, payload, 512) == -ERANGE);
    REQUIRE(write_checked(31, payload, 1024) == -ERANGE);
    REQUIRE(write_checked(UINT32_MAX, payload, 512) == -ERANGE);
    no_hardware();
    REQUIRE(context.error == 0);
    REQUIRE(write_checked(31, payload, 512) == 0);
    REQUIRE(sector_calls == 1 && written_lba[0] == 1031);

    reset_case();
    /* Valid prepared region ending just below the u32 address limit. */
    context.start_sector = UINT32_MAX - 16;
    context.nr_sectors = 16;
    REQUIRE(write_checked(15, payload, 1024) == -ERANGE);
    no_hardware();
    REQUIRE(write_checked(15, payload, 512) == 0);
    REQUIRE(written_lba[0] == UINT32_MAX - 1);

    reset_case();
    context.start_sector = 0;
    context.nr_sectors = UINT32_MAX;
    REQUIRE(write_checked(UINT32_MAX - 2, payload, 1536) == -ERANGE);
    no_hardware();
    REQUIRE(write_checked(UINT32_MAX - 1, payload, 512) == 0);
    REQUIRE(written_lba[0] == UINT32_MAX - 1);
    puts("PASS region boundaries and overflow-resistant range rejection");
}
static void test_success_and_reuse(void)
{
    int i;
    reset_case();
    host.mrq = &host;
    REQUIRE(write_checked(3, payload + 1, 4096) == 0);
    REQUIRE(sector_calls == 8 && takeover_calls == 1);
    for (i = 0; i < 8; ++i) {
        REQUIRE(written_lba[i] == (u32)(1003 + i));
        REQUIRE(written_buf[i] == payload + 1 + i * 512);
    }
    REQUIRE(context.taken_over);
    REQUIRE(write_checked(12, payload, 512) == 0);
    REQUIRE(sector_calls == 9 && takeover_calls == 1);
    REQUIRE(written_lba[8] == 1012 && check_calls == 2);

    reset_case();
    REQUIRE(write_checked(0, payload, 512) == 0);
    REQUIRE(takeover_calls == 1 && context.taken_over);

    reset_case();
    host.wait_dma = true;
    REQUIRE(write_checked(0, payload, 512) == 0);
    REQUIRE(takeover_calls == 1);
    puts("PASS 512/4096 bytes, unaligned buffer, sequential writes, one takeover");
}
static void test_failure_fuse(void)
{
    reset_case();
    fail_sector_call = 2;
    REQUIRE(write_checked(0, payload, 4096) == -EIO);
    REQUIRE(sector_calls == 2 && context.error == -EIO);
    fail_sector_call = 0;
    REQUIRE(write_checked(8, payload, 512) == -EIO);
    REQUIRE(sector_calls == 2 && check_calls == 1);

    reset_case();
    host.mrq = &host;
    takeover_ret = -ETIMEDOUT;
    REQUIRE(write_checked(0, payload, 512) == -ETIMEDOUT);
    REQUIRE(sector_calls == 0 && takeover_calls == 1 && !context.taken_over);
    takeover_ret = 0;
    REQUIRE(write_checked(1, payload, 512) == -ETIMEDOUT);
    REQUIRE(takeover_calls == 1);

    reset_case();
    card_ret = -EHOSTDOWN;
    REQUIRE(write_checked(0, payload, 512) == -EHOSTDOWN);
    REQUIRE(sector_calls == 0);
    card_ret = 0;
    REQUIRE(write_checked(0, payload, 512) == -EHOSTDOWN);
    REQUIRE(check_calls == 1);
    puts("PASS partial failure stops writes and latches errors without retry");
}
static void test_busy_and_reentrancy(void)
{
    int which;
    for (which = 0; which < 4; ++which) {
        reset_case();
        if (which == 0) host.lock = 1;
        if (which == 1) host.mrq_busy = &host;
        if (which == 2) host.mrq_retry = &host;
        if (which == 3) host.manual_stop_mrq = &host;
        REQUIRE(write_checked(0, payload, 512) == -EBUSY);
        REQUIRE(sector_calls == 0 && context.error == -EBUSY);
    }
    reset_case();
    context.writing.value = 1;
    REQUIRE(sunxi_mmc_panic_write(0, payload, 512) == -EBUSY);
    REQUIRE(rcu_depth == 0 && context.writing.value == 1 && context.error == 0);
    no_hardware();
    context.writing.value = 0;
    REQUIRE(write_checked(0, payload, 512) == 0);

    reset_case();
    inject_nested = true;
    REQUIRE(write_checked(0, payload, 1024) == 0);
    REQUIRE(nested_ret == -EBUSY && sector_calls == 2 && context.error == 0);
    puts("PASS busy host rejection and nested writer exclusion");
}
int main(void)
{
    test_input_contract();
    test_range();
    test_success_and_reuse();
    test_failure_fuse();
    test_busy_and_reentrancy();
    return 0;
}
#else
int main(void)
{
    unsigned char buf[512];
    REQUIRE(sunxi_mmc_panic_write(0, buf, sizeof(buf)) == -EOPNOTSUPP);
    REQUIRE(sunxi_mmc_panic_write(UINT32_MAX, NULL, SIZE_MAX) == -EOPNOTSUPP);
    puts("PASS CONFIG_ARM_ARCH_TIMER disabled");
    return 0;
}
#endif
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path,
                        help="sunxi-mmc.c containing the actual implementation")
    parser.add_argument("--header", type=Path,
                        help="public header; defaults to the source tree's include/linux/mmc/sunxi-mmc-panic.h")
    args = parser.parse_args()
    header = args.header or (args.source.resolve().parents[3] /
                             "include/linux/mmc/sunxi-mmc-panic.h")
    header_text = header.read_text()
    constants = []
    for name in ("SUNXI_MMC_PANIC_SECTOR_SIZE", "SUNXI_MMC_PANIC_MAX_BYTES"):
        match = re.search(r"^#define\s+" + name + r"\s+[^\n]+", header_text, re.M)
        if not match:
            raise ValueError(f"cannot find {name} in {header}")
        constants.append(match.group())
    preamble = PREAMBLE.replace("/* PUBLIC_CONSTANTS */", "\n".join(constants))
    function = extract_function(args.source.read_text(errors="replace"),
                                "sunxi_mmc_panic_write")
    compiler = shlex.split(os.environ.get("CC", "clang"))
    with tempfile.TemporaryDirectory(prefix="sunxi-panic-interface-") as work:
        work = Path(work)
        c_file = work / "write-contract.c"
        c_file.write_text(preamble + "\n" + function + "\n" + TESTS)
        for timer in (True, False):
            executable = work / ("timer-on" if timer else "timer-off")
            cmd = compiler + ["-std=c99", "-Wall", "-Wextra", "-Werror",
                              "-Wno-unused-parameter", "-O1", "-g",
                              "-fsanitize=address,undefined"]
            if timer:
                cmd.append("-DCONFIG_ARM_ARCH_TIMER=1")
            subprocess.run(cmd + [str(c_file), "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)
    print("All write contract checks passed; hardware behavior is mocked.")


if __name__ == "__main__":
    main()
