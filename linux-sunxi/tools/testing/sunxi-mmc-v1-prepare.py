#!/usr/bin/env python3
"""Compile the actual v1 preparation code with a native C lifecycle model.

Run with Python 3 and clang (or CC). This exercises discovery, retained device
and clock references, failure cleanup, invalidation and work cancellation.
No board is accessed. This is not an ARM kernel build, an SMP proof or a test
of actual MMC timing and power management.
"""

import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
DRIVER = ROOT / "drivers/mmc/host/sunxi-mmc-v1.c"


def kernel_defines(path, prefixes):
    source = path.read_text(errors="replace").replace("\\\n", " ")
    return "\n".join(
        line for line in source.splitlines()
        if any(re.match(r"#define\s+" + prefix, line) for prefix in prefixes)
    )


def preparation_code():
    source = DRIVER.read_text(errors="replace")
    start = source.index("/* T3 panic context preparation. */")
    end = source.index("/* End of T3 panic preparation. */", start)
    return source[start:end]


PREAMBLE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;
#define BIT(n) (1U << (n))
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define container_of(p, type, member) ((type *)((char *)(p) - offsetof(type, member)))
#define ACCESS_ONCE(x) (x)
#define cpu_relax() do { } while (0)
#define smp_wmb() do { } while (0)
#define smp_rmb() do { } while (0)
#define smp_mb() do { } while (0)
#define REQUIRE(x) do { if (!(x)) { \
    fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #x); assert(x); \
} } while (0)
static int lock_depth;
typedef int spinlock_t;
#define DEFINE_SPINLOCK(name) spinlock_t name
#define spin_lock_irqsave(lock, flags) do { (void)(lock); (flags) = 0; \
    REQUIRE(lock_depth == 0); ++lock_depth; } while (0)
#define spin_unlock_irqrestore(lock, flags) do { (void)(lock); (void)(flags); \
    REQUIRE(lock_depth == 1); --lock_depth; } while (0)
struct device_node { bool compatible; };
struct bus_type { const char *name; };
struct device { struct bus_type *bus; struct device_node *of_node; int refs; };
struct work_struct { void (*func)(struct work_struct *); };
struct delayed_work { struct work_struct work; bool queued; unsigned long delay; };
#define INIT_DELAYED_WORK(dw, fn) do { (dw)->work.func = (fn); \
    (dw)->queued = false; } while (0)
#define to_delayed_work(w) container_of(w, struct delayed_work, work)
static int system_queue;
#define system_wq (&system_queue)
static unsigned int scheduled, cancelled;
static unsigned long msecs_to_jiffies(unsigned int value) { return value; }
static bool mod_delayed_work(void *queue, struct delayed_work *dw, unsigned long delay)
{
    bool was_queued = dw->queued;
    (void)queue;
    ++scheduled;
    dw->queued = true;
    dw->delay = delay;
    return was_queued;
}
static bool schedule_delayed_work(struct delayed_work *dw, unsigned long delay)
{
    return mod_delayed_work(system_wq, dw, delay);
}
static bool cancel_delayed_work(struct delayed_work *dw)
{
    bool was_queued = dw->queued;
    ++cancelled;
    dw->queued = false;
    return was_queued;
}
static bool cancel_delayed_work_sync(struct delayed_work *dw)
{
    REQUIRE(lock_depth == 0);
    return cancel_delayed_work(dw);
}
struct mmc_card;
struct mmc_ios { u32 clock, bus_width, timing, signal_voltage, power_mode; };
struct mmc_host {
    struct mmc_card *card;
    struct mmc_ios ios;
    struct device class_dev;
    struct device *parent;
    bool clk_gated;
};
struct mmc_ext_csd { u32 sectors, data_sector_size, part_config, cache_ctrl; };
struct mmc_card {
    struct device dev;
    struct mmc_host *host;
    struct mmc_ext_csd ext_csd;
    u32 rca, state;
    bool is_mmc, blockaddr, removed, present;
};
struct sunxi_mmc_host { struct mmc_host *mmc; int ferror, phy_index; };
struct mmc_command { u32 opcode, arg, flags, resp[4]; int error; };
#define mmc_card_mmc(card) ((card)->is_mmc)
#define mmc_card_blockaddr(card) ((card)->blockaddr)
#define mmc_card_removed(card) ((card)->removed)
#define mmc_card_present(card) ((card)->present)
#define mmc_classdev(mmc) (&(mmc)->class_dev)
#define mmc_dev(mmc) ((mmc)->parent)
#define mmc_dev_to_card(ptr) container_of(ptr, struct mmc_card, dev)
static struct bus_type mmc_bus = { "mmc" };
static struct device_node node = { true };
static struct device parent_device;
static struct mmc_host mmc;
static struct mmc_card card;
static struct sunxi_mmc_host host;
static bool child_available;
static int claims, clock_refs, clock_holds, clock_releases, command_calls;
static int command_ret, info_logs, warn_logs, debug_logs;
static u32 command_r1, timer_rate;
static u64 timer_ticks, timer_step;
static void (*hold_hook)(void), (*command_hook)(void), (*release_hook)(void);
static bool of_device_is_compatible(struct device_node *np, const char *compat)
{
    (void)compat;
    return np && np->compatible;
}
static struct device *device_find_child(struct device *parent, void *data,
                                       int (*match)(struct device *, void *))
{
    REQUIRE(parent == &mmc.class_dev);
    if (!child_available || !match(&card.dev, data))
        return NULL;
    ++card.dev.refs;
    return &card.dev;
}
static struct device *get_device(struct device *dev) { ++dev->refs; return dev; }
static void put_device(struct device *dev) { REQUIRE(dev->refs > 1); --dev->refs; }
static void mmc_claim_host(struct mmc_host *m)
{
    REQUIRE(lock_depth == 0 && m == &mmc && claims == 0);
    ++claims;
}
static void mmc_release_host(struct mmc_host *m)
{
    REQUIRE(lock_depth == 0 && m == &mmc && claims == 1);
    --claims;
}
static void mmc_host_clk_hold(struct mmc_host *m)
{
    REQUIRE(lock_depth == 0 && m == &mmc);
    ++clock_refs;
    ++clock_holds;
    if (hold_hook)
        hold_hook();
}
static void mmc_host_clk_release(struct mmc_host *m)
{
    REQUIRE(lock_depth == 0 && m == &mmc && clock_refs > 0);
    --clock_refs;
    ++clock_releases;
    if (release_hook)
        release_hook();
}
static u32 arch_timer_get_rate(void) { return timer_rate; }
static u64 mock_counter(void) { timer_ticks += timer_step; return timer_ticks; }
static u64 (*arch_timer_read_counter)(void) = mock_counter;
static int mmc_wait_for_cmd(struct mmc_host *m, struct mmc_command *cmd, int retries)
{
    REQUIRE(lock_depth == 0 && m == &mmc && claims == 1);
    REQUIRE(retries == 0);
    REQUIRE(cmd->arg == (card.rca << 16));
    ++command_calls;
    cmd->resp[0] = command_r1;
    if (command_hook)
        command_hook();
    return command_ret;
}
#define dev_info(dev, ...) do { (void)(dev); ++info_logs; } while (0)
#define dev_warn(dev, ...) do { (void)(dev); ++warn_logs; } while (0)
#define dev_dbg(dev, ...) do { (void)(dev); ++debug_logs; } while (0)
'''

TESTS = r'''
#ifdef CONFIG_ARM_ARCH_TIMER
static void setup(void)
{
    memset(&mmc, 0, sizeof(mmc));
    memset(&card, 0, sizeof(card));
    memset(&host, 0, sizeof(host));
    memset(&sunxi_mmc_panic_ctx, 0, sizeof(sunxi_mmc_panic_ctx));
    lock_depth = 0;
    parent_device.of_node = &node;
    mmc.parent = &parent_device;
    mmc.card = &card;
    mmc.ios.clock = 50000000;
    mmc.ios.power_mode = MMC_POWER_ON;
    mmc.ios.timing = MMC_TIMING_MMC_HS;
    card.host = &mmc;
    card.dev.bus = &mmc_bus;
    card.dev.refs = 1;
    card.rca = 1;
    card.is_mmc = card.blockaddr = card.present = true;
    card.ext_csd.sectors = 16000000U;
    card.ext_csd.data_sector_size = 512;
    host.mmc = &mmc;
    host.phy_index = 2;
    child_available = true;
    node.compatible = true;
    claims = clock_refs = clock_holds = clock_releases = command_calls = 0;
    scheduled = cancelled = 0;
    command_ret = info_logs = warn_logs = debug_logs = 0;
    command_r1 = R1_READY_FOR_DATA | (R1_STATE_TRAN << 9);
    timer_rate = 24000000;
    timer_ticks = 0;
    timer_step = 1;
    arch_timer_read_counter = mock_counter;
    hold_hook = command_hook = release_hook = NULL;
    sunxi_mmc_panic_start(&host);
}

static void run_work(void)
{
    struct delayed_work *dw = &sunxi_mmc_panic_ctx.prepare_work;
    REQUIRE(dw->queued && dw->work.func);
    dw->queued = false;
    dw->work.func(&dw->work);
    REQUIRE(lock_depth == 0 && claims == 0);
}

static void stopped_balanced(void)
{
    sunxi_mmc_panic_stop(&host);
    REQUIRE(!sunxi_mmc_panic_ctx.ready);
    REQUIRE(!sunxi_mmc_panic_ctx.prepare_work.queued);
    REQUIRE(sunxi_mmc_panic_ctx.card == NULL);
    REQUIRE(card.dev.refs == 1 && clock_refs == 0 && claims == 0);
    REQUIRE(clock_holds == clock_releases);
}

static void test_discovery_and_success(void)
{
    unsigned int writes;
    setup();
    child_available = false;
    run_work();
    REQUIRE(!sunxi_mmc_panic_ctx.ready);
    REQUIRE(sunxi_mmc_panic_ctx.prepare_work.queued);
    REQUIRE(card.dev.refs == 1 && command_calls == 0 && clock_refs == 0);
    child_available = true;
    card.present = false;
    run_work();
    REQUIRE(!sunxi_mmc_panic_ctx.ready && command_calls == 0);
    REQUIRE(card.dev.refs == 1 && clock_refs == 0);
    card.present = true;
    run_work();
    REQUIRE(sunxi_mmc_panic_ctx.ready);
    REQUIRE(sunxi_mmc_panic_ctx.host == &host);
    REQUIRE(sunxi_mmc_panic_ctx.card == &card);
    REQUIRE(card.dev.refs == 2 && clock_refs == 1 && command_calls == 1);
    writes = command_calls;
    /* A stale self-queued work must not duplicate the successful preparation. */
    sunxi_mmc_panic_ctx.prepare_work.queued = true;
    run_work();
    REQUIRE(command_calls == (int)writes);
    REQUIRE(card.dev.refs == 2 && clock_refs == 1);
    stopped_balanced();
    puts("PASS card discovery retries, ready publication and retained references");
}

static void test_failure_cleanup(void)
{
    int variant;
    for (variant = 0; variant < 12; ++variant) {
        setup();
        switch (variant) {
        case 0: card.ext_csd.sectors = SUNXI_PSTORE_START_SECTOR; break;
        case 1: card.ext_csd.sectors = SUNXI_PSTORE_START_SECTOR + SUNXI_PSTORE_NR_SECTORS - 1; break;
        case 2: card.ext_csd.cache_ctrl = 1; break;
        case 3: card.ext_csd.part_config = 1; break;
        case 4: card.ext_csd.data_sector_size = 4096; break;
        case 5: card.blockaddr = false; break;
        case 6: timer_rate = 0; break;
        case 7: arch_timer_read_counter = NULL; break;
        case 8: timer_step = 0; break;
        case 9: command_ret = -ETIMEDOUT; break;
        case 10: command_r1 |= R1_ERROR; break;
        case 11: command_r1 = R1_STATE_PRG << 9; break;
        }
        run_work();
        REQUIRE(!sunxi_mmc_panic_ctx.ready);
        REQUIRE(sunxi_mmc_panic_ctx.card == NULL);
        REQUIRE(card.dev.refs == 1 && clock_refs == 0 && claims == 0);
        REQUIRE(clock_holds == clock_releases);
        stopped_balanced();
    }
    puts("PASS geometry, unsupported card, timer and CMD13 failure cleanup");
}

static void invalidate_current(void)
{
    sunxi_mmc_panic_invalidate(&host);
}

static void test_reprepare_and_generation(void)
{
    setup();
    run_work();
    sunxi_mmc_panic_invalidate(&host);
    REQUIRE(!sunxi_mmc_panic_ctx.ready);
    REQUIRE(sunxi_mmc_panic_ctx.prepare_work.queued);
    run_work();
    REQUIRE(sunxi_mmc_panic_ctx.ready && command_calls == 2);
    REQUIRE(card.dev.refs == 2 && clock_refs == 1);
    stopped_balanced();

    setup();
    command_hook = invalidate_current;
    run_work();
    REQUIRE(!sunxi_mmc_panic_ctx.ready);
    REQUIRE(card.dev.refs == 1 && clock_refs == 0);
    REQUIRE(sunxi_mmc_panic_ctx.prepare_work.queued);
    command_hook = NULL;
    run_work();
    REQUIRE(sunxi_mmc_panic_ctx.ready);
    stopped_balanced();

    /* Core clock ungating itself can call set_ios: accept that new generation. */
    setup();
    hold_hook = invalidate_current;
    run_work();
    REQUIRE(sunxi_mmc_panic_ctx.ready);
    hold_hook = NULL;
    stopped_balanced();
    puts("PASS invalidation, generation race rejection and clock-ungating reprepare");
}

static void gate_after_release(void)
{
    mmc.ios.clock = 0;
    sunxi_mmc_panic_invalidate(&host);
}

static void test_clock_gate_feedback(void)
{
    setup();
    hold_hook = invalidate_current;
    release_hook = gate_after_release;
    command_r1 |= R1_ERROR;
    run_work();
    REQUIRE(!sunxi_mmc_panic_ctx.ready);
    REQUIRE(!sunxi_mmc_panic_ctx.prepare_work.queued);
    REQUIRE(card.dev.refs == 1 && clock_refs == 0);

    /* A later real nonzero-clock change must restart preparation. */
    mmc.ios.clock = 50000000;
    sunxi_mmc_panic_invalidate(&host);
    REQUIRE(sunxi_mmc_panic_ctx.prepare_work.queued);
    hold_hook = release_hook = NULL;
    command_r1 = R1_READY_FOR_DATA | (R1_STATE_TRAN << 9);
    run_work();
    REQUIRE(sunxi_mmc_panic_ctx.ready);
    stopped_balanced();
    puts("PASS failed prepare clock-gating feedback and later configuration retry");
}

static void test_pause_resume_stop(void)
{
    unsigned int before;
    setup();
    run_work();
    sunxi_mmc_panic_pause(&host);
    REQUIRE(!sunxi_mmc_panic_ctx.ready && sunxi_mmc_panic_ctx.paused);
    REQUIRE(!sunxi_mmc_panic_ctx.prepare_work.queued);
    REQUIRE(card.dev.refs == 1 && clock_refs == 0);
    before = scheduled;
    sunxi_mmc_panic_invalidate(&host);
    REQUIRE(scheduled == before && !sunxi_mmc_panic_ctx.prepare_work.queued);
    sunxi_mmc_panic_resume(&host);
    REQUIRE(sunxi_mmc_panic_ctx.prepare_work.queued);
    run_work();
    REQUIRE(sunxi_mmc_panic_ctx.ready);
    stopped_balanced();
    before = scheduled;
    sunxi_mmc_panic_invalidate(&host);
    sunxi_mmc_panic_resume(&host);
    REQUIRE(scheduled == before && !sunxi_mmc_panic_ctx.prepare_work.queued);
    puts("PASS suspend cancellation, resume preparation and final stop");
}

int main(void)
{
    test_discovery_and_success();
    test_failure_cleanup();
    test_reprepare_and_generation();
    test_pause_resume_stop();
    test_clock_gate_feedback();
    return 0;
}
#else
int main(void)
{
    memset(&host, 0, sizeof(host));
    host.mmc = &mmc;
    sunxi_mmc_panic_start(&host);
    sunxi_mmc_panic_invalidate(&host);
    sunxi_mmc_panic_pause(&host);
    sunxi_mmc_panic_resume(&host);
    sunxi_mmc_panic_stop(&host);
    REQUIRE(command_calls == 0 && clock_refs == 0 && scheduled == 0);
    puts("PASS CONFIG_ARM_ARCH_TIMER disabled stubs");
    return 0;
}
#endif
'''


def main():
    macros = "\n".join([
        kernel_defines(ROOT / "include/linux/mmc/mmc.h", ("MMC_", "R1_", "EXT_CSD_")),
        kernel_defines(ROOT / "include/linux/mmc/core.h", ("MMC_",)),
        kernel_defines(ROOT / "include/linux/mmc/host.h", ("MMC_POWER_", "MMC_TIMING_")),
    ])
    program = PREAMBLE + "\n" + macros + "\n" + preparation_code() + "\n" + TESTS
    compiler = shlex.split(os.environ.get("CC", "clang"))
    with tempfile.TemporaryDirectory(prefix="sunxi-v1-prepare-") as work:
        work = Path(work)
        source = work / "prepare.c"
        source.write_text(program)
        for timer in (True, False):
            output = work / ("timer-on" if timer else "timer-off")
            flags = ["-std=gnu89", "-Wall", "-Wextra", "-Werror",
                     "-Wno-unused-function", "-Wno-unused-variable",
                     "-Wno-unused-parameter", "-O1", "-g",
                     "-fsanitize=address,undefined", "-fno-strict-overflow"]
            if timer:
                flags += ["-DCONFIG_ARM_ARCH_TIMER=1", "-DCONFIG_MMC_CLKGATE=1"]
            subprocess.run(compiler + flags + [str(source), "-o", str(output)], check=True)
            subprocess.run([str(output)], check=True)
    print("All v1 prepare checks passed (mock lifecycle, not board validation).")


if __name__ == "__main__":
    main()
