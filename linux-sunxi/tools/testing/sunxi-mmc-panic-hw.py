#!/usr/bin/env python3
"""Compile the real panic helpers against a tiny MMIO model, then smoke-test.

Run from any directory with Python 3 and a native C compiler (CC or cc).
This checks C/control-flow and register protocol assumptions, not real hardware
timing, kernel linkage, DMA coherency or SMP exclusion. No board writes occur.
"""

import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
DRIVER = ROOT / "drivers/mmc/host/sunxi-mmc.c"


def defines(path, prefixes):
    """Use register/command definitions from the target kernel, not copies."""
    text = path.read_text(encoding="utf-8", errors="replace")
    # A multi-line macro must remain one logical definition.
    text = text.replace("\\\n", " ")
    return "\n".join(
        line for line in text.splitlines()
        if any(re.match(r"#define\s+" + prefix, line) for prefix in prefixes)
    )


def hardware_helpers():
    source = DRIVER.read_text(encoding="utf-8", errors="replace")
    start = source.index("#define SUNXI_PANIC_FIFO_OFF")
    end = source.index("static int sunxi_mmc_panic_takeover(", start)
    brace = source.index("{", end)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
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
#define ACCESS_ONCE(x) (x)
#define cpu_relax() do { } while (0)
#define wmb() do { } while (0)
struct mmc_command { u32 opcode; };
struct mmc_data { u32 blksz, blocks, flags; struct mmc_command *stop; };
struct mmc_request {
    struct mmc_command *cmd, *sbc;
    struct mmc_data *data;
};
struct sunxi_mmc_host {
    u8 *reg_base;
    int ferror;
    struct mmc_request *mrq;
    u32 int_sum;
    bool wait_dma;
};
static u32 readl(const void *address);
static void writel(u32 value, void *address);
#define mmc_readl(host, reg) readl((host)->reg_base + SDXC_##reg)
#define mmc_writel(host, reg, value) writel(value, (host)->reg_base + SDXC_##reg)
static u64 ticks;
static u64 mock_counter(void) { ticks += 100; return ticks; }
static u64 (*arch_timer_read_counter)(void) = mock_counter;
static u32 arch_timer_get_rate(void) { return 1000000; }
'''

MODEL = r'''
static u32 regs[0x400 / 4], fifo[256], lbas[2];
static unsigned int fifo_words, sector_words, data_commands, stop_commands;
static unsigned int dma_resets, fifo_resets, status_commands;
static int cmd13_mode; /* 1: R1 failure, 2: no completion, 3: response timeout */
static bool write_pending;

static size_t offset_of(const void *address)
{
    return (const u8 *)address - (const u8 *)regs;
}

static u32 readl(const void *address)
{
    size_t offset = offset_of(address);
    assert(offset < sizeof(regs));
    return regs[offset / 4];
}

static void writel(u32 value, void *address)
{
    size_t offset = offset_of(address);
    assert(offset < sizeof(regs));
    if (offset == SDXC_REG_RINTR || offset == SDXC_REG_IDST) {
        regs[offset / 4] &= ~value;
        return;
    }
    if (offset == SDXC_REG_GCTRL) {
        if (value & SDXC_DMA_RESET) {
            dma_resets++;
            regs[SDXC_REG_IDST / 4] = 0;
        }
        if (value & SDXC_FIFO_RESET) {
            /* Writing FIFO reset while the card is busy violates the protocol. */
            assert(!(regs[SDXC_REG_STAS / 4] & SDXC_CARD_DATA_BUSY));
            fifo_resets++;
            regs[SDXC_REG_STAS / 4] = SDXC_FIFO_EMPTY;
        }
        value &= ~SDXC_HARDWARE_RESET;
    }
    if (offset == SDXC_REG_DMAC && (value & SDXC_IDMAC_SOFT_RESET)) {
        dma_resets++;
        value &= ~SDXC_IDMAC_SOFT_RESET;
    }
    regs[offset / 4] = value;
    if (offset == SDXC_REG_CMDR && (value & SDXC_START)) {
        u32 opcode = value & 0x3f;
        regs[offset / 4] &= ~SDXC_START;
        regs[SDXC_REG_RESP0 / 4] = R1_READY_FOR_DATA | (R1_STATE_TRAN << 9);
        if (opcode == MMC_SEND_STATUS) {
            status_commands++;
            if (cmd13_mode == 2)
                return;
            regs[SDXC_REG_RINTR / 4] |= SDXC_COMMAND_DONE;
            if (cmd13_mode == 1)
                regs[SDXC_REG_RESP0 / 4] |= R1_ERROR;
            if (cmd13_mode == 3)
                regs[SDXC_REG_RINTR / 4] |= SDXC_RESP_TIMEOUT;
        } else if (opcode == MMC_WRITE_BLOCK) {
            assert(data_commands < 2);
            lbas[data_commands++] = regs[SDXC_REG_CARG / 4];
            regs[SDXC_REG_RINTR / 4] |= SDXC_COMMAND_DONE;
            sector_words = 0;
        } else if (opcode == MMC_STOP_TRANSMISSION) {
            stop_commands++;
            regs[SDXC_REG_RINTR / 4] |= SDXC_COMMAND_DONE;
            regs[SDXC_REG_STAS / 4] = write_pending ? 0 : SDXC_FIFO_EMPTY;
        } else {
            assert(!"unexpected command");
        }
    }
    if (offset == 0x200) {
        assert(fifo_words < 256);
        fifo[fifo_words++] = value;
        if (++sector_words == 128)
            regs[SDXC_REG_RINTR / 4] |= SDXC_DATA_OVER;
    }
}

static void reset_model(struct sunxi_mmc_host *host)
{
    memset(host, 0, sizeof(*host));
    memset(regs, 0, sizeof(regs));
    memset(fifo, 0, sizeof(fifo));
    host->reg_base = (u8 *)regs;
    regs[SDXC_REG_CLKCR / 4] = SDXC_CARD_CLOCK_ON;
    regs[SDXC_REG_GCTRL / 4] = SDXC_INTERRUPT_ENABLE_BIT;
    regs[SDXC_REG_STAS / 4] = SDXC_FIFO_EMPTY;
    ticks = 0;
    fifo_words = sector_words = data_commands = stop_commands = 0;
    dma_resets = fifo_resets = status_commands = 0;
    cmd13_mode = 0;
    write_pending = false;
}
'''

TESTS = r'''
int main(void)
{
    struct sunxi_mmc_host host;
    struct mmc_command cmd = { MMC_READ_MULTIPLE_BLOCK }, stop = { 0 };
    struct mmc_data data = { 512, 8, MMC_DATA_READ, &stop };
    struct mmc_request mrq = { &cmd, NULL, &data };
    union { u32 align; u8 bytes[1025]; } storage;
    u8 *unaligned = storage.bytes + 1;
    unsigned int i;

    /* Two calls must preserve bytes, including an unaligned source pointer. */
    reset_model(&host);
    for (i = 0; i < 1024; i++)
        unaligned[i] = (u8)(i * 7 + 3);
    assert(sunxi_mmc_panic_takeover(&host, 1U << 16) == 0);
    assert(sunxi_mmc_panic_write_sector(&host, 1U << 16, 400, unaligned) == 0);
    assert(sunxi_mmc_panic_write_sector(&host, 1U << 16, 401, unaligned + 512) == 0);
    assert(fifo_words == 256 && data_commands == 2);
    assert(lbas[0] == 400 && lbas[1] == 401);
    assert(memcmp(fifo, unaligned, 1024) == 0);
    assert(!(regs[SDXC_REG_GCTRL / 4] & SDXC_INTERRUPT_ENABLE_BIT));
    assert(!stop_commands && !dma_resets);
    puts("PASS: two CMD24 writes, unaligned bytes and sector order");

    reset_model(&host);
    cmd13_mode = 1;
    assert(sunxi_mmc_panic_takeover(&host, 1U << 16) == -EIO);
    assert(!data_commands);
    reset_model(&host);
    cmd13_mode = 2;
    assert(sunxi_mmc_panic_takeover(&host, 1U << 16) == -ETIMEDOUT);
    assert(ticks < 20000 && !data_commands);
    reset_model(&host);
    cmd13_mode = 3;
    assert(sunxi_mmc_panic_takeover(&host, 1U << 16) == -ETIMEDOUT);
    puts("PASS: CMD13 R1 error, missing completion and controller timeout");

    reset_model(&host);
    host.mrq = &mrq;
    host.int_sum = SDXC_COMMAND_DONE | SDXC_DATA_OVER | SDXC_AUTO_COMMAND_DONE;
    regs[SDXC_REG_CMDR / 4] = MMC_READ_MULTIPLE_BLOCK | SDXC_SEND_AUTO_STOP;
    regs[SDXC_REG_BLKSZ / 4] = 512;
    regs[SDXC_REG_BCNTR / 4] = 4096;
    regs[SDXC_REG_GCTRL / 4] |= SDXC_DMA_ENABLE_BIT;
    regs[SDXC_REG_DMAC / 4] = SDXC_IDMAC_IDMA_ON;
    regs[SDXC_REG_RESP0 / 4] = R1_READY_FOR_DATA | (R1_STATE_TRAN << 9);
    assert(sunxi_mmc_panic_takeover(&host, 1U << 16) == 0);
    assert(!stop_commands && !dma_resets && status_commands == 1);
    puts("PASS: completion acknowledged by IRQ survives in host->int_sum");

    reset_model(&host);
    host.mrq = &mrq;
    regs[SDXC_REG_CMDR / 4] = MMC_READ_MULTIPLE_BLOCK | SDXC_SEND_AUTO_STOP;
    regs[SDXC_REG_BLKSZ / 4] = 512;
    regs[SDXC_REG_BCNTR / 4] = 4096;
    regs[SDXC_REG_RINTR / 4] = SDXC_DATA_TIMEOUT;
    assert(sunxi_mmc_panic_takeover(&host, 1U << 16) == -ETIMEDOUT);
    assert(!stop_commands && !dma_resets && !status_commands);
    puts("PASS: hardware timeout never enters DMA abort");

    /* Software expiry can abort a still-active multi-block read or write. */
    for (i = 0; i < 2; i++) {
        reset_model(&host);
        write_pending = i;
        cmd.opcode = i ? MMC_WRITE_MULTIPLE_BLOCK : MMC_READ_MULTIPLE_BLOCK;
        data.flags = i ? MMC_DATA_WRITE : MMC_DATA_READ;
        host.mrq = &mrq;
        regs[SDXC_REG_CMDR / 4] = cmd.opcode | SDXC_SEND_AUTO_STOP |
            (i ? SDXC_WRITE : 0);
        regs[SDXC_REG_BLKSZ / 4] = 512;
        regs[SDXC_REG_BCNTR / 4] = 4096;
        regs[SDXC_REG_RINTR / 4] = SDXC_COMMAND_DONE;
        regs[SDXC_REG_STAS / 4] = SDXC_FIFO_EMPTY | SDXC_DATA_FSM_BUSY;
        regs[SDXC_REG_RESP0 / 4] = R1_READY_FOR_DATA | (R1_STATE_TRAN << 9);
        regs[SDXC_REG_GCTRL / 4] |= SDXC_DMA_ENABLE_BIT;
        regs[SDXC_REG_DMAC / 4] = SDXC_IDMAC_IDMA_ON;
        regs[SDXC_REG_IDST / 4] = SDXC_IDMAC_READ;
        assert(sunxi_mmc_panic_takeover(&host, 1U << 16) == 0);
        assert(stop_commands == 1 && dma_resets == 2);
        assert(fifo_resets == i && status_commands == 1);
    }
    puts("PASS: software deadline aborts CMD18/CMD25; only write resets FIFO");
    puts("Hardware helper smoke tests passed (MMIO model, not board validation).");
    return 0;
}
'''


def main():
    macros = defines(ROOT / "drivers/mmc/host/sunxi-mmc.h", ["SDXC_"])
    macros += "\n" + defines(ROOT / "include/linux/mmc/mmc.h", ["R1_", "MMC_"])
    macros += "\n" + defines(ROOT / "include/linux/mmc/core.h", ["MMC_DATA_"])
    source = PREAMBLE + macros + "\n" + MODEL + hardware_helpers() + TESTS
    with tempfile.TemporaryDirectory(prefix="sunxi-mmc-panic-") as directory:
        cfile = Path(directory) / "test.c"
        binary = Path(directory) / "test"
        cfile.write_text(source)
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=gnu89", "-Wall", "-Wextra", "-Werror", "-O1",
            str(cfile), "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
