/*
 * PSRAM driver for PGA2350 (APS6404L-3SQR, 8 MB QSPI PSRAM).
 *
 * Based on AudioMorphology/PSRAM (MIT license, originally from SparkFun).
 * Adapted: hardcoded CS pin, removed printf, cleaned up for project use.
 *
 * All public functions are __no_inline_not_in_flash_func because they
 * reconfigure the QSPI bus that flash also sits on.
 */

#include "psram.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

/* ---- Internal: reconfigure flash (CS0) timing after QMI disturbance ---- */

static void __no_inline_not_in_flash_func(psram_set_qmi_timing)(void) {
    /* Wait for flash CS to deassert */
    while ((ioqspi_hw->io[1].status &
            IO_QSPI_GPIO_QSPI_SS_STATUS_OUTTOPAD_BITS) !=
           IO_QSPI_GPIO_QSPI_SS_STATUS_OUTTOPAD_BITS) {
    }

    /* Recalculate flash timing assuming max 133 MHz flash */
    const int max_flash_freq = 133000000;
    const int divisor = (clock_get_hz(clk_sys) + max_flash_freq - 1) / max_flash_freq;
    const int rxdelay = divisor;
    qmi_hw->m[0].timing =
        (1 << QMI_M0_TIMING_COOLDOWN_LSB) |
        (rxdelay << QMI_M0_TIMING_RXDELAY_LSB) |
        (divisor << QMI_M0_TIMING_CLKDIV_LSB);

    /* Force a read through XIP to apply timing */
    volatile uint32_t *ptr = (volatile uint32_t *)0x14000000;
    (void)*ptr;
}

/* ---- Internal: detect PSRAM via SPI Read ID command ---- */

static size_t __no_inline_not_in_flash_func(psram_detect)(void) {
    size_t psram_size = 0;

    uint32_t intr_stash = save_and_disable_interrupts();

    /* Enter QMI direct mode at safe slow clock */
    qmi_hw->direct_csr = 30 << QMI_DIRECT_CSR_CLKDIV_LSB |
                          QMI_DIRECT_CSR_EN_BITS;

    /* Wait for any prior XIP cooldown to expire */
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {}

    /* Exit quad mode in case already initialised */
    qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    qmi_hw->direct_tx = QMI_DIRECT_TX_OE_BITS |
                         (QMI_DIRECT_TX_IWIDTH_VALUE_Q << QMI_DIRECT_TX_IWIDTH_LSB) |
                         0xF5;  /* Exit QPI */
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {}
    (void)qmi_hw->direct_rx;
    qmi_hw->direct_csr &= ~QMI_DIRECT_CSR_ASSERT_CS1N_BITS;

    /* Read ID: command 0x9F, then 6 dummy/data bytes */
    qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;

    uint8_t kgd = 0, eid = 0;
    for (int i = 0; i < 7; i++) {
        qmi_hw->direct_tx = (i == 0) ? 0x9F : 0xFF;
        while (!(qmi_hw->direct_csr & QMI_DIRECT_CSR_TXEMPTY_BITS)) {}
        while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {}

        if (i == 5)      kgd = (uint8_t)qmi_hw->direct_rx;
        else if (i == 6) eid = (uint8_t)qmi_hw->direct_rx;
        else             (void)qmi_hw->direct_rx;
    }

    qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS |
                             QMI_DIRECT_CSR_EN_BITS);

    if (kgd == 0x5D) {
        psram_size = 1024 * 1024;  /* base 1 MiB */
        uint8_t size_id = eid >> 5;
        if (eid == 0x26 || size_id == 2)
            psram_size *= 8;   /* 8 MiB */
        else if (size_id == 0)
            psram_size *= 2;   /* 2 MiB */
        else if (size_id == 1)
            psram_size *= 4;   /* 4 MiB */
    }

    restore_interrupts(intr_stash);
    return psram_size;
}

/* ---- Public: initialise PSRAM ---- */

size_t __no_inline_not_in_flash_func(psram_init)(void) {
    gpio_set_function(PSRAM_CS_PIN, GPIO_FUNC_XIP_CS1);

    size_t sz = psram_detect();
    if (!sz) return 0;

    psram_set_qmi_timing();

    /* Enter QMI direct mode targeting CS1, clkdiv=10 (safe for init) */
    qmi_hw->direct_csr = (10 << QMI_DIRECT_CSR_CLKDIV_LSB) |
                          QMI_DIRECT_CSR_EN_BITS |
                          QMI_DIRECT_CSR_AUTO_CS1N_BITS;
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {}

    /* Enable QPI mode on the APS6404 */
    qmi_hw->direct_tx = QMI_DIRECT_TX_NOPUSH_BITS | 0x35;
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {}

    /* Calculate PSRAM timing from system clock */
    const int max_psram_freq = 133000000;
    const int clock_hz = clock_get_hz(clk_sys);
    int divisor = (clock_hz + max_psram_freq - 1) / max_psram_freq;
    if (divisor == 1 && clock_hz > 100000000)
        divisor = 2;
    int rxdelay = divisor;
    if (clock_hz / divisor > 100000000)
        rxdelay += 1;

    /* Max select <= 8µs (in units of 64 sys clocks) */
    const int64_t clock_period_fs = 1000000000000000LL / clock_hz;
    const int max_select = (int)((125 * 1000000) / clock_period_fs);
    /* Min deselect >= 18ns */
    const int min_deselect = (int)((18 * 1000000 + (clock_period_fs - 1)) /
                                   clock_period_fs) - (divisor + 1) / 2;

    qmi_hw->m[1].timing =
        (1 << QMI_M1_TIMING_COOLDOWN_LSB) |
        (QMI_M1_TIMING_PAGEBREAK_VALUE_1024 << QMI_M1_TIMING_PAGEBREAK_LSB) |
        (max_select << QMI_M1_TIMING_MAX_SELECT_LSB) |
        (min_deselect << QMI_M1_TIMING_MIN_DESELECT_LSB) |
        (rxdelay << QMI_M1_TIMING_RXDELAY_LSB) |
        (divisor << QMI_M1_TIMING_CLKDIV_LSB);

    /* Quad read: command 0xEB, 6 dummy cycles, all widths quad */
    qmi_hw->m[1].rfmt =
        (QMI_M0_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_PREFIX_WIDTH_LSB) |
        (QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_RFMT_ADDR_WIDTH_LSB) |
        (QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB) |
        (QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_RFMT_DUMMY_WIDTH_LSB) |
        (QMI_M0_RFMT_DATA_WIDTH_VALUE_Q << QMI_M0_RFMT_DATA_WIDTH_LSB) |
        (QMI_M0_RFMT_PREFIX_LEN_VALUE_8 << QMI_M0_RFMT_PREFIX_LEN_LSB) |
        (6 << QMI_M0_RFMT_DUMMY_LEN_LSB);
    qmi_hw->m[1].rcmd = 0xEB;

    /* Quad write: command 0x38, no dummy cycles, all widths quad */
    qmi_hw->m[1].wfmt =
        (QMI_M0_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_PREFIX_WIDTH_LSB) |
        (QMI_M0_WFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_WFMT_ADDR_WIDTH_LSB) |
        (QMI_M0_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_SUFFIX_WIDTH_LSB) |
        (QMI_M0_WFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_WFMT_DUMMY_WIDTH_LSB) |
        (QMI_M0_WFMT_DATA_WIDTH_VALUE_Q << QMI_M0_WFMT_DATA_WIDTH_LSB) |
        (QMI_M0_WFMT_PREFIX_LEN_VALUE_8 << QMI_M0_WFMT_PREFIX_LEN_LSB);
    qmi_hw->m[1].wcmd = 0x38;

    /* Disable direct mode, enable memory-mapped writes */
    qmi_hw->direct_csr = 0;
    hw_set_bits(&xip_ctrl_hw->ctrl, XIP_CTRL_WRITABLE_M1_BITS);

    return sz;
}
