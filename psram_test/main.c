/*
 * PSRAM test firmware for PGA2350.
 *
 * Validates PSRAM initialization, data integrity, bandwidth, and write
 * jitter.  Reports results over USB CDC (printf).
 *
 * Run: flash psram_test.uf2, connect serial terminal at any baud rate.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/structs/systick.h"
#include "hardware/xip_cache.h"

#include "psram.h"

/* ---- Helpers ---- */

static void wait_for_usb(void) {
    while (!stdio_usb_connected())
        sleep_ms(100);
    sleep_ms(500);  /* let terminal settle */
}

/* ARM cycle counter via SysTick (24-bit, counts DOWN at CPU clock).
 * At 250 MHz, wraps every ~67ms — fine for single-write timing. */

static inline void systick_init(void) {
    /* Enable SysTick with max reload, no interrupt, CPU clock source */
    systick_hw->csr = 0;
    systick_hw->rvr = 0x00FFFFFF;  /* max 24-bit */
    systick_hw->cvr = 0;
    systick_hw->csr = 0x05;       /* enable, CPU clock, no interrupt */
}

static inline uint32_t systick_get(void) {
    return systick_hw->cvr;
}

/* Elapsed cycles (handles wrap, 24-bit counter counts DOWN) */
static inline uint32_t systick_elapsed(uint32_t start, uint32_t end) {
    return (start - end) & 0x00FFFFFF;
}

/* ---- Test 1: Init and detect ---- */

static size_t test_init(void) {
    printf("\n=== Test 1: PSRAM Init & Detect ===\n");
    size_t sz = psram_init();
    if (sz == 0) {
        printf("FAIL: No PSRAM detected!\n");
    } else {
        printf("PASS: PSRAM detected, %u bytes (%u MB)\n",
               (unsigned)sz, (unsigned)(sz / (1024 * 1024)));
    }
    printf("System clock: %u MHz\n", (unsigned)(clock_get_hz(clk_sys) / 1000000));
    return sz;
}

/* ---- Test 2: Sequential write/read verification ---- */

static void test_data_integrity(size_t psram_size) {
    printf("\n=== Test 2: Data Integrity (uncached) ===\n");
    volatile uint8_t *base = (volatile uint8_t *)PSRAM_BASE_UNCACHED;
    uint32_t errors = 0;

    /* Pattern 1: 0xAA */
    printf("  Writing 0xAA pattern...\n");
    for (size_t i = 0; i < psram_size; i++)
        base[i] = 0xAA;
    printf("  Verifying...\n");
    for (size_t i = 0; i < psram_size; i++) {
        if (base[i] != 0xAA) {
            if (errors < 10)
                printf("    MISMATCH at 0x%06X: expected 0xAA, got 0x%02X\n",
                       (unsigned)i, base[i]);
            errors++;
        }
    }

    /* Pattern 2: 0x55 */
    printf("  Writing 0x55 pattern...\n");
    for (size_t i = 0; i < psram_size; i++)
        base[i] = 0x55;
    printf("  Verifying...\n");
    for (size_t i = 0; i < psram_size; i++) {
        if (base[i] != 0x55) {
            if (errors < 10)
                printf("    MISMATCH at 0x%06X: expected 0x55, got 0x%02X\n",
                       (unsigned)i, base[i]);
            errors++;
        }
    }

    /* Pattern 3: address-based */
    printf("  Writing address-based pattern...\n");
    for (size_t i = 0; i < psram_size; i++)
        base[i] = (uint8_t)(i & 0xFF);
    printf("  Verifying...\n");
    for (size_t i = 0; i < psram_size; i++) {
        uint8_t expected = (uint8_t)(i & 0xFF);
        if (base[i] != expected) {
            if (errors < 10)
                printf("    MISMATCH at 0x%06X: expected 0x%02X, got 0x%02X\n",
                       (unsigned)i, expected, base[i]);
            errors++;
        }
    }

    if (errors == 0)
        printf("PASS: All patterns verified (%u bytes)\n", (unsigned)psram_size);
    else
        printf("FAIL: %u mismatches\n", (unsigned)errors);
}

/* ---- Test 3: Cache coherency ---- */

static void test_cache_coherency(size_t psram_size) {
    printf("\n=== Test 3: Cache Coherency ===\n");
    volatile uint32_t *uncached = (volatile uint32_t *)PSRAM_BASE_UNCACHED;
    volatile uint32_t *cached   = (volatile uint32_t *)PSRAM_BASE_CACHED;
    uint32_t words = psram_size / 4;
    uint32_t test_count = (words > 1024) ? 1024 : words;
    uint32_t errors = 0;

    /* Write via uncached, invalidate cache, read via cached */
    for (uint32_t i = 0; i < test_count; i++)
        uncached[i] = 0xDEAD0000 | i;

    /* Invalidate the XIP cache so cached reads pick up new data */
    xip_cache_invalidate_all();

    for (uint32_t i = 0; i < test_count; i++) {
        uint32_t expected = 0xDEAD0000 | i;
        uint32_t got = cached[i];
        if (got != expected) {
            if (errors < 10)
                printf("  MISMATCH [%u]: expected 0x%08X, got 0x%08X\n",
                       (unsigned)i, (unsigned)expected, (unsigned)got);
            errors++;
        }
    }

    if (errors == 0)
        printf("PASS: Uncached write → cached read coherent (%u words)\n",
               (unsigned)test_count);
    else
        printf("FAIL: %u mismatches\n", (unsigned)errors);
}

/* ---- Test 4: DMA write, CPU read-back ---- */

static void test_dma(size_t psram_size) {
    printf("\n=== Test 4: DMA Write → CPU Read ===\n");

    /* Source buffer in SRAM */
    #define DMA_TEST_WORDS 1024
    static uint32_t src[DMA_TEST_WORDS];
    for (int i = 0; i < DMA_TEST_WORDS; i++)
        src[i] = 0xCAFE0000 | i;

    volatile uint32_t *dst = (volatile uint32_t *)PSRAM_BASE_UNCACHED;

    int chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, true);

    dma_channel_configure(chan, &cfg, (void *)dst, src, DMA_TEST_WORDS, true);
    dma_channel_wait_for_finish_blocking(chan);
    dma_channel_unclaim(chan);

    /* Verify via uncached read */
    uint32_t errors = 0;
    for (int i = 0; i < DMA_TEST_WORDS; i++) {
        uint32_t expected = 0xCAFE0000 | i;
        if (dst[i] != expected) {
            if (errors < 10)
                printf("  MISMATCH [%d]: expected 0x%08X, got 0x%08X\n",
                       i, (unsigned)expected, (unsigned)dst[i]);
            errors++;
        }
    }

    if (errors == 0)
        printf("PASS: DMA write verified (%d words)\n", DMA_TEST_WORDS);
    else
        printf("FAIL: %u mismatches\n", (unsigned)errors);
}

/* ---- Test 5: Average bandwidth ---- */

static void test_bandwidth(size_t psram_size) {
    printf("\n=== Test 5: Average Bandwidth ===\n");

    /* Use 1 MB for bandwidth tests */
    const size_t test_size = (psram_size > 1024 * 1024) ? 1024 * 1024 : psram_size;
    volatile uint8_t *uncached = (volatile uint8_t *)PSRAM_BASE_UNCACHED;
    volatile uint8_t *cached   = (volatile uint8_t *)PSRAM_BASE_CACHED;
    uint64_t t0, t1, us;

    /* Uncached sequential write */
    t0 = time_us_64();
    memset((void *)uncached, 0x42, test_size);
    t1 = time_us_64();
    us = t1 - t0;
    printf("  Uncached write: %u KB in %llu us = %.1f MB/s\n",
           (unsigned)(test_size / 1024), us,
           (double)test_size / us);

    /* Uncached sequential read */
    volatile uint32_t sink = 0;
    volatile uint32_t *rp = (volatile uint32_t *)uncached;
    uint32_t words = test_size / 4;
    t0 = time_us_64();
    for (uint32_t i = 0; i < words; i++)
        sink = rp[i];
    t1 = time_us_64();
    us = t1 - t0;
    (void)sink;
    printf("  Uncached read:  %u KB in %llu us = %.1f MB/s\n",
           (unsigned)(test_size / 1024), us,
           (double)test_size / us);

    /* Cached sequential read (prime cache first) */
    xip_cache_invalidate_all();
    volatile uint32_t *cp = (volatile uint32_t *)cached;
    t0 = time_us_64();
    for (uint32_t i = 0; i < words; i++)
        sink = cp[i];
    t1 = time_us_64();
    us = t1 - t0;
    (void)sink;
    printf("  Cached read:    %u KB in %llu us = %.1f MB/s\n",
           (unsigned)(test_size / 1024), us,
           (double)test_size / us);

    /* DMA write bandwidth */
    static uint8_t dma_src[4096];
    memset(dma_src, 0x55, sizeof(dma_src));

    int chan = dma_claim_unused_channel(true);
    uint32_t total_dma = 0;
    t0 = time_us_64();
    for (size_t off = 0; off < test_size; off += sizeof(dma_src)) {
        size_t chunk = test_size - off;
        if (chunk > sizeof(dma_src)) chunk = sizeof(dma_src);

        dma_channel_config cfg = dma_channel_get_default_config(chan);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
        channel_config_set_read_increment(&cfg, true);
        channel_config_set_write_increment(&cfg, true);
        dma_channel_configure(chan, &cfg,
                              (void *)(uncached + off), dma_src,
                              chunk / 4, true);
        dma_channel_wait_for_finish_blocking(chan);
        total_dma += chunk;
    }
    t1 = time_us_64();
    us = t1 - t0;
    dma_channel_unclaim(chan);
    printf("  DMA write:      %u KB in %llu us = %.1f MB/s\n",
           (unsigned)(total_dma / 1024), us,
           (double)total_dma / us);
}

/* ---- Test 6: Write jitter (per-record latency) ---- */

/*
 * Simulates the real trace workload: writes 8-byte records sequentially
 * to PSRAM, measuring each write with the cycle counter.
 *
 * The critical constraint: at 4 MHz Z80 (8M PIO samples/sec), the 32KB
 * DMA ring buffer holds ~1ms of samples.  Any PSRAM stall >1ms risks
 * PIO FIFO overflow.  Records are emitted at ~1.3M/sec (one per ~4-6
 * samples), so the budget per write is ~770ns (~192 cycles at 250 MHz).
 */

typedef struct {
    uint16_t address;
    uint8_t  data;
    uint8_t  cycle_type;
    uint8_t  refresh;
    uint8_t  wait_count;
    uint8_t  flags;
    uint8_t  seq;
} test_record_t;

_Static_assert(sizeof(test_record_t) == 8, "record must be 8 bytes");

#define JITTER_RECORDS  (1024 * 1024)  /* 1M records = full ring buffer */
#define JITTER_RING_MASK (JITTER_RECORDS - 1)

static void test_write_jitter(void) {
    printf("\n=== Test 6: Write Jitter (8-byte records) ===\n");

    volatile test_record_t *ring =
        (volatile test_record_t *)PSRAM_BASE_UNCACHED;

    test_record_t rec = {
        .address = 0x1234,
        .data = 0xAB,
        .cycle_type = 0,
        .refresh = 0x42,
        .wait_count = 0,
        .flags = 0,
        .seq = 0,
    };

    uint32_t cpu_mhz = clock_get_hz(clk_sys) / 1000000;
    printf("  CPU clock: %u MHz\n", (unsigned)cpu_mhz);
    printf("  Writing %u records (8 bytes each)...\n", JITTER_RECORDS);

    /* Collect timing for every write using SysTick */
    uint32_t min_cycles = 0x00FFFFFF;
    uint32_t max_cycles = 0;
    uint64_t total_cycles = 0;

    /* Histogram: bucket[i] = count of writes taking i..i+3 cycles,
     * last bucket catches everything >= HIST_BUCKETS*4 cycles */
    #define HIST_BUCKET_SHIFT 2  /* 4 cycles per bucket */
    #define HIST_BUCKETS 256
    static uint32_t hist[HIST_BUCKETS + 1];
    memset(hist, 0, sizeof(hist));

    /* Track worst-case consecutive stall */
    uint32_t stall_threshold = cpu_mhz;  /* 1µs in cycles */
    uint32_t current_stall_run = 0;
    uint32_t max_stall_run = 0;

    systick_init();

    for (uint32_t i = 0; i < JITTER_RECORDS; i++) {
        rec.seq = (uint8_t)(i & 0x7F);
        rec.address = (uint16_t)(i & 0xFFFF);

        uint32_t t0 = systick_get();
        ring[i & JITTER_RING_MASK] = rec;
        uint32_t t1 = systick_get();

        uint32_t elapsed = systick_elapsed(t0, t1);
        total_cycles += elapsed;
        if (elapsed < min_cycles) min_cycles = elapsed;
        if (elapsed > max_cycles) max_cycles = elapsed;

        uint32_t bucket = elapsed >> HIST_BUCKET_SHIFT;
        if (bucket >= HIST_BUCKETS) bucket = HIST_BUCKETS;
        hist[bucket]++;

        if (elapsed > stall_threshold) {
            current_stall_run += elapsed;
            if (current_stall_run > max_stall_run)
                max_stall_run = current_stall_run;
        } else {
            current_stall_run = 0;
        }
    }

    double avg_cycles = (double)total_cycles / JITTER_RECORDS;
    double avg_ns = avg_cycles * 1000.0 / cpu_mhz;
    double min_ns = (double)min_cycles * 1000.0 / cpu_mhz;
    double max_ns = (double)max_cycles * 1000.0 / cpu_mhz;
    double max_stall_us = (double)max_stall_run / cpu_mhz;

    printf("  Min:  %u cycles (%.0f ns)\n", (unsigned)min_cycles, min_ns);
    printf("  Max:  %u cycles (%.0f ns)\n", (unsigned)max_cycles, max_ns);
    printf("  Avg:  %.1f cycles (%.0f ns)\n", avg_cycles, avg_ns);

    /* Calculate p99 from histogram */
    uint32_t p99_target = JITTER_RECORDS - JITTER_RECORDS / 100;
    uint32_t cumulative = 0;
    uint32_t p99_bucket = 0;
    for (uint32_t b = 0; b <= HIST_BUCKETS; b++) {
        cumulative += hist[b];
        if (cumulative >= p99_target) {
            p99_bucket = b;
            break;
        }
    }
    uint32_t p99_cycles = (p99_bucket < HIST_BUCKETS)
        ? (p99_bucket << HIST_BUCKET_SHIFT)
        : max_cycles;
    double p99_ns = (double)p99_cycles * 1000.0 / cpu_mhz;
    printf("  P99:  ~%u cycles (~%.0f ns)\n", (unsigned)p99_cycles, p99_ns);

    printf("  Max consecutive stall: %.1f us\n", max_stall_us);

    /* Assess: 32KB DMA buffer at 8M samples/sec = 1024 us slack */
    printf("  DMA buffer slack: 1024 us\n");
    if (max_stall_us < 1024.0)
        printf("  PASS: Worst-case stall (%.1f us) << 1024 us DMA buffer\n",
               max_stall_us);
    else
        printf("  WARNING: Worst-case stall (%.1f us) may risk DMA overflow!\n",
               max_stall_us);

    /* Print histogram tail (buckets with >0 entries above p99) */
    printf("\n  Histogram (tail, >p99 region):\n");
    for (uint32_t b = p99_bucket; b <= HIST_BUCKETS && b < p99_bucket + 20; b++) {
        if (hist[b] > 0) {
            uint32_t lo = b << HIST_BUCKET_SHIFT;
            if (b < HIST_BUCKETS)
                printf("    %4u-%4u cycles: %u\n", (unsigned)lo,
                       (unsigned)(lo + (1 << HIST_BUCKET_SHIFT) - 1),
                       (unsigned)hist[b]);
            else
                printf("    %4u+    cycles: %u\n",
                       (unsigned)(HIST_BUCKETS << HIST_BUCKET_SHIFT),
                       (unsigned)hist[b]);
        }
    }

    /* Effective write throughput */
    double total_bytes = (double)JITTER_RECORDS * 8;
    double total_us = (double)total_cycles / cpu_mhz;
    printf("\n  Effective throughput: %.1f MB/s (%.0f us for %u records)\n",
           total_bytes / total_us, total_us, JITTER_RECORDS);
}

/* ---- Test 7: Sustained write at target rate ---- */

/*
 * Writes at the expected trace rate (~1.3M records/sec) for several
 * seconds, counting writes that exceed a threshold.
 */

static void test_sustained_write(void) {
    printf("\n=== Test 7: Sustained Write (5 seconds at target rate) ===\n");

    volatile test_record_t *ring =
        (volatile test_record_t *)PSRAM_BASE_UNCACHED;

    test_record_t rec = {
        .address = 0x5678,
        .data = 0xCD,
        .cycle_type = 1,
        .refresh = 0x10,
        .wait_count = 0,
        .flags = 0,
        .seq = 0,
    };

    uint32_t cpu_mhz = clock_get_hz(clk_sys) / 1000000;

    /* Target: ~1.3M records/sec = ~770ns/record = ~192 cycles at 250MHz.
     * We write as fast as possible (no artificial delay) and just report. */
    const uint32_t duration_us = 5000000;  /* 5 seconds */
    const uint32_t threshold_cycles = 100 * cpu_mhz;  /* 100 µs */

    uint64_t t_start = time_us_64();
    uint64_t t_end = t_start + duration_us;
    uint32_t write_idx = 0;
    uint32_t stalls_over_100us = 0;
    uint32_t stalls_over_10us = 0;
    uint32_t max_cycles = 0;

    systick_init();

    while (time_us_64() < t_end) {
        rec.seq = (uint8_t)(write_idx & 0x7F);
        rec.address = (uint16_t)(write_idx & 0xFFFF);

        uint32_t t0 = systick_get();
        ring[write_idx & JITTER_RING_MASK] = rec;
        uint32_t t1 = systick_get();

        uint32_t elapsed = systick_elapsed(t0, t1);
        if (elapsed > max_cycles) max_cycles = elapsed;
        if (elapsed > threshold_cycles) stalls_over_100us++;
        if (elapsed > 10 * cpu_mhz) stalls_over_10us++;

        write_idx++;
    }

    uint64_t actual_us = time_us_64() - t_start;
    double records_per_sec = (double)write_idx / ((double)actual_us / 1e6);
    double max_us = (double)max_cycles / cpu_mhz;

    printf("  Duration: %llu us\n", actual_us);
    printf("  Records written: %u (%.1f M/sec)\n",
           (unsigned)write_idx, records_per_sec / 1e6);
    printf("  Max write time: %u cycles (%.1f us)\n",
           (unsigned)max_cycles, max_us);
    printf("  Stalls >10us:  %u\n", (unsigned)stalls_over_10us);
    printf("  Stalls >100us: %u\n", (unsigned)stalls_over_100us);

    if (stalls_over_100us == 0 && max_us < 1000.0)
        printf("  PASS: No dangerous stalls in %llu us\n", actual_us);
    else if (max_us >= 1000.0)
        printf("  WARNING: Max stall %.1f us approaches DMA buffer limit!\n",
               max_us);
    else
        printf("  CAUTION: %u stalls >100us (max %.1f us)\n",
               (unsigned)stalls_over_100us, max_us);
}

/* ---- Main ---- */

int main(void) {
    set_sys_clock_khz(250000, true);
    stdio_init_all();
    wait_for_usb();

    printf("\n========================================\n");
    printf("  PGA2350 PSRAM Test Suite\n");
    printf("========================================\n");

    size_t psram_size = test_init();
    if (psram_size == 0) {
        printf("\nABORT: No PSRAM found. Check wiring/board.\n");
        while (1) tight_loop_contents();
    }

    test_data_integrity(psram_size);
    test_cache_coherency(psram_size);
    test_dma(psram_size);
    test_bandwidth(psram_size);
    test_write_jitter();
    test_sustained_write();

    printf("\n========================================\n");
    printf("  All tests complete.\n");
    printf("========================================\n");

    while (1) tight_loop_contents();
}
