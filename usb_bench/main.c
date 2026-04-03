/*
 * USB CDC bandwidth benchmark for PGA2350.
 *
 * Sends a continuous stream of bytes over USB CDC as fast as TinyUSB allows.
 * The host-side script measures receive throughput.
 *
 * Protocol:
 *   Host sends 'S' -> firmware starts streaming
 *   Host sends 'X' -> firmware stops streaming
 *
 * The stream is a repeating 256-byte pattern (0x00..0xFF) so the host can
 * verify data integrity.  Every byte is sent through tud_cdc_write() in
 * the largest chunks the write buffer will accept.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "tusb.h"

/* Pre-built pattern buffer — 4 KB of repeating 0x00..0xFF */
#define PATTERN_SIZE 4096
static uint8_t pattern[PATTERN_SIZE];

static void build_pattern(void) {
    for (int i = 0; i < PATTERN_SIZE; i++)
        pattern[i] = (uint8_t)(i & 0xFF);
}

int main(void) {
    set_sys_clock_khz(250000, true);   /* match tracer clock */
    stdio_init_all();
    build_pattern();

    bool streaming = false;

    while (true) {
        tud_task();

        if (!tud_cdc_connected()) {
            streaming = false;
            continue;
        }

        /* Check for commands */
        while (tud_cdc_available()) {
            uint8_t ch;
            tud_cdc_read(&ch, 1);
            if (ch == 'S') streaming = true;
            else if (ch == 'X') streaming = false;
        }

        if (streaming) {
            /* Stuff as much data as the USB write buffer will take */
            uint32_t avail = tud_cdc_write_available();
            if (avail > 0) {
                /* Write in chunks from the pattern buffer */
                uint32_t offset = 0;
                while (avail > 0) {
                    uint32_t chunk = avail < PATTERN_SIZE ? avail : PATTERN_SIZE;
                    uint32_t written = tud_cdc_write(pattern + (offset % PATTERN_SIZE), chunk);
                    avail -= written;
                    offset += written;
                    if (written == 0) break;
                }
            }
            tud_cdc_write_flush();
        }
    }
}
