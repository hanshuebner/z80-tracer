/*
 * WAIT signal test firmware for PGA2350.
 *
 * Toggles WAIT (GPIO 32) every second using open-drain style:
 * output latch = 0, toggle direction (GPIO_OUT = drive low = assert,
 * GPIO_IN = high-Z = released, external pull-up restores high).
 *
 * This is safe for the shared WAIT line (also used by the screen
 * controller) because we never drive it high.
 *
 * Connect a serial terminal to see status messages.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/padsbank0.h"

#include "z80_trace.h"

// GPIO 32 is bit 0 in the "high" bank (32-47)
#define WAIT_HI_BIT  (1u << (PIN_WAIT - 32))

int main(void) {
    set_sys_clock_khz(300000, true);
    stdio_init_all();

    while (!stdio_usb_connected())
        sleep_ms(100);

    printf("WAIT test on GPIO %d (high bank bit %d)\n\n", PIN_WAIT, PIN_WAIT - 32);

    // Step 0: sample the pin for 1 second to observe display controller activity
    printf("Sampling WAIT for 1 second (before gpio_init)...\n");
    {
        uint32_t samples = 0, highs = 0, lows = 0, transitions = 0;
        int prev = -1;
        uint64_t t_end = time_us_64() + 1000000;
        while (time_us_64() < t_end) {
            int val = (sio_hw->gpio_hi_in & WAIT_HI_BIT) ? 1 : 0;
            samples++;
            if (val) highs++; else lows++;
            if (prev >= 0 && val != prev) transitions++;
            prev = val;
        }
        printf("  %lu samples: %lu high, %lu low, %lu transitions\n\n",
               samples, highs, lows, transitions);
    }

    // Step 1: gpio_init
    gpio_init(PIN_WAIT);
    uint32_t funcsel = iobank0_hw->io[PIN_WAIT].ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS;
    uint32_t pad = padsbank0_hw->io[PIN_WAIT];
    printf("After gpio_init: funcsel=%lu (expect 5=SIO)\n", funcsel);
    printf("Pad register: 0x%08lx\n", pad);
    printf("  OD (output disable) = %lu\n", (pad >> 7) & 1);
    printf("  IE (input enable)   = %lu\n", (pad >> 6) & 1);
    printf("  DRIVE               = %lu\n", (pad >> 4) & 3);
    printf("  PUE (pull-up)       = %lu\n", (pad >> 3) & 1);
    printf("  PDE (pull-down)     = %lu\n", (pad >> 2) & 1);
    printf("  ISO (isolation)     = %lu\n", (pad >> 8) & 1);

    // Step 2: set output value to 0
    gpio_put(PIN_WAIT, 0);
    printf("After gpio_put(0): gpio_hi_out=0x%08lx (bit0 should be 0)\n",
           sio_hw->gpio_hi_out);

    // Step 3: set as input initially
    gpio_set_dir(PIN_WAIT, GPIO_IN);
    gpio_disable_pulls(PIN_WAIT);
    printf("After set_dir(IN): gpio_hi_oe=0x%08lx (bit0 should be 0)\n",
           sio_hw->gpio_hi_oe);
    printf("Pin reads: %d (expect 1 from pull-up)\n\n", gpio_get(PIN_WAIT));

    // Step 4: try driving high and low to verify driver works at all
    printf("Testing output driver...\n");

    // Drive HIGH
    sio_hw->gpio_hi_set = WAIT_HI_BIT;
    sio_hw->gpio_hi_oe_set = WAIT_HI_BIT;
    sleep_ms(10);
    printf("  Drive HIGH: oe=0x%08lx out=0x%08lx pin=%d\n",
           sio_hw->gpio_hi_oe, sio_hw->gpio_hi_out, gpio_get(PIN_WAIT));

    // Drive LOW
    sio_hw->gpio_hi_clr = WAIT_HI_BIT;
    sleep_ms(10);
    printf("  Drive LOW:  oe=0x%08lx out=0x%08lx pin=%d\n",
           sio_hw->gpio_hi_oe, sio_hw->gpio_hi_out, gpio_get(PIN_WAIT));

    // High-Z
    sio_hw->gpio_hi_oe_clr = WAIT_HI_BIT;
    sleep_ms(10);
    printf("  High-Z:     oe=0x%08lx out=0x%08lx pin=%d\n",
           sio_hw->gpio_hi_oe, sio_hw->gpio_hi_out, gpio_get(PIN_WAIT));

    // Step 5: scan SAFE GPIOs only (skip active control signals)
    // GP28=/WR GP29=/RFSH GP30=CLK GP31=/HALT — driving these crashes the Z80
    // GP32=/WAIT GP33=/INT GP34=/NMI GP35=/RESET — these are safe to assert briefly
    printf("\nScanning safe GPIOs (32-35) — drive low 1s each...\n");
    printf("Watch if Z80 freezes on any pin.\n");
    int safe_pins[] = { 32, 33 };
    const char *safe_names[] = { "WAIT", "INT" };
    for (int i = 0; i < 2; i++) {
        int pin = safe_pins[i];
        printf("  GPIO %d (%s) — driving low...", pin, safe_names[i]);
        gpio_init(pin);
        gpio_put(pin, 0);
        gpio_set_dir(pin, GPIO_OUT);  // drive low
        sleep_ms(1000);
        printf(" pin reads %d. Releasing...", gpio_get(pin));
        gpio_set_dir(pin, GPIO_IN);   // release
        sleep_ms(1000);
        printf(" pin reads %d\n", gpio_get(pin));
    }

    printf("\nDone.\n");
    while (1) tight_loop_contents();
}
