#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "z80_trace.h"

int main(void) {
    stdio_init_all();

    // Open-drain style: output latch = 0, toggle direction
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_IN);
    gpio_disable_pulls(PIN_WAIT);
    gpio_put(PIN_WAIT, 0);

    while (!stdio_usb_connected())
        sleep_ms(100);

    printf("/WAIT toggle test — 1 second intervals (open-drain)\n");

    bool asserted = false;
    while (true) {
        asserted = !asserted;
        gpio_set_dir(PIN_WAIT, asserted ? GPIO_OUT : GPIO_IN);
        printf("/WAIT %s\n", asserted ? "ASSERTED (driving low)" : "RELEASED (high-Z)");
        sleep_ms(1000);
    }
}
