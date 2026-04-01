#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "z80_trace.h"

int main(void) {
    stdio_init_all();

    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 1);  // released

    while (!stdio_usb_connected())
        sleep_ms(100);

    printf("/WAIT toggle test — 1 second intervals\n");

    bool asserted = false;
    while (true) {
        asserted = !asserted;
        gpio_put(PIN_WAIT, asserted ? 0 : 1);
        printf("/WAIT %s\n", asserted ? "ASSERTED (Z80 paused)" : "RELEASED (Z80 running)");
        sleep_ms(1000);
    }
}
