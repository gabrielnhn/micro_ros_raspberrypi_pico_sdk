#include <stdio.h>
#include <iso646.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico_uart_transports.h"
#include "libdcmotor/DC_Motor.h"
#define LED_PIN 25

int main()
{
    DC_Motor left_motor, right_motor;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    init_DC_Motor(&left_motor, 6, 7, 8);
    init_DC_Motor(&right_motor, 4, 3, 2);

    while(1)
    {
        forward_DC_Motor(&left_motor, 150);
        forward_DC_Motor(&right_motor, 150);
    }
}