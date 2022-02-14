#include "DC_Motor.h"

void init_DC_Motor(DC_Motor *motor, uint8_t clockwise_pin,
                   uint8_t counterclockwise_pin, uint8_t pwm_pin)
{
    motor->clockwise_pin = clockwise_pin;
    motor->counterclockwise_pin = counterclockwise_pin;
    motor->pwm_pin = pwm_pin;

    // --- Digital pin setup ---
    gpio_init(motor->clockwise_pin);
    gpio_set_dir(motor->clockwise_pin, GPIO_OUT);

    gpio_init(motor->counterclockwise_pin);
    gpio_set_dir(motor->counterclockwise_pin, GPIO_OUT);

    // --- PWM pin setup ---
    gpio_set_function(motor->pwm_pin, GPIO_FUNC_PWM);       // Define as PWM function
    uint slice_num = pwm_gpio_to_slice_num(motor->pwm_pin); // PWM slice of GPIO
    pwm_set_wrap(slice_num, MAX_CYCLES);                    // Set the max number of cycles
    pwm_set_gpio_level(motor->pwm_pin, 0);                  // Initialize with PWM level 0
    pwm_set_enabled(slice_num, 1);                          // Enable PWM
}

void forward_DC_Motor(DC_Motor *motor, uint16_t speed)
{
    gpio_put(motor->clockwise_pin, 1);
    gpio_put(motor->counterclockwise_pin, 0);
    pwm_set_gpio_level(motor->pwm_pin, speed <= MAX_CYCLES ? speed : MAX_CYCLES);
}

void backwards_DC_Motor(DC_Motor *motor, uint16_t speed)
{
    gpio_put(motor->clockwise_pin, 0);
    gpio_put(motor->counterclockwise_pin, 1);
    pwm_set_gpio_level(motor->pwm_pin, speed <= MAX_CYCLES ? speed : MAX_CYCLES);
}

void stop_DC_Motor(DC_Motor *motor)
{
    gpio_put(motor->clockwise_pin, 0);
    gpio_put(motor->counterclockwise_pin, 0);
    pwm_set_gpio_level(motor->pwm_pin, 0);
}

void short_brake_DC_Motor(DC_Motor *motor)
{
    gpio_put(motor->clockwise_pin, 1);
    gpio_put(motor->counterclockwise_pin, 1);
    pwm_set_gpio_level(motor->pwm_pin, 0);
}