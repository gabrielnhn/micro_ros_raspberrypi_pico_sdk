#ifndef __DC_MOTOR_H__
#define __DC_MOTOR_H__

#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define MAX_CYCLES 256

// Struct that represents a motor
typedef struct DC_Motor
{
    uint8_t clockwise_pin;
    uint8_t counterclockwise_pin;
    uint8_t pwm_pin;
} DC_Motor;

/*!
    @brief  Init a motor

    @param  motor                   Motor's reference
    @param  clockwise_pin           Clockwise pin
    @param  counterclockwise_pin    Counterclockwise pin
    @param  pwm_pin                 PWM pin   
*/
void init_DC_Motor(DC_Motor *motor, uint8_t clockwise_pin,
                   uint8_t counterclockwise_pin, uint8_t pwm_pin);

/*!
    @brief  Motor goes forward

    @param  motor   Motor's reference
    @param  speed   Desired speed
*/
void forward_DC_Motor(DC_Motor *motor, uint16_t speed);

/*!
    @brief  Motor goes backwards

    @param  motor   Motor's reference
    @param  speed   Desired speed
*/
void backwards_DC_Motor(DC_Motor *motor, uint16_t speed);


/*!
    @brief  Run motor according to the absolute value of the speed

    @param  motor   Motor's reference
*/
void run_DC_MOTOR(DC_Motor *motor, int speed);


/*!
    @brief  Stop a motor

    @param  motor   Motor's reference
*/
void stop_DC_Motor(DC_Motor *motor);

/*!
    @brief  Brake a motor

    @param  motor   Motor's reference
*/
void short_brake_DC_Motor(DC_Motor *motor);

#endif