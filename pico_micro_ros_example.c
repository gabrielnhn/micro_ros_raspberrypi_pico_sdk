#include <stdio.h>
#include <iso646.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>
#include "pico/stdlib.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "libdcmotor/DC_Motor.h"



#define CHECK_INTERVAL 100
#define MAX_TIMEOUT 20


// DEBUG VALUES
#define CONNECTED 69
#define RUNNING 420

#define A_ON_B_OFF  0b10
#define A_OFF_B_ON  0b01
#define A_ON_B_ON   0b11
#define A_OFF_B_OFF 0b00


#define PIN_ENCODER_LEFT_A 13
#define PIN_ENCODER_LEFT_B 12
#define LED_PIN 25

#define LINEAR_PROPORTION 400
#define ANGULAR_PROPORTION 100


// ROS globals
// rcl_node_t node;
// rcl_publisher_t encoder_publisher;
// rcl_publisher_t debug_publisher;
// rcl_subscription_t subscriber;
// rcl_timer_t timer;
// rcl_allocator_t allocator;
// rclc_support_t support;
// rclc_executor_t executor;
// geometry_msgs__msg__Twist command;

// std_msgs__msg__Int32 encoder_msg;
// std_msgs__msg__Int32 debug_msg;

// Motors
DC_Motor left_motor, right_motor;


// bool led_value = false;
// void my_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
// {
//     gpio_put(LED_PIN, led_value);
//     led_value = not led_value;
// }

// int count_encoder_pulses(int32_t* encoder_left_pulses, int32_t* encoder_right_pulses);

// void subscription_callback(const void * msgin)
// `/cmd_vel` callback
// {
    // Cast received message to used type
    // geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *) msgin;

    // command = *msg;

    // debug_msg.data = command.linear.x * ;
    // rcl_publish(&debug_publisher, &debug_msg, NULL);
    
    // rcl_ret_t ret = rcl_publish(&publisher, &command, NULL);
// }

int command_linear_x = 0;
int command_angular_z = 0;
char buffer[30];
int timeout_counter = 0;


void check_serial()
{
    int i = 0;
    char a = getchar_timeout_us(0);
    while ((a != '\xff') and (a != PICO_ERROR_TIMEOUT) and (a != '\n') and (i < 30))
    {
        buffer[i] = a;
        i++;
        a = getchar_timeout_us(0);
    }
    buffer[i] = '\n';

    if ((a != '\xff'))
    {
        timeout_counter = 0;

        // valid budder
        sscanf(buffer, "%d %d", &command_linear_x, &command_angular_z);

        // printf("buffer == %s!\n", buffer);
    }
    else
    {
        // printf("TIMEOUT\n", buffer);
        timeout_counter++;
    }

    if (timeout_counter > MAX_TIMEOUT)
    {
        command_linear_x = 0;
        command_angular_z = 0;
    }
}


int main()
{
    // STDIO SHIT:
    stdio_init_all();
    
    // /* -------- ROS INIT --------- */


    /* -------- ROS INIT FINISHED --------- */

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    /* -------- MOTOR SETUP ------------- */
    init_DC_Motor(&left_motor, 6, 7, 9);
    init_DC_Motor(&right_motor, 4, 3, 2);

    /* -------- MOTOR TEST ------------- */
    // while(1)
    // {
        // run_DC_Motor(&left_motor, 150);
        // run_DC_Motor(&right_motor, 150);
        // forward_DC_Motor(&left_motor, 150);
        // forward_DC_Motor(&right_motor, 150);
    // }

    /* -------- ENCODER SETUP ------------- */

//     gpio_init(PIN_ENCODER_LEFT_A);
//     gpio_set_dir(PIN_ENCODER_LEFT_A, GPIO_IN);
//     int32_t encoder_left_pulses = 0;
//     int32_t encoder_right_pulses = 0;

//     gpio_init(PIN_ENCODER_LEFT_B);
//     gpio_set_dir(PIN_ENCODER_LEFT_B, GPIO_IN);

//     int old_pulses = 0;
    while (true)
    {
        // rclc_executor_spin_some(&executor, 100);
        check_serial();

        run_DC_Motor(&left_motor, command_linear_x - command_angular_z);
        run_DC_Motor(&right_motor, command_linear_x + command_angular_z);

        sleep_ms(CHECK_INTERVAL);
    }

//     return 0;
// }


// int count_encoder_pulses(int32_t* encoder_left_pulses, int32_t* encoder_right_pulses)
// {
//     static int encoder_left_A;
//     static int encoder_left_B;

//     static int encoder_right_A;
//     static int encoder_right_B;

//     encoder_left_A = gpio_get(PIN_ENCODER_LEFT_A);
//     encoder_left_B = gpio_get(PIN_ENCODER_LEFT_B);

//     // encoder_right_A = gpio_get(PIN_ENCODER_RIGHT_A);
//     // encoder_right_B = gpio_get(PIN_ENCODER_RIGHT_B);

//     static bool last_encoder_left_A = false;
//     // last_encoder_left_A = gpio_get(PIN_ENCODER_LEFT_A);

//     // static bool last_encoder_right_A = gpio_get(PIN_ENCODER_RIGHT_A);


//     if(encoder_left_A and !last_encoder_left_A)
//     {
//         if(encoder_left_B)
//             (*encoder_left_pulses)++;
//         else
//             (*encoder_left_pulses)--;
        
//         // Serial.print("Num. pulses: ");
//         // Serial.println(encoder_left_pulses);
//         // encoder_msg.data = encoder_left_pulses;
//         // rcl_publish(&encoder_publisher, &encoder_msg, NULL);
//     }

    // if(encoder_right_A and !last_encoder_right_A)
    // {
    //     if(encoder_right_B)
    //         (*encoder_right_pulses)++;
    //     else
    //         (*encoder_right_pulses)--;
         
    //     // Serial.print("Num. pulses: ");
    //     // Serial.println(encoder_right_pulses);
    //     encoder_msg.data = encoder_right_pulses;
    //     rcl_publish(&encoder_publisher, &encoder_msg, NULL);
    // }

    // last_encoder_left_A = encoder_left_A;
    // last_encoder_right_A = encoder_right_A;

}