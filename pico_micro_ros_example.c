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
#include "pico_uart_transports.h"
#include "libdcmotor/DC_Motor.h"

// DEBUG VALUES
#define CONNECTED 69
#define RUNNING 420

#define A_ON_B_OFF  0b10
#define A_OFF_B_ON  0b01
#define A_ON_B_ON   0b11
#define A_OFF_B_OFF 0b00


#define PIN_ENCODER_LEFT_A 10
#define PIN_ENCODER_LEFT_B 11
#define LED_PIN 25


// ROS globals
rcl_node_t node;
rcl_publisher_t encoder_publisher;
rcl_publisher_t debug_publisher;
rcl_subscription_t subscriber;
rcl_timer_t timer;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
geometry_msgs__msg__Twist command;

std_msgs__msg__Int32 encoder_msg;
std_msgs__msg__Int32 debug_msg;


// bool led_value = false;
// void my_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
// {
//     gpio_put(LED_PIN, led_value);
//     led_value = not led_value;
// }

void subscription_callback(const void * msgin)
// `/cmd_vel` callback
{
    // Cast received message to used type
    // geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *) msgin;

    // command = *msg;
    // rcl_ret_t ret = rcl_publish(&publisher, &command, NULL);
}

int main()
{
    // micro-ROS connection setup
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    // set up LED GPIO
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    /* -------- ROS INIT --------- */

    // ROS memory allocator
    allocator = rcl_get_default_allocator();

    // Wait for micro-ROS agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    // Init ROS support structures
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

    // PUBLISHERS

    // rclc_publisher_init_default(
    //     &publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    //     "pico_publisher"
    // );

    rclc_publisher_init_default(
        &debug_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_debug");

    rclc_publisher_init_default(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder_position");

    // SUBSCRIPTION
    rcl_ret_t rc = rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"
    );

    // TIMER
    // rclc_timer_init_default(
    //     &timer,
    //     &support,
    //     RCL_MS_TO_NS(1000),
    //     my_timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rc = rclc_executor_add_subscription(
        &executor, &subscriber, &command,
        &subscription_callback, ON_NEW_DATA
    );
    // rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);
    
    debug_msg.data = CONNECTED;
    rcl_publish(&debug_publisher, &debug_msg, NULL);


    /* -------- ROS INIT FINISHED --------- */

    /* -------- ENCODER SETUP ------------- */

    gpio_init(PIN_ENCODER_LEFT_A);
    gpio_set_dir(PIN_ENCODER_LEFT_A, GPIO_IN);

    bool last_encoder_left_A = gpio_get(PIN_ENCODER_LEFT_A);

    gpio_init(PIN_ENCODER_LEFT_B);
    gpio_set_dir(PIN_ENCODER_LEFT_B, GPIO_IN);

    int encoder_left_A;
    int encoder_left_B;
    int encoder_left_pulses = 0;

    while (true)
    {
        // debug_msg.data = RUNNING;
        // rcl_publish(&debug_publisher, &debug_msg, NULL);

        // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        encoder_left_A = gpio_get(PIN_ENCODER_LEFT_A);
        encoder_left_B = gpio_get(PIN_ENCODER_LEFT_B);

        // debug_msg.data = (encoder_left_A << 1) | encoder_left_B;
        // debug_msg.data = encoder_left_B;

        // rcl_publish(&debug_publisher, &debug_msg, NULL);

        if(encoder_left_A and !last_encoder_left_A)
        {
            if(encoder_left_B)
                encoder_left_pulses++;
            else
                encoder_left_pulses--;
            
            // Serial.print("Num. pulses: ");
            // Serial.println(encoder_left_pulses);
            encoder_msg.data = encoder_left_pulses;
            rcl_publish(&encoder_publisher, &encoder_msg, NULL);
        }

        last_encoder_left_A = encoder_left_A;
    }

    return 0;
}
