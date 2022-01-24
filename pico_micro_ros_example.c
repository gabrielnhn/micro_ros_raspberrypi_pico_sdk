#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

const uint LED_PIN = 25;


// ROS globals
rcl_node_t node;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_timer_t timer;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
geometry_msgs__msg__Twist command;


void subscription_callback(const void * msgin)
// `/cmd_vel` callback
{
    // Cast received message to used type
    geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *) msgin;

    command = *msg;
    rcl_ret_t ret = rcl_publish(&publisher, &command, NULL);
}

int main()
{
    // micro-ROS setup
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

    // PUBLISHER
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "pico_publisher"
    );

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
    //     timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rc = rclc_executor_add_subscription(
        &executor, &subscriber, &command,
        &subscription_callback, ON_NEW_DATA
    );
    // rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    return 0;
}
