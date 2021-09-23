/**************************************************************
   ROS2Handler
 **************************************************************/

// Need to run the microagent on the host computer (e.g., Raspberry Pi) to communicate with this node
// $ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_subscription_t robotStateSubscriber;
rcl_publisher_t imuZAxisPublisher;

std_msgs__msg__Int32 robotStateMsg;
std_msgs__msg__Float32 imuZAxisMsg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t imuZAxisPublisherTimer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){errorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


/**************************************************************
   errorLoop()
 **************************************************************/
void errorLoop()
{
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    // Reset the Teensy (need to do this after Pi power up to help keep ROS communications going)
    //SCB_AIRCR = 0x05FA0004;

    delay(2000);
  }
}

// For some reason, without this duplicate method definition, the dumb-ass
// Arduino compiler wouldn't not do it's job.
void imuZaxisTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
/**************************************************************
   imuZaxisTimerCallback()
 **************************************************************/
void imuZaxisTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer != NULL)
  {
    imuZAxisMsg.data = zAxis;
    RCSOFTCHECK(rcl_publish(&imuZAxisPublisher, &imuZAxisMsg, NULL));
  }
}

/**************************************************************
   robotStateSubscriptionCallback()
 **************************************************************/
void robotStateSubscriptionCallback(const void * msgin)
{
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

  robotState = msg->data;
}

/**************************************************************
   ros2HandlerSetup()
 **************************************************************/
void ros2HandlerSetup()
{
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
            &robotStateSubscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "micro_ros_arduino_robot_state_subscriber"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
            &imuZAxisPublisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "micro_ros_arduino_imu_zaxis_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
            &imuZAxisPublisherTimer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            imuZaxisTimerCallback));

  // create executor
  // The "2" in the init call below is because we have a timer and a subscriber.
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &imuZAxisPublisherTimer));
  RCCHECK(rclc_executor_add_subscription(&executor, &robotStateSubscriber, &robotStateMsg, &robotStateSubscriptionCallback, ON_NEW_DATA));

  imuZAxisMsg.data = 0.0;
}

/**************************************************************
   ros2HandlerLoop()
 **************************************************************/
void ros2HandlerLoop()
{
  while (1)
  {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    delay(100);
  }
}
