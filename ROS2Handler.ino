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
#include <vector>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>

rcl_subscription_t robotStateSubscriber;

rcl_publisher_t imuMsgPublisher;
rcl_publisher_t jointStateMsgPublisher;

std_msgs__msg__Int32 robotStateMsg;
sensor_msgs__msg__Imu *imuMsg;
sensor_msgs__msg__JointState jointStateMsg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t imuMsgPublisherTimer;
rcl_timer_t jointStateMsgPublisherTimer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){errorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

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
void imuMsgTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
/**************************************************************
   imuMsgTimerCallback()
 **************************************************************/
void imuMsgTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    struct timespec tv = {0};
    clock_gettime(0, &tv);

    if (timer != NULL)
    {
      imuMsg->header.stamp.nanosec = tv.tv_nsec;
      imuMsg->header.stamp.sec = tv.tv_sec;

      imuMsg->orientation.x = quatX;
      imuMsg->orientation.y = quatY;
      imuMsg->orientation.z = quatZ;
      imuMsg->orientation.w = quatW;

      imuMsg->angular_velocity.x = xVelocity;
      imuMsg->angular_velocity.y = yVelocity;
      imuMsg->angular_velocity.z = zVelocity;

      imuMsg->linear_acceleration.x = xAcc;
      imuMsg->linear_acceleration.y = yAcc;
      imuMsg->linear_acceleration.z = zAcc;
      
      RCSOFTCHECK(rcl_publish(&imuMsgPublisher, imuMsg, NULL));
    }
}

// For some reason, without this duplicate method definition, the dumb-ass
// Arduino compiler wouldn't not do it's job.
void jointStateMsgTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
/**************************************************************
   jointStateMsgTimerCallback()
 **************************************************************/
void jointStateMsgTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  struct timespec tv = {0};
  clock_gettime(0, &tv);

  if (timer != NULL)
  {
    // Joint State Message Header
    jointStateMsg.header.stamp.nanosec = tv.tv_nsec;
    jointStateMsg.header.stamp.sec = tv.tv_sec;

    jointStateMsg.header.frame_id.data = (char*)malloc(100 * sizeof(char));
    char frameIdString[] = "joint_state";
    memcpy(jointStateMsg.header.frame_id.data, frameIdString, strlen(frameIdString) + 1);
    jointStateMsg.header.frame_id.size = strlen(jointStateMsg.header.frame_id.data);
    jointStateMsg.header.frame_id.capacity = 100;

    // Initialize data sizes for Left and Right Joints
    jointStateMsg.name.size = 2;
    jointStateMsg.velocity.size = 2;
    jointStateMsg.position.size = 2;
    jointStateMsg.effort.size = 2;

    // Allocate memory for message data
    jointStateMsg.name.capacity = 100;
    jointStateMsg.name.data = (rosidl_runtime_c__String*) malloc(jointStateMsg.name.capacity * sizeof(rosidl_runtime_c__String));

    jointStateMsg.velocity.capacity = 100;
    jointStateMsg.velocity.data = (double*) malloc(jointStateMsg.velocity.capacity * sizeof(double));

    jointStateMsg.position.capacity = 100;
    jointStateMsg.position.data = (double*) malloc(jointStateMsg.position.capacity * sizeof(double));

    jointStateMsg.effort.capacity = 100;
    jointStateMsg.effort.data = (double*) malloc(jointStateMsg.effort.capacity * sizeof(double));

    // Left Joint
    jointStateMsg.name.data[0].capacity = 100;
    jointStateMsg.name.data[0].data = (char*) malloc(jointStateMsg.name.data[0].capacity * sizeof(char));
    strcpy(jointStateMsg.name.data[0].data, "drivewhl_l_joint");
    jointStateMsg.name.data[0].size = strlen(jointStateMsg.name.data[0].data);

    jointStateMsg.velocity.data[0] = angularVelocityLeft;
    jointStateMsg.position.data[0] = 0.0;
    jointStateMsg.effort.data[0] = 0.0;

    // Right Joint
    jointStateMsg.name.data[1].capacity = 100;
    jointStateMsg.name.data[1].data = (char*) malloc(jointStateMsg.name.data[1].capacity * sizeof(char));
    strcpy(jointStateMsg.name.data[1].data, "drivewhl_r_joint");
    jointStateMsg.name.data[1].size = strlen(jointStateMsg.name.data[1].data);

    jointStateMsg.velocity.data[1] = angularVelocityRight;
    jointStateMsg.position.data[1] = 0.0;
    jointStateMsg.effort.data[1] = 0.0;

    RCSOFTCHECK(rcl_publish(&jointStateMsgPublisher, &jointStateMsg, NULL));
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
   createRobotStateSubscriber()
 **************************************************************/
void createRobotStateSubscriber()
{
  RCCHECK(rclc_subscription_init_default(
            &robotStateSubscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "robot_state"));
}

/**************************************************************
   createImuDataMsgPublisher()
 **************************************************************/
void createImuDataMsgPublisher()
{
  RCCHECK(rclc_publisher_init_default(
            &imuMsgPublisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "/imu/data"));

  const unsigned int imu_msg_timer_timeout = 250;
  RCCHECK(rclc_timer_init_default(
            &imuMsgPublisherTimer,
            &support,
            RCL_MS_TO_NS(imu_msg_timer_timeout),
            imuMsgTimerCallback));
}

/**************************************************************
   initializeImuMessage()
 **************************************************************/
void initializeImuMessage()
{
  imuMsg = sensor_msgs__msg__Imu__create();

  imuMsg->header.frame_id.data = (char*)malloc(100 * sizeof(char));
  char frameIdString[] = "imu";
  memcpy(imuMsg->header.frame_id.data, frameIdString, strlen(frameIdString) + 1);
  imuMsg->header.frame_id.size = strlen(imuMsg->header.frame_id.data);
  imuMsg->header.frame_id.capacity = 100;

  imuMsg->orientation_covariance[0] = -1;
  imuMsg->angular_velocity_covariance[0] = -1;
  imuMsg->linear_acceleration_covariance[0] = -1;
}

/**************************************************************
   createJointStateMsgPublisher()
 **************************************************************/
void createJointStateMsgPublisher()
{
  RCCHECK(rclc_publisher_init_default(
            &jointStateMsgPublisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "/joint_states"));

  const unsigned int joint_state_msg_timer_timeout = 250;
  RCCHECK(rclc_timer_init_default(
            &jointStateMsgPublisherTimer,
            &support,
            RCL_MS_TO_NS(joint_state_msg_timer_timeout),
            jointStateMsgTimerCallback));
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

  createRobotStateSubscriber();

  createImuDataMsgPublisher();
  //createJointStateMsgPublisher();

  // Create Executor -- MAY NEED TO UPDATE THE MAGIC NUMBER BELOW !!!!!!
  // *** The magic number equals the number of timers plus the number of subscribers ***
  // *** Publishers don't factor into this number, I guess.                           ***
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  initializeImuMessage();

  RCCHECK(rclc_executor_add_timer(&executor, &imuMsgPublisherTimer));
  //RCCHECK(rclc_executor_add_timer(&executor, &jointStateMsgPublisherTimer));

  RCCHECK(rclc_executor_add_subscription(&executor, &robotStateSubscriber, &robotStateMsg, &robotStateSubscriptionCallback, ON_NEW_DATA));
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
