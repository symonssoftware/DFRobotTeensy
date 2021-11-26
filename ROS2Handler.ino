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
#include <example_interfaces/msg/float32.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist.h>

#include <std_msgs/msg/header.h>
#include <builtin_interfaces/msg/time.h>

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>

rcl_subscription_t robotStateSubscriber;
rcl_subscription_t velocitySubscriber;
rcl_subscription_t clockSubscriber;

rcl_publisher_t imuMsgPublisher;
rcl_publisher_t jointStateMsgPublisher;

std_msgs__msg__Int32 robotStateMsg;
builtin_interfaces__msg__Time clockMsg;

sensor_msgs__msg__Imu *imuMsg;
sensor_msgs__msg__JointState *jointStateMsg;
geometry_msgs__msg__Twist velocityTwistMsg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t imuMsgPublisherTimer;
rcl_timer_t jointStateMsgPublisherTimer;

int32_t currentClockSeconds;
uint32_t currentClockNanoseconds;

static const double WHEEL_BASE_METERS = 0.26;
static const double WHEEL_RADIUS_METERS = 0.07;

const unsigned long CMD_VEL_MSG_INTERVAL = 1000;
unsigned long cmdVelMsgPreviousTime = 0;
unsigned long currentTime;

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
   robotStateSubscriptionCallback()
 **************************************************************/
void robotStateSubscriptionCallback(const void * msgin)
{
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

  robotState = msg->data;
}

/**************************************************************
   createClockSubscriber()
 **************************************************************/
void createClockSubscriber()
{
  RCCHECK(rclc_subscription_init_default(
            &clockSubscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(builtin_interfaces, msg, Time),
            "/pumpkin/clock"));
}

/**************************************************************
   clockSubscriptionCallback()
 **************************************************************/
void clockSubscriptionCallback(const void * msgin)
{
  const builtin_interfaces__msg__Time *msg = (const builtin_interfaces__msg__Time *)msgin;

  currentClockSeconds = msg->sec;
  currentClockNanoseconds = msg->nanosec;
}

/**************************************************************
   createVelocitySubscriber()
 **************************************************************/
void createVelocitySubscriber()
{
  RCCHECK(rclc_subscription_init_default(
            &velocitySubscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel"));
}

/**************************************************************
   velocitySubscriptionCallback()
 **************************************************************/
void velocitySubscriptionCallback(const void * msgin)
{
  if (robotState > ROBOT_STATE_DISABLED)
  {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    // msg->linear.x (range of -1.0 to 1.0)
    // msg->angular.z (range of 1.0 to -1.0)

    double speedLeft = 0.0;
    double speedRight = 0.0;

    // When commanded to perform a turn only, there isn't enough juice sent to the wheels
    // with the current approach to get them to move on carpet at least.
    if ((msg->linear.x == 0.0) && (msg->angular.z >= 0.99))
    {
      speedLeft = -3.0;
      speedRight = 3.0;
    }
    else if ((msg->linear.x == 0.0) && (msg->angular.z <= -0.99))
    {
      speedLeft =  3.0;
      speedRight = -3.0;
    }
    else
    {
      // https://answers.ros.org/question/334022/how-to-split-cmd_vel-into-left-and-right-wheel-of-2wd-robot/

      speedLeft = ((msg->linear.x - (msg->angular.z * WHEEL_BASE_METERS / 2.0)) / WHEEL_RADIUS_METERS);
      speedRight = ((msg->linear.x + (msg->angular.z * WHEEL_BASE_METERS / 2.0)) / WHEEL_RADIUS_METERS);
    }

    // Handle Right Motor
    bool rightMotorDirection = (speedRight > 0);
    double calcSpeedRight = mapf(abs(speedRight), 0.0, 16.14286, MIN_AUTO_MOTOR_SPEED, MAX_AUTO_MOTOR_SPEED);
    moveRightMotor(rightMotorDirection, calcSpeedRight);

    // Handle Left Motor
    bool leftMotorDirection = (speedLeft > 0);
    double calcSpeedLeft = mapf(abs(speedLeft), 0.0, 16.14286, MIN_AUTO_MOTOR_SPEED, MAX_AUTO_MOTOR_SPEED);
    moveLeftMotor(leftMotorDirection, calcSpeedLeft);

    cmdVelMsgPreviousTime = currentTime;
  }
  else
  {
    stopMotors();
  }
}

/**************************************************************
   mapf()
 **************************************************************/
double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
    imuMsg->header.stamp.nanosec = currentClockNanoseconds;
    imuMsg->header.stamp.sec = currentClockSeconds;

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

  const unsigned int imu_msg_timer_timeout = 30;
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
  char frameIdString[] = "imu_link";
  memcpy(imuMsg->header.frame_id.data, frameIdString, strlen(frameIdString) + 1);
  imuMsg->header.frame_id.size = strlen(imuMsg->header.frame_id.data);
  imuMsg->header.frame_id.capacity = 100;

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

  //imuMsg->orientation_covariance[0] = -1;
  //imuMsg->angular_velocity_covariance[0] = -1;
  //imuMsg->linear_acceleration_covariance[0] = -1;
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
    jointStateMsg->header.stamp.nanosec = currentClockNanoseconds;
    jointStateMsg->header.stamp.sec = currentClockSeconds;

    jointStateMsg->velocity.data[0] = angularVelocityLeft;
    jointStateMsg->velocity.data[1] = angularVelocityRight;

    jointStateMsg->position.data[0] = positionLeft;
    jointStateMsg->position.data[1] = positionRight;

    RCSOFTCHECK(rcl_publish(&jointStateMsgPublisher, jointStateMsg, NULL));
  }
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

  const unsigned int joint_state_msg_timer_timeout = 30;
  RCCHECK(rclc_timer_init_default(
            &jointStateMsgPublisherTimer,
            &support,
            RCL_MS_TO_NS(joint_state_msg_timer_timeout),
            jointStateMsgTimerCallback));
}

/**************************************************************
   initializeJointStateMessage()
 **************************************************************/
void initializeJointStateMessage()
{
  jointStateMsg = sensor_msgs__msg__JointState__create();

  jointStateMsg->header.frame_id.data = (char*)malloc(100 * sizeof(char));
  char frameIdString[] = "";
  memcpy(jointStateMsg->header.frame_id.data, frameIdString, strlen(frameIdString) + 1);
  jointStateMsg->header.frame_id.size = strlen(jointStateMsg->header.frame_id.data);
  jointStateMsg->header.frame_id.capacity = 100;

  rosidl_runtime_c__String__Sequence__init(&jointStateMsg->name, 2);
  rosidl_runtime_c__double__Sequence__init(&jointStateMsg->velocity, 2);
  rosidl_runtime_c__double__Sequence__init(&jointStateMsg->position, 2);
  rosidl_runtime_c__double__Sequence__init(&jointStateMsg->effort, 2);

  // Initialize data sizes for Left and Right Joints
  jointStateMsg->name.size = 2;
  jointStateMsg->velocity.size = 2;
  jointStateMsg->position.size = 2;
  jointStateMsg->effort.size = 2;

  // Allocate memory for message data
  jointStateMsg->name.capacity = 100;
  jointStateMsg->name.data = (rosidl_runtime_c__String*) malloc(jointStateMsg->name.capacity * sizeof(rosidl_runtime_c__String));

  jointStateMsg->velocity.capacity = 100;
  jointStateMsg->velocity.data = (double*) malloc(jointStateMsg->velocity.capacity * sizeof(double));

  jointStateMsg->position.capacity = 100;
  jointStateMsg->position.data = (double*) malloc(jointStateMsg->position.capacity * sizeof(double));

  jointStateMsg->effort.capacity = 100;
  jointStateMsg->effort.data = (double*) malloc(jointStateMsg->effort.capacity * sizeof(double));

  // Left Joint
  jointStateMsg->name.data[0].capacity = 100;
  jointStateMsg->name.data[0].data = (char*) malloc(jointStateMsg->name.data[0].capacity * sizeof(char));
  strcpy(jointStateMsg->name.data[0].data, "drivewhl_l_joint");
  jointStateMsg->name.data[0].size = strlen(jointStateMsg->name.data[0].data);

  jointStateMsg->velocity.data[0] = angularVelocityLeft;
  jointStateMsg->position.data[0] = positionLeft;
  jointStateMsg->effort.data[0] = 0.0;

  // Right Joint
  jointStateMsg->name.data[1].capacity = 100;
  jointStateMsg->name.data[1].data = (char*) malloc(jointStateMsg->name.data[1].capacity * sizeof(char));
  strcpy(jointStateMsg->name.data[1].data, "drivewhl_r_joint");
  jointStateMsg->name.data[1].size = strlen(jointStateMsg->name.data[1].data);

  jointStateMsg->velocity.data[1] = angularVelocityRight;
  jointStateMsg->position.data[1] = positionRight;
  jointStateMsg->effort.data[1] = 0.0;
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
  createVelocitySubscriber();
  createClockSubscriber();

  createImuDataMsgPublisher();
  createJointStateMsgPublisher();

  // Create Executor -- MAY NEED TO UPDATE THE MAGIC NUMBER BELOW !!!!!!
  // *** The magic number equals the number of timers plus the number of subscribers ***
  // *** Publishers don't factor into this number, I guess.                           ***
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

  initializeImuMessage();
  initializeJointStateMessage();

  RCCHECK(rclc_executor_add_timer(&executor, &imuMsgPublisherTimer));
  RCCHECK(rclc_executor_add_timer(&executor, &jointStateMsgPublisherTimer));

  RCCHECK(rclc_executor_add_subscription(&executor, &clockSubscriber, &clockMsg, &clockSubscriptionCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &robotStateSubscriber, &robotStateMsg, &robotStateSubscriptionCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &velocitySubscriber, &velocityTwistMsg, &velocitySubscriptionCallback, ON_NEW_DATA));
}

/**************************************************************
   ros2HandlerLoop()
 **************************************************************/
void ros2HandlerLoop()
{
  currentTime = millis();

  // If we haven't received a new CMD_VEL msg in the expected time, stop the motors
  if (((currentTime - cmdVelMsgPreviousTime) >= CMD_VEL_MSG_INTERVAL) && (cmdVelMsgPreviousTime > 0))
  {
    stopMotors();
  }

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
