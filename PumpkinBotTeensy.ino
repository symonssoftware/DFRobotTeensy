#include <Metro.h>

/**************************************************************
   DFRobot
 **************************************************************/
#include <Arduino.h>

static const int ROBOT_STATE_DISABLED = 0;
static const int ROBOT_STATE_TELEOP_ENABLED = 1;
static const int ROBOT_STATE_AUTONOMOUS_ENABLED = 2;

int robotState = ROBOT_STATE_AUTONOMOUS_ENABLED;

/**************************************************************
   setup()
 **************************************************************/
void setup()
{
  //Serial.begin(9600);

  ros2HandlerSetup();
  motorControlSetup();
  controllerHandlingSetup();
  imuSetup();
}

/**************************************************************
   loop()
 **************************************************************/
void loop()
{  
  switch (robotState)
  {
    case ROBOT_STATE_DISABLED:
      imuLoop();
      stopMotors();
      ros2HandlerLoop();
      break;

    case ROBOT_STATE_TELEOP_ENABLED:
      imuLoop();
      controllerHandlingLoop();
      ros2HandlerLoop();
      break;

    case ROBOT_STATE_AUTONOMOUS_ENABLED:
      imuLoop();
      motorControlLoop();
      ros2HandlerLoop();
      break;

    default:
      stopMotors();
  }
}
