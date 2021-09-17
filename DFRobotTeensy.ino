/**************************************************************
   DFRobot
 **************************************************************/
#include <Arduino.h>
#include <TeensyThreads.h>

static const int ROBOT_STATE_DISABLED = 0;
static const int ROBOT_STATE_TELEOP_ENABLED = 1;
static const int ROBOT_STATE_AUTONOMOUS_ENABLED = 2;

int robotState = ROBOT_STATE_DISABLED;

/**************************************************************
   setup()
 **************************************************************/
void setup()
{
  ros2HandlerSetup();
  motorControlSetup();
  controllerHandlingSetup();
  //imuSetup();
  
  threads.addThread(ros2HandlerLoop);
}

/**************************************************************
   loop()
 **************************************************************/
void loop()
{ 
  switch (robotState)
  {
    case ROBOT_STATE_DISABLED:
      stopMotors();
      break;

    case ROBOT_STATE_TELEOP_ENABLED:
      controllerHandlingLoop();
      break;

    case ROBOT_STATE_AUTONOMOUS_ENABLED:
      //imuLoop();
       break;

    default:
      stopMotors();
  }
}
