/**************************************************************
   DFRobot
 **************************************************************/

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
  //controllerHandlingSetup();
  //imuSetup();

}

/**************************************************************
   loop()
 **************************************************************/
void loop()
{ 
  switch (robotState)
  {
    case ROBOT_STATE_DISABLED:
      ros2HandlerLoop();
      motorControlLoop();
      stopMotors();
      break;

    case ROBOT_STATE_TELEOP_ENABLED:
      motorControlLoop();
      controllerHandlingLoop();
      // have to power-cycle to go to autonomous???
      break;

    case ROBOT_STATE_AUTONOMOUS_ENABLED:
      ros2HandlerLoop();
      motorControlLoop();
      //imuLoop();
      moveForward();
      break;

    default:
      //ros2HandlerLoop();
      stopMotors();
  }
}
