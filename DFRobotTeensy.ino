/**************************************************************
   DFRobot
 **************************************************************/

static const int ROBOT_STATE_DISABLED = 0;
static const int ROBOT_STATE_ENABLED = 1;

int robotState = ROBOT_STATE_DISABLED;

/**************************************************************
   setup()
 **************************************************************/
void setup() 
{ 
  //Serial.begin(115200);

  ros2HandlerSetup();
  motorControlSetup();
  controllerHandlingSetup();
  //imuSetup();
  
} 

/**************************************************************
   loop()
 **************************************************************/
void loop() 
{

  ros2HandlerLoop();

  //if (robotState == ROBOT_STATE_ENABLED)
  {
     motorControlLoop();
     controllerHandlingLoop();
     //imuLoop();
  }
}
