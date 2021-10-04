/**************************************************************
   MotorControl
 **************************************************************/

static const int LEFT_MOTOR_SPEED_CTRL_PIN = 22;     
static const int RIGHT_MOTOR_SPEED_CTRL_PIN = 23;     

static const int LEFT_MOTOR_DIRECTION_CTRL_PIN = 33; 
static const int RIGHT_MOTOR_DIRECTION_CTRL_PIN = 36; 

static const int LEFT_MOTOR_ENCODER_A_PIN = 34;      
static const int LEFT_MOTOR_ENCODER_B_PIN = 35;      
static const int RIGHT_MOTOR_ENCODER_A_PIN = 37;      
static const int RIGHT_MOTOR_ENCODER_B_PIN = 38;      

static const int DRIVE_SPEED = 200; //255 is max;
static const int TURN_SPEED = 200;

static const int MAX_MOTOR_SPEED = 220;
static const int MIN_MOTOR_SPEED = 0;
static const int MOTOR_DEADBAND = 10;

static const int ENC_COUNT_PER_REV = 663;

static const float RPM_TO_RADIANS = 0.10471975512;

byte leftEncoderPinALast;
byte rightEncoderPinALast;

int leftEncoderPulseCount;
int rightEncoderPulseCount;

boolean leftEncoderDirection;
boolean rightEncoderDirection;

int encoderCalculationInterval = 1000; // One second intervals
long encoderCalculationPreviousMillis = 0;
long encoderCalculationCurrentMillis = 0;

float rpmRight = 0.0;
float angularVelocityRight = 0.0;
float angularVelocityRightDegrees = 0.0;

float rpmLeft = 0.0;
float angularVelocityLeft = 0.0;
float angularVelocityLeftDegrees = 0.0;

/**************************************************************
   motorControlSetup()
 **************************************************************/
void motorControlSetup()
{
  pinMode(LEFT_MOTOR_SPEED_CTRL_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_SPEED_CTRL_PIN, OUTPUT);

  // Need to increase the PWM frequency beyond the audible
  // range to get rid of the annoying "whine"
  analogWriteFrequency(LEFT_MOTOR_SPEED_CTRL_PIN, 12500);
  analogWriteFrequency(RIGHT_MOTOR_SPEED_CTRL_PIN, 12500);
  
  pinMode(LEFT_MOTOR_DIRECTION_CTRL_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIRECTION_CTRL_PIN, OUTPUT);

  leftEncoderInit();
  rightEncoderInit();

  stopMotors();
}

/**************************************************************
   motorControlLoop()
 **************************************************************/
void motorControlLoop()
{
  encoderCalculationCurrentMillis = millis();

  if ((encoderCalculationCurrentMillis - encoderCalculationPreviousMillis) > encoderCalculationInterval) 
  {
    encoderCalculationPreviousMillis = encoderCalculationCurrentMillis;

    rpmRight = (float)(rightEncoderPulseCount * 60.0 / ENC_COUNT_PER_REV);
    angularVelocityRight = rpmRight * RPM_TO_RADIANS;   
    //angularVelocityRightDegrees = angularVelocityRight * RAD_TO_DEG;

    rpmLeft = (float)(rightEncoderPulseCount * 60.0 / ENC_COUNT_PER_REV);
    angularVelocityLeft = rpmLeft * RPM_TO_RADIANS;   
    ///angularVelocityLeftDegrees = angularVelocityLeft * RAD_TO_DEG;

    leftEncoderPulseCount = 0;
    rightEncoderPulseCount = 0;
  }
}

/**************************************************************
   leftEncoderInit()
 **************************************************************/
void leftEncoderInit()
{
  leftEncoderPulseCount = 0;
  leftEncoderDirection = true; //default -> Forward
  pinMode(LEFT_MOTOR_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_MOTOR_ENCODER_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_MOTOR_ENCODER_A_PIN), leftWheelPulse, CHANGE); 
}

/**************************************************************
   rightEncoderInit()
 **************************************************************/
void rightEncoderInit()
{
  rightEncoderPulseCount = 0;
  rightEncoderDirection = true; //default -> Forward
  pinMode(RIGHT_MOTOR_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_MOTOR_ENCODER_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RIGHT_MOTOR_ENCODER_A_PIN), rightWheelPulse, CHANGE);
}

/**************************************************************
   leftWheelPulse()
 **************************************************************/
void leftWheelPulse()
{
  int Lstate = digitalRead(LEFT_MOTOR_ENCODER_A_PIN);
  if((leftEncoderPinALast == LOW) && (Lstate == HIGH))
  {
    int val = digitalRead(LEFT_MOTOR_ENCODER_B_PIN);
    
    if(val == LOW && leftEncoderDirection)
    {
      leftEncoderDirection = false; //Reverse
    }
    else if(val == HIGH && !leftEncoderDirection)
    {
      leftEncoderDirection = true;  //Forward
    }
  }
  
  leftEncoderPinALast = Lstate;

  if(leftEncoderDirection)
  {
    leftEncoderPulseCount--;
  }
  else  
  {
    leftEncoderPulseCount++;
  }
}

/**************************************************************
   rightWheelSpeed()
 **************************************************************/
void rightWheelPulse()
{
  int Lstate = digitalRead(RIGHT_MOTOR_ENCODER_A_PIN);
  if((rightEncoderPinALast == LOW) && (Lstate == HIGH))
  {
    int val = digitalRead(RIGHT_MOTOR_ENCODER_B_PIN);
    
    if(val == LOW && rightEncoderDirection)
    {
      rightEncoderDirection = false; //Reverse
    }
    else if(val == HIGH && !rightEncoderDirection)
    {
      rightEncoderDirection = true;  //Forward
    }
  }
  
  rightEncoderPinALast = Lstate;

  if(rightEncoderDirection)
  {
    rightEncoderPulseCount--;
  }
  else  
  {
    rightEncoderPulseCount++;
  }
}

/**************************************************************
   stopMotors()
 **************************************************************/
void stopMotors()
{
  analogWrite(LEFT_MOTOR_SPEED_CTRL_PIN, 0);
  digitalWrite(LEFT_MOTOR_DIRECTION_CTRL_PIN, LOW);
  analogWrite(RIGHT_MOTOR_SPEED_CTRL_PIN, 0);
  digitalWrite(RIGHT_MOTOR_DIRECTION_CTRL_PIN, LOW);
}

/**************************************************************
   moveForward()
 **************************************************************/
void moveForward()
{
  analogWrite (LEFT_MOTOR_SPEED_CTRL_PIN, DRIVE_SPEED);
  digitalWrite(LEFT_MOTOR_DIRECTION_CTRL_PIN, HIGH);
  analogWrite (RIGHT_MOTOR_SPEED_CTRL_PIN, DRIVE_SPEED);
  digitalWrite(RIGHT_MOTOR_DIRECTION_CTRL_PIN, HIGH);
}

/**************************************************************
   moveBackward()
 **************************************************************/
void moveBackward()
{
  analogWrite (LEFT_MOTOR_SPEED_CTRL_PIN, DRIVE_SPEED);
  digitalWrite(LEFT_MOTOR_DIRECTION_CTRL_PIN, LOW);
  analogWrite (RIGHT_MOTOR_SPEED_CTRL_PIN, DRIVE_SPEED);
  digitalWrite(RIGHT_MOTOR_DIRECTION_CTRL_PIN, LOW);
}

/**************************************************************
   turnLeft()
 **************************************************************/
void turnLeft()
{
  analogWrite (LEFT_MOTOR_SPEED_CTRL_PIN, TURN_SPEED);
  digitalWrite(LEFT_MOTOR_DIRECTION_CTRL_PIN, LOW);
  analogWrite (RIGHT_MOTOR_SPEED_CTRL_PIN, TURN_SPEED);
  digitalWrite(RIGHT_MOTOR_DIRECTION_CTRL_PIN, HIGH);
}

/**************************************************************
   turnRight()
 **************************************************************/
void turnRight()
{
  analogWrite (LEFT_MOTOR_SPEED_CTRL_PIN, TURN_SPEED);
  digitalWrite(LEFT_MOTOR_DIRECTION_CTRL_PIN, HIGH);
  analogWrite (RIGHT_MOTOR_SPEED_CTRL_PIN, TURN_SPEED);
  digitalWrite(RIGHT_MOTOR_DIRECTION_CTRL_PIN, LOW);
}


/**************************************************************
   moveLeftMotor()
 **************************************************************/
 void moveLeftMotor(boolean fwdDirection, int speed)
{
  analogWrite (LEFT_MOTOR_SPEED_CTRL_PIN, speed);

  if (fwdDirection)
  {
    digitalWrite(LEFT_MOTOR_DIRECTION_CTRL_PIN, HIGH);
  }
  else
  {
    digitalWrite(LEFT_MOTOR_DIRECTION_CTRL_PIN, LOW);
  }
}

/**************************************************************
   moveRightMotor()
 **************************************************************/
 void moveRightMotor(boolean fwdDirection, int speed)
{
  analogWrite (RIGHT_MOTOR_SPEED_CTRL_PIN, speed);

  if (fwdDirection)
  {
    digitalWrite(RIGHT_MOTOR_DIRECTION_CTRL_PIN, HIGH);
  }
  else
  {
    digitalWrite(RIGHT_MOTOR_DIRECTION_CTRL_PIN, LOW);
  }
}


/**************************************************************
   handleDriveMotors()
 **************************************************************/
void handleDriveMotors()
{
  float throttle = -map(radioLinkDriveY, 200, 1800, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  float turn = -map(radioLinkDriveX, 200, 1800, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

  int tempRight = ((MAX_MOTOR_SPEED - abs(turn)) * (throttle / (float)MAX_MOTOR_SPEED)) + throttle;
  int tempLeft = ((MAX_MOTOR_SPEED - abs(throttle)) * (turn / (float)MAX_MOTOR_SPEED)) + turn;

  int right = (tempRight + tempLeft) / 2;
  int left = (tempRight - tempLeft) / 2;
  
  if ((left > MOTOR_DEADBAND) && (left <= MAX_MOTOR_SPEED))
  {
    moveLeftMotor(true, left);
  }
  else if ((left < -MOTOR_DEADBAND) && (left >= -MAX_MOTOR_SPEED))
  {
    moveLeftMotor(false, -left);
  }
  else
  {
    moveLeftMotor(false, 0);
  }
  
  if ((right > MOTOR_DEADBAND) && (right <= MAX_MOTOR_SPEED))
  {
    moveRightMotor(true, right);
  }
  else if ((right < -MOTOR_DEADBAND) && (right >= -MAX_MOTOR_SPEED))
  {
    moveRightMotor(false, -right);
  }
  else
  {
    moveRightMotor(false, 0);
  }
}
