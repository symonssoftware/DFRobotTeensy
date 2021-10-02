/**************************************************************
   IMU
  **************************************************************/
#include <Wire.h>
#include <JY901.h>

const float DEG2RAD = PI / 180.0f;

const unsigned long IMU_EVENT_INTERVAL = 100;
unsigned long imuPreviousTime = 0;

float xAxis = 0.0;
float yAxis = 0.0;
float zAxis = 0.0;

float quatX = 0.0;
float quatY = 0.0;
float quatZ = 0.0;
float quatW = 0.0;

float xVelocity = 0.0;
float yVelocity = 0.0;
float zVelocity = 0.0;

float xAcc = 0.0;
float yAcc = 0.0;
float zAcc = 0.0;

/**************************************************************
   imuSetup()
 **************************************************************/
void imuSetup()
{
  Serial3.begin(115200);

  // Initialize Z-axis to 0
  char angleInitCmdString[3] = {0xff, 0xaa, 0x52};
  Serial3.write(angleInitCmdString);
}

/**************************************************************
   imuLoop()
 **************************************************************/
void imuLoop()
{
  unsigned long currentTime = millis();

  if (currentTime - imuPreviousTime >= IMU_EVENT_INTERVAL)
  {
    imuPreviousTime = currentTime;

    while (Serial3.available())
    {
      JY901.CopeSerialData(Serial3.read());
    }
      
    xVelocity = (float)JY901.stcGyro.w[0] / 32768.0 * 2000.0;
    yVelocity = (float)JY901.stcGyro.w[1] / 32768.0 * 2000.0;
    zVelocity = (float)JY901.stcGyro.w[2] / 32768.0 * 2000.0;

    xAcc = (float)JY901.stcAcc.a[0] / 32768.0 * 16.0;
    yAcc = (float)JY901.stcAcc.a[1] / 32768.0 * 16.0;
    zAcc = (float)JY901.stcAcc.a[2] / 32768.0 * 16.0;
  
    xAxis = (float)JY901.stcAngle.Angle[0] / 32768.0 * 180.0;
    yAxis = (float)JY901.stcAngle.Angle[1] / 32768.0 * 180.0;
    zAxis = (float)JY901.stcAngle.Angle[2] / 32768.0 * 180.0;

    float yaw = zAxis * DEG2RAD;
    float pitch = yAxis * DEG2RAD;
    float roll = xAxis * DEG2RAD;
    
    float rollOver2 = roll * 0.5f;
    float sinRollOver2 = (float)sin((double)rollOver2);
    float cosRollOver2 = (float)cos((double)rollOver2);
    
    float pitchOver2 = pitch * 0.5f;
    float sinPitchOver2 = (float)sin((double)pitchOver2);
    float cosPitchOver2 = (float)cos((double)pitchOver2);
    
    float yawOver2 = yaw * 0.5f;
    float sinYawOver2 = (float)sin((double)yawOver2);
    float cosYawOver2 = (float)cos((double)yawOver2);
    
    quatW = cosYawOver2 * cosPitchOver2 * cosRollOver2 + sinYawOver2 * sinPitchOver2 * sinRollOver2;
    quatX = cosYawOver2 * sinPitchOver2 * cosRollOver2 + sinYawOver2 * cosPitchOver2 * sinRollOver2;
    quatY = sinYawOver2 * cosPitchOver2 * cosRollOver2 - cosYawOver2 * sinPitchOver2 * sinRollOver2;
    quatZ = cosYawOver2 * cosPitchOver2 * sinRollOver2 - sinYawOver2 * sinPitchOver2 * cosRollOver2;

    //Serial.print("Z-Axis: ");
    //Serial.println(zAxis);
  }
}
