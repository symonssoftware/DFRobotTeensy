/**************************************************************
   IMU
  **************************************************************/
#include <Wire.h>
#include <JY901.h>

const float DEG2RAD = PI / 180.0f;

const unsigned long IMU_EVENT_INTERVAL = 30;
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
   euler_to_quat()
 **************************************************************/
const void euler_to_quat(float x, float y, float z, double* q) 
{
    float c1 = cos((y*3.14/180.0)/2);
    float c2 = cos((z*3.14/180.0)/2);
    float c3 = cos((x*3.14/180.0)/2);

    float s1 = sin((y*3.14/180.0)/2);
    float s2 = sin((z*3.14/180.0)/2);
    float s3 = sin((x*3.14/180.0)/2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3;
    q[1] = s1 * s2 * c3 + c1 * c2 * s3;
    q[2] = s1 * c2 * c3 + c1 * s2 * s3;
    q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

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

    //float yaw = zAxis; * DEG2RAD;
    //float pitch = yAxis; * DEG2RAD;
    //float roll = xAxis; * DEG2RAD;

    double q[4];
    euler_to_quat(xAxis, yAxis, zAxis, q);
    
    quatW = q[0];
    quatX = q[1];
    quatY = q[2];
    quatZ = q[3];

    //Serial.print("Z-Axis: ");
    //Serial.println(zAxis);
  }
}
