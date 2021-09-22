/**************************************************************
   IMU
  **************************************************************/
#include <Wire.h>
#include <JY901.h>

const unsigned long imuEventInterval = 100;
unsigned long imuPreviousTime = 0;
float zAxis = 0.0;

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

  if (currentTime - imuPreviousTime >= imuEventInterval)
  {
    imuPreviousTime = currentTime;

    while (Serial3.available())
    {
      JY901.CopeSerialData(Serial3.read());
    }

    zAxis = (float)JY901.stcAngle.Angle[2] / 32768 * 180;
    //Serial.print("Z-Axis: ");
    //Serial.println(zAxis);
  }
}
