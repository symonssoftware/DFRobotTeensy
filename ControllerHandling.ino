/**************************************************************
   CoontrollerHandling
 **************************************************************/

/* Radio link channels */
static const int RIGHT_STICK_X = 1;
static const int LEFT_STICK_Y = 3;
static const int RIGHT_STICK_Y = 2;
static const int LEFT_STICK_X = 4;
static const int RIGHT_TOGGLE = 5;
static const int BUTTON = 6;
static const int LEFT_TOGGLE = 7;
static const int DIAL = 8;

static int channels[18];

static const int RADIOLINK_CONTROLLER_MINIMUM_VALUE = 200;
static const int RADIOLINK_CONTROLLER_NEUTRAL_VALUE = 1000;
static const int RADIOLINK_CONTROLLER_MAXIMUM_VALUE = 1800;

int radioLinkDriveX = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
int radioLinkDriveY = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
int radioLinkControlX = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
int radioLinkControlY = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
int radioLinkToggleRight = -1;
int radioLinkToggleLeft = -1;
int radioLinkButton = -1;

static byte sBusBuffer[25];
static int sBusPacketsLost = 0;

/**************************************************************
   controllerHandlingSetup()
 **************************************************************/
void controllerHandlingSetup()
{
  //The SBUS is a non standard baud rate of 100 kbs
  Serial4.begin(100000, SERIAL_8E2);
  // put your setup code here, to run once:
  Serial4.flush();
}

/**************************************************************
   controllerHandlingLoop()
 **************************************************************/
void controllerHandlingLoop()
{
  processControllerData();
}

/**************************************************************
   processControllerData()
 **************************************************************/
void processControllerData()
{
  static int sBusErrors = 0;
  static int sBusByteIndex = 0;
  byte nextSBusByte;

  //Check the SBus serial port for incoming SBus data
  if (Serial4.available ())
  {
    nextSBusByte = Serial4.read ();
    //handle drive motors is called from inside processControllerData();
    //This is a new package and it's not the first byte then it's probably the start byte B11110000 (sent MSB)
    //so start reading the 25 byte packet
    if ((sBusByteIndex == 0) && (nextSBusByte != 0x0F))
    {
      // Serial.println("Enabled");
      // error - keep waiting for the start byte
    }
    else
    {
      sBusBuffer[sBusByteIndex++] = nextSBusByte;  // fill the buffer with the bytes until the end byte B0000000 is received
    }

    // If we've got 25 bytes then this is a good packet so start to decode
    if (sBusByteIndex == 25)
    {
      sBusByteIndex = 0;

      if (sBusBuffer[24] == 0x00)
      {
        processSBusBuffer();
        handleDriveMotors();
      }
      else
      {
        Serial.println("Error!");
        sBusErrors++; //?????
      }
    }
  }
}

/**************************************************************
   processSBusBuffer()
 **************************************************************/
void processSBusBuffer()
{
  // 25 byte packet received is little endian. Details of how the package is explained on this website:
  // http://www.robotmaker.eu/ROBOTmaker/quadcopter-3d-proximity-sensing/sbus-graphical-representation

  channels[1]  = ((sBusBuffer[1]       | sBusBuffer[2] << 8)                        & 0x07FF);
  channels[2]  = ((sBusBuffer[2] >> 3  | sBusBuffer[3] << 5)                        & 0x07FF);
  channels[3]  = ((sBusBuffer[3] >> 6  | sBusBuffer[4] << 2 | sBusBuffer[5] << 10)  & 0x07FF);
  channels[4]  = ((sBusBuffer[5] >> 1  | sBusBuffer[6] << 7)                        & 0x07FF);
  channels[5]  = ((sBusBuffer[6] >> 4  | sBusBuffer[7] << 4)                        & 0x07FF);
  channels[6]  = ((sBusBuffer[7] >> 7  | sBusBuffer[8] << 1 | sBusBuffer[9] << 9)   & 0x07FF);
  channels[7]  = ((sBusBuffer[9] >> 2  | sBusBuffer[10] << 6)                       & 0x07FF);
  channels[8]  = ((sBusBuffer[10] >> 5 | sBusBuffer[11] << 3)                       & 0x07FF);

/*  
    Serial.print("CH1: ");
    Serial.println(channels[1]);
    Serial.print("CH2: ");
    Serial.println(channels[2]);
    Serial.print("CH3: ");
    Serial.println(channels[3]);
    Serial.print("CH4: ");
    Serial.println(channels[4]);
    Serial.print("CH5: ");
    Serial.println(channels[5]);
    Serial.print("CH6: ");
    Serial.println(channels[6]);
    Serial.print("CH7: ");
    Serial.println(channels[7]);
    Serial.print("CH8: ");
    Serial.println(channels[8]);
 */
  // Check for signal loss
  if ((sBusBuffer[23] >> 2) & 0x0001)
  {
    sBusPacketsLost++;
    //Serial.print("Signal Lost: ");
    //Serial.println(sBusPacketsLost);

    //if (receivedOneSBusPacketSinceReset) {
    // failSafeEnabled = true;
    //}

    // Make sure the robot doesn't move when the signal is lost
    /*radioLinkTurnValue = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
      radioLinkThrottleValue = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
      radioLinkStrafeValue = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;*/
  }
  else // Everything is okay so process the current values
  {
    //receivedOneSBusPacketSinceReset = true;
    //wdt_reset();

    //TODO TOGGLES AND BUTTON
    radioLinkDriveX = channels[RIGHT_STICK_X];
    radioLinkDriveY = channels[RIGHT_STICK_Y];
    radioLinkControlX = channels[LEFT_STICK_X];
    radioLinkControlY = channels[LEFT_STICK_Y];
    radioLinkToggleRight = channels[RIGHT_TOGGLE];
    radioLinkToggleLeft = channels[LEFT_TOGGLE];
    radioLinkButton = channels[BUTTON];

  }
}
