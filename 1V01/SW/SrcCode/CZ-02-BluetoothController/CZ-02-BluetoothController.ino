  /***********************************************************************************************/
/*
 *  \file       : CZ-02-BluetoothController.ino
 *  \date       : 02-DEC-2018 
 *  \author     : Vignesh S 
 *  \email      : vignura@gmail.com
 *  \copyright  : All Rights are Reserved 
/*
/***********************************************************************************************/
// headers
#include <SoftwareSerial.h>
#include <Relay.h>

/*************************************** Pin Mappings ******************************************/
// ground plane
#define GND_PLANE_PIN    PIN_A4

// user LEDS
#define USR_LED_1       PIN_A2
#define USR_LED_2       PIN_A3

// HC-05 
#define HC05_ST         PIN_A1    /* status pin */
#define HC05_EN         PIN_A5    /* enable pin */

// Relay 
#define RELAY           PIN_A0

// Emergency Stop 
#define EMG_STOP        4

/***********************************************************************************************/
/* comment the below macro to disable debug prints */
#define PRINT_DEBUG

/* comment the below macro to disable Bluetooth Response messages */
#define SEND_BTRES

#define MAX_DEBUG_MSG_SIZE                  128
#define MAX_CMD_STRING_SIZE                 32

#define SELF_TEST_COUNT                     0x01
#define HC05_BUAD_RATE                      9600
#define DEBUG_BUAD_RATE                     9600
#define HC05_SERIAL_READ_DELAY_MS           0x02

/* bluetooth command Strings*/
#define CMD_RELAY_ON                      "RlyOn"
#define CMD_RELAY_ON_TIMER                "RlyOn " /* RlyOn hh:mm:ss */
#define CMD_RELAY_OFF                     "RlyOff"
#define CMD_START_TEST                    "StTest"

/* bluetooth response Strings*/
#define RES_RELAY_ON                      "RlyOn"
#define RES_RELAY_OFF                     "RlyOff"
#define RES_START_TEST                    "StTest"
#define RES_EMG_STOP                      "EmgStop"

/* bluetooth command IDs */
#define CMD_INVALID_CMD_ID                  -1
#define CMD_RELAY_ON_ID                     0x01 
#define CMD_RELAY_ON_TIMER_ID               0x02
#define CMD_RELAY_OFF_ID                    0x03
#define CMD_START_TEST_ID                   0x04

/* Emergency stop Interrupt states */
#define EMG_STOP_INT_STATE_CLEAR            0x00
#define EMG_STOP_INT_STATE_SET              0x01

/****************************************** globals ********************************************/
/* SoftwareSerial (RX, TX) */
SoftwareSerial SS_Bluetooth(3, 4);

/* Set the relay to active low */
Relay MotorRly(RELAY, true);

#ifdef PRINT_DEBUG
  char g_arrcMsg[MAX_DEBUG_MSG_SIZE] = {0};
#endif

#ifdef SEND_BTRES
  char g_arrcBTMsg[MAX_CMD_STRING_SIZE] = {0};
#endif

unsigned long g_ulOnTimeSec = 0;
volatile int g_viEmgStopInt = 0;

/***********************************************************************************************/
/*! 
* \fn         :: setup()
* \author     :: Vignesh S
* \date       :: 02-DEC-2018
* \brief      :: This function performs the following initializations
*                1) Sets the Relay pins as output and sets its state to low
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void setup() {
  
  // Configure the Ground plane pin as input
  // do not change this pin sate after initialization
  pinMode(GND_PLANE_PIN, INPUT);

  // LED initialization
  pinMode(USR_LED_1, OUTPUT);
  pinMode(USR_LED_2, OUTPUT);

  // Relay initialization
  MotorRly.setState(RELAY_OFF);

  // Serial port initialization
  #ifdef PRINT_DEBUG
    Serial.begin( DEBUG_BUAD_RATE);
  #endif
  
  SS_Bluetooth.begin(HC05_BUAD_RATE);

  /* set the Emergency stop pin to input pullup */
  pinMode(EMG_STOP, INPUT_PULLUP);

  // Attach interrupt for Emergency stop.
  attachInterrupt(digitalPinToInterrupt(EMG_STOP), EmgStopInterrupt, FALLING);
    
  // perform self test
  //SelfTest(SELF_TEST_COUNT);

}

/***********************************************************************************************/
/*! 
* \fn         :: loop()
* \author     :: Vignesh S
* \date       :: 02-DEC-2018
* \brief      :: This functions is called repeatedly from a infinite for loop in main().
*                It does the following functions
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void loop() {

  char arrcCmd[MAX_CMD_STRING_SIZE] = {0};
  int iReadBytes = 0;
  int iCmdID = 0;

  // check for Emergency stop interrupt
  if(g_viEmgStopInt == EMG_STOP_INT_STATE_SET)
  {
    // clear the interrupt
    g_viEmgStopInt = EMG_STOP_INT_STATE_CLEAR;

    // turn off the relay
    CmdProcess(CMD_RELAY_OFF);

    #ifdef SEND_BTRES
      sprintf(g_arrcBTMsg, "%s", RES_EMG_STOP);
      SS_Bluetooth.println(g_arrcBTMsg);
    #endif
  }

  // read data from HC-05 if available
  iReadBytes = RecvCmd(arrcCmd, MAX_CMD_STRING_SIZE); 
  if(iReadBytes > 0)
  {
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg, "Received: [%d] %s", iReadBytes, arrcCmd);
      Serial.println(g_arrcMsg);
    #endif

    // validate the command
    if(isValidCmd(arrcCmd, iReadBytes, &iCmdID) == true)
    {
      // if valid command is received, process it
      CmdProcess(iCmdID);
    }
    else
    {
      // do nothing
    }
  }

  // run relay timer task
  MotorRly.TimerTask();

}

/***********************************************************************************************/
/*! 
* \fn         :: BuletoothTerminal()
* \author     :: Vignesh S
* \date       :: 03-DEC-2018
* \brief      :: This function perform bluethooth terminal emulation
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void BuletoothTerminal()
{
  // if data is available in bluetooth, send it to debug
  if(SS_Bluetooth.available())
  {
    Serial.write(SS_Bluetooth.read());
    // echo
    //SS_Bluetooth.write(SS_Bluetooth.read());
    //digitalWrite(LED_BUILTIN, HIGH);
  }

  // if data is available in debug, send it to bluetooth 
  if(Serial.available())
  {
    SS_Bluetooth.write(Serial.read());
    //digitalWrite(LED_BUILTIN, LOW);
  }
}

/***********************************************************************************************/
/*! 
* \fn         :: SelfTest()
* \author     :: Vignesh S
* \date       :: 02-DEC-2018
* \brief      :: This function perform selftest of onboard peripherals
* \param[in]  :: iTestCount (number of times to repeat the test)
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void SelfTest(int iTestCount)
{
  int iCount = 0;

  #ifdef PRINT_DEBUG
    sprintf(g_arrcMsg, "Performing Self Test..\r\nTest Count: %d", iTestCount);
    Serial.println(g_arrcMsg);
  #endif

  for(iCount = 0; iCount < iTestCount; iCount++)
  {
    // turn on the two LEDs
    digitalWrite(USR_LED_1, HIGH);
    digitalWrite(USR_LED_2, HIGH);
    // turn on Relay
    MotorRly.setState(RELAY_ON);    

    delay(10000);
    
    // turn off the two LEDs
    digitalWrite(USR_LED_1, LOW);
    digitalWrite(USR_LED_2, LOW);
    // turn off Relay
    MotorRly.setState(RELAY_OFF);

    delay(500);   
  }
}

/***********************************************************************************************/
/*! 
* \fn         :: PrintBytes()
* \author     :: Vignesh S
* \date       :: 02-DEC-2018
* \brief      :: This function prints the buffer byte by byte 
* \param[in]  :: ucBuffer 
* \param[in]  :: iBuflen
* \return     :: None
*/
/***********************************************************************************************/
void PrintBytes(unsigned char *ucBuffer, int iBuflen)
{
  int iIndex = 0;

  if(ucBuffer == NULL)
  {
    return;
  }

  for(iIndex = 0; iIndex < iBuflen; iIndex++)
  {
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg,"BY%0d: %#X[%c]", ucBuffer[iIndex]);
      Serial.println(g_arrcMsg);
    #endif
  }
}

/***********************************************************************************************/
/*! 
* \fn         :: isValidCmd()
* \author     :: Vignesh S
* \date       :: 05-DEC-2018
* \brief      :: This function validates the received command and return true if it a valid 
*                command else returns false, if a valid command is received, then the 
*                corresponding command ID is returned through out_iCmdID parameter, similarly
*                Invalid command ID error code is returned.
* \param[in]  :: parrcCmd, iCmdLen 
* \param[out] :: out_iCmdID
* \return     :: true or false
*/
/***********************************************************************************************/
bool isValidCmd(char *parrcCmd, int iCmdLen, int *out_iCmdID)
{
  unsigned long ulHour = 0;
  unsigned long ulMin = 0;
  unsigned long ulSec = 0;
  int iRetVal = 0;

  if((parrcCmd == NULL) || (out_iCmdID == NULL) || (iCmdLen <= 0))
  {
    return false;
  }

  if(StrnCmp(parrcCmd, CMD_RELAY_ON, iCmdLen) == true)
  {
    *out_iCmdID = CMD_RELAY_ON_ID;
    return true;
  }
  else if (StrnCmp(parrcCmd, CMD_RELAY_ON_TIMER, strlen(CMD_RELAY_ON_TIMER)) == true)
  {
    iRetVal = sscanf(parrcCmd, CMD_RELAY_ON_TIMER "%d:%d:%d", &ulHour, &ulMin, &ulSec);
    if(iRetVal != 0x03)
    {
        // invalid command
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg,"Invalid Pramater: %s", parrcCmd);
        Serial.println(g_arrcMsg);
      #endif
      *out_iCmdID = CMD_INVALID_CMD_ID;
      
      return false;
    }
    else
    {
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg,"Cmd: %s\r\nHour: %ld\r\nMin: %ld\r\nSec: %ld", parrcCmd, ulHour, ulMin, ulSec);
        Serial.println(g_arrcMsg);
      #endif

      // set the global timer variable
      g_ulOnTimeSec = (ulHour * 3600UL) + (ulMin * 60UL) + ulSec;
    }
   
   *out_iCmdID = CMD_RELAY_ON_TIMER_ID; 
    return true;
  }
  else if (StrnCmp(parrcCmd, CMD_RELAY_OFF, iCmdLen) == true)
  {
    *out_iCmdID = CMD_RELAY_OFF_ID;
    return true;
  }
  else if(StrnCmp(parrcCmd, CMD_START_TEST, iCmdLen) == true)
  {
    *out_iCmdID = CMD_START_TEST_ID;
    return true;
  }
  else
  {
    // invalid command
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg,"Invalid Cmd: %s", parrcCmd);
      Serial.println(g_arrcMsg);
    #endif
    *out_iCmdID = CMD_INVALID_CMD_ID;
  }

  return false;
}

/***********************************************************************************************/
/*! 
* \fn         :: StrnCmp()
* \author     :: Vignesh S
* \date       :: 05-DEC-2018
* \brief      :: This function compares two strings, returns true if they are identical else
*                false.  
* \param[in]  :: pString1, pString2, iLen 
* \return     :: true or false
*/
/***********************************************************************************************/
bool StrnCmp(char *pString1, char *pString2, int iLen)
{
  if((pString1 == NULL) || (pString2 == NULL) || (iLen <= 0))
  {
    return false;
  }

  for(int iIndex = 0; (iIndex < iLen); iIndex++)
  {
    if(pString1[iIndex] != pString2[iIndex])
    {
      return false;
    }
  }

  return true;
}

/***********************************************************************************************/
/*! 
* \fn         :: CmdProcess()
* \author     :: Vignesh S
* \date       :: 05-DEC-2018
* \brief      :: This function processes the recceived command and preforms corresponding task
* \param[in]  :: iCmdID
* \return     :: None
*/
/***********************************************************************************************/
void CmdProcess(int iCmdID)
{ 
  switch(iCmdID)
  {
    case CMD_RELAY_ON_ID:
      
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay ON");
        Serial.println(g_arrcMsg);
      #endif
      
      MotorRly.setState(RELAY_ON);
    break;

    case CMD_RELAY_ON_TIMER_ID:
      
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay ON for %ld seconds", g_ulOnTimeSec);
        Serial.println(g_arrcMsg);
      #endif
      
      MotorRly.setTimer(g_ulOnTimeSec);
    break;

    case CMD_RELAY_OFF_ID:

      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Turning Relay OFF");
        Serial.println(g_arrcMsg);
      #endif

      MotorRly.setState(RELAY_OFF);
    break;

    case CMD_START_TEST_ID:

      // perform self test
      SelfTest(0x01);
    break;

    default:
      ;// do nothing 
  }
}

/***********************************************************************************************/
/*! 
* \fn         :: RecvCmd()
* \author     :: Vignesh S
* \date       :: 07-DEC-2018
* \brief      :: This function cheks HC-05 serial port for received data and if data is available
*                then reads it, the number of bytes read is returned.
* \param[in]  :: pBuff, iBuflen
* \return     :: iIndex
*/
/***********************************************************************************************/
int RecvCmd(char *pBuff, int iBuflen)
{
  int iIndex = 0;

  if(pBuff == NULL)
  {
    return -1;
  }
  
  while(iIndex < iBuflen)
  {
    delay(HC05_SERIAL_READ_DELAY_MS);
    
    if(SS_Bluetooth.available())
    {
      pBuff[iIndex] = SS_Bluetooth.read();
      iIndex++;
    }
    else
    {
      break;
    }
  }

  return iIndex;
}

/***********************************************************************************************/
/*! 
* \fn         :: EmgStopInterrupt()
* \author     :: Vignesh S
* \date       :: 16-SEP-2019
* \brief      :: This function handles the Emergency Stop Interrupt and updates the global flag
* \param[in]  :: None
* \return     :: None
*/
/***********************************************************************************************/
void EmgStopInterrupt()
{
  /* set the global flag */
  g_viEmgStopInt = EMG_STOP_INT_STATE_SET;
}

