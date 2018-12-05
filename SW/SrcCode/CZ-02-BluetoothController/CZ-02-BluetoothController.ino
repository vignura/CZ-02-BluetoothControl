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

SoftwareSerial SS_Debug(10, 11);
int g_iRelayState = 0;

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
/***********************************************************************************************/

#define MAX_DEBUG_MSG_SIZE                  32
#define MAX_CMD_STRING_SIZE                 10

#define SELF_TEST_COUNT                     0x01
#define HC05_BUAD_RATE                      9600
#define DEBUG_BUAD_RATE                     9600
#define HC05_SERIAL_READ_DELAY_MS           0x02

/* bluetooth command Strings*/
#define CMD_RELAY_ON                      "RelayOn"
#define CMD_RELAY_OFF                     "RelayOff"
#define CMD_START_TEST                    "StartTest"

/* bluetooth command IDs */
#define CMD_INVALID_CMD_ID                  -1
#define CMD_RELAY_ON_ID                     0x01 
#define CMD_RELAY_OFF_ID                    0x02
#define CMD_START_TEST_ID                   0x03

/* Relay states */
#define RELAY_STATE_ON                      0x00
#define RELAY_STATE_OFF                     0x01 

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
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);
  g_iRelayState = RELAY_STATE_OFF;

  // Serial port initialization
  Serial.begin(HC05_BUAD_RATE);
  SS_Debug.begin(DEBUG_BUAD_RATE);

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
  char arrcMsg[MAX_DEBUG_MSG_SIZE]  = {0};
  int iIndex = 0;
  int iCmdID = 0;

  // read data from HC-05 if available
  while(iIndex < 10)
  {
    delay(HC05_SERIAL_READ_DELAY_MS);
    
    if(Serial.available())
    {
      arrcCmd[iIndex] = Serial.read();
      iIndex++;
    }
    else
    {
      break;
    }
  }
  
  // print to debug if data is avialible
  if(iIndex > 0)
  {
    sprintf(arrcMsg, "[%d] %s", iIndex, arrcCmd);
    SS_Debug.println(arrcMsg);
  }
  
  // validate the command
  if(isValidCmd(arrcCmd, iIndex, &iCmdID) == true)
  {
    CmdProcess(iCmdID);
  }
  else
  {
    // do nothing
  }
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
  if(Serial.available())
  {
    SS_Debug.write(Serial.read());
    // echo
    //Serial.write(Serial.read());
    //digitalWrite(LED_BUILTIN, HIGH);
  }

  // if data is available in debug, send it to bluetooth 
  if(SS_Debug.available())
  {
    Serial.write(SS_Debug.read());
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

  for(iCount = 0; iCount < iTestCount; iCount++)
  {
    // turn on the two LEDs
    digitalWrite(USR_LED_1, HIGH);
    digitalWrite(USR_LED_1, HIGH);
    // turn on Relay
    digitalWrite(RELAY, HIGH);

    delay(2000);
    
    // turn off the two LEDs
    digitalWrite(USR_LED_1, LOW);
    digitalWrite(USR_LED_1, LOW);
    // turn off Relay
    digitalWrite(RELAY, LOW);

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
  char arrcMsg[MAX_DEBUG_MSG_SIZE] = {0};

  if(ucBuffer == NULL)
  {
    return;
  }

  for(iIndex = 0; iIndex < iBuflen; iIndex++)
  {
     sprintf(arrcMsg,"BY%0d: %#X[%c]", ucBuffer[iIndex]);
     
     // print arrcMsg to serial port
     // Serial.Println(arrcMsg);
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
  char arrcMsg[MAX_DEBUG_MSG_SIZE] = {0};

  if((parrcCmd == NULL) || (out_iCmdID == NULL) || (iCmdLen <= 0))
  {
    return false;
  }

  if(StrnCmp(parrcCmd, CMD_RELAY_ON, iCmdLen) == true)
  {
    *out_iCmdID = CMD_RELAY_ON_ID;
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
    sprintf(arrcMsg,"Invalid Cmd: %s", parrcCmd);
    SS_Debug.println(arrcMsg);
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

  for(int iIndex = 0; iIndex < iLen; iIndex++)
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
  char arrcMsg[MAX_DEBUG_MSG_SIZE] = {0};
  
  switch(iCmdID)
  {
    case CMD_RELAY_ON_ID:

      // turn on relay
      digitalWrite(RELAY, HIGH);
      // set the relay state flag to on state
      g_iRelayState = RELAY_STATE_ON;

      sprintf(arrcMsg, "Turning Relay ON");
      SS_Debug.println(arrcMsg);
    break;

    case CMD_RELAY_OFF_ID:

      // turn off relay
      digitalWrite(RELAY, LOW);
      // set the relay state flag to off state
      g_iRelayState = RELAY_STATE_OFF;

      sprintf(arrcMsg, "Turning Relay OFF");
      SS_Debug.println(arrcMsg);
    break;

    case CMD_START_TEST_ID:

      sprintf(arrcMsg, "Performing Self Test");
      SS_Debug.println(arrcMsg);
      // perform self test
      SelfTest(0x01);
    break;

    default:
      ;// do nothing 
  }
}

