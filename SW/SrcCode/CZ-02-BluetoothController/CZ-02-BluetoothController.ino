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

/*************************************** Pin Mappings ******************************************/
// ground plane
#define GND_PLANE_PIN		PIN_A4

// user LEDS
#define USR_LED_1				PIN_A2
#define USR_LED_2				PIN_A3

// HC-05 
#define HC05_ST					PIN_A1		/* status pin */
#define HC05_EN					PIN_A5		/* enable pin */

// Relay 
#define RELAY 					PIN_A0
/***********************************************************************************************/

#define SELF_TEST_COUNT						0x01
#define MAX_DATA_LEN_BYTES				128

#define BTH_CMD_BASE							0x00
#define CMD_PER_SELF_TEST					(BTH_CMD_BASE + 1)
#define CMD_SET_RELAY_STATE				(BTH_CMD_BASE + 2)

#pragma pack (push, 1)

typedef struct 
{
	unsigned short usHeader;	/* 0xAA55 */
	unsigned short usDataLen;	/* length of data */
	unsigned short usCmdID;		/* command ID */
	unsigned char ucData[MAX_DATA_LEN_BYTES];

}S_BTH_PACKET;

/************************* Command  Structure for CMD_SET_RELAY_STATE **************************/
typedef struct 
{
	usigned char ucRelaySate; /* 0 - OFF; 1 - ON */

}S_CMD_SET_RELAY_STATE;

#pragma pack (pop)

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

	// perform self test
	SelfTest(SELF_TEST_COUNT);

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

	// check for any commands

	// if any command is received, process it

	// write the Relay ON time to EEPROM 
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
