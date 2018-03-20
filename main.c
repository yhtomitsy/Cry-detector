/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_drv_clock.h"
#include "app_error.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "math.h"
#include "peer_manager.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"
#include "nrf_delay.h"
#include <stdbool.h>
#include <stdio.h>
#include "nrf_drv_config.h"
#include "nrf_adc.h"
#include "boards.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "app_pwm.h"




/*
 * the device settings in this order
 * 0 - volume TBD
 * 1 - light intensity (0 - 100)
 * 2 - number of sound bits (0 - 10)
 * 3 - number of light pulses (0 - 20)
 * 4 - loud 1st sound cycle (0/1) TBD
 * 5 - whistle sensor state (0/1) TBD
 * 6 - auto off time (1 - 25[never]) TBD
 * 7 - melody (1- 4)
 * 8 - plays (0 - 2)
 * 9 - whistle 0 (0/1) tbd
 */
volatile struct DeviceSettings{
  uint32_t deviceSettings[10];
};

static uint32_t defaultSettings[10] = {100, 100, 2, 8, 0, 0, 1, 1, 0, 0}; 																				// default settings
struct DeviceSettings settings = {10, 100, 2, 8, 1, 0, 1, 3, 1, 0};                                               // current device settings
static uint32_t instance[10] = {0x2222, 0x2223, 0x2224, 0x2225, 0x2226, 0x2227, 0x22238, 0x2229, 0x222A, 0x222B}; // instance keys for fds records
static uint32_t type[10] = {0x1111, 0x1112, 0x1113, 0x1114, 0x1115, 0x1116, 0x1117, 0x1118, 0x1119, 0x111A};      // type keys for fds records
static bool updateReceived = false;                       																												// set whenever any value is updated
static bool saveUpdates = false;																																									// indicates that data can proceed to be saved
static char command[6] = "";                              																												// holds incoming data
static bool existingRecord = false;                       																												// holds state of records (either existing or not) for purposes of updating or creating new records
static bool testSettings = false;
//static char arrayNames[10] = {'V', 'L', 'N', 'P', 'S', 'W', 'A', 'M', 'B', 'O'}; 																// for debugging
//static uint32_t sampleResult = 0;                         																											// for debugging 


#define BLELED  24                      // shows BLE status
#define BATLED1  5                      // Shows battery life
#define BATLED2  20                     // Shows battery life
#define piezoINTPin 3                   // interrupt pin for the piezo
#define piezoAnalog 4                   // analog pin for piezo
#define audioPin 6                      // analog pin for audio input from amplifier

#define sampleDuration 100              // time between piezo buzzer sampling 20Hz 
#define numberOfTaps 5                  // maximum number of taps to turn on/off device
#define knockInterval 500               // maximum time between knocks*/
#define VBAT_MAX_IN_MV 3000             // maximum battery voltage in millivolts
#define VBAT_MIN_IN_MV 2000							// minimum battery voltage in milivoltas

#define NOTE_G3  196                    // notes used in lullaby playing
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976

static long onTime = 0;
bool whistleDetected = false;
int ledPulseTime = 4;                 	//(100 * 4)/settings.deviceSettings[1]; // calculate time interval for fade

// variables for BLE LED control
volatile bool BLEDisconnected = false; 	// flag that enables us to know when the device has disconnected
bool BLEWindow = false; 								// determines if BLE can be activated
volatile bool BLEActivated = false; 		// determines if BLE can be activated
long timer = 0; 												// manages blinking of BLE led
uint32_t bleWindowTimer = 0; 						// holds time duration of BLE window
static bool goToSleep = false;          // flag to determine whether device is to be shutdown or left running



// variables for piezo control:
volatile long timeTrack = 0; 						// holds execution time
volatile uint8_t taps = 0; 							//holds the number of consecutive taps

enum DeviceState{
  ON,
  OFF
}deviceState = OFF;


//variables for playing music on piezo {};//
int ONNote[]= {NOTE_C4, NOTE_G4};
int OFFNote[] = {NOTE_G4, NOTE_C4};//gc
int lullaby0[] = {NOTE_G4, NOTE_A4, NOTE_D5, NOTE_A4,0};// gada palis melody // {NOTE_G4, NOTE_A4, NOTE_D5, NOTE_A4}
int lullaby1[] = {NOTE_AS4, NOTE_C5, NOTE_GS4, NOTE_GS3, NOTE_DS4}; // F#, G#, E, e, H close encounters Bflat C Aflat (octavelower) AFlat E flat
int lullaby2[] = {NOTE_A4, NOTE_A4, NOTE_A4, NOTE_F4, NOTE_C5, NOTE_A4, NOTE_F4, NOTE_C5, NOTE_A4, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_F5, NOTE_C5, NOTE_GS4, NOTE_F4, NOTE_C5, NOTE_A4}; // A,A,A,F,C, A,F,C,A, E,E,E,f,C, G#, F,C,A imperial march
int lullaby3[] = {NOTE_G4, NOTE_G4, NOTE_D5, NOTE_D5, NOTE_E5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_C5, NOTE_B4, NOTE_B4, NOTE_A4, NOTE_A4, NOTE_G4}; // G,G,D,D,E,E,D,C,C,B,B,A,A,G twinkle twinkle little star
int ONbeats[] = { 1, 2};
int lullabyBeats0[] = {2, 2, 9, 11,2};
int lullabyBeats1[] = {2, 3, 4, 3, 12};
int lullabyBeats2[] = {4, 4, 4, 3, 1, 4, 3, 1, 8, 4, 4, 4, 3, 1, 4, 3, 1, 8};
int lullabyBeats3[] = {2, 2, 2, 2, 2, 2, 5, 2, 2, 2, 2, 2, 2, 4};
int tempo = 150;

// implements LED breathing when lullaby is playing
volatile int tSign = 1;
volatile int fade = 0;
volatile uint8_t fadeTrack = 0;
volatile bool interruptDisabled = false;

volatile static uint32_t millis = 0;
ret_code_t err_code = 0;
//bool palisAdvertising = false;
volatile bool babyCrying = false;



/********************************************************************BLE constants and variables ****************************************************/

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define DEVICE_NAME                     "Palis"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      30                                          /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             1                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */
APP_TIMER_DEF(millis_timer_id);                                                     /**< will hold the millis timer id. */
APP_TIMER_DEF(led_action_timer);                                                    /**< will hold the leb blink timer id. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    15                                           /**< Number of attempts before giving up the connection parameter negotiation. */

//#define START_STRING                    "Start...\n"                                /**< The string that will be sent over the UART when the application starts. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */



/**************************************audio sampling variables and constants*********************************************/

//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

//#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
//#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

#ifndef NRF_APP_PRIORITY_HIGH
#define NRF_APP_PRIORITY_HIGH                   1
#endif

//// config FFT parameters
#define ARRAY_SIZE                              64                              // the size of FFT
#define LOGN_ARRAY_SIZE                         6
#define BIN_STEP                                50000/ARRAY_SIZE/2
//// ATTENTION! It's important to configure fft offset below. 
//// This is the adc value you get in total silence in float. 
//// My mic have few volts offset do this value is 172
#define FFT_OFFSET                              170.0                           // value to distract 
//// Cry detection sensivity parameters
#define AVER_POWER                              4                               // average power of signal shoulâ be higher
#define HAPR                                    25                              // harmonic to average power ratio of signal should be higher
// from frequency sampling tests it was found that the babry cry was ranging from 2200Hz to 3000Hz
// frequency for the whistle was found to be 
#define LOW_BAND                                2200//700                       // pitch freq of signal should be between 700 Hz ...
#define HIGH_BAND                               3000//4000                      // ... and 4000 Hz
#define WHISTLE_LOW_BAND                        2000
#define WHISTLE_HIGH_BAND                       2200

#define SILENCE_TRSHLD                          35//5//                          37 // signal with power lower than this considered as silence
#define NUM_SILENT_SAMPLES                      100//15//                        // if it's a baby's cry there should more than 15 silent samples


volatile bool data1Ready = false, data2Ready = false;
float inArrayR[2][ARRAY_SIZE] = {0};                                            // vars to process FFT 
float inArrayI[ARRAY_SIZE] = {0};                                               // vars to process FFT 
static bool cryDetected = false;

/***************************************************PWM variables*********************************************************/
// to get APP_PWM_INSTANCE(PWM1, 1) to work define TIMER1 as 1 in the nrf_drv_config.h file, otherwise you get the error
// TIMER1_INSTANCE is not defined

APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.
static volatile bool ready_flag;            // A flag indicating PWM status.




/*****************************************************************prototypes*******************************************************************/

bool  FFT(float *Rdat, float *Idat, int N, int LogN);                          // fft function
uint16_t maxFreqAnalysis(float * arr, uint16_t size);                          // function that analyses the sampled frequencies
void isBabyCrying(void);                                                       // function detects whether a baby is crying or not
void ADC_IRQHandler(void);                                                     // samples analog readigns and fills arrays with the data
void adcConfig(void);                                                          // configure the ADC
void calcNewOffset(float*);																										 // calculates new offset
static void uart_init(void);                                                   // configuring the UART parameters
void uart_error_handle(app_uart_evt_t *);                              // handling data received in UART
//void timer2Setup(unsigned int ms);                                             // sets up timer interrupt
void attachInterrupt(uint8_t, uint8_t, uint8_t);           // function to set hardware interrupts
void in_pin_handler(nrf_drv_gpiote_pin_t, nrf_gpiote_polarity_t);   // hardware interrupt for detecting knocks
void changePinStates(bool x);                                                  // changes conficuration of piezo pins
void turnOn(void);                                                             // turns on device features after waking up
void sendToSleep(void);                                                        // does some houskeeping before sending device into deep sleep
static void sleep_mode_enter(void);                                            // sends device into deep sleep
void knockDetect(void);                                                        // listens to number of knocks/taps on the piezo
void playTone(int tone, int duration, int led);                                // play the tone selected by the user
void playNote(int note, int duration, int led);                                // play note seleected by user
void onOffTone(int index);                                                     // play the on/off tone
void test(void);                                                               // play a test tone
void pwm_ready_callback(uint32_t pwm_id);                                      // PWM callback function
void PWM_config(void);                                                         // PWM configuration file
void analogWrite(uint8_t channel, uint8_t value);                              // Function that performs PWM
uint8_t battery_level_get(void);                                               // gets the battery level
void batteryLevelCheck(void);                                                  // blinks battery indicator LEDs
void lullabySong(uint8_t i);                                                   // sets the lullaby that is going to play
void playLullaby(void);                                                        // controls the way the lullaby plays
void TIMER2_IRQHandler(void);                                                  // timer2 IRQ handler
//void startAdvertising(void);                                                 // function for starting ble advertisment
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
static void gap_params_init(void);
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length);
static void services_init(void);
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
static void conn_params_error_handler(uint32_t nrf_error);
static void conn_params_init(void);
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);
static void on_ble_evt(ble_evt_t * p_ble_evt);
static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
static void ble_stack_init(void);
void bsp_event_handler(bsp_event_t event);
static void advertising_init(void);
static void buttons_leds_init(bool * p_erase_bonds);
static void power_manage(void);
static void advertising(bool);                                                      // starts/stops advertisement of bluetooth device
static void millis_timer_handler(void *);                                           // function used to calculate elapsed time in milliseconds
static void createTimers(void);																											// creates timer for keeping track of millis
static void startTimers(void);                                                      // starts timer for counting millis                    
static void sendSettings(void);																											// sends the saved settings to tablet/phone
static void sendData(uint8_t *, uint8_t);																						// send data over BLE
static void sendBat(void);																													// send battery level
void startADC(bool);                                                                // starts/stops ADC
static void led_pulses(void *);																											// handles led pulses
static void startLedPulses(void);                                                   // function that starts the led pulse timer
/********************************************************************BLE data handling functions****************************************************/

/**@brief Function for converting a character array into int
 *
 * @details This function will be called in the extractdata function to get integers from received string
 *          The startIndex marks the cell of the array where the conversion will start and the lastIndex 
 *					marks where the conversion will end
 */
static int charArrayToInt(char cRef[], uint8_t startIndex, uint8_t lastIndex){
	int x = 0;                                                         // holds the integer result after conversion
	uint8_t j = 0;                                                     // holds position of new line character
	
	for(uint8_t i = lastIndex; i >= startIndex; i--){                  // find position of new line character
		if(cRef[i] == '\n' || cRef[i] == '.') j = i - 1;
	}
	// printf("%d \n", j);
	for(uint8_t i = j; i >= startIndex; i--){                          // convert characters, between start index and new line character, to integers
			x += ((cRef[i] - '0') * pow(10,(j - i)));
	}
	return x;
}

/**@brief Function for retrieving settings from received string
 *
 * @details This function will be called in the nus_data_handler and will extract the settings from the received string
 */
void extractData(char c[]){
  if(c[0] == 'V'){
    settings.deviceSettings[0] = charArrayToInt(c, 1, 4);
    if(settings.deviceSettings[0] > 0){
			//changePinStates(0);
      test();
      //changePinStates(1);
    }
  }
  else if(c[0] == 'L'){
    settings.deviceSettings[1] = charArrayToInt(c, 1, 4);
    // display LED brightness
		analogWrite(0, settings.deviceSettings[1]); 
		analogWrite(1, settings.deviceSettings[1]); 
		
    ledPulseTime = (100 * 4)/settings.deviceSettings[1]; // calculate time interval for fade
    //timer2Setup(ledPulseTime);                           // setup the LED pulse timer to correspond to the brightness.
  }
  else if(c[0] == 'N')settings.deviceSettings[2] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'P')settings.deviceSettings[3] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'S')settings.deviceSettings[4] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'W')settings.deviceSettings[5] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'A')settings.deviceSettings[6] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'M'){
    settings.deviceSettings[7] = charArrayToInt(c, 1, 4);
    if(settings.deviceSettings[0] > 0){
      //changePinStates(0);
      lullabySong(settings.deviceSettings[7]);
      //changePinStates(1);
    }
  }
  else if(c[0] == 'B')settings.deviceSettings[8] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'O')settings.deviceSettings[9] = charArrayToInt(c, 1, 4);
  else if(c[0] == '#'){
    sendBat(); 																																		 // send battery voltage to phone app
  }
  else if(c[0] == '$'){
    for(uint8_t i = 0; i < 9; i++)settings.deviceSettings[i] = defaultSettings[i]; // return device back to default settings
  }
  else if(c[0] == '@'){
		sendSettings();                     																					 // send current settings to phone
  }
	else if(c[0] == '*'){
		//testSettings = true;
		//changePinStates(0);
		babyCrying = true;
	  playLullaby();
  }
	
}

/*****************************************************end of BLE data handling functions**********************************************/




/********************************************************************BLE functions****************************************************/
// to indicate advertising using pin 24 the library file bsp.c is edited. In the case BSP_INDICATE_ADVERTISING and BSP_INDICATE_CONNECTED
// in the function bsp_led_indication the bsp lED is changed from BSP_LED_0_MASK to BSP_LED_3_MASK. Also, the set features of the pins are inverted
// therefore change the LEDS_ON to LEDS_OFF for each BSP_LED_3_MASK statement. Otherwise the LED will be ON when it is supposed to be OFF and
// vice versa.
// NB: BSP leds are turned ON by setting the pin LOW an turned OFF by setting the pin HIGH

/**@brief Function for stopping unwanted activities when device is connected.
 *
 * @details This function stops the ADC, and disables interrupt on PIN 3 
 */
void onConnectPrep(){
		changePinStates(0);       // deactivate all interrupts except interrupts used by Softdevice
}

/**@brief Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in] p_ble_evt S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt){
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						onConnectPrep();
						//printf(">>BLE Connected\n");
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						BLEDisconnected = true;                                  // indicates that phone has disconnected from Palis
						if(updateReceived == true){
								saveUpdates = true;                                  // shows that there is data available for saving
						}
						//printf(">>BLE Disconnected\n");
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length){
    for (uint32_t i = 0; i < length; i++)
    {
        command[i] = p_data[i];                              // received data is copied to command array
				//printf("%c", command[i]);													   // data in command array is printed onto Serial monitor
    }             
		extractData(command);																		 // extract settings from recieved commands
		updateReceived = true;                                   // indicate that there is new data that has been received
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name){
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void){
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}





/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void){
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt){
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error){
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void){
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void){
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    //err_code = bsp_btn_ble_sleep_mode_prepare();
    //APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt){
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
						goToSleep = true;                                            // indicate to device that it should go to sleep
            break;
        default:
            break;
    }
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt){
    fs_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for dispatching a S110 SoftDevice event to all modules with a S110 SoftDevice 
 *        event handler.
 *
 * @details This function is called from the S110 SoftDevice event interrupt handler after a S110 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt){
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    
}


/**@brief Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void){
    uint32_t err_code;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(S132))
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
		
		// without adding the system event handler to the BLE stack fds will not work. This link will
	  // enlighten you more https://devzone.nordicsemi.com/f/nordic-q-a/22265/can-t-get-a-callback-from-fds-no-matter-what---help
		err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
		
		
}

/**@brief Function used to stop and start advertising
 */
static void advertising(bool state){
		uint32_t err_code;
		if(state){
				err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
				APP_ERROR_CHECK(err_code);
				//printf(">>BLE started\n");
		}
		else{
				err_code = ble_advertising_start(BLE_ADV_MODE_IDLE); 	 // set advertising to IDLE to prevent false advertisement triggers
				APP_ERROR_CHECK(err_code);
				err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
				sd_ble_gap_adv_stop();                                   // stops advertisement
				//printf(">>BLE stopped\n");
		}
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event){
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void){
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds){
    bsp_event_t startup_event;
		// edit this so that the bsp buttons are not included. Its initially BSP_INIT_LED|BSP_INIT_BUTTONS
    uint32_t err_code = bsp_init(BSP_INIT_LED,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void){
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for sending data to connected client
 *
 * @details This function is called from the sendSettings and sendbat functions that are
 *          are used to send the device settings and battery level to the connected client
 *          had to edit ble_nus_string_send function in the ble_nus.c file and commented out
 *          the IF statement that checks if notifications are enabled (they are enabled)
 *
 * @param[in] array[]  data to be sent.
 * @param[in] arraySize  length of data that is being sent.
 */
static void sendData(uint8_t array[], uint8_t arraySize){
		uint32_t       err_code;
		
		//m_nus.is_notification_enabled = true;
		err_code = ble_nus_string_send(&m_nus, array, arraySize);
		//printf("DS\n");
		if (err_code != NRF_SUCCESS &&
        err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
        err_code != NRF_ERROR_INVALID_STATE &&
        err_code != BLE_ERROR_NO_TX_BUFFERS)
    {
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for sending the battery level to client. Its called in the
 *        extractData function
 */
static void sendBat(){
		uint8_t batLevel = battery_level_get(); // gets battery level
		uint8_t b[5] = {0};          						// will hold the battery level 
		uint8_t j = 1;               						// manages array b
		b[0] = 'B';
		if(batLevel < 10){
				b[j] = batLevel + '0'; 
				j++;
		}
		else if(batLevel < 100){
				b[j] = (int)(batLevel / 10) + '0'; 
				j++;	
				b[j] = (int)(batLevel % 10) + '0'; 
				j++;
		}
		else if(batLevel >= 100){
				b[j] = (int)(batLevel / 100) + '0'; 
				j++;
				b[j] = (int)((batLevel / 10) - ((batLevel / 100) * 10)) + '0'; 
				j++;	
				b[j] = (int)(batLevel % 10) + '0'; 
				j++;
		}
		//b[j] = '\n';
		sendData(b, sizeof(b));               // send data to client
}


/**@brief Function for sending the device settings to client. Its called in the
 *        extractData function
 */
static void sendSettings(){
		uint8_t y[17] = {0};         // will hold first 5 settings of the device
		uint8_t z[12] = {0};         // will hold the remaining 5 settings of the device
		uint8_t j = 1;               // manage array y
		
		for(uint8_t i = 0; i < 5; i++){
				if(settings.deviceSettings[i] < 10){
						y[j] = settings.deviceSettings[i] + '0'; 
						j++;
				}
				else if(settings.deviceSettings[i] < 100){
						y[j] = (int)(settings.deviceSettings[i] / 10) + '0'; 
						j++;	
						y[j] = (int)(settings.deviceSettings[i] % 10) + '0'; 
						j++;
				}
				else if(settings.deviceSettings[i] >= 100){
						y[j] = (int)(settings.deviceSettings[i] / 100) + '0'; 
						j++;
						y[j] = (int)((settings.deviceSettings[i] / 10) - ((settings.deviceSettings[i] / 100) * 10)) + '0'; 
						j++;	
						y[j] = (int)(settings.deviceSettings[i] % 10) + '0'; 
						j++;
				}
				// add a comma between each setting and newline character at the end of string
				if(i != 4){
						y[j] = ',';
						j++;
				}
				else{
						y[j] = '\n';
						j = 1;
				}
		}
		for(uint8_t i = 5; i < 10; i++){
				if(settings.deviceSettings[i] < 10){
						z[j] = settings.deviceSettings[i] + '0'; 
						j++;
				}
				else if(settings.deviceSettings[i] < 100){
						z[j] = (int)(settings.deviceSettings[i] / 10) + '0'; 
						j++;	
						z[j] = (int)(settings.deviceSettings[i] % 10) + '0'; 
						j++;
				}
				else if(settings.deviceSettings[i] >= 100){
						z[j] = (int)(settings.deviceSettings[i] / 100) + '0'; 
						j++;
						z[j] = (int)((settings.deviceSettings[i] / 10) - ((settings.deviceSettings[i] / 100) * 10)) + '0'; 
						j++;	
						z[j] = (int)(settings.deviceSettings[i] % 10) + '0'; 
						j++;
				}
				// add a comma between each setting and newline character at the end of string
				if(i != 9){
						z[j] = ',';
						j++;
				}
				else{
						z[j] = '\n';
						j = 1;
				}
		}
		y[0] = 'Y';
		z[0] = 'Z';
		
		// send device settings to the client
		sendData(y, sizeof(y));
		nrf_delay_ms(100);
		sendData(z, sizeof(z));
}
/****************************************************************** end of BLE functions************************************************/



/**********************************************************NRF based functions********************************************************/
// ADC code
/**@brief   Function for configuring up ADC.
 *
 * @details ADC configured to read from pin 6 with a sample rate of 50kHz.
 *          Analog samples are filled in 2 arrays for analysis
 *          
 */
/**@snippet [configuring the ADC] */
void adcConfig(void) {
    const nrf_adc_config_t nrf_adc_config = { 
      NRF_ADC_CONFIG_RES_8BIT,                    // 20 us ack time -> 50kHz sample rate
      NRF_ADC_CONFIG_SCALING_INPUT_TWO_THIRDS, 
      NRF_ADC_CONFIG_REF_SUPPLY_ONE_HALF 
    };
    nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
    nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_7);
    nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
    NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_HIGH);
    NVIC_EnableIRQ(ADC_IRQn);
}
/**@snippet [used to calculate offset dynamically] */
void calcNewOffset(float * offset) {
  float val = 0.0;
  for (uint8_t i = 0; i < ARRAY_SIZE; i++) 
    val += inArrayR[0][i] + *offset;
  val /= ARRAY_SIZE;
  *offset = (*offset + val) / 2;
}

/**@snippet [ADC handler fills 2 arrays with data] */
void ADC_IRQHandler(void) {
  static uint32_t sampleCount = 0, measureArr = 1;
  static float offset = 172.0;          // auto offset calculation
  nrf_adc_conversion_event_clean();
  
  if (data1Ready == false || data2Ready == false) {
    /*uint16_t adcResult = nrf_adc_result_get();
    if (adcResult < FFT_OFFSET) adcResult = 0;
    else adcResult = adcResult - FFT_OFFSET;
    inArrayR[0][sampleCount] = (float)adcResult;*/

    inArrayR[0][sampleCount] = (float) nrf_adc_result_get() - offset;
    sampleCount++;
    
    if (sampleCount >= ARRAY_SIZE && measureArr == 1) {
      data1Ready = true;                // first frame ready to analyze
      measureArr = 2;
      calcNewOffset(&offset); 
    }
    if (sampleCount >= ARRAY_SIZE * 2 && measureArr == 2) {
      data2Ready = true;                // second frame ready to analyze
      sampleCount = 0;
      measureArr = 1;
      calcNewOffset(&offset); 
    }
  }    
  nrf_adc_start();                      // trigger next ADC conversion
}

// Timer interrupt code
/**@brief   Function for setting up a timer interrupt.
 *
 * @details This function will setup a timer interrupt using timer 2
 *          The frequency of the interrupt will be defined by the variable ms that you pass into the function
 *          In this application the timer will have a frequency of 1kHz and will be used to control the BAT LEDs
 *          This means that we will pass a value 1 into ms. 
 *          (ms stands for milliseconds so passing 1 means the timer interrupt is set at 1kHz. setting 10 means its runnign at 100Hz)
 *          
 */
/**@snippet [Setting up the timer] */
/*void timer2Setup(unsigned int ms){                                 // directly pass the value you want the cycle to be in mS
		NRF_TIMER2->TASKS_STOP = 1;                                      // Stop timer
		NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;                        // sets the timer to TIME mode (doesn't make sense but OK!)
		NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;               // with BLE only Timer 1 and Timer 2 and that too only in 16bit mode
		NRF_TIMER2->PRESCALER = 9;                                       // Prescaler 9 produces 31250 Hz timer frequency => t = 1/f =>  32 uS
																																		 // The figure 31250 Hz is generated by the formula (16M) / (2^n)
																																		 // where n is the prescaler value
																																		 // hence (16M)/(2^9)=31250
		NRF_TIMER2->TASKS_CLEAR = 1;                                     // Clear timer
	 
		//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		//        Conversion to make cycle calculation easy
		//        Since the cycle is 32 uS hence to generate cycles in mS we need 1000 uS
		//        1000/32 = 31.25  Hence we need a multiplication factor of 31.25 to the required cycle time to achive it
		//        e.g to get a delay of 10 mS      we would do
		//        NRF_TIMER2->CC[0] = (10*31)+(10/4);
		//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	 
		NRF_TIMER2->CC[0] = (ms * 31) + (ms / 4);                                                                                  //CC[0] register holds interval count value i.e your desired cycle
		NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;                                     // Enable COMAPRE0 Interrupt
		NRF_TIMER2->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);                             // Count then Complete mode enabled
		//err_code = sd_nvic_SetPriority(TIMER2_IRQn, APP_IRQ_PRIORITY_LOW);
		//APP_ERROR_CHECK(err_code);
		NVIC_EnableIRQ(TIMER2_IRQn);                                                                                               // also used in variant.cpp in the RFduino2.2 folder to configure the RTC1                                                                                              // Start TIMER
		NRF_TIMER2->TASKS_START = 1;
}*/


/**@breif Configure hardware interrupts */


/**@details  Sets hardware interrupt on the pin that is passed to the function using the pin variable. 
 *           The interrupt can be on the single pin or on the port, depending on the value of singlePin variable
 *           Set singlePin as 0 to use a port event and 1 to use a dedicated event
 */
/**@snippet [Hardware interrupt set up] */
void attachInterrupt(uint8_t singlePin, uint8_t pin, uint8_t state){
		if(state){
			//Initialize gpiote module
			err_code = nrf_drv_gpiote_init();
			APP_ERROR_CHECK(err_code);
			//Configure sense input pin to enable wakeup and interrupt on button press.
			nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(singlePin);    //Configure to generate interrupt and wakeup on pin signal low. "false" / "0" means that gpiote will use the PORT event, which is low power, i.e. does not add any noticable current consumption (<<1uA). Setting this to "true" will make the gpiote module use GPIOTE->IN events which add ~8uA for nRF52 and ~1mA for nRF51.
			in_config.pull = NRF_GPIO_PIN_NOPULL;                                               //Do not enable the internal pullup resistor. Pin will be left floating. Its normal status will depend on input from the connected peripheral
			err_code = nrf_drv_gpiote_in_init(pin, &in_config, in_pin_handler);                 //Initialize the pin with interrupt handler in_pin_handler
			APP_ERROR_CHECK(err_code);                                                          //Check potential error
			nrf_drv_gpiote_in_event_enable(pin, true);                                          //Enable event and interrupt for the wakeup pin                                                          //Enable interrupts
		}
		else{
				nrf_drv_gpiote_in_event_disable(pin);
		}
}


/** @brief Procedures that keep track of the current milliseconds
 *   
 *  @details RTC1 is used to provide the time tracking. Its counter increments 
 *					 with a frequency of 1kHz derived from the LFCLK. The variable millis
 *					 is used to track the increments and its value used to keep track of 
 *					 time in milliseconds. The resolution is 1ms. The number of milliseconds 
 *           since the device started are stored on a vaiable called millis.
 */
/** @snipet Function that creates timer on RTC1*/
static void createTimers(){   
    uint32_t err_code;

    // Create timer for keeping track of millis
    err_code = app_timer_create(&millis_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                millis_timer_handler);
    APP_ERROR_CHECK(err_code);
	  
	  // Create timer for handling led pulses
    err_code = app_timer_create(&led_action_timer,
                                APP_TIMER_MODE_REPEATED,
                                led_pulses);
    APP_ERROR_CHECK(err_code);
}

/** @snipet Function that starts millis timer on RTC1*/
static void startTimers(){
		err_code = app_timer_start(millis_timer_id, APP_TIMER_TICKS(1, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);		
}

/** @snipet Function that starts led pulse timer on RTC1*/
static void startLedPulses(){
		err_code = app_timer_start(led_action_timer, APP_TIMER_TICKS(ledPulseTime, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}


/** @snipet Function that keeps track of millis*/
static void millis_timer_handler(void * p_context){
    millis++;
}

/** @brief RTC handler for controlling LED pulses
 *   
 *  @details This function controls the LED pulses during the lullaby
 *					 I commented out the timer2_IRQhandler function from nrf_drv_timer.c file, to prevent the 'multiply defined' error
 * 					 
 */
/** @snipet [Control LED pulses]*/
static void led_pulses(void * p_context){
    // manage glowing and fading
    if (fade >= settings.deviceSettings[1]){
      tSign = -1;
      fadeTrack++;
    }
    else if (fade <= 0){
      tSign = 1;
      fadeTrack++;
    }
    fade += tSign;

    // detect when lullaby is over and continue LED fading
    if(babyCrying == false){
      analogWrite(0,fade);
			analogWrite(1,fade);
    }
    
    // detect when maximum number of fades is reached and end timer interrupt
    if(fadeTrack == settings.deviceSettings[3] * 2 && fade == 0){
      fadeTrack = 0;                     // initialize number of fades
      fade = 0;                          // initialize fade value
      nrf_gpio_pin_set(BATLED1);         // completely turn off LED
			nrf_gpio_pin_set(BATLED2);         // completely turn off LED
			app_timer_stop(led_action_timer);  // stop RTC timer event
    }
}

/** @brief IRQ handler for timer2 event
 *   
 *  @details This function controls the LED pulses during the lullaby
 *					 I commented out the timer2_IRQhandler function from nrf_drv_timer.c file, to prevent the 'multiply defined' error
 * 					 
 *
/** @snipet [Control LED pulses]*/
/*void TIMER2_IRQHandler(void){
  if (NRF_TIMER2->EVENTS_COMPARE[0] != 0){
    // manage glowing and fading
    if (fade >= settings.deviceSettings[1]){
      tSign = -1;
      fadeTrack++;
    }
    else if (fade <= 0){
      tSign = 1;
      fadeTrack++;
    }
    fade += tSign;

    // detect when lullaby is over and continue LED fading
    if(babyCrying == false){
      analogWrite(0,fade);
			analogWrite(1,fade);
    }
    
    // detect when maximum number of fades is reached and end timer interrupt
    if(fadeTrack == settings.deviceSettings[3] * 2 && fade == 0){
      fadeTrack = 0;                     // initialize number of fades
      fade = 0;                          // initialize fade value
      nrf_gpio_pin_set(BATLED1);         // completely turn off LED
			nrf_gpio_pin_set(BATLED2);         // completely turn off LED
			NRF_TIMER2->EVENTS_COMPARE[0] = 0; // clear the compare flag
      NRF_TIMER2->TASKS_STOP = 1;        // disable timer interrupt
    }
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
  }
}*/


/** @brief Procedures that configures and controls PWM
 *   
 *  @details PWM is derived from TIMER1
 */
/** @snipet [PWM config]*/
void PWM_config(){
	  ret_code_t err_code;
	  /* 2-channel PWM, 200Hz, output on pins 20 and 5. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, BATLED1, BATLED2);
    
    /* Switch the polarity of the channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    
    /* Initialize and enable PWM. */
		err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);
}

/** @snipet [PWM call back function]*/
void pwm_ready_callback(uint32_t pwm_id){    // PWM callback function
    ready_flag = true;
}

/** @snipet [Function that performs PWM]*/
void analogWrite(uint8_t channel, uint8_t value){
    /* Set the duty cycle - keep trying until PWM is ready... */
    while (app_pwm_channel_duty_set(&PWM1, channel, value) == NRF_ERROR_BUSY);
		//app_pwm_channel_duty_set(&PWM1, channel, value);
}

/** @brief Procedures gets the battery voltage
 *   
 *  @details Battery voltage is calculated by first setting the analog reference to VBG
 *           VBG (internal 1.2 V band gap reference). 
 */
/** @snipet [get battery level]*/
uint8_t battery_level_get(void){
		// Configure ADC
		NRF_ADC->CONFIG     = (ADC_CONFIG_RES_8bit                        << ADC_CONFIG_RES_Pos)     |
													(ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |
													(ADC_CONFIG_REFSEL_VBG                      << ADC_CONFIG_REFSEL_Pos)  |
													(ADC_CONFIG_PSEL_Disabled                   << ADC_CONFIG_PSEL_Pos)    |
													(ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);
		NRF_ADC->EVENTS_END = 0;
		NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;

		NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
		NRF_ADC->TASKS_START = 1;

		while (!NRF_ADC->EVENTS_END)
		{
		}

		uint16_t vbg_in_mv = 1200;
		uint8_t adc_max = 255;
		uint16_t vbat_current_in_mv = (NRF_ADC->RESULT * 3 * vbg_in_mv) / adc_max;
		//printf("vBAT: %d\n", vbat_current_in_mv);

		startADC(true);
		
		nrf_delay_ms(100);
		return (uint8_t) (((vbat_current_in_mv - VBAT_MIN_IN_MV) * 100) / (VBAT_MAX_IN_MV - VBAT_MIN_IN_MV));
}

/** @snipet [stops ADC functions]*/
void startADC(bool state){
		if(!state){
				NRF_ADC->EVENTS_END     = 0;   // stop any running conversions
				NRF_ADC->TASKS_STOP     = 1;   // stop the ADC
		}
		else{
				NRF_ADC->EVENTS_END     = 0;   // stop any running conversions
				NRF_ADC->TASKS_START    = 1;   // start the ADC
		}
}

/** @snipet [stops/starts all interrupts]*/
void enableInterrupts(bool state){
		if(!state){
				startADC(false); 												      // stop ADC
				app_timer_stop(millis_timer_id);              // stop RTC timer event // stop millis
				nrf_gpio_cfg_output(piezoINTPin);             // set piezo pin 3 as output to play music
				nrf_drv_gpiote_in_event_disable(piezoINTPin); // stop pin interrupt (can do this onhandler using flag)
				interruptDisabled = true;
		}
		else{
			  startADC(true); 												           // start ADC
				startTimers();                                     // start RTC timer event // stop millis
				nrf_gpio_cfg_input(piezoINTPin, NRF_GPIO_PIN_NOPULL);
				nrf_drv_gpiote_in_event_enable(piezoINTPin, true); // start pin interrupt (can do this onhandler using flag)
				interruptDisabled = false;	
		} 
}

/**********************************************************************************************************************************************/




/*****************************************************************UART functions********************************************************/
/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event){
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
		uint32_t       err_code;
		static uint8_t index = 0;    
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;
            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
								printf("Error: %d\n", err_code);
								printf("conn_handle: %d\n", m_nus.conn_handle);
								printf("is_notification_enabled: %d\n", m_nus.is_notification_enabled);
								if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void){
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_ENABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

/*****************************************************************end of UART functions**************************************************/




/*****************************************************************Flash storage functions*************************************************/
// FDS library for SDK10 s110 is used for saving data to the flash memory

/**@brief Function for setting the system events for fds
 *
 * @details This function is called in the sys_evt_displatch function.
 *            For fds read and write to work, the following should be 
 *            added to the ble_stack_init function: 
 *            err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
 *            APP_ERROR_CHECK(err_code);
 */

static void fds_evt_handler(ret_code_t       result,
                            fds_cmd_id_t     cmd,
                            fds_record_id_t  record_id,
                            fds_record_key_t record_key){
    // Handle events here
		
}

/**@brief Function for registering and initializing fds
 */
static ret_code_t fds_initialize (void){
	ret_code_t retval = fds_register(fds_evt_handler); // register fds
	if(retval != NRF_SUCCESS){
			return retval;
	}
	retval = fds_init();                               // initialize fds
	if (retval != NRF_SUCCESS){
			//printf(">> fds intialization not successful\n");
			nrf_delay_ms(2000);
			return retval;
	}
	//printf(">> fds intialization successful!\n");
	nrf_delay_ms(2000);
	return NRF_SUCCESS;
}

/**@brief Function for writing a new record using fds
 * 
 * @param[in] keyType, keyInstance and the data to be written onto the flash memory
 */
static ret_code_t fds_write_record(uint32_t  data, uint32_t keyType, uint32_t keyInstance){
		ret_code_t         retval;
		fds_record_key_t   key;
		fds_record_chunk_t chunk;
		fds_record_desc_t  descriptor;
		// Fill in the record keys.
		key.type     = keyType;
		key.instance = keyInstance;
			
		// Initialize the record chunk, pointing it to the data to be stored.
		chunk.p_data       = &data;
		chunk.length_words = 1;
		// Write the record to flash.
		retval = fds_write(&descriptor,
											key,
											1 /*number of chunks*/,
											&chunk);
		if (retval != NRF_SUCCESS){
				printf(">> writing not successful!\n");
				//nrf_delay_ms(2000);
				return retval;
		}
		// The command was queued. Wait for a callback signalling its completion.
		// printf(">> writing successful!\n");
		// nrf_delay_ms(2000);
		return NRF_SUCCESS;
}

/**@brief Function for reading data from the flash memory
 * 
 * @param[in] keyType, keyInstance of record whose contents are to be read from
 *
 * @param[out] retData outputs the data that has been read from the record
 */
static ret_code_t fds_read(uint32_t *retData, uint32_t keyType, uint32_t keyInstance){
		ret_code_t         retval;
		typedef uint32_t  my_type;
		my_type           data;
		fds_type_id_t     type;
		fds_instance_id_t inst;
		fds_find_token_t  tok;
		fds_record_desc_t descriptor;
		fds_record_t      record;
		
		// Set the record keys.
		type = keyType;
		inst = keyInstance;
	
		// initialize this flag as false before searching for record
		existingRecord = false; 
	
		// Loop until all records with the given key pair has been found.
		//printf(">> searching for record\n");
	  //nrf_delay_ms(1000);
		while (fds_find(type, inst, &descriptor, &tok) == NRF_SUCCESS){
				existingRecord = true; 						// indicate that record exists
				fds_open(&descriptor, &record);		// A record was found. Open the record to check its contents.
				data = (uint32_t)(*record.p_data);// Cast the data.
				*retData = data;									// copy read data to the variable you passed to this function
				
				// When you are done, close the record using its descriptor.
				retval = fds_close(&descriptor);
				if (retval != NRF_SUCCESS){
					return retval;	
				}
		}
		return NRF_SUCCESS;
}

/**@brief Function for deleting data from the flash memory
 * 
 * @param[in] keyType, keyInstance of record whose contents are to be read from
*/
/*static ret_code_t fds_find_and_delete (uint32_t keyType, uint32_t keyInstance){
		ret_code_t         retval;
		fds_type_id_t     type;
		fds_instance_id_t inst;
		fds_find_token_t  tok;
		fds_record_desc_t descriptor;
		//fds_record_t      record;
		// Set the record keys.
		type = keyType;
		inst = keyInstance;
		printf(">> searching for record\n");
		// Loop until all records with the given key pair has been found.
		while (fds_find(type, inst, &descriptor, &tok) == NRF_SUCCESS)
		{
				retval = fds_clear(&descriptor);
				if (retval != NRF_SUCCESS)
				{
						return retval;	
				}
				printf(">> Record found and deleted\n");
		}
		ret_code_t ret = fds_gc();
		if (ret != NRF_SUCCESS)
		{
				return ret;
		}
		return NRF_SUCCESS;
}*/  

/**@brief Function for updating an existing record using fds
 * 
 * @param[in] keyType, keyInstance and the data to be updated onto the flash memory
 */
static ret_code_t fds_update_record (uint32_t data, uint32_t keyType, uint32_t keyInstance){
		ret_code_t         retval;
		fds_record_key_t   key;
		fds_record_chunk_t chunk;
		fds_type_id_t     type;
		fds_instance_id_t inst;
		fds_find_token_t  tok;
		fds_record_desc_t  descriptor;
	
		// Fill in the record keys.
		type = key.type     = keyType;
		inst = key.instance = keyInstance;
			
		// Initialize the record chunk, pointing it to the data to be stored.
		chunk.p_data       = &data;
		chunk.length_words = 1;
		while (fds_find(type, inst, &descriptor, &tok) == NRF_SUCCESS){
			// Write the record to flash.
			retval = fds_update(&descriptor,
												key,
												1 /*number of chunks*/,
												&chunk);
			if (retval != NRF_SUCCESS)
			{
					return retval;
			}
		}
		// The command was queued. Wait for a callback signalling its completion.
		return NRF_SUCCESS;
}

/**@brief Function that manages the saving and updating of records on the flash memory
 */
static void saveData(){
		uint32_t err_code;
		static uint32_t savedData;
		for(uint8_t i = 0; i < 10; i++){
				err_code = fds_read(&savedData, type[i], instance[i]);                                // check if record exists
				nrf_delay_ms(100);
				// check if record exists
				//printf("\nRecord: %d\n", i+1);
				//printf("Value in Flash: %d\n", savedData);
				//printf("Value in variable: %d\n", settings.deviceSettings[i]);
				if(existingRecord && savedData != settings.deviceSettings[i]){                        // Record exists, value has been updated
						err_code = fds_update_record(settings.deviceSettings[i], type[i], instance[i]);  // update record
						nrf_delay_ms(100);
						while(err_code != NRF_SUCCESS);                                                 // wait for record to be written/updated
						//printf(">>Updating record: %d\n", i+1);
						//nrf_delay_ms(100);
				}
				else if(!existingRecord && savedData != settings.deviceSettings[i]){                  // record does not exist, value has been updated
						err_code = fds_write_record (settings.deviceSettings[i], type[i], instance[i]);   // create new record
						nrf_delay_ms(100);
						while(err_code != NRF_SUCCESS);                                                 // wait for record to be written/updated
						//printf(">>Creating record: %d\n", i+1);
						//nrf_delay_ms(100);
				}                                                     			
				/*/debug code
				do{
						err_code = fds_read(&samp, type[i], instance[i]);
						nrf_delay_ms(100);
						printf(".");
				}while(!existingRecord);
				printf("New Value is: %d\n",samp);*/
				
		}
}

/**@brief Function for retrieving saved settings from flash memory.
 */
static void retrieveData(){
		uint32_t err_code;
		//printf(">> Retrieving data\n");
		for(uint8_t i = 0; i < 10; i++){
				do{
						err_code = fds_read(&settings.deviceSettings[i], type[i], instance[i]); // copy stored variable from flash memory
						nrf_delay_ms(100);
						if(!existingRecord)settings.deviceSettings[i] = defaultSettings[i];     // no stored data, set setting to default setting
						//printf("%d,", settings.deviceSettings[i]);
				}while(err_code != NRF_SUCCESS);
		}
		//printf("\n");
}
/********************************************************End of Flash storage functions***************************************************/



/********************************************************Process Functions for device opeartion*******************************************/
// NAME:          FFT.
// PARAMETERS:  
//    float *Rdat    [in, out] - Real part of Input and Output Data (Signal or Spectrum)
//    float *Idat    [in, out] - Imaginary part of Input and Output Data (Signal or Spectrum)
//    int    N       [in]      - Input and Output Data length (Number of samples in arrays)
//    int    LogN    [in]      - Logarithm2(N)
// RETURN VALUE:  false on parameter error, true on success.
//_________________________________________________________________________________________
// NOTE: In this algorithm N and LogN can be only:
//       N    = 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384;
//       LogN = 2, 3,  4,  5,  6,   7,   8,   9,   10,   11,   12,   13,    14;
//_________________________________________________________________________________________
// FFT & ADC CONFIGURATION:
// sample rate - 50000 Hz (from ADC configuration)
// bins - 50000/ARRAY_SIZE/2 - 195,3125 Hz if ARRAY_SIZE = 128
// bin1: -97,65625..97,65625  Hz (center 0 Hz)
// bin2: 97,65625 ..292,96875 Hz (center 195,3125)
// bin3: 292,96875..488,28125 Hz
// bin4: 488,28125..683,59375 Hz
// bin5...
#define  NUMBER_IS_2_POW_K(x)   ((!((x)&((x)-1)))&&((x)>1))  // x is pow(2, k), k=1,2, ...

bool  FFT(float *Rdat, float *Idat, int N, int LogN) {
  // parameters error check:
  /*if((Rdat == NULL) || (Idat == NULL))                return false;
  if((N > 16384) || (N < 1))                            return false;
  if(!NUMBER_IS_2_POW_K(N))                             return false;
  if((LogN < 2) || (LogN > 14))                         return false;*/

  register int  i, j, n, k, io, ie, in, nn;
  float         ru, iu, rtp, itp, rtq, itq, rw, iw, sr;
  
  static const float Rcoef[14] =
  {  -1.0000000000000000F,  0.0000000000000000F,  0.7071067811865475F,
      0.9238795325112867F,  0.9807852804032304F,  0.9951847266721969F,
      0.9987954562051724F,  0.9996988186962042F,  0.9999247018391445F,
      0.9999811752826011F,  0.9999952938095761F,  0.9999988234517018F,
      0.9999997058628822F,  0.9999999264657178F
  };
  static const float Icoef[14] =
  {   0.0000000000000000F, -1.0000000000000000F, -0.7071067811865474F,
     -0.3826834323650897F, -0.1950903220161282F, -0.0980171403295606F,
     -0.0490676743274180F, -0.0245412285229122F, -0.0122715382857199F,
     -0.0061358846491544F, -0.0030679567629659F, -0.0015339801862847F,
     -0.0007669903187427F, -0.0003834951875714F
  };
  
  nn = N >> 1;
  ie = N;
  for(n=1; n<=LogN; n++) {
    rw = Rcoef[LogN - n];
    iw = Icoef[LogN - n];
    in = ie >> 1;
    ru = 1.0F;
    iu = 0.0F;
    for(j=0; j<in; j++) {
      for(i=j; i<N; i+=ie) {
        io       = i + in;
        rtp      = Rdat[i]  + Rdat[io];
        itp      = Idat[i]  + Idat[io];
        rtq      = Rdat[i]  - Rdat[io];
        itq      = Idat[i]  - Idat[io];
        Rdat[io] = rtq * ru - itq * iu;
        Idat[io] = itq * ru + rtq * iu;
        Rdat[i]  = rtp;
        Idat[i]  = itp;
      }
      sr = ru;
      ru = ru * rw - iu * iw;
      iu = iu * rw + sr * iw;
    }
    ie >>= 1;
  }
  for(j=i=1; i<N; i++) {
    if(i < j) {
      io       = i - 1;
      in       = j - 1;
      rtp      = Rdat[in];
      itp      = Idat[in];
      Rdat[in] = Rdat[io];
      Idat[in] = Idat[io];
      Rdat[io] = rtp;
      Idat[io] = itp;
    }
    k = nn;
    while(k < j){
      j   = j - k;
      k >>= 1;
    }

    j = j + k;
  }
  return true;
}

/**@brief   Function for anaysing the sampled frequencies.
 */
uint16_t maxFreqAnalysis(float * arr, uint16_t size) {
  float maxValNum = arr[0];
  uint16_t maxValIndex = 0;
  
  for (uint16_t i = 1; i < size; i++) {
    if (arr[i] > maxValNum) {
      maxValNum = arr[i];
      maxValIndex = i;
    }
  }
  return maxValIndex;
}

/**@brief   Function for detecting the characteristics of a baby's cry
 */
void isBabyCrying(){
		if (data1Ready == true || data2Ready == true) {
      //// part to check how FFT works
      uint8_t arrNum;
      if (data1Ready == true) arrNum = 0;
      else arrNum = 1;
      
      //// FFT and signal analysis
      for(uint16_t i=0; i<ARRAY_SIZE; i++) inArrayI[i] = 0.0;         
      FFT(&inArrayR[arrNum][0], &inArrayI[0], ARRAY_SIZE, LOGN_ARRAY_SIZE);     // calc real and im parts
      //static uint16_t fftRes[ARRAY_SIZE/2] = {0};                               // this is only for debug purposes
      for(uint16_t i = 0; i < ARRAY_SIZE/2; i++) {
        inArrayR[arrNum][i] = sqrt(inArrayR[arrNum][i]*inArrayR[arrNum][i]+inArrayI[i]*inArrayI[i]);    // calc energy
        //fftRes[i] = inArrayR[arrNum][i];                                        // this is only for debug purposes
				//printf("%d\n", fftRes[i]);
			}
      
      //// DSP analysis
      static uint16_t maxFreqIndex;
      maxFreqIndex = maxFreqAnalysis(&inArrayR[arrNum][0], ARRAY_SIZE/2);
      
      static uint16_t maxFreq;
      maxFreq = maxFreqIndex * BIN_STEP;
      
      static float maxFreqPower;
      maxFreqPower = inArrayR[arrNum][maxFreqIndex];
        
      static float averPower;
      averPower = 0;
      for(uint16_t i = 0; i < ARRAY_SIZE/2; i++) averPower += inArrayR[arrNum][i];
      averPower = ((averPower - maxFreqPower) / (ARRAY_SIZE/2 - 1));
      /*if(xnb > averPower){
				xnb = averPower;
				printf("%f\n", xnb);
			}*/
      static uint16_t hapr;
      if (averPower < 0.1) hapr = 0;                                            // important, if hapr is too low let it be equal to 0
      else hapr = (uint16_t)(((float)maxFreqPower/(float)averPower)*10.0);
      
      // statistics collection
#define ARRAY_SIZE_2POW         8      
#define STAT_ARRAY_SIZE         256                 // size of data array we want to analyze
      static uint16_t dataCounter = 0; 
      static uint16_t maxFreqArr[STAT_ARRAY_SIZE], maxFreqPowerArr[STAT_ARRAY_SIZE], 
                      averPowerArr[STAT_ARRAY_SIZE], haprArr[STAT_ARRAY_SIZE], silArr[STAT_ARRAY_SIZE];
      
      maxFreqArr[dataCounter] = maxFreq;                        // 
      maxFreqPowerArr[dataCounter] = (uint16_t) maxFreqPower;
      averPowerArr[dataCounter] = (uint16_t) averPower;
      haprArr[dataCounter] = hapr;
			//printf("AverPower: %f", averPower);
      if (averPower < SILENCE_TRSHLD) silArr[dataCounter] = 1;
      else silArr[dataCounter] = 0;
      //printf("SLNCTHRS: %d\n", SILENCE_TRSHLD);
      if(++dataCounter > (STAT_ARRAY_SIZE-1)) dataCounter = 0;
      // dataCounter goes from 0 to STAT_ARRAY_SIZE value every time this block is executed
      // because of it we have maxFreqArr[], maxFreqPowerArr[], averPowerArr[], haprArr[], silArr[]
      // filled with most actual data. We can calc average value of last STAT_ARRAY_SIZE data for example
      // To make system less inertial it makes sense to reduce STAT_ARRAY_SIZE value (don't forget to change ARRAY_SIZE_2POW too)
      
      // perform some calculations
      static int32_t maxFreqTt;
      static int32_t maxFreqPowerTt;
      static int32_t averPowerTt;
      static int32_t haprTt;
      static int16_t silTt;
      maxFreqTt = maxFreqPowerTt = averPowerTt = haprTt = silTt = 0;
      
      for (uint16_t i = 0; i < STAT_ARRAY_SIZE; i++) {
        maxFreqTt += maxFreqArr[i];
        maxFreqPowerTt += maxFreqPowerArr[i];
        averPowerTt += averPowerArr[i];
        haprTt += haprArr[i];
        silTt += silArr[i];
      }
			//printf("%d\n", silTt);
      maxFreqTt >>= ARRAY_SIZE_2POW;            // calc average values from last STAT_ARRAY_SIZE measurements
      maxFreqPowerTt >>= ARRAY_SIZE_2POW;       // calc average values from last STAT_ARRAY_SIZE measurements
      averPowerTt >>= ARRAY_SIZE_2POW;          // calc average values from last STAT_ARRAY_SIZE measurements
      haprTt >>= ARRAY_SIZE_2POW;               // calc average values from last STAT_ARRAY_SIZE measurements
      
      // let's compare calc results with the pattern
      static uint8_t cryDetectedTimes = 0;
      cryDetected = false;       
      // We made a lot of calculations above this line
      // Every line with "if()" below checks one of the parameters
      // Let's decide if this sample is can characterized as signal from baby cry
      // You can comment any line you want and see how it will work without it
			//printf("haprTt: %d", haprTt);
			//printf(" HAPR: %d\n", HAPR);
			//printf("%d\n", haprTt);
			//printf("%d\n", averPowerTt);
			printf("%d\n", maxFreqTt);
			//printf("%d\n", silTt);
			if (haprTt > HAPR){                                // ratio dominant frequency's energy to average energy shiold be higher then 
				//printf("%d\n", haprTt);
				//printf("averPowerTt: %d", averPowerTt);
				//printf(" AVER_POWER: %d\n", AVER_POWER);
				if (averPowerTt > AVER_POWER){                   // at the same time the sound should be loud, so averPowerTt should be higher than AVER_POWER value
				//printf("%d\n", averPowerTt);
				//printf("maxFreqTt: %d", maxFreqTt);
				//printf(" LOW_BAND: %d\n", LOW_BAND);          
					if(maxFreqTt > LOW_BAND){                      // dominant frequency should be higher then LOW_BAND value
					//printf("%d", maxFreqTt);
				  //printf("maxFreqTt: %d", maxFreqTt);
					//printf(" HIGH_BAND: %d\n", HIGH_BAND);
						if (maxFreqTt < HIGH_BAND){                  // dominant frequency should be lower then HIGH_BAND value
              //printf("silTt: %d", silTt);
							//printf(" NS: %d\n", NUM_SILENT_SAMPLES);
							if (silTt > NUM_SILENT_SAMPLES) {         // since baby have to stop cry to breathe sometimes, there should be some samples with silent
                                                        // try to remove it first because it's depend on the noise parameters of your hardware
                //printf("G ");
								//if (cryDetectedTimes < 200) cryDetectedTimes++;   
              }
              else {
								//printf("L ");
                //if (cryDetectedTimes > 0) cryDetectedTimes--;   
              }
						}
					}
				}
			}
			/*if (haprTt > HAPR && averPowerTt > AVER_POWER && (maxFreqTt > LOW_BAND && maxFreqTt < HIGH_BAND) && silTt > NUM_SILENT_SAMPLES) {
					/*if (silTt > NUM_SILENT_SAMPLES) {
						if (cryDetectedTimes < 200) cryDetectedTimes++;
					}
					else {
						if (cryDetectedTimes > 0) cryDetectedTimes--;
					}*/
					/*if (cryDetectedTimes < 200) cryDetectedTimes++;
      }
			else{
					if (cryDetectedTimes > 0) cryDetectedTimes--;
			}*/
      //printf("%d\n", cryDetectedTimes);
			if (cryDetectedTimes > 100){
					cryDetected = true;   // to avoid triggering from noise those conditions should be met at least 100 times
					//cryDetectedTimes = 0;
					//printf("%d\n", cryDetected);
			}
			else{
					cryDetected = false;
					//printf("%d\n", cryDetected);
			}
      
      if (arrNum == 0) data1Ready = false;
      else data2Ready = false;
			//printf("\n\n");
    }
}


/**@breif Detecting knocks on the device */
/**@details  IRQ for detecting when the piezo is knocked. 
 *           Hardware interrupt on pin 3
 */
/**@snippet Hardware interrupt IRQ */
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
	 if(!interruptDisabled){
		 if((millis - timeTrack) >= sampleDuration){
				taps++;              // increment number of taps
				timeTrack = millis;  // record time of tap
		 }
	 }
}

/** @brief Procedure that changes the pin states for piezo control
 *  @details  changes the piezo buzzer pin states and enables/disables 
 *            interrupts on those pins depending on the value in variable x passed into the function. 
 */
/**@snippet Hardware interrupt IRQ */
void changePinStates(bool x){
  if(x){
		enableInterrupts(true);                                        // enable all interrupts except ones used by Softdevice and pwm
		/*if(BLEActivated){
			nrf_gpio_cfg_input(piezoINTPin, NRF_GPIO_PIN_NOPULL);        // set piezo pin 3 as input  
		}
		else{	
			//__enable_irq();
			nrf_gpio_cfg_input(piezoINTPin, NRF_GPIO_PIN_NOPULL);	
			nrf_drv_gpiote_in_event_enable(piezoINTPin, true); 		
		}	*/		
  }
  else{
		enableInterrupts(false);                                       // disable all interrupts except ones used by Softdevice and pwm
		/*if(BLEActivated){
			nrf_gpio_cfg_output(piezoINTPin);                            // set piezo pin 3 as output to play music
			nrf_drv_gpiote_in_event_disable(piezoINTPin);								 // disable interrupt on pin 3
		}
		else{
			nrf_gpio_cfg_output(piezoINTPin);														 // set piezo pin 3 as output to play music                                            
			nrf_drv_gpiote_in_event_disable(piezoINTPin);
			//__disable_irq();                         
		}*/                                      
  }
}  

/** @brief Procedure blinks battery indicator LEDs 
 *  @details  The procedure controls the number of times the battery indicator LEDs will blink 
 *            depending on the battery voltage. Since the battery is usually 2V (65% of voltage) when the charge 
 *						gets depleted, the battery level will be calculated between the range of 65% to 100%
 */
/**@snippet [Blink battery indicator LEDs] */
void batteryLevelCheck(){
  uint8_t x = battery_level_get();                     // read battery voltage level in percentage in the range of 0 to 3 V
  int num = (x * 10) / 100;                     			 // calculate number of times LEDs should blink
	for(uint8_t i = 0; i <= num; i++){                   // blink the Battery indicator LEDs
		analogWrite(0, settings.deviceSettings[1]);        // glow leds with set brightness
		analogWrite(1, settings.deviceSettings[1]);
    nrf_delay_ms(200); 
    analogWrite(0, 0);                                 // turn off leds
		analogWrite(1, 0);
    nrf_delay_ms(200);
  }
}


/** @brief Procedure that wakes up module from deep sleep
 *   
 *  @details This function plays the on tone and wakes the device from deep sleep
 *					 The module is woken up by input on pin 3 
 */
/** @snipet Wake up module*/
void turnOn(){
  for(uint8_t i = 0; i <= settings.deviceSettings[1]; i++){ // glow LED
    analogWrite(0, i); 
		analogWrite(1, i);
    nrf_delay_ms(4);
  }        
  if(settings.deviceSettings[0] > 0){												// if volume is not set to 0
    changePinStates(0);   																	// set pin 3 as output to allow piezo to play and disable interrupts
    onOffTone(1); 																					// sound startup tone
    changePinStates(1); 																		// return pin 3 to INPUT and enable interrupts 
  } 
  //nrf_delay_ms(1000);
  batteryLevelCheck(); 																			// indicate the battery voltage on the indicator LEDs
	nrf_delay_ms(100);                                        // short delay between change in ADC configuration
	adcConfig();                            								  // reconfigure ADC so that it can start sampling audio
	nrf_adc_start();                        									// start ADC conversion
  deviceState = ON; 																				// indicate device is ON
  taps = 0; 																								// initialize the number of taps to zero
  BLEWindow = true; 																				// start the BLE window
  bleWindowTimer = millis; 																	// mark time BLE window opens
  onTime = millis / 1000; 																	// mark time device comes On
  printf(">>Device on\n");
  //printf(">>BLE window: ");printf("%d\n", BLEWindow);
}

/** @brief Procedure that sends module to sleep
 *   
 *  @details This function ends the bluetooth connection, plays the off tone, saves settings
 *					 and sends the module to sleep. The module is woken up by input on pin 4 
 */
/** @snipet Force module to enter deep sleep*/
void sendToSleep(){
		if(deviceState == ON){
			// save available updated settings
			if(saveUpdates == true){
					saveData();
					nrf_delay_ms(100);
					//printf(">>Data saved successfully!!\n");
					updateReceived = false;                                   // indicate that there is no new data that has been received
					saveUpdates = false;
			}
			for(int i = settings.deviceSettings[1]; i >= 0; i--){     // fade out LED
					analogWrite(0, i); 
					analogWrite(1, i);
					nrf_delay_ms(4);
			}   
			if(settings.deviceSettings[0] > 0){
					changePinStates(0);                                      // set pin 2 as output to allow piezo to play and disable interrupts
					onOffTone(0);                                            // sound shutdonwn tone
					changePinStates(1);                                      // return pin 2 to INPUT and enable interrupts
			}
			deviceState = OFF;                                           // indicate that device has gone to deep sleep
		}
		taps = 0;                                                    // initialize the number of taps to zero
		printf(">>Device shutdown\n");
		nrf_delay_ms(2000);
		sleep_mode_enter();                                          // enter deep sleep
}


/** @brief Procedure that processes taps on the device
 *   
 *  @details This function checks to see how many taps have been made 
 *					 5 taps when device is OFF, plays ON tune and wakes up device
 *					 5 taps when device is ON, plays OFF tune and puts the device into deep sleep
 * 					 5 taps when the device is in the BLE window turns on the bluetooth
 */
/** @snipet Control device using taps*/
void knockDetect(){
  if(!BLEWindow){
    if (taps >= numberOfTaps && deviceState == ON){ // 5 taps while device is ON
      //printf(">>5 taps detected\n");
			goToSleep = true;                                            // indicate to device that it should go to sleep
    }
    else if (taps >= numberOfTaps - 3 && deviceState == OFF){ // 5 taps while device is OFF
      //printf(">>5 taps detected\n");
      turnOn(); // turn on device
    }
  }
  else{
    if(millis - bleWindowTimer < 10000){										 // BLE window is open. User can activate BLE through tapping 5 times
      if(taps >= numberOfTaps){														   // BLE advertising started after 5 taps
        BLEDisconnected = false;                             // flag for determining if device is not connected
				BLEWindow = false;                                   // exit the BLE window
				BLEActivated = true;                                 // BLE is activated
        taps = 0;                                            // initialize the number of taps to zero
        bleWindowTimer = millis;                             // record time BLE was activated
        timer = 0;
        advertising(true);                                   // start advertising device
				//printf(">>BLE activated!\n");
        //printf(">>BLE activation window closed!\n");
        return;
      }
    }
		else if((millis - bleWindowTimer > 10000) && (millis - bleWindowTimer < 30000)){
				// if the BLE window is open for more than 10 seconds it closes preventing
				// user from activating BLE using taps
				BLEWindow = false;
				taps = 0; // initialize the number of taps to zero
				//printf(">>BLE activation window closed!\n");
				return;
		}
  }
}


/** @brief Procedure that plays a tone on the Piezo
 *   
 *  @details This function plays the selected tone on the piezo buzzer by toggling the pizo pins
 *					 low and high using the time delays assigned to each tone. The volume is also adjusted 
 *					 depending on the settings provided by the user
 * 					 
 */
/** @snipet [Play selected tone]*/
void playTone(int tone, int duration, int led) {
  int x = 0;
  if(led){
    if(fade == 0 || fade == 100)x = 20;
    else x = 7;
  }
  //volume adjustment
  float volume = (float)((float)(settings.deviceSettings[0] * 45) / (float)100);         //map(settings.deviceSettings[0], 0, 100, 0, 45)
  volume /= 1000;
  int delay1 = ((float)(tone)* 2) * (1 - volume);
  int delay2 = ((float)(tone)* 2) * volume;
  if(led && fadeTrack != 0){
		if(delay1 > x)delay1 = delay1 - x;
	}
	for (long i = 0; i < duration * 1000L; i += tone * 2) {
    nrf_gpio_pin_clear(piezoINTPin);
		nrf_delay_us(delay1);
		nrf_gpio_pin_set(piezoINTPin);
    nrf_delay_us(delay2); //tone-x
    if(led && fadeTrack != 0){
			analogWrite(0,fade);
			analogWrite(1,fade);
			//nrf_delay_ms(1);
		}
  }
}

/** @snipet [Play note]*/
void playNote(int note, int duration, int led) {
  double c = (float)(1)/(float)(note);
  c *=1000000;
  c/=2;
  playTone((int)(c), duration,led);
}

/** @brief Procedure that plays the on or off note
 *   
 *  @details This function plays the on note when the index passed to it is 1 and plays the off note
 *					 when the index passed to it is 0
 * 					 
 */
/** @snipet [Play the or off note]*/
void onOffTone(int index){
  if(index == 1){
    for (int i = 0; i < 2; i++) {
      if (ONNote[i] == 0) {
        nrf_delay_ms(ONbeats[i] * tempo); // rest
      } else {
        playNote(ONNote[i], ONbeats[i] * 100, 0);
      }
      // pause between notes
      nrf_delay_ms(tempo / 2); 
    }
  }
  else if(index == 0){
    for (int i = 0; i < 2; i++) {
      if (OFFNote[i] == 0) {
        nrf_delay_ms(ONbeats[i] * tempo); // rest
      } else {
        playNote(OFFNote[i], ONbeats[i] * 100, 0);
      }
      // pause between notes
      nrf_delay_ms(tempo / 2); 
    }
  }
}

/** @snipet [Play the test note when changing settings]*/
void test(){
  playNote(1000, 2 * 100, 0);
}


/** @brief Procedure that plays the selected lullaby
 *   
 *  @details These function play the lullaby that has been selected
 *					 The Lullaby indexes are passed onto the procedure via variable i
 *           The variable settings.deviceSettings[8] is used to determine whether the LED should pulse
 *           simultaneously as the lullaby plays.
 * 					 
 */
/** @snipet [Play lullaby]*/
void lullabySong(uint8_t i){
  bool led = false;
  if(settings.deviceSettings[8] == 2)led = true;                // led will pulse simulatenously as the lullaby plays
  switch(i){
    case 0:																											// first lullaby
      for (int i = 0; i < 4; i++) {
        if (lullaby0[i] == 0) {
          nrf_delay_ms(lullabyBeats0[i] * tempo); // rest
        } else {
          playNote(lullaby0[i], lullabyBeats0[i] * tempo,led);
        }
        // pause between notes
        nrf_delay_ms(tempo / 2); 
      }
      break;
    case 1:																											// second lullaby
      for (int i = 0; i < 5; i++) {
        if (lullaby1[i] == ' ') {
          nrf_delay_ms(lullabyBeats1[i] * tempo); // rest
        } else {
          playNote(lullaby1[i], lullabyBeats1[i] * tempo,led);
        }
        // pause between notes
        nrf_delay_ms(tempo / 2); 
      }
      break;
    case 2:																										 // third lullaby
      for (int i = 0; i < 18; i++) {
        if (lullaby2[i] == ' ') {
          nrf_delay_ms(lullabyBeats2[i] * tempo); // rest
        } else {
          playNote(lullaby2[i], lullabyBeats2[i] * tempo,led);
        }
        // pause between notes
        nrf_delay_ms(tempo / 2); 
      }
      break;
    case 3:																										// fourth lullaby
      for (int i = 0; i < 14; i++) {
        if (lullaby3[i] == ' ') {
          nrf_delay_ms(lullabyBeats3[i] * tempo); // rest
        } else {
          playNote(lullaby3[i], lullabyBeats3[i] * tempo,led);
        }
        // pause between notes
        nrf_delay_ms(tempo / 2); 
      }
      break;
  }
}

/** @brief Procedure that controls the lullaby playing event
 *   
 *  @details This function manages the different aspects of the lullaby playing event depending on the user settings.
 *					 It uses settings.deviceSettings[8] to determine when the LEDs should pulse
 *           It uses settings.deviceSettings[3] to determine the number of LED pulses
 *           settings.deviceSettings[7] determines the lullaby melody to play
 *           settings.deviceSettings[0] determines the volume of the melody
 *					 settings.deviceSettings[2] determines the number of times the melody will be played
 *           settings.deviceSettings[4] determines whether the first cycle will be louder than the rest
 * 					 
 */
/** @snipet [Control how lullaby plays]*/
void playLullaby(){
  if(settings.deviceSettings[8] == 2){   // play melody simulateneously
		if(settings.deviceSettings[3] > 0){
			startLedPulses();                  // start RTC1 timer for LED pulses
		}
    uint8_t buff = 0;
    if(settings.deviceSettings[7] != 4 && settings.deviceSettings[0] > 0){
      for(int i = 0; i < settings.deviceSettings[2]; i++){
        // if 1st cycle is set to be louder 
        if(settings.deviceSettings[4] == 1){
          if(i == 0){
            buff = settings.deviceSettings[0]; // default volume
            settings.deviceSettings[0] = 100;  // set maximum volume
          }
          else{
            settings.deviceSettings[0] = buff; // revert back to default volume
          }
        }
        lullabySong(settings.deviceSettings[7]);
        for(int i = 0; i < 1000; i++){
          analogWrite(0, fade);
					analogWrite(1, fade);
          nrf_delay_ms(1);
        }
				babyCrying = false;
      }
    }
    babyCrying = false;
    if(!BLEActivated)changePinStates(1);
  }
  else if(settings.deviceSettings[8] == 0){ // play melody before light pulses
		uint8_t buff = 0;
    if(settings.deviceSettings[7] != 4 && settings.deviceSettings[0] > 0){
      for(int i = 0; i < settings.deviceSettings[2]; i++){
        // if 1st cycle is set to be louder 
        if(settings.deviceSettings[4] == 1){
          if(i == 0){
            buff = settings.deviceSettings[0]; // default volume
            settings.deviceSettings[0] = 100; // set maximum volume
          }
          else{
            settings.deviceSettings[0] = buff; // revert back to default volume
          }
        }
        lullabySong(settings.deviceSettings[7]); // play lullaby
      }
    }
    // light pulses
    if(settings.deviceSettings[3] > 0){ 
			startLedPulses();
      for(int i = 0; i < 10000; i++){ // can go up to 50 pulses
        analogWrite(0, fade);
			  analogWrite(1, fade);
        if(fadeTrack == 0 && fade == 0) break;
        nrf_delay_ms(10);
      }
      nrf_gpio_pin_set(BATLED1);
			nrf_gpio_pin_set(BATLED2);
    }
    babyCrying = false;
    if(!BLEActivated)changePinStates(1);
  }
  else if(settings.deviceSettings[8] == 1){ // play melody after light pulses
		uint8_t buff = 0;
    // light pulses
    if(settings.deviceSettings[3] > 0){ 
			startLedPulses();
      for(int i = 0; i < 10000; i++){ // can go up to 50 pulses
        analogWrite(0, fade);
				analogWrite(1, fade);
        if(fadeTrack == 0 && fade == 0) break;
        nrf_delay_ms(10);
      }
      nrf_gpio_pin_set(BATLED1);
			nrf_gpio_pin_set(BATLED2);
    }
    //play lullaby
    if(settings.deviceSettings[7] != 4 && settings.deviceSettings[0] > 0){
      for(int i = 0; i < settings.deviceSettings[2]; i++){
        // if 1st cycle is set to be louder 
        if(settings.deviceSettings[4] == 1){
          if(i == 0){
            buff = settings.deviceSettings[0]; // default volume
            settings.deviceSettings[0] = 100; // set maximum volume
          }
          else{
            settings.deviceSettings[0] = buff; // revert back to default volume
          }
        }
        lullabySong(settings.deviceSettings[7]); // play lullaby
      }
    }
    babyCrying = false;
    if(!BLEActivated)changePinStates(1);
  }
}

/****************************************************end of process functions*************************************************************/


/**@brief Application main function.
 */
int main(void)
{		
		uart_init();
		printf("Start configuration..\n");
	  nrf_delay_ms(100);
	
    uint32_t err_code;
    bool erase_bonds;
		ledPulseTime = (100 * 4)/settings.deviceSettings[1];        						// calculate time interval for fade
		
    
		ble_stack_init(); 																											// softdevice initialized here so no need to start the LFCLK
		APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);		// setup RTC1
		createTimers();																													// create RTC1 timer that will keep track of millis
		startTimers();                                                          // start RTC1 timer to keep track of millis
		attachInterrupt(0, piezoINTPin, 1);                        							// configure interrupt on pin 3
	
    buttons_leds_init(&erase_bonds);																				// initialize bsp
    gap_params_init();
    services_init();
    advertising_init();
		LEDS_ON(BSP_LED_3_MASK);                                                // led stays off when starting up
    conn_params_init();
		
		
		err_code = fds_initialize();
		APP_ERROR_CHECK(err_code);
	
		// define pin configurations
		nrf_gpio_cfg_output(BATLED1);
		nrf_gpio_cfg_output(BATLED2);
		//nrf_gpio_cfg_input(piezoAnalog, NRF_GPIO_PIN_NOPULL);
		
		
		retrieveData();                 																				// copy data from flash memory to settings	
		PWM_config();  																													// configure pulse width modulation                                       						
		printf("Start\n");
		nrf_delay_ms(100);
    // Enter main loop.
    for (;;){	
				__SEV();
				__WFE();
				__WFE();
				//babyCrying = true;
			  //changePinStates(0);
				//nrf_delay_ms(1000);
				//playLullaby();
				//printf("FT: %d\n", fadeTrack);
				//nrf_delay_ms(100);
				if(goToSleep || (((millis/1000) - onTime) > (settings.deviceSettings[6]*3600) && deviceState == ON)){		// forced shutdown or autoshutdown
						goToSleep = false; 
						sendToSleep();                                                  																		// send device to sleep
				}
				else{
						if(cryDetected && fadeTrack == 0){
								//printf(">>Baby crying\n");
								changePinStates(0);
								playLullaby();
								cryDetected = false;
						}
						else if(BLEActivated){
								power_manage();
								if(BLEDisconnected){
										printf("disconnecting");
										nrf_delay_ms(100);
										advertising(false);                                      // stop advertising
										BLEWindow = false;                                       // close BLE window
										BLEActivated = false;                                    // indicates that device is no longer advertising
										goToSleep = true;                                        // will signal module to go to sleep in the main loop
								}
						}
						else{
								//printf("Taps: %d\n", taps);
								//nrf_delay_ms(100);
								knockDetect();
								if (taps != 5 && (millis - timeTrack) > knockInterval && deviceState == OFF){     // accidental tap when device is in sleep mode
										//printf(">>Accidental tap->shutdown: %d\n", taps);
										taps = 0; // initialize taps
										nrf_delay_ms(1000);
										sendToSleep(); // send device to sleep
								}
								else if (taps != 5 && taps != 0 &&  deviceState == ON && (millis - timeTrack) > knockInterval){ // accidental tap when device is ON
										//printf(">>Accidental tap: %d\n", taps);
										taps = 0; // initialize taps
								}
								if(taps == 0 && deviceState == ON && !BLEActivated && !BLEWindow){
										//printf("detecting crying\n");
										isBabyCrying();
										//printf("%f\n", analogSample);
										//printf("%d\n", fadeTrack);
										//printf("%d\n", cryDetected);
								}
						}
				}
		}
}


/** 
 * @}
 */
