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
struct DeviceSettings{
  uint32_t deviceSettings[10];
};

static uint32_t defaultSettings[10] = {100, 100, 2, 8, 0, 0, 1, 1, 0, 0}; // default settings
struct DeviceSettings settings = {10, 100, 2, 8, 1, 0, 1, 3, 1, 0}; 
static uint32_t instance[10] = {0x2222, 0x2223, 0x2224, 0x2225, 0x2226, 0x2227, 0x22238, 0x2229, 0x222A, 0x222B};
static uint32_t type[10] = {0x1111, 0x1112, 0x1113, 0x1114, 0x1115, 0x1116, 0x1117, 0x1118, 0x1119, 0x111A};
static bool updateReceived = false;                       // set whenever any value is updated
static bool saveUpdates = false;													// indicates that data can proceed to be saved
static char command[6] = "";                              // holds incoming data
static uint32_t sampleResult = 0;                         // for debugging 
static bool existingRecord = false;                       // holds state of records (either existing or not) for purposes of updating or creating new records
static char arrayNames[10] = {'V', 'L', 'N', 'P', 'S', 'W', 'A', 'M', 'B', 'O'}; // for debugging

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      30                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define START_STRING                    "Start...\n"                                /**< The string that will be sent over the UART when the application starts. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */





/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}




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
		if(cRef[i] == '\n') j = i - 1;
	}
	//printf("%d \n", j);
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
      //printf("play test tone, %d", settings.deviceSettings[0]);
			/*changePinStates(0);
      test();
      changePinStates(1);*/
    }
  }
  else if(c[0] == 'L'){
    settings.deviceSettings[1] = charArrayToInt(c, 1, 4);
    //analogWrite(BATLED, settings.deviceSettings[1]); // display LED brightness
    //ledPulseTime = (255 * 4)/settings.deviceSettings[1]; // calculate time interval for fade
    //Serial.print("timer setting");Serial.println(ledPulseTime);
    //timer1Setup(ledPulseTime); // setup the LED pulse timer to correspond to the brightness.
    //delay(1000);
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
      //lullabySong(settings.deviceSettings[7]);
      //changePinStates(1);
    }
  }
  else if(c[0] == 'B')settings.deviceSettings[8] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'O')settings.deviceSettings[9] = charArrayToInt(c, 1, 4);
  else if(c[0] == '#'){
    //RFduinoBLE.sendFloat(batteryLevel()); // send battery voltage to phone app
  }
  else if(c[0] == '$'){
    for(uint8_t i = 0; i < 9; i++)settings.deviceSettings[i] = defaultSettings[i]; // return device back to default settings
  }
  else if(c[0] == '@'){
    //sendData();
  }
}

/**@brief Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in] p_ble_evt S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						printf("Disconnected");
						if(updateReceived == true){
								saveUpdates = true;
						}
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
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        command[i] = p_data[i];                              // received data is copied to command array
				//printf("%c>", command[i]);													 // data in command array is printed onto Serial monitor
			  //while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
		extractData(command);
		/*for(uint8_t i = 0; i < sizeof(settings.deviceSettings); i++){
			printf("%c ", arrayNames[i]);
			printf("%d \n", settings.deviceSettings[i]);
		}*/
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
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
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
static void services_init(void)
{
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
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
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
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
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
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

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
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
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
static void sys_evt_dispatch(uint32_t sys_evt)
{
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
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
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
static void ble_stack_init(void)
{
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
	
		err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
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


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
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
static void uart_init(void)
{
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


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
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
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


static void fds_evt_handler(ret_code_t       result,
                            fds_cmd_id_t     cmd,
                            fds_record_id_t  record_id,
                            fds_record_key_t record_key){
    // Handle events here
		
}

static ret_code_t fds_initialize (void){
	ret_code_t retval = fds_register(fds_evt_handler);

	if(retval != NRF_SUCCESS)
	{
			return retval;
	}
	retval = fds_init();
	if (retval != NRF_SUCCESS)
	{
			printf(">> fds intialization not successful\n");
			nrf_delay_ms(2000);
			return retval;
	}
	printf(">> fds intialization successful!\n");
	nrf_delay_ms(2000);
	return NRF_SUCCESS;
}

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
		if (retval != NRF_SUCCESS)
		{
				//printf(">> writing not successful!\n");
				nrf_delay_ms(2000);
				return retval;
		}
		// The command was queued. Wait for a callback signalling its completion.
		// printf(">> writing successful!\n");
		// nrf_delay_ms(2000);
		return NRF_SUCCESS;
}

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
		// Loop until all records with the given key pair has been found.
		//printf(">> searching for record\n");
	  //nrf_delay_ms(1000);
		while (fds_find(type, inst, &descriptor, &tok) == NRF_SUCCESS)
		{
				//printf(">> record found\n");
				existingRecord = true; // indicates that record exists
				// A record was found. Open the record to check its contents.
				fds_open(&descriptor, &record);
				// Cast the data.
				data = (uint32_t)(*record.p_data);
				*retData = data;
				/*printf(" %d\n", data);
				//store the data in the selected 
				if (data == 100)
				{
						printf(">> DataMatch found! \n");		// Do something.	
				}
				else{
						printf(">> Data does not match! \n");		// Do something.
				}*/
				   
				//printf("Record data is: %d\n", data);
				//if (data == SOME_VALUE)
				//{
						// Do something.
				//}
				// When you are done, close the record using its descriptor.
				retval = fds_close(&descriptor);
				if (retval != NRF_SUCCESS)
				{
					return retval;	
				}
		}
		return NRF_SUCCESS;
}

static ret_code_t fds_find_and_delete (uint32_t keyType, uint32_t keyInstance){
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
}

static ret_code_t fds_update_record (uint32_t data, uint32_t keyType, uint32_t keyInstance){
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
		retval = fds_update(&descriptor,
											key,
											1 /*number of chunks*/,
											&chunk);
		if (retval != NRF_SUCCESS)
		{
				return retval;
		}
		// The command was queued. Wait for a callback signalling its completion.
		return NRF_SUCCESS;
}

static void saveData(){
		uint32_t err_code;
		static uint32_t savedData;
		for(uint8_t i = 0; i < 10; i++){
				err_code = fds_read(&savedData, type[i], instance[i]);
				nrf_delay_ms(100);
				// check if record exists
				printf("\nRecord: %d\n", i+1);
				printf("Value in Flash: %d\n", savedData);
				printf("Value in variable: %d\n", settings.deviceSettings[i]);
				if(existingRecord && savedData != settings.deviceSettings[i]){              // Record exists, value has been updated
						//do{
								err_code = fds_update_record (settings.deviceSettings[i], type[i], instance[i]);  // update record
								nrf_delay_ms(100);
						while(err_code != NRF_SUCCESS);                                                      // wait for record to be written/updated
						printf(">>Updating record: %d\n", i+1);
				}
				else if(!existingRecord && savedData != settings.deviceSettings[i]){         // record does not exist, value has been updated
						
						err_code = fds_write_record (settings.deviceSettings[i], type[i], instance[i]);   // create new record
						nrf_delay_ms(100);
						while(err_code != NRF_SUCCESS);                                                      // wait for record to be written/updated
						printf(">>Creating record: %d\n", i+1);
				}                                                     // wait for record to be written/updated
				existingRecord = false;                               // reset state of record
				
				//debug code
				do{
						err_code = fds_read(&sampleResult, type[i], instance[i]);
						nrf_delay_ms(100);
						printf(".");
				}while(!existingRecord);
				printf("New Value is: %d\n",sampleResult);
				existingRecord = false;                               // reset state of record
				
				nrf_delay_ms(1000);
		}
}

static void retrieveData(){
		uint32_t err_code;
		printf(">> Retrieving data\n");
		for(uint8_t i = 0; i < 10; i++){
				do{
						err_code = fds_read(&settings.deviceSettings[i], type[i], instance[i]);
						nrf_delay_ms(100);
						printf(".");
				}while(!existingRecord);
		}
		printf("\n");
}

static void advertising(bool state){
		uint32_t err_code;
		if(state){
				err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
				APP_ERROR_CHECK(err_code);
		}
		else{
				//err_code = bsp_indication_set(BSP_INDICATE_CONNECTED); // stop LED from blinking
				//APP_ERROR_CHECK(err_code);
				err_code = ble_advertising_start(BLE_ADV_MODE_IDLE); 	 // set advertising to IDLE to prevent false advertisement triggers
				APP_ERROR_CHECK(err_code);
				sd_ble_gap_adv_stop();                                 // stops advertisement
		}
}

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
    uint8_t  start_string[] = START_STRING;
    
    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    uart_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
		
    services_init();
    advertising_init();
    conn_params_init();
		err_code = fds_initialize();
		APP_ERROR_CHECK(err_code);
    retrieveData();                 // copy data from flash memory to settings
    printf("%s\n",start_string);
		
		
		//char cha[5] = "V5432";
		//printf("%d", charArrayToInt(cha, 1, 4));
		
		
		//err_code = fds_find_and_delete(0x1111, 0x2222);
		//nrf_delay_ms(1000);
		//APP_ERROR_CHECK(err_code);
		//err_code =fds_write_record(100, 0x1111, 0x2222);
		//nrf_delay_ms(5000);
		//APP_ERROR_CHECK(err_code);
		//wait until the write is finished. 
		//printf("Data written is %d\n", 0x0A);
		//while (err_code != NRF_SUCCESS);
		//fds_read(&sampleResult, 0x1111, 0x2222);
		//nrf_delay_ms(5000);
		//printf("Complete! Sample data is: %d\n", sampleResult);
    
		err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
		
    
    // Enter main loop.
    for (;;)
    {
				if(saveUpdates == true){
						advertising(false);   // stop advertising
						nrf_delay_ms(2000);		
						saveData();
						printf("Data saved successfully!!");
						updateReceived = false;                                   // indicate that there is no new data that has been received
						saveUpdates = false;
				}
				power_manage();
    }
}


/** 
 * @}
 */
