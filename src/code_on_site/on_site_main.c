#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"


#include <stdbool.h>
#include <stdio.h>
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "app_util_platform.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//#include "SerialClass.h"
//#include "app_uart.h"


#define SAMPLES_IN_BUFFER 1
#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_DISABLED  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;


typedef enum
{
    LOG_LEVEL_INFO  = 1,
    LOG_LEVEL_DEBUG = 2
} log_level;
#define LOG_LEVEL LOG_LEVEL_INFO



// #define TX_POWER_LEVEL -8
// #define TX_POWER_LEVEL -4
// #define TX_POWER_LEVEL 0
// #define TX_POWER_LEVEL 4
#define TX_POWER_LEVEL 8

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define UNEXPECTED_LED                  BSP_BOARD_LED_3                         /**< LED to be toggled when the an error occurs */
// #define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "SBMS_on_site"                                  /**< Name of device. Will be included in the advertising data. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

// #define TX_POWER_LEVEL -8
// #define TX_POWER_LEVEL 0
// #define TX_POWER_LEVEL 4
#define TX_POWER_LEVEL 8

// #define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

// #define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UUID_BASE {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define UUID_SERVICE 0x1234
#define UUID_DURATION_CHAR 0xabcd


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

ble_gatts_char_handles_t duration_handles;                             /** Duration Characteristic */



/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

// /**@brief Function for assert macro callback.
//  *
//  * @details This function will be called in case of an assert in the SoftDevice.
//  *
//  * @warning This handler is an example only and does not fit a final product. You need to analyze
//  *          how your product is supposed to react in case of Assert.
//  * @warning On assert from the SoftDevice, the system can only recover on reset.
//  *
//  * @param[in] line_num    Line number of the failing ASSERT call.
//  * @param[in] p_file_name File name of the failing ASSERT call.
//  */
// void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
// {
//     app_error_handler(DEAD_BEEF, line_num, p_file_name);
// }

/*
Descripttion : Sets up all the the LEDs used by the program
*/
static void leds_init(void)
{
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_DEBUG
    NRF_LOG_DEBUG("LED init");
    #endif
    bsp_board_init(BSP_INIT_LEDS);

}


// /**@brief Function for the Timer initialization.
//  *
//  * @details Initializes the timer module.
//  */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_DEBUG
    NRF_LOG_DEBUG("TIMER init");
    #endif
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void ble_gap_params_init(void)
{
    // ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    sd_ble_gap_ppcp_set(&gap_conn_params);
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_DEBUG
    NRF_LOG_DEBUG("GAP init");
    #endif
}


// /**@brief Function for initializing the GATT module.
//  */
static void ble_gatt_init(void)
{
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_DEBUG
    NRF_LOG_DEBUG("GATT init");
    #endif
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void ble_advertising_init(void)
{
    ble_uuid_t ble_uuid;
    ble_add_char_params_t add_char_params;
    ble_uuid128_t base_uuid = {UUID_BASE};
    uint8_t uuid_type;
    sd_ble_uuid_vs_add(&base_uuid, &uuid_type);
    ble_uuid.type = uuid_type;
    ble_uuid.uuid = UUID_SERVICE;
    uint16_t service_handle;

    sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service_handle);


    // Setup characteristic paramters

    memset(&add_char_params, 0 , sizeof(add_char_params));
    add_char_params.uuid_type = uuid_type;
    add_char_params.init_len = sizeof(uint16_t);
    add_char_params.max_len = sizeof(uint16_t);
    add_char_params.char_props.read = 1;
    add_char_params.char_props.notify = 1;
    add_char_params.read_access = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    add_char_params.uuid = UUID_DURATION_CHAR;
    characteristic_add(service_handle, &add_char_params, &voltage_low_handles);         //Setup duration characteristic



    // ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

    // Build and set advertising data.
    ble_advdata_t advdata;
    ble_advdata_t srdata;
    ble_uuid_t adv_uuids[] = {{UUID_SERVICE, uuid_type}};
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

     ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    

     ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

     sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_DEBUG
    NRF_LOG_DEBUG("Advertising init");
    #endif
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    // APP_ERROR_HANDLER(nrf_error); // Ignore errors TODO: 
}


/**@brief Function for initializing services that will be used by the application.
 */
static void ble_services_init(void)
{
    // ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    nrf_ble_qwr_init(&m_qwr, &qwr_init);
    // APP_ERROR_CHECK(err_code);
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_DEBUG
    NRF_LOG_DEBUG("BLE Services init");
    #endif
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
     sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    // APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void ble_connection_params_init(void)
{

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

    ble_conn_params_init(&cp_init);
    // APP_ERROR_CHECK(err_code);
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_DEBUG
    NRF_LOG_DEBUG("BLE connection paramters init");
    #endif
}


/**@brief Function for starting advertising.
 */
static void ble_advertising_start(void)
{
    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, TX_POWER_LEVEL);
    sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    bsp_board_led_on(ADVERTISING_LED);
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_INFO
    NRF_LOG_DEBUG("Advertising Init");
    #endif
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
  

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_INFO
            NRF_LOG_INFO("Connected");
            #endif
            bsp_board_led_on(CONNECTED_LED);
            bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, TX_POWER_LEVEL);
          nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            
            
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_INFO
            NRF_LOG_INFO("Disconnected");
            #endif
            bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            // err_code = app_button_disable();
            // APP_ERROR_CHECK(err_code);
            ble_advertising_start();
            break;

        // case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        //     // Pairing not supported
        //     err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
        //                                            BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
        //                  NULL,
        //                                            NULL);
        //     APP_ERROR_CHECK(err_code);
        //     break;

        // case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        // {
        //     NRF_LOG_DEBUG("PHY update request.");
        //     ble_gap_phys_t const phys =
        //     {
        //         .rx_phys = BLE_GAP_PHY_AUTO,
        //         .tx_phys = BLE_GAP_PHY_AUTO,
        //     };
        //     err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        //     APP_ERROR_CHECK(err_code);
        // } break;

        // case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        //     // No system attributes have been stored.
        //     err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
        //     APP_ERROR_CHECK(err_code);
        //     break;

        // case BLE_GATTC_EVT_TIMEOUT:
        //     // Disconnect on GATT Client timeout event.
        //     NRF_LOG_DEBUG("GATT Client Timeout.");
        //     err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        //                                      BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        //     APP_ERROR_CHECK(err_code);
        //     break;

        // case BLE_GATTS_EVT_TIMEOUT:
        //     // Disconnect on GATT Server timeout event.
        //     NRF_LOG_DEBUG("GATT Server Timeout.");
        //     err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        //                                      BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        //     APP_ERROR_CHECK(err_code);
        //     break;

        // default:
        //     // No implementation needed.
        //     break;
    }
}

/**@brief Function which inits the BLE stack(enables the stack, sets up the observer)
 */
static void ble_stack_init(void)
{
    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);

    // Enable BLE stack.
    nrf_sdh_ble_enable(&ram_start);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_DEBUG
        NRF_LOG_DEBUG("BLE Stack INIT");
    #endif
}



/**@brief Function which writes a value to the characteristic mentioned
 * 
 * @param  characteristic_value [in] Value being written to the characteristic
 * @param char_handle [in] handle of the characteristic
 */
void ble_write_to_characteristic(uint8_t characteristic_value, ble_gatts_char_handles_t char_handle)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(characteristic_value);
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = char_handle.value_handle;
    params.p_data = &characteristic_value;
    params.p_len = &len;
    sd_ble_gatts_hvx(m_conn_handle, &params);
}

/**@brief Funtion for the button handler that uses interrupts to check if a button has been pushed
 * 
 * @param pin [in] variable which holds the button which the handler is handling
 * @param action [in] variable which stores the action which the button is doing.
 */
static void button_handler(uint8_t pin, uint8_t action)
{
    if(pin == BSP_BUTTON_0){
        if(action == APP_BUTTON_PUSH){
            bsp_board_led_on(BSP_BOARD_LED_0);
        }
        else if (action == APP_BUTTON_RELEASE)
        {
            bsp_board_led_off(BSP_BOARD_LED_0);
        }
        ble_write_to_characteristic(action, duration_handles);
        
    }
}

/**@brief Function initializing the button and the soft device interrupt handler
 */
static void button_init(void)
{
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_DEBUG
        NRF_LOG_DEBUG("Button INIT");
    #endif
    nrf_sdh_enable_request();
    static app_button_cfg_t buttons[] ={
        {BSP_BUTTON_0, false, BUTTON_PULL, button_handler}
    };
    app_button_init(buttons, ARRAY_SIZE(buttons), APP_TIMER_TICKS(50));
    app_button_enable();
}

/**@brief Function collecting a sample from the GPIO pins
 */
void saadc_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    // NRF_LOG_INFO("Timer interrupt");
    
    NRF_SAADC->TASKS_SAMPLE = 1;
    /*
    // Alternatively, you can use the driver to trigger a sample:
    nrf_drv_saadc_sample();
    */
}

/**@brief Function to setup how frequently sampling should be done
 */
void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, saadc_timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 1000ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 1000);

    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
   err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
   APP_ERROR_CHECK(err_code);

   err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                         timer_compare_event_addr,
                                         saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function enabling the PPI(programmable peripheral interconnect) peripheral to allow the SAADC to sample without intervention from the CPU
 */
void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for setting up the SAADC callback function
 * 
 * param p_event Event descriptor for the SAADC event
 */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_INFO
        NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);
        #endif
        double V;
        uint8_t ADC_RESOLUTION = 12;
        uint8_t ADC_SCALING = 3;
        double ADC_REF_VOLTAGE = 3.6; //[V]
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            // TODO: Eventually add a string here with all the data 
            V = (((p_event->data.done.p_buffer[i] * ADC_REF_VOLTAGE) / (2^(ADC_RESOLUTION-1)) / ADC_SCALING));
            #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_INFO
                NRF_LOG_INFO("ADC val %d", p_event->data.done.p_buffer[i]);
                NRF_LOG_INFO("ADC Convertied Voltage [V] %f", V);
            #endif
            ble_write_to_characteristic(p_event->data.done.p_buffer[i], voltage_char_handles);
        }
        m_adc_evt_counter++;
    }
}

/**@brief Function for setting up the SAADC peripheral.
 */
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
    //Configure SAADC
    // nrf_drv_saadc_config_t saadc_config;
    // saadc_config.low_power_mode = true;                                                   //Enable low power mode.
    // saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=4096 (when input voltage is 3.6V for channel gain setting of 1/6).
    // saadc_config.oversample = SAADC_OVERSAMPLE;                                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    // saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_INFO
        NRF_LOG_INFO("SAADC INIT");
    #endif
}

/**@brief Function for setting up logs.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @brief BLE Heart Rate Collector application main file.
 *
 * This file contains the source code for a sample heart rate collector.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_util.h"
#include "app_error.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_hrs_c.h"
#include "ble_bas_c.h"
#include "app_util.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_ble_scan.h"


#define APP_BLE_CONN_CFG_TAG        1                                   /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_BLE_OBSERVER_PRIO       3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO       1                                   /**< Applications' SoC observer priority. You shouldn't need to modify this value. */

#define LESC_DEBUG_MODE             0                                   /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND              1                                   /**< Perform bonding. */
#define SEC_PARAM_MITM              0                                   /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC              1                                   /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS          0                                   /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES   BLE_GAP_IO_CAPS_NONE                /**< No I/O capabilities. */
#define SEC_PARAM_OOB               0                                   /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE      7                                   /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE      16                                  /**< Maximum encryption key size in octets. */

#define SCAN_DURATION_WITELIST      3000                                /**< Duration of the scanning in units of 10 milliseconds. */

#define TARGET_UUID                 CONFIGUATION_CHAR         /**< Target device uuid that application is looking for. */

/**@brief Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)


NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                    /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
BLE_HRS_C_DEF(m_hrs_c);                                             /**< Structure used to identify the heart rate client module. */
BLE_BAS_C_DEF(m_bas_c);                                             /**< Structure used to identify the Battery Service client module. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                    /**< DB discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                           /**< Scanning module instance. */

static uint16_t m_conn_handle;                                      /**< Current connection handle. */
static bool     m_whitelist_disabled;                               /**< True if whitelist has been temporarily disabled. */
static bool     m_memory_access_in_progress;                        /**< Flag to keep track of ongoing operations on persistent memory. */

/**< Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_param =
{
    .active        = 0x01,
#if (NRF_SD_BLE_API_VERSION > 7)
    .interval_us   = NRF_BLE_SCAN_SCAN_INTERVAL * UNIT_0_625_MS,
    .window_us     = NRF_BLE_SCAN_SCAN_WINDOW * UNIT_0_625_MS,
#else
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
#endif // (NRF_SD_BLE_API_VERSION > 7)
    .filter_policy = BLE_GAP_SCAN_FP_WHITELIST,
    .timeout       = SCAN_DURATION_WITELIST,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
};


/**@brief Names which the central applications will scan for, and which will be advertised by the peripherals.
 *  if these are set to empty strings, the UUIDs defined below will be used
 */
static char const m_target_periph_name[] = "";      /**< If you want to connect to a peripheral using a given advertising name, type its name here. */
static bool is_connect_per_addr = false;            /**< If you want to connect to a peripheral with a given address, set this to true and put the correct address in the variable below. */

static ble_gap_addr_t const m_target_periph_addr =
{
    /* Possible values for addr_type:
       BLE_GAP_ADDR_TYPE_PUBLIC,
       BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
       BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE,
       BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE. */
    .addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
    .addr      = {0x8D, 0xFE, 0x23, 0x86, 0x77, 0xD9}
};


static void scan_start(void);


/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling the Heart Rate Service Client and Battery Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_hrs_on_db_disc_evt(&m_hrs_c, p_evt);
    ble_bas_on_db_disc_evt(&m_bas_c, p_evt);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            // Bonds are deleted. Start scanning.
            scan_start();
            break;

        default:
            break;
    }
}


/**
 * @brief Function for shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected.");

            // Discover peer's services.
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
        } break;

        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected, reason 0x%x.",
                         p_gap_evt->params.disconnected.reason);

            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);

            if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        default:
            break;
    }
}


/**@brief SoftDevice SoC event handler.
 *
 * @param[in]   evt_id      SoC event.
 * @param[in]   p_context   Context.
 */
static void soc_evt_handler(uint32_t evt_id, void * p_context)
{
    switch (evt_id)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register handlers for BLE and SoC events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);
}



/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/** @brief Clear bonding information from persistent storage
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for disabling the use of whitelist for scanning.
 */
static void whitelist_disable(void)
{
    if (!m_whitelist_disabled)
    {
        NRF_LOG_INFO("Whitelist temporarily disabled.");
        m_whitelist_disabled = true;
        nrf_ble_scan_stop();
        scan_start();
    }
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            whitelist_disable();
            break;

        default:
            break;
    }
}


/**@brief Heart Rate Collector Handler.
 */
static void hrs_c_evt_handler(ble_hrs_c_t * p_hrs_c, ble_hrs_c_evt_t * p_hrs_c_evt)
{
    ret_code_t err_code;

    switch (p_hrs_c_evt->evt_type)
    {
        case BLE_HRS_C_EVT_DISCOVERY_COMPLETE:
        {
            NRF_LOG_DEBUG("Heart rate service discovered.");

            err_code = ble_hrs_c_handles_assign(p_hrs_c,
                                                p_hrs_c_evt->conn_handle,
                                                &p_hrs_c_evt->params.peer_db);
            APP_ERROR_CHECK(err_code);

            // Heart rate service discovered. Enable notification of Heart Rate Measurement.
            err_code = ble_hrs_c_hrm_notif_enable(p_hrs_c);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_HRS_C_EVT_HRM_NOTIFICATION:
        {
            NRF_LOG_INFO("Heart Rate = %d.", p_hrs_c_evt->params.hrm.hr_value);

            if (p_hrs_c_evt->params.hrm.rr_intervals_cnt != 0)
            {
                uint32_t rr_avg = 0;
                for (uint32_t i = 0; i < p_hrs_c_evt->params.hrm.rr_intervals_cnt; i++)
                {
                    rr_avg += p_hrs_c_evt->params.hrm.rr_intervals[i];
                }
                rr_avg = rr_avg / p_hrs_c_evt->params.hrm.rr_intervals_cnt;
                NRF_LOG_DEBUG("rr_interval (avg) = %d.", rr_avg);
            }
        } break;

        default:
            break;
    }
}


/**@brief Battery level Collector Handler.
 */
static void bas_c_evt_handler(ble_bas_c_t * p_bas_c, ble_bas_c_evt_t * p_bas_c_evt)
{
    ret_code_t err_code;

    switch (p_bas_c_evt->evt_type)
    {
        case BLE_BAS_C_EVT_DISCOVERY_COMPLETE:
        {
            err_code = ble_bas_c_handles_assign(p_bas_c,
                                                p_bas_c_evt->conn_handle,
                                                &p_bas_c_evt->params.bas_db);
            APP_ERROR_CHECK(err_code);

            // Battery service discovered. Enable notification of Battery Level.
            NRF_LOG_DEBUG("Battery Service discovered. Reading battery level.");

            err_code = ble_bas_c_bl_read(p_bas_c);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_DEBUG("Enabling Battery Level Notification.");
            err_code = ble_bas_c_bl_notif_enable(p_bas_c);
            APP_ERROR_CHECK(err_code);

        } break;

        case BLE_BAS_C_EVT_BATT_NOTIFICATION:
            NRF_LOG_INFO("Battery Level received %d %%.", p_bas_c_evt->params.battery_level);
            break;

        case BLE_BAS_C_EVT_BATT_READ_RESP:
            NRF_LOG_INFO("Battery Level Read as %d %%.", p_bas_c_evt->params.battery_level);
            break;

        default:
            break;
    }
}


/**
 * @brief Heart rate collector initialization.
 */
static void hrs_c_init(void)
{
    ble_hrs_c_init_t hrs_c_init_obj;

    hrs_c_init_obj.evt_handler   = hrs_c_evt_handler;
    hrs_c_init_obj.error_handler = service_error_handler;
    hrs_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;

    ret_code_t err_code = ble_hrs_c_init(&m_hrs_c, &hrs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Battery level collector initialization.
 */
static void bas_c_init(void)
{
    ble_bas_c_init_t bas_c_init_obj;

    bas_c_init_obj.evt_handler   = bas_c_evt_handler;
    bas_c_init_obj.error_handler = service_error_handler;
    bas_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;

    ret_code_t err_code = ble_bas_c_init(&m_bas_c, &bas_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(db_init));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);

    APP_ERROR_CHECK(err_code);
}


/**@brief Retrieve a list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                     *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}


static void whitelist_load()
{
    ret_code_t   ret;
    pm_peer_id_t peers[8];
    uint32_t     peer_cnt;

    memset(peers, PM_PEER_ID_INVALID, sizeof(peers));
    peer_cnt = (sizeof(peers) / sizeof(pm_peer_id_t));

    // Load all peers from flash and whitelist them.
    peer_list_get(peers, &peer_cnt);

    ret = pm_whitelist_set(peers, peer_cnt);
    APP_ERROR_CHECK(ret);

    // Setup the device identies list.
    // Some SoftDevices do not support this feature.
    ret = pm_device_identities_list_set(peers, peer_cnt);
    if (ret != NRF_ERROR_NOT_SUPPORTED)
    {
        APP_ERROR_CHECK(ret);
    }
}


static void on_whitelist_req(void)
{
    ret_code_t err_code;

    // Whitelist buffers.
    ble_gap_addr_t whitelist_addrs[8];
    ble_gap_irk_t  whitelist_irks[8];

    memset(whitelist_addrs, 0x00, sizeof(whitelist_addrs));
    memset(whitelist_irks,  0x00, sizeof(whitelist_irks));

    uint32_t addr_cnt = (sizeof(whitelist_addrs) / sizeof(ble_gap_addr_t));
    uint32_t irk_cnt  = (sizeof(whitelist_irks)  / sizeof(ble_gap_irk_t));

    // Reload the whitelist and whitelist all peers.
    whitelist_load();

    // Get the whitelist previously set using pm_whitelist_set().
    err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                whitelist_irks,  &irk_cnt);

    if (((addr_cnt == 0) && (irk_cnt == 0)) ||
        (m_whitelist_disabled))
    {
        // Don't use whitelist.
        err_code = nrf_ble_scan_params_set(&m_scan, NULL);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    if (nrf_fstorage_is_busy(NULL))
    {
        m_memory_access_in_progress = true;
        return;
    }

    NRF_LOG_INFO("Starting scan.");

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t  err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the power management module. */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    switch (p_evt->evt_id)
    {
        case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
        {
            NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                         p_evt->conn_handle,
                         p_evt->params.att_mtu_effective);
        } break;

        case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
        {
            NRF_LOG_INFO("Data length for connection 0x%x updated to %d.",
                         p_evt->conn_handle,
                         p_evt->params.data_length);
        } break;

        default:
            break;
    }
}


static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_WHITELIST_REQUEST:
        {
            on_whitelist_req();
            m_whitelist_disabled = false;
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("Scan timed out.");
            scan_start();
        } break;

        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
            break;
        case NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT:
            break;

        default:
          break;
    }
}


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initialization scanning and setting filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.p_scan_param     = &m_scan_param;
    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    ble_uuid_t uuid =
    {
        .uuid = TARGET_UUID,
        .type = BLE_UUID_TYPE_BLE,
    };

    err_code = nrf_ble_scan_filter_set(&m_scan,
                                       SCAN_UUID_FILTER,
                                       &uuid);
    APP_ERROR_CHECK(err_code);

    if (strlen(m_target_periph_name) != 0)
    {
        err_code = nrf_ble_scan_filter_set(&m_scan,
                                           SCAN_NAME_FILTER,
                                           m_target_periph_name);
        APP_ERROR_CHECK(err_code);
    }

    if (is_connect_per_addr)
    {
       err_code = nrf_ble_scan_filter_set(&m_scan,
                                          SCAN_ADDR_FILTER,
                                          m_target_periph_addr.addr);
       APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_ble_scan_filters_enable(&m_scan,
                                           NRF_BLE_SCAN_ALL_FILTER,
                                           false);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;
    
    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
}


/**@brief Function for starting a scan, or instead trigger it from peer manager (after
 *        deleting bonds).
 *
 * @param[in] p_erase_bonds Pointer to a bool to determine if bonds will be deleted before scanning.
 */
void scanning_start(bool * p_erase_bonds)
{
    // Start scanning for peripherals and initiate connection
    // with devices that advertise GATT Service UUID.
    if (*p_erase_bonds == true)
    {
        // Scan is started by the PM_EVT_PEERS_DELETE_SUCCEEDED event.
        delete_bonds();
    }
    else
    {
        scan_start();
    }
}



/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_INFO
        NRF_LOG_INFO("Program Start");
    #endif
    // power_management_init();
    log_init();
    leds_init();
    timers_init();
    button_init();
    buttons_leds_init(&erase_bonds);

    ble_stack_init();
    ble_gap_params_init();
    ble_gatt_init();
    ble_services_init();
    ble_advertising_init();
    ble_connection_params_init();
    peer_manager_init();
    db_discovery_init();
    scan_init();

    // Start execution.
    ble_advertising_start();
    scanning_start(&erase_bonds);


    
    // Enter main loop.
    for (;;)
    {

            NRF_LOG_PROCESS();
        // idle_state_handle();
        // nrf_delay_us(3000);        
    }
}