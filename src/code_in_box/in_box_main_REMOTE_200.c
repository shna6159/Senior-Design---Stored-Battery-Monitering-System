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


#define TX_POWER_LEVEL 8

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define UNEXPECTED_LED                  BSP_BOARD_LED_3                         /**< LED to be toggled when the an error occurs */
// #define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "SBMS_in_box"                                  /**< Name of device. Will be included in the advertising data. */
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
#define UUID_VOLTAGE_CHAR 0x1234
#define UUID_TEMPERATURE_1_CHAR 0x3456
#define UUID_TEMPERATURE_2_CHAR 0x5678



NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

ble_gatts_char_handles_t voltage_char_handles;                                   /** Voltage Sensor Characteristic */
ble_gatts_char_handles_t temperature_1_char_handles;                             /** Temperature Sensor 1 Characteristic */
ble_gatts_char_handles_t temperature_2_char_handles;                             /** Temperature Sensor 2 Characteristic */

#define output_pin NRF_GPIO_PIN_MAP(0,22)
#define SAADC_CHANNEL 0

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

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void leds_init(void)
{
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_DEBUG
    NRF_LOG_DEBUG("LED init");
    #endif
    bsp_board_init(BSP_INIT_LEDS);

}

static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    #if defined(LOG_LEVEL) && LOG_LEVEL == LOG_LEVEL_DEBUG
    NRF_LOG_DEBUG("TIMER init");
    #endif
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

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

    add_char_params.uuid = UUID_VOLTAGE_CHAR;
    characteristic_add(service_handle, &add_char_params, &voltage_char_handles);         //Setup voltage characteristic

    add_char_params.uuid = UUID_TEMPERATURE_1_CHAR;
    characteristic_add(service_handle, &add_char_params, &temperature_1_char_handles);   //Setup_temperature characteristic

    add_char_params.uuid = UUID_TEMPERATURE_2_CHAR;
    characteristic_add(service_handle, &add_char_params, &temperature_2_char_handles);   //Setup_temperature characteristic



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

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
     sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    
    }
}

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
    // bsp_board_led_on(ADVERTISING_LED);
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
        ble_write_to_characteristic(action, voltage_char_handles);
        
    }
}

/**
 * @brief Not used in this example, but driver API requiers a callback function to be proivded.
 */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
}


/**
 * @brief Function for confguring SAADC channel 0 for sampling AIN0 (P0.02).
 */
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(SAADC_CHANNEL, &channel_config);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    log_init();
    leds_init();
    timers_init();
    ble_stack_init();
    ble_gap_params_init();
    ble_gatt_init();
    ble_services_init();
    ble_advertising_init();
    ble_connection_params_init();

    ret_code_t err_code;
    nrf_saadc_value_t sample;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    saadc_init();
    ble_advertising_start();

    while (1)
    {
        err_code = nrfx_saadc_sample_convert(SAADC_CHANNEL, &sample);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Sample: %i", sample);
        ble_write_to_characteristic(sample, voltage_char_handles);
        NRF_LOG_FLUSH();
        bsp_board_led_on(LEDBUTTON_LED);
        nrf_delay_ms(5000);
        bsp_board_led_off(LEDBUTTON_LED);
        nrf_delay_ms(5000);
    }
}


/** @} */