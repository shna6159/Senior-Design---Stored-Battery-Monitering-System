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
#include "math.h"

#include "ble_nus.h"
#include "app_uart.h"

#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"

#include <stdbool.h>
#include <stdio.h>
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "app_util_platform.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//                                      DEFINES
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
#define TEMP_SENSOR_1 NRF_GPIO_PIN_MAP(1, 15)
#define TEMP_SENSOR_2 NRF_GPIO_PIN_MAP(1, 13)
#define output_pin NRF_GPIO_PIN_MAP(0, 12)
#define output_pin1 NRF_GPIO_PIN_MAP(0, 13)
#define output_pin2 NRF_GPIO_PIN_MAP(0, 15)
#define output_pin3 NRF_GPIO_PIN_MAP(0, 17)
#define output_pin4 NRF_GPIO_PIN_MAP(0, 20)
#define output_pin5 NRF_GPIO_PIN_MAP(0, 22)
#define output_pin6 NRF_GPIO_PIN_MAP(0, 24)
#define output_pin7 NRF_GPIO_PIN_MAP(1, 00)

#define TX_POWER_LEVEL 8

#define ADVERTISING_LED BSP_BOARD_LED_0 /**< Is on when device is advertising. */
#define CONNECTED_LED BSP_BOARD_LED_1   /**< Is on when device has connected. */
#define LEDBUTTON_LED BSP_BOARD_LED_2   /**< LED to be toggled with the help of the LED Button Service. */
#define UNEXPECTED_LED BSP_BOARD_LED_3  /**< LED to be toggled when the an error occurs */
// #define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME "SBMS_in_box" /**< Name of device. Will be included in the advertising data. */
#define APP_BLE_OBSERVER_PRIO 3   /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG 1    /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL 64                                    /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION 1000 /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS) /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(20000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                        /**< Number of attempts before giving up the connection parameter negotiation. */

// #define TX_POWER_LEVEL -8
// #define TX_POWER_LEVEL 0
// #define TX_POWER_LEVEL 4
#define TX_POWER_LEVEL 8

// #define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

// #define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UUID_BASE                                                                                      \
    {                                                                                                  \
        0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00 \
    }
#define UUID_SERVICE 0x1234
#define UUID_VOLTAGE_1_CHAR 0x5514
#define UUID_VOLTAGE_2_CHAR 0x4514
#define UUID_TEMPERATURE_1_CHAR 0x3456
#define UUID_TEMPERATURE_2_CHAR 0x5678
#define UUID_RTC_CONFIG_CHAR 0x9123
// RTC_VAL_IN_SEC is actually given in seconds, so you can simply change 3 into whatever amount of seconds you want to use.
#define RTC_VAL_IN_SEC  (20UL)   //keep at 20 sec or higher  /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */
#define NUM_TEMPERATURE_PERIODS 1000

#define SAADC_CHANNEL1 0
#define SAADC_CHANNEL2 1

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//                                      LITERAL CONSTS
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
const nrf_drv_rtc_t rtc = NRFX_RTC_INSTANCE(2); // Create a handle that will point to the RTC 2 of nrf device

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);   
NRF_BLE_GATT_DEF(m_gatt); /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);   /**< Context for the Queued Write module.*/
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */


static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                /**< Handle of the current connection. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;           /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];            /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffer for storing an encoded scan data. */

ble_gatts_char_handles_t voltage_1_char_handles;     /** Voltage Sensor 1 Characteristic */
ble_gatts_char_handles_t voltage_2_char_handles;     /** Voltage Sensor 2 Characteristic */
ble_gatts_char_handles_t temperature_1_char_handles; /** Temperature Sensor 1 Characteristic */
ble_gatts_char_handles_t temperature_2_char_handles; /** Temperature Sensor 2 Characteristic */
ble_gatts_char_handles_t rtc_config_char_handles;    /** RTC Configuration Characteristic */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
    {
        .adv_data =
            {
                .p_data = m_enc_advdata,
                .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX},
        .scan_rsp_data =
            {
                .p_data = m_enc_scan_response_data,
                .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX

            }};

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//                                      GLOBAL VARIABLES
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
uint16_t valid_temp_counter = 0;
double valid_duty_cycle[NUM_TEMPERATURE_PERIODS];
uint16_t frequency = 0;
double duty_cycle = 0;
double temperature;
uint16_t temperature_encoded;
bool temp_sensor = false;
int expo;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//                                      PROCEDURES - BLE
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void ble_gap_params_init(void)
{
    // ret_code_t              err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    sd_ble_gap_device_name_set(&sec_mode,
                               (const uint8_t *)DEVICE_NAME,
                               strlen(DEVICE_NAME));

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    sd_ble_gap_ppcp_set(&gap_conn_params);

    NRF_LOG_DEBUG("GAP init");
}

// /**@brief Function for initializing the GATT module.
//  */
static void ble_gatt_init(void)
{

    NRF_LOG_DEBUG("GATT init");

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

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid_type = uuid_type;
    add_char_params.init_len = sizeof(uint16_t);
    add_char_params.max_len = sizeof(uint16_t);
    add_char_params.char_props.read = 1;
    add_char_params.char_props.notify = 1;
    add_char_params.read_access = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    add_char_params.uuid = UUID_VOLTAGE_1_CHAR;
    characteristic_add(service_handle, &add_char_params, &voltage_1_char_handles); // Setup voltage characteristic

    add_char_params.uuid = UUID_VOLTAGE_2_CHAR;
    characteristic_add(service_handle, &add_char_params, &voltage_2_char_handles); // Setup voltage characteristic

    add_char_params.uuid = UUID_TEMPERATURE_1_CHAR;
    characteristic_add(service_handle, &add_char_params, &temperature_1_char_handles); // Setup_temperature characteristic

    add_char_params.uuid = UUID_TEMPERATURE_2_CHAR;
    characteristic_add(service_handle, &add_char_params, &temperature_2_char_handles); // Setup_temperature characteristic

     add_char_params.uuid = UUID_RTC_CONFIG_CHAR;
    characteristic_add(service_handle, &add_char_params, &rtc_config_char_handles); // setup_rtc_config characteristic

    // ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

    // Build and set advertising data.
    ble_advdata_t advdata;
    ble_advdata_t srdata;
    ble_uuid_t adv_uuids[] = {{UUID_SERVICE, uuid_type}};
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids = adv_uuids;

    ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);

    ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
    adv_params.duration = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr = NULL;
    adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
    adv_params.interval = APP_ADV_INTERVAL;


    sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);

    NRF_LOG_DEBUG("Advertising init");
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


static void ble_send(const uint8_t * str, size_t len){
    nrf_delay_ms(1000);
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    memcpy(data_array, str, len);
    //const uint8_t test_string[] = "yest";
    // static uint8_t index = 0;
    uint32_t       err_code;

    for(int index = 0; index <= BLE_NUS_MAX_DATA_LEN; index++){

        if ((data_array[index - 1] == '\n') || (data_array[index - 1] == '\r') || (index >= m_ble_nus_max_data_len)){

            if (index > 1){
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                do{
                    uint16_t length = (uint16_t)index;
                    err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                    if ((err_code != NRF_ERROR_INVALID_STATE) &&
                        (err_code != NRF_ERROR_RESOURCES) &&
                        (err_code != NRF_ERROR_NOT_FOUND)){
                        APP_ERROR_CHECK(err_code);
                    }
                } while (err_code == NRF_ERROR_RESOURCES);
            }

            break;
        }
    }
}

static uint32_t tapp_uart_put(uint8_t byte){
    return NRF_SUCCESS;
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */

int interval = 0;

static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        // bsp_board_led_invert(UNEXPECTED_LED);
        // nrf_delay_ms(500);
        // bsp_board_led_invert(UNEXPECTED_LED);
        uint32_t err_code;

        //printf("Received data from BLE NUS. Writing data on UART.\r\n");
        //printf("%s\r\n", p_evt->params.rx_data.p_data);
        // Using the amounts of bytes stored in buffer to determine interval
        int rate = p_evt->params.rx_data.length;
    
            if (rate == 1)
            {
                interval = 0; //1 min in sec
            }
            if (rate == 2)
            {
                interval = 1; //5 min in sec
            }
            if (rate == 3)
            {
                interval = 2; //1 hr in sec
            }
            if (rate == 4)
            {
                interval = 3; //6 hr in sec
            }
            if (rate == 5)
            {
                interval = 4; //12 hr in sec
            }
            if (rate == 6)
            {
                interval = 5; //1 day in sec
            }
            if (rate == 7)
            {
                interval = 6; //1 week in sec
            }
            if (rate == 8)
            {
                interval = 7; //4 week in sec
            }
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        


        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = tapp_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (tapp_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */

static void ble_services_init(void)
{
    // ret_code_t         err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    nrf_ble_qwr_init(&m_qwr, &qwr_init);
    // APP_ERROR_CHECK(err_code);

    //Initialize nus
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    ble_nus_init(&m_nus, &nus_init);

    NRF_LOG_DEBUG("BLE Services init");
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
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
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

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    ble_conn_params_init(&cp_init);
    // APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("BLE connection paramters init");
}

/**@brief Function for starting advertising.
 */
static void ble_advertising_start(void)
{
    // ble_advertising_init();
    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, TX_POWER_LEVEL);
    sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    bsp_board_led_on(ADVERTISING_LED);
    nrf_delay_ms(1000);
    bsp_board_led_off(ADVERTISING_LED);
    // bsp_board_led_off(LEDBUTTON_LED);

    NRF_LOG_INFO("Advertising Start \n\n");
}

// static void ble_advertising_stop(void)
// {
//     ret_code_t err_code = sd_ble_gap_adv_stop(m_adv_handle);
//     APP_ERROR_CHECK(err_code);
//     if(err_code == NRF_SUCCESS)
//     {
//         bsp_board_led_off(ADVERTISING_LED);
//         bsp_board_led_on(LEDBUTTON_LED);
//     }
    

//     NRF_LOG_INFO("Advertising Stop \n\n");
// }

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:

        NRF_LOG_INFO("Connected");

        // bsp_board_led_on(CONNECTED_LED);
        // bsp_board_led_off(ADVERTISING_LED);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, TX_POWER_LEVEL);
        nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);

        break;

    case BLE_GAP_EVT_DISCONNECTED:

        NRF_LOG_INFO("Disconnected");

        // bsp_board_led_off(CONNECTED_LED);
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        // err_code = app_button_disable();
        // APP_ERROR_CHECK(err_code);
        // ble_advertising_start();
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
        //     NRF_LOG_INFO("PHY update request.");
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
        //     NRF_LOG_INFO("GATT Client Timeout.");
        //     err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        //                                      BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        //     APP_ERROR_CHECK(err_code);
        //     break;

        // case BLE_GATTS_EVT_TIMEOUT:
        //     // Disconnect on GATT Server timeout event.
        //     NRF_LOG_INFO("GATT Server Timeout.");
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
    nrf_sdh_enable_request();
    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);

    // Enable BLE stack.
    nrf_sdh_ble_enable(&ram_start);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    NRF_LOG_INFO("BLE Stack INIT");
}

/**@brief Function which writes a value to the characteristic mentioned
 *
 * @param  characteristic_value [in] Value being written to the characteristic
 * @param char_handle [in] handle of the characteristic
 */
void ble_write_to_characteristic(uint8_t int_val, uint8_t dec_val, ble_gatts_char_handles_t char_handle)
{
    ble_gatts_hvx_params_t params;
    uint8_t data[2] = { int_val, dec_val};
        NRF_LOG_DEBUG("Writing the following values to the characteristic %d and %d \n", data[0], data[1]);

    uint16_t len = sizeof(data);
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = char_handle.value_handle;

    params.p_data = &data[1];
    params.p_data = &dec_val;
    params.p_len = &len;
    sd_ble_gatts_hvx(m_conn_handle, &params);
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//                                      PROCEDURES - MISC
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
// /**
//  * @brief Function for configuration of REGOUT0 register.
//  */
// void vddInit(void)
// {
//   if (NRF_UICR->REGOUT0 != UICR_REGOUT0_VOUT_3V3) 
//   {
// 	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;    //write enable
// 	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
// 	NRF_UICR->REGOUT0 = UICR_REGOUT0_VOUT_3V3;                        //configurate REGOUT0
// 	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
// 	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
//         NVIC_SystemReset();                                               // Reset device
//   } 
// }

static int decimal_part(double num){
  int intpart = (int)num;
  double decpart = num - intpart;
  int decimal = decpart*100;
  NRF_LOG_DEBUG("decimal part is %d", decimal);
  return decimal;
}

static int exponent_part(double num){
    int intpart = (int)num;
      NRF_LOG_DEBUG("int part is %d", intpart);
    return intpart;
}


// Init output pins
void output_pin_init()
{
     nrf_gpio_cfg_output(output_pin1);
    //  nrf_gpio_cfg_output(output_pin2);
     nrf_gpio_cfg_output(output_pin3);
     nrf_gpio_cfg_output(output_pin4);
     nrf_gpio_cfg_output(output_pin5);
    //  nrf_gpio_cfg_output(output_pin6);
    //  nrf_gpio_cfg_output(output_pin7);
}

void output_pin_enable()
{
     nrf_gpio_pin_set(output_pin1); 
    //  nrf_gpio_pin_set(output_pin2); 
     nrf_gpio_pin_set(output_pin3); 
     nrf_gpio_pin_set(output_pin4);
     nrf_gpio_pin_set(output_pin5);
    //  nrf_gpio_pin_set(output_pin6);
    //  nrf_gpio_pin_set(output_pin7);
}

void output_pin_disable()
{
    nrf_gpio_pin_clear(output_pin1); 
    //nrf_gpio_pin_clear(output_pin2); 
    nrf_gpio_pin_clear(output_pin3); 
    nrf_gpio_pin_clear(output_pin4);
    nrf_gpio_pin_clear(output_pin5);
    //nrf_gpio_pin_set(output_pin6);
    // nrf_gpio_pin_clear(output_pin7);
}

/*
Descripttion : Sets up all the the LEDs used by the program
*/
static void leds_init(void)
{
    NRF_LOG_DEBUG("LED init");

    bsp_board_init(BSP_INIT_LEDS);
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//                                      PROCEDURES - SAADC
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
}

/**
 * @brief Function for confguring SAADC channel 0 for sampling AIN0 (P0.02).
 */
void saadc_init()
{

    ret_code_t err_code;
	
    nrf_drv_saadc_config_t saadc_config;
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_HIGHEST;
    saadc_config.low_power_mode = false;
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;
	
    nrf_saadc_channel_config_t channel_1_config;
    //     NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    channel_1_config.pin_p      = NRF_SAADC_INPUT_AIN5; // NRF_GPIO_PIN_MAP(0, 29)
    channel_1_config.pin_n      = NRF_SAADC_INPUT_AIN0;
    channel_1_config.mode       = NRF_SAADC_MODE_DIFFERENTIAL;
    channel_1_config.acq_time   = NRF_SAADC_ACQTIME_40US;
    channel_1_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    // channel_1_config.gain       = NRF_SAADC_GAIN1_6;
    channel_1_config.gain       = NRF_SAADC_GAIN1_4;
    channel_1_config.resistor_p = NRF_SAADC_RESISTOR_PULLUP;
    channel_1_config.resistor_n = NRF_SAADC_RESISTOR_PULLDOWN;
    channel_1_config.burst      = NRF_SAADC_BURST_ENABLED;

    nrf_saadc_channel_config_t channel_2_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);
    channel_2_config.acq_time   = NRF_SAADC_ACQTIME_40US;
    channel_2_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_2_config.gain       = NRF_SAADC_GAIN1_4;
    channel_2_config.resistor_p = NRF_SAADC_RESISTOR_PULLUP;
    channel_2_config.resistor_n = NRF_SAADC_RESISTOR_PULLDOWN;
    channel_2_config.burst      = NRF_SAADC_BURST_ENABLED;

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_saadc_channel_init(SAADC_CHANNEL1, &channel_1_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(SAADC_CHANNEL2, &channel_2_config);
    APP_ERROR_CHECK(err_code);
    nrf_drv_saadc_calibrate_offset();
}

static void saadc_disable()
{
    ret_code_t err_code;

	err_code = nrf_drv_saadc_channel_uninit(SAADC_CHANNEL1);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_uninit(SAADC_CHANNEL2);
    APP_ERROR_CHECK(err_code);
	nrf_drv_saadc_uninit();
}


void saadc_sample_write_ble()
{   int buffy = 10;
    char hot_boi[buffy];
    
    saadc_init();

    nrf_delay_ms(1000);

    ret_code_t err_code;
    nrf_saadc_value_t sample;
    double totalSamples;
    for(int i = 0; i < 10; i++)
    {
        err_code = nrfx_saadc_sample_convert(SAADC_CHANNEL1, &sample);
        double V1 = (double)((sample * 3.334) / (pow(2,11)));
        V1 *= (998 + 104.7)/(104.7);
        totalSamples += V1;
        nrf_delay_ms(50);
    }
    double V1 = totalSamples/10;
    sprintf(hot_boi, "b%i.%i", exponent_part(V1),decimal_part(V1));
    const uint8_t *kyle = (uint8_t*) hot_boi;
    ble_send(kyle,buffy);

    
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("1st Voltage[V]: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(V1));
    ble_write_to_characteristic(exponent_part(V1), decimal_part(V1), voltage_1_char_handles);

    nrf_delay_ms(1000);

    err_code = nrfx_saadc_sample_convert(SAADC_CHANNEL2, &sample);
    APP_ERROR_CHECK(err_code);
    
    // double V2 = (double)((sample * 4 * NRF_SAADC_REFERENCE_VDD4) / (pow(2,12)));
    // double V2 = (double)((sample * 3.002) / (pow(2,12)));
    double V2 = (double)((sample * 3.334) / (pow(2,12)));
    V2 = V2 * 1.3;
    sprintf(hot_boi, "c%i.%i", exponent_part(V2),decimal_part(V2));
    kyle = (uint8_t*) hot_boi;
    ble_send(kyle,buffy);
    // V2 = 30;
    NRF_LOG_INFO( "2nd Voltage[V]: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(V2));
    ble_write_to_characteristic(exponent_part(V2), decimal_part(V2), voltage_2_char_handles);

    nrf_delay_ms(1000);

    saadc_disable();
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//                                      PROCEDURES - TEMP SENSOR
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
static void timer_init()
{
    nrf_gpio_cfg_output(output_pin);
    // nrf_gpio_pin_set(output_pin);

    NVIC_EnableIRQ(TIMER3_IRQn);
    NVIC_SetPriority(TIMER3_IRQn, APP_IRQ_PRIORITY_LOW);
    nrf_gpio_cfg_input(TEMP_SENSOR_1, NRF_GPIO_PIN_NOPULL);

    NRF_TIMER3->TASKS_STOP = 1;
    NRF_TIMER3->TASKS_CLEAR = 1;
    NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER3->PRESCALER = 0; // uses 16 MHz clk
    NRF_TIMER3->CC[0] = 40000; // approx - 10^-2 / 4 s

    NRF_TIMER3->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);

    NRF_TIMER3->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);

    NRF_TIMER3->EVENTS_COMPARE[0] = 0;

    NRF_LOG_DEBUG("Timer1 setup");
}
static void counter_init()
{
    NRF_TIMER2->TASKS_STOP = 1;
    NRF_TIMER2->TASKS_CLEAR = 1;
    NRF_TIMER2->MODE = TIMER_MODE_MODE_Counter;
    NRF_TIMER2->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);
    NRF_TIMER2->TASKS_CLEAR = 1;

    NRF_LOG_DEBUG("Timer2 setup");
}
static void setup_gpiote_event(uint32_t pin)
{
    NRF_GPIOTE->CONFIG[0] = 0x01 << 0;                         // MODE: Event
    NRF_GPIOTE->CONFIG[0] |= pin << 8;                         // Pin number
    NRF_GPIOTE->CONFIG[0] |= NRF_GPIOTE_POLARITY_LOTOHI << 16; // Event rising edge

    NRF_LOG_DEBUG("gpiote_init rising edge");

    NRF_GPIOTE->CONFIG[1] = 0x01 << 0;                         // MODE: Event
    NRF_GPIOTE->CONFIG[1] |= pin << 8;                         // Pin number
    NRF_GPIOTE->CONFIG[1] |= NRF_GPIOTE_POLARITY_HITOLO << 16; // Event rising edge

    NRF_LOG_DEBUG("gpiote_init_falling edge");
}

static void setup_timer_and_counter_ppi()
{
    NRF_PPI->CHEN |= 1 << 0;
    *(&(NRF_PPI->CH0_EEP)) = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[0];
    *(&(NRF_PPI->CH0_TEP)) = (uint32_t)&NRF_TIMER2->TASKS_CAPTURE[0];
    *(&(NRF_PPI->FORK[0].TEP)) = (uint32_t)&NRF_TIMER3->TASKS_STOP;
    NRF_PPI->CHENSET |= 1 << 0;

    NRF_LOG_DEBUG("ppi_timer_stop_counter_init");

    NRF_PPI->CHEN |= 1 << 1;
    *(&(NRF_PPI->CH1_EEP)) = (uint32_t)&NRF_GPIOTE->EVENTS_IN[0];
    *(&(NRF_PPI->CH1_TEP)) = (uint32_t)&NRF_TIMER2->TASKS_COUNT;
    *(&(NRF_PPI->FORK[1].TEP)) = (uint32_t)&NRF_TIMER3->TASKS_CAPTURE[2];
    NRF_PPI->CHENSET |= 1 << 1;

    NRF_LOG_DEBUG("ppi_gpiote_counter_init_rising");

    NRF_PPI->CHEN |= 1 << 2;
    *(&(NRF_PPI->CH2_EEP)) = (uint32_t)&NRF_GPIOTE->EVENTS_IN[1];
    *(&(NRF_PPI->FORK[2].TEP)) = (uint32_t)&NRF_TIMER3->TASKS_CAPTURE[3];
    NRF_PPI->CHENSET |= 1 << 2;

    NRF_LOG_DEBUG("ppi_gpiote_counter_init_falling");

    NRF_PPI->CHEN |= 1 << 3;
    *(&(NRF_PPI->CH3_EEP)) = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[0];
    *(&(NRF_PPI->CH3_TEP)) = (uint32_t)&NRF_TIMER2->TASKS_STOP;
    NRF_PPI->CHENSET |= 1 << 3;

    NRF_LOG_DEBUG("Setting up the ppi for timer pin high"); // TODO: remove this

}


void TIMER3_IRQHandler(void)
{
    if (NRF_TIMER3->EVENTS_COMPARE[0] == 1)
    {
        NRF_TIMER3->EVENTS_COMPARE[0] = 0;

        int pulse_width = NRF_TIMER3->CC[3] - NRF_TIMER3->CC[2];


        if ((pulse_width) < 0)
        {
            NRF_TIMER3->TASKS_CLEAR = 1;
            NRF_TIMER2->TASKS_CLEAR = 1;

            NRF_TIMER2->CC[0] = 0;
            NRF_TIMER3->CC[2] = 0;
            NRF_TIMER3->CC[3] = 0;

            NRF_TIMER3->TASKS_START = 1;
            NRF_TIMER2->TASKS_START = 1;
        }
        else
        {

            frequency = NRF_TIMER2->CC[0] * 4 * 100;
            duty_cycle = (double)(frequency) * (pulse_width) / 16000000;
            NRF_TIMER3->TASKS_CLEAR = 1;
            NRF_TIMER2->TASKS_CLEAR = 1;
            if (valid_temp_counter == NUM_TEMPERATURE_PERIODS)
            {

                double average_duty_cycle = 0;
                for (uint16_t i = 0; i < NUM_TEMPERATURE_PERIODS; i++)
                {
                    average_duty_cycle = average_duty_cycle + valid_duty_cycle[i];
                }
                average_duty_cycle = average_duty_cycle / NUM_TEMPERATURE_PERIODS;
                // NRF_LOG_INFO("Averaged Duty Cycle " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(average_duty_cycle));
                temperature = -1.43 * average_duty_cycle * average_duty_cycle + 214.56 * average_duty_cycle - 68.60;
                valid_temp_counter = 0;


                expo = exponent_part(temperature);
                temperature_encoded = decimal_part(temperature);
                NRF_LOG_INFO("Temperature [Deg C] " NRF_LOG_FLOAT_MARKER "\r", NRF_LOG_FLOAT(temperature));
                int buffy = 10;
                char hot_boi[buffy];


                if(temp_sensor == false){
                    ble_write_to_characteristic(expo, temperature_encoded, temperature_1_char_handles);
                    temp_sensor = true;
                    setup_gpiote_event(TEMP_SENSOR_2);
                    //itoa(expo,hot_boi,10);
                    sprintf(hot_boi, "a%i.%i", expo,temperature_encoded);
                    const uint8_t *kyle = (uint8_t*) hot_boi;
                    ble_send(kyle,buffy);

                    NRF_TIMER2->CC[0] = 0;
                    NRF_TIMER3->CC[2] = 0;
                    NRF_TIMER3->CC[3] = 0;
                    NRF_TIMER3->TASKS_START = 1;
                    NRF_TIMER2->TASKS_START = 1;
                }
                else{
                    ble_write_to_characteristic(expo, temperature_encoded, temperature_2_char_handles);
                    temp_sensor = false;
                    saadc_sample_write_ble();
                    sprintf(hot_boi, "d%i.%i", expo,temperature_encoded);
                    const uint8_t *kyle = (uint8_t*) hot_boi;
                    ble_send(kyle,buffy);
                    
                    NRF_TIMER2->CC[0] = 0;
                    NRF_TIMER3->CC[2] = 0;
                    NRF_TIMER3->CC[3] = 0;
                    
            output_pin_disable();
            // ble_advertising_start();
            // nrf_delay_ms(7000);
            // ble_advertising_stop();

                }

                // nrf_delay_ms(10000);
                // NRF_TIMER3->TASKS_START = 1;
                // NRF_TIMER2->TASKS_START = 1;
            }
            else
            {
                valid_duty_cycle[valid_temp_counter] = duty_cycle;
                valid_temp_counter += 1;
                
                NRF_TIMER2->CC[0] = 0;
                NRF_TIMER3->CC[2] = 0;
                NRF_TIMER3->CC[3] = 0;
                NRF_TIMER3->TASKS_START = 1;
                NRF_TIMER2->TASKS_START = 1;
            }
        }
    }
}

static void temp_sensor_measure(void){
    NRF_TIMER3->TASKS_START = 1;
    NRF_TIMER2->TASKS_START = 1;
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//                                      PROCEDURES - REAL TIME COUNTER
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
// static void lfclk_config(void)
// {
//     // Initialize the low frequency clock
//     ret_code_t err_code = nrf_drv_clock_init();
//     APP_ERROR_CHECK(err_code); // check for the errors

//     // Request the clock to not to generate events
//     nrf_drv_clock_lfclk_request(NULL);
//     NRF_LOG_DEBUG("Low frequency clock setup");
// }
bool is_advertising = false;
// RTC interrupt handler which will be used to handle the interrupt events
static void rtc_handler(nrfx_rtc_int_type_t int_type)
{

    // Check if the interrupt occurred due to tick event
    if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        // perform some action
        // bsp_board_led_invert(ADVERTISING_LED);
    }
    else if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        output_pin_enable();
        ble_advertising_start();
        NRF_LOG_DEBUG("RTC compare event");
        // bsp_board_led_invert(UNEXPECTED_LED);
        // nrf_delay_ms(1000);
        // bsp_board_led_invert(UNEXPECTED_LED);
        // nrf_delay_ms(1000);
        // bsp_board_led_invert(UNEXPECTED_LED);
        // nrf_delay_ms(1000);
        // bsp_board_led_invert(UNEXPECTED_LED);
        // nrf_delay_ms(1000);
        // bsp_board_led_invert(UNEXPECTED_LED);
        // nrf_delay_ms(1000);
        nrf_rtc_task_trigger(rtc.p_reg, NRF_RTC_TASK_CLEAR);

        // temp sensor code
        timer_init();
        counter_init();
        setup_gpiote_event(TEMP_SENSOR_1);
        setup_timer_and_counter_ppi();
        temp_sensor_measure();        
        // nrf_drv_rtc_cc_set(&rtc,0,RTC_VAL_IN_SEC * 8,true);
        //nrf_delay_ms(10000);
        



        // unsigned long RTC_CONFIG_CHARVAL = RTC_VAL_IN_SEC; // Replace with characteristic reading (may not even need to do anything)
        //unsigned long RTC_CONFIG_CHARVAL = RTC_VAL_IN_SEC;
        // Handle different cases. Mapping is as follows:
        // 0->1min, 1->5min, 2->1hr, 3->8hr, 4->24hr
        switch (interval)
        {
        case RTC_VAL_IN_SEC: // Default

            nrf_drv_rtc_cc_set(&rtc,0,RTC_VAL_IN_SEC * 8,true);
            break;

        case 0:

            nrf_drv_rtc_cc_set(&rtc,0,1*60 * 8,true);
            break;

        case 1:

            nrf_drv_rtc_cc_set(&rtc,0,5*60 * 8,true);
            break;

        case 2:

            nrf_drv_rtc_cc_set(&rtc,0,1*60*60 * 8,true);
            break;

        case 3:

            nrf_drv_rtc_cc_set(&rtc,0,6*60*60 * 8,true);
            break;

        case 4:

            nrf_drv_rtc_cc_set(&rtc,0,12*60*60 * 8,true);
            break;
        
        case 5:

            nrf_drv_rtc_cc_set(&rtc,0,24*60*60 * 8,true);
            break; 

        case 6:

            nrf_drv_rtc_cc_set(&rtc,0,7*24*60*60 * 8,true);
            break;

        case 7:

            nrf_drv_rtc_cc_set(&rtc,0,4*7*24*60*60 * 8,true);
            break;
        }
    }
    else
    {
        // default action
        // leave it empty
    }
}

// A function to configure and intialize the RTC
static void rtc_config(void)
{

    uint32_t err_code; // a variable to hold the error values

    // Create a struct of type nrfx_rtc_config_t and assign it default values
    nrf_drv_rtc_config_t rtc_config = NRFX_RTC_DEFAULT_CONFIG;

    // Configure the prescaler to generate ticks for a specific time unit
    // Configured it to tick every 125ms
    rtc_config.prescaler = 4095; // tick =  32768 / (4095 + 1) = 8Hz = 125ms

    // Initialize the rtc and pass the configurations along with the interrupt handler
    err_code = nrf_drv_rtc_init(&rtc, &rtc_config, rtc_handler);
    APP_ERROR_CHECK(err_code); // check for errors

    // nrf_drv_rtc_tick_enable(&rtc, true);

    err_code = nrf_drv_rtc_cc_set(&rtc,0,RTC_VAL_IN_SEC * 8,true);
    APP_ERROR_CHECK(err_code);

    // start the rtc
    NRF_LOG_DEBUG("RTC config");
}

static void rtc_start(void){
    nrf_drv_rtc_enable(&rtc);
    NRF_LOG_DEBUG("RTC start");
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    
    log_init();
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("Program Start!!!!");
    NRF_LOG_FLUSH();
    leds_init();
    output_pin_init();
    ble_stack_init();
    ble_gap_params_init();
    ble_gatt_init();
    ble_services_init();
    ble_advertising_init();
    ble_connection_params_init();
    
    // // call the clock configuration
    // lfclk_config();

    // call the rtc configuration
    rtc_config();
    rtc_start();
    
    ble_advertising_start();

    // Sleep in the while loop until an event is generated
    while (true)
    {
        
        NRF_LOG_FLUSH();
        // __SEV();
        // __WFE();
        // __WFE();
        nrf_pwr_mgmt_run();
    }
}