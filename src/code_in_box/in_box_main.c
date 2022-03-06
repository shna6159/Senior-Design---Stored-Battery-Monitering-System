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


#define TEMP_SENSOR_1 NRF_GPIO_PIN_MAP(0, 11)
#define TEMP_SENSOR_2 NRF_GPIO_PIN_MAP(0, 12)

#define output_pin NRF_GPIO_PIN_MAP(0, 13)
#define NUM_TEMPERATURE_PERIODS 1000


static nrf_ppi_channel_t m_ppi_channel1;
static nrf_ppi_channel_t m_ppi_channel2;
static nrf_ppi_channel_t m_ppi_channel3;
static nrf_ppi_channel_t m_ppi_channel4;

static const nrf_drv_timer_t m_timer1 = NRF_DRV_TIMER_INSTANCE(1);
static const nrf_drv_timer_t m_timer2 = NRF_DRV_TIMER_INSTANCE(2);

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
//                                      PROCEDURES - TEMP SENSOR
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
static void timer1_event_handler(nrf_timer_event_t event_type, void * p_context)
{
int pulse_width = NRF_TIMER1->CC[3] - NRF_TIMER1->CC[2];
NRF_LOG_INFO("pulse_width %d", pulse_width);
NRF_LOG_INFO("pulse_width %d", NRF_TIMER1->CC[3]);
NRF_LOG_INFO("pulse_width %d", NRF_TIMER1->CC[2]);
NRF_LOG_INFO("pulse_width %d", NRF_TIMER1->CC[1]);
NRF_LOG_INFO("pulse_width %d", NRF_TIMER1->CC[0]);
NRF_LOG_INFO("pulse_width %d", NRF_TIMER2->CC[0]);

}
/* Timer event handler. Not used since Timer1 and Timer2 are used only for PPI. */
static void empty_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
}


static void timer_init()
{
   
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_16MHz;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;

    ret_code_t err_code = nrf_drv_timer_init(&m_timer1, &timer_cfg, timer1_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_extended_compare(&m_timer1,
                                   NRF_TIMER_CC_CHANNEL0,
                                   40000,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK,
                                   true);

    NRF_LOG_DEBUG("Timer1 timer mode setup");
}
static void counter_init()
{
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.mode = TIMER_MODE_MODE_Counter;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    ret_code_t err_code = nrf_drv_timer_init(&m_timer2, &timer_cfg, empty_timer_handler);
    
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("Timer2 counter mode setup");
}

static void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
// empty handler
}

uint32_t event_address_rising;
uint32_t event_address_falling;

static void setup_gpiote_event()
{

    uint32_t err_code = nrf_drv_gpiote_init();
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);

    err_code = nrf_drv_gpiote_in_init(TEMP_SENSOR_1, &config, gpiote_event_handler);
    APP_ERROR_CHECK(err_code);

    event_address_rising = nrf_drv_gpiote_in_event_addr_get(TEMP_SENSOR_1);

        NRF_LOG_DEBUG("gpiote_init rising edge");

    nrf_drv_gpiote_in_config_t config1 = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    err_code = nrf_drv_gpiote_in_init(TEMP_SENSOR_2, &config1, gpiote_event_handler);
    APP_ERROR_CHECK(err_code);

    event_address_falling = nrf_drv_gpiote_in_event_addr_get(TEMP_SENSOR_2);

    NRF_LOG_DEBUG("gpiote_init_falling edge");


}


static void setup_timer_and_counter_ppi()
{
    uint32_t err_code = NRF_SUCCESS;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
    /* Configure 1st available PPI channel to stop TIMER0 counter on TIMER1 COMPARE[0] match,
     * which is every even number of seconds.
     */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel1);
    APP_ERROR_CHECK(err_code);

    uint32_t timer2_capture0 = nrf_drv_timer_capture_task_address_get(&m_timer2,NRF_TIMER_TASK_CAPTURE0);
    uint32_t timer1_compare0 =  nrf_drv_timer_event_address_get(&m_timer1,NRF_TIMER_EVENT_COMPARE0);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel1,timer1_compare0, timer2_capture0);
    APP_ERROR_CHECK(err_code);


    uint32_t task_address_counter_increment = nrf_drv_timer_task_address_get(&m_timer2, NRF_TIMER_TASK_COUNT);
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel2);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel2, event_address_rising, task_address_counter_increment);
    APP_ERROR_CHECK(err_code);


    uint32_t timer1_capture2 = nrf_drv_timer_capture_task_address_get(&m_timer1,NRF_TIMER_TASK_CAPTURE2);
    err_code = nrf_drv_ppi_channel_fork_assign(m_ppi_channel2, timer1_capture2);
    APP_ERROR_CHECK(err_code); 



    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel3);
    APP_ERROR_CHECK(err_code);  

    uint32_t timer1_capture3 = nrf_drv_timer_capture_task_address_get(&m_timer1,NRF_TIMER_TASK_CAPTURE3);
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel3, event_address_falling, timer1_capture3);
    APP_ERROR_CHECK(err_code);



    
    uint32_t timer2_stop = nrf_drv_timer_task_address_get(&m_timer2,NRF_TIMER_TASK_STOP);
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel4);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel4, timer1_compare0, timer2_stop);
    APP_ERROR_CHECK(err_code);


    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel1);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel2);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel3);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel4);
    APP_ERROR_CHECK(err_code);


    nrf_drv_gpiote_in_event_enable(TEMP_SENSOR_1, true);
    nrf_drv_gpiote_in_event_enable(TEMP_SENSOR_2, true);
}




// static int decimal_part(double num){
//   int intpart = (int)num;
//   double decpart = num - intpart;
//   int decimal = decpart*100;
//   NRF_LOG_DEBUG("decimal part is %d", decimal);
//   return decimal;
// }

// static int exponent_part(double num){
//     int intpart = (int)num;
//       NRF_LOG_DEBUG("int part is %d", intpart);
//     return intpart;
// }

// static void temp_sensor_measure(void){
//     NRF_TIMER1->TASKS_START = 1;
//     NRF_TIMER2->TASKS_START = 1;
// }






//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
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



    // // call the clock configuration
    // lfclk_config();

    // call the rtc configuration

        // temp sensor code
    
    timer_init();
    counter_init();
    setup_gpiote_event();
    setup_timer_and_counter_ppi();
    nrf_drv_timer_enable(&m_timer1);
    nrf_drv_timer_enable(&m_timer2);

    // Sleep in the while loop until an event is generated
    while (true)
    {
        NRF_LOG_FLUSH();
        // __SEV();
        // __WFE();
        // __WFE();
    }
}