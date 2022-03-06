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
#define TEMP_SENSOR_2 NRF_GPIO_PIN_MAP(0, 11)

#define output_pin NRF_GPIO_PIN_MAP(0, 12)
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
static void timer_init()
{
    // nrf_gpio_cfg_output(output_pin);
    // // nrf_gpio_pin_set(output_pin);

    // NVIC_EnableIRQ(TIMER1_IRQn);
    // NVIC_SetPriority(TIMER1_IRQn, APP_IRQ_PRIORITY_LOW);
    // nrf_gpio_cfg_input(TEMP_SENSOR_1, NRF_GPIO_PIN_NOPULL);

    // NRF_TIMER1->TASKS_STOP = 1;
    // NRF_TIMER1->TASKS_CLEAR = 1;
    // NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
    // NRF_TIMER1->PRESCALER = 0; // uses 16 MHz clk
    // NRF_TIMER1->CC[0] = 40000; // approx - 10^-2 / 4 s

    // NRF_TIMER1->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);

    // NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);

    // NRF_TIMER1->EVENTS_COMPARE[0] = 0;
    
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_16MHz;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;

    ret_code_t err_code = nrf_drv_timer_init(&m_timer0, &timer_cfg, timer0_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_extended_compare(&m_timer0,
                                   NRF_TIMER_CC_CHANNEL0,
                                   nrf_drv_timer_ms_to_ticks(&m_timer1,
                                                             40000),
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK,
                                   true);

    NRF_LOG_DEBUG("Timer1 setup");
}

static void counter_init()
{
    // NRF_TIMER2->TASKS_STOP = 1;
    // NRF_TIMER1->TASKS_CLEAR = 1;
    // NRF_TIMER2->MODE = TIMER_MODE_MODE_Counter;
    // NRF_TIMER2->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);
    // NRF_TIMER2->TASKS_CLEAR = 1;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_16MHz;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;


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
    uint32_t err_code = NRF_SUCCESS;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
    /* Configure 1st available PPI channel to stop TIMER0 counter on TIMER1 COMPARE[0] match,
     * which is every even number of seconds.
     */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel1,
                                        nrf_drv_timer_event_address_get(&m_timer1,
                                                                        NRF_TIMER_EVENT_COMPARE0),
                                        nrf_drv_timer_task_address_get(&m_timer0,
                                                                        NRF_TIMER_TASK_STOP));
}

nrf_gpiote_event_addr_get

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

static void temp_sensor_measure(void){
    NRF_TIMER1->TASKS_START = 1;
    NRF_TIMER2->TASKS_START = 1;
}


static void timer1_event_handler(nrf_timer_event_t event_type, void * p_context)
{
    ++m_counter;
}
/* Timer event handler. Not used since Timer1 and Timer2 are used only for PPI. */
static void empty_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
}

// void TIMER1_IRQHandler(void)
// {
//     if (NRF_TIMER1->EVENTS_COMPARE[0] == 1)
//     {
//         NRF_TIMER1->EVENTS_COMPARE[0] = 0;

//         int pulse_width = NRF_TIMER1->CC[3] - NRF_TIMER1->CC[2];


//         if ((pulse_width) < 0)
//         {
//             NRF_TIMER1->TASKS_CLEAR = 1;
//             NRF_TIMER2->TASKS_CLEAR = 1;

//             NRF_TIMER2->CC[0] = 0;
//             NRF_TIMER1->CC[2] = 0;
//             NRF_TIMER1->CC[3] = 0;

//             NRF_TIMER1->TASKS_START = 1;
//             NRF_TIMER2->TASKS_START = 1;
//         }
//         else
//         {
//             frequency = NRF_TIMER2->CC[0] * 4 * 100;
//             duty_cycle = (double)(frequency) * (pulse_width) / 16000000;
//             NRF_TIMER1->TASKS_CLEAR = 1;
//             NRF_TIMER2->TASKS_CLEAR = 1;
//             if (valid_temp_counter == NUM_TEMPERATURE_PERIODS)
//             {

//                 double average_duty_cycle = 0;
//                 for (uint16_t i = 0; i < NUM_TEMPERATURE_PERIODS; i++)
//                 {
//                     average_duty_cycle = average_duty_cycle + valid_duty_cycle[i];
//                 }
//                 average_duty_cycle = average_duty_cycle / NUM_TEMPERATURE_PERIODS;
//                 // NRF_LOG_INFO("Averaged Duty Cycle " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(average_duty_cycle));
//                 temperature = -1.43 * average_duty_cycle * average_duty_cycle + 214.56 * average_duty_cycle - 68.60;
//                 valid_temp_counter = 0;

//                 expo = exponent_part(temperature);
//                 temperature_encoded = decimal_part(temperature);
//                 NRF_LOG_INFO("Temperature [Deg C] " NRF_LOG_FLOAT_MARKER "\r", NRF_LOG_FLOAT(temperature));
                

//                 if(temp_sensor == false){
//                     // ble_write_to_characteristic(expo, temperature_encoded, temperature_1_char_handles);
//                     temp_sensor = true;
//                     setup_gpiote_event(TEMP_SENSOR_2);

//                     NRF_TIMER2->CC[0] = 0;
//                     NRF_TIMER1->CC[2] = 0;
//                     NRF_TIMER1->CC[3] = 0;
//                     NRF_TIMER1->TASKS_START = 1;
//                     NRF_TIMER2->TASKS_START = 1;
//                 }
//                 else{
//                     // ble_write_to_characteristic(expo, temperature_encoded, temperature_2_char_handles);
//                     temp_sensor = false;
                    
//                     NRF_TIMER2->CC[0] = 0;
//                     NRF_TIMER1->CC[2] = 0;
//                     NRF_TIMER1->CC[3] = 0;

//                     temp_sensor_measure();
//                 }
                
//                 // nrf_delay_ms(10000);
//                 // NRF_TIMER1->TASKS_START = 1;
//                 // NRF_TIMER2->TASKS_START = 1;
//             }
//             else
//             {
//                 valid_duty_cycle[valid_temp_counter] = duty_cycle;
//                 valid_temp_counter += 1;
                
//                 NRF_TIMER2->CC[0] = 0;
//                 NRF_TIMER1->CC[2] = 0;
//                 NRF_TIMER1->CC[3] = 0;
//                 NRF_TIMER1->TASKS_START = 1;
//                 NRF_TIMER2->TASKS_START = 1;
//             }
//         }
//     }
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
    setup_gpiote_event(TEMP_SENSOR_1);
    setup_timer_and_counter_ppi();
    temp_sensor_measure();

    temp_sensor_measure();


    // Sleep in the while loop until an event is generated
    while (true)
    {
        NRF_LOG_FLUSH();
        // __SEV();
        // __WFE();
        // __WFE();
    }
}