#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "boards.h"
#include "app_error.h"
#include <stdint.h>
#include <stdbool.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// COMPARE_COUNTERTIME is actually given in seconds, so you can simply change 3 into whatever amount of seconds you want to use.
#define COMPARE_COUNTERTIME  (3UL)                                        /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */

#define ADVERTISING_LED BSP_BOARD_LED_1 /**< Is on when device is advertising. */
#define UNEXPECTED_LED BSP_BOARD_LED_3 
// Create a handle that will point to the RTC 2 of nrf device
const nrf_drv_rtc_t rtc = NRFX_RTC_INSTANCE(2); // rtc 2 handle


// Initialize the low frequency clock so that the rtc can be fed by this clock
// when using soft-device, this function is not needed, we will see this in future tutorials
// once we start to program the bluetooth communication

static void lfclk_config(void)
{
    // Initialize the low frequency clock
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code); // check for the errors

    // Request the clock to not to generate events
    nrf_drv_clock_lfclk_request(NULL);
    NRF_LOG_DEBUG("Low frequency clock setup");
}

// RTC interrupt handler which will be used to handle the interrupt events
static void rtc_handler(nrfx_rtc_int_type_t int_type)
{

    // Check if the interrupt occurred due to tick event
    if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        // perform some action
        bsp_board_led_invert(ADVERTISING_LED);
    }
    else if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        NRF_LOG_DEBUG("RTC compare event");
        bsp_board_led_invert(UNEXPECTED_LED);
        nrf_rtc_task_trigger(rtc.p_reg, NRF_RTC_TASK_CLEAR);
        nrf_drv_rtc_cc_set(&rtc,0,COMPARE_COUNTERTIME * 8,true);
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

    // Generate a tick event on each tick
    // nrf_drv_rtc_tick_enable(&rtc, true);

    err_code = nrf_drv_rtc_cc_set(&rtc,0,COMPARE_COUNTERTIME * 8,true);
    APP_ERROR_CHECK(err_code);

    // start the rtc
    nrf_drv_rtc_enable(&rtc);
    NRF_LOG_DEBUG("RTC config");
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
log_init();
    // Initialize the LEDS
    bsp_board_init(BSP_INIT_LEDS);

    // // call the clock configuration
    lfclk_config();

    // call the rtc configuration
    rtc_config();

    // Sleep in the while loop until an event is generated
    while (true)
    {
        NRF_LOG_FLUSH();
        __SEV();
        __WFE();
        __WFE();
    }
}