// // // The main code for the on-site controller goes here
// // int main(){
// //     return 0;
// // }
// #include <stdint.h>
// #include <string.h>
// #include "nordic_common.h"
// #include "nrf.h"
// #include "app_error.h"
// #include "ble.h"
// #include "ble_err.h"
// #include "ble_hci.h"
// #include "ble_srv_common.h"
// #include "ble_advdata.h"
// #include "ble_conn_params.h"
// #include "nrf_sdh.h"
// #include "nrf_sdh_ble.h"
// #include "boards.h"
// #include "app_timer.h"
// #include "app_button.h"
// #include "ble_lbs.h"
// #include "nrf_ble_gatt.h"
// #include "nrf_ble_qwr.h"
// #include "nrf_pwr_mgmt.h"
// #include "nrf_delay.h"
// #include "nrf_gpio.h"
// #include "nrf_drv_gpiote.h"

// #include <stdbool.h>
// #include <stdio.h>
// #include "nrf_drv_saadc.h"
// #include "nrf_drv_ppi.h"
// #include "nrf_drv_timer.h"
// #include "app_util_platform.h"

// #include "nrf_log.h"
// #include "nrf_log_ctrl.h"
// #include "nrf_log_default_backends.h"

// #define FREQ_MEASURE_PIN NRF_GPIO_PIN_MAP(0,11)
// #define output_pin NRF_GPIO_PIN_MAP(0,30)


// static void timer_init()
// {
// 	NRF_TIMER1->TASKS_STOP = 1;
// 	NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
// 	NRF_TIMER1->PRESCALER = 8;	// Fhck / 2^8 
// 	NRF_TIMER1->CC[0] = 62500;	// 62500 - 1s
	
// 	NRF_TIMER1->BITMODE = (TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos);	
	
// 	NRF_TIMER1->TASKS_CLEAR = 1;
// 	NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
	
// 	NRF_TIMER1->EVENTS_COMPARE[0] = 0;
// }

// static void counter_init()
// {
// 	NRF_TIMER2->TASKS_STOP = 1;	
// 	NRF_TIMER2->MODE = TIMER_MODE_MODE_Counter;
// 	NRF_TIMER2->BITMODE = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);
// 	NRF_TIMER2->TASKS_CLEAR = 1;
// 	NRF_TIMER2->EVENTS_COMPARE[0] = 0;
// }

// static void gpiote_init(uint32_t pin)
// {
// 	NRF_GPIOTE->CONFIG[0] 	= 	0x01 << 0; 								// MODE: Event
// 	NRF_GPIOTE->CONFIG[0] 	|= 	pin << 8;								// Pin number
// 	NRF_GPIOTE->CONFIG[0] 	|= 	NRF_GPIOTE_POLARITY_LOTOHI	<< 16;		// Event rising edge 	
// }

// static void ppi_timer_stop_counter_init()
// {
// 	NRF_PPI->CHEN |= 1 << 0;
// 	*(&(NRF_PPI->CH0_EEP)) = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[0];
// 	*(&(NRF_PPI->CH0_TEP)) = (uint32_t)&NRF_TIMER2->TASKS_STOP;
// 	NRF_PPI->CHENSET |= 1 << 0;
// }

// static void ppi_gpiote_counter_init()
// {
// 	NRF_PPI->CHEN |= 1 << 1;
// 	*(&(NRF_PPI->CH1_EEP)) = (uint32_t)&NRF_GPIOTE->EVENTS_IN[0];
// 	*(&(NRF_PPI->CH1_TEP)) = (uint32_t)&NRF_TIMER2->TASKS_COUNT;
// 	NRF_PPI->CHENSET |= 1 << 1;
// }

// int main()
// {
//     // Soft Device initialization..
//     NRF_LOG_INFO("Program Start");
//    	NVIC_EnableIRQ(TIMER1_IRQn);
//     NVIC_SetPriority(TIMER1_IRQn, APP_IRQ_PRIORITY_LOW);	

//     nrf_gpio_cfg_input(FREQ_MEASURE_PIN, NRF_GPIO_PIN_NOPULL);

// 	counter_init();
// 	timer_init();
// 	gpiote_init(FREQ_MEASURE_PIN);
// 	ppi_gpiote_counter_init();
// 	ppi_timer_stop_counter_init();

// 	NRF_TIMER1->TASKS_START = 1;
// 	NRF_TIMER2->TASKS_START = 1;
	
// 	for(;;) {
// 		// power manage //
// 	}
// }

// void TIMER1_IRQHandler(void) 
// {
// 	if (NRF_TIMER1->EVENTS_COMPARE[0] != 0)
// 	{
// 		NRF_TIMER1->EVENTS_COMPARE[0] = 0;
// 		NRF_TIMER2->TASKS_CAPTURE[0] = 1;
				
// 		NRF_LOG_INFO("cc: %dHz", NRF_TIMER2->CC[0]);



// 		NRF_TIMER1->TASKS_CLEAR = 1;
// 		NRF_TIMER2->TASKS_CLEAR = 1;	
						
// 		// NRF_TIMER2->TASKS_START = 1;			
//     }
// }



// // static void timer_init_2()
// // {
// // 	NRF_TIMER1->TASKS_STOP = 1;
// // 	NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
// // 	NRF_TIMER1->PRESCALER = 8; //8;	// Fhck / 2^8 
// // 	NRF_TIMER1->CC[0] =  62500;	// 62500 - 1s
	
// // 	NRF_TIMER1->BITMODE = (TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos);	
	
// // 	NRF_TIMER1->TASKS_CLEAR = 1;
// // 	NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
	
// // 	NRF_TIMER1->EVENTS_COMPARE[0] = 0;
// // }

// // static void counter_init()
// // {
// // 	NRF_TIMER2->TASKS_STOP = 1;	
// // 	NRF_TIMER2->MODE = TIMER_MODE_MODE_Counter;
// // 	NRF_TIMER2->BITMODE = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);
// // 	NRF_TIMER2->TASKS_CLEAR = 1;
// // 	NRF_TIMER2->EVENTS_COMPARE[0] = 0;
// // }

// // static void gpiote_init(uint32_t pin)
// // {
// // 	NRF_GPIOTE->CONFIG[0] 	= 	0x01 << 0; 								// MODE: Event
// // 	NRF_GPIOTE->CONFIG[0] 	|= 	pin << 8;								// Pin number
// // 	NRF_GPIOTE->CONFIG[0] 	|= 	NRF_GPIOTE_POLARITY_LOTOHI	<< 16;		// Event rising edge 	
// // }

// // static void ppi_timer_stop_counter_init()
// // {
// // 	NRF_PPI->CHEN |= 1 << 0;
// // 	*(&(NRF_PPI->CH0_EEP)) = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[0];
// // 	*(&(NRF_PPI->CH0_TEP)) = (uint32_t)&NRF_TIMER2->TASKS_STOP;
// // 	NRF_PPI->CHENSET |= 1 << 0;
// // }

// // static void ppi_gpiote_counter_init()
// // {
// // 	NRF_PPI->CHEN |= 1 << 1;
// // 	*(&(NRF_PPI->CH1_EEP)) = (uint32_t)&NRF_GPIOTE->EVENTS_IN[0];
// // 	*(&(NRF_PPI->CH1_TEP)) = (uint32_t)&NRF_TIMER2->TASKS_COUNT;
// // 	NRF_PPI->CHENSET |= 1 << 1;
// // }

// // void TIMER1_IRQHandler(void) 
// // {
// // 	if (NRF_TIMER1->EVENTS_COMPARE[0] != 0)
// // 	{
// // 		NRF_TIMER1->EVENTS_COMPARE[0] = 0;
// // 		NRF_TIMER2->TASKS_CAPTURE[0] = 1;
				
// // 		NRF_LOG_INFO("cc: %dHz", NRF_TIMER2->CC[0]);
		
      	
// // 		NRF_TIMER1->TASKS_CLEAR = 1;
// // 		NRF_TIMER2->TASKS_CLEAR = 1;	
						
// // 		NRF_TIMER2->TASKS_START = 1;			
// //     }
// // }


// // int main(void)
// // {
    
// //     // Start scanning for peripherals and initiate connection
// //     // with devices that advertise KALE UUID.
// // 			NRF_LOG_INFO("KEYFOB example started.");
		
// // 		 // Soft Device initialization..
// // 			nrf_gpio_cfg_output(output_pin);
// // 			NVIC_EnableIRQ(TIMER1_IRQn);
// // 			NVIC_SetPriority(TIMER1_IRQn, 7);	

// // 			nrf_gpio_cfg_input(FREQ_MEASURE_PIN, NRF_GPIO_PIN_NOPULL);

// // 			counter_init();
// // 			timer_init_2();
// // 			gpiote_init(FREQ_MEASURE_PIN);
// // 			ppi_gpiote_counter_init();
// // 			ppi_timer_stop_counter_init();

// // 			NRF_TIMER1->TASKS_START = 1;
// // 			NRF_TIMER2->TASKS_START = 1;
			
// // 	for(;;)
// // 	{
// //         // if (NRF_LOG_PROCESS() == true)
// //         // {
// //         //     nrf_pwr_mgmt_run();
// //         // }
// //     }
// // }


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

#include "nrf_delay.h"
#include "app_error.h"

#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "app_timer.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_gpiote.h"
#include "nrfx_gpiote.h"
#include <math.h>

#define PIN_IN NRF_GPIO_PIN_MAP(0,11)
static const nrf_drv_timer_t m_timer_capture = NRFX_TIMER_INSTANCE(1);
static const nrf_drv_timer_t m_timer_compare = NRFX_TIMER_INSTANCE(2);

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
NRF_LOG_INFO("In pin handler");
}

void timer_handler_read(nrf_timer_event_t event_type, void * p_context)
{NRF_LOG_INFO("timer handler read");}

uint8_t data_high=0;
uint8_t data_array[23];
uint8_t index1 = 0;
void timer_handler_compare(nrf_timer_event_t event_type, void * p_context)
{
    // ret_code_t err_code;
    int distance_1;
    uint8_t data_high;
    distance_1 = nrfx_timer_capture_get(&m_timer_capture, NRF_TIMER_CC_CHANNEL0);
    
    if (distance_1 < 1200 && distance_1 > 70){
        data_high = fmin( fmax(round((distance_1 - 16.0 - 52.0 - 16.0 + 4.0) / 4.0), 0.0), 255.0);
        //NRF_LOG_INFO("%d", distance_1);
        data_array[index1]=data_high;
        index1++;
        
        // if (index1 >= 10)
        // {
        //     if (index1 >= 0)
        //     {
        //         do
        //         {
                    //NRF_LOG_INFO("H");
        //             // uint16_t length = 10;
        //             // err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);

        //         } while (err_code == NRF_ERROR_RESOURCES);
        //     }
        //     index1 = 0;
        // }
    }
}

/**
 * @brief Function for configuring: PIN_IN pin for input sensing
 */
static void gpiote_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_gpiote_in_event_enable(PIN_IN, false);
    NRF_LOG_INFO("gpiote init");
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    
    err_code = nrfx_timer_init(&m_timer_capture, &timer_cfg, timer_handler_read);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_timer_init(&m_timer_compare, &timer_cfg, timer_handler_compare);
    APP_ERROR_CHECK(err_code);

    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer_compare, 10);
    nrf_drv_timer_extended_compare(&m_timer_compare,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);

    nrf_drv_timer_enable(&m_timer_capture);
    nrf_drv_timer_enable(&m_timer_compare);
    NRF_LOG_INFO("timers_init");
    
}
nrf_ppi_channel_t ppi_channel_1;
void ppi_init()
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel_1);
    APP_ERROR_CHECK(err_code);
    
    
    uint32_t gpiote_evt_addr_1              = nrf_drv_gpiote_in_event_addr_get(PIN_IN);
    
    uint32_t timer_capture_task_addr  = nrf_drv_timer_task_address_get(&m_timer_capture, NRF_TIMER_TASK_CAPTURE0);
    uint32_t timer_clear_task_addr    = nrf_drv_timer_task_address_get(&m_timer_capture, NRF_TIMER_TASK_CLEAR);
    
    
    
    err_code = nrf_drv_ppi_channel_assign(ppi_channel_1, gpiote_evt_addr_1, timer_capture_task_addr);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_fork_assign(ppi_channel_1, timer_clear_task_addr);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_enable(ppi_channel_1);
    APP_ERROR_CHECK(err_code);
    //err_code = nrf_drv_ppi_channel_enable(ppi_channel_2);
    APP_ERROR_CHECK(err_code);
}
/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    gpiote_init();
    timers_init();
    ppi_init();

    NRF_LOG_INFO("PPI example started.?!?!?!?!");

    for (;;)
    {

            NRF_LOG_PROCESS();
        // idle_state_handle();
        // nrf_delay_us(3000);        
    }
}

/** @} */