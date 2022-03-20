/**
 * Copyright (c) 2017 - 2021, Nordic Semiconductor ASA
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
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include <app_usbd_cdc_acm.h>
#include "app_usbd_serial_num.h"

#include "nrf_delay.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_cli.h"
#include "nrf_cli.h"
#include "nrf_cli_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_usbd.h"

//#define LED_USB_RESUME      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN    (BSP_BOARD_LED_0)
#define LED_CDC_ACM_RX      (BSP_BOARD_LED_1)
//#define LED_CDC_ACM_TX      (BSP_BOARD_LED_1)

#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1


/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
static bool m_send_flag = 1;
int interval;

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            //Setup first transfer
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   READ_SIZE);
            UNUSED_VARIABLE(ret);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            //bsp_board_led_off(LED_CDC_ACM_OPEN);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            //bsp_board_led_invert(LED_CDC_ACM_TX);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            // LED ensures that RX is completed
            bsp_board_led_invert(LED_CDC_ACM_OPEN);

            // Using the amounts of bytes stored in buffer to determine interval
            if (app_usbd_cdc_acm_bytes_stored(&m_app_cdc_acm) == 1)
            {
                interval = 1800; //30 mins in sec
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
            }
            if (app_usbd_cdc_acm_bytes_stored(&m_app_cdc_acm) == 2)
            {
                interval = 3600; //1 hr in sec
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
            }
            if (app_usbd_cdc_acm_bytes_stored(&m_app_cdc_acm) == 3)
            {
                interval = 7200; //2 hr in sec
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
            }
            if (app_usbd_cdc_acm_bytes_stored(&m_app_cdc_acm) == 4)
            {
                interval = 21600; //6 hr in sec
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                 bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
            }
            if (app_usbd_cdc_acm_bytes_stored(&m_app_cdc_acm) == 5)
            {
                interval = 43200; //12 hr in sec
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                 bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500); 
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
            }
            if (app_usbd_cdc_acm_bytes_stored(&m_app_cdc_acm) == 6)
            {
                interval = 86400; //1 day in sec
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                 bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500); 
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500); 
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
            }
            if (app_usbd_cdc_acm_bytes_stored(&m_app_cdc_acm) == 7)
            {
                interval = 604800; //1 week in sec
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                 bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500); 
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500); 
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500); 
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
            }
            if (app_usbd_cdc_acm_bytes_stored(&m_app_cdc_acm) == 8)
            {
                interval = 2416200; //4 week in sec
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                 bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500); 
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500); 
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500); 
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500); 
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
                bsp_board_led_invert(LED_CDC_ACM_RX);
                nrf_delay_ms(1500);
            }
            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            //bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
            //bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            //NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            //NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            //NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}

int main(void)
{
    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_led_invert(0);
    nrf_delay_ms(500);
    bsp_board_led_invert(1);
    nrf_delay_ms(500);
    bsp_board_led_invert(0);
    nrf_delay_ms(500);
    bsp_board_led_invert(1);
    nrf_delay_ms(500);

    int Temp1 = 15;
    int Temp2 = 17;
    int Volt1 = 21;
    int Volt2 = 23;

    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    
    nrf_drv_clock_lfclk_request(NULL);

    while(!nrf_drv_clock_lfclk_is_running())
    {
         //Just waiting 
    }

    app_usbd_serial_num_generate();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        app_usbd_enable();
        app_usbd_start();
    }

    while(true)
    {
        while (app_usbd_event_queue_process())
        {
             //Nothing to do 
        }
        
        if(m_send_flag)
        {
            bsp_board_led_invert(0);

            size_t size = sprintf(m_tx_buffer, "%u.%u,%u.%u\r\n", Temp1,Temp2,Volt1,Volt2);
            ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, size);

        }  

        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
         //Sleep CPU only if there was no interrupt since last loop processing 
        __WFE();
    }
}

/** @} */
