/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
 * @defgroup ble_sdk_app_hrs_eval_led led.c
 * @{
 * @ingroup ble_sdk_app_hrs_eval
 * @brief LED control for the HRS example application
 *
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "boards.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "led.h"
#include "app_util.h"

#define SQUARE_WAVE_PIN_NO               		21                                      /**< Is on when device is advertising. */

#define PPI_CHAN0_TO_TOGGLE_LED              0                                         /**< The PPI Channel that connects CC0 compare event to the GPIOTE Task that toggles the Advertising LED. */
#define GPIOTE_CHAN_FOR_LED_TASK             0                                         /**< The GPIOTE Channel used to perform write operation on the Advertising LED pin. */
#define TIMER_PRESCALER                      9                                         /**< Prescaler setting for timer. */
#define CAPTURE_COMPARE_0_VALUE              0x1E84                                    /**< Capture compare value that corresponds to 250 ms. */

/** @brief Function for Timer1 initialization.
 *
 * @details This function will initialise Timer 1 peripheral. This timer is used only to
 *          generate capture compare events that toggle the advertising LED state.
 */
static void timer1_init(void)
{
   /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED    = 0;
    NRF_CLOCK->TASKS_HFCLKSTART       = 1;
    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)       
    {
        // Do nothing.
    }
		
		
		NRF_TIMER2->TASKS_CLEAR = 1;
    NRF_TIMER2->MODE        = TIMER_MODE_MODE_Timer;
		NRF_TIMER2->PRESCALER   = 2; // 1MHz

    /* Load initial values to Timer 2 CC registers */
    /* Set initial CC0 value to anything > 1 */
		
		NRF_TIMER2->CC[0] = 1;
		NRF_TIMER2->CC[1] = 2;
  
		NRF_TIMER2->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
		
}


/** @brief Function for the PPI initialization.
 *
 * @details This function will initialise Programmable Peripheral Interconnect peripheral. It will
 *          configure the PPI channels as follows -
 *              PPI Channel 0 - Connecting CC0 Compare event to GPIOTE Task to toggle the LED state
 *          This configuration will feed a PWM input to the LED thereby making it flash in an
 *          interval that is dependent on the TIMER configuration.
 */
static void ppi_init(void)
{
    uint32_t err_code;

	 err_code = sd_ppi_channel_assign(0,
                                     &(NRF_TIMER2->EVENTS_COMPARE[0]),
                                     &(NRF_GPIOTE->TASKS_OUT[0]));
    APP_ERROR_CHECK(err_code);
	
		err_code = sd_ppi_channel_assign(1,
                                      &(NRF_TIMER2->EVENTS_COMPARE[1]),
                                     &(NRF_GPIOTE->TASKS_OUT[0]));
    APP_ERROR_CHECK(err_code);
	
	
		err_code = sd_ppi_channel_assign(2,
                                     &(NRF_TIMER2->EVENTS_COMPARE[1]),
                                     &(NRF_TIMER2->TASKS_CLEAR));
    APP_ERROR_CHECK(err_code);
	
		err_code = sd_ppi_channel_enable_set( (PPI_CHEN_CH0_Msk) | (PPI_CHEN_CH1_Msk)| (PPI_CHEN_CH2_Msk));
    APP_ERROR_CHECK(err_code);
    // Enable PPI channel 0 
}

static void gpiote_init(void)
{
    // Configure the GPIOTE Task to toggle the LED state.
    nrf_gpiote_task_config(GPIOTE_CHAN_FOR_LED_TASK,
                           SQUARE_WAVE_PIN_NO,
                           NRF_GPIOTE_POLARITY_TOGGLE,
                           NRF_GPIOTE_INITIAL_VALUE_LOW);
}




/**
 * @}
 */
