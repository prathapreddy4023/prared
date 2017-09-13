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
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "fds.h"
#include "fstorage.h"
#include "nrf_ble_gatt.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "app_twi.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_twi.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_twi.h"
#include "lm75b.h"
#include "mma7660.h"
#include "compiler_abstraction.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define MAX30100_ADDRESS 0x57

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define DEVICE_NAME                      "Nordic_HRM"                                /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 300                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       180                                         /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                           /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)  /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                81                                          /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                100                                         /**< Maximum simulated 7battery level. */
#define BATTERY_LEVEL_INCREMENT          1                                           /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL         APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< Heart rate measurement interval (ticks). */
#define MIN_HEART_RATE                   140                                         /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                   300                                         /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT             10                                          /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL             APP_TIMER_TICKS(300, APP_TIMER_PRESCALER)   /**< RR interval interval (ticks). */
#define MIN_RR_INTERVAL                  100                                         /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                  500                                         /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT            1                                           /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(400, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(650, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                    0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_FEATURE_NOT_SUPPORTED        BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define MAX_PENDING_TRANSACTIONS    5

#define MAX_PENDING_TRANSACTIONS    5

#define APP_TIMER_PRESCALER         0
//#define APP_TIMER_OP_QUEUE_SIZE     2

// [use "/ 32" instead of ">> 5", as the result of right-shifting of a signed
//  type value is implementation-defined]
#define LM75B_GET_TEMPERATURE_VALUE_(temp_hi, temp_lo) \
    (temp_hi+temp_lo*0.0625)

static uint16_t  m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static ble_bas_t m_bas;                                   /**< Structure used to identify the battery service. */
static ble_hrs_t m_hrs;                                   /**< Structure used to identify the heart rate service. */
static bool      m_rr_interval_enabled = true;            /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

static nrf_ble_gatt_t m_gatt;                             /**< Structure for gatt module*/

static sensorsim_cfg_t   m_battery_sim_cfg;               /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;             /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t   m_heart_rate_sim_cfg;            /**< Heart Rate sensor simulator configuration. */
static sensorsim_state_t m_heart_rate_sim_state;          /**< Heart Rate sensor simulator state. */
static sensorsim_cfg_t   m_rr_interval_sim_cfg;           /**< RR Interval sensor simulator configuration. */
static sensorsim_state_t m_rr_interval_sim_state;         /**< RR Interval sensor simulator state. */

static nrf_drv_twi_t m_twi = 	NRF_DRV_TWI_INSTANCE(0);
static nrf_drv_rtc_t const m_rtc = NRF_DRV_RTC_INSTANCE(0);

APP_TIMER_DEF(m_battery_timer_id);                        /**< Battery timer. */
APP_TIMER_DEF(m_heart_rate_timer_id);                     /**< Heart rate measurement timer. */
APP_TIMER_DEF(m_rr_interval_timer_id);                    /**< RR interval timer. */                 /**< RR interval timer. */
APP_TIMER_DEF(m_sensor_contact_timer_id);                 /**< Sensor contact detected timer. */


// Pin number for indicating communication with sensors.
#ifdef BSP_LED_3
    #define READ_ALL_INDICATOR  BSP_LED_3
#else
    #error "Please choose an output pin"
#endif

// Buffer for data read from sensors.
#define BUFFER_SIZE  11
static uint8_t m_buffer[BUFFER_SIZE];

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HEART_RATE_SERVICE, BLE_UUID_TYPE_BLE},
                                   {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
                                   {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

																	 // Data structures needed for averaging of data read from sensors.
// [max 32, otherwise "int16_t" won't be sufficient to hold the sum
//  of temperature samples]
#define NUMBER_OF_SAMPLES  16
typedef struct
{
    int16_t temp;
    int16_t x;
    int16_t y;
    int16_t z;
} sum_t;
static sum_t m_sum = { 0, 0, 0, 0 };
typedef struct
{
    // [use bit fields to fit whole structure into one 32-bit word]
    uint16_t IR  :16;
	  int16_t  RED :16; 
	  float32_t   TEMP;
	//int8_t  y      : 6;
	//int8_t  z      : 6;
} sample_t;
static sample_t m_samples[NUMBER_OF_SAMPLES] = { { 0, 0,0} };
static uint8_t m_sample_idx = 0;

// Value previously read from MMA7660's Tilt register - used to detect change
// in orientation, shake signaling etc.
static uint8_t m_prev_tilt = 0;

#if defined( __GNUC__ ) && (__LINT__ == 0)
    // This is required if one wants to use floating-point values in 'printf'
    // (by default this feature is not linked together with newlib-nano).
    // Please note, however, that this adds about 13 kB code footprint...
    __ASM(".global _printf_float");
#endif

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    ret_code_t err_code;

    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling File Data Storage events.
 *
 * @param[in] p_evt  Peer Manager event.
 * @param[in] cmd
 */
static void fds_evt_handler(fds_evt_t const * const p_evt)
{
    if (p_evt->id == FDS_EVT_GC)
    {
        NRF_LOG_DEBUG("GC completed\n");
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */

void read_all_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("read_all_cb - error: %d\r\n", (int)result);
        return;
    }

    sample_t * p_sample = &m_samples[m_sample_idx];
    m_sum.temp -= p_sample->IR;
    m_sum.x    -= p_sample->RED;
    //m_sum.y    -= p_sample->y;
    //m_sum.z    -= p_sample->z;

    uint8_t temp_hi = m_buffer[0];
    uint8_t temp_lo = m_buffer[1];
    uint8_t x_out   = m_buffer[2];
    uint8_t y_out   = m_buffer[3];
    uint8_t z_out    = m_buffer[4];
    uint8_t tilt    = m_buffer[5];

    p_sample->IR = LM75B_GET_TEMPERATURE_VALUE(temp_hi, temp_lo);
    p_sample->RED = LM75B_GET_TEMPERATURE_VALUE(x_out,y_out);
    p_sample->TEMP = LM75B_GET_TEMPERATURE_VALUE_(z_out,tilt);
		//GET_ACC_VALUE(p_sample->x, x_out);
    //GET_ACC_VALUE(p_sample->y, y_out);
    //GET_ACC_VALUE(p_sample->z, z_out);
    if (!MMA7660_DATA_IS_VALID(tilt))
    {
        tilt = m_prev_tilt;
    }

    m_sum.temp += p_sample->IR;
    m_sum.x    += p_sample->RED;
    //m_sum.y    += p_sample->y;
    //m_sum.z    += p_sample->z;

    ++m_sample_idx;
    if (m_sample_idx >= NUMBER_OF_SAMPLES)
    {
        m_sample_idx = 0;
    }

    // Show current average values every time sample index rolls over (for RTC
    // ticking at 32 Hz and 16 samples it will be every 500 ms) or when tilt
    // status changes.
    if (m_sample_idx == 0 || (m_prev_tilt && m_prev_tilt != tilt))
    {
      //  char const * orientation;
//        switch (MMA7660_GET_ORIENTATION(tilt))
//        {
//            case MMA7660_ORIENTATION_LEFT:  orientation = "LEFT";  break;
//            case MMA7660_ORIENTATION_RIGHT: orientation = "RIGHT"; break;
//            case MMA7660_ORIENTATION_DOWN:  orientation = "DOWN";  break;
//            case MMA7660_ORIENTATION_UP:    orientation = "UP";    break;
//					  default:            							orientation = "?";     break;
//        }
//NRF_LOG_INFO("HEART_RATE: " NRF_LOG_FLOAT_MARKER "\t""SPO2_DATA: "NRF_LOG_FLOAT_MARKER"\t""TEMPURATURE: "NRF_LOG_FLOAT_MARKER"\n",p_sample->IR,p_sample->RED,p_sample->TEMP);
				  NRF_LOG_INFO("HEART_RATE : " NRF_LOG_FLOAT_MARKER "\t",p_sample->RED);
				  NRF_LOG_INFO("SPO2_DATA  : " NRF_LOG_FLOAT_MARKER "\t ",p_sample->IR);
				  NRF_LOG_INFO("TEMPURATURE: " NRF_LOG_FLOAT_MARKER "\n ",p_sample->TEMP);
				 //NRF_LOG_INFO("BIT RATE: " NRF_LOG_FLOAT_MARKER "\r\n ",
            //NRF_LOG_FLOAT((float)((m_sum.temp * 0.125) / NUMBER_OF_SAMPLES)));
//        NRF_LOG_INFO("BIT RATE: " NRF_LOG_FLOAT_MARKER " | X: %3d, Y: %3d, Z: %3d ",
//            NRF_LOG_FLOAT((float)((m_sum.temp * 0.125) / NUMBER_OF_SAMPLES)),
//            m_sum.x / NUMBER_OF_SAMPLES,
//            m_sum.y / NUMBER_OF_SAMPLES,
//            m_sum.z / NUMBER_OF_SAMPLES);

//        NRF_LOG_RAW_INFO("| %s%s%s\r\n",
//            (uint32_t)orientation,
//            (uint32_t)(MMA7660_TAP_DETECTED(tilt)   ? " TAP"   : ""),
//            (uint32_t)(MMA7660_SHAKE_DETECTED(tilt) ? " SHAKE" : ""));
//        m_prev_tilt = tilt;
    }
	}
static void read_all(void)
{
    // Signal on LED that something is going on.
    nrf_gpio_pin_toggle(READ_ALL_INDICATOR);

    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static app_twi_transfer_t const transfers[] =
    {
			    LM75B_READ(&lm75b_temp_reg_addr,  &m_buffer[0],4),
			    LM75B_READ(&lm75b_tos_reg_addr,&m_buffer[4],1),
          LM75B_READ(&lm75b_thyst_reg_addr,&m_buffer[5],1)
     			//LM75B_READ_TEMP(&m_buffer[0]),
     			//LM75B_READ_TEMP(&m_buffer[1]),
//			  LM75B_READ_TEMP(&m_buffer[2]),
//			  LM75B_READ_TEMP(&m_buffer[3])
      
 //       MMA7660_READ_XYZ_AND_TILT(&m_buffer[2])
    };
    static app_twi_transaction_t const transaction =
    {
        .callback            = read_all_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

   // APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        // On each RTC tick (their frequency is set in "nrf_drv_config.h")
        // we read data from our sensors.
        read_all();
			//read_lm75b_registers();
    }
}// RTC tick events generation.
static void rtc_config(void)
{
    uint32_t err_code;

    // Initialize RTC instance with default configuration.
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
	config.prescaler = RTC_FREQ_TO_PRESCALER(32); //Set RTC frequency to 32Hz
    err_code = nrf_drv_rtc_init(&m_rtc, &config, rtc_handler);
	//  err_code = nrf_drv_rtc_init(&m_rtc, &config, NULL);
    //APP_ERROR_CHECK(err_code);

     //Enable tick event and interrupt.
   // nrf_drv_rtc_tick_enable(&m_rtc, true);

    // Power on RTC instance.
    nrf_drv_rtc_enable(&m_rtc);
}
uint32_t sensorsim_measure(sensorsim_state_t     * p_state,
                           const sensorsim_cfg_t * p_cfg)
{
	uint16_t        heart_rate;
	uint8_t temp_hi = m_buffer[0];
    uint8_t temp_lo = m_buffer[1];

    if (p_state->is_increasing)
    {
        sensorsim_increment(p_state, p_cfg);
    }
    else
    {
        sensorsim_decrement(p_state, p_cfg);
    }
	//rtc_config();
	//nrf_drv_rtc_disable(&m_rtc);
    return p_state->current_val;
		//
	//return m_buffer[0];
}

static void heart_rate_meas_timeout_handler(void * p_context)
{
    static uint32_t cnt = 0;
    uint32_t        err_code;
    uint16_t        heart_rate;
	
	  uint8_t temp_hi = m_buffer[0];
    uint8_t temp_lo = m_buffer[1];

    UNUSED_PARAMETER(p_context);
    //rtc_config();
	  read_all();
    //heart_rate = (uint16_t)sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);
    heart_rate=LM75B_GET_TEMPERATURE_VALUE(temp_hi, temp_lo);
	//rtc_config();
    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

    // Disable RR Interval recording every third heart rate measurement.
    // NOTE: An application will normally not do this. It is done here just for testing generation
    // of messages without RR Interval measurements.
    m_rr_interval_enabled = ((cnt % 3) != 0);
}


/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void rr_interval_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_rr_interval_enabled)
    {
        uint16_t rr_interval;

        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }
}


/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sensor_contact_detected_timeout_handler(void * p_context)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(p_context);

    sensor_contact_detected = !sensor_contact_detected;
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_heart_rate_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                heart_rate_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_rr_interval_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                rr_interval_timeout_handler );
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sensor_contact_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_contact_detected_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
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
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_hrs_init_t hrs_init;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
    uint8_t        body_sensor_location;

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    m_heart_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    m_rr_interval_sim_cfg.start_at_max = false;

    sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_rr_interval_timer_id, RR_INTERVAL_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
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
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
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
            NRF_LOG_INFO("Fast Advertising\r\n");
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


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d\r\n",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST


        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
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
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(NRF_BLE_CENTRAL_LINK_COUNT,
                                                    NRF_BLE_PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(NRF_BLE_CENTRAL_LINK_COUNT, NRF_BLE_PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
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
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
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

    err_code = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
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


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}

/*GATT generic Event handler*/
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t * p_evt)
{
    ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}


/*GATT Module init*/
void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}
//////////////////////////////////////////////////////////////////////////////////
//// Reading of data from sensors - current temperature from LM75B and from
//// MMA7660: X, Y, Z and tilt status.
//#if (BUFFER_SIZE < 6)
//    #error Buffer too small.
//#endif
//#define GET_ACC_VALUE(axis, reg_data) \
//    do { \
//        if (MMA7660_DATA_IS_VALID(reg_data)) \
//        { \
//            axis = MMA7660_GET_ACC(reg_data); \
//        } \
//    } while (0)

//void read_all_cb(ret_code_t result, void * p_user_data)
//{
//    if (result != NRF_SUCCESS)
//    {
//        NRF_LOG_INFO("read_all_cb - error: %d\r\n", (int)result);
//        return;
//    }

//    sample_t * p_sample = &m_samples[m_sample_idx];
//    m_sum.temp -= p_sample->IR;
//    m_sum.x    -= p_sample->RED;
//    //m_sum.y    -= p_sample->y;
//    //m_sum.z    -= p_sample->z;

//    uint8_t temp_hi = m_buffer[0];
//    uint8_t temp_lo = m_buffer[1];
//    uint8_t x_out   = m_buffer[2];
//    uint8_t y_out   = m_buffer[3];
//    uint8_t z_out    = m_buffer[4];
//    uint8_t tilt    = m_buffer[5];

//    p_sample->IR = LM75B_GET_TEMPERATURE_VALUE(temp_hi, temp_lo);
//    p_sample->RED = LM75B_GET_TEMPERATURE_VALUE(x_out,y_out);
//    p_sample->TEMP = LM75B_GET_TEMPERATURE_VALUE_(z_out,tilt);
//		//GET_ACC_VALUE(p_sample->x, x_out);
//    //GET_ACC_VALUE(p_sample->y, y_out);
//    //GET_ACC_VALUE(p_sample->z, z_out);
//    if (!MMA7660_DATA_IS_VALID(tilt))
//    {
//        tilt = m_prev_tilt;
//    }

//    m_sum.temp += p_sample->IR;
//    m_sum.x    += p_sample->RED;
//    //m_sum.y    += p_sample->y;
//    //m_sum.z    += p_sample->z;

//    ++m_sample_idx;
//    if (m_sample_idx >= NUMBER_OF_SAMPLES)
//    {
//        m_sample_idx = 0;
//    }

//    // Show current average values every time sample index rolls over (for RTC
//    // ticking at 32 Hz and 16 samples it will be every 500 ms) or when tilt
//    // status changes.
//    if (m_sample_idx == 0 || (m_prev_tilt && m_prev_tilt != tilt))
//    {
//      //  char const * orientation;
////        switch (MMA7660_GET_ORIENTATION(tilt))
////        {
////            case MMA7660_ORIENTATION_LEFT:  orientation = "LEFT";  break;
////            case MMA7660_ORIENTATION_RIGHT: orientation = "RIGHT"; break;
////            case MMA7660_ORIENTATION_DOWN:  orientation = "DOWN";  break;
////            case MMA7660_ORIENTATION_UP:    orientation = "UP";    break;
////					  default:            							orientation = "?";     break;
////        }
////NRF_LOG_INFO("HEART_RATE: " NRF_LOG_FLOAT_MARKER "\t""SPO2_DATA: "NRF_LOG_FLOAT_MARKER"\t""TEMPURATURE: "NRF_LOG_FLOAT_MARKER"\n",p_sample->IR,p_sample->RED,p_sample->TEMP);
//				  NRF_LOG_INFO("HEART_RATE : " NRF_LOG_FLOAT_MARKER "\t",p_sample->RED);
//				  NRF_LOG_INFO("SPO2_DATA  : " NRF_LOG_FLOAT_MARKER "\t ",p_sample->IR);
//				  NRF_LOG_INFO("TEMPURATURE: " NRF_LOG_FLOAT_MARKER "\n ",p_sample->TEMP);
//				 //NRF_LOG_INFO("BIT RATE: " NRF_LOG_FLOAT_MARKER "\r\n ",
//            //NRF_LOG_FLOAT((float)((m_sum.temp * 0.125) / NUMBER_OF_SAMPLES)));
////        NRF_LOG_INFO("BIT RATE: " NRF_LOG_FLOAT_MARKER " | X: %3d, Y: %3d, Z: %3d ",
////            NRF_LOG_FLOAT((float)((m_sum.temp * 0.125) / NUMBER_OF_SAMPLES)),
////            m_sum.x / NUMBER_OF_SAMPLES,
////            m_sum.y / NUMBER_OF_SAMPLES,
////            m_sum.z / NUMBER_OF_SAMPLES);

////        NRF_LOG_RAW_INFO("| %s%s%s\r\n",
////            (uint32_t)orientation,
////            (uint32_t)(MMA7660_TAP_DETECTED(tilt)   ? " TAP"   : ""),
////            (uint32_t)(MMA7660_SHAKE_DETECTED(tilt) ? " SHAKE" : ""));
////        m_prev_tilt = tilt;
//    }
//	}
//static void read_all(void)
//{
//    // Signal on LED that something is going on.
//    nrf_gpio_pin_toggle(READ_ALL_INDICATOR);

//    // [these structures have to be "static" - they cannot be placed on stack
//    //  since the transaction is scheduled and these structures most likely
//    //  will be referred after this function returns]
//    static app_twi_transfer_t const transfers[] =
//    {
//			    LM75B_READ(&lm75b_temp_reg_addr,  m_buffer,4),
//			    LM75B_READ(&lm75b_tos_reg_addr,&m_buffer[4],1),
//          LM75B_READ(&lm75b_thyst_reg_addr,&m_buffer[5],1)
//     			//LM75B_READ_TEMP(&m_buffer[0]),
//     			//LM75B_READ_TEMP(&m_buffer[1]),
////			  LM75B_READ_TEMP(&m_buffer[2]),
////			  LM75B_READ_TEMP(&m_buffer[3])
//      
// //       MMA7660_READ_XYZ_AND_TILT(&m_buffer[2])
//    };
//    static app_twi_transaction_t const transaction =
//    {
//        .callback            = read_all_cb,
//        .p_user_data         = NULL,
//        .p_transfers         = transfers,
//        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
//    };

//    APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
//}
// TWI (with transaction manager) initialization.
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    //APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
		err_code = nrf_drv_twi_init(&m_twi,&config,NULL,NULL);
		
     APP_ERROR_CHECK(err_code);
		nrf_drv_twi_enable(&m_twi);
}

//void twi_init (void)
//{
//    ret_code_t err_code;

//    const nrf_drv_twi_config_t twi_si_7021_config = {
//       .scl                = ARDUINO_SCL_PIN,
//       .sda                = ARDUINO_SDA_PIN,
//       .frequency          = NRF_TWI_FREQ_400K,
//       .interrupt_priority = APP_IRQ_PRIORITY_LOW // was high
//    };

//    err_code = nrf_drv_twi_init(&m_twi_si_7021, &twi_si_7021_config, NULL, NULL);
//    APP_ERROR_CHECK(err_code);

//    nrf_drv_twi_enable(&m_twi_si_7021);
//}
//static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
//{
//    if (int_type == NRF_DRV_RTC_INT_TICK)
//    {
//        // On each RTC tick (their frequency is set in "nrf_drv_config.h")
//        // we read data from our sensors.
//        read_all();
//			//read_lm75b_registers();
//    }
//}// RTC tick events generation.
//static void rtc_config(void)
//{
//    uint32_t err_code;

//    // Initialize RTC instance with default configuration.
//    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
//    config.prescaler = RTC_FREQ_TO_PRESCALER(32); //Set RTC frequency to 32Hz
//    err_code = nrf_drv_rtc_init(&m_rtc, &config, rtc_handler);
//	//  err_code = nrf_drv_rtc_init(&m_rtc, &config, NULL);
//    APP_ERROR_CHECK(err_code);

//    // Enable tick event and interrupt.
//    nrf_drv_rtc_tick_enable(&m_rtc, true);

//    // Power on RTC instance.
//    nrf_drv_rtc_enable(&m_rtc);
//}

static void lfclk_config(void)
{
    uint32_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

//#define mx1_INIT_TRANSFER_COUNT 1
//// Set default configuration of LM75B - write 0 to Conf register.
//static uint8_t const mx1_config[] = { 0x06,0x0b};//0x0b
//app_twi_transfer_t const mx1_init_transfers[mx1_INIT_TRANSFER_COUNT] =
//{
//    APP_TWI_WRITE(0x57, mx1_config, sizeof(mx1_config), 0),
//			
//};

//static uint8_t const mx1_config_temp[] = { 0x06,1<<3};//0x0b
//app_twi_transfer_t const mx1_init_transfers_temp[mx1_INIT_TRANSFER_COUNT] =
//{
//    APP_TWI_WRITE(0x57, mx1_config, sizeof(mx1_config), 0),
//			
//};
//#define mx2_INIT_TRANSFER_COUNT 1
//// Set default configuration of LM75B - write 0 to Conf register.
//static uint8_t const mx2_config[] = { 0x07,0x03};//0x03
//app_twi_transfer_t const mx2_init_transfers[mx2_INIT_TRANSFER_COUNT] =
//{
//    APP_TWI_WRITE(0x57, mx2_config, sizeof(mx2_config), 0)
//			
//};
//#define mx3_INIT_TRANSFER_COUNT 1
//// Set default configuration of LM75B - write 0 to Conf register.
//static uint8_t const mx3_config[] = { 0x09,(0x08<<4)|0x0f};
//app_twi_transfer_t const mx3_init_transfers[mx3_INIT_TRANSFER_COUNT] =
//{
//    APP_TWI_WRITE(0x57, mx3_config, sizeof(mx3_config), 0)
//			
//};
/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
	  bool detected_device;
    bool     erase_bonds;
	
	  uint8_t const mode_config[] = { 0x06,0x0b};//0x0b
		uint8_t const spo2_config[] = { 0x07,0x03};//0x03
		uint8_t const led_config[] =  {0x09,(0x08<<4)|0x0f};
		uint8_t const reg_address=0x05;
	  uint8_t   const reg_temp_int_address=0x16;
		uint8_t const reg_temp_frac_address=0x17;
      // Initialize.
     err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    timers_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    peer_manager_init(erase_bonds);
    if (erase_bonds == true)
    {
        NRF_LOG_INFO("Bonds erased!\r\n");
    }
    gap_params_init();
    advertising_init();
    gatt_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
    twi_config();
		   nrf_drv_twi_tx(&m_twi,MAX30100_ADDRESS,mode_config,sizeof(mode_config),false);
    	 nrf_drv_twi_tx(&m_twi,MAX30100_ADDRESS,spo2_config,sizeof(spo2_config),false);
			 nrf_drv_twi_tx(&m_twi,MAX30100_ADDRESS,led_config,sizeof(led_config),false);
		// lfclk_config();
		
    // Start execution.
    NRF_LOG_INFO("Heart Rate Sensor Start!\r\n");
    application_timers_start();
    advertising_start();
		 
		for (;;)
    {
			   // nrf_delay_ms(1000);
			    err_code = nrf_drv_twi_tx(&m_twi,MAX30100_ADDRESS,&reg_address,1,false);
			   //nrf_delay_ms(10);
					//APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_twi_rx(&m_twi,MAX30100_ADDRESS,m_buffer,4);
			
			  err_code = nrf_drv_twi_tx(&m_twi,MAX30100_ADDRESS,&reg_temp_int_address,1,false);
			  err_code = nrf_drv_twi_rx(&m_twi,MAX30100_ADDRESS,&m_buffer[4],1);
			  err_code = nrf_drv_twi_tx(&m_twi,MAX30100_ADDRESS,&reg_temp_frac_address,1,false);
			  err_code = nrf_drv_twi_rx(&m_twi,MAX30100_ADDRESS,&m_buffer[5],1);
			  
			        uint16_t heart_rate=LM75B_GET_TEMPERATURE_VALUE(m_buffer[0], m_buffer[1]);
			        uint16_t spo2_value=LM75B_GET_TEMPERATURE_VALUE(m_buffer[2], m_buffer[3]);
			        uint16_t temp_value=LM75B_GET_TEMPERATURE_VALUE_(m_buffer[4], m_buffer[5]);
			         
			NRF_LOG_INFO("HEART_RATE : " NRF_LOG_FLOAT_MARKER "\t",heart_rate);
			  //nrf_delay_ms(1000);
 			NRF_LOG_INFO("SPO2_RATE : " NRF_LOG_FLOAT_MARKER "\t",spo2_value);
			 // nrf_delay_ms(1000);
			NRF_LOG_INFO("TEMP_RATE : " NRF_LOG_FLOAT_MARKER "\n",temp_value);
			   err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
			   //nrf_delay_ms(10);
					//APP_ERROR_CHECK(err_code);
//       if (err_code == NRF_SUCCESS)
//        {
//            detected_device = true;
//					  //nrf_delay_ms(1000);
//           // NRF_LOG_INFO("TWI device detected at address 0x%x\r\n",MAX30100_ADDRESS );
//					  
//					 //NRF_LOG_INFO("TWI device detected at address 0x%x.\r\n", m_buffer[1]);  
//        }
        NRF_LOG_FLUSH();
    }
		
		
		 // uint8_t device_add=0x57;
		 //# define DEVICE_ADD (device_add<<1)
	   
   
			// Initialize sensors.
//    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, lm75b_init_transfers,
//        LM75B_INIT_TRANSFER_COUNT, NULL));
//    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, mma7660_init_transfers,
//        MMA7660_INIT_TRANSFER_COUNT, NULL));
//     
//		   APP_ERROR_CHECK(app_twi_perform(&m_app_twi,mx1_init_transfers,
//         mx1_INIT_TRANSFER_COUNT, NULL));
//			  APP_ERROR_CHECK(app_twi_perform(&m_app_twi,mx2_init_transfers,
//       mx2_INIT_TRANSFER_COUNT, NULL));
//			  APP_ERROR_CHECK(app_twi_perform(&m_app_twi,mx3_init_transfers,
//       mx3_INIT_TRANSFER_COUNT, NULL));
       
			 
			 
			 //rtc_config();
    // Enter main loop.
//    while(1)
//    {
//			// __WFI();
////			APP_ERROR_CHECK(app_twi_perform(&m_app_twi,mx1_init_transfers_temp,
////         mx1_INIT_TRANSFER_COUNT, NULL));
//			//APP_ERROR_CHECK(app_twi_perform(&m_app_twi,mx1_init_transfers,
//        // mx1_INIT_TRANSFER_COUNT, NULL));
//			  //APP_ERROR_CHECK(app_twi_perform(&m_app_twi,mx2_init_transfers,
//       //mx2_INIT_TRANSFER_COUNT, NULL));
//			 // APP_ERROR_CHECK(app_twi_perform(&m_app_twi,mx3_init_transfers,
//       //mx3_INIT_TRANSFER_COUNT, NULL));
//			//read_lm75b_registers();
//		//	mx_rtc_handler();
//       //NRF_LOG_FLUSH();
////        if (NRF_LOG_PROCESS() == false)
////        {
////            power_manage();
////        }
//    }
}


