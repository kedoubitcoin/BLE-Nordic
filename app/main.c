/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "ble_dis.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_lesc.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_bas.h"
#include "app_error.h"
#include "nrf_drv_saadc.h"
#include "sdk_macros.h"
#include "app_timer.h"
#include "app_uart.h"
#include "nfc.h"
#include "my_board.h"
#include "app_scheduler.h"
#include "ble_dfu.h"
#include "nrf_delay.h"
#include "nrf_bootloader_info.h"
#include "nrf_drv_gpiote.h"
#include "nrf_power.h"

#define NAME_ADDR                       0x30000
#define NAMETK_LEN                      18
#define BLENAMELEN                      8

#define MANUFACTURER_NAME               "甯佷俊"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUMBER                    "one"                                     /**< Model Number string. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 0x55AA55AA55                                /**< DUMMY Manufacturer ID. Will be passed to Device Information Service. You shall use the ID for your Company*/
#define ORG_UNIQUE_ID                   0xEEBBEE                                    /**< DUMMY Organisation Unique ID. Will be passed to Device Information Service. You shall use the Organisation Unique ID relevant for your Company */
#define HW_REVISION                     "1.0.0"
#define FW_REVISION                     "s132_nrf52_7.0.1"
#define SW_REVISION                     "1.0.0"

#define APDU_TAG_BLE                    0x44

#define BLE_DEFAULT                     0
#define BLE_CONNECT                     1
#define BLE_DISCONNECT                  2
#define BLE_DIS_PIN                     3
#define BLE_PIN_ERR                     4
#define BLE_PIN_TIMEOUT                 5
#define BLE_PAIR_SUCCESS                6
#define BLE_PIN_CANCEL                  7
#define BLE_RCV_DATA                    8
#define BLE_SEND_I2C_DATA               9
#define BLE_READ_I2C_HEAD               10
#define BLE_READ_I2C_DATA               11


#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                40                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

	// SCHEDULER CONFIGS
#define SCHED_MAX_EVENT_DATA_SIZE       64             //!< Maximum size of the scheduler event data.
#define SCHED_QUEUE_SIZE                20                                          //!< Size of the scheduler queue.

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(3000)                      /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                          /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (10 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (100 ms) */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define ON_SECOND_INTERVAL              APP_TIMER_TICKS(1000)

#define RST_ONE_SECNOD_COUNTER()        one_second_counter = 0;
#define TWI_TIMEOUT_COUNTER             10

#define MAX_CONN_PARAM_UPDATE_COUNT     3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define LESC_DEBUG_MODE                 0                                           /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  1                                           /**< Man In The Middle protection required (applicable when display module is detected). */
#define SEC_PARAM_LESC                  1                                           /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_DISPLAY_ONLY                /**< Display I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define PASSKEY_LENGTH                  6                                           /**< Length of pass-key received by the stack for display. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  600  //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS 270  //!< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com.
#define ADC_RES_10BIT                  1024 //!< Maximum digital value for 10-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION   6    //!< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
BLE_BAS_DEF(m_bas);    
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */
APP_TIMER_DEF(m_1second_timer_id);

static uint8_t one_second_counter=0;
static uint8_t ble_evt_flag = BLE_DEFAULT;
static uint8_t mac_ascii[24];
static uint8_t mac[6]={0x42,0x13,0xc7,0x98,0x95,0x1a}; //Device MAC address
uint8_t ble_adv_name[BLENAMELEN+6] = "BixinKEY123456" ; 
static char DEVICE_NAME[20]="BixinKEY";

static nrf_saadc_value_t adc_buf[2];
static uint16_t          m_batt_lvl_in_milli_volts; //!< Current battery level.

static pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;
static uint16_t     m_conn_handle        = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static ble_uuid_t   m_adv_uuids[] =                                                 /**< Universally unique service identifiers. */
{
#if BLE_DIS_ENABLED
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
#endif		
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},    
		{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE}
};

static void advertising_start(void);


/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}                                             /**< Structure used to identify the battery service. */

static void saadc_event_handler(nrf_drv_saadc_evt_t const * p_evt)
{
    if (p_evt->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint8_t percentage_batt_level;
        uint32_t err_code;
		
        adc_result = p_evt->data.done.p_buffer[0];
        err_code = nrf_drv_saadc_buffer_convert(p_evt->data.done.p_buffer,1);
        APP_ERROR_CHECK(err_code);
        m_batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);

        percentage_batt_level = battery_level_in_percent(m_batt_lvl_in_milli_volts);
		
        err_code = ble_bas_battery_level_update(&m_bas,percentage_batt_level,BLE_CONN_HANDLE_ALL);
        if((err_code != NRF_SUCCESS)&&
            (err_code != NRF_ERROR_INVALID_STATE)&&
            (err_code != NRF_ERROR_RESOURCES)&&
            (err_code != NRF_ERROR_BUSY)&&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
          )
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}

void battery_level_meas_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
	
    ret_code_t err_code;
    err_code =  nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}

void one_second_timeout_hander(void * p_context)
{
    UNUSED_PARAMETER(p_context);

	one_second_counter++;
	if(one_second_counter >=20)
	{
        one_second_counter=0;
	}
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
void adc_configure(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  Battery Service structure.
 * @param[in] p_evt  Event received from the Battery Service.
 */
void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
            // Start battery timer
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_BAS_EVT_NOTIFICATION_ENABLED

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            break; // BLE_BAS_EVT_NOTIFICATION_DISABLED

        default:
            // No implementation needed.
            break;
    }
}

void bas_timer_init(void)
{
    ret_code_t err_code;
	
    err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

//battery service init
void sys_bas_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t     bas_init;
	
    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler          = on_bas_evt;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            pm_conn_sec_status_t conn_sec_status;
#if 0
            data_recived_buf[0] = APDU_TAG_BLE;
            data_recived_buf[1] = 0x00;
            data_recived_buf[2] = 0x3f;
            data_recived_buf[3] = BLE_CONNECT;
            i2c_master_write(data_recived_buf,64);		
            uint8_t count=60000;
            while(count--)
            {
                if(nrf_gpio_pin_read(TWI_STATUS_GPIO)==0)//can read
                {
                    i2c_master_read();
                    break;
                }
            }
#endif
            // Check if the link is authenticated (meaning at least MITM).
            err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
            APP_ERROR_CHECK(err_code);
            if (conn_sec_status.mitm_protected)
            {
                NRF_LOG_INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d",
                             ble_conn_state_role(p_evt->conn_handle),
                             p_evt->conn_handle,
                             p_evt->params.conn_sec_succeeded.procedure);
            }
            else
            {
                // The peer did not use MITM, disconnect.
                NRF_LOG_INFO("Collector did not use MITM, disconnecting");
                err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
				{
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        }   break; // PM_EVT_CONN_SEC_CONFIG_REQ
				
        case PM_EVT_CONN_SEC_FAILED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start();
            break;

        default:
            break;
    }
}

void mac_address_get(void)
{
	ble_gap_addr_t  Mac_address;
	unsigned char i,j=0;
	//_sd_ble_gap_addr_get(&Mac_address);

    uint32_t err_code = sd_ble_gap_addr_get(&Mac_address);
    APP_ERROR_CHECK(err_code);
     
    memcpy(mac,Mac_address.addr,6);
	for(i=0;i<6;i++)
	{
	if((mac[i]>>4)<0x0a)
	{
		mac_ascii[j]=0x30+(mac[i]>>4);
		j++;
	}
	else
	{
		mac_ascii[j]=0x31;
		j++;
		mac_ascii[j]=0x30+(mac[i]>>4)%0x0a;
		j++;
	}

	if((mac[i]&0x0f)<0x0a)
	{
			mac_ascii[j]=0x30+(mac[i]&0x0f);
			j++;
	}
	else
	{
		mac_ascii[j]=0x31;
		j++;
		mac_ascii[j]=0x30+(mac[i]&0x0f)%0x0a;
		j++;
	}
	}		

	memcpy(&DEVICE_NAME[8],mac_ascii,12);
}
void read_name_record(void)
{
	uint32_t i ;
	if (('U' == *(uint8_t *)NAME_ADDR)&&('2'==*((uint8_t *)NAME_ADDR+1)))
	{
		for(i=0;i<12;i++)
		{
			ble_adv_name[i] = *((uint8_t *)NAME_ADDR+i) ;
		}
		for(i=12;i<NAMETK_LEN;i++)
		{
			ble_adv_name[i-12] = *((uint8_t *)NAME_ADDR+i) ;
		}	
	}
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

	//Create 1s timer.
	err_code = app_timer_create(&m_1second_timer_id,
								APP_TIMER_MODE_REPEATED,
								one_second_timeout_hander);
    APP_ERROR_CHECK(err_code);
}

static ret_code_t check_twi_timeout(void)
{
    if(one_second_counter>=TWI_TIMEOUT_COUNTER)
    {
        RST_ONE_SECNOD_COUNTER();
		return false;
	}
	else
	{
        return true;
	}
}
/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
#if FIXED_PIN
	    //set fixed Passkey
    ble_opt_t ble_opt; 
    uint8_t g_ucBleTK[6] = "123456" ; 
    ble_opt.gap_opt.passkey.p_passkey = g_ucBleTK; 
#endif	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_GLUCOSE_METER);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
#if FIXED_PIN																					
    //set fixed Passkey													
    err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &ble_opt);	
    APP_ERROR_CHECK(err_code);	
#endif																					
}

static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
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
    APP_ERROR_HANDLER(nrf_error);
}

static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}


static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{    	
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
//        uint32_t err_code;
        uint32_t msg_len,data_len;
		
        //NRF_LOG_INFO("Received data from BLE NUS.");
        //NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        memcpy(data_recived_buf,p_evt->params.rx_data.p_data,p_evt->params.rx_data.length);
        data_recived_len=p_evt->params.rx_data.length;
		
        if(data_recived_len<9)
            return;
		
        msg_len=(uint32_t)((data_recived_buf[5] << 24) + 
                           (data_recived_buf[6] << 16) + 
                           (data_recived_buf[7] << 8) + 
                           (data_recived_buf[8]));
        if(0 == msg_len)
        {
            data_len = 9;
        }else
        {
            data_len=msg_len+9;
        }
        
        if(data_recived_buf[0] == '?' || data_recived_buf[1] == '#' || data_recived_buf[2] == '#')
        {
            if(data_len>=9)
            {                
                if(data_len == (msg_len +9))
                {        
                    data_recived_flag = false;                                        
                    ble_evt_flag = BLE_RCV_DATA;
                }	
            }	
        }
		
		#if 0
		//read
		uint32_t count=6000;
		uint8_t flag=0;
		while(1)
		{   
		    if(0 == flag)
		    {
	 		    if(0 == nrf_gpio_pin_read(TWI_STATUS_GPIO))//can read
	 		    {
	 		        i2c_master_read();
					flag = 1;
	 		    }	
		    }
			else
			{
			   if(true == data_recived_flag)
			   {
				   break;
			   }

			}
	       
        }			
	     #endif    	
#if 0
        //Send data
        do
        {
            uint16_t length = (uint16_t)data_recived_len;
            err_code = ble_nus_data_send(&m_nus, data_recived_buf, &length, m_conn_handle);
            if ((err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != NRF_ERROR_RESOURCES) &&
                (err_code != NRF_ERROR_NOT_FOUND))
            {
                APP_ERROR_CHECK(err_code);
            }
        } while (err_code == NRF_ERROR_RESOURCES);
#endif				
    }

}
/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Glucose, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t                err_code;
    ble_dis_init_t            dis_init;
    ble_nus_init_t            nus_init;
    nrf_ble_qwr_init_t        qwr_init = {0};
    ble_dfu_buttonless_init_t dfus_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
#ifdef BUTTONLESS_ENABLED
    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
#endif	
    // Initialize Battery Service.
    sys_bas_init();

#if BLE_DIS_ENABLED
    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, MODEL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str,HW_REVISION);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str,FW_REVISION);
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str,SW_REVISION);
		
    ble_dis_sys_id_t system_id;
    system_id.manufacturer_id            = MANUFACTURER_ID;
    system_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                    = &system_id;

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
#endif		
    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_1second_timer_id, ON_SECOND_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameter events.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail configuration parameter, but instead we use the
 *                event handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

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
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAM_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
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
    ret_code_t err_code;;
    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
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
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break; // BLE_ADV_EVT_FAST

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break; // BLE_ADV_EVT_IDLE

        default:
            break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    pm_handler_secure_on_connection(p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected");
			#if 0
			data_recived_buf[0] = APDU_TAG_BLE;
            data_recived_buf[1] = 0x00;
            data_recived_buf[2] = 0x3f;
            data_recived_buf[3] = BLE_DISCONNECT;
            i2c_master_write(data_recived_buf,64);	
			uint16_t count=60000;
			while(count--)
			{
				if(nrf_gpio_pin_read(TWI_STATUS_GPIO)==0)//can read
				{
					i2c_master_read();
					break;
				}
			}
			#endif
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            // Check if the last connected peer had not used MITM, if so, delete its bond information.
            if (m_peer_to_be_deleted != PM_PEER_ID_INVALID)
            {
                err_code = pm_peer_delete(m_peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_DEBUG("Collector's bond deleted");
                m_peer_to_be_deleted = PM_PEER_ID_INVALID;
            }
        } break;

        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected");
			ble_evt_flag = BLE_CONNECT;
			
            m_peer_to_be_deleted = PM_PEER_ID_INVALID;
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            // Start Security Request timer.
            
#if 0
		data_recived_buf[0] = APDU_TAG_BLE;
        data_recived_buf[1] = 0x00;
        data_recived_buf[2] = 0x3f;
        data_recived_buf[3] = BLE_CONNECT;
		i2c_master_write(data_recived_buf,64);	

		ble_evt_flag = 1;
#endif		
#if 0
		//read
		uint32_t count=60000000;
		uint8_t flag = 0;
		while(count--)
		{
		    if(0 == flag)
		    {
                if(nrf_gpio_pin_read(TWI_STATUS_GPIO)==0)//can read
				{
				    //nrf_delay_ms(1);
					i2c_master_read();	
					flag = 1;
				}
			}			
			if(true == data_recived_flag)
			{
				break;
			}
		}
#endif
        } break;

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

        case BLE_GAP_EVT_PASSKEY_DISPLAY:
        {
            char passkey[PASSKEY_LENGTH + 1];
            memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, PASSKEY_LENGTH);
			memcpy(&data_recived_buf[4],passkey,PASSKEY_LENGTH);
            passkey[PASSKEY_LENGTH] = 0;
            #if 0
			data_recived_buf[0] = APDU_TAG_BLE;
            data_recived_buf[1] = 0x00;
            data_recived_buf[2] = 0x3f;
            data_recived_buf[3] = BLE_DIS_PIN;
            i2c_master_write(data_recived_buf,64);		
		    uint8_t count=60000;
			while(count--)
			{
				if(nrf_gpio_pin_read(TWI_STATUS_GPIO)==0)//can read
				{
					i2c_master_read();
					break;
				}
			}
			#endif
            NRF_LOG_INFO("Passkey: %s", nrf_log_push(passkey));
        } break;
        
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

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

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

#if 0
/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}
#endif

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = false;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

//static void twi_write_data(void *p_event_data,uint16_t event_size)
static void twi_write_data(void)
{
    if(BLE_RCV_DATA == ble_evt_flag)
    {
        i2c_master_write(data_recived_buf,data_recived_len);
		ble_evt_flag = BLE_SEND_I2C_DATA;
		RST_ONE_SECNOD_COUNTER();
	}
}

//static void twi_read_data(void *p_event_data,uint16_t event_size)
static void twi_read_data(void)
{
	ret_code_t err_code;
	
#if 1	
    if(BLE_SEND_I2C_DATA == ble_evt_flag)
    {
        if(false == check_twi_timeout())
        {
            ble_evt_flag = BLE_DEFAULT;
			return;
		}
        //nrf_delay_ms(30);
        if(nrf_gpio_pin_read(TWI_STATUS_GPIO)==0)//can read
        {
            //nrf_delay_ms(30);
            i2c_master_read();			
            ble_evt_flag = BLE_READ_I2C_HEAD;
		    RST_ONE_SECNOD_COUNTER();
        }
    }
	
    if(BLE_READ_I2C_HEAD == ble_evt_flag)
    {
        if(false == check_twi_timeout())
        {
            ble_evt_flag = BLE_DEFAULT;
			return;
		}
        if(true == data_recived_flag)
        {
            ble_evt_flag = BLE_READ_I2C_DATA;
            //Send data
            do
            {
                uint16_t length = (uint16_t)data_recived_len;
                err_code = ble_nus_data_send(&m_nus, data_recived_buf, &length, m_conn_handle);
                if ((err_code != NRF_ERROR_INVALID_STATE) &&
                    (err_code != NRF_ERROR_RESOURCES) &&
                    (err_code != NRF_ERROR_NOT_FOUND))
                {
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_RESOURCES);
            ble_evt_flag = BLE_DEFAULT;
			RST_ONE_SECNOD_COUNTER();
        }
    }
  
#endif	
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

	  //app_sched_execute();
#if 1
    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);
    
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
#else   
    err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
#endif	
}

static void gpio_init(void)
{
    //Detect USB insert status.
    nrf_gpio_cfg_input(USB_INS_PIN,NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(TWI_STATUS_GPIO,NRF_GPIO_PIN_PULLUP);
}
static void system_init()
{ 
    gpio_init();
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

    APP_ERROR_CHECK(err_code);
}
#if 0
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}
#endif

static void main_loop(void)
{   
    //app_sched_event_put(NULL,NULL,twi_write_data);
    //app_sched_event_put(NULL,NULL,twi_read_data);
    twi_write_data();
    twi_read_data();
}
/**@brief Application main function.
 */
int main(void)
{    
#ifdef BUTTONLESS_ENABLED
	// Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    ret_code_t err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);
#endif
    // Initialize.
    system_init();
    //scheduler_init();
	
    //uart_init();
    log_init();
	//定时器初始化,按键消抖,系统定时用户创建定时任务等
    timers_init();
	//电源管理初始化
    power_management_init();
	//BLE协议栈初始化，使能softdevice,协议栈参数
	//注册BLE协议栈回调处理函数
    ble_stack_init();
    adc_configure();
    read_name_record();
    mac_address_get();
		peer_manager_init();
	//设备名称,连接安全模式,外观特征,ppcp默认连接参数
    gap_params_init();
	//连接参数更新和数据长度更细
    gatt_init();
    advertising_init();    
    services_init();    
	//连接参数更新初始化
    conn_params_init();	  
		
    application_timers_start();
    // Start execution.
    //printf("\r\nUART started.\r\n");
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
	//启动广播
    advertising_start();
		
    twi_master_init();
    nfc_init();

    // Enter main loop.
    for (;;)
    {
        main_loop();
        idle_state_handle();
    }
}

/**
 * @}
 */
