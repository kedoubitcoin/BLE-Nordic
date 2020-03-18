/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
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
 * @defgroup nfc_uart_tag_example_main main.c
 * @{
 * @ingroup nfc_uart_tag_example
 * @brief The application main file of NFC UART Tag example.
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "app_error.h"
#include "app_fifo.h"
#include "app_uart.h"
#include "boards.h"
#include "nrf_drv_twi.h"
#include "nfc_t4t_lib.h"
#include "sdk_config.h"

#include "nrf_log_default_backends.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"

#include "nfc.h"

#define MAX_APDU_LEN      0xF0   /**< Maximal APDU length, Adafruit limitation. */
//#define HEADER_FIELD_SIZE 1      /**< Header field size. */
#define HEADER_FIELD_SIZE 0				 /**< no header */	

#define NFC_RX_BUFF_SIZE 1024
#define NFC_TX_BUFF_SIZE 1024

//NFC RECEIVED
uint8_t nfc_data_buf[1024];
uint32_t nfc_data_len;

bool data_recived_flag=false;
uint8_t data_recived_buf[1024];
uint32_t data_recived_len=0;

static app_fifo_t m_nfc_rx_fifo; /**< FIFO instance for data that is received from NFC. */
static app_fifo_t m_nfc_tx_fifo; /**< FIFO instance for data that will be transmitted over NFC. */

static void apdu_command(void);

/**
 * @brief Callback function for handling UART errors.
 */
//void uart_error_handle(app_uart_evt_t * p_event)
//{
//    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
//    {
//        APP_ERROR_HANDLER(p_event->data.error_communication);
//    }
//    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
//    {
//        APP_ERROR_HANDLER(p_event->data.error_code);
//    }
//}


/**
 * @brief Function for flushing all FIFOs that are used by the application.
 */
static void fifos_flush(void)
{
    UNUSED_RETURN_VALUE(app_uart_flush());
    UNUSED_RETURN_VALUE(app_fifo_flush(&m_nfc_rx_fifo));
    UNUSED_RETURN_VALUE(app_fifo_flush(&m_nfc_tx_fifo));
}

//TWI driver
static volatile bool twi_xfer_done = false ;
static uint8_t twi_xfer_dir = 0; //0-write 1-read

/**
 * @brief TWI master instance.
 *
 * Instance of TWI master driver that will be used for communication with simulated
 * EEPROM memory.
 */
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);

static void twi_handler(nrf_drv_twi_evt_t const * p_event, void *p_context )
{
	static uint8_t read_state = READSTATE_IDLE;
	static uint32_t data_len =0;
	switch (p_event->type)
	{
		case NRF_DRV_TWI_EVT_DONE:
			twi_xfer_done=true;
			if(twi_xfer_dir==1)
			{
				if(read_state == READSTATE_IDLE)
				{
						if(data_recived_buf[0] == '?' && data_recived_buf[1] == '#' && data_recived_buf[2] == '#')
						{
							read_state = READSTATE_READ_INFO;
							data_recived_len= 3;
							nrf_drv_twi_rx(&m_twi_master,SLAVE_ADDR,data_recived_buf+data_recived_len,6);//read id+len bytes len
						}
				}
				else if(read_state == READSTATE_READ_INFO)
				{
					data_recived_len += 6;
					data_len = ((uint32_t)data_recived_buf[5] << 24) + (data_recived_buf[6] << 16) + (data_recived_buf[7] << 8) + data_recived_buf[8];
					if(data_len > 0)
					{
						read_state = READSTATE_READ_DATA;
						nrf_drv_twi_rx(&m_twi_master,SLAVE_ADDR,data_recived_buf+data_recived_len,data_len);//read id+len bytes len
					}							
					else
					{						
						data_recived_flag = true;
						read_state = READSTATE_IDLE;
					}
				}
				else if(read_state == READSTATE_READ_DATA)
				{
					data_recived_len += data_len;
					data_recived_flag = true;
					read_state = READSTATE_IDLE;
				}
			}
			break;
		case NRF_DRV_TWI_EVT_ADDRESS_NACK:
			break;
		case NRF_DRV_TWI_EVT_DATA_NACK:
			break;
		default:
			break;
	}
}

/**
 * @brief Initialize the master TWI.
 *
 * Function used to initialize the master TWI interface that would communicate with simulated EEPROM.
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
int twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    ret = nrf_drv_twi_init(&m_twi_master, &config, twi_handler, NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi_master);
    }

    return ret;
}

bool i2c_master_write(const uint8_t *buf,uint32_t len)
{
	ret_code_t err_code;
	
	twi_xfer_done = false;
	
	err_code = nrf_drv_twi_tx(&m_twi_master,SLAVE_ADDR,buf,len,false);
	twi_xfer_dir = 0;
	//while(twi_xfer_done == false);
	if(NRF_SUCCESS != err_code)
	{
		return false;
	}	
//	twi_xfer_dir = 1;
//	data_recived_len= 0;
//	nrf_drv_twi_rx(&m_twi_master,SLAVE_ADDR,data_recived_buf,3);//read 3 bytes header
	return true;
}

bool i2c_master_read(void)
{	
	uint32_t offset = 0;
	ret_code_t err_code;

	twi_xfer_dir = 1;
	data_recived_len= 0;
	
	err_code=nrf_drv_twi_rx(&m_twi_master,SLAVE_ADDR,data_recived_buf+offset,3);
	if(NRF_SUCCESS != err_code)
	{
		return false;
	}
//	if(data_recived_buf[0] != '?' || data_recived_buf[1] != '#' || data_recived_buf[2] != '#')
//	{
//		return false;
//	}	
//	offset += 3;
//	
//	err_code=nrf_drv_twi_rx(&m_twi_master,SLAVE_ADDR,data_recived_buf+offset,6);
//	if(NRF_SUCCESS != err_code)
//	{
//		return false;
//	}
//	offset += 6;
//	data_len = ((uint32_t)data_recived_buf[5] << 24) + (data_recived_buf[6] << 16) + (data_recived_buf[7] << 8) + data_recived_buf[8];
//	if(data_len == 0)
//	{
//		data_recived_len = offset;
//		data_recived_flag = true;
//	}
//	else
//	{
//		err_code=nrf_drv_twi_rx(&m_twi_master,SLAVE_ADDR,data_recived_buf+offset,data_len);
//		if(NRF_SUCCESS != err_code)
//		{
//			return false;
//		}
//		offset += data_len;
//		data_recived_len = offset;
//		data_recived_flag = true;
//		return true;
//	}
	return true;
}

/**
 * @brief Callback function for handling NFC events.
 */
static void nfc_callback(void          * context,
                         nfc_t4t_event_t event,
                         const uint8_t * data,
                         size_t          dataLength,
                         uint32_t        flags)
{
    ret_code_t err_code;
    uint32_t   resp_len;

    static uint8_t  apdu_buf[APDU_BUFF_SIZE]; // Buffer for APDU data
    static uint32_t apdu_len = 0;             // APDU length

    (void)context;

    switch (event)
    {
        case NFC_T4T_EVENT_FIELD_ON:
            NRF_LOG_INFO("NFC Tag has been selected. UART transmission can start...");
            //bsp_board_led_on(BSP_BOARD_LED_0);

            // Flush all FIFOs. Data that was collected from UART channel before selecting
            // the tag is discarded.
            fifos_flush();
            break;

        case NFC_T4T_EVENT_FIELD_OFF:
            NRF_LOG_INFO("NFC field lost. Data from UART will be discarded...");
            //bsp_board_leds_off();
            break;

        case NFC_T4T_EVENT_DATA_IND:
            if (apdu_len + dataLength > APDU_BUFF_SIZE)
            {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
            }
            memcpy(apdu_buf + apdu_len, data, dataLength);
            apdu_len += dataLength;

            if (flags != NFC_T4T_DI_FLAG_MORE)
            {
                // Store data in NFC RX FIFO if payload is present.
                if (apdu_len > HEADER_FIELD_SIZE)
                {
                    NRF_LOG_DEBUG("NFC RX data length: %d", apdu_len);
                    uint32_t buff_size;

                    apdu_len -= HEADER_FIELD_SIZE;
                    buff_size = apdu_len;
                    err_code  = app_fifo_write(&m_nfc_rx_fifo,
                                               apdu_buf + HEADER_FIELD_SIZE,
                                               &buff_size);
                    if ((buff_size != apdu_len) || (err_code == NRF_ERROR_NO_MEM))
                    {
                        NRF_LOG_WARNING("NFC RX FIFO buffer overflow");
                    }
                }
                apdu_len = 0;
								
                apdu_command();								
								
                // Check if there is any data in NFC TX FIFO that needs to be transmitted.
                resp_len = MIN(APDU_BUFF_SIZE - HEADER_FIELD_SIZE, MAX_APDU_LEN);
                if (app_fifo_read(&m_nfc_tx_fifo, apdu_buf + HEADER_FIELD_SIZE, &resp_len) ==
                    NRF_ERROR_NOT_FOUND)
                {
                    resp_len = 0;
                }

                if (resp_len > 0)
                {
                    NRF_LOG_DEBUG("NFC TX data length: %d", resp_len);
										// Send the response PDU over NFC.
										err_code = nfc_t4t_response_pdu_send(apdu_buf, resp_len + HEADER_FIELD_SIZE);
										APP_ERROR_CHECK(err_code);
                }                

                //bsp_board_led_off(BSP_BOARD_LED_1);
            }
            else
            {
                //bsp_board_led_on(BSP_BOARD_LED_1);
            }
            break;

        default:
            break;
    }
}


/**
 * @brief Function for initializing UART module.
 */
//static ret_code_t uart_init(void)
//{
//    ret_code_t err_code;

//    const app_uart_comm_params_t comm_params =
//    {
//        RX_PIN_NUMBER,
//        TX_PIN_NUMBER,
//        RTS_PIN_NUMBER,
//        CTS_PIN_NUMBER,
//        APP_UART_FLOW_CONTROL_DISABLED,
//        false,
//        UART_BAUDRATE_BAUDRATE_Baud115200
//    };

//    APP_UART_FIFO_INIT(&comm_params,
//                       UART_RX_BUFF_SIZE,
//                       UART_TX_BUFF_SIZE,
//                       uart_error_handle,
//                       APP_IRQ_PRIORITY_LOWEST,
//                       err_code);
//    return err_code;
//}


/**
 * @brief Function for initializing FIFO instances that are used to exchange data over NFC.
 */
static ret_code_t nfc_fifos_init(void)
{
    ret_code_t err_code;

    static uint8_t nfc_rx_buff[NFC_RX_BUFF_SIZE]; // Buffer for NFC RX FIFO instance
    static uint8_t nfc_tx_buff[NFC_TX_BUFF_SIZE]; // Buffer for NFC TX FIFO instance

    err_code = app_fifo_init(&m_nfc_rx_fifo, nfc_rx_buff, sizeof(nfc_rx_buff));
    VERIFY_SUCCESS(err_code);

    err_code = app_fifo_init(&m_nfc_tx_fifo, nfc_tx_buff, sizeof(nfc_tx_buff));
    return err_code;
}

/**
 *@brief Function for initializing logging.
 */
//static void log_init(void)
//{
//    ret_code_t err_code = NRF_LOG_INIT(NULL);
//    APP_ERROR_CHECK(err_code);

//    NRF_LOG_DEFAULT_BACKENDS_INIT();
//}


/**
 * @brief Function for application main entry.
 */
int nfc_init(void)
{
    ret_code_t err_code;
	

    /* Set up UART */
//    err_code = uart_init();
//    APP_ERROR_CHECK(err_code);

    /* Set up FIFOs for data that are transfered over NFC */
    err_code = nfc_fifos_init();
    APP_ERROR_CHECK(err_code);

    /* Set up NFC */
    err_code = nfc_t4t_setup(nfc_callback, NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("NFC UART Tag example started.");

    /* Start sensing NFC field */
    err_code = nfc_t4t_emulation_start();
    APP_ERROR_CHECK(err_code);
		return 0;

}

static void apdu_command(void)
{
		uint32_t msg_len,data_len;
		static bool reading=false;
		
		data_len=sizeof(nfc_data_buf);
		app_fifo_read(&m_nfc_rx_fifo,nfc_data_buf,&data_len);

		if(nfc_data_buf[0] == '?' || nfc_data_buf[1] == '#' || nfc_data_buf[2] == '#')
		{
			if(data_len>=9)
			{
				msg_len=(uint32_t)(nfc_data_buf[5] << 24) + (nfc_data_buf[6] << 16) + (nfc_data_buf[7] << 8) + nfc_data_buf[8];
				if(data_len == (msg_len +9))
				{
					data_recived_flag = false;
					//usart
					//app_uart_send(nfc_data_buf, data_len);
					//i2c
					i2c_master_write(nfc_data_buf,data_len);
					data_len = 3;
					app_fifo_flush(&m_nfc_tx_fifo);		
					app_fifo_write(&m_nfc_tx_fifo,"#**",&data_len);	
				}	
			}		
		}

		else if(nfc_data_buf[0] == '#' || nfc_data_buf[1] == '*' || nfc_data_buf[2] == '*')
		{
				//usart
			if(reading==false)
			{
				if(nrf_gpio_pin_read(TWI_STATUS_GPIO)==0)//can read
				{
					i2c_master_read();
					reading = true;
				}
				data_len = 3;
				app_fifo_flush(&m_nfc_tx_fifo);		
				app_fifo_write(&m_nfc_tx_fifo,"#**",&data_len);
				
			}
			else 
			{
				if(data_recived_flag == false)
				{
					data_len = 3;
					app_fifo_flush(&m_nfc_tx_fifo);		
					app_fifo_write(&m_nfc_tx_fifo,"#**",&data_len);
				}
				else
				{
					data_recived_flag = false;
					reading = false;
					app_fifo_flush(&m_nfc_tx_fifo);
					app_fifo_write(&m_nfc_tx_fifo,data_recived_buf,&data_recived_len);
				}
			}
		}
		else
		{
			data_len = 2;
			app_fifo_flush(&m_nfc_tx_fifo);		
			app_fifo_write(&m_nfc_tx_fifo,"\x6D\x00",&data_len);
		}
		
}
/** @} */
