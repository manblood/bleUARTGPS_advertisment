/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "nrf_pwr_mgmt.h"

#include "app_uart.h"
#include "app_timer.h"

#include "nrf_delay.h"
#include "nrf.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_error.h"

#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "ble_conn_state.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "ble_nus.h"
#include "ble_advertising.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          30                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */
//#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
//                                        0x45, 0x56, 0x67, 0x78, \
//                                        0x89, 0x9a, 0xab, 0xbc, \
//                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define APP_BEACON_UUID                 0x01, 0x12						/**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define DEVICE_NAME                     "COMOTRIAL"                       /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

static ble_gap_adv_params_t m_adv_params; /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffer for storing an encoded advertising set. */
static ble_advdata_t advdata;

static void advertisement_update(void);
static void advertising_start(void);

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] = /**< Information advertised by the Beacon. */
{
APP_DEVICE_TYPE, // Manufacturer specific information. Specifies the device type in this
				 // implementation.
		APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
							 // manufacturer specific data in this implementation.
		APP_BEACON_UUID,     // 4 bit UUID value.
		APP_MAJOR_VALUE, // Major arbitrary value that can be used to distinguish between Beacons.
		APP_MINOR_VALUE, // Minor arbitrary value that can be used to distinguish between Beacons.
		APP_MEASURED_RSSI // Manufacturer specific information. The Beacon's measured TX power in                      // this implementation.
		};

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data = { .adv_data = { .p_data = m_enc_advdata,
		.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX }, .scan_rsp_data = { .p_data =
		NULL, .len = 0

} };

#define PRINTF_USES_UART
#ifdef PRINTF_USES_UART
int _write(int file, char *ptr, int len) {

	int i = 0;
	uint8_t cr;
	for (i = 0; i < len; i++) {
		cr = *ptr++;
		while (app_uart_put(cr) != NRF_SUCCESS)
			;
	}
	return len;
}
#endif

void setData();

void uart_error_handle(app_uart_evt_t *p_event) {
	if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR) {
		APP_ERROR_HANDLER(p_event->data.error_communication);
	} else if (p_event->evt_type == APP_UART_FIFO_ERROR) {
		APP_ERROR_HANDLER(p_event->data.error_code);
	}
}

long strtolong(uint8_t *str){
	uint8_t nums[6];
	for(int i = 0; i < 7; i++){
		nums[i]= str[i] - 48;
	}

	return nums[5] + (nums[4] *10) + (nums[3] *100) + (nums[2] *1000) + (nums[1] *10000) + (nums[0] *100000);
}

void uart_event_handle(app_uart_evt_t *p_event) {
	static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
	static uint8_t index = 0;
	uint32_t err_code;

	switch (p_event->evt_type) {
	case APP_UART_DATA_READY:
		UNUSED_VARIABLE(app_uart_get(&data_array[index]));
		index++;

		if ((data_array[index - 1] == '\n') || (data_array[index - 1] == '\r')
				|| index > 15) {
		//	printf("data ready\r\n");

			if (index >= 14) {
				//printf("patates\r\n");
				data_array[index + 1] = 0;

				//is data numeric
				uint8_t isNumeric = 1;
				for (int i = 0; i < 14; i++) {
					printf("no %d\r\n", data_array[i]);
					if (!((data_array[i] >= 48 && data_array[i] <= 57) || data_array[i] == '-' || data_array[i] == '+')) {
						isNumeric = 0;
						printf("errn");
					}

				}
				char temp[7];
				temp[6] = 0;
				if (isNumeric == 1) {
					printf("numeric\r\n");
					memcpy(temp, data_array + 1, 6);
					//long x = strtol(temp, NULL, 16);
					long x = strtolong(temp);
					memcpy(temp, data_array + 8, 6);

					//long y = strtol(temp, NULL, 16);
					long y = strtolong(temp);
					printf("elma4\r\n");

					if(data_array[0]== '-')
						x = -1 * x;
					if(data_array[7] == '-')
						y = -1 * y;

					printf("x: %x  y: %x\r\n", x, y);

					if (x <= 180000l && x >= -180000 && y <= 180000l && y >= -180000) { //check coorsinates are good
						printf("portakal\r\n");

						uint8_t *px = &x;
						uint8_t *py = &y;
						for (int i = 0; i < 4; i++) {
							m_beacon_info[9 + i] = *(px + i);
							m_beacon_info[13 + i] = *(py + i);
						}
						//printf("m: %d\r\n",m_beacon_info[9]);

						err_code = sd_ble_gap_adv_stop(m_adv_handle);
						APP_ERROR_CHECK(err_code);

						err_code = ble_advdata_encode(&advdata,
								m_adv_data.adv_data.p_data,
								&m_adv_data.adv_data.len);
						APP_ERROR_CHECK(err_code);

						err_code = sd_ble_gap_adv_set_configure(&m_adv_handle,
								&m_adv_data, &m_adv_params);
						APP_ERROR_CHECK(err_code);
						printf("updated\r\n");

						advertising_start();

					}
				}
			}

			index = 0;
			printf("reset\r\n");
		}
		break;

	case APP_UART_COMMUNICATION_ERROR:
		APP_ERROR_HANDLER(p_event->data.error_communication);
		break;

	case APP_UART_FIFO_ERROR:
		APP_ERROR_HANDLER(p_event->data.error_code);
		break;

	default:
		break;
	}
}
/**@snippet [Handling the data received over UART] */

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void) {

	uint32_t err_code;
	uint8_t flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

	ble_advdata_manuf_data_t manuf_specific_data;
	//ble_advdata_t advdata;

	manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
	manuf_specific_data.data.p_data = (uint8_t*) m_beacon_info;
	manuf_specific_data.data.size = 9 + 7;

	// Build and set advertising data.
	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type = BLE_ADVDATA_FULL_NAME; //BLE_ADVDATA_NO_NAME;//BLE_ADVDATA_FULL_NAME; //BLE_ADVDATA_NO_NAME;
	//advdata.flags                 = flags;
	advdata.short_name_len = 6;
	advdata.p_manuf_specific_data = &manuf_specific_data;

	// Initialize advertising parameters (used when starting advertising).
	memset(&m_adv_params, 0, sizeof(m_adv_params));

	m_adv_params.properties.type =
			BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
	m_adv_params.p_peer_addr = NULL;    // Undirected advertisement.
	m_adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
	m_adv_params.interval = NON_CONNECTABLE_ADV_INTERVAL;
	m_adv_params.duration = 0;       // Never time out.

	memset(m_beacon_info+9,0,8);
//    m_enc_advdata[0] = 0x02;
//    m_enc_advdata[1] = BLE_GAP_AD_TYPE_FLAGS;
//    m_enc_advdata[2] = 0x11;
//    m_enc_advdata[3] = 0x22;
//    m_enc_advdata[4] = 0x33;
//	m_beacon_info[10] = 0;
//	m_beacon_info[11] = 0;
//	m_beacon_info[12] = 'A';
//	m_beacon_info[13] = 'A';
//	m_beacon_info[14] = 'A';
//	m_beacon_info[15] = 'A';
//	m_beacon_info[16] = 'A';

	err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data,
			&m_adv_data.adv_data.len);
	APP_ERROR_CHECK(err_code);
	printf("elma\r\n");

	err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data,
			&m_adv_params);
	APP_ERROR_CHECK(err_code);
	printf("armut\r\n");
}

//void setData(){
//	//m_beacon_info[0] =
//    uint32_t      err_code;
//	memcpy(m_beacon_info, "ERKANERSOY",10);
//
//	  //err_code = ble_advdata_set(&advdata, 0);
//	  APP_ERROR_CHECK(err_code);
//}

/**@brief Function for starting advertising.
 */
static void advertising_start(void) {
	ret_code_t err_code;

	err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
	APP_ERROR_CHECK(err_code);

	err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
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
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void) {
	ret_code_t err_code;
	ble_gap_conn_params_t gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
			(const uint8_t*) DEVICE_NAME, strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	/*err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
	 APP_ERROR_CHECK(err_code); */

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing logging. */
static void log_init(void) {
	ret_code_t err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void) {
	ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing timers. */
static void timers_init(void) {
	ret_code_t err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
	ret_code_t err_code;
	err_code = nrf_pwr_mgmt_init();
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {

	if (NRF_LOG_PROCESS() == false) {
		nrf_pwr_mgmt_run();
	}
}

#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

/**
 * @brief Function for application main entry.
 */
int main(void) {
	uint32_t err_code;

	// Initialize.
	log_init();
	timers_init();
	leds_init();
	power_management_init();
	uint8_t b_single = 0;

	const app_uart_comm_params_t comm_params = { RX_PIN_NUMBER, TX_PIN_NUMBER,
			RTS_PIN_NUMBER, CTS_PIN_NUMBER,
			UART_HWFC, false,
#if defined (UART_PRESENT)
    	          NRF_UART_BAUDRATE_115200
    	#else
			NRF_UARTE_BAUDRATE_115200
#endif
			};

	APP_UART_FIFO_INIT(&comm_params,
	UART_RX_BUF_SIZE,
	UART_TX_BUF_SIZE, uart_event_handle, APP_IRQ_PRIORITY_LOWEST, err_code);

	APP_ERROR_CHECK(err_code);

	ble_stack_init();

	gap_params_init();
	advertising_init();

	// Start execution.
	NRF_LOG_INFO("Beacon example started.");
	printf("Beacon started\r\n");
	uint8_t cr;

	advertising_start();

	// Enter main loop.
	for (;;) {

		idle_state_handle();

	}
}

/**
 * @}
 */
