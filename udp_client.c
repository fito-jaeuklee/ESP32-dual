/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include <esp_heap_caps.h>

#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_pm.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "addr_from_stdin.h"

#define ESP_COMMU_UART UART_NUM_1
#define EX_UART_NUM UART_NUM_0
#define UART_BUF_SIZE (1024)
#define RD_BUF_SIZE (UART_BUF_SIZE)
static QueueHandle_t uart0_queue;
static uint8_t ble_hr_share_val;
static int offset = 0;

#define GATTC_TAG "JEFF:GATTC_DEMO"
#define __ESP_FILE__ "nvs-manage"
//jeff : heart rate service 
//#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_SERVICE_UUID   ESP_GATT_UUID_HEART_RATE_SVC

//jeff: heart rate charateristic 
#define REMOTE_NOTIFY_CHAR_UUID    0x2A37 //0xFF01


#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

#define ECHO_TEST_TXD  (GPIO_NUM_17)
#define ECHO_TEST_RXD  (GPIO_NUM_16)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

static char udp_server_ip[16];
static char wifi_client_ip[16];
static char wifi_ssid[16];
static char remote_device_name[24];
static int wifi_connection_flag = 0;

static const char *ESP_ACK_OK = "ESP32OK\n";
static const char *ESP_CONFIG_START = "START";
static const char *ESP_CONFIG_START_OK_STM = "OK";
static const char *ESP_WIFI_DISCONNECTED = "FAIL\n";
static const char *ESP_WIFI_CONNECTED = "CONNECTED\n";

static bool ble_connect    = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

typedef struct {
    int messageSyncByte;
    int messageSyncByte2;
    int messageBodySize;
    int messageCommand;
    int messageBody[128];
    int checkSum;
} tSerialMessageFormat; //Serial Data Format
tSerialMessageFormat mcuToHost;
tSerialMessageFormat hostToMcu;


esp_err_t nvs_write_string(const char *key, const char *value, nvs_handle nvs_handle) {
  nvs_set_str(nvs_handle, key, value);
  nvs_commit(nvs_handle);
  ESP_LOGI(__ESP_FILE__, "NVS written) Key: %s / Value: %s", key, value);
  return ESP_OK;
}

esp_err_t nvs_write_u8(const char *key, uint8_t value, nvs_handle nvs_handle){
  nvs_set_u8(nvs_handle, key, value);
  nvs_commit(nvs_handle);
  ESP_LOGI(__ESP_FILE__, "NVS written) Key: %s / Value: %d", key, value);
  return ESP_OK;
}

esp_err_t nvs_write_i64(const char *key, uint8_t value, nvs_handle nvs_handle){
  nvs_set_i64(nvs_handle, key, value);
  nvs_commit(nvs_handle);
  ESP_LOGI(__ESP_FILE__, "NVS written) Key: %s / Value: %d", key, value);
  return ESP_OK;
}

char *nvs_read_str(char *key, nvs_handle nvs_handle) {
  size_t required_size;
  nvs_get_str(nvs_handle, key, NULL, &required_size);
  char *value_str = malloc(required_size);
  nvs_get_str(nvs_handle, key, value_str, &required_size);
  ESP_LOGI(__ESP_FILE__, "NVS read) Key: %s / Value: %s", key, value_str);
  return value_str;
}

uint8_t nvs_read_u8(char *key, nvs_handle nvs_handle) {
  uint8_t value_u8 = 0;
  nvs_get_u8(nvs_handle, key, &value_u8);
  return value_u8;
}

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success");
        break;
// jeff :  discovery service 		
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "discover service complete conn_id %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;


   case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "service found");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(GATTC_TAG, "Get service information from remote device");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(GATTC_TAG, "Get service information from flash");
        } else {
            ESP_LOGI(GATTC_TAG, "unknown service source");
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
         break;
		 
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }else{
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    }
                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     descr_elem_result[0].handle,
                                                                     sizeof(notify_en),
                                                                     (uint8_t *)&notify_en,
                                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                                     ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(GATTC_TAG, "decsr not found");
            }

        }
        break;
    }
//jeff: notify data.	
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }else{
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
	// heart rate value .	notify.value_len= 2, 00 48, 
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        for (int i = 0; i < p_data->notify.value_len; i++) 
        {
            ESP_LOGI(GATTC_TAG, "HR: %d %x:", i, p_data->notify.value[i]);
        }
        ble_hr_share_val = p_data->notify.value[1];

        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success ");
        uint8_t write_char_data[35];
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = i % 256;
        }
        esp_ble_gattc_write_char( gattc_if,
                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                  sizeof(write_char_data),
                                  write_char_data,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(GATTC_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write char success ");
        break;
//jeff :  disconnect 		
    case ESP_GATTC_DISCONNECT_EVT:
        ble_connect = false;
        get_server = false;
	    uint32_t duration = 3600;
        esp_ble_gap_start_scanning(duration);

        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    // char remote_device_name[24];
    // // char wifi_ssid[16];

    // nvs_handle nvs_handle_ble;
    // ESP_ERROR_CHECK(nvs_open("WIFI_BLE_CONFIG", NVS_READWRITE, &nvs_handle_ble));

    // strcat(remote_device_name, nvs_read_str("ble_device_name", nvs_handle_ble));
    
    // nvs_close(nvs_handle_ble);


    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        ESP_LOGI(GATTC_TAG, "BLE reconnect here");
        uint32_t duration = 3600;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "scan start success");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
           //  esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);

            //ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);


	    adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
	   if(scan_result->scan_rst.adv_data_len == 29)
	    {
	     	// ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
	    	// ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len); 
	   }
			
            // esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);

#if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP
            if (scan_result->scan_rst.adv_data_len > 0) {
                ESP_LOGI(GATTC_TAG, "adv data:");
                esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
            }
            if (scan_result->scan_rst.scan_rsp_len > 0) {
                ESP_LOGI(GATTC_TAG, "scan resp:");
                esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
            }
#endif

            // ESP_LOGI(GATTC_TAG, "waiting reconnect ble device \n");
            // vTaskDelay(1000/portTICK_PERIOD_MS);
if(scan_result->scan_rst.adv_data_len == 29)
{

	    // ESP_LOGI(GATTC_TAG, "jeff len %d, %d\r\n:", strlen(remote_device_name), adv_name_len);
	    // ESP_LOGI(GATTC_TAG, "jeff name %s, %s\r\n:", remote_device_name, (char *)adv_name);
}
            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    // ESP_LOGI(GATTC_TAG, "searched device %s\n", remote_device_name);
		    ESP_LOGI(GATTC_TAG, "================  find  ===============\n");
					
                    if (ble_connect == false) {
                        ble_connect = true;
                        ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop scan successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        printf("Check WDT!\n");
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}


#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "example";
// static const char *payload = "Message from ESP32 ";


#define EXAMPLE_ESP_WIFI_SSID      "FITO2"
#define EXAMPLE_ESP_WIFI_PASS      "fito2021"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5000

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// static const char *TAG = "wifi station";

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            wifi_connection_flag = 0;
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
            
        } else {
            ESP_LOGE(TAG, "--- WIFI reconnection timeover ---");
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            wifi_connection_flag = 0;
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connection_flag = 1;

        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        // if s_retry_num 
        // udp_msg_sent();

        s_retry_num = 0;
        
    }
}

void wifi_init_sta(void)
{
    // char wifi_client_ip[16];
    // char wifi_ssid[16];

    // nvs_handle nvs_handle_wifi;
    // ESP_ERROR_CHECK(nvs_open("WIFI_BLE_CONFIG", NVS_READWRITE, &nvs_handle_wifi));

    // strcat(wifi_client_ip, nvs_read_str("wifi_client", nvs_handle_wifi));
    // strcat(wifi_ssid, nvs_read_str("wifi_ssid", nvs_handle_wifi));

    // nvs_close(nvs_handle_wifi);

    printf("!!!!!!!!!!!! %s !!!!!!!!!!!!!!!! %s !!!!!!!!!!!!!\n", wifi_client_ip, wifi_ssid);
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // esp_netif_create_default_wifi_sta();

    esp_netif_t * sta = esp_netif_create_default_wifi_sta();
    esp_netif_dhcpc_stop(sta);
    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr = ipaddr_addr(wifi_client_ip);
    ip_info.gw.addr = ipaddr_addr("192.168.0.1");
    ip_info.netmask.addr = ipaddr_addr("255.255.255.0");
    esp_netif_set_ip_info(sta, & ip_info);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            // .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .listen_interval = 3,

            // .pmf_cfg = {
            //     .capable = true,
            //     .required = false
            // },
        },
    };
    strcpy((char *)wifi_config.sta.ssid, (char *)wifi_ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "----------WiFi power save mode start-----------\n");
    esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 wifi_ssid, EXAMPLE_ESP_WIFI_PASS);
        
        // udp_msg_sent();
        wifi_connection_flag = 1;
        
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_ssid, EXAMPLE_ESP_WIFI_PASS);
        wifi_connection_flag = 0;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    // vEventGroupDelete(s_wifi_event_group);
    // esp_err_t ret_wifi = esp_wifi_get_ps(&WIFI_PS_MIN_MODEM);
    // printf("@@@@@@@@@@@@@@@@@@ %s @@@@@@@@@@@@@@@@@@@@\n", esp_err_to_name(ret_wifi));

}

static void wifi_connect_manager(void *pvParameters)
{
    wifi_init_sta();
    vTaskDelete(NULL);
}


static void udp_msg_sent(char *server_ip, char *payload)
{
    char rx_buffer[128];
    char host_ip = (char *)server_ip;
    int addr_family = 0;
    int ip_protocol = 0;
    int sendcnt = 0;

#if defined(CONFIG_EXAMPLE_IPV4)
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(server_ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
    struct sockaddr_in6 dest_addr = { 0 };
    inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(PORT);
    dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
    struct sockaddr_in6 dest_addr = { 0 };
    ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
#endif

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

    

    int err = sendto(sock, payload, 240, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }
    ESP_LOGI(TAG, "Message sent %d", sendcnt);
    sendcnt++;

    // struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
    // socklen_t socklen = sizeof(source_addr);
    // int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

    // // Error occurred during receiving
    // if (len < 0) {
    //     ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
    //     break;
    // }
    // // Data received
    // else {
    //     rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
    //     ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
    //     ESP_LOGI(TAG, "%s", rx_buffer);
    //     if (strncmp(rx_buffer, "OK: ", 4) == 0) {
    //         ESP_LOGI(TAG, "Received expected message, reconnecting");
    //         break;
    //     }
    // }

    // vTaskDelay(2000 / portTICK_PERIOD_MS);


    if (sock != -1) {
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    // vTaskDelete(NULL);
}

static void ble_main(void)
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

 
}

static void ble_hr_task(void *pvParameters)
{
    ble_main();
    vTaskDelete(NULL);
}

int hostUartParser(unsigned int u8KeyIn)    //Host to Front Parser(Uart Parser)
{
    static  int State = 0;
    static  int* pDst;
    static  int BufCnt;
    //uint8_t   u8KeyIn;
    int ParseOK = 0;
    //if (VCP_Ready() == 0)
    //  return  0;
    ParseOK = 0;
    printf("in data 0x%x\r\n", u8KeyIn);
    //u8KeyIn = VCP_getch();
    switch (State)
    {
    case 0:
        if (u8KeyIn == 0xAC)
        {
            State = 1;  //Sync. Byte(Host --> RF4CE Controller)
            hostToMcu.checkSum = u8KeyIn;
        }
        break;
    case 1:
        if (u8KeyIn == 0xC0)
        {
            hostToMcu.messageSyncByte2 = u8KeyIn;
            hostToMcu.checkSum ^= u8KeyIn;
            // Header Parser
            State = 2;
        }
        else
        {
            State = 0;
        }
        break;
    case 2:
        printf("case2\r\n");
        hostToMcu.messageBodySize = u8KeyIn;
        if (hostToMcu.messageBodySize > 128)
        {
            State = 0;
            break;
        }
        hostToMcu.checkSum ^= u8KeyIn;
        pDst = (int*)&hostToMcu.messageCommand;
        BufCnt = 0;
        State = 3;
        break;
	case 3:
		printf("case3\r\n");
		*pDst = u8KeyIn;
		pDst++;
		hostToMcu.checkSum ^= u8KeyIn;
		BufCnt++;
		if (BufCnt >= hostToMcu.messageBodySize)
		{
			State = 4;
		}
		break;
    case 4:
        printf("case4, checksum 0x%x, datain 0x%x\r\n", hostToMcu.checkSum, u8KeyIn);
        if (u8KeyIn == hostToMcu.checkSum)
        {
            printf("parser ok ..............\r\n");
            ParseOK = 1;
        }
        else
        {
            printf("parser failed\r\n");
        }
        State = 0;
        break;
    default:
        State = 0;
        break;
    }
    return  ParseOK;
}


static void udp_client_task(void *pvParameters)
{
    int reset_flag = 1;
    int nvs_config_data_change_flag = 0;
    int config_start_flag = 0;
    // sprintf(ble_hr_share_val, "%s", "0");

    nvs_handle nvs_handle;
    
    ESP_ERROR_CHECK(nvs_open("WIFI_BLE_CONFIG", NVS_READWRITE, &nvs_handle));

    int receive_ok_from_mcu = 0;
    int data_integ_cnt = 0;
    
    int wifi_ble_config_processing_done_flag = 0;
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *) malloc(RD_BUF_SIZE);
    uint8_t *gps_hr_chunck_data = (uint8_t *) malloc(256);
    uint8_t *hr_data_to_mcu = (uint8_t *) malloc(4);
    memset(gps_hr_chunck_data, 0, 256);
    memset(hr_data_to_mcu, 0, 4);

    // char udp_server_ip[16];
    // char wifi_client_ip[16];
    // char wifi_ssid[16];
    // char remote_device_name[24];

    // printf("%d\n", nvs_read_u8("is_written", nvs_handle));
    // nvs_write_u8("is_written", 1, nvs_handle);
    // printf("%d\n", nvs_read_u8("is_written", nvs_handle));
    // nvs_write_u8("is_written", 0, nvs_handle);
    // printf("%d\n", nvs_read_u8("is_written", nvs_handle));

    // vTaskDelay(10000000/portTICK_PERIOD_MS);

    for (;;) {
        if (xQueueReceive(uart0_queue, (void *)&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            // ESP_LOGI(TAG, "uart[%d] event:", ESP_COMMU_UART);

            switch (event.type) {
                // Event of UART receving data
                // We'd better handler data event fast, there would be much more data events than
                // other types of events. If we take too much time on data event, the queue might be full.
                case UART_DATA:
                    // ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(ESP_COMMU_UART, dtmp, event.size, portMAX_DELAY);
                    // ESP_LOGI(TAG, "[DATA EVT]:");
                    uart_write_bytes(ESP_COMMU_UART, (const char *) dtmp, event.size);
                    // printf("%s\n", (const char*)dtmp);

                    // TODO: find min length of configuration data
                    // TODO: change configuration process 
                    // When event size = 77 and is_written flag also added if condition
                    
                    if (event.size == 77 && !wifi_ble_config_processing_done_flag)
                    {
                        wifi_ble_config_processing_done_flag = 1;
                        ESP_LOGE(TAG, "-------------------  WIFI(UDP) & BLE Configuration !!  -------------------");
                        for (int i = 0; i < event.size; i++)
                        {
                            hostUartParser(dtmp[i]);
                        }

                        // parser ok. and get wifi configuration data.
                        printf("----------------1-------------\r\n");
                        for (int i = 0; i < 16; i++)
                        {
                            // printf("0x%x ",hostToMcu.messageBody[i]);
                            printf("%c",hostToMcu.messageBody[i]);
                            udp_server_ip[i] = hostToMcu.messageBody[i];
                        }
                        printf("\r\n\r\n");

                        printf("----------------2-------------\r\n");
                        for (int i = 0; i < 16; i++)
                        {
                            // printf("0x%x ", hostToMcu.messageBody[i+16]);
                            printf("%c",hostToMcu.messageBody[i+16]);
                            wifi_client_ip[i] = hostToMcu.messageBody[i+16];
                        }
                        printf("\r\n\r\n");
                        printf("----------------3-------------\r\n");
                        for (int i = 0; i < 16; i++)
                        {
                            // printf("0x%x ", hostToMcu.messageBody[i+32]);
                            printf("%c",hostToMcu.messageBody[i+32]);
                            wifi_ssid[i] = hostToMcu.messageBody[i+32];
                        }
                        printf("\r\n\r\n");

                        printf("----------------4-------------\r\n");
                        char string_ble_name_buf[25];
                        // memset(string_ble_name_buf, 0, 25 * sizeof(char));
                        // printf("ble before %s", string_ble_name_buf);
                        for (int i = 0; i < 24; i++)
                        {
                            // printf("0x%x ", hostToMcu.messageBody[i+48]);
                            printf("%c",hostToMcu.messageBody[i+48]);
                            string_ble_name_buf[i] = hostToMcu.messageBody[i+48];
                        }
                        printf("\r\n\r\n");

                        strcat(remote_device_name, string_ble_name_buf);
                        // printf("ble name 2%s\n", remote_device_name);

                        printf("\r\n\r\n");

                        for (int i = 0; i < 10; i++)
                        {
                            uart_write_bytes(ESP_COMMU_UART, (const char*)ESP_ACK_OK, sizeof(ESP_ACK_OK)*2);
                            printf("ESP32OK send\n");
                            vTaskDelay(100/portTICK_PERIOD_MS);
                        }
                        
                        printf("\r\n\r\n");


                        if (!nvs_read_u8("is_written", nvs_handle)) 
                        {
                            printf("No saved WIFI-BLE information ------- Save config info \n");
                            nvs_write_string("udp_server_ip", (char *)udp_server_ip, nvs_handle);
                            nvs_write_string("wifi_client_ip", (char *)wifi_client_ip, nvs_handle);
                            printf("%s\n", (char *)wifi_ssid);
                            nvs_write_string("wifi_ssid", (char *)wifi_ssid, nvs_handle);
                            printf("%s\n", (char *)remote_device_name);
                            nvs_write_string("ble_device_name", (char *)remote_device_name, nvs_handle);

                            vTaskDelay(1000/portTICK_PERIOD_MS);

                            ESP_LOGI(TAG, "WIFI Task Start !");
                            xTaskCreate(wifi_connect_manager, "wifi_connect", 4096, NULL, 5, NULL);
                            vTaskDelay(2000/portTICK_PERIOD_MS);

                            // ESP_LOGI(TAG, "BLE Task Start !");

                            printf("jaeuk 11 %c \n", string_ble_name_buf[0]);
                            printf("jaeuk 22 %c \n", string_ble_name_buf[1]);
                            printf("jaeuk 33 %c \n", string_ble_name_buf[2]);

                            // TODO : if ble name size zero, won't turn on ble task.
                            printf("ble str len = %d \n", strlen(string_ble_name_buf));

                            if (strlen(string_ble_name_buf) != 0)
                            {
                                ESP_LOGI(TAG, "BLE Task Start !");
                                // xTaskCreate(ble_hr_task, "ble_client", 4096, NULL, 5, NULL);
                            }

                            vTaskDelay(2000/portTICK_PERIOD_MS);
                            
                            nvs_write_u8("is_written", 1, nvs_handle);

                            reset_flag = 0;
                        }

       

                        // ESP_LOGI(TAG, "WIFI Task Start !");

                        // xTaskCreate(wifi_connect_manager, "wifi_connect", 4096, NULL, 5, NULL);
                        // vTaskDelay(2000/portTICK_PERIOD_MS);

                        // // ESP_LOGI(TAG, "BLE Task Start !");

                        // printf("jaeuk 11 %c \n", string_ble_name_buf[0]);
                        // printf("jaeuk 22 %c \n", string_ble_name_buf[1]);
                        // printf("jaeuk 33 %c \n", string_ble_name_buf[2]);

                        // // TODO : if ble name size zero, won't turn on ble task.
                        // printf("ble str len = %d \n", strlen(string_ble_name_buf));

                        // if (strlen(string_ble_name_buf) != 0)
                        // {
                        //     ESP_LOGI(TAG, "BLE Task Start !");
                        //     xTaskCreate(ble_hr_task, "ble_client", 4096, NULL, 5, NULL);
                        // }

                        // vTaskDelay(2000/portTICK_PERIOD_MS);

                        
                    }
                    else if (event.size == 22)
                    {
                        if (nvs_read_u8("is_written", nvs_handle) && reset_flag)
                        {
                            memset(udp_server_ip, 0, 16);
                            memset(wifi_client_ip, 0, 16);
                            memset(wifi_ssid, 0, 16);
                            memset(remote_device_name, 0, 24);

                            ESP_LOGI(TAG, "After reset restart wifi-ble task");
                            strcat(udp_server_ip, nvs_read_str("udp_server_ip", nvs_handle));
                            strcat(wifi_client_ip, nvs_read_str("wifi_client_ip", nvs_handle));
                            strcat(wifi_ssid, nvs_read_str("wifi_ssid", nvs_handle));
                            strcat(remote_device_name, nvs_read_str("ble_device_name", nvs_handle));
                            printf("check------------\n %s \n %s \n %s \n %s \n", udp_server_ip, wifi_client_ip, wifi_ssid, remote_device_name);

                            ESP_LOGI(TAG, "WIFI Task Start !");

                            xTaskCreate(wifi_connect_manager, "wifi_connect", 4096, NULL, 5, NULL);
                            vTaskDelay(2000/portTICK_PERIOD_MS);

                            
                            // printf("BLE config name check = %s", remote_device_name);
                            printf("jaeuk 1 %c \n", remote_device_name[0]);
                            printf("jaeuk 2 %c \n", remote_device_name[1]);
                            printf("jaeuk 3 %c \n", remote_device_name[2]);

                            // TODO : if ble name size zero, won't turn on ble task.
                            // if (strlen(string_ble_name_buf != 0))
                            printf("ble str len --2--= %d \n", strlen(remote_device_name));
                            if (strlen(remote_device_name) != 0)
                            {
                                ESP_LOGI(TAG, "BLE Task Start !");
                                // xTaskCreate(ble_hr_task, "ble_client", 4096, NULL, 5, NULL);
                            }

                            // xTaskCreate(ble_hr_task, "ble_client", 4096, NULL, 5, NULL);
                            vTaskDelay(2000/portTICK_PERIOD_MS);

                            reset_flag = 0;
                        }
 

                        for (int i=0; i<22; i++)
                        {
                            gps_hr_chunck_data[i + offset] = dtmp[i];
                            // printf("%x", gps_hr_chunck_data[i + offset]);
                        }
                        
                        // printf("\r\n");

                        gps_hr_chunck_data[22 + offset] = ble_hr_share_val;
                        gps_hr_chunck_data[23 + offset] = '\n';

                        
                        offset += 24;
                        data_integ_cnt ++;

                        if (data_integ_cnt == 10)
                        {
                            printf("UDP data send here!!!!!!!!!!!!!!!!\n");
                            printf("--------- HR data : %d --------\n", ble_hr_share_val);
                            printf("%s\n", (const char*)gps_hr_chunck_data);
                            ESP_LOGI(TAG, "@@@@@@@@@@@ heap3 is %u", heap_caps_get_free_size(MALLOC_CAP_8BIT));
                            // printf("length of udp data = %d \n", strlen((const char*)gps_hr_chunck_data));
                            udp_msg_sent((const char *)udp_server_ip ,(const char *)gps_hr_chunck_data);

                            hr_data_to_mcu[0] = 'H';
                            hr_data_to_mcu[1] = 'R';
                            hr_data_to_mcu[2] =  ble_hr_share_val;
                            hr_data_to_mcu[3] = '\n';



                            printf("Check ESP -> MCU HR data uart1 = %s \n", (const char*)hr_data_to_mcu);

                            for (int k = 0; k < 5; k++)
                            {
                                uart_write_bytes(ESP_COMMU_UART, (const char *) hr_data_to_mcu, 4);
                                vTaskDelay(10/portTICK_PERIOD_MS);
                            }
                            
                            memset(gps_hr_chunck_data, 0, 256);
                            memset(hr_data_to_mcu, 0, 4);
                            data_integ_cnt = 0;
                            offset = 0;
                        }
                        
                    }
                    // TODO : add nvs flash erase process from st mcu command 
                    else if (event.size == 6 && dtmp[0] == 'S' && dtmp[1] == 'T' && dtmp[2] == 'A')
                    {
                        // printf("Erase WIFI & BLE configuration information \n");
                        // is_written change & erase nvs flash reset
                        // nvs_flash_erase();
                        // esp_restart();

                        printf("Check WiFi connection status ! \n");

                        // Send wifi connection state to mcu using uart (check wifi)
                        // if (WIFI_FAIL_BIT)
                        // {
                        //     printf(" ------------ WiFi status = Disconnect ! ------------\n");
                        //     for (int z=0; z < 5; z++)
                        //     {
                        //         uart_write_bytes(ESP_COMMU_UART, (const char *)ESP_WIFI_DISCONNECTED, 5);
                        //         vTaskDelay(10/portTICK_PERIOD_MS);
                        //     }
                        // }
                        if (wifi_connection_flag)
                        {
                            printf(" ------------ WiFi status = Connected ! ------------\n");
                            for (int z=0; z < 5; z++)
                            {
                                uart_write_bytes(ESP_COMMU_UART, (const char *)ESP_WIFI_CONNECTED, 10);
                                vTaskDelay(10/portTICK_PERIOD_MS);
                            }
                        }
                    }
                    else if (event.size == 5 && dtmp[0] == 'E' && dtmp[1] == 'R' && dtmp[2] == 'A')
                    {
                        // printf("%c \n%c \n%c \n", dtmp[0], dtmp[1], dtmp[2]);
                        printf("Erase WIFI & BLE configuration information \n");
                        // is_written change & erase nvs flash reset
                        nvs_flash_erase();
                        esp_restart();
                    }
                    else
                    {
                        printf("Unfiltered data = %s\n", (const char*)dtmp);
                    }
                    
                    break;

                // Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    // ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(ESP_COMMU_UART);
                    xQueueReset(uart0_queue);
                    break;

                // Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    // ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(ESP_COMMU_UART);
                    xQueueReset(uart0_queue);
                    break;

                case UART_PARITY_ERR:
                    // ESP_LOGI(TAG, "uart parity error");
                    break;

                // Event of UART frame error
                case UART_FRAME_ERR:
                    // ESP_LOGI(TAG, "uart frame error");
                    break;

                // Others
                default:
                    // ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
    // nvs_close(nvs_handle);
}


void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    // /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    //  * Read "Establishing Wi-Fi or Ethernet Connection" section in
    //  * examples/protocols/README.md for more information about this function.
    //  */
    // ESP_ERROR_CHECK(example_connect());
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(ESP_COMMU_UART, &uart_config);
    uart_set_pin(ESP_COMMU_UART, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);

    // Install UART driver, and get the queue.
    uart_driver_install(ESP_COMMU_UART, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 100, &uart0_queue, 0);

    #if CONFIG_PM_ENABLE
        // Configure dynamic frequency scaling:
        // maximum and minimum frequencies are set in sdkconfig,
        // automatic light sleep is enabled if tickless idle support is enabled.
    #if CONFIG_IDF_TARGET_ESP32
        esp_pm_config_esp32_t pm_config = {
    #elif CONFIG_IDF_TARGET_ESP32S2
        esp_pm_config_esp32s2_t pm_config = {
    #endif
                .max_freq_mhz = 80,
                .min_freq_mhz = 80,
    #if CONFIG_FREERTOS_USE_TICKLESS_IDLE
                .light_sleep_enable = true
    #endif
        };
        ESP_ERROR_CHECK( esp_pm_configure(&pm_config));
    #endif // CONFIG_PM_ENABLE

    xTaskCreate(udp_client_task, "udp_client", 8192, NULL, 5, NULL);
    // xTaskCreate(ble_hr_task, "ble_client", 4096, NULL, 5, NULL);
}
