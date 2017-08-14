#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/xtensa_api.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_wpa2.h"

#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_heap_alloc_caps.h"
#include "hwcrypto/sha.h"
#include "esp_system.h"
#include "wpa2/utils/base64.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "nvs_flash.h"
#include "tcpip_adapter.h"

#include "lwip/api.h"

#include "sdkconfig.h"

#include "ads1115.h"
#include "initi2c.h"
#include "WebSocket_Task.h"
#include "Ledc_Task.h"
#include "car_control.h"
//#include "mongoose.h"
//#include "dirent.h"

static const char *TAG = "example";
const int CONNECTED_BIT = 0x00000001;

#define AP_EXAMPLE_WIFI_SSID "ESPLink"
#define AP_EXAMPLE_WIFI_PASS "ESPKey01"
xQueueHandle gpio_evt_queue = NULL;
//#define STA_EXAMPLE_WIFI_SSID "Syrp_Wireless"
//#define STA_EXAMPLE_WIFI_PASS "87654321"

static EventGroupHandle_t wifi_event_group;
//WebSocket frame receive queue
QueueHandle_t WebSocket_rx_queue;

void task_process_WebSocket( void *pvParameters ){
    (void)pvParameters;

    //frame buffer
    WebSocket_frame_t __RX_frame;

    //create WebSocket RX Queue
    WebSocket_rx_queue = xQueueCreate(10,sizeof(WebSocket_frame_t));

    while (1){
        //receive next WebSocket frame from queue
        if(xQueueReceive(WebSocket_rx_queue,&__RX_frame, 3*portTICK_PERIOD_MS)==pdTRUE){

        	//write frame inforamtion to UART
        	printf("New Websocket frame. Length %d, payload %.*s \r\n", __RX_frame.payload_length, __RX_frame.payload_length, __RX_frame.payload);
            //fillColor(0,255,0); //use pings instead
        	//loop back frame
        	WS_write_data(__RX_frame.payload, __RX_frame.payload_length);

        	//free memory
			if (__RX_frame.payload != NULL)
				free(__RX_frame.payload);

        }
    }
}
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_AP_START:
    	printf("\nSYSTEM_EVENT_AP_STA_START\n");
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        if(is_WS_conn_Null()){
        	printf("WS_conn Null");
        }else{
        	printf("WS_conn not Null");
        }
        printf("\nSYSTEM_EVENT_AP_STACONNECTED\n");
        fillColor(255,165,0);
        clientconnected=1;
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
    	fillColor(255,0,0);
    	clientconnected=0;
    	if(is_WS_conn_Null()){
    	        	printf("WS_conn Null");
    	        }else{
    	        	printf("WS_conn not Null");
    	        }
     	//clientdisconnectedclose();
    	printf("\nSYSTEM_EVENT_AP_STADISCONNECTED\n");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

void initialise_wifi(void)
{
tcpip_adapter_init();
wifi_event_group = xEventGroupCreate();
ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
wifi_config_t wifi_config = {
.ap = {
.ssid = AP_EXAMPLE_WIFI_SSID,
.ssid_len = 0,
.password = AP_EXAMPLE_WIFI_PASS,
.channel = 1,
.authmode = WIFI_AUTH_WPA2_PSK,
.beacon_interval = 100,
.max_connection = 4,
}
};

ESP_LOGI(TAG, "Setting WiFi AP: SSID %s", wifi_config.ap.ssid);
ESP_LOGI(TAG, " Pass %s", wifi_config.ap.password);

ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );
ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_AP, &wifi_config) );
//ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
ESP_ERROR_CHECK( esp_wifi_start());
}

void app_main()
{

    nvs_flash_init();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //vTaskDelay(250 / portTICK_PERIOD_MS);
    printf("\ntrying to initialise wifi..\n");
    initialise_wifi();
    printf("\ntrying to initialise ledc..\n");
    initialise_ledc();
    printf("\ntrying to initialise gpio..\n");
    initialise_gpio();
    printf("\ntrying to initialise i2c..\n");
    i2c_master_init();
    scani2c();
    printf("\ntrying to initialise ads1115..\n");
    Adafruit_ADS1115(0x48);
    printf("\ntrying to initialise ws2812b..\n");
    initialise_ws2812b();
    printf("\ntrying to start webserver..\n");


    xTaskCreate(&gpio_task, "gpio_task", 2048, NULL, 10, NULL);
    //create WebSocker receive task
        xTaskCreate(&task_process_WebSocket, "ws_process_rx", 2048, NULL, 5, NULL);
    //websocket task
        xTaskCreate(&ws_server, "ws_server", 2048, NULL, 5, NULL);
    //distance task
        xTaskCreate(&safetyLoop, "safetyLoop", 2048, NULL, 4, NULL);
    //occasional battery check
        xTaskCreate(&batterycheck, "batterycheck", 2048, NULL, 4, NULL);
    //xTaskCreate(&mongooseTask, "mongooseTask", 2048, NULL, 5, NULL);
}

/*
static sig_atomic_t s_signal_received = 0;
static const char *s_http_port = "8000";

static void signal_handler(int sig_num) {
  signal(sig_num, signal_handler);  // Reinstantiate signal handler
  s_signal_received = sig_num;
}

static int is_websocket(const struct mg_connection *nc) {
  return nc->flags & MG_F_IS_WEBSOCKET;
}

static void broadcast(struct mg_connection *nc, const struct mg_str msg) {
  struct mg_connection *c;
  char buf[500];
  char addr[32];
  mg_sock_addr_to_str(&nc->sa, addr, sizeof(addr),
                      MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);

  snprintf(buf, sizeof(buf), "%s %.*s", addr, (int) msg.len, msg.p);
  printf("%s\n", buf); // Local echo.
  for (c = mg_next(nc->mgr, NULL); c != NULL; c = mg_next(nc->mgr, c)) {
    if (c == nc) continue; //Don't send to the sender.
    mg_send_websocket_frame(c, WEBSOCKET_OP_TEXT, buf, strlen(buf));
  }
}

static void ev_handler(struct mg_connection *nc, int ev, void *ev_data) {
  switch (ev) {
    case MG_EV_WEBSOCKET_HANDSHAKE_DONE: {
      // New websocket connection. Tell everybody.
      broadcast(nc, mg_mk_str("++ joined"));
      break;
    }
    case MG_EV_WEBSOCKET_FRAME: {
      struct websocket_message *wm = (struct websocket_message *) ev_data;
      // New websocket message. Tell everybody.
      struct mg_str d = {(char *) wm->data, wm->size};
      broadcast(nc, d);
      break;
    }
    case MG_EV_CLOSE: {
      // Disconnect. Tell everybody.
      if (is_websocket(nc)) {
        broadcast(nc, mg_mk_str("-- left"));
      }
      break;
    }
  }
}
*/

/*
void mongooseTask(){
	  struct mg_mgr mgr;
	      struct mg_connection *nc;

	      signal(SIGTERM, signal_handler);
	      signal(SIGINT, signal_handler);
	      setvbuf(stdout, NULL, _IOLBF, 0);
	      setvbuf(stderr, NULL, _IOLBF, 0);

	      mg_mgr_init(&mgr, NULL);

	      nc = mg_bind(&mgr, s_http_port, ev_handler);
	      mg_set_protocol_http_websocket(nc);

	      printf("Started on port %s\n", s_http_port);
	      while (s_signal_received == 0) {
	        mg_mgr_poll(&mgr, 200);
	      }
	      mg_mgr_free(&mgr);

}
*/

/* SPI Master example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
