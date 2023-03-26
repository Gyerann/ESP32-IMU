#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "math.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_http_server.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/netdb.h>

//Conditional compilation defs
#define SERVER_DEBUG
#define DEBUG_MODE

//Network connection defs
#define WIFI_SSID "Telekom-04ff8f-2.4GHz"
#define WIFI_PASS "9FGJ3AJQ7RMZ"
#define MAX_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

//Pin defs
#define PIN_G_X ADC1_CHANNEL_3
#define PIN_G_Y ADC1_CHANNEL_7
#define PIN_G_Z ADC1_CHANNEL_6

#define PITCH   0
#define ROLL    1

#define UNIT_RAD    0
#define UNIT_DEGREE 1

//Globals
float g_gX, g_gY, g_gZ;
int g_offset_X = 1650, 
    g_offset_Y = 1650,
    g_offset_Z = 1650;
float pitch, roll;

char orientationHTML[128];

//FreeRTOS event group to signal connection
static EventGroupHandle_t s_wifi_event_group;

static int s_retry_num = 0;

esp_err_t send_web_page(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "access-control-allow-origin", "*");
    int response = httpd_resp_send(req, orientationHTML, HTTPD_RESP_USE_STRLEN);    
    return response;
}

esp_err_t get_req_handler(httpd_req_t *req)
{
    return send_web_page(req);
}

esp_err_t orientation_handler(httpd_req_t *req)
{

    return send_web_page(req);
}

static httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_req_handler,
    .user_ctx = NULL};

static httpd_uri_t uri_orientation = {
    .uri = "/orientation",
    .method = HTTP_GET,
    .handler = orientation_handler,
    .user_ctx = NULL};

void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAX_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI("Server", "Retrying to connect");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI("Server", "Connection failed");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("Server", "SERVER_IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    
    #ifdef SERVER_DEBUG
        ESP_LOGI("Server", "Event handler set up");
    #endif
}

void connect_wifi(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

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
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI("Server", "Connected to WiFi. SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI("Server", "Failed to connect to WiFi. SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    }
    else
    {
        ESP_LOGE("Server", "UNEXPECTED EVENT");
    }

    vEventGroupDelete(s_wifi_event_group);
}

httpd_handle_t setup_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_orientation);
        //httpd_register_uri_handler(server, &uri_off);
    }
    return server;
}

//Initializes ADC on the board
void init_adc(){  
    //ADC reference voltage calibration
    esp_adc_cal_characteristics_t adc1_chars;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);

    //Setting bit width
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);

    //Setting ADC voltage range
    adc1_config_channel_atten(PIN_G_X, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(PIN_G_Y, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(PIN_G_Z, ADC_ATTEN_DB_11);
}

//Calibrates accelerometer, device has to be upside down on a flat surface
//Upside down because my cables wouldnt fit otherwise lol
void calibrate_sensor() {
    g_offset_X = (float)adc1_get_raw(PIN_G_X);
    g_offset_Y = (float)adc1_get_raw(PIN_G_Y);
    g_offset_Z = (float)adc1_get_raw(PIN_G_Z) + 800;

    #ifdef DEBUG_MODE
        ESP_LOGI("Ameter offsets:", "(%i, %i, %i) mV", g_offset_X, g_offset_Y, g_offset_Z);
    #endif
}

//Reads accelerometer output voltage and calculates degrees
void read_g() {
    //Sensitivity at range 1.5g is 800mV/g
    //At rest: (1650, 1650, 2450) on paper, but youll be better off calibrating before each use
    g_gX = ((float)adc1_get_raw(PIN_G_X) - g_offset_X) / 800; 
    g_gY = ((float)adc1_get_raw(PIN_G_Y) - g_offset_Y) / 800;
    g_gZ = ((float)adc1_get_raw(PIN_G_Z) - g_offset_Z) / 800;

    #ifdef DEBUG_MODE
        ESP_LOGI("Accelerometer:", "G(%.2f, %.2f, %.2f)", g_gX, g_gY, g_gZ);
    #endif
}

//Calculates orientation based on accelerometer readings
//Note that only pitch and roll can be calculated
void calc_orientation(int unit) {
    if(unit == UNIT_RAD) {
        pitch = (atan(g_gX/sqrt(pow(g_gY,2) + pow(g_gZ,2))));
        roll = (atan(g_gY/sqrt(pow(g_gX,2) + pow(g_gZ,2))));

        #ifdef DEBUG_MODE
        ESP_LOGI("Rad:", "Pitch: %.1f, Roll: %.1f", pitch, roll);
        #endif
    } else {
        pitch = (atan(g_gX/sqrt(pow(g_gY,2) + pow(g_gZ,2))) * (180 / M_PI));
        roll = (atan(g_gY/sqrt(pow(g_gX,2) + pow(g_gZ,2))) * (180 / M_PI));

        #ifdef DEBUG_MODE
        ESP_LOGI("Degree:", "Pitch: %.1f, Roll: %.1f", pitch, roll);
        #endif
    }   
}

//Initializes NVS flash
void init_flash(){
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    #ifdef DEBUG_MODE
    ESP_LOGI("Flash:", "NVS Flash initialized");
    #endif
}

void init_network() {
    connect_wifi();
    setup_server();
}

void app_main() {

    init_adc();
    calibrate_sensor();
    init_flash();
    init_network();   

    while(true){
        read_g();
        calc_orientation(UNIT_DEGREE);
        sprintf(orientationHTML, "{\"pitch\":%.1f,\"roll\":%.1f}", pitch, roll); 
        vTaskDelay(50);
    }

}