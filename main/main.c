/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "network.h"
#include "app_cli.h"
#include "app_shell.h"

#include "driver/gpio.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "ping/ping_sock.h"

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  0

#define TXD_PIN             GPIO_NUM_19
#define RXD_PIN             GPIO_NUM_20

#define EX_UART_NUM         UART_NUM_1
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;
void RxData_Handle(uint8_t* Rxdata, int length);

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t m_event_bit_network;


#define BIT_WIFI_GOT_IP    BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;
WifiInfo_t m_wifi_info;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
                s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(m_event_bit_network, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(m_event_bit_network, BIT_WIFI_GOT_IP);
    }
}
bool network_is_connected(void)
{
    EventBits_t network = xEventGroupGetBits(m_event_bit_network);
    if (network & BIT_WIFI_GOT_IP)
    {
        return true;
    }
    return false;
}
void wifi_change_info(char *ssid, char *password)
{
    ESP_LOGI(TAG, "Wifi user name %s, pass %s\r\n", ssid, password);

    m_event_bit_network = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_wifi_stop() );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    sprintf((char *)wifi_config.sta.ssid, "%s", ssid);
    sprintf((char *)wifi_config.sta.password, "%s", password);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

void wifi_init_sta(char *ssid, char *password)
{
    m_event_bit_network = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    sprintf((char *)wifi_config.sta.ssid, "%s", ssid);
    sprintf((char *)wifi_config.sta.password, "%s", password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    //size_t buffered_size;
    uint8_t* data = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) { 
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(data, RD_BUF_SIZE);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    int data_len = uart_read_bytes(EX_UART_NUM, data, event.size, portMAX_DELAY);

                    RxData_Handle(data, data_len);
                    //uart_write_bytes(EX_UART_NUM, (const char*) data, data_size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    break;
                //Others
                default:
                    break;
            }
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    free(data);
    data = NULL;
    vTaskDelete(NULL);
}

void RxData_Handle(uint8_t* Rxdata, int length)
{
    if (length > 0)
    {    
        if (strstr((char*)Rxdata, "wifi:") != NULL)
        {
            //uart_write_bytes(EX_UART_NUM, (char*)Rxdata, length);
            ESP_LOGE(TAG, "rx : %s", (char *) Rxdata);

            char *pTemp = strstr((char*)Rxdata,"wifi:");
		
	        if(pTemp == 0) return;

            pTemp = strtok(pTemp,":"); 	
	        pTemp = strtok(NULL,","); // wifi stt

            sprintf(m_wifi_info.cmd,"%s", pTemp);

            pTemp = strtok(NULL,","); // wifi name

            sprintf(m_wifi_info.ssid,"%s", pTemp);

            pTemp = strtok(NULL,","); // wifi password

            sprintf(m_wifi_info.pass,"%s", pTemp);

            ESP_LOGI(TAG, "%s, %s, %s", m_wifi_info.cmd, m_wifi_info.ssid, m_wifi_info.pass);
        }
    }
}
static void cmd_ping_on_ping_success(esp_ping_handle_t hdl, void *args)
{
    uint8_t ttl;
    uint16_t seqno;
    uint32_t elapsed_time, recv_len;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
    printf("%" PRIu32 " bytes from %s icmp_seq=%" PRIu16 " ttl=%" PRIu16 " time=%" PRIu32 " ms\n",
           recv_len, ipaddr_ntoa((ip_addr_t *)&target_addr), seqno, ttl, elapsed_time);
}
static int do_ping_cmd(void)
{
    // Ping configuration
    esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();

    // IP address
    config.target_addr.type = IPADDR_TYPE_V4;
    ipaddr_aton("8.8.8.8", &config.target_addr);
    const char *ip_string = ipaddr_ntoa(&config.target_addr);
    printf("IP Address for config.target_addr: %s\n", ip_string);

    // Set callback function
    esp_ping_callbacks_t cbs = {
        .cb_args = NULL,
        .on_ping_success = cmd_ping_on_ping_success,
    };

    // Ping session parameters: (config - ping configuration, cbs - callback functions, hdl_out  - handle of ping session)
    esp_ping_handle_t ping;
    esp_ping_new_session(&config, &cbs, &ping);
    esp_ping_start(ping);

    return 0;
}
static int cli_console_puts(const char *msg)
{
    if (msg)
    {
        printf("%s", msg);
        fflush(stdout);
    }
    return 0;
}
void cli_console_write(uint8_t *buffer, uint32_t size)
{
    if (!buffer)
    {
        return;
    }
    for (int i = 0; i < size; i++)
    {
        putchar(buffer[i]);
    }
    fflush(stdout);
}
void cli_console_close()
{
}

static app_cli_cb_t m_tcp_cli = 
{
    .puts = cli_console_write,
    .printf = cli_console_puts,
    .terminate = cli_console_close
};

static void cli_task (void *pvParameters)
{
    app_cli_start(&m_tcp_cli);
    for (;;)
    {
        while (1)
        {
            char c = getchar();
            if (c == 0xFF)
            {
                break;
            }
            else
            {
                app_cli_poll(c);
            }
        }
        vTaskDelay(10);
    }
}

static void ping_task (void *pvParameters)
{
    for (;;)
    {
        if (network_is_connected())
        {
            do_ping_cmd();
        }
        vTaskDelay(6000 / portTICK_PERIOD_MS);
    }
}

void uart_init(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
}
bool m_wifi_enable = false;
bool do_reconnect_wifi = false;
static void wifi_process(void)
{
    if (m_wifi_enable == false)
    {
        return;
    }
    if (do_reconnect_wifi)
    {
        do_reconnect_wifi = false;
        ESP_LOGE(TAG, "wifi_process");
        wifi_change_info(m_wifi_info.ssid, m_wifi_info.pass);
    }
}
void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta("BTIOT", "bytech123");

    xTaskCreate(cli_task, "cli_task", 4096, NULL, 12, NULL);
    xTaskCreate(ping_task, "ping_task", 4096, NULL, 12, NULL);
    //uart_init();
    
    while (1)
    {
        wifi_process();

        ESP_LOGI(TAG, "2s");
        // static int count = 0;
        // if (count++ == 6)
        // {
        //     // Write data to UART.
        //     char* test_str = "wifi:enable,BYTECH,bytech@2020,";
        //     uart_write_bytes(EX_UART_NUM, (const char*)test_str, strlen(test_str));
        // }
        vTaskDelay(2000/ portTICK_PERIOD_MS);
    }
    
}
