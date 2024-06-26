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

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "app_debug.h"

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
EventGroupHandle_t m_event_bit_network;
static SemaphoreHandle_t m_semaphore_cdc_data;
static SemaphoreHandle_t m_debug_lock;

#define CDC_APP_BUFFER_SIZE 256

typedef struct
{
    uint8_t buf[CDC_APP_BUFFER_SIZE];
    uint16_t index;
} cdc_rx_buffer_t;

static const char *TAG = "wifi station";

static int s_retry_num = 0;
WifiInfo_t m_wifi_info;
static cdc_rx_buffer_t m_cdc_rx_buf;

// Process RX data
static void application_on_rx_callback(uint8_t *data, uint32_t len)
{
    // if (len >= 5 && memcmp(m_cdc_rx_buf.buf, "Hello", len) == 0)
    // {
    //     DEBUG_INFO(TAG, "Helooo!!!!!!!!!!!!!!!!!!!!!!!");
    // }
    for (uint32_t count = 0; count < len; count++)
    {
        app_cli_poll(data[count]);
    }
}
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    xSemaphoreGive(m_semaphore_cdc_data);
}
// // Connect & disconnect event callback
// static void application_on_disconnected_callback()
// {
//     char* send_str = "application disconnected";
//     tinyusb_cdcacm_write_queue(0, (uint8_t *)send_str, strlen(send_str));
//     tinyusb_cdcacm_write_flush(0, 0);
// }

// static void application_on_connected_callback(void)
// {
//     char* send_str = "application connected";
//     tinyusb_cdcacm_write_queue(0, (uint8_t *)send_str, strlen(send_str));
//     tinyusb_cdcacm_write_flush(0, 0);
// }
void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    DEBUG_INFO("Line state changed on channel %d: DTR:%d, RTS:%d\r\n", itf, dtr, rts);
    // if (dtr == 1 && rts == 1)
    // {
    //     application_on_connected_callback();
    // }
    // else if (dtr == 0 && rts == 1)
    // {
    //     application_on_disconnected_callback();
    // }
}

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
            DEBUG_INFO("retry to connect to the AP\r\n");
        } else {
            xEventGroupSetBits(m_event_bit_network, WIFI_FAIL_BIT);
        }
        DEBUG_INFO("connect to the AP fail\r\n");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        DEBUG_INFO("got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        DEBUG_INFO("\r\n");
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
    DEBUG_INFO("Wifi user name %s, pass %s\r\n", ssid, password);

    m_event_bit_network = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_wifi_stop() );
    // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // ESP_ERROR_CHECK(esp_wifi_init(&cfg));

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
    if (strlen(password) < 3) wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    DEBUG_INFO("wifi_init_sta finished.\r\n");
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
    if (strlen(password) < 3) wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    DEBUG_INFO("wifi_init_sta finished.\r\n");
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
            DEBUG_ERROR("rx : %s\r\n", (char *) Rxdata);

            char *pTemp = strstr((char*)Rxdata,"wifi:");
		
	        if(pTemp == 0) return;

            pTemp = strtok(pTemp,":"); 	
	        pTemp = strtok(NULL,","); // wifi stt

            sprintf(m_wifi_info.cmd,"%s", pTemp);

            pTemp = strtok(NULL,","); // wifi name

            sprintf(m_wifi_info.ssid,"%s", pTemp);

            pTemp = strtok(NULL,","); // wifi password

            sprintf(m_wifi_info.pass,"%s", pTemp);

            DEBUG_INFO("%s, %s, %s\r\n", m_wifi_info.cmd, m_wifi_info.ssid, m_wifi_info.pass);
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
    DEBUG_RAW("%u bytes from %s icmp_seq=%u ttl=%u time=%u ms\r\n",
           recv_len, ipaddr_ntoa((ip_addr_t *)&target_addr), seqno, ttl, elapsed_time);
}
static void cmd_ping_on_ping_end(esp_ping_handle_t hdl, void *args)
{
    int transmitted;
    int received;
    int total_time_ms;

    esp_ping_get_profile(hdl, ESP_PING_PROF_REQUEST, &transmitted, sizeof(transmitted));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &received, sizeof(received));
    esp_ping_get_profile(hdl, ESP_PING_PROF_DURATION, &total_time_ms, sizeof(total_time_ms));
    DEBUG_RAW("%u packets transmitted, %u received, time %u ms\r\n", transmitted, received, total_time_ms);

    esp_ping_stop(hdl);
    esp_ping_delete_session(hdl);
}
static int do_ping_cmd(void)
{
    // Ping configuration
    esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();

    // IP address
    config.target_addr.type = IPADDR_TYPE_V4;
    ipaddr_aton("8.8.8.8", &config.target_addr);
    const char *ip_string = ipaddr_ntoa(&config.target_addr);
    DEBUG_INFO("IP Address for config.target_addr: %s\r\n", ip_string);

    // Set callback function
    esp_ping_callbacks_t cbs = {
        .cb_args = NULL,
        .on_ping_success = cmd_ping_on_ping_success,
        .on_ping_end = cmd_ping_on_ping_end
    };

    // Ping session parameters: (config - ping configuration, cbs - callback functions, hdl_out  - handle of ping session)
    esp_ping_handle_t ping;
    esp_ping_new_session(&config, &cbs, &ping);
    esp_ping_start(ping);

    // vTaskDelay(5000 / portTICK_PERIOD_MS);

    // esp_ping_stop(ping);
    // esp_ping_delete_session(ping);

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
    char c;
    while(1)
    {
        int nread = fread(&c, 1, 1, stdin);
        if(nread > 0 && c != 0xFF)
        {
            // putchar('x')
            app_cli_poll(c);
        }
        else
        {
            vTaskDelay(10/ portTICK_PERIOD_MS);
        }
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
        vTaskDelay(12000 / portTICK_PERIOD_MS);
    }
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
        DEBUG_ERROR("wifi_process\r\n");
        wifi_change_info(m_wifi_info.ssid, m_wifi_info.pass);
    }
}
static void wifi_task (void *pvParameters)
{
    for (;;)
    {
        wifi_process();

        static int32_t count_log = 0;
        if (count_log++ >= 5)
        {
            count_log = 0;
            DEBUG_INFO("10s\r\n");
        }
        vTaskDelay(2000/ portTICK_PERIOD_MS);
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
uint32_t debug_serial_print(const void *buffer, uint32_t len)
{
    char *ptr = (char*)buffer;
    for (int i = 0; i < len; i++)
    {
        putchar(ptr[i]);
    }
    // fflush(stdout);
    return len;
}
uint32_t debug_tiny_usb_print(const void *buffer, uint32_t len)
{
    char *ptr = (char*)buffer;
    for (int i = 0; i < len; i++)
    {
        tinyusb_cdcacm_write_queue(0, (uint8_t *)&ptr[i], 1);
        tinyusb_cdcacm_write_flush(0, 0);
    }
    // fflush(stdout);
    return len;
}
uint32_t debug_get_ms()
{
    return xTaskGetTickCount();
}


bool debug_get_lock(bool lock, uint32_t timeout_ms)
{
    if (lock)
    {
        return xSemaphoreTake(m_debug_lock, timeout_ms);
    }
    xSemaphoreGive(m_debug_lock);
    return true;
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

    m_debug_lock = xSemaphoreCreateMutex();
    xSemaphoreGive(m_debug_lock);
    app_debug_init(debug_get_ms, debug_get_lock);
    app_debug_register_callback_print(debug_tiny_usb_print);
    app_debug_register_callback_print(debug_serial_print);

    DEBUG_INFO("ESP_WIFI_MODE_STA\r\n");
    wifi_init_sta("Kaka", "");

    xTaskCreate(cli_task, "cli_task", 4096, NULL, 13, NULL);
    xTaskCreate(ping_task, "ping_task", 4096, NULL, 12, NULL);
    xTaskCreate(wifi_task, "wifi_task", 4096, NULL, 12, NULL);
    //uart_init();

    m_semaphore_cdc_data = xSemaphoreCreateCounting(10, 0);
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 256,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    /* the second way to register a callback */
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));
int retries_received = 2;
    int wait_time = 50;
    uint8_t tmp[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
    size_t rx_size = 0;
    while (1)
    {
        bool allow_process = false;
        while (xSemaphoreTake(m_semaphore_cdc_data, wait_time))
        {
            if (retries_received)
            {
                retries_received--;
                wait_time = 50;
            }

            /* read */
            esp_err_t ret = tinyusb_cdcacm_read(0, tmp, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
            if (ret != ESP_OK) 
            {
                DEBUG_ERROR("CDC usb RX error\r\n");
                continue;
            } 

            int max_size = CDC_APP_BUFFER_SIZE-m_cdc_rx_buf.index;
            if (max_size < rx_size)
            {
                DEBUG_ERROR("Ringbuffer USB CDC full\r\n");
            }
            else
            {
                max_size = rx_size;
            }

            memcpy(m_cdc_rx_buf.buf+m_cdc_rx_buf.index, tmp, max_size);
            m_cdc_rx_buf.index += max_size;

            allow_process = true;
            /* write back */
            tinyusb_cdcacm_write_queue(0, m_cdc_rx_buf.buf, rx_size);
            tinyusb_cdcacm_write_flush(0, 0);
        }
        // else if (retries_received)
        // {
        //     retries_received--;
        //     if (retries_received == 0)
        //     {
        //         retries_received = 0;
        //         allow_process = true;
        //         wait_time = 50;
        //     }
        // }

        // Process data
        if (allow_process && m_cdc_rx_buf.index)
        {
            // // // // Read data 
            esp_err_t ret = tinyusb_cdcacm_read(0, tmp, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
            if (ret == ESP_OK) 
            {
                int max_size = CDC_APP_BUFFER_SIZE-m_cdc_rx_buf.index;
                if (max_size < rx_size)
                {
                    DEBUG_ERROR("Ringbuffer USB CDC full\r\n");
                }
                else
                {
                    max_size = rx_size;
                }

                memcpy(m_cdc_rx_buf.buf+m_cdc_rx_buf.index, tmp, max_size);
                m_cdc_rx_buf.index += max_size;
            } 

            application_on_rx_callback(m_cdc_rx_buf.buf, m_cdc_rx_buf.index);
            memset(m_cdc_rx_buf.buf, 0, CDC_APP_BUFFER_SIZE);
            m_cdc_rx_buf.index = 0;
        }
    }
    
}
