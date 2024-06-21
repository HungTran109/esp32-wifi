#ifndef APP_WIFI_ETH_H
#define APP_WIFI_ETH_H

#include <stdint.h>
#include <stdbool.h>

typedef struct 
{
    char cmd[2];
    char ssid[50];
    char pass[16];
    uint8_t status;
} WifiInfo_t;

/** System event types enumeration */
typedef enum {
    SYSTEM_EVENT_WIFI_READY = 0,           /*!< ESP32 WiFi ready */
    SYSTEM_EVENT_SCAN_DONE,                /*!< ESP32 finish scanning AP */
    SYSTEM_EVENT_STA_START,                /*!< ESP32 station start */
    SYSTEM_EVENT_STA_STOP,                 /*!< ESP32 station stop */
    SYSTEM_EVENT_STA_CONNECTED,            /*!< ESP32 station connected to AP */
    SYSTEM_EVENT_STA_DISCONNECTED,         /*!< ESP32 station disconnected from AP */
    SYSTEM_EVENT_STA_AUTHMODE_CHANGE,      /*!< the auth mode of AP connected by ESP32 station changed */
    SYSTEM_EVENT_STA_GOT_IP,               /*!< ESP32 station got IP from connected AP */
    SYSTEM_EVENT_STA_LOST_IP,              /*!< ESP32 station lost IP and the IP is reset to 0 */
    SYSTEM_EVENT_STA_BSS_RSSI_LOW,         /*!< ESP32 station connected BSS rssi goes below threshold */
    SYSTEM_EVENT_STA_WPS_ER_SUCCESS,       /*!< ESP32 station wps succeeds in enrollee mode */
    SYSTEM_EVENT_STA_WPS_ER_FAILED,        /*!< ESP32 station wps fails in enrollee mode */
    SYSTEM_EVENT_STA_WPS_ER_TIMEOUT,       /*!< ESP32 station wps timeout in enrollee mode */
    SYSTEM_EVENT_STA_WPS_ER_PIN,           /*!< ESP32 station wps pin code in enrollee mode */
    SYSTEM_EVENT_STA_WPS_ER_PBC_OVERLAP,   /*!< ESP32 station wps overlap in enrollee mode */
    SYSTEM_EVENT_AP_START,                 /*!< ESP32 soft-AP start */
    SYSTEM_EVENT_AP_STOP,                  /*!< ESP32 soft-AP stop */
    SYSTEM_EVENT_AP_STACONNECTED,          /*!< a station connected to ESP32 soft-AP */
    SYSTEM_EVENT_AP_STADISCONNECTED,       /*!< a station disconnected from ESP32 soft-AP */
    SYSTEM_EVENT_AP_STAIPASSIGNED,         /*!< ESP32 soft-AP assign an IP to a connected station */
    SYSTEM_EVENT_AP_PROBEREQRECVED,        /*!< Receive probe request packet in soft-AP interface */
    SYSTEM_EVENT_ACTION_TX_STATUS,         /*!< Receive status of Action frame transmitted */
    SYSTEM_EVENT_ROC_DONE,                 /*!< Indicates the completion of Remain-on-Channel operation status */
    SYSTEM_EVENT_STA_BEACON_TIMEOUT,       /*!< ESP32 station beacon timeout */
    SYSTEM_EVENT_FTM_REPORT,               /*!< Receive report of FTM procedure */
    SYSTEM_EVENT_GOT_IP6,                  /*!< ESP32 station or ap or ethernet interface v6IP addr is preferred */
    SYSTEM_EVENT_ETH_START,                /*!< ESP32 ethernet start */
    SYSTEM_EVENT_ETH_STOP,                 /*!< ESP32 ethernet stop */
    SYSTEM_EVENT_ETH_CONNECTED,            /*!< ESP32 ethernet phy link up */
    SYSTEM_EVENT_ETH_DISCONNECTED,         /*!< ESP32 ethernet phy link down */
    SYSTEM_EVENT_ETH_GOT_IP,               /*!< ESP32 ethernet got IP from connected AP */
    SYSTEM_EVENT_MAX                       /*!< Number of members in this enum */
} system_event_id_t;

typedef enum
{
    NETWORK_INTERFACE_UNKNOWN,
    NETWORK_INTERFACE_WIFI,
    NETWORK_INTERFACE_ETH,
    NETWORK_INTERFACE_PPP,
} network_interface_t;

typedef enum
{
    NETWORK_INTERFACE_SRC_WIFI,
    NETWORK_INTERFACE_SRC_PPP,
    NETWORK_INTERFACE_SRC_ETH,
    NETWORK_INTERFACE_SRC_SMART_CFG
} network_interface_src_t;

typedef enum
{
    NETWORK_LOCAL_IP_WIFI,
    NETWORK_LOCAL_IP_ETH
} network_local_ip_t;


typedef void (*network_event_cb_t)(void *src, void *event, void *data, uint32_t size);




#endif /* APP_WIFI_ETH_H */