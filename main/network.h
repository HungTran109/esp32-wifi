#ifndef APP_WIFI_ETH_H
#define APP_WIFI_ETH_H

#include <stdint.h>
#include <stdbool.h>

#define BIT_WIFI_GOT_IP    BIT0
#define WIFI_FAIL_BIT      BIT1
typedef struct 
{
    char cmd[20];
    char ssid[50];
    char pass[16];
} WifiInfo_t;






#endif /* APP_WIFI_ETH_H */