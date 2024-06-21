#ifndef APP_WIFI_ETH_H
#define APP_WIFI_ETH_H

#include <stdint.h>
#include <stdbool.h>

typedef struct 
{
    char cmd[20];
    char ssid[50];
    char pass[16];
    uint8_t status;
} WifiInfo_t;




#endif /* APP_WIFI_ETH_H */