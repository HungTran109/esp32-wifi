/******************************************************************************
 * @file:    app_cli.c
 * @brief:
 * @version: V0.0.0
 * @date:    2019/11/12
 * @author:
 * @email:
 *
 * THE SOURCE CODE AND ITS RELATED DOCUMENTATION IS PROVIDED "AS IS". VINSMART
 * JSC MAKES NO OTHER WARRANTY OF ANY KIND, WHETHER EXPRESS, IMPLIED OR,
 * STATUTORY AND DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * SATISFACTORY QUALITY, NON INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * THE SOURCE CODE AND DOCUMENTATION MAY INCLUDE ERRORS. VINSMART JSC
 * RESERVES THE RIGHT TO INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER
 * REVISIONS OF IT, AND TO MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR
 * THE PRODUCTS OR TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME.
 *
 * VINSMART JSC SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGE OR LIABILITY ARISING FROM YOUR USE OF THE SOURCE CODE OR
 * ANY DOCUMENTATION, INCLUDING BUT NOT LIMITED TO, LOST REVENUES, DATA OR
 * PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL OR CONSEQUENTIAL NATURE, PUNITIVE
 * DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS ARISING OUT OF OR IN CONNECTION
 * WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF ADVISED OF THE POSSIBILITY OR
 * PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM FOR SUCH DAMAGE IS BASED UPON
 * WARRANTY, CONTRACT, TORT, NEGLIGENCE OR OTHERWISE.
 *
 * (C)Copyright VINSMART JSC 2019 All rights reserved
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "app_cli.h"
#include "app_shell.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "network.h"

static const char *TAG = "cli";

static int32_t cli_reset(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_handle_wifi_cfg(p_shell_context_t context, int32_t argc, char **argv);

static const shell_command_context_t cli_command_table[] =
{
    {"reset", "\treset: Reset\r\n", cli_reset, 0},
    {"wifi", "\tconfig: Config wifi ssid, pass\r\n", cli_handle_wifi_cfg, 3},
};

static shell_context_struct m_user_context;
static app_cli_cb_t *m_cb;

void app_cli_poll(uint8_t ch)
{
    app_shell_task(ch);
}

void app_cli_start(app_cli_cb_t *callback)
{
    m_cb = callback;
    app_shell_set_context(&m_user_context);
    app_shell_init(&m_user_context,
                   m_cb->puts,
                   m_cb->printf,
                   ">",
                   true);

    /* Register CLI commands */
    for (int i = 0; i < sizeof(cli_command_table) / sizeof(shell_command_context_t); i++)
    {
        app_shell_register_cmd(&cli_command_table[i]);
    }

    /* Run CLI task */
    app_shell_task(APP_SHELL_INVALID_CHAR);
}

/* Reset System */
static int32_t cli_reset(p_shell_context_t context, int32_t argc, char **argv)
{
    esp_restart();
    return 0;
}

static void printf_hex_str(char *input, int len)
{
    printf("\r\n");
    printf("%d bytes\r\n", len);

    for (int i = 0; i < len; i++)
    {
        printf("%02X", input[i]);
    }
    printf("\r\n");
    fflush(stdout);
}

extern bool m_wifi_enable;
extern bool do_reconnect_wifi;
extern WifiInfo_t m_wifi_info;

static int32_t cli_handle_wifi_cfg(p_shell_context_t context, int32_t argc, char **argv)
{
    if (strstr(argv[1], "enable"))
    {
        sprintf(m_wifi_info.ssid,"%s", argv[2]);
        sprintf(m_wifi_info.pass,"%s", argv[3]);
        
        ESP_LOGI(TAG, "%s, %s", m_wifi_info.ssid, m_wifi_info.pass);
        m_wifi_enable = true;
        do_reconnect_wifi = true;
    }
    return 0;
}