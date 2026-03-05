#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/twai.h"

/*
  ESP32-S3 CAN sniffer bridge for SavvyCAN

    - Listen on TWAI (CAN) and stream frames out over UART
    - SavvyCAN connects and shows frames live

    - TWAI_TX -> transceiver TXD
    - TWAI_RX -> transceiver RXD
    - CANH/CANL -> bus

*/

static const char *TAG = "gvret";

#define CAN_TX GPIO_NUM_5
#define CAN_RX GPIO_NUM_4

#define DEFAULT_BITRATE 500000 // 500k is common default, try 250k or 125k if unsuccessful

// UART0 is wired to the USB-serial on most esp32 devkits, so you can connect savvyCAN directy to it.
#define GVRET_UART UART_NUM_0
#define GVRET_TX GPIO_NUM_43
#define GVRET_RX GPIO_NUM_44
#define GVRET_BAUD 1000000





#define UART_RX_BUF 2048

// GVRET state
static bool bin_mode = true;
static uint32_t can_bitrate = DEFAULT_BITRATE;
static bool can_on = true;
static bool listen_only = false;           // true = just sniffing (safe)
static const uint8_t NUM_BUSES = 1;       // we only do bus0 here

static void uart_send(const uint8_t *buf, int len)
{
    if (!buf || len <= 0) return;
    uart_write_bytes(GVRET_UART, (const char *)buf, len);
}

/*
  GVRET messages are basically:
    0xF1, cmd, payload..., 0x00

  (0x00 is the terminator)
*/
static void send_reply(uint8_t cmd, const uint8_t *payload, size_t payload_len)
{
    uint8_t out[64];
    size_t idx = 0;

    out[idx++] = 0xF1;
    out[idx++] = cmd;

    if (payload && payload_len)
    {
        size_t max_len = sizeof(out) - 3;
        if (payload_len > max_len) payload_len = max_len;

        memcpy(&out[idx], payload, payload_len);
        idx += payload_len;
    }

    out[idx++] = 0x00;
    uart_send(out, (int)idx);
}

static void reply_keepalive(void)
{
    // ping DE AD to keep SavvyCan connected.
    uint8_t p[2] = {0xDE, 0xAD};
    send_reply(0x09, p, sizeof(p));
}

static void reply_time(void)
{
    // just keeping track of current time since boot in microseconds for sync
    uint32_t us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFu);

    uint8_t p[4] = {
        (uint8_t)(us & 0xFF),
        (uint8_t)((us >> 8) & 0xFF),
        (uint8_t)((us >> 16) & 0xFF),
        (uint8_t)((us >> 24) & 0xFF),
    };

    send_reply(0x01, p, sizeof(p));
}

static void reply_device_info(void)
{
    // SavvyCAN asks for this.
    uint16_t build = 1;
    uint8_t eeprom_ver = 1;
    uint8_t file_out_type = 0;
    uint8_t autostart = 0;
    uint8_t singlewire = 0;

    uint8_t p[6] = {
        (uint8_t)(build & 0xFF),
        (uint8_t)((build >> 8) & 0xFF),
        eeprom_ver,
        file_out_type,
        autostart,
        singlewire,
    };

    send_reply(0x07, p, sizeof(p));
}

static void reply_bus_config(void)
{
    // bit0 = enable, bit4 = listen-only
    uint8_t flags = 0;
    if (can_on) flags |= 0x01;
    if (listen_only) flags |= 0x10;

    uint32_t spd = can_bitrate;

    uint8_t p[5] = {
        flags,
        (uint8_t)(spd & 0xFF),
        (uint8_t)((spd >> 8) & 0xFF),
        (uint8_t)((spd >> 16) & 0xFF),
        (uint8_t)((spd >> 24) & 0xFF),
    };

    send_reply(0x06, p, sizeof(p));
}

static void reply_num_buses(void)
{
    uint8_t p[1] = {NUM_BUSES};
    send_reply(0x0C, p, sizeof(p));
}

static esp_err_t start_can(uint32_t bitrate, bool want_listen_only)
{
    (void)twai_stop();
    (void)twai_driver_uninstall();

    twai_timing_config_t tcfg;
    switch (bitrate)
    {
    case 125000:  tcfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS(); break;
    case 250000:  tcfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS(); break;
    case 500000:  tcfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS(); break;
    case 1000000: tcfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS(); break;
    default:
        ESP_LOGW(TAG, "bitrate %u not in my little switch-case, defaulting to %u",
                 (unsigned)bitrate, (unsigned)DEFAULT_BITRATE);
        bitrate = DEFAULT_BITRATE;
        tcfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
        break;
    }

    twai_general_config_t gcfg = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    gcfg.mode = want_listen_only ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;

    // accept everything
    twai_filter_config_t fcfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&gcfg, &tcfg, &fcfg);
    if (err != ESP_OK) return err;

    err = twai_start();
    if (err != ESP_OK) return err;

    can_bitrate = bitrate;
    listen_only = want_listen_only;
    can_on = true;

    return ESP_OK;
}

/*
  Send a CAN frame to host in GVRET rx format

    0xF1 0x00
    timestamp u32 (us)
    id u32 (bit31 set means extended)
    (bus<<4) | dlc
    data bytes
    0x00
*/
static void send_can_to_host(const twai_message_t *m)
{
    uint8_t out[64];
    size_t idx = 0;

    uint32_t ts = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFu);

    uint32_t id = m->identifier;
    if (m->extd) id |= (1u << 31);

    uint8_t dlc = (m->data_length_code > 8) ? 8 : m->data_length_code;
    uint8_t bus = 0;

    out[idx++] = 0xF1;
    out[idx++] = 0x00;

    out[idx++] = (uint8_t)(ts & 0xFF);
    out[idx++] = (uint8_t)((ts >> 8) & 0xFF);
    out[idx++] = (uint8_t)((ts >> 16) & 0xFF);
    out[idx++] = (uint8_t)((ts >> 24) & 0xFF);

    out[idx++] = (uint8_t)(id & 0xFF);
    out[idx++] = (uint8_t)((id >> 8) & 0xFF);
    out[idx++] = (uint8_t)((id >> 16) & 0xFF);
    out[idx++] = (uint8_t)((id >> 24) & 0xFF);

    out[idx++] = (uint8_t)(((bus & 0x0F) << 4) | (dlc & 0x0F));

    for (int i = 0; i < dlc; i++) out[idx++] = m->data[i];

    out[idx++] = 0x00;
    uart_send(out, (int)idx);
}

//GVRET command handler.
static void handle_cmd(uint8_t cmd, const uint8_t *payload, size_t payload_len)
{
    switch (cmd)
    {
    case 0x01: reply_time(); break;
    case 0x06: reply_bus_config(); break;
    case 0x07: reply_device_info(); break;
    case 0x09: reply_keepalive(); break;
    case 0x0C: reply_num_buses(); break;

    case 0x05:
    {
        if (payload_len < 4) break;

        uint32_t cfg = ((uint32_t)payload[0]) |
                       ((uint32_t)payload[1] << 8) |
                       ((uint32_t)payload[2] << 16) |
                       ((uint32_t)payload[3] << 24);

        bool use_it   = (cfg & 0x80000000u) != 0;
        bool enable   = (cfg & 0x40000000u) != 0;
        bool sniff    = (cfg & 0x20000000u) != 0;
        uint32_t spd  = (cfg & 0x1FFFFFFFu);

        if (use_it)
        {
            if (!enable)
            {
                (void)twai_stop();
                (void)twai_driver_uninstall();
                can_on = false;
            }
            else
            {
                esp_err_t err = start_can(spd, sniff);
            }
        }
        break;
    }

    case 0x00:
    {
        // transmit frame from host -> CAN bus.
        // this'll fail in listen only mode
        if (payload_len < 5) break;

        uint32_t id = ((uint32_t)payload[0]) |
                      ((uint32_t)payload[1] << 8) |
                      ((uint32_t)payload[2] << 16) |
                      ((uint32_t)payload[3] << 24);

        bool ext = false;
        if (id & 0x80000000u)
        {
            ext = true;
            id &= 0x7FFFFFFFu;
        }

        uint8_t dlc = payload[4] & 0x0F;
        if (dlc > 8) dlc = 8;

        twai_message_t msg = {0};
        msg.identifier = id;
        msg.extd = ext;
        msg.data_length_code = dlc;

        for (int i = 0; i < dlc && (5 + i) < (int)payload_len; i++)
            msg.data[i] = payload[5 + i];

        esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
        if (err != ESP_OK)
            ESP_LOGW(TAG, "tx failed (probably listen-only): %s", esp_err_to_name(err));

        break;
    }

    default:
        break;
    }
}

static void uart_rx_task(void *arg)
{
    (void)arg;

    uint8_t rx[256];

    // little state machine for: 0xF1 <cmd> <payload...> 0x00
    enum { WAITING, GOT_F1, IN_PAYLOAD } st = WAITING;

    uint8_t cmd = 0;
    uint8_t payload[32];
    size_t pay_len = 0;

    while (1)
    {
        int n = uart_read_bytes(GVRET_UART, rx, sizeof(rx), pdMS_TO_TICKS(50));
        if (n <= 0) continue;

        for (int i = 0; i < n; i++)
        {
            uint8_t b = rx[i];

            // SavvyCAN yells 0xE7 at you to force binary mode.
            if (st == WAITING && b == 0xE7)
            {
                bin_mode = true;
                continue;
            }

            if (!bin_mode)
            {
                // I don't need text mode.
                continue;
            }

            switch (st)
            {
            case WAITING:
                if (b == 0xF1) st = GOT_F1;
                break;

            case GOT_F1:
                cmd = b;
                pay_len = 0;
                st = IN_PAYLOAD;
                break;

            case IN_PAYLOAD:
                if (b == 0x00)
                {
                    handle_cmd(cmd, payload, pay_len);
                    st = WAITING;
                }
                else
                {
                    if (pay_len < sizeof(payload))
                        payload[pay_len++] = b;
                    // else just drop it.
                }
                break;
            }
        }
    }
}

static void can_rx_task(void *arg)
{
    (void)arg;

    twai_message_t msg;

    while (1)
    {
        esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(100));
        if (err == ESP_OK)
        {
            if (bin_mode)
                send_can_to_host(&msg);
        }
    }
}

static void setup_uart(void)
{
    uart_config_t cfg = {
        .baud_rate = GVRET_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(GVRET_UART, UART_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GVRET_UART, &cfg));

    if (GVRET_UART != UART_NUM_0)
    {
        ESP_ERROR_CHECK(uart_set_pin(GVRET_UART, GVRET_TX, GVRET_RX,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    }
}

void app_main(void)
{
    //had to keep logs quiet on the streamm
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    setup_uart();

    ESP_ERROR_CHECK(start_can(DEFAULT_BITRATE, false));


    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(can_rx_task, "can_rx", 4096, NULL, 9, NULL, 1);
}