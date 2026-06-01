/**
 * Firmware KIIRA – ESP32‑S3 (Compatível com ESP‑IDF v6.0.1)
 * - WiFi Manager com portal web para credenciais e IP do Backend
 * - Detecção de pedestres (PedestrianDetect)
 * - Buzzer PWM no pino GPIO3
 * - Display OLED SSD1306 via I2C0 (GPIO14/21)
 * - Câmera OV5640 via I2C1 com correções de artefatos e endianness
 * - Armazenamento no MicroSD com proteção contra remoção
 * - Upload para backend FastAPI com retentativas e monitoramento
 */

#include <string.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_check.h"
#include "pedestrian_detect.hpp"
#include "cJSON.h"

static const char *TAG = "kiira";
static PedestrianDetect *s_detect = nullptr;
static char device_id_str[18] = {0};

// ---------- Hardware da KIIRA ----------
#define BUZZER_PIN          GPIO_NUM_3
#define BUZZER_LEDC_CH      LEDC_CHANNEL_0
#define BUZZER_LEDC_TIMER   LEDC_TIMER_0
#define BUZZER_FREQ_HZ      2000
#define BUZZER_RESOLUTION   LEDC_TIMER_10_BIT
#define BUZZER_DUTY_ON      (512)

// Display SSD1306 (I2C0: SDA=14, SCL=21)
#define I2C_DISPLAY_PORT    I2C_NUM_0
#define DISPLAY_SDA_IO      GPIO_NUM_14
#define DISPLAY_SCL_IO      GPIO_NUM_21
#define SSD1306_ADDR        0x3C

// MicroSD (SDMMC 1-bit)
#define SDMMC_CLK_PIN       GPIO_NUM_39
#define SDMMC_CMD_PIN       GPIO_NUM_38
#define SDMMC_D0_PIN        GPIO_NUM_40

// Endpoints do backend
#define UPLOAD_ENDPOINT     "/upload"
#define CONFIG_ENDPOINT     "/config"
#define BACKEND_PORT        8000

// Parâmetros ajustáveis
static float min_score       = 0.5f;
static int   sleep_ms        = 2000;
static bool  enable_detection = true;

// WiFi Manager
#define WIFI_AP_SSID         "KIIRA-Config"
#define WIFI_CONNECT_TIMEOUT_MS  15000
#define MAX_SSID_LEN         32
#define MAX_PASS_LEN         64
#define NVS_NAMESPACE        "wifi"
#define NVS_KEY_SSID         "ssid"
#define NVS_KEY_PASS         "pass"
#define NVS_KEY_BACKEND_IP   "backend_ip"

typedef enum {
    WIFI_STATE_INIT,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_STA_CONNECTED,
    WIFI_STATE_AP_CONFIG
} wifi_state_t;

static wifi_state_t wifi_state = WIFI_STATE_INIT;
static char g_ssid[33] = "?";
static char g_ip[16]   = "?.?.?.?";
static char g_backend_ip[32] = "192.168.1.127"; // IP Padrão
static wifi_state_t g_display_state = WIFI_STATE_INIT;
static float g_last_score = 0.0;
static bool  g_detected   = false;
static volatile bool should_restart = false;

// SD Card
static bool sd_mounted = false;
static SemaphoreHandle_t sd_mutex = NULL;

// Backend Status
static int backend_fail_count = 0;
static bool backend_offline = false;

// Fonte 8x8 Completa (96 caracteres ASCII imprimíveis)
static const uint8_t font8x8[96][8] = {
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0020 (space)
 { 0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00},   // U+0021 (!)
 { 0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0022 (")
 { 0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36, 0x00},   // U+0023 (#)
 { 0x0C, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x0C, 0x00},   // U+0024 ($)
 { 0x00, 0x63, 0x33, 0x18, 0x0C, 0x66, 0x63, 0x00},   // U+0025 (%)
 { 0x1C, 0x36, 0x1C, 0x6E, 0x3B, 0x33, 0x6E, 0x00},   // U+0026 (&)
 { 0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0027 (')
 { 0x18, 0x0C, 0x06, 0x06, 0x06, 0x0C, 0x18, 0x00},   // U+0028 (()
 { 0x06, 0x0C, 0x18, 0x18, 0x18, 0x0C, 0x06, 0x00},   // U+0029 ())
 { 0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00},   // U+002A (*)
 { 0x00, 0x0C, 0x0C, 0x3F, 0x0C, 0x0C, 0x00, 0x00},   // U+002B (+)
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x06},   // U+002C (,)
 { 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00},   // U+002D (-)
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00},   // U+002E (.)
 { 0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00},   // U+002F (/)
 { 0x3E, 0x63, 0x73, 0x7B, 0x6F, 0x67, 0x3E, 0x00},   // U+0030 (0)
 { 0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F, 0x00},   // U+0031 (1)
 { 0x1E, 0x33, 0x30, 0x1C, 0x06, 0x33, 0x3F, 0x00},   // U+0032 (2)
 { 0x1E, 0x33, 0x30, 0x1C, 0x30, 0x33, 0x1E, 0x00},   // U+0033 (3)
 { 0x38, 0x3C, 0x36, 0x33, 0x7F, 0x30, 0x78, 0x00},   // U+0034 (4)
 { 0x3F, 0x03, 0x1F, 0x30, 0x30, 0x33, 0x1E, 0x00},   // U+0035 (5)
 { 0x1C, 0x06, 0x03, 0x1F, 0x33, 0x33, 0x1E, 0x00},   // U+0036 (6)
 { 0x3F, 0x33, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x00},   // U+0037 (7)
 { 0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E, 0x00},   // U+0038 (8)
 { 0x1E, 0x33, 0x33, 0x3E, 0x30, 0x18, 0x0E, 0x00},   // U+0039 (9)
 { 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x00},   // U+003A (:)
 { 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x06},   // U+003B (;)
 { 0x18, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x18, 0x00},   // U+003C (<)
 { 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F, 0x00, 0x00},   // U+003D (=)
 { 0x06, 0x0C, 0x18, 0x30, 0x18, 0x0C, 0x06, 0x00},   // U+003E (>)
 { 0x1E, 0x33, 0x30, 0x18, 0x0C, 0x00, 0x0C, 0x00},   // U+003F (?)
 { 0x3E, 0x63, 0x7B, 0x7B, 0x7B, 0x03, 0x1E, 0x00},   // U+0040 (@)
 { 0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00},   // U+0041 (A)
 { 0x3F, 0x66, 0x66, 0x3E, 0x66, 0x66, 0x3F, 0x00},   // U+0042 (B)
 { 0x3C, 0x66, 0x03, 0x03, 0x03, 0x66, 0x3C, 0x00},   // U+0043 (C)
 { 0x1F, 0x36, 0x66, 0x66, 0x66, 0x36, 0x1F, 0x00},   // U+0044 (D)
 { 0x7F, 0x46, 0x16, 0x1E, 0x16, 0x46, 0x7F, 0x00},   // U+0045 (E)
 { 0x7F, 0x46, 0x16, 0x1E, 0x16, 0x06, 0x0F, 0x00},   // U+0046 (F)
 { 0x3C, 0x66, 0x03, 0x03, 0x73, 0x66, 0x7C, 0x00},   // U+0047 (G)
 { 0x33, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33, 0x00},   // U+0048 (H)
 { 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+0049 (I)
 { 0x78, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E, 0x00},   // U+004A (J)
 { 0x67, 0x66, 0x36, 0x1E, 0x36, 0x66, 0x67, 0x00},   // U+004B (K)
 { 0x0F, 0x06, 0x06, 0x06, 0x46, 0x66, 0x7F, 0x00},   // U+004C (L)
 { 0x63, 0x77, 0x7F, 0x7F, 0x6B, 0x63, 0x63, 0x00},   // U+004D (M)
 { 0x63, 0x67, 0x6F, 0x7B, 0x73, 0x63, 0x63, 0x00},   // U+004E (N)
 { 0x1C, 0x36, 0x63, 0x63, 0x63, 0x36, 0x1C, 0x00},   // U+004F (O)
 { 0x3F, 0x66, 0x66, 0x3E, 0x06, 0x06, 0x0F, 0x00},   // U+0050 (P)
 { 0x1E, 0x33, 0x33, 0x33, 0x3B, 0x1E, 0x38, 0x00},   // U+0051 (Q)
 { 0x3F, 0x66, 0x66, 0x3E, 0x36, 0x66, 0x67, 0x00},   // U+0052 (R)
 { 0x1E, 0x33, 0x07, 0x0E, 0x38, 0x33, 0x1E, 0x00},   // U+0053 (S)
 { 0x3F, 0x2D, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+0054 (T)
 { 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3F, 0x00},   // U+0055 (U)
 { 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00},   // U+0056 (V)
 { 0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00},   // U+0057 (W)
 { 0x63, 0x63, 0x36, 0x1C, 0x1C, 0x36, 0x63, 0x00},   // U+0058 (X)
 { 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x0C, 0x1E, 0x00},   // U+0059 (Y)
 { 0x7F, 0x63, 0x31, 0x18, 0x4C, 0x66, 0x7F, 0x00},   // U+005A (Z)
 { 0x1E, 0x06, 0x06, 0x06, 0x06, 0x06, 0x1E, 0x00},   // U+005B ([)
 { 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x40, 0x00},   // U+005C (\)
 { 0x1E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1E, 0x00},   // U+005D (])
 { 0x08, 0x1C, 0x36, 0x63, 0x00, 0x00, 0x00, 0x00},   // U+005E (^)
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF},   // U+005F (_)
 { 0x0C, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0060 (`)
 { 0x00, 0x00, 0x1E, 0x30, 0x3E, 0x33, 0x6E, 0x00},   // U+0061 (a)
 { 0x07, 0x06, 0x06, 0x3E, 0x66, 0x66, 0x3B, 0x00},   // U+0062 (b)
 { 0x00, 0x00, 0x1E, 0x33, 0x03, 0x33, 0x1E, 0x00},   // U+0063 (c)
 { 0x38, 0x30, 0x30, 0x3E, 0x33, 0x33, 0x6E, 0x00},   // U+0064 (d)
 { 0x00, 0x00, 0x1E, 0x33, 0x3F, 0x03, 0x1E, 0x00},   // U+0065 (e)
 { 0x1C, 0x36, 0x06, 0x0F, 0x06, 0x06, 0x0F, 0x00},   // U+0066 (f)
 { 0x00, 0x00, 0x6E, 0x33, 0x33, 0x3E, 0x30, 0x1F},   // U+0067 (g)
 { 0x07, 0x06, 0x36, 0x6E, 0x66, 0x66, 0x67, 0x00},   // U+0068 (h)
 { 0x0C, 0x00, 0x0E, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+0069 (i)
 { 0x30, 0x00, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E},   // U+006A (j)
 { 0x07, 0x06, 0x66, 0x36, 0x1E, 0x36, 0x67, 0x00},   // U+006B (k)
 { 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+006C (l)
 { 0x00, 0x00, 0x33, 0x7F, 0x7F, 0x6B, 0x63, 0x00},   // U+006D (m)
 { 0x00, 0x00, 0x1F, 0x33, 0x33, 0x33, 0x33, 0x00},   // U+006E (n)
 { 0x00, 0x00, 0x1E, 0x33, 0x33, 0x33, 0x1E, 0x00},   // U+006F (o)
 { 0x00, 0x00, 0x3B, 0x66, 0x66, 0x3E, 0x06, 0x0F},   // U+0070 (p)
 { 0x00, 0x00, 0x6E, 0x33, 0x33, 0x3E, 0x30, 0x78},   // U+0071 (q)
 { 0x00, 0x00, 0x3B, 0x6E, 0x66, 0x06, 0x0F, 0x00},   // U+0072 (r)
 { 0x00, 0x00, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x00},   // U+0073 (s)
 { 0x08, 0x0C, 0x3E, 0x0C, 0x0C, 0x2C, 0x18, 0x00},   // U+0074 (t)
 { 0x00, 0x00, 0x33, 0x33, 0x33, 0x33, 0x6E, 0x00},   // U+0075 (u)
 { 0x00, 0x00, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00},   // U+0076 (v)
 { 0x00, 0x00, 0x63, 0x6B, 0x7F, 0x7F, 0x36, 0x00},   // U+0077 (w)
 { 0x00, 0x00, 0x63, 0x36, 0x1C, 0x36, 0x63, 0x00},   // U+0078 (x)
 { 0x00, 0x00, 0x33, 0x33, 0x33, 0x3E, 0x30, 0x1F},   // U+0079 (y)
 { 0x00, 0x00, 0x3F, 0x19, 0x0C, 0x26, 0x3F, 0x00},   // U+007A (z)
 { 0x38, 0x0C, 0x0C, 0x07, 0x0C, 0x0C, 0x38, 0x00},   // U+007B ({)
 { 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x00},   // U+007C (|)
 { 0x07, 0x0C, 0x0C, 0x38, 0x0C, 0x0C, 0x07, 0x00},   // U+007D (})
 { 0x6E, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+007E (~)
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}    // U+007F (DEL)
};

// ---------- Novo I2C (Display) ----------
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t ssd1306_dev = NULL;

static esp_err_t i2c_display_init(void) {
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.i2c_port = I2C_DISPLAY_PORT;
    bus_cfg.scl_io_num = DISPLAY_SCL_IO;
    bus_cfg.sda_io_num = DISPLAY_SDA_IO;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &i2c_bus_handle), TAG, "new bus failed");

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = SSD1306_ADDR;
    dev_cfg.scl_speed_hz = 400000;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &ssd1306_dev),
                        TAG, "add device failed");

    esp_err_t probe_ret = i2c_master_probe(i2c_bus_handle, SSD1306_ADDR, 1000);
    if (probe_ret != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 nao encontrado no endereco 0x%02X. Verifique a conexao!", SSD1306_ADDR);
        return probe_ret;
    }
    ESP_LOGI(TAG, "SSD1306 encontrado com sucesso.");
    return ESP_OK;
}

static esp_err_t ssd1306_write_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    return i2c_master_transmit(ssd1306_dev, buf, sizeof(buf), -1);
}

static esp_err_t ssd1306_write_data(const uint8_t *data, size_t len) {
    uint8_t *buf = (uint8_t*)malloc(len + 1);
    if (!buf) return ESP_ERR_NO_MEM;
    buf[0] = 0x40;
    memcpy(buf + 1, data, len);
    esp_err_t ret = i2c_master_transmit(ssd1306_dev, buf, len + 1, -1);
    free(buf);
    return ret;
}

void ssd1306_init(void) {
    ssd1306_write_cmd(0xAE); ssd1306_write_cmd(0xD5); ssd1306_write_cmd(0x80);
    ssd1306_write_cmd(0xA8); ssd1306_write_cmd(0x3F); ssd1306_write_cmd(0xD3);
    ssd1306_write_cmd(0x00); ssd1306_write_cmd(0x40); ssd1306_write_cmd(0x8D);
    ssd1306_write_cmd(0x14); ssd1306_write_cmd(0x20); ssd1306_write_cmd(0x00);
    ssd1306_write_cmd(0xA1); ssd1306_write_cmd(0xC8); ssd1306_write_cmd(0xDA);
    ssd1306_write_cmd(0x12); ssd1306_write_cmd(0x81); ssd1306_write_cmd(0xCF);
    ssd1306_write_cmd(0xD9); ssd1306_write_cmd(0xF1); ssd1306_write_cmd(0xDB);
    ssd1306_write_cmd(0x40); ssd1306_write_cmd(0xA4); ssd1306_write_cmd(0xA6);
    ssd1306_write_cmd(0x2E); ssd1306_write_cmd(0xAF);

    for (int page = 0; page < 8; page++) {
        ssd1306_write_cmd(0xB0 + page);
        ssd1306_write_cmd(0x00);
        ssd1306_write_cmd(0x10);
        for (int i = 0; i < 128; i++) {
            uint8_t z = 0;
            ssd1306_write_data(&z, 1);
        }
    }
}

void ssd1306_set_cursor(uint8_t page, uint8_t col) {
    ssd1306_write_cmd(0xB0 + page);
    ssd1306_write_cmd(0x00 + (col & 0x0F));
    ssd1306_write_cmd(0x10 + ((col >> 4) & 0x0F));
}

void ssd1306_draw_string(uint8_t page, uint8_t col, const char *str) {
    while (*str) {
        if (*str < 0x20 || *str > 0x7E) { str++; continue; }
        ssd1306_set_cursor(page, col);
        ssd1306_write_data(font8x8[*str - 0x20], 8);
        col += 8;
        if (col > 120) break;
        str++;
    }
}

void ssd1306_clear(void) {
    for (int page = 0; page < 8; page++) {
        ssd1306_set_cursor(page, 0);
        for (int i = 0; i < 128; i++) {
            uint8_t z = 0;
            ssd1306_write_data(&z, 1);
        }
    }
}

// Buzzer PWM
static void buzzer_beep(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CH, BUZZER_DUTY_ON);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CH);
    vTaskDelay(pdMS_TO_TICKS(200));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CH, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CH);
}

// SD Card
static esp_err_t sd_card_init(void) {
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk = SDMMC_CLK_PIN;
    slot.cmd = SDMMC_CMD_PIN;
    slot.d0  = SDMMC_D0_PIN;
    slot.width = 1;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(ret));
        return ret;
    }
    sd_mounted = true;
    sd_mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "SD card mounted.");
    return ESP_OK;
}

static void save_detection_to_sd(const uint8_t *jpg, size_t jpg_len, float score,
                                 int bx, int by, int bw, int bh) {
    if (!sd_mounted || !sd_mutex) return;
    if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) return;

    static int counter = 0;
    char fname[64];
    snprintf(fname, sizeof(fname), "/sdcard/detect_%04d.jpg", counter++);

    FILE *f = fopen(fname, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Falha ao abrir %s. SD Card removido?", fname);
        sd_mounted = false;
        xSemaphoreGive(sd_mutex);
        return;
    }

    size_t written = fwrite(jpg, 1, jpg_len, f);
    fclose(f);

    if (written != jpg_len) {
        ESP_LOGE(TAG, "Falha ao gravar imagem. SD Card removido?");
        sd_mounted = false;
        xSemaphoreGive(sd_mutex);
        return;
    }
    ESP_LOGI(TAG, "Saved %s (score=%.2f)", fname, score);

    FILE *log = fopen("/sdcard/detections.csv", "a");
    if (log) {
        if (fprintf(log, "%s,%.2f,%d,%d,%d,%d\n", fname, score, bx, by, bw, bh) < 0) {
            ESP_LOGE(TAG, "Falha ao gravar log. SD Card removido?");
            sd_mounted = false;
        }
        fclose(log);
    } else {
        sd_mounted = false;
    }
    xSemaphoreGive(sd_mutex);
}

// Wi-Fi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
            wifi_state = WIFI_STATE_CONNECTING;
            g_display_state = wifi_state;
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (wifi_state == WIFI_STATE_STA_CONNECTED) {
                wifi_state = WIFI_STATE_AP_CONFIG;
                g_display_state = wifi_state;
            }
        } else if (event_id == WIFI_EVENT_AP_START) {
            wifi_state = WIFI_STATE_AP_CONFIG;
            g_display_state = wifi_state;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(g_ip, sizeof(g_ip), IPSTR, IP2STR(&event->ip_info.ip));
        wifi_state = WIFI_STATE_STA_CONNECTED;
        g_display_state = wifi_state;
    }
}

// Servidor HTTP de configuração
static esp_err_t config_html_handler(httpd_req_t *req) {
    const char html[] = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>KIIRA Config</title></head>"
        "<body><h2>Configurar KIIRA</h2><form action='/save' method='post'>"
        "SSID: <input name='ssid' placeholder='SSID' required><br>"
        "Senha: <input name='pass' type='password' placeholder='Senha' required><br>"
        "IP Backend: <input name='ip' placeholder='192.168.1.127' value='%s'><br><br>"
        "<button type='submit'>Salvar e Conectar</button></form></body></html>";

    char html_buf[1024];
    snprintf(html_buf, sizeof(html_buf), html, g_backend_ip);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t config_save_handler(httpd_req_t *req) {
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) return httpd_resp_send_500(req);
    buf[ret] = '\0';

    char ssid[MAX_SSID_LEN] = {0}, pass[MAX_PASS_LEN] = {0}, ip[32] = {0};
    char *p = strtok(buf, "&");
    while (p) {
        if (strncmp(p, "ssid=", 5) == 0) strlcpy(ssid, p + 5, MAX_SSID_LEN);
        if (strncmp(p, "pass=", 5) == 0) strlcpy(pass, p + 5, MAX_PASS_LEN);
        if (strncmp(p, "ip=", 3) == 0) strlcpy(ip, p + 3, sizeof(ip));
        p = strtok(NULL, "&");
    }
    for (int i = 0; ssid[i]; i++) if (ssid[i] == '+') ssid[i] = ' ';
    for (int i = 0; pass[i]; i++) if (pass[i] == '+') pass[i] = ' ';
    for (int i = 0; ip[i]; i++) if (ip[i] == '+') ip[i] = ' ';

    if (strlen(ssid) == 0 || strlen(pass) == 0) {
        httpd_resp_set_status(req, "400");
        httpd_resp_sendstr(req, "Preencha SSID e senha.");
        return ESP_OK;
    }

    if (strlen(ip) > 0) {
        strlcpy(g_backend_ip, ip, sizeof(g_backend_ip));
    }

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, NVS_KEY_SSID, ssid);
        nvs_set_str(nvs, NVS_KEY_PASS, pass);
        nvs_set_str(nvs, NVS_KEY_BACKEND_IP, g_backend_ip);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    httpd_resp_sendstr(req, "OK. Reiniciando...");
    should_restart = true;
    return ESP_OK;
}

// Upload
static char* build_multipart_body(const uint8_t *jpg, size_t len, float score,
                                  int bx, int by, int bw, int bh, size_t *o) {
    const char *boundary = "----KiiraBoundary";
    char score_str[16];
    snprintf(score_str, sizeof(score_str), "%.2f", score);
    char tmp[256];
    int pre = snprintf(tmp, sizeof(tmp), "--%s\r\nContent-Disposition: form-data; name=\"file\"; filename=\"detect.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n", boundary);
    int post = 2;
    int d = snprintf(tmp, sizeof(tmp), "--%s\r\nContent-Disposition: form-data; name=\"device_id\"\r\n\r\n%s\r\n", boundary, device_id_str);
    int s = snprintf(tmp, sizeof(tmp), "--%s\r\nContent-Disposition: form-data; name=\"score\"\r\n\r\n%s\r\n", boundary, score_str);
    int bx_len = snprintf(tmp, sizeof(tmp), "--%s\r\nContent-Disposition: form-data; name=\"box_x\"\r\n\r\n%d\r\n", boundary, bx);
    int by_len = snprintf(tmp, sizeof(tmp), "--%s\r\nContent-Disposition: form-data; name=\"box_y\"\r\n\r\n%d\r\n", boundary, by);
    int bw_len = snprintf(tmp, sizeof(tmp), "--%s\r\nContent-Disposition: form-data; name=\"box_w\"\r\n\r\n%d\r\n", boundary, bw);
    int bh_len = snprintf(tmp, sizeof(tmp), "--%s\r\nContent-Disposition: form-data; name=\"box_h\"\r\n\r\n%d\r\n", boundary, bh);
    int cl = snprintf(tmp, sizeof(tmp), "--%s--\r\n", boundary);
    *o = pre + len + post + d + s + bx_len + by_len + bw_len + bh_len + cl;
    char *body = (char*)malloc(*o);
    if (!body) return NULL;
    char *ptr = body;
    ptr += sprintf(ptr, "--%s\r\nContent-Disposition: form-data; name=\"file\"; filename=\"detect.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n", boundary);
    memcpy(ptr, jpg, len); ptr += len;
    memcpy(ptr, "\r\n", 2); ptr += 2;
    ptr += sprintf(ptr, "--%s\r\nContent-Disposition: form-data; name=\"device_id\"\r\n\r\n%s\r\n", boundary, device_id_str);
    ptr += sprintf(ptr, "--%s\r\nContent-Disposition: form-data; name=\"score\"\r\n\r\n%s\r\n", boundary, score_str);
    ptr += sprintf(ptr, "--%s\r\nContent-Disposition: form-data; name=\"box_x\"\r\n\r\n%d\r\n", boundary, bx);
    ptr += sprintf(ptr, "--%s\r\nContent-Disposition: form-data; name=\"box_y\"\r\n\r\n%d\r\n", boundary, by);
    ptr += sprintf(ptr, "--%s\r\nContent-Disposition: form-data; name=\"box_w\"\r\n\r\n%d\r\n", boundary, bw);
    ptr += sprintf(ptr, "--%s\r\nContent-Disposition: form-data; name=\"box_h\"\r\n\r\n%d\r\n", boundary, bh);
    ptr += sprintf(ptr, "--%s--\r\n", boundary);
    return body;
}

static esp_err_t upload_image(const uint8_t *jpg, size_t len, float score,
                              int bx, int by, int bw, int bh) {
    size_t body_len;
    char *body = build_multipart_body(jpg, len, score, bx, by, bw, bh, &body_len);
    if (!body) return ESP_ERR_NO_MEM;

    char url[256];
    snprintf(url, sizeof(url), "http://%s:%d%s", g_backend_ip, BACKEND_PORT, UPLOAD_ENDPOINT);

    esp_err_t ret = ESP_FAIL;
    for (int attempt = 0; attempt < 3; attempt++) {
        esp_http_client_config_t cfg = {};
        cfg.url = url;
        cfg.method = HTTP_METHOD_POST;
        cfg.timeout_ms = 5000;
        esp_http_client_handle_t client = esp_http_client_init(&cfg);
        esp_http_client_set_header(client, "Content-Type", "multipart/form-data; boundary=----KiiraBoundary");
        esp_http_client_set_post_field(client, body, body_len);

        ret = esp_http_client_perform(client);
        if (ret == ESP_OK) {
            int status = esp_http_client_get_status_code(client);
            if (status >= 200 && status < 300) {
                ESP_LOGI(TAG, "Upload HTTP %d", status);
                esp_http_client_cleanup(client);
                free(body);
                return ESP_OK;
            }
        }
        ESP_LOGW(TAG, "Upload tentativa %d falhou: %s", attempt + 1, esp_err_to_name(ret));
        esp_http_client_cleanup(client);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    free(body);
    return ret;
}

static void fetch_config(void) {
    char url[256];
    snprintf(url, sizeof(url), "http://%s:%d%s?device_id=%s", g_backend_ip, BACKEND_PORT, CONFIG_ENDPOINT, device_id_str);
    esp_http_client_config_t cfg = {};
    cfg.url = url;
    cfg.method = HTTP_METHOD_GET;
    cfg.timeout_ms = 3000;
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (esp_http_client_perform(client) == ESP_OK && esp_http_client_get_status_code(client) == 200) {
        int len = esp_http_client_get_content_length(client);
        if (len > 0 && len < 512) {
            char *buf = (char*)malloc(len + 1);
            if (buf) {
                esp_http_client_read(client, buf, len);
                buf[len] = 0;
                cJSON *root = cJSON_Parse(buf);
                if (root) {
                    cJSON *item = cJSON_GetObjectItem(root, "min_score");
                    if (item) min_score = item->valuedouble;
                    item = cJSON_GetObjectItem(root, "sleep_ms");
                    if (item) sleep_ms = item->valueint;
                    item = cJSON_GetObjectItem(root, "detect");
                    if (item) enable_detection = (item->valueint != 0);
                    cJSON_Delete(root);
                }
                free(buf);
            }
        }
    }
    esp_http_client_cleanup(client);
}

// Câmera (Otimizada para OV5640 + ESP-DL)
static esp_err_t camera_init(void) {
    camera_config_t cfg = {};
    cfg.pin_pwdn = -1;
    cfg.pin_reset = -1;
    cfg.pin_xclk = 15;
    cfg.pin_sccb_sda = 4;
    cfg.pin_sccb_scl = 5;
    cfg.pin_d7 = 16; cfg.pin_d6 = 17; cfg.pin_d5 = 18; cfg.pin_d4 = 12;
    cfg.pin_d3 = 10; cfg.pin_d2 = 8;  cfg.pin_d1 = 9;  cfg.pin_d0 = 11;
    cfg.pin_vsync = 6;
    cfg.pin_href = 7;
    cfg.pin_pclk = 13;

    // OV5640 PRECISA de 20MHz para gerar RGB565 válido para a IA
    cfg.xclk_freq_hz = 20000000; 

    cfg.ledc_timer = LEDC_TIMER_0;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.pixel_format = PIXFORMAT_RGB565;
    cfg.frame_size = FRAMESIZE_VGA;
    cfg.jpeg_quality = 12;

    // 3 buffers evitam o "Tearing" (glitch horizontal) pois o DMA tem mais folga
    cfg.fb_count = 3; 
    cfg.fb_location = CAMERA_FB_IN_PSRAM;

    // WHEN_EMPTY garante que a IA leia um buffer que NÃO está sendo gravado pelo DMA
    cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY; 
    cfg.sccb_i2c_port = 1; 

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    // Patch específico para o OV5640 corrigir cores psicodélicas e estabilizar ISP
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_wb_mode(s, 0); // Auto White Balance
        s->set_aec2(s, 1);
        s->set_ae_level(s, 0);
    }

    // Descartar frames iniciais para o AWB/AEC estabilizar
    ESP_LOGI(TAG, "Aguardando estabilizacao do sensor OV5640...");
    for (int i = 0; i < 5; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return ESP_OK;
}

static void update_display(void) {
    ssd1306_clear();
    if (g_display_state == WIFI_STATE_AP_CONFIG) {
        ssd1306_draw_string(0, 0, "AP: KIIRA-Config");
        ssd1306_draw_string(2, 0, "IP: 192.168.4.1");
        ssd1306_draw_string(4, 0, "Aguardando config");
    } else {
        ssd1306_draw_string(0, 0, "STA:");
        ssd1306_draw_string(0, 30, g_ssid);
        ssd1306_draw_string(2, 0, "IP:");
        ssd1306_draw_string(2, 30, g_ip);
        char buf[20];
        if (g_detected) snprintf(buf, sizeof(buf), "Det:%.2f", g_last_score);
        else snprintf(buf, sizeof(buf), "Det:none");
        ssd1306_draw_string(4, 0, buf);
    }

    if (backend_offline) {
        ssd1306_draw_string(6, 0, "BACKEND OFFLINE");
    } else {
        ssd1306_draw_string(6, 0, "Backend: OK    ");
    }
}

static void init_device_id(void) {
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(device_id_str, sizeof(device_id_str), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void draw_rect_rgb565(uint8_t *buf, int fbw, int fbh,
                             int x, int y, int w, int h, uint16_t color) {
    int x0 = (x < 0) ? 0 : x;
    int y0 = (y < 0) ? 0 : y;
    int x1 = ((x + w) > fbw)  ? fbw  - 1 : (x + w);
    int y1 = ((y + h) > fbh) ? fbh - 1 : (y + h);
    for (int col = x0; col <= x1; col++) {
        ((uint16_t *)buf)[y0 * fbw + col] = color;
        ((uint16_t *)buf)[y1 * fbw + col] = color;
    }
    for (int row = y0; row <= y1; row++) {
        ((uint16_t *)buf)[row * fbw + x0] = color;
        ((uint16_t *)buf)[row * fbw + x1] = color;
    }
}

// WiFi Manager
static void wifi_manager(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    char saved_ssid[MAX_SSID_LEN] = {0}, saved_pass[MAX_PASS_LEN] = {0};
    bool has = false;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        size_t len = sizeof(saved_ssid);
        if (nvs_get_str(nvs, NVS_KEY_SSID, saved_ssid, &len) == ESP_OK) {
            len = sizeof(saved_pass);
            nvs_get_str(nvs, NVS_KEY_PASS, saved_pass, &len);
            has = true;
        }
        nvs_close(nvs);
    }

    if (has && strlen(saved_ssid) > 0) {
        strlcpy(g_ssid, saved_ssid, sizeof(g_ssid));
        wifi_config_t wcfg = {};
        strlcpy((char*)wcfg.sta.ssid, saved_ssid, sizeof(wcfg.sta.ssid));
        strlcpy((char*)wcfg.sta.password, saved_pass, sizeof(wcfg.sta.password));
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_set_config(WIFI_IF_STA, &wcfg);
        esp_wifi_start();

        TickType_t start = xTaskGetTickCount();
        while (wifi_state != WIFI_STATE_STA_CONNECTED) {
            if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS)) break;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        if (wifi_state == WIFI_STATE_STA_CONNECTED) return;
        esp_wifi_stop();
    }

    wifi_config_t apcfg = {};
    strlcpy((char*)apcfg.ap.ssid, WIFI_AP_SSID, sizeof(apcfg.ap.ssid));
    apcfg.ap.ssid_len = strlen(WIFI_AP_SSID);
    apcfg.ap.channel = 1;
    apcfg.ap.authmode = WIFI_AUTH_OPEN;
    apcfg.ap.max_connection = 4;

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &apcfg);
    esp_wifi_start();

    httpd_handle_t server = NULL;
    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    httpd_start(&server, &hcfg);

    httpd_uri_t root = {};
    root.uri = "/";
    root.method = HTTP_GET;
    root.handler = config_html_handler;
    httpd_register_uri_handler(server, &root);

    httpd_uri_t save = {};
    save.uri = "/save";
    save.method = HTTP_POST;
    save.handler = config_save_handler;
    httpd_register_uri_handler(server, &save);

    while (!should_restart) {
        update_display();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    httpd_stop(server);
    esp_wifi_stop();
    esp_restart();
}

extern "C" void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        size_t len = sizeof(g_backend_ip);
        if (nvs_get_str(nvs, NVS_KEY_BACKEND_IP, g_backend_ip, &len) != ESP_OK) {
            strcpy(g_backend_ip, "192.168.1.127");
        }
        nvs_close(nvs);
    }

    i2c_display_init();
    ssd1306_init();
    ssd1306_draw_string(0, 0, "Iniciando...");

    wifi_manager();
    init_device_id();
    sd_card_init();
    camera_init();

    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = BUZZER_RESOLUTION;
    timer.timer_num = BUZZER_LEDC_TIMER;
    timer.freq_hz = BUZZER_FREQ_HZ;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {};
    ch.gpio_num = BUZZER_PIN;
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.channel = BUZZER_LEDC_CH;
    ch.timer_sel = BUZZER_LEDC_TIMER;
    ch.duty = 0;
    ledc_channel_config(&ch);

    s_detect = new PedestrianDetect();
    if (!s_detect) { ESP_LOGE(TAG, "Detector falhou"); return; }
    fetch_config();

    bool last_detected = false;
    TickType_t last_fetch = xTaskGetTickCount();
    TickType_t last_disp = 0;

    while (1) {
        if (!enable_detection) {
            vTaskDelay(pdMS_TO_TICKS(sleep_ms));
            continue;
        }
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

        dl::image::img_t img = {
            .data     = fb->buf,
            .width    = (uint16_t)fb->width,
            .height   = (uint16_t)fb->height,
            // OV5640 usa Big Endian. Se as cores no backend ficarem trocadas, mude para DL_IMAGE_PIX_TYPE_RGB565LE
            .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE, 
        };

        auto &res = s_detect->run(img);
        bool det = !res.empty();

        if (det) {
            g_detected = true;
            g_last_score = res.front().score;
            for (auto &r : res) {
                draw_rect_rgb565(fb->buf, fb->width, fb->height,
                                 r.box[0], r.box[1], r.box[2], r.box[3], 0xF800);
            }
            if (!last_detected) buzzer_beep();

            uint8_t *jpg = NULL;
            size_t jpg_len = 0;
            if (fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
                        PIXFORMAT_RGB565, 75, &jpg, &jpg_len)) {

                esp_err_t up_ret = upload_image(jpg, jpg_len, res.front().score,
                             res.front().box[0], res.front().box[1],
                             res.front().box[2], res.front().box[3]);

                if (up_ret == ESP_OK) {
                    backend_fail_count = 0;
                    backend_offline = false;
                } else {
                    backend_fail_count++;
                    if (backend_fail_count >= 10) {
                        backend_offline = true;
                    }
                }

                save_detection_to_sd(jpg, jpg_len, res.front().score,
                                     res.front().box[0], res.front().box[1],
                                     res.front().box[2], res.front().box[3]);
                free(jpg);
            }
        } else {
            g_detected = false;
        }
        esp_camera_fb_return(fb);
        last_detected = det;

        if ((xTaskGetTickCount() - last_fetch) > pdMS_TO_TICKS(30000)) {
            fetch_config();
            last_fetch = xTaskGetTickCount();
        }
        if ((xTaskGetTickCount() - last_disp) > pdMS_TO_TICKS(2000)) {
            update_display();
            last_disp = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(sleep_ms));
    }
}