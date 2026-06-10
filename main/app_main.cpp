/**
 * Firmware KIIRA – ESP32‑S3 (Compatível com ESP‑IDF v6.0.1)
 * - WiFi Manager com portal web para credenciais e IP do Backend
 * - Detecção de pedestres (PedestrianDetect)
 * - Buzzer PWM no pino GPIO3
 * - Display OLED SSD1306 via I2C0 (GPIO14/21)
 * - Câmera OV5640 via I2C1 com correções de artefatos e endianness
 * - Armazenamento no MicroSD com proteção contra remoção
 * - Upload para backend FastAPI com retentativas e monitoramento
 *
 * ====================================================================
 *  ESTRATÉGIA ANTI-CORRUPÇÃO (v2 — captura serializada)
 * --------------------------------------------------------------------
 *  A corrupção em faixas (bandas arco-íris) é STARVATION de DMA: o GDMA
 *  da câmera escreve o frame na PSRAM enquanto a CPU martela a PSRAM
 *  (inferência + fmt2jpg) e/ou a flash (faltas de cache do WiFi) no
 *  mesmo controlador de memória externa (MSPI). Linhas se perdem -> lixo.
 *
 *  Princípio-chave: um frame só corre risco DURANTE a própria escrita do
 *  DMA. Enquanto o framebuffer está SEGURADO (sem esp_camera_fb_return),
 *  o DMA não escreve nele — logo inferência e fmt2jpg sobre o buffer
 *  segurado são imunes. Toda a corrupção vinha de capturar o keeper no
 *  meio do loop que estava inferindo/encodando o frame anterior.
 *
 *  Solução aplicada:
 *   1) ST_VERIFY roda SÓ inferência (sem fmt2jpg) — remove o maior
 *      poluidor de PSRAM do laço de captura.
 *   2) ST_EVIDENCE captura cada keeper em JANELA SILENCIOSA: dreno de
 *      frames com a CPU ociosa -> grab limpo -> segura o buffer ->
 *      inferência + JPEG sobre o buffer segurado (frame que o DMA
 *      escreve nesse meio tempo é sacrificial e descartado no próximo
 *      dreno).
 *   3) Upload e SD acontecem DEPOIS da fase de captura, nunca no meio.
 *   4) A FSM roda numa task fixada no core 1; o WiFi fica no core 0.
 * ====================================================================
 */

#include <string.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_attr.h"
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
// IMPORTANTE: a câmera usa LEDC_TIMER_0 + LEDC_CHANNEL_0 para gerar o XCLK.
// O buzzer PRECISA usar timer e canal diferentes, senão cada beep reconfigura
// o clock do sensor e o frame seguinte sai corrompido. Por isso TIMER_1/CH1.
#define BUZZER_LEDC_CH      LEDC_CHANNEL_1
#define BUZZER_LEDC_TIMER   LEDC_TIMER_1
#define BUZZER_FREQ_HZ      5000          // KIIRA: saída PWM de LED/Buzzer a 5 kHz
#define BUZZER_RESOLUTION   LEDC_TIMER_10_BIT
#define BUZZER_DUTY_ON      (512)         // ~50% de 1023 (10 bits)

// Gatilho de presença: sensor PIR no GPIO41 (pull-down, borda de subida).
#define BUTTON1_GPIO        GPIO_NUM_41

// ===== Gatilho + FSM de detecção sob demanda =====
// Em repouso a INFERÊNCIA fica desligada (parte cara); a câmera segue aquecida
// (sensor ligado, AEC/AWB adaptando) para que o gatilho seja instantâneo.
#define TRIGGER_DEBOUNCE_MS   200     // ignora repique do gatilho
#define KEEPALIVE_MS          200     // dreno de 1 frame em repouso (mantém AEC quente)
#define VERIFY_WINDOW_MS      1500    // janela máx. para confirmar presença
#define VERIFY_MAX_FRAMES     8       // teto de frames analisados na verificação
#define CONFIRM_HITS          2       // nº de acertos p/ confirmar (anti-falso-positivo)
#define HIGH_CONFIDENCE       0.85f   // 1 frame >= isto confirma na hora (caminho rápido)
#define MAX_EVIDENCE          3       // nº de evidências guardadas/enviadas
#define EVIDENCE_SPACING_MS   400     // intervalo entre evidências (mostra deslocamento)
#define COOLDOWN_MS           5000    // após registrar, ignora gatilhos

// ---- Parâmetros da captura em janela silenciosa ----
// DRENO: nº de frames descartados (grab+return) com a CPU ociosa antes de cada
// keeper. Deve ser > fb_count para garantir que os buffers escritos durante a
// contenção anterior sejam totalmente reciclados em condição limpa.
#define EVIDENCE_DRAIN_FRAMES    5
#define EVIDENCE_JPEG_QUALITY    90    // qualidade JPEG do fmt2jpg (0-100, maior = melhor)
#define EVIDENCE_TOTAL_WINDOW_MS 8000  // teto de tempo p/ coletar MAX_EVIDENCE

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
static const char *g_state_str = "ARMADO";
static volatile bool should_restart = false;

// SD Card
static bool sd_mounted = false;
static SemaphoreHandle_t sd_mutex = NULL;

// Backend Status
static int backend_fail_count = 0;
static bool backend_offline = false;

// Fonte 8x8 Completa (96 caracteres ASCII imprimíveis)
static const uint8_t font8x8[97][8] = {
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0020 (space)
 { 0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x00},   // U+0021 (!)
 { 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00},   // U+0022 (")
 { 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, 0x00, 0x00},   // U+0023 (#)
 { 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00, 0x00, 0x00},   // U+0024 ($)
 { 0x23, 0x13, 0x08, 0x64, 0x62, 0x00, 0x00, 0x00},   // U+0025 (%)
 { 0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x00, 0x00},   // U+0026 (&)
 { 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0027 (')
 { 0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x00, 0x00},   // U+0028 (()
 { 0x00, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00},   // U+0029 ())
 { 0x14, 0x08, 0x3E, 0x08, 0x14, 0x00, 0x00, 0x00},   // U+002A (*)
 { 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00, 0x00},   // U+002B (+)
 { 0x00, 0x50, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+002C (,)
 { 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00},   // U+002D (-)
 { 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+002E (.)
 { 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00, 0x00},   // U+002F (/)
 { 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00, 0x00},   // U+0030 (0)
 { 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 0x00, 0x00},   // U+0031 (1)
 { 0x42, 0x61, 0x51, 0x49, 0x46, 0x00, 0x00, 0x00},   // U+0032 (2)
 { 0x21, 0x41, 0x45, 0x4B, 0x31, 0x00, 0x00, 0x00},   // U+0033 (3)
 { 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 0x00, 0x00},   // U+0034 (4)
 { 0x27, 0x45, 0x45, 0x45, 0x39, 0x00, 0x00, 0x00},   // U+0035 (5)
 { 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, 0x00, 0x00},   // U+0036 (6)
 { 0x01, 0x71, 0x09, 0x05, 0x03, 0x00, 0x00, 0x00},   // U+0037 (7)
 { 0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00},   // U+0038 (8)
 { 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00, 0x00},   // U+0039 (9)
 { 0x00, 0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+003A (:)
 { 0x00, 0x56, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+003B (;)
 { 0x08, 0x14, 0x22, 0x41, 0x00, 0x00, 0x00, 0x00},   // U+003C (<)
 { 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00, 0x00},   // U+003D (=)
 { 0x00, 0x41, 0x22, 0x14, 0x08, 0x00, 0x00, 0x00},   // U+003E (>)
 { 0x02, 0x01, 0x51, 0x09, 0x06, 0x00, 0x00, 0x00},   // U+003F (?)
 { 0x32, 0x49, 0x79, 0x41, 0x3E, 0x00, 0x00, 0x00},   // U+0040 (@)
 { 0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00, 0x00, 0x00},   // U+0041 (A)
 { 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00},   // U+0042 (B)
 { 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, 0x00, 0x00},   // U+0043 (C)
 { 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00},   // U+0044 (D)
 { 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, 0x00, 0x00},   // U+0045 (E)
 { 0x7F, 0x09, 0x09, 0x09, 0x01, 0x00, 0x00, 0x00},   // U+0046 (F)
 { 0x3E, 0x41, 0x49, 0x49, 0x7A, 0x00, 0x00, 0x00},   // U+0047 (G)
 { 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x00, 0x00},   // U+0048 (H)
 { 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, 0x00, 0x00},   // U+0049 (I)
 { 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, 0x00, 0x00},   // U+004A (J)
 { 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00, 0x00},   // U+004B (K)
 { 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00},   // U+004C (L)
 { 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00, 0x00, 0x00},   // U+004D (M)
 { 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, 0x00, 0x00},   // U+004E (N)
 { 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, 0x00, 0x00},   // U+004F (O)
 { 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00},   // U+0050 (P)
 { 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, 0x00, 0x00},   // U+0051 (Q)
 { 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, 0x00, 0x00},   // U+0052 (R)
 { 0x46, 0x49, 0x49, 0x49, 0x31, 0x00, 0x00, 0x00},   // U+0053 (S)
 { 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00, 0x00, 0x00},   // U+0054 (T)
 { 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, 0x00, 0x00},   // U+0055 (U)
 { 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, 0x00, 0x00},   // U+0056 (V)
 { 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00, 0x00, 0x00},   // U+0057 (W)
 { 0x63, 0x14, 0x08, 0x14, 0x63, 0x00, 0x00, 0x00},   // U+0058 (X)
 { 0x07, 0x08, 0x70, 0x08, 0x07, 0x00, 0x00, 0x00},   // U+0059 (Y)
 { 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x00, 0x00},   // U+005A (Z)
 { 0x00, 0x7F, 0x41, 0x41, 0x00, 0x00, 0x00, 0x00},   // U+005B ([)
 { 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00, 0x00},   // U+005C (\)
 { 0x00, 0x41, 0x41, 0x7F, 0x00, 0x00, 0x00, 0x00},   // U+005D (])
 { 0x04, 0x02, 0x01, 0x02, 0x04, 0x00, 0x00, 0x00},   // U+005E (^)
 { 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40},   // U+005F (_)
 { 0x00, 0x01, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00},   // U+0060 (`)
 { 0x20, 0x54, 0x54, 0x54, 0x78, 0x00, 0x00, 0x00},   // U+0061 (a)
 { 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00},   // U+0062 (b)
 { 0x38, 0x44, 0x44, 0x44, 0x20, 0x00, 0x00, 0x00},   // U+0063 (c)
 { 0x38, 0x44, 0x44, 0x48, 0x7F, 0x00, 0x00, 0x00},   // U+0064 (d)
 { 0x38, 0x54, 0x54, 0x54, 0x18, 0x00, 0x00, 0x00},   // U+0065 (e)
 { 0x08, 0x7E, 0x09, 0x01, 0x02, 0x00, 0x00, 0x00},   // U+0066 (f)
 { 0x0C, 0x52, 0x52, 0x52, 0x3E, 0x00, 0x00, 0x00},   // U+0067 (g)
 { 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x00, 0x00},   // U+0068 (h)
 { 0x00, 0x44, 0x7D, 0x40, 0x00, 0x00, 0x00, 0x00},   // U+0069 (i)
 { 0x20, 0x40, 0x44, 0x3D, 0x00, 0x00, 0x00, 0x00},   // U+006A (j)
 { 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00, 0x00},   // U+006B (k)
 { 0x00, 0x41, 0x7F, 0x40, 0x00, 0x00, 0x00, 0x00},   // U+006C (l)
 { 0x7C, 0x04, 0x18, 0x04, 0x78, 0x00, 0x00, 0x00},   // U+006D (m)
 { 0x7C, 0x08, 0x04, 0x04, 0x78, 0x00, 0x00, 0x00},   // U+006E (n)
 { 0x38, 0x44, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00},   // U+006F (o)
 { 0x7C, 0x14, 0x14, 0x14, 0x08, 0x00, 0x00, 0x00},   // U+0070 (p)
 { 0x08, 0x14, 0x14, 0x18, 0x7C, 0x00, 0x00, 0x00},   // U+0071 (q)
 { 0x7C, 0x08, 0x04, 0x04, 0x08, 0x00, 0x00, 0x00},   // U+0072 (r)
 { 0x48, 0x54, 0x54, 0x54, 0x20, 0x00, 0x00, 0x00},   // U+0073 (s)
 { 0x04, 0x3F, 0x44, 0x40, 0x20, 0x00, 0x00, 0x00},   // U+0074 (t)
 { 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00, 0x00, 0x00},   // U+0075 (u)
 { 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00, 0x00, 0x00},   // U+0076 (v)
 { 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00, 0x00, 0x00},   // U+0077 (w)
 { 0x44, 0x28, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00},   // U+0078 (x)
 { 0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00, 0x00, 0x00},   // U+0079 (y)
 { 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, 0x00, 0x00},   // U+007A (z)
 { 0x00, 0x08, 0x36, 0x41, 0x00, 0x00, 0x00, 0x00},   // U+007B ({)
 { 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+007C (|)
 { 0x00, 0x41, 0x36, 0x08, 0x00, 0x00, 0x00, 0x00},   // U+007D (})
 { 0x02, 0x01, 0x02, 0x04, 0x02, 0x00, 0x00, 0x00},   // U+007E (~)
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

    // Nome único por boot: <device>_<uptime_ms>_<random>.jpg evita sobrescrever.
    char fname[96];
    snprintf(fname, sizeof(fname), "/sdcard/%s_%lld_%04X.jpg",
             device_id_str, (long long)(esp_timer_get_time() / 1000),
             (unsigned)(esp_random() & 0xFFFF));

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
// XCLK fica em 20 MHz e a resolução em XGA. A estratégia anti-corrupção NÃO é
// mais por banda (resolução/clock), e sim TEMPORAL: o keeper é capturado em
// janela silenciosa (ver capture_evidence_clean). fb_count=3 dá folga para o
// dreno reciclar buffers; WHEN_EMPTY garante que a IA leia um buffer que não
// está sendo escrito pelo DMA.
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
    cfg.xclk_freq_hz = 24000000;

    cfg.ledc_timer = LEDC_TIMER_0;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.pixel_format = PIXFORMAT_RGB565;
    cfg.frame_size = FRAMESIZE_XGA;
    cfg.jpeg_quality = 10;

    // 3 buffers dão folga ao dreno (recicla buffers contaminados) e ao DMA.
    cfg.fb_count = 3;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;

    // WHEN_EMPTY garante que a IA leia um buffer que NÃO está sendo gravado pelo DMA
    cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    cfg.sccb_i2c_port = -1;

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
        ssd1306_draw_string(2, 20, g_ip);
        char buf[24];
        if (g_detected) snprintf(buf, sizeof(buf), "%s %.2f", g_state_str, g_last_score);
        else            snprintf(buf, sizeof(buf), "%s", g_state_str);
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

static void __attribute__((unused)) draw_rect_rgb565(uint8_t *buf, int fbw, int fbh,
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
        esp_wifi_set_max_tx_power(80);

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

// ====================== Gatilho de presença (PIR no GPIO41) ======================
static volatile bool    s_btn_trigger = false;
static volatile int64_t s_btn_us = 0;

static void IRAM_ATTR button_isr(void *arg) {
    (void)arg;
    s_btn_trigger = true;
    s_btn_us = esp_timer_get_time();
}

static void button_init(void) {
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << BUTTON1_GPIO;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_ENABLE; // Pull-down interno garante LOW em repouso (PIR)
    io.intr_type = GPIO_INTR_POSEDGE;       // Borda de subida = presença detectada pelo PIR
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)BUTTON1_GPIO, button_isr, NULL);
    ESP_LOGI(TAG, "Gatilho: PIR no GPIO%d", BUTTON1_GPIO);
}

// Consome o gatilho com debounce (rejeita repiques dentro da janela).
static bool trigger_fired(void) {
    if (!s_btn_trigger) return false;
    static int64_t last_us = 0;
    int64_t t = s_btn_us;
    s_btn_trigger = false;
    if ((t - last_us) < (int64_t)TRIGGER_DEBOUNCE_MS * 1000) return false;
    last_us = t;
    return true;
}

// ============== Verificação: 1 frame, SÓ inferência (sem JPEG) ==============
// Esta é a parte barata. NÃO codifica JPEG — codificar aqui (fmt2jpg lê o
// frame inteiro da PSRAM) é o que mais rouba banda do DMA e corrompe o frame
// seguinte. A evidência de qualidade é capturada à parte (capture_evidence_clean).
static bool detect_once(float *out_score) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;

    dl::image::img_t img = {
        .data     = fb->buf,
        .width    = (uint16_t)fb->width,
        .height   = (uint16_t)fb->height,
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE,
    };
    auto &res = s_detect->run(img);

    const dl::detect::result_t *best = nullptr;
    for (auto &r : res) {
        if (r.score >= min_score && (!best || r.score > best->score)) best = &r;
    }
    if (out_score) *out_score = best ? best->score : 0.0f;
    bool hit = (best != nullptr);

    esp_camera_fb_return(fb);
    return hit;
}

// Uma evidência (JPEG já codificado + metadados da detecção).
typedef struct {
    uint8_t *jpg;
    size_t   len;
    float    score;
    int      bx, by, bw, bh;
} evidence_t;

static void evidence_free_all(evidence_t *ev, int n) {
    for (int i = 0; i < n; i++) {
        if (ev[i].jpg) { free(ev[i].jpg); ev[i].jpg = NULL; }
    }
}

// ============== Captura de evidência em JANELA SILENCIOSA ==============
// Princípio: um frame só corre risco de corrupção DMA DURANTE a própria
// escrita. Enquanto o buffer está SEGURADO (sem fb_return), o DMA não escreve
// nele — então inferência e fmt2jpg sobre o buffer segurado são imunes.
//
// Sequência:
//  1) DRENO: cicla buffers com a CPU ociosa para descartar os frames escritos
//     durante a contenção anterior (inferência/JPEG do keeper anterior). Como
//     o dreno > fb_count, os últimos buffers reciclam em condição limpa.
//  2) KEEPER: captura agora, com o barramento livre -> frame íntegro.
//  3) Inferência sobre o keeper SEGURADO -> box/score atualizados. O DMA enche
//     outro buffer (sacrificial) nesse meio tempo; será descartado no próximo
//     dreno.
//  4) fmt2jpg sobre o keeper SEGURADO -> JPEG limpo de alta qualidade.
static bool capture_evidence_clean(evidence_t *out, float fallback_score) {
    out->jpg = NULL;

    // 1) Dreno com a CPU ociosa: nada além do DMA toca a PSRAM aqui.
    for (int i = 0; i < EVIDENCE_DRAIN_FRAMES; i++) {
        camera_fb_t *d = esp_camera_fb_get();
        if (d) esp_camera_fb_return(d);
    }

    // 2) Keeper limpo, capturado em janela silenciosa.
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;

    // 3) Inferência no buffer SEGURADO (imune ao DMA) -> box/score.
    dl::image::img_t img = {
        .data     = fb->buf,
        .width    = (uint16_t)fb->width,
        .height   = (uint16_t)fb->height,
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE,
    };
    auto &res = s_detect->run(img);
    const dl::detect::result_t *best = nullptr;
    for (auto &r : res) {
        if (r.score >= min_score && (!best || r.score > best->score)) best = &r;
    }

    float sc = fallback_score; int bx = 0, by = 0, bw = 0, bh = 0;
    if (best) {
        int x1 = best->box[0], y1 = best->box[1];
        int x2 = best->box[2], y2 = best->box[3];
        if (x1 < 0) x1 = 0;
        if (y1 < 0) y1 = 0;
        if (x2 > fb->width)  x2 = fb->width;
        if (y2 > fb->height) y2 = fb->height;
        sc = best->score;
        bx = x1; by = y1;
        bw = (x2 > x1) ? (x2 - x1) : 0;
        bh = (y2 > y1) ? (y2 - y1) : 0;
    }

    // 4) JPEG no buffer SEGURADO. O frame que o DMA escreve agora é sacrificial.
    uint8_t *jpg = NULL; size_t len = 0;
    bool ok = fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
                      PIXFORMAT_RGB565, EVIDENCE_JPEG_QUALITY, &jpg, &len);

    esp_camera_fb_return(fb);

    if (!ok || !jpg) return false;
    out->jpg = jpg; out->len = len; out->score = sc;
    out->bx = bx; out->by = by; out->bw = bw; out->bh = bh;
    return true;
}

// ====================== FSM de vigilância (task no core 1) ======================
static void guard_task(void *arg) {
    (void)arg;

    typedef enum { ST_IDLE, ST_VERIFY, ST_EVIDENCE, ST_COOLDOWN } guard_state_t;
    guard_state_t state = ST_IDLE;
    g_state_str = "ARMADO";

    TickType_t last_fetch = xTaskGetTickCount();
    TickType_t last_disp  = 0;
    int64_t cooldown_start = 0;

    float confirmed_score  = 0.0f;
    int   confirmed_hits   = 0;
    int   confirmed_frames = 0;

    s_btn_trigger = false;  // ignora qualquer gatilho acumulado no boot

    while (1) {
        switch (state) {

        // ---------- REPOUSO: câmera quente, inferência desligada ----------
        case ST_IDLE: {
            g_detected = false;
            g_state_str = enable_detection ? "ARMADO" : "DESARMADO";

            if (enable_detection && trigger_fired()) {
                state = ST_VERIFY;
                g_state_str = "VERIFICANDO";
                break;
            }
            // Keep-alive: drena 1 frame para manter AEC/AWB adaptando, SEM
            // rodar inferência (barato). Garante que o primeiro frame
            // pós-gatilho já esteja bem exposto.
            if (enable_detection) {
                camera_fb_t *fb = esp_camera_fb_get();
                if (fb) esp_camera_fb_return(fb);
            }
            vTaskDelay(pdMS_TO_TICKS(KEEPALIVE_MS));
            break;
        }

        // ---------- VERIFICAÇÃO: confirma presença (só inferência) ----------
        case ST_VERIFY: {
            int   hits = 0;
            int   frames = 0;
            float best_sc = 0.0f;
            bool  confirmed = false;
            int64_t t0 = esp_timer_get_time();

            while (!confirmed && frames < VERIFY_MAX_FRAMES &&
                   (esp_timer_get_time() - t0) < (int64_t)VERIFY_WINDOW_MS * 1000) {
                float sc = 0;
                bool hit = detect_once(&sc);   // SÓ inferência — sem encode
                frames++;
                if (hit) {
                    hits++;
                    if (sc > best_sc) best_sc = sc;
                    // Caminho rápido: 1 frame muito confiante confirma na hora.
                    if (sc >= HIGH_CONFIDENCE || hits >= CONFIRM_HITS) confirmed = true;
                }
                vTaskDelay(pdMS_TO_TICKS(1));   // cede CPU / alimenta watchdog
            }

            if (confirmed) {
                confirmed_score  = best_sc;
                confirmed_hits   = hits;
                confirmed_frames = frames;
                state = ST_EVIDENCE;
            } else {
                ESP_LOGI(TAG, "Falso positivo: presenca nao confirmada (%d frame(s))", frames);
                g_detected = false;
                s_btn_trigger = false;
                update_display();
                state = ST_IDLE;
            }
            break;
        }

        // ---------- EVIDÊNCIA: captura limpa + (depois) rede/SD ----------
        case ST_EVIDENCE: {
            ESP_LOGI(TAG, "INTRUSO confirmado (%d acerto(s) em %d frame(s))",
                     confirmed_hits, confirmed_frames);
            g_detected   = true;
            g_state_str  = "INTRUSO";
            g_last_score = confirmed_score;
            update_display();
            buzzer_beep();   // LEDC independente do XCLL — não perturba a câmera

            // --- FASE DE CAPTURA: só DMA + CPU da inferência/encode no buffer
            //     segurado. NADA de rede ou SD aqui dentro. ---
            evidence_t ev[MAX_EVIDENCE];
            int ev_n = 0;
            int64_t tcap = esp_timer_get_time();
            while (ev_n < MAX_EVIDENCE &&
                   (esp_timer_get_time() - tcap) < (int64_t)EVIDENCE_TOTAL_WINDOW_MS * 1000) {
                evidence_t e;
                if (capture_evidence_clean(&e, confirmed_score)) {
                    ev[ev_n++] = e;
                }
                // Espaçamento p/ mostrar deslocamento entre evidências.
                if (ev_n < MAX_EVIDENCE) vTaskDelay(pdMS_TO_TICKS(EVIDENCE_SPACING_MS));
            }

            // --- SÓ AGORA: rede e SD, FORA da janela de captura ---
            bool ok_any = false;
            for (int i = 0; i < ev_n; i++) {
                esp_err_t up = upload_image(ev[i].jpg, ev[i].len, ev[i].score,
                                            ev[i].bx, ev[i].by, ev[i].bw, ev[i].bh);
                if (up == ESP_OK) ok_any = true;
                save_detection_to_sd(ev[i].jpg, ev[i].len, ev[i].score,
                                     ev[i].bx, ev[i].by, ev[i].bw, ev[i].bh);
            }
            evidence_free_all(ev, ev_n);

            if (ev_n > 0) {
                if (ok_any) { backend_fail_count = 0; backend_offline = false; }
                else { backend_fail_count++; if (backend_fail_count >= 10) backend_offline = true; }
            }

            s_btn_trigger = false;  // limpa gatilhos acumulados durante a captura
            update_display();
            state = ST_COOLDOWN;
            cooldown_start = esp_timer_get_time();
            break;
        }

        // ---------- COOLDOWN: ignora gatilhos por um tempo ----------
        case ST_COOLDOWN: {
            g_state_str = "COOLDOWN";
            if ((esp_timer_get_time() - cooldown_start) >= (int64_t)COOLDOWN_MS * 1000) {
                s_btn_trigger = false;
                state = ST_IDLE;
            } else {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            break;
        }
        }

        // Tarefas periódicas: SÓ em repouso/cooldown. Nunca entre a
        // confirmação (ST_VERIFY) e a captura (ST_EVIDENCE) — um fetch_config
        // HTTP nesse intervalo atrasaria a evidência e geraria tráfego WiFi
        // às portas da janela de captura.
        if (state == ST_IDLE || state == ST_COOLDOWN) {
            if ((xTaskGetTickCount() - last_fetch) > pdMS_TO_TICKS(30000)) {
                fetch_config();
                last_fetch = xTaskGetTickCount();
            }
            if ((xTaskGetTickCount() - last_disp) > pdMS_TO_TICKS(2000)) {
                update_display();
                last_disp = xTaskGetTickCount();
            }
        }
    }
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

    button_init();          // gatilho de presença (PIR no GPIO41)
    fetch_config();

    // FSM de vigilância FIXADA no core 1 (APP_CPU). O WiFi roda no core 0
    // (PRO_CPU), então a pilha de rede não preempta a captura/inferência.
    // Combine com os ajustes de sdkconfig (WiFi em IRAM, PSRAM Octal 80 MHz,
    // código fora da PSRAM) para reduzir as faltas de cache na flash que
    // disputam o controlador MSPI com o DMA da câmera.
    // Stack generoso (16 KB) para acomodar a inferência ESP-DL + fmt2jpg;
    // reduza se confirmar folga, aumente se vir stack overflow.
    xTaskCreatePinnedToCore(guard_task, "guard", 16384, NULL, 5, NULL, 1);
}