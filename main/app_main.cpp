#include <string.h>
#include "esp_log.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "pedestrian_detect.hpp"

static const char *TAG = "guard";
static PedestrianDetect *s_detect = nullptr;

// Se as cores saírem trocadas no navegador, muda pra 1 (byte swap do RGB565)
#define SWAP_RGB565_BYTES 0

// --- Pinos da câmera (ESP32-S3-EYE) ---
#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   15
#define CAM_PIN_SIOD    4
#define CAM_PIN_SIOC    5
#define CAM_PIN_D7     16
#define CAM_PIN_D6     17
#define CAM_PIN_D5     18
#define CAM_PIN_D4     12
#define CAM_PIN_D3     10
#define CAM_PIN_D2      8
#define CAM_PIN_D1      9
#define CAM_PIN_D0     11
#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK   13

static esp_err_t camera_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn     = CAM_PIN_PWDN,
        .pin_reset    = CAM_PIN_RESET,
        .pin_xclk     = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7, .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5, .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3, .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1, .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href  = CAM_PIN_HREF,
        .pin_pclk  = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_RGB565, 
        .frame_size   = FRAMESIZE_QVGA,      
        .jpeg_quality = 12,
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
        .sccb_i2c_port = -1,
    };
    return esp_camera_init(&cfg);
}

// Página com auto-refresh do feed
static esp_err_t index_handler(httpd_req_t *req)
{
    static const char html[] =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>ESP Guard Vision</title>"
        "<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center}"
        "img{max-width:95vw;border:2px solid #0f0;margin-top:10px}"
        "#st{font-size:1.2em;margin:8px}</style></head>"
        "<body><h2>ESP Guard Vision</h2>"
        "<p id='st'>aguardando...</p><img id='f'>"
        "<script>"
        "function tick(){"
        " fetch('/face?'+Date.now()).then(r=>{"
        "  if(r.status===200){return r.blob().then(b=>{"
        "   document.getElementById('f').src=URL.createObjectURL(b);"
        "   document.getElementById('st').textContent='PESSOA DETECTADA';});}"
        "  else{document.getElementById('st').textContent='sem pessoa';}"
        " }).catch(()=>{document.getElementById('st').textContent='...';});"
        "}"
        "setInterval(tick,700);tick();"
        "</script></body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t face_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGW(TAG, "frame perdido");
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_send(req, NULL, 0);
    }

    // Detecção direto no RGB565 (sem cópia, sem perda de desempenho)
    dl::image::img_t img = {
        .data     = fb->buf,
        .width    = (uint16_t)fb->width,
        .height   = (uint16_t)fb->height,
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE,
    };

    auto &results = s_detect->run(img);

    if (results.empty()) {
        esp_camera_fb_return(fb);
        httpd_resp_set_status(req, "204 No Content");
        return httpd_resp_send(req, NULL, 0);
    }

    for (const auto &r : results) {
        ESP_LOGI(TAG, "Pessoa score=%.2f box=[%d,%d,%d,%d]",
                 r.score, r.box[0], r.box[1], r.box[2], r.box[3]);
    }

#if SWAP_RGB565_BYTES
    // Só pra correção de cor no JPEG; a detecção já rodou antes disso
    uint8_t *p = fb->buf;
    for (size_t i = 0; i + 1 < fb->len; i += 2) {
        uint8_t t = p[i]; p[i] = p[i + 1]; p[i + 1] = t;
    }
#endif

    // RGB565 -> JPEG por software (encoder na CPU, não no sensor)
    uint8_t *jpg = NULL;
    size_t   jpg_len = 0;
    bool ok = fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
                      PIXFORMAT_RGB565, 75, &jpg, &jpg_len);
    esp_camera_fb_return(fb);

    if (!ok) {
        ESP_LOGE(TAG, "fmt2jpg falhou");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t rc = httpd_resp_send(req, (const char *)jpg, jpg_len);
    free(jpg);
    return rc;
}

static httpd_handle_t start_server(void)
{
    httpd_handle_t srv = NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;

    if (httpd_start(&srv, &cfg) != ESP_OK) return NULL;

    httpd_uri_t index_uri = { .uri = "/",     .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
    httpd_uri_t face_uri  = { .uri = "/face", .method = HTTP_GET, .handler = face_handler,  .user_ctx = NULL };
    httpd_register_uri_handler(srv, &index_uri);
    httpd_register_uri_handler(srv, &face_uri);
    return srv;
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    esp_wifi_set_ps(WIFI_PS_NONE);           // banda de rede estável
    ESP_ERROR_CHECK(camera_init());


    s_detect = new PedestrianDetect();       // modelo via Kconfig (RODATA)

    if (!start_server()) {
        ESP_LOGE(TAG, "HTTP server falhou");
        return;
    }
    ESP_LOGI(TAG, "OK. Abra http://<ip>/ no navegador");
}