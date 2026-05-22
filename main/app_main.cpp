#include <string.h>
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "pedestrian_detect.hpp"
#include "img_converters.h"
#include "esp_heap_caps.h"

static const char *TAG = "face_test";
static PedestrianDetect *s_detect = nullptr;

// --- Pinos da câmera (ESP32-S3-EYE) - AJUSTE PARA SUA PLACA ---
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
        .xclk_freq_hz = 24000000,             // OV5640 precisa de 24MHz
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,       // OV5640 instável em RGB565
        .frame_size   = FRAMESIZE_VGA,       // 320x240
        .jpeg_quality = 15,
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
        .sccb_i2c_port = -1
    };
    return esp_camera_init(&cfg);
}

static esp_err_t face_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGW(TAG, "frame perdido");
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_send(req, NULL, 0);
    }

    // Decodifica JPEG -> RGB888 pro detector
    size_t rgb_len = (size_t)fb->width * fb->height * 3;
    uint8_t *rgb = (uint8_t *)heap_caps_malloc(rgb_len, MALLOC_CAP_SPIRAM);
    if (!rgb || !fmt2rgb888(fb->buf, fb->len, fb->format, rgb)) {
        ESP_LOGE(TAG, "fmt2rgb888 falhou");
        if (rgb) free(rgb);
        esp_camera_fb_return(fb);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    dl::image::img_t img = {
        .data     = rgb,
        .width    = (uint16_t)fb->width,
        .height   = (uint16_t)fb->height,
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888,
    };

    auto &results = s_detect->run(img);
    free(rgb);

    if (results.empty()) {
        esp_camera_fb_return(fb);
        httpd_resp_set_status(req, "204 No Content");
        return httpd_resp_send(req, NULL, 0);
    }

    for (const auto &r : results) {
        ESP_LOGI(TAG, "Pessoa score=%.2f box=[%d,%d,%d,%d]",
                 r.score, r.box[0], r.box[1], r.box[2], r.box[3]);
    }

    // fb->buf JÁ é JPEG — manda direto, sem reconverter
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t rc = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return rc;
}

static httpd_handle_t start_server(void)
{
    httpd_handle_t srv = NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;

    if (httpd_start(&srv, &cfg) != ESP_OK) return NULL;

    httpd_uri_t uri = {
        .uri      = "/face",
        .method   = HTTP_GET,
        .handler  = face_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(srv, &uri);
    return srv;
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());     // WiFi/Eth via menuconfig

    ESP_ERROR_CHECK(camera_init());

    for (int i = 0; i < 3; i++) {
        camera_fb_t *wfb = esp_camera_fb_get();
        if (wfb) esp_camera_fb_return(wfb);
    }

    s_detect = new PedestrianDetect();       // usa o modelo do Kconfig

    if (!start_server()) {
        ESP_LOGE(TAG, "HTTP server falhou");
        return;
    }
    ESP_LOGI(TAG, "OK. GET http://<ip>/face");
}