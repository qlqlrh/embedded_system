#include "esp_camera.h"
#include <HardwareSerial.h>
#include <WiFi.h>

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================

void setupLedFlash(int pin);

// UART 설정
HardwareSerial mySerial(1); // ESP32-CAM의 UART1 사용

void setup() {
  
  mySerial.begin(115200, SERIAL_8N1, 3, 1); // ESP32-CAM의 TX=1, RX=3 핀 활성화, 즉 STM32 보드의 UART 핀에 연결되는 핀

  // 부팅 로그 비활성화
  Serial.flush();
  esp_log_level_set("*", ESP_LOG_NONE);

  // UART 버퍼 초기화
  while (Serial.available() > 0) { Serial.read(); }

  // 카메라 설정
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA; // 해상도 설정 (QVGA = 320x240)
  config.pixel_format = PIXFORMAT_JPEG; // JPEG 포맷 설정
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12; // JPEG 품질 설정 (0: 최고 품질, 63: 최저 품질)
  config.fb_count = 1; // 프레임 버퍼 개수 설정

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
    #if CONFIG_IDF_TARGET_ESP32S3
        config.fb_count = 2;
    #endif
  }

  #if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
  #endif

    // 카메라 초기화
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }
    Serial.println("ESP32-CAM Initialized!");

    sensor_t *s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);        // flip it back
      s->set_brightness(s, 1);   // up the brightness just a bit
      s->set_saturation(s, -2);  // lower the saturation
    }
    // drop down frame size for higher initial frame rate
    if (config.pixel_format == PIXFORMAT_JPEG) {
      s->set_framesize(s, FRAMESIZE_QVGA);
    }

  #if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  #endif

  #if defined(CAMERA_MODEL_ESP32S3_EYE)
    s->set_vflip(s, 1);
  #endif

  // Setup LED FLash if LED pin is defined in camera_pins.h
  #if defined(LED_GPIO_NUM)
    setupLedFlash(LED_GPIO_NUM);
  #endif
}


// ===========================
// Main Loop Function
// ===========================
void loop() {

  // 이미지 정보 UART로 전송
  mySerial.printf("Data From ESP32-CAM!!!\n");

  delay(3000);

  // // 이미지 캡처
  // camera_fb_t *fb = esp_camera_fb_get();
  // if (!fb) {
  //   Serial.println("Failed to capture image");
  //   return;
  // }

  // // 캡처한 이미지 정보 출력
  // Serial.println("Captured Image:");
  // Serial.printf("- Image size: %u bytes\n", fb->len); // 이미지 크기 출력
  // Serial.printf("- Resolution: %dx%d\n", fb->width, fb->height); // 해상도 출력

  // // 이미지 버퍼 반환
  // esp_camera_fb_return(fb);

  // // 3초 대기
  // delay(3000);

}
