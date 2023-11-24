#include <Arduino.h>
#include "esp_camera.h"
#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <Suveillance_camera_with_object_detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

//#define CAMERA_MODEL_XIAO_ESP32S3
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"



#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture


const char ssid[] = "NITDA-ICT-HUB";
const char pass[] = "6666.2524#";

#define CONFIG_BROKER_URL "172.16.120.87"

#define CONFIG_BROKER_USERNAME "esp_camera"
#define CONFIG_BROKER_PASSWORD "0000"

//For the First Camera
#define ESP32CAM_DEVICE "ESP32-CAM-0"
#define ESP32CAM_PUBLISH_TOPIC "esp32/cam_0"
//For the Second Camera
//#define ESP32CAM_DEVICE "ESP32-CAM-1"
//#define ESP32CAM_PUBLISH_TOPIC "esp32/cam_1"

const int bufferSize = 1024 * 25;  // 25KB

WiFiClient net;
MQTTClient client = MQTTClient(bufferSize);

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.print("\nconnecting...");
  while (!client.connect(ESP32CAM_DEVICE, CONFIG_BROKER_USERNAME, CONFIG_BROKER_PASSWORD)) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");
}

bool camInit(){
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
      config.frame_size = FRAMESIZE_VGA;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_VGA;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return false;
  }

  return true;
}

void uploadImage() {

  // Serial.println("uploadImage via MQTT");
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    // Serial.println("Camera capture failed");
    return;
  }

  if (fb->format != PIXFORMAT_JPEG) {
    // Serial.println("Non-JPEG data not implemented");
    esp_camera_fb_return(fb);
    return;
  }

  if (!client.publish(ESP32CAM_PUBLISH_TOPIC, (const char*)fb->buf, fb->len)) {
    // Serial.println("[Failure] Uploading Image via MQTT");
  }

  esp_camera_fb_return(fb);
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

void setup() {
  Serial.begin(115200);
  if (!camInit()) {
    Serial.println("[Failure] Cam Init");
    return;
  }
  WiFi.begin(ssid, pass);
  client.begin(CONFIG_BROKER_URL, 1883, net);
  connect();
}

void loop() {
  client.loop();
  // delay(10);

  if (!client.connected()) {
    connect();
  } else {
    uploadImage();
  }
  ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

// Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
        auto bb = result.bounding_boxes[ix];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("    %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }
    if (!bb_found) {
        ei_printf("    No objects found\n");
    }
#else
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label,
                                    result.classification[ix].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif


    free(snapshot_buf);

}


static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
