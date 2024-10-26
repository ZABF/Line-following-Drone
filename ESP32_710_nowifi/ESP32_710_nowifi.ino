#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

const char* ssid = "Huawei Mate 70 Pro";
const char* password = "123456987";
//const char* ssid = "DODO-A0C8_EXT";
//const char* pass//word = "ZHEX4BUHSC";


#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"

#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

#define FACE_COLOR_WHITE 0x00FFFFFF
#define FACE_COLOR_BLACK 0x00000000
#define FACE_COLOR_RED 0x000000FF
#define FACE_COLOR_GREEN 0x0000FF00
#define FACE_COLOR_BLUE 0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

#define LED_pin 4

typedef struct {
  size_t size;   //number of values used for filtering
  size_t index;  //current value index
  size_t count;  //value count
  int sum;
  int *values;  //array to be filled with values
} ra_filter_t;

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;

static mtmn_config_t mtmn_config = { 0 };
static int8_t detection_enabled = 1;
static int8_t recognition_enabled = 0;
static int8_t is_enrolling = 0;
static face_id_list id_list = { 0 };

static void draw_cross(dl_matrix3du_t *image_matrix, int i, uint8_t r, uint8_t g, uint8_t b) {
  int w = image_matrix->w;
  int h = image_matrix->h;
  if (i < 0) i = 0;
  if (i > w * h-1) i = w*h-1;
  image_matrix->item[i * 3] = b;
  image_matrix->item[i * 3 + 1] = g;
  image_matrix->item[i * 3 + 2] = r;

  int i_new = i - 1;
  if (i_new < 0) i_new = 0;
  image_matrix->item[i_new * 3] = b;
  image_matrix->item[i_new * 3 + 1] = g;
  image_matrix->item[i_new * 3 + 2] = r;

  i_new = i + 1;
  if (i_new > w * h-1) i_new = w * h-1;
  image_matrix->item[i_new * 3] = b;
  image_matrix->item[i_new * 3 + 1] = g;
  image_matrix->item[i_new * 3 + 2] = r;

  i_new = i - w;
  if (i_new < 0) i_new = 0;
  image_matrix->item[i_new * 3] = b;
  image_matrix->item[i_new * 3 + 1] = g;
  image_matrix->item[i_new * 3 + 2] = r;

  i_new = i + w;
  if (i_new > w * h-1) i_new = w * h-1;
  image_matrix->item[i_new * 3] = b;
  image_matrix->item[i_new * 3 + 1] = g;
  image_matrix->item[i_new * 3 + 2] = r;

  ///////////////////////////
  i_new = i - 2;
  if (i_new < 0) i_new = 0;
  image_matrix->item[i_new * 3] = b;
  image_matrix->item[i_new * 3 + 1] = g;
  image_matrix->item[i_new * 3 + 2] = r;

  i_new = i + 2;
  if (i_new > w * h-1) i_new = w * h-1;
  image_matrix->item[i_new * 3] = b;
  image_matrix->item[i_new * 3 + 1] = g;
  image_matrix->item[i_new * 3 + 2] = r;

  i_new = i - 2*w;
  if (i_new < 0) i_new = 0;
  image_matrix->item[i_new * 3] = b;
  image_matrix->item[i_new * 3 + 1] = g;
  image_matrix->item[i_new * 3 + 2] = r;

  i_new = i + 2*w;
  if (i_new > w * h-1) i_new = w * h-1;
  image_matrix->item[i_new * 3] = b;
  image_matrix->item[i_new * 3 + 1] = g;
  image_matrix->item[i_new * 3 + 2] = r;
}


static uint8_t* my_image_process(dl_matrix3du_t *image_matrix, uint8_t* coordinate) {
  static uint16_t black_num = 0;
  static uint16_t black_sum = 0;
  static int16_t split = 10;
  static uint16_t black_mid[40];
  uint32_t temp;

  float avr_gray = 0;
  int avr_gray_num = 0;
  //计算平均灰度 arvage gray
  for (int x = 0; x < image_matrix->w; x++) {
    x = x + 3;
    for (int y = 0; y < image_matrix->h; y++) {
      y = y + 3;
      int i = (y * image_matrix->w + x) * 3;
      uint8_t b = image_matrix->item[i];      // Blue channel
      uint8_t g = image_matrix->item[i + 1];  // Green channel
      uint8_t r = image_matrix->item[i + 2];  // Red channel
      uint8_t gray = (uint8_t)(0.299 * r + 0.587 * g + 0.114 * b);
      avr_gray += gray;
      avr_gray_num++;
    }
  }
  avr_gray = avr_gray / avr_gray_num;

  //二值化 一列再取平均点 binary and arvage points
  int image_h_times = 0;
  for (int x = 0; x < image_matrix->w; x++) {
    black_num = 0;
    black_sum = 0;
    image_h_times++;
    for (int y = 0; y < image_matrix->h; y++) {
      int i = (y * image_matrix->w + x) * 3;
      uint8_t b = image_matrix->item[i];      // Blue channel
      uint8_t g = image_matrix->item[i + 1];  // Green channel
      uint8_t r = image_matrix->item[i + 2];  // Red channel

      // Calculate grayscale value using weighted formula
      uint8_t gray = (uint8_t)(0.299 * r + 0.587 * g + 0.114 * b);

      // Perform binarization based on the threshold
      if (gray > avr_gray + 50)  //偏白色
      //if (gray < avr_gray - 50)  //偏黑色
      {
        //image_matrix->item[i] = image_matrix->item[i] * 0.5;
        //image_matrix->item[i + 1] = image_matrix->item[i + 1] * 0.5;
        //image_matrix->item[i + 2] = 80 + image_matrix->item[i + 2] * 0.5;  // redish
        black_num += 1;
        black_sum += y;
      }
    }

    //取列平均点 arvage points in list
    if (black_num == 0) black_num = 1;
    if (image_h_times == split) {
      black_mid[x / split] = black_sum / black_num;
      temp = (black_mid[x / split] * image_matrix->w + x);
//      draw_cross(image_matrix, temp, 0, 255, 0);//green
      image_h_times = 0;
    }
  }

  //取平均点
  int up_sum_avr = 0;
  int up_sum_num = 0;
  int mid_sum_avr = 0;
  int mid_sum_num = 0;
  int i;
  int i_one_third = 5;//image_matrix->w / split / 3;
  for (i = 0; i < i_one_third; i++) {
    if (black_mid[i +10] != 0){
      up_sum_avr += black_mid[i + 10];
      up_sum_num ++;
    }
  }
  if(up_sum_num != 0) up_sum_avr = up_sum_avr / up_sum_num;

  for (i = 0; i < i_one_third; i++) {
    if (black_mid[i + i_one_third] != 0){
      mid_sum_avr += black_mid[i + i_one_third];
      mid_sum_num ++;
    }
  }
  if(mid_sum_num != 0) mid_sum_avr = mid_sum_avr / mid_sum_num;

//  temp = (up_sum_avr * image_matrix->w + image_matrix->w -image_matrix->w / 6);
//  draw_cross(image_matrix, temp, 255, 210, 0);

//  temp = (mid_sum_avr * image_matrix->w + image_matrix->w / 2);
//  draw_cross(image_matrix, temp, 255, 210, 0);

  coordinate[0] = up_sum_avr;
  coordinate[1] = mid_sum_avr;

}

static void stream() {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];
  dl_matrix3du_t *image_matrix = NULL;
  bool detected = false;
  int64_t fr_start = 0;
  int64_t fr_ready = 0;
  int64_t fr_face = 0;
  int64_t fr_recognize = 0;
  int64_t fr_encode = 0;

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }
  Serial.println("TFRE");

  while (true) {
    detected = false;
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      fr_start = esp_timer_get_time();
      fr_ready = fr_start;
      fr_face = fr_start;
      fr_encode = fr_start;
      fr_recognize = fr_start;
{
        image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

        if (!image_matrix) {
          Serial.println("dl_matrix3du_alloc failed");
          res = ESP_FAIL;
        } else {
          if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)) {
            Serial.println("fmt2rgb888 failed");
            res = ESP_FAIL;
          } else {
            fr_ready = esp_timer_get_time();
            if (detection_enabled) {
              uint8_t coordinate [2];
              my_image_process(image_matrix,coordinate);
            //  Serial.print(coordinate[0]);
              Serial.write(0xF5); //head
              Serial.write(0x02); //byte num
              Serial.write(coordinate[0]);//0 - 120
            //  Serial.print(coordinate[1]);
              Serial.write(coordinate[1]);
            }
            //Serial.print("ending");
            fr_face = esp_timer_get_time();
            // fr_recognize = fr_face;
            // if ( net_boxes || fb->format != PIXFORMAT_JPEG) {
            //   if (!fmt2jpg(image_matrix->item, fb->width * fb->height * 3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)) {
            //     Serial.println("fmt2jpg failed");
            //     res = ESP_FAIL;
            //   }
            //   esp_camera_fb_return(fb);
            //   fb = NULL;
            // } else {
            //   _jpg_buf = fb->buf;
            //   _jpg_buf_len = fb->len;
            // }
            fr_encode = esp_timer_get_time();
          }
          dl_matrix3du_free(image_matrix);
        }
      }
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    int64_t fr_end = esp_timer_get_time();

    int64_t ready_time = (fr_ready - fr_start) / 1000;
    int64_t face_time = (fr_face - fr_ready) / 1000;
    int64_t recognize_time = (fr_recognize - fr_face) / 1000;
    int64_t encode_time = (fr_encode - fr_recognize) / 1000;
    int64_t process_time = (fr_encode - fr_start) / 1000;

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    //uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
    // Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps), %u+%u+%u+%u=%u %s%d\n",
    //               (uint32_t)(_jpg_buf_len),
    //               (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
    //               avg_frame_time, 1000.0 / avg_frame_time,
    //               (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time,
    //               (detected) ? "DETECTED " : "", face_id);
  }

  last_frame = 0;
}



void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG,0);

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG,0);
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
  //config.pixel_format = PIXFORMAT_JPEG;
  config.pixel_format = PIXFORMAT_RGB565;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif
  // LED init
  pinMode(LED_pin, OUTPUT);
  // 
  digitalWrite(LED_pin,LOW);
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QQVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  // WiFi.begin(ssid, password);

  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("");
   Serial.println("connected");

  // startCameraServer();

  // Serial.print("Camera Ready! Use 'http://");
  // Serial.print(WiFi.localIP());
  // Serial.println("' to connect");
}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(10000);
  //Serial.println("ready connected");
  //digitalWrite(LED_pin,LOW);
  //delay(990);
 // digitalWrite(LED_pin,HIGH);
  //delay(10);
  stream();
  Serial.println("end connected");
}