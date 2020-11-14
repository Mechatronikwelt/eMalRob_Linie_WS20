/********************************
*
*Line Detection Porgramm
*
*Informationstechnik Labor WS20
*
*Dieses Programm wertet mithilfe einer ESP32-Cam eine Schwarze Linie auf weißen Hintergrund aus. 
*Die Kameraeinstellungen sind für eine schwarze Linie gezeichnet auf einem Tablet mit voller Hintergrundbeleuchtung angepasst.
*Es wird ein Vektor mit Winkel ausgegeben.
*  
* Author:   Christoph Weiss, wech1030
*           Fabian Marschall, mafa1024
*           
* Datum:    14.11.2020
* 
* Inspired by: https://eloquentarduino.github.io/2020/01/motion-detection-with-esp32-cam-only-arduino-version/
* 
* More Informations (): http://hit-karlsruhe.de/hit-info/info-ws20/eM-Linie/index.html
* 
* Which is which variable? See here --> http://hit-karlsruhe.de/hit-info/info-ws20/eM-Linie/0403Softwaredoku.html
************************************/
#define CAMERA_MODEL_AI_THINKER 

#include <Adafruit_NeoPixel.h>
#include "esp_camera.h"
#include "camera_pins.h"

//--- CAM Defines


#define FRAME_SIZE FRAMESIZE_VGA
#define WIDTH 640
#define HEIGHT 480
#define BLOCK_SIZE 10
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)
#define BLOCK_DIFF_THRESHOLD 0.2
#define IMAGE_DIFF_THRESHOLD 0.1
#define DEBUG 1


//---LED Defines, currently not used
//--- LEDs at pin 15, LED-Ring with 12 LEDs

#define PIN         15        
#define NUMPIXELS   12
#define BRIGHTNESS  255
#define LED_count   12     //Anzahl zu leuchtende LEDs
Adafruit_NeoPixel matrix = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


int angel;    //Vector angel
int pos_1;    //Vector positions
int pos_2;    
int vector[4]; //creates vektor with two coordinates
int32_t prev_frame[H][W] = { 0 };
int32_t current_frame[H][W] = { 0 };
int32_t current_averaged_frame[H][W] = { 0 };

bool setup_camera(framesize_t);
bool capture_still();
void print_frame(int32_t frame[H][W]);                    // Show frame in Serial Monitor
int find_average_line(int32_t pframe [H][W], int row);    // line detection function
int diff_1(int row);                                      // diff, Highpass filter
int get_Angel(int pos1, int pos2);                        // find Angle of Vector
int set_vector(int pos1, int pos2);                       // create vektor
void LED_Panel(uint32_t c, uint8_t count);                // function for LED, currently not used

/**
 *
 */
void setup() {
    Serial.begin(115200);
    Serial.println(setup_camera(FRAME_SIZE) ? "OK" : "ERR INIT");

//---- LED init
    matrix.setBrightness(BRIGHTNESS);
    matrix.begin();
    matrix.show();
    LED_Panel(matrix.Color(0, 0, 0),LED_count);
}

/**
 *
 */
void loop() {
    if (!capture_still()) {
        Serial.println("Failed capture");
        delay(3000);
        return;
    }
}

/**
 * Camera Settings
 */
bool setup_camera(framesize_t frameSize) {
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
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 20;
    config.fb_count = 1;

   
    esp_err_t err = esp_camera_init(&config);
      if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return true;
      }
    sensor_t *s = esp_camera_sensor_get(); 
    
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 0);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 0);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
s->set_exposure_ctrl(s, 0);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);       // -2 to 2
s->set_aec_value(s, 200);    // 0 to 1200
    s->set_gain_ctrl(s, 0);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 0);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    
    return true;
} 

/**
 * Capture image and do down-sampling & averageing 
 */
bool capture_still() {
    camera_fb_t *frame_buffer = esp_camera_fb_get();

    if (!frame_buffer)
        return false;

    // set all 0/255s in current frames
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            {
             current_frame[y][x] = 0;
             current_averaged_frame[y][x]=255;
            }


    // down-sample image in blocks
    for (uint32_t i = 0; i < WIDTH * HEIGHT; i++) {
        const uint16_t x = i % WIDTH;
        const uint16_t y = floor(i / WIDTH);
        const uint16_t block_x = floor(x / BLOCK_SIZE);
        const uint16_t block_y = floor(y / BLOCK_SIZE);
        const uint32_t pixel = frame_buffer->buf[i];
        const uint32_t current = current_frame[block_y][block_x];

        // average pixels in block (accumulate)
        current_frame[block_y][block_x] += pixel;
    }

    // average pixels in block (rescale)
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;


//Moving Average first and last row in Array with Highpass Filter - Glättung
  
    int yn1=0;    //first row
    
    for (int x = 1; x < (W - 1); x++)
      current_averaged_frame[yn1][x] = (current_frame[yn1][x - 1] + current_frame[yn1][x] + current_frame[yn1][x + 1]) / 3;
    diff_1(yn1);
    pos_1=find_average_line(current_averaged_frame,yn1 );
    
    int yn2=47;     //last row
    
    for (int x = 1; x < (W - 1); x++)
      current_averaged_frame[yn2][x] = (current_frame[yn2][x - 1] + current_frame[yn2][x] + current_frame[yn2][x + 1]) / 3;
    diff_1(yn2);
    pos_2=find_average_line(current_averaged_frame,yn2);

    set_vector(pos_1,yn1,pos_2,yn2);
    angel=get_Angel(pos_1,pos_2);
    
#if DEBUG
    //Serial.println("Current frame:");
    //print_frame(current_frame);
    //Serial.println("Averaged Current frame:");
    //print_frame(current_averaged_frame);


    Serial.println("~~~~~~~ LINE DETECTION ~~~~~~~~~~~");
    Serial.println("Vektor (x1,y1)(x2,y2):");
    Serial.print(vector[0]);
    Serial.print("\t");
    Serial.print(vector[1]);
    Serial.print("\t");
    Serial.print(vector[2]);
    Serial.print("\t");
    Serial.println(vector[3]);
    Serial.println("Winkel:");
    Serial.println(angel);
    Serial.println("---------------");
#endif

    return true;
}



int get_Angel(int pos1, int pos2)       //Angel setter function
{
  float Angle;
  float b = H;
  float a = pos1 - pos2;
  a = abs(a);

  if (pos1 < pos2)
  {
    Angle = atan(b / a);
    Angle = Angle * (180 / 3.14);
    return Angle;
  }
  else
  {
    Angle = atan(a / b);
    Angle = Angle * (180 / 3.14);
    return Angle+90;
  }

}

int set_vector(int pos1x, int pos1y, int pos2x, int pos2y){
  vector[0]=pos1x;
  vector[1]=pos1y;
  vector[2]=pos2x;
  vector[3]=pos2y;
}

int find_average_line(int32_t pframe[H][W], int row) {    // finds lowest grey value in row, detects it with diff1-function
  int small_pos = 0;
  int smallest = pframe[row][1];
  int big_pos = 0;
  int biggest = pframe[row][1];
  int pos = 0;

  for (int i = 1; i < (W - 2); i++)
  {
    if (smallest > pframe[row][i])
    {
      smallest = pframe[row][i];
      small_pos = i;
    }
  }

  for (int i = 1; i < (W - 2); i++)
  {
    if (biggest < pframe[row][i])
    {
      biggest = pframe[row][i];
      big_pos = i;
    }
  }

  return ((big_pos - small_pos) / 2 + small_pos);
}

int diff_1(int row)     //Highpass filter function
{
  for (int i = 1; i < (W - 2); i++)
  {
    current_averaged_frame[row][i] = current_averaged_frame[row][i + 1] - current_averaged_frame[row][i];
  }
  return 0;
};

void print_frame(int32_t frame[H][W]) {               //serial print frame
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Serial.print(frame[y][x]);
            Serial.print('\t');
        }

        Serial.println();
    }
}

void LED_Panel(uint32_t c, uint8_t count) {           //LED Ring 
  for(uint16_t i=0; i<count; i++) {
    matrix.setPixelColor(i, c);
    matrix.show();
  }
}
