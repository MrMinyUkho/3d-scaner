//---------Библиотеки--------------

#include "I2Cdev.h"  // Интерфейсы и прочее
#include <EEPROM.h>

#include <EncButton.h>  // Энкодер

#include <NewPing.h>  // Сонар

#include <Adafruit_GFX.h>     // Графика для дисплея
#include <Adafruit_ST7735.h>  // Дисплей
#include <SPI.h>

#include "MPU6050_6Axis_MotionApps20.h"  // Гироскоп и
// акселерометр

#include <MechaQMC5883.h>  // Магнетометр

#include "bitmaps.h"  // Элементы интерфейса
#include "func.h"     // Внешние функции

//---------Конфиги-----------------

#define EEPROM_SIZE 64
/*

  0-19   WIFI_SSID
  20-59   WIFI_KEY
  60-63   SERV_IP

*/

//----------Порты-устройств--------

#define TFT_CS 5
#define TFT_DC 4
#define TFT_RST 15

#define SON_ECH 17
#define SON_TRG 16

#define ENC_SCK 13
#define ENC_SW 14
#define ENC_DT 2

#define MPU_INT 27

//---------------------------------

#define ul unsigned long

//--------Устройства---------------

Adafruit_ST7735 tft_disp = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
NewPing sonar(SON_TRG, SON_ECH, 400);
MPU6050 mpu;
MechaQMC5883 qmc;
EncButton<EB_TICK, ENC_SCK, ENC_DT, ENC_SW> enc;

//--------Переменные---------------

uint8_t MCoord[6][2] = {
  { 10, 26 },
  { 10, 74 },
  { 58, 26 },
  { 58, 74 },
  { 106, 26 },
  { 106, 74 }
};

int8_t MM_cs = 0;

uint8_t CastFlag = 0;

volatile bool mpuFlag = false;  // флаг прерывания готовности MPU
uint8_t fifoBuffer[45];         // буфер

float gyrX_f, gyrY_f, gyrZ_f;
float accX_f, accY_f, accZ_f;
float x, y, z;

float dist_3[3] = { 0.0, 0.0, 0.0 };  // переменные сонара
float dist, dist_filtered;
float k;
byte s_count, delta;

ul sensTimer;
ul dispTimer;
ul wifiTimer;
ul serlTimer;
ul mpu_Timer;
ul dt__Timer;

char ssid[20];      // The SSID (name) of the Wi-Fi network you want to connect to
char password[40];  // The password of the Wi-Fi network

const char *host = "192.168.1.101";  // server IP Address
const int port = 65342;              // server port

int wmod = 0;  // flag to check for connection existance
int smod = 0;
int cmod = 0;

int32_t data1[3] = { 0, 0, 0 };
int32_t pr_data[3] = { 0, 0, 0 };
ul time_rec[2] = { 0, 0 };


uint16_t rgb[] = { 0xF800, 0x07E0, 0x001F, 0x07FF, 0xF81F, 0xFFE0 };

kalman fl_X;
kalman fl_Y;
kalman fl_Z;

void IRAM_ATTR dmpReady() {
  mpuFlag = true;
}

void setup() {

  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(ENC_SCK, INPUT_PULLUP);

  enc.setEncType(EB_HALFSTEP);

  tft_disp.initR(INITR_BLACKTAB);                    // инициализация
  tft_disp.setRotation(tft_disp.getRotation() + 3);  // крутим дисп
  tft_disp.fillScreen(ST7735_BLACK);                 // чисти чисти

  for (int i = 0; i < 128; ++i) {
    for (int j = 0; j < 160; ++j) {
      uint16_t px = pgm_read_word(&Logo[i * 160 + j]);
      if (px != 0) tft_disp.drawPixel(j, i, px);
    }
  }

  tft_disp.drawRect(27, 27, 10, 10, 0x7BCF);
  tft_disp.drawRect(27, 42, 10, 10, 0x7BCF);
  tft_disp.drawRect(27, 57, 10, 10, 0x7BCF);

  Wire.begin();

  qmc.init();

  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);

  attachInterrupt(MPU_INT, dmpReady, RISING);

  tft_disp.fillRect(29, 29, 6, 6, !wmod ? 0xF800 : 0x07E0);

  delay(3000);

  tft_disp.fillScreen(0);
  mpu.CalibrateAccel(15);
  mpu.CalibrateGyro(15);

  Serial.begin(115200 * 2);
  Serial.println("z x y");

  drawFromProgMem()
}

void loop() {
  enc.tick();

  if (enc.left()) {
    MM_cs--;
    if (MM_cs < 0) MM_cs = 5;
  } else if (enc.right()) {
    MM_cs++;
    if (MM_cs > 5) MM_cs = 0;
  }

  //Serial.print(enc.tick());
  //Serial.print(" ");
  if (millis() - sensTimer > 50) {  // измерение и вывод каждые 50 мс
    if (s_count > 1) s_count = 0;
    else s_count++;

    dist_3[s_count] = (float)sonar.ping() / 57.5;         // получить расстояние в текущую ячейку массива
    dist = middle_of_3(dist_3[0], dist_3[1], dist_3[2]);  // фильтровать медианным фильтром из 3ёх последних измерений

    delta = abs(dist_filtered - dist);  // расчёт изменения с предыдущим
    if (delta > 1) k = 0.7;             // если большое - резкий коэффициент
    else k = 0.1;                       // если маленькое - плавный коэффициент

    dist_filtered = dist * k + dist_filtered * (1 - k);  // фильтр "бегущее среднее"

    sensTimer = millis();  // сбросить таймер
  }

  if (millis() - dispTimer > 70) {
    //tft_disp.fillRect(10, 10, 66, 24, 0x0000);
    for (int i = 0; i < 6; ++i) {
      if (i == MM_cs)
        tft_disp.drawRoundRect(MCoord[i][0], MCoord[i][1], 44, 44, 12, 0x55A9);
      else
        tft_disp.drawRoundRect(MCoord[i][0], MCoord[i][1], 44, 44, 12, 0x0000);
    }
    dispTimer = millis();
  }
  if (mpuFlag && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // переменные для расчёта (ypr можно вынести в глобал)
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    // расчёты
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuFlag = false;
    // выводим результат в радианах (-3.14, 3.14)
    gyrZ_f = ypr[0];  // вокруг оси Z
    gyrY_f = ypr[1];  // вокруг оси Y
    gyrX_f = ypr[2];  // вокруг оси X
    mpu_Timer = millis();
  }
  if (millis() - dt__Timer >= 0 and CastFlag) {
    getData();
    dt__Timer = millis();
  }
}

//-------------Функции-отрисовки---------

void drawText(char *text, uint16_t color, int x, int y) {
  tft_disp.setCursor(x, y);
  tft_disp.setTextColor(color);
  tft_disp.setTextWrap(true);
  tft_disp.print(text);
}

void drawFromProgMem( 
                      uint16_t *bitmap,  // адрес массива
                      int16_t x = 0,     // начало рисования по x
                      int16_t y = 0,     //                     y
                      int16_t w = 160,   // ширина 
                      int16_t h = 128    // высота картинки
                    ){
  for (int i = 0; i < h; ++i) {
    for (int j = 0; j < w; ++j) {
      uint16_t px = pgm_read_word(bitmap + i * 160 + j);
      if (px != 0) tft_disp.drawPixel(j+x, i+y, px);
    }
  }
}

void getData() {

  float gr_x, gr_y, gr_z;

  gr_x = 0;
  gr_y = 0;
  gr_z = -10;

  // Вращение по X

  gr_x = gr_x;
  gr_y = gr_z * sinf(gyrX_f);
  gr_z = gr_z * cosf(gyrX_f);

  // Вращение по Y

  gr_x = gr_z * sinf(gyrY_f);
  gr_y = gr_y;
  gr_z = gr_z * cosf(gyrY_f);

  // Вращение по Z

  gr_x = gr_x * cosf(gyrZ_f) - gr_y * sinf(gyrZ_f);
  gr_y = -gr_x * sinf(gyrZ_f) + gr_y * cosf(gyrZ_f);
  gr_z = gr_z;

  time_rec[1] = time_rec[0];

  data1[0] = mpu.getAccelerationX();
  data1[1] = mpu.getAccelerationY();
  data1[2] = mpu.getAccelerationZ();
  time_rec[0] = millis();

  accX_f = data1[0] / 3276.8 * 2;
  accY_f = data1[1] / 3276.8 * 2;
  accZ_f = data1[2] / 3276.8 * 2;

  x = fl_X.f(accX_f);
  y = fl_Y.f(accY_f);
  z = fl_Z.f(accZ_f);

  Serial.print(z);
  Serial.print(" ");
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(gr_z);
  Serial.print(" ");
  Serial.print(gr_x);
  Serial.print(" ");
  Serial.print(gr_y);
  Serial.println(" ");
}
