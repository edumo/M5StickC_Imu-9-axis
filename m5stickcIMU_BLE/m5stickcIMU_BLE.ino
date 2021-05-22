#include <M5StickC.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_pm.h"
#define SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_RX_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"

//#include <Wire.h>
//#include "Adafruit_Sensor.h"
//#include <Adafruit_BMP280.h>
#include "bmm150.h"
#include "bmm150_defs.h"

bmm150_mag_data value_offset;
BMM150 bmm = BMM150();

BLEServer *pServer = NULL;
BLEService *pService = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

float temp = 0;

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float magX = 0;
float magY = 0;
float magZ = 0;

float acc[3];
float accOffset[3];
float gyro[3];
float gyroOffset[3];
float mag[3];
float magOffset[3];
float magmax[3];
float magmin[3];

uint8_t setup_flag = 1;
uint8_t action_flag = 1;

uint8_t teapotPacket[10] = { '$', 0, 0, 0, 0, 0, 0, 0, 0, '\n' };
//Mahony fix updated_balac_pid
float power = 0;
float dt, preTime;
float loopfreq;
float myQ0 = 1.0, myQ1 = 0.0, myQ2 = 0.0, myQ3 = 0.0;

boolean withMagnet = false;

long lastBLEMsg = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


bool InitBLEServer()
{

  M5.begin();
  M5.IMU.Init();
  M5.Lcd.setRotation(3);
  //M5.Axp.ScreenBreath(7);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(30, 0);
  M5.Lcd.println("IMU TEST");

  BLEDevice::init("M5StickC-IMU-BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_RX_UUID,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_WRITE  |
                        BLECharacteristic::PROPERTY_NOTIFY |
                        BLECharacteristic::PROPERTY_INDICATE
                      );

  pTxCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  return true;
}


void setup() {
  // put your setup code here, to run once:
  M5.begin();
  Wire.begin(0, 26);
  Serial.begin(115200);
  InitBLEServer();
  delay(50);
  pService->start();
  pServer->getAdvertising()->start();

  M5.Axp.ScreenBreath(8);
  M5.MPU6886.Init();

  if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("Chip ID can not read!");
    withMagnet = false;
  } else {
    Serial.println("Initialize done!");
    withMagnet = true;
  }

  Serial.print("\n\rCalibrate done..");
  initData();
}


/* Modified version of the M5.MPU6886.GetAhrsData to accept the input of the sample frequency */
void myGetAhrsData(float samplefrequency) {
  applycalibration();

  bmm150_mag_data value;
  bmm.read_mag_data();

  magX = bmm.raw_mag_data.raw_datax - value_offset.x;
  magY = bmm.raw_mag_data.raw_datay - value_offset.y;
  magZ = bmm.raw_mag_data.raw_dataz - value_offset.z;

  if (withMagnet)
    myMahonyAHRSupdateIMU9Axis(gyro[0] * DEG_TO_RAD, gyro[1] * DEG_TO_RAD, gyro[2] * DEG_TO_RAD, acc[0], acc[1], acc[2], -magX, magY, -magZ, &myQ0, &myQ1, &myQ2, &myQ3, samplefrequency);
  else
    myMahonyAHRSupdateIMU(gyro[0] * DEG_TO_RAD, gyro[1] * DEG_TO_RAD, gyro[2] * DEG_TO_RAD, acc[0], acc[1], acc[2], &myQ0, &myQ1, &myQ2, &myQ3, samplefrequency);
}

void loop() {

  M5.update();
  if (M5.BtnB.wasReleased()) {
    esp_restart();
  }

  dt = (micros() - preTime) / 1000000; // 処理時間を求める
  preTime = micros(); // 処理時間を記録
  loopfreq = 1 / dt;

  myGetAhrsData(loopfreq);

  // from 0 to 2 , max is 65536 -> 2 bytes
  unsigned int qxInt = (myQ0 + 1) * 32768;
  unsigned int qyInt = (myQ1 + 1) * 32768;
  unsigned int qzInt = (myQ2 + 1) * 32768;
  unsigned int qwInt = (myQ3 + 1) * 32768;

  M5.Lcd.setCursor(0, 15, 2);
  M5.Lcd.printf("q0: %6d %2.3f", qxInt, myQ0);
  M5.Lcd.setCursor(0, 31, 2);
  M5.Lcd.printf("q1: %6d %2.3f", qyInt, myQ1);
  M5.Lcd.setCursor(0, 48, 2);
  M5.Lcd.printf("q2: %6d %2.3f", qzInt, myQ2);
  M5.Lcd.setCursor(0, 64, 2);
  M5.Lcd.printf("q3: %6d %2.3f", qwInt, myQ3);

  drawScreen();

  teapotPacket[3] = qxInt & 0xff;
  teapotPacket[2] = (qxInt >> 8);
  teapotPacket[5] = qyInt & 0xff;
  teapotPacket[4] = (qyInt >> 8);
  teapotPacket[7] = qzInt & 0xff;
  teapotPacket[6] = (qzInt >> 8);
  teapotPacket[9] = qwInt & 0xff;
  teapotPacket[8] = (qwInt >> 8);

  if (millis() > lastBLEMsg + 20) {
    pTxCharacteristic->setValue( teapotPacket, 10 );
    pTxCharacteristic->notify();
    lastBLEMsg = millis();
    Serial.write(teapotPacket, 10);
    Serial.println();
  }


  float batVoltage = M5.Axp.GetBatVoltage();
  power = ( batVoltage < 3.2 ) ? 0 : ( batVoltage - 3.2 ) * 100;


  if (M5.BtnA.wasPressed())
  {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print("begin calibration IMU in 3 seconds, dont move");
    delay(3000);
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.print("Calibrating");
    calibrate6886();
    if (withMagnet) {
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.print("begin calibration compass in 3 seconds, move");
      delay(3000);
      M5.Lcd.setCursor(0, 40);
      M5.Lcd.print("Flip + rotate core calibration");
      calibrate(10000);
      delay(100);
    }
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(GREEN, BLACK);
  }

}

void drawScreen() {

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("bat:%2.1f ", power);
  M5.Lcd.printf("fps:%3.2f ", loopfreq);
  M5.Lcd.printf("mag %o", withMagnet);
}
