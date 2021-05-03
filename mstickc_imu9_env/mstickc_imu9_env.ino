#include <M5StickC.h>
#include <Wire.h>
#include <Kalman.h>
#include "bmm150.h"
#include "bmm150_defs.h"
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>

BMM150 bmm = BMM150();
bmm150_mag_data value_offset;
Adafruit_BMP280 bme;

float acc[3];
float accOffset[3];
float gyro[3];
float gyroOffset[3];
float mag[3];
float t,t0;
float magOffset[3];
float magmax[3];
float magmin[3];
uint8_t setup_flag = 1;

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

uint8_t teapotPacket[10] = { '$', 0x02, 0, 0, 0, 0, 0, 0, '\r', '\n' };


void setup() {
  M5.begin();
  Wire.begin(0,26);
  M5.IMU.Init();
  Serial.begin(115200);
  calibrate6886();
  pinMode(M5_BUTTON_HOME, INPUT);

  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(40, 0);
  M5.Lcd.println("IMU TEST");
  M5.Lcd.setCursor(0, 10);
  M5.Lcd.println("  X       Y       Z");

  t=0; t0=0;
  magOffset[0]=0;
  magOffset[1]=0;
  magOffset[2]=0;
  if(bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("Chip ID can not read!");
    while(1);
  } else {
    Serial.println("Initialize done!");
    delay(1);
  }
  if (!bme.begin(0x76)){
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  calibrateENV(10);
}

void calibrate6886(){
  float gyroSum[3];
  float accSum[3];
  int counter = 500;
  for(int i = 0; i < counter; i++){
    M5.IMU.getGyroData(&gyro[0],&gyro[1],&gyro[2]);
    M5.IMU.getAccelData(&acc[0],&acc[1],&acc[2]);
    gyroSum[0] += gyro[0];
    gyroSum[1] += gyro[1];
    gyroSum[2] += gyro[2];
    accSum[0] += acc[0];
    accSum[1] += acc[1];
    accSum[2] += acc[2];
    delay(2);
  }
  gyroOffset[0] = gyroSum[0]/counter;
  gyroOffset[1] = gyroSum[1]/counter;
  gyroOffset[2] = gyroSum[2]/counter;
  accOffset[0] = accSum[0]/counter;
  accOffset[1] = accSum[1]/counter;
  accOffset[2] = (accSum[2]/counter) - 1.0;//重力加速度1G、つまりM5ボタンが上向きで行う想定
}

void applycalibration(){
  M5.IMU.getGyroData(&gyro[0],&gyro[1],&gyro[2]);
  M5.IMU.getAccelData(&acc[0],&acc[1],&acc[2]);
  gyro[0] -= gyroOffset[0];
  gyro[1] -= gyroOffset[1];
  gyro[2] -= gyroOffset[2];
  acc[0] -= accOffset[0];
  acc[1] -= accOffset[1];
  acc[2] -= accOffset[2];

  bmm150_mag_data value;
  bmm.read_mag_data();
  value.x = bmm.raw_mag_data.raw_datax - value_offset.x;
  value.y = bmm.raw_mag_data.raw_datay - value_offset.y;
  value.z = bmm.raw_mag_data.raw_dataz - value_offset.z;
  mag[0] = value.x;
  mag[1] = value.y;
  mag[2] = value.z;
}

void calibrateENV(uint32_t timeout)
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  bmm.read_mag_data();
  value_x_min = bmm.raw_mag_data.raw_datax;
  value_x_max = bmm.raw_mag_data.raw_datax;
  value_y_min = bmm.raw_mag_data.raw_datay;
  value_y_max = bmm.raw_mag_data.raw_datay;
  value_z_min = bmm.raw_mag_data.raw_dataz;
  value_z_max = bmm.raw_mag_data.raw_dataz;
  delay(100);

  timeStart = millis();

  while((millis() - timeStart) < timeout)
  {
    bmm.read_mag_data();
    if(value_x_min > bmm.raw_mag_data.raw_datax)
    {
      value_x_min = bmm.raw_mag_data.raw_datax;
    }
    else if(value_x_max < bmm.raw_mag_data.raw_datax)
    {
      value_x_max = bmm.raw_mag_data.raw_datax;
    }
    if(value_y_min > bmm.raw_mag_data.raw_datay)
    {
      value_y_min = bmm.raw_mag_data.raw_datay;
    }
    else if(value_y_max < bmm.raw_mag_data.raw_datay)
    {
      value_y_max = bmm.raw_mag_data.raw_datay;
    }
    if(value_z_min > bmm.raw_mag_data.raw_dataz)
    {
      value_z_min = bmm.raw_mag_data.raw_dataz;
    }
    else if(value_z_max < bmm.raw_mag_data.raw_dataz)
    {
      value_z_max = bmm.raw_mag_data.raw_dataz;
    }
    if(digitalRead(M5_BUTTON_HOME) == LOW){
      setup_flag = 1;
      while(digitalRead(M5_BUTTON_HOME) == LOW);
      break;
    }
    delay(1);
  }
  value_offset.x = value_x_min + (value_x_max - value_x_min)/2;
  value_offset.y = value_y_min + (value_y_max - value_y_min)/2;
  value_offset.z = value_z_min + (value_z_max - value_z_min)/2;
}

void loop() {
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  M5.IMU.getAhrsData(&pitch,&roll,&yaw);

  applycalibration();

//debug gyro
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);//deg
  M5.Lcd.setCursor(140, 20);
  M5.Lcd.print("o/s");

  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyro[0], gyro[1], gyro[2]);//deg
  M5.Lcd.setCursor(140, 30);
  M5.Lcd.print("o/s");

//debug accel
  M5.Lcd.setCursor(0, 40);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", accX, accY, accZ);
  M5.Lcd.setCursor(140, 40);
  M5.Lcd.print("G");

  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", acc[0], acc[1], acc[2]);
  M5.Lcd.setCursor(140, 50);
  M5.Lcd.print("G");

//debug mag
  float heading = atan2(mag[0], mag[1]);
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  bmm.read_mag_data();
  float heading_raw = atan2(bmm.raw_mag_data.raw_datax, bmm.raw_mag_data.raw_datay);
  if(heading_raw < 0)
    heading_raw += 2*PI;
  if(heading_raw > 2*PI)
    heading_raw -= 2*PI;

  M5.Lcd.setCursor(0, 60);
  M5.Lcd.printf("headingDegrees: %2.1f", heading_raw * 180/M_PI);
  M5.Lcd.setCursor(0, 70);
  M5.Lcd.printf("headingDegrees: %2.1f", heading * 180/M_PI);

  delay(10);

  if(!setup_flag){
     setup_flag = 1;
     if(bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    while(1);
  } else {
    delay(0);
  }
  if (!bme.begin(0x76)){
    while (1);
  }
  calibrate6886();
  calibrateENV(100000);
 }
 if(digitalRead(M5_BUTTON_HOME) == LOW){
  setup_flag = 0;
  while(digitalRead(M5_BUTTON_HOME) == LOW);
 }
 M5.Lcd.setCursor(140, 70);
 if (setup_flag==0) {
   M5.Lcd.printf("CAL");
 } else {
   M5.Lcd.printf("   ");
 }


 int yawInt = yaw * 100 + 18000;
  int pitchInt = pitch * 100 + 18000;
  int rollInt = roll * 100 + 18000;

  teapotPacket[3] = yawInt & 0xff;
  teapotPacket[2] = (yawInt >> 8);
  teapotPacket[5] = pitchInt & 0xff;
  teapotPacket[4] = (pitchInt >> 8);
  teapotPacket[7] = rollInt & 0xff;
  teapotPacket[6] = (rollInt >> 8);

  Serial.write(teapotPacket, 10);
}
