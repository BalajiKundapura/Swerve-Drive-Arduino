#include <Wire.h>
#include <MPU6050_light.h>

TwoWire Wire2(PC9, PA8); // Create a new i2c bus on these pins, adafruit motor shield also will use this bus
MPU6050 mpu(Wire2);



void setupGyro() {

  pinMode(PC9, INPUT_PULLUP); // SCL
  pinMode(PA8, INPUT_PULLUP);  // SDA
  delay(10); // Give the lines a millisecond to stabilize
  Wire2.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelerometer offsets
  Serial.println("Done!\n");
}

float getYaw() {
  mpu.update();
  return -mpu.getAngleZ();
}