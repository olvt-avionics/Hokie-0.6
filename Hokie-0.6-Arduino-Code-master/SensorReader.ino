#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*
 * Author: Nicholas Jones
 * 
 * This program reads and prints data from two ADXL377s and two BNO055s to a serial stream.
 * The ADXL377s must share a common ground
 * 
 * SPECIAL PINOUTS:
 * ADXL377 1 X Y Z pins to A0 A1 A2
 * ADXL377 2 X Y Z pins to A4 A5 A6
 * BNO055 2 ADR pin to Digital 0
 */

int SAMPLE_RATE_DELAY = 1;

int X1PIN = A0;
int Y1PIN = A1;
int Z1PIN = A2;

int X2PIN = A4;
int Y2PIN = A5;
int Z2PIN = A6;

int AddressPin = 0;

Adafruit_BNO055 bno1 = Adafruit_BNO055(-1, BNO055_ADDRESS_A);
Adafruit_BNO055 bno2 = Adafruit_BNO055(-1, BNO055_ADDRESS_B);

int counter;
long time;

void setup() {
  Serial.begin(115200);
  
  pinMode(X1PIN, INPUT);
  pinMode(Y1PIN, INPUT);
  pinMode(Z1PIN, INPUT);

  pinMode(X2PIN, INPUT);
  pinMode(Y2PIN, INPUT);
  pinMode(Z2PIN, INPUT);

  analogReference(EXTERNAL);

  pinMode(AddressPin, OUTPUT);
  digitalWrite(AddressPin, HIGH);

  if (!bno1.begin(bno1.OPERATION_MODE_NDOF)) {
    Serial.print("BNO055 1 not detected");
    while(1);
  }
  bno1.setExtCrystalUse(true);

  if (!bno2.begin(bno2.OPERATION_MODE_NDOF)) {
    Serial.print("BNO055 2 not detected");
    while(1);
  }
  bno2.setExtCrystalUse(true);

  counter = 0;

  delay(5000);
}

void loop() {
    time = millis();
    
    //Read ADXL377 1
    int rawX1 = analogRead(X1PIN);
    int rawY1 = analogRead(Y1PIN);
    int rawZ1 = analogRead(Z1PIN);
    
    //Read ADXL377 2
    int rawX2 = analogRead(X2PIN);
    int rawY2 = analogRead(Y2PIN);
    int rawZ2 = analogRead(Z2PIN);
    
    //Read BNO055 1
    int temp1 = bno1.getTemp();
    imu::Quaternion quat1 = bno1.getQuat();
    imu::Vector<3> accel1 = bno1.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> magnet1 = bno1.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> gyro1 = bno1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> gravity1 = bno1.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    sensors_event_t event1;
    bno1.getEvent(&event1);

    //Read BNO055 2
    int temp2 = bno2.getTemp();
    imu::Quaternion quat2 = bno2.getQuat();
    imu::Vector<3> accel2 = bno2.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> magnet2 = bno2.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> gyro2 = bno2.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> gravity2 = bno2.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    sensors_event_t event2;
    bno2.getEvent(&event2);

    Serial.print(counter);Serial.print(", ");
    Serial.print(time);Serial.print(", ");
  
    //Print ADXL377 1 Data
    Serial.print(rawX1);Serial.print(", ");
    Serial.print(rawY1);Serial.print(", ");
    Serial.print(rawZ1);Serial.print(", ");

    //Print ADXL377 2 Data
    Serial.print(rawX2);Serial.print(", ");
    Serial.print(rawY2);Serial.print(", ");
    Serial.print(rawZ2);Serial.print(", ");
  
    //BNO055 1 Calibration Status
    displayCalStatus(1);
  
    //Raw BNO055 1 Data
    Serial.print(temp1);Serial.print(", ");
    Serial.print(quat1.w(), 4);Serial.print(", ");
    Serial.print(quat1.x(), 4);Serial.print(", ");
    Serial.print(quat1.y(), 4);Serial.print(", ");
    Serial.print(quat1.z(), 4);Serial.print(", ");
    Serial.print(accel1.x());Serial.print(", ");
    Serial.print(accel1.y());Serial.print(", ");
    Serial.print(accel1.z());Serial.print(", ");
    Serial.print(magnet1.x());Serial.print(", ");
    Serial.print(magnet1.y());Serial.print(", ");
    Serial.print(magnet1.z());Serial.print(", ");
    Serial.print(gyro1.x());Serial.print(", ");
    Serial.print(gyro1.y());Serial.print(", ");
    Serial.print(gyro1.z());Serial.print(", ");
    Serial.print(gravity1.x());Serial.print(", ");
    Serial.print(gravity1.y());Serial.print(", ");
    Serial.print(gravity1.z());Serial.print(", ");
  
    //Fusion BNO055 1 Data
    Serial.print(event1.orientation.x, 4);Serial.print(", ");
    Serial.print(event1.orientation.y, 4);Serial.print(", ");
    Serial.print(event1.orientation.z, 4);Serial.print(", ");

    //BNO055 2 Calibration Status
    displayCalStatus(2);
  
    //Raw BNO055 2 Data
    Serial.print(temp2);Serial.print(", ");
    Serial.print(quat2.w(), 4);Serial.print(", ");
    Serial.print(quat2.x(), 4);Serial.print(", ");
    Serial.print(quat2.y(), 4);Serial.print(", ");
    Serial.print(quat2.z(), 4);Serial.print(", ");
    Serial.print(accel2.x());Serial.print(", ");
    Serial.print(accel2.y());Serial.print(", ");
    Serial.print(accel2.z());Serial.print(", ");
    Serial.print(magnet2.x());Serial.print(", ");
    Serial.print(magnet2.y());Serial.print(", ");
    Serial.print(magnet2.z());Serial.print(", ");
    Serial.print(gyro2.x());Serial.print(", ");
    Serial.print(gyro2.y());Serial.print(", ");
    Serial.print(gyro2.z());Serial.print(", ");
    Serial.print(gravity2.x());Serial.print(", ");
    Serial.print(gravity2.y());Serial.print(", ");
    Serial.print(gravity2.z());Serial.print(", ");
  
    //Fusion BNO055 2 Data
    Serial.print(event2.orientation.x, 4);Serial.print(", ");
    Serial.print(event2.orientation.y, 4);Serial.print(", ");
    Serial.print(event2.orientation.z, 4);Serial.println();

    counter = counter + 1;
    delay(SAMPLE_RATE_DELAY);
}


void displayCalStatus(int select)
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  if (select == 1) {
    bno1.getCalibration(&system, &gyro, &accel, &mag);
  }
  if (select == 2) {
    bno2.getCalibration(&system, &gyro, &accel, &mag);
  }
  
  Serial.print(system, DEC);Serial.print(", ");
  Serial.print(gyro, DEC);Serial.print(", ");
  Serial.print(accel, DEC);Serial.print(", ");
  Serial.print(mag, DEC);Serial.print(", ");
}
