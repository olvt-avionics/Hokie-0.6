/*
 * Calibration RPM: (30 / pi) sqrt(scale * g / r) = rpm (x, y scales are +- 16g, z scale is +- 75g)
 * +X (r = 0.125 m) (GND at row 20): 338.38 rpm
 * -X (r = 0.125 m) (GND at row 22): 338.38 rpm
 * +Y (r = 0.096 m) (GND at row 5): 386.13 rpm
 * -Y (r = 0.11 m) (GND at row 7): 360.72 rpm
 * +Z (r = 0.133 m) (GND at row 4): 710.25 rpm
 * -Z (r = 0.092 m) (GND at row 6): 853.97 rpm
 */
#include <SD.h>

File myFile;

const int XPIN = A2;
const int YPIN = A1;
const int ZPIN = A0;
const int PWRPIN = 3;

float startTime;
float currentTime;
float sumX = 0;
float sumY = 0;
float sumZ = 0;
int count = 0;
boolean done = false;

void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  pinMode(XPIN, INPUT);
  pinMode(YPIN, INPUT);
  pinMode(ZPIN, INPUT);
  pinMode(PWRPIN, OUTPUT);

  analogReference(EXTERNAL);
  digitalWrite(PWRPIN, HIGH);

  if (!SD.begin(10)) {
    Serial.println("SD Error");
    return;
  }
  
  myFile = SD.open("cali.txt", FILE_WRITE);
  
  if (myFile) {
    myFile.println("Calibration");
    myFile.println("X\tY\tZ");
    myFile.close();
  } else {
    Serial.println("File Error");
  }

  delay(5000);
  startTime = millis();
}

void loop() {

  if (!done) {
    float rawX = analogRead(XPIN);
    float rawY = analogRead(YPIN);
    float rawZ = analogRead(ZPIN);
    
    sumX += rawX;
    sumY += rawY;
    sumZ += rawZ;
    count++;
    
    Serial.print(rawX); Serial.print("\t");
    Serial.print(rawY); Serial.print("\t");
    Serial.print(rawZ); Serial.println();
    
    myFile = SD.open("cali.txt", FILE_WRITE);
    
    if (myFile) {
      myFile.print(rawX); myFile.print("\t");
      myFile.print(rawY); myFile.print("\t");
      myFile.print(rawZ); myFile.println();
      myFile.close();
      }
      else {
        Serial.println("File Error in Loop");
      }
  }

  currentTime = millis();
  
  if ((currentTime - startTime) >= 30000 && !done) {
    done = true;
    float avgX = sumX / count;
    float avgY = sumY / count;
    float avgZ = sumZ / count;

    myFile = SD.open("cali.txt", FILE_WRITE);
    
    if (myFile) {
      myFile.print("Averages ("); myFile.print(count); myFile.print(" measurements):"); myFile.println();
      myFile.print(avgX); myFile.print("\t");
      myFile.print(avgY); myFile.print("\t");
      myFile.print(avgZ); myFile.println();
      myFile.close();
      }
      else {
        Serial.println("File Error in Termination");
      }
  }

}
