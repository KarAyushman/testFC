#include <Arduino.h>
#include "I2CScanner.h"
#include <Wire.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFunLSM6DSO.h>
#include <Adafruit_BMP085.h>
#include <SD.h>
#include <SdFat.h>
#include <SimpleKalmanFilter.h>

// =============================================
// ===               BMP180                  ===
// =============================================

Adafruit_BMP085 bmp;
SimpleKalmanFilter pressureEstimate(1, 1, 1.1);
int bmpInitCnt = 0;
float gndAlt = 0.0, actAlt = 0.0, actPress = 0.0, est_alt = 0.0, lastAlt = 0.0;
double bmpTemp = 0.0;


// =============================================
// ===              LSM6DSO                  ===
// =============================================

LSM6DSO imu;
int lsmInitCnt = 0;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
double launchAx = 0.00, launchAy = 0.00, launchAz = 0.00;
double lsmTemp = 0.0;

double yaw = 0.00;
double pitch = 0.00;


// =============================================
// ===              SD CARD                  ===
// =============================================

String filename;
File myFile;
int sd_count = 0;
bool FL = false;
bool fileclosed = false;

// =============================================
// ===          Pin Definitions              === 
// =============================================

int jj = 0; // 0 = red, 1 = green, 2 = blue
const int led_pins[3] = {25,26,27};
const int BUZZ = 32;
const int SD_PIN = 4;

// =============================================
// ===          Err Definitions              ===
// =============================================

int errState = 0;
// 0 = NoError, 1 = SensorInitError

// =============================================
// ===          State Variables              ===
// =============================================

double launchTime = 0.0;
double launchThreshold = 1.2;
int launchState = 0, pyroState = 0, landState = 0;


void ledSetup(void){
  analogWrite(led_pins[0], 0);
  analogWrite(led_pins[1], 255);
  analogWrite(led_pins[2], 255);
  delay(100);
  analogWrite(led_pins[0], 255);
  analogWrite(led_pins[1], 0);
  analogWrite(led_pins[2], 255);
  delay(100);
  analogWrite(led_pins[0], 255);
  analogWrite(led_pins[1], 255);
  analogWrite(led_pins[2], 0);
  delay(100);
}

void ledState(int i){
  // Startup
  if(i == 1){
    analogWrite(led_pins[0], 3);
    analogWrite(led_pins[1], 78);
    analogWrite(led_pins[2], 252);
  }
  // Launch Detect
  else if(i == 2){
    analogWrite(led_pins[0], 252);
    analogWrite(led_pins[1], 78);
    analogWrite(led_pins[2], 3);
  }
  // Launch Ascent
  else if(i==3){
    analogWrite(led_pins[0], 3);
    analogWrite(led_pins[1], 252);
    analogWrite(led_pins[2], 211);
  }
  // Launch Descent
  else if(i==4){
    analogWrite(led_pins[0], 252);
    analogWrite(led_pins[1], 215);
    analogWrite(led_pins[2], 3);
  }
  // Landed
  else if(i==5){
    analogWrite(led_pins[0], 244);
    analogWrite(led_pins[1], 3);
    analogWrite(led_pins[2], 252);
  }
}

void state0();
void state1();
void state2();
void state3();

void lsmInit(void){
  if(imu.begin())Serial.println("IMU Initialized");
  else if(lsmInitCnt<5){ 
    delay(200);
    lsmInitCnt++;
    lsmInit();
  }
  else{
    ledState(6);
  }
  // LSM Configuration Settings
  if( imu.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings.");
  imu.setAccelRange(16);

}

void bmpInit(void){
  if(bmp.begin())Serial.println("BMP Initialized");
  else if(bmpInitCnt<5){ 
    delay(200);
    bmpInitCnt++;
    bmpInit();
  }
  else{
    ledState(6);
  }
  for(int i = 0; i < 20; i++){
    gndAlt += bmp.readAltitude(101325)*0.05;
  }
}

void Write() {
  myFile = SD.open(filename, FILE_APPEND);

  if (myFile) {

    //Writing in SD Card!
    myFile.print(millis());
    myFile.print(",");
    myFile.print(actPress);
    myFile.print(",");
    myFile.print(actAlt);
    myFile.print(",");
    myFile.print(est_alt);
    myFile.print(",");
    myFile.print(lsmTemp);
    myFile.print(",");
    myFile.print(bmpTemp);
    myFile.print(",");
    myFile.print(accX);
    myFile.print(",");
    myFile.print(accY);
    myFile.print(",");
    myFile.print(accZ);
    myFile.print(",");
    myFile.print(gyroX);
    myFile.print(",");
    myFile.print(gyroY);
    myFile.print(",");
    myFile.print(gyroZ);
    myFile.print(",");
    myFile.print(yaw);
    myFile.print(",");
    myFile.print(pitch);
    myFile.print(",");
    myFile.print(launchState);
    myFile.print(",");
    myFile.print(pyroState);
    myFile.print(",");
    myFile.println(landState);
    myFile.close();

    Serial.print(millis());
    Serial.print(",");
    Serial.print(actPress);
    Serial.print(",");
    Serial.print(actAlt);
    Serial.print(",");
    Serial.print(est_alt);
    Serial.print(",");
    Serial.print(lsmTemp);
    Serial.print(",");
    Serial.print(bmpTemp);
    Serial.print(",");
    Serial.print(accX);
    Serial.print(",");
    Serial.print(accY);
    Serial.print(",");
    Serial.print(accZ);
    Serial.print(",");
    Serial.print(gyroX);
    Serial.print(",");
    Serial.print(gyroY);
    Serial.print(",");
    Serial.print(gyroZ);
    Serial.print(",");
    Serial.print(yaw);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(launchState);
    Serial.print(",");
    Serial.print(pyroState);
    Serial.print(",");
    Serial.println(landState);
  }
}

boolean loadSDFile() {
  int i = 0;
  boolean file = false;

  while (!file && i < 1024) {
    filename = "/"+(String)i + "FL.csv";

    if (!SD.exists(filename)) {
      myFile = SD.open(filename, FILE_WRITE);
      delay(10);
      myFile.close();
      file = true;
    }
    i++;
  }

  return file;
}

void initSD() {
  SD.begin(SD_PIN);
  //Create a file with new name
  if (!loadSDFile()) {
    Serial.println("SD Initialization Failed");
   ledState(6);
  }
  Serial.println(filename);
  myFile = SD.open(filename, FILE_WRITE);
  Serial.println(myFile);
  if (myFile) {
    myFile.print("Time");
    myFile.print(",");
    myFile.print("Pressure"); 
    myFile.print(",");
    myFile.print("Altitude");
    myFile.print(",");
    myFile.print("Kal_Alt");
    myFile.print(",");
    myFile.print("lsmTemp");
    myFile.print(",");
    myFile.print("bmpTemp");
    myFile.print(",");
    myFile.print("Ax");
    myFile.print(",");
    myFile.print("Ay");
    myFile.print(",");
    myFile.print("Az");
    myFile.print(",");
    myFile.print("Gx");
    myFile.print(",");
    myFile.print("Gy");
    myFile.print(",");
    myFile.print("Gz");
    myFile.print(",");
    myFile.print("Yaw");
    myFile.print(",");
    myFile.print("Pitch");
    myFile.print(",");
    myFile.print("Launch");
    myFile.print(",");
    myFile.print("Pyro");
    myFile.print(",");  
    myFile.println("Land");

    myFile.close();
  }
}

void getAlt() {
  actAlt = bmp.readAltitude(101325) - gndAlt;
  actPress = bmp.readPressure();
  // est_alt = pressureEstimate.updateEstimate(actAlt);
  bmpTemp = bmp.readTemperature();
  est_alt = actAlt;
}

void getIMU(){
  accX = imu.readFloatAccelX();
  accY = imu.readFloatAccelY();
  accZ = imu.readFloatAccelZ();
  gyroX = imu.readFloatGyroX();
  gyroY = imu.readFloatGyroY();
  gyroZ = imu.readFloatGyroZ();
  lsmTemp = imu.readTempC();
}

void getData(void){
  getIMU();
  getAlt();
  Write();
  
}

void chargePyro(int i){
  if(i == 1){
    digitalWrite(2, HIGH);
    pyroState = 1;
  }
  else if(millis()-launchTime > 22000 && launchTime > 0 && pyroState!=0){
    digitalWrite(2, HIGH);
    // state3();
  }
  
}

void state0(void){
  int checkCnt = 0;
  int flag = 0;
  float lastAlt;
  double lastTime = 0.0;
  ledState(2);
  while(!flag){
    lastAlt = est_alt;
    Serial.println(est_alt);
    getData();
    chargePyro(0);
    if(checkCnt >= 3){
      flag=1;
      launchState = 1;
      launchTime = millis();
    }
    else if(est_alt>= 1.5 && millis()-lastTime>100){
      checkCnt++;
      lastTime = millis();
    }
    else if (checkCnt>0 && est_alt<1){
      checkCnt--;
    }
  }
  state1();
}

void state1(void){
  int checkCnt = 0;
  int flag = 0;
  double maxAlt = 0.0;
  ledState(3);
  while(!flag){
    Serial.println("Ascent");
    lastAlt = est_alt;
    if(est_alt>maxAlt)maxAlt = est_alt;
    getData();
    chargePyro(0);
    if(checkCnt >= 3){
      flag=1;
      chargePyro(1);
    }
    else if(est_alt <= lastAlt && maxAlt-est_alt>=1){
      checkCnt++;
    }
    else if(checkCnt>0){
      checkCnt--;
    }
  }
  state2();
}

void state2(void){
  int checkCnt = 0;
  int flag = 0;
  double lastTime = 0.0;
  ledState(4);
  while(!flag){
    Serial.println("Descent");
    lastAlt = est_alt;
    getData();
    chargePyro(0);
    if(checkCnt >= 5){
      flag = 1;
    }
    else if(est_alt - lastAlt < 0.05 && millis()-lastTime>500){
      lastTime = millis();
      checkCnt++;

    }
    else if(checkCnt>0 && est_alt-lastAlt >= 0.02){
      checkCnt--;
    }
    if(millis()-launchTime >= 35000){
      state3();
    }
  }
  state3();
}

void state3(void){
  Serial.println("Landed");
  ledState(5);
  getData();
  landState = 1;
  chargePyro(0);
  state3();
}


// void buzzMeUp(void){

// }

void setup() {
  ledSetup();
  // ledState(1);
  // Wire.begin();
  // Serial.begin(115200);
  // pinMode(BUZZ, OUTPUT);
  // lsmInit();
  // bmpInit();
  // initSD();
  // delay(2000);
  // ledState(2);
}

void loop() {
  // state0();
  // if(pyroState == 1){
  //   state3();
  // }
}
