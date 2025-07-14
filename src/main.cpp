#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <SdFat.h>
#include <ESP32Servo.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <queue>

FsFile dataFile;
String logfilename;
MPU6050 mpu;
Adafruit_BMP280 bmp;
SdFat sd;
Servo myservo;
TinyGPSPlus gps;

#define SD_CS 5      
#define SDA_PIN 21
#define SCL_PIN 22
#define SERVO_PIN 25 

HardwareSerial gpsSerial(2);
#define GPS_RX 16
#define GPS_TX 17

HardwareSerial telemetrySerial(1);
#define TELEMETRY_RX 9
#define TELEMETRY_TX 10


String logfile(){
  String basename = "log";
  String extension = ".txt";
  String filename = basename + extension;
  int index = 1;

  while (sd.exists(filename.c_str())){
    filename = basename + String(index++) + extension;
  }
  return filename;
}

bool ignition = false;
bool incline = false;
bool ejection = false;


int count = 0;
unsigned long lastTime = 0;
unsigned long lastSend = 0;


//bmp280 reset code
unsigned long lastResetTime = 0;
const float MIN_VALID_ALTITUDE = -10.0; 
const float MAX_VALID_ALTITUDE = 2000;
const unsigned long RESET_COOLDOWN = 500; // 0.5초 이상 지나야 리셋 다시 시도

bool isPressureInValid(float height) {
  return (height <= MIN_VALID_ALTITUDE || height >= MAX_VALID_ALTITUDE);
}

void ResetBMP280() {
  unsigned long nowtime = millis();
  if (nowtime - lastResetTime < RESET_COOLDOWN) return;

  Serial.println("[BMP280] Resetting sensor due to abnormal pressure");

  //software reset
  Wire.end();
  delay(50);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(50);

  if(!bmp.begin(0x77)) {
    Serial.println("[BMP280] Reinitialization failed!");
  } 
  else{
    Serial.println("[BMP280] Reinitialized successfully.");
  }

  lastResetTime = nowtime;
}




void setup() {
  Serial.begin(9600);
  Wire.begin(SDA_PIN, SCL_PIN);

  //mpu6050 초기화
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 no connection");
    while (1);
  }

  //bmp280 초기화
  if (!bmp.begin(0x77)) {
    Serial.println("BMP280 no connection");
    while (1);
  }

  //sd card 초기화
  if (!sd.begin(SD_CS, SD_SCK_MHZ(10))) {
    Serial.println("SD card no connection");
    while (1);
  }

  //file name 생성
  logfilename = logfile();
  dataFile = sd.open(logfilename.c_str(),FILE_WRITE);
  if(dataFile){
    dataFile.println("No Xg Yg Zg Altitude");
    dataFile.close();
    Serial.print("데이터 저장 파일:");
    Serial.println(logfilename);  
  }
  else{
    Serial.println("File create fail");
    while(1);
  }

  //servo motor 초기화
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);// Standard 50hz servo
  myservo.attach(SERVO_PIN, 500, 2400);
  myservo.write(20);

  Serial.println("시스템 초기화 완료");
  
  
  //neo-6m, telemetry 초기화  
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  telemetrySerial.begin(57600, SERIAL_8N1, TELEMETRY_RX, TELEMETRY_TX);

  Serial.println("Telemetry start");
  
}

// 최근 5개의 roll 값만 가지고 평균을 구하는 슬라이딩 윈도우 함수
std::queue<float> rollQueue;
const int ROLL_WINDOW_SIZE = 5;
float rollSum = 0.0;
float avg_roll = 0.0;

void updateRollAverage(float newRoll) {
  rollQueue.push(newRoll);
  rollSum += newRoll;

  // 최근 5개를 초과하면 가장 오래된 값 제거
  if (rollQueue.size() > ROLL_WINDOW_SIZE) {
    rollSum -= rollQueue.front();
    rollQueue.pop();
  }

  // 평균 계산
  if (rollQueue.size() == ROLL_WINDOW_SIZE) {
    avg_roll = rollSum / ROLL_WINDOW_SIZE;
  } else {
    avg_roll = 0.0; // 아직 5개가 안 됐을 때는 0.0
  }
}


void loop() {
  
  int16_t ax, ay, az;
  
  mpu.getAcceleration(&ax, &ay, &az);
    
  //가속도
  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;

  //고도
  float altitude = bmp.readAltitude(1013.25);

  //roll
  float roll  = atan2(ayg, sqrt(axg * axg + azg * azg)) * 180.0 / PI;
  

  //acceleration
  float acceleration = sqrt(axg*axg + ayg*ayg + azg*azg);

  static unsigned long ignitionTime = 0;


  if(millis() - lastTime >= 333) {
    lastTime = millis();
    
    //로켓 발사 시 측정 시작
    if(!ignition){
      if(acceleration >= 2.0){
        ignition = true;
        dataFile = sd.open(logfilename.c_str(), FILE_WRITE);
        if (dataFile) {
          dataFile.println("Ignition!");
          dataFile.close();
        } 
        Serial.println("Ignition!");
      }
      else{
        return;
      }
    }
    
    count++;

    //roll평균 함수 호출
    updateRollAverage(roll);

    //bmp280 reset code
    if(isPressureInValid(altitude)) {
      ResetBMP280();
    }

    Serial.printf("%d: X=%.2f Y=%.2f Z=%.2f Alt=%.2f roll(avg):%.2f roll:%.2f\n", count, axg, ayg, azg, altitude, avg_roll, roll);
    
    //SD card에 가속도, 고도 기록
    dataFile = sd.open(logfilename.c_str(), FILE_WRITE);
    if (dataFile) {
      dataFile.printf("%d %.2f %.2f %.2f %.2f\n", count, axg, ayg, azg, altitude);
      dataFile.close();
    } 
    else {
      Serial.println("File access fail");
    }
  }
  
  if(ignition && ignitionTime == 0) {
    ignitionTime = millis(); 
  }
  

  while(gpsSerial.available()){
    gps.encode(gpsSerial.read());
  }

  //GPS 데이터 측정
  if(millis() - lastTime >= 333) {
    lastSend = millis();

    if(gps.location.isUpdated()){
      String gpsData = "LAT:" + String(gps.location.lat(), 6) + " LNG:" + String(gps.location.lng(), 6);

      //telemetry로 데이터 수신
      telemetrySerial.println(gpsData);
      Serial.println(gpsData);
    }
    else{
      telemetrySerial.println("check");
      Serial.println("check");
    }
  }
  

  //낙하산 기울기 조건
  if(!incline && (avg_roll <= -18.0)) {
    incline = true;
    Serial.print("incline complete! avgroll: ");
    Serial.println(avg_roll);

    dataFile = sd.open(logfilename.c_str(), FILE_WRITE);
    if(dataFile) {
      dataFile.print("incline complete! avgroll: ");
      dataFile.println(avg_roll);
      dataFile.close();
    } 
  }
  
  //낙하선 사출
  if(incline && ignition && !ejection && (millis() - ignitionTime >= 8000)){  
    Serial.println("Ejection!");
    myservo.write(120);
    ejection = true;
  //낙하산 사출 기록 
    
    dataFile = sd.open(logfilename.c_str(), FILE_WRITE);
    if(dataFile) {
      dataFile.println("Ejection!");
      dataFile.close();
    } 
  }
}