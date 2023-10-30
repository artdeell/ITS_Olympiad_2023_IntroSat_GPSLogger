#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <HardwareTimer.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

SoftwareSerial SerialGPS(PB8, PB9); // RX, TX
HardwareTimer gyroClock(TIM1);
MPU9250_WE mpu(0x68);
TinyGPSPlus gps;
File logFile;

uint32_t centisecondTimestamp;
FixMode gpsFixMode;
double  gpsLat;
double  gpsLng;


void led_off() {
  digitalWrite(PC13, 1);
}

void led_on() {
  digitalWrite(PC13, 0);
}

void die() {
  while(1) {
      delay(500);
      led_off();
      delay(500);
      led_on();
    }
}

void timer_vector() {
  const xyzFloat angle = mpu.getAngles();
  logFile.print("G,");
  logFile.print(millis());
  logFile.print(',');
  logFile.print(angle.x);
  logFile.print(',');
  logFile.print(angle.y);
  logFile.print(',');
  logFile.print(angle.z);
  logFile.println();
  logFile.flush();
}

void setup() {
  pinMode(PC13, OUTPUT);
  led_off();

  //gyroClock.setOverflow(60, HERTZ_FORMAT);
  //gyroClock.attachInterrupt(timer_vector);
  
  Serial1.begin(115200);
  SerialGPS.begin(9600);

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  if(!mpu.init()) {
    Serial1.println("MPU dues not respond. But, i guess it is fine...");
    //die();
  }
  mpu.autoOffsets();
  mpu.setAccRange(MPU9250_ACC_RANGE_2G);
  mpu.enableAccDLPF(true);
  mpu.setAccDLPF(MPU9250_DLPF_6); 
  
  SPI.setMOSI(PA7);
  SPI.setMISO(PA6);
  SPI.setSCLK(PA5);
  SPI.setSSEL(PA4);
  if(!SD.begin(PA4)) {
    Serial1.println("Failed to initialize SD-card");
    die();
  }
  
  if(!(logFile = SD.open("gps_log.txt", FILE_WRITE))) {
    die();
  }
  logFile.println();
  logFile.println();
  
  led_on();
  Serial1.println("Successfully initialized!");
  //gyroClock.resume();
}

void loop() {
  // put your main code here, to run repeatedly:
  bool gpsHasUpdates = false;
  while(SerialGPS.available() > 0) 
      gps.encode(SerialGPS.read());
  
  if(gps.location.isUpdated()) {
    gpsHasUpdates = true;
    if(gps.location.isValid()) led_off();
    else led_on();
  }
  
  if(gps.time.isUpdated()) {
    gpsHasUpdates = true;
    centisecondTimestamp = 
      gps.time.centisecond() + 
      gps.time.second() * 100 + 
      gps.time.minute() * 60 * 100 +
      gps.time.hour() * 60 * 60 * 100;
  }
  if(gpsHasUpdates) writeLogLine();
}

uint32_t getTimestampCorrection() {
  return gps.time.age() / 100;
}

void writeLogLine() {
  //gyroClock.pause();
  
  logFile.print("L,");
  logFile.print(millis());
  logFile.print(',');
  if(gps.time.isValid()) {
    logFile.print(centisecondTimestamp + getTimestampCorrection());
  }
  logFile.print(',');
  if(gps.location.isValid()) {
    logFile.write(gps.location.Mode());
    logFile.print(',');
    logFile.print(gps.location.lat(), 6);
    logFile.print(',');
    logFile.print(gps.location.lng(), 6);
    logFile.print(',');
    logFile.print(gps.altitude.meters());
  } else {
    logFile.print('I');
    logFile.print(',');
    logFile.print(',');
    logFile.print(',');
  }
  logFile.println();
  logFile.flush();
  
  //gyroClock.resume();
}
