#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <HardwareTimer.h>
#include <MPU9250_WE.h>
#include <OneWire.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

SoftwareSerial SerialGPS(PB8, PB9); // RX, TX
MPU9250_WE mpu(0x68);
OneWire dallas(PA0);
TinyGPSPlus gps;
File logFile;

bool     isHeaterOn;
uint32_t centisecondTimestamp;
FixMode  gpsFixMode;
double   gpsLat;
double   gpsLng;


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

void dallas_routine() {
  byte temp[2];
  dallas.reset();
  dallas.write(0xCC);
  dallas.write(0xBE);
  temp[0] = dallas.read();
  temp[1] = dallas.read();
  float temperature = ((temp[1] << 8) | temp[0]) * 0.0625;
  if(temperature < 10 && !isHeaterOn) {
    digitalWrite(PB3, 1);
    isHeaterOn = true;
  }
  if(temperature > 15 && isHeaterOn) {
    digitalWrite(PB3, 0);
    isHeaterOn = false;
  }
  
  dallas.reset();
  dallas.write(0xCC);
  dallas.write(0x44);
}

void gyro_routine() {
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
  pinMode(PB3, OUTPUT);
  digitalWrite(PB3, 0);
  led_off();

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
}

unsigned long lastGyroRefreshTime = 0;
unsigned long lastDallasRefreshTime = 0;

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long refMillis;
  if((refMillis = millis()) - lastGyroRefreshTime >= 16) {
    gyro_routine();
    lastGyroRefreshTime = refMillis;
  }
  if((refMillis = millis()) - lastDallasRefreshTime >= 1000) {
    dallas_routine();
    lastDallasRefreshTime = refMillis;
  }
 
  bool gpsHasUpdates = false;

  while(SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  
  if(gps.location.isUpdated()) {
    gpsHasUpdates = true;
    if(gps.location.isValid()) led_off();
    else led_on();
  }

  //if(gps.satellitesStats.isValid() && gps.satellitesStats.isUpdated()) {
  //  Serial1.print("Satellites visible: ");
  //  Serial1.print(gps.satellitesStats.nrSatsVisible());
  //  Serial1.write('\t');
  //  Serial1.print("Satellites tracked: ");
  //  Serial1.println(gps.satellitesStats.nrSatsTracked());
  //}
  
  if(gps.time.isUpdated()) {
    gpsHasUpdates = true;
    centisecondTimestamp = 
      gps.time.centisecond() + 
      gps.time.second() * 100 + 
      gps.time.minute() * 60 * 100 +
      gps.time.hour() * 60 * 60 * 100;
  }
  if(gpsHasUpdates) {
    writeLogLine();
  }
}

uint32_t getTimestampCorrection() {
  return gps.time.age() / 100;
}

void writeLogLine() {  
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
}
