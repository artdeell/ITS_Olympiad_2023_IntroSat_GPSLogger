#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>
TinyGPSPlus gps;
File logFile;

unsigned long lastGpsReadTime;
const unsigned long gpsTimeout = 1000;
uint32_t centisecondTimestamp;
bool    gpsTimedOut;
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

SoftwareSerial SerialGPS(PB8, PB9); // RX, TX
void setup() {
  pinMode(PC13, OUTPUT);
  led_off();
  Serial1.begin(115200);
  SerialGPS.begin(9600);
  SPI.setMOSI(PA7);
  SPI.setMISO(PA6);
  SPI.setSCLK(PA5);
  SPI.setSSEL(PA4);
  if(!SD.begin(PA4)) {
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

void loop() {
  // put your main code here, to run repeatedly:
  if(SerialGPS.available() > 0) {
    lastGpsReadTime = millis();
    gpsTimedOut = 0;
    while(SerialGPS.available() > 0) 
      gps.encode(SerialGPS.read());
  }else{
    if(millis() - lastGpsReadTime > gpsTimeout)
      gpsTimedOut = 1;
  }
  if(gps.location.isUpdated()) {
    if(gps.location.isValid()) led_off();
    else led_on();
  }
  
  if(gps.satellitesStats.isUpdated()) {
    Serial1.print("Satellites visible: ");
    Serial1.println(gps.satellitesStats.nrSatsVisible());
    Serial1.print("Satellites tracked: ");
    Serial1.println(gps.satellitesStats.nrSatsTracked());
  }
  
  if(gps.time.isUpdated()) {
    centisecondTimestamp = 
      gps.time.centisecond() + 
      gps.time.second() * 100 + 
      gps.time.minute() * 60 * 100 +
      gps.time.hour() * 60 * 60 * 100;
  }
  writeLogLine();
}

uint32_t getTimestampCorrection() {
  return gps.time.age() / 100;
}

void writeLogLine() {
  logFile.print(millis());
  logFile.print(',');
  if(!gpsTimedOut && gps.time.isValid()) {
    logFile.print(centisecondTimestamp + getTimestampCorrection());
  }
  logFile.print(',');
  if(!gpsTimedOut && gps.location.isValid()) {
    logFile.write(gps.location.Mode());
    logFile.print(',');
    logFile.print(gps.location.lat(), 6);
    logFile.print(',');
    logFile.print(gps.location.lng(), 6);
    logFile.print(',');
    logFile.print(gps.altitude.meters());
  } else {
    logFile.print(gpsTimedOut ? 'T' : 'I');
    logFile.print(',');
    logFile.print(',');
    logFile.print(',');
  }
  logFile.println();
  logFile.flush();
}
