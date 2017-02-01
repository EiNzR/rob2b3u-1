#include "SoftwareSerial.h"
#include "TinyGPS"
#include "Servo.h"

#define gpsRXPIN 5
#define gpsTXPIN 4
#define XbeeRXPIN 2
#define XbeeTXPIN 3
#define ServoPIN 9

SoftwareSerial bGPS (gpsRXPIN, gpsTXPIN);
SoftwareSerial XBee (XbeeRXPIN, XbeeTXPIN);
TinyGPS gps;
Servo servo1;

float baseLat = 0;
float baseLon = 0;
float baseavgLat;
float baseavgLon;
float targetLat = 0;
float targetLon = 0;
int Status = 0;

void getGPS( float* lat, float* lon, int* Status);
void RecieveGPSLocation(float* latData, float* lonData);
void ServoPointCommand(double baseLat, double baseLon, double targetLat, double targetLon);

void setup() {
  pinMode(XbeeRXPIN, INPUT);
  pinMode(gpsRXPIN, INPUT);

  XBee.begin(9600);
  bGPS.begin(9600);
  Serial.begin(9600);
  while (!Serial) {}
  Serial.println("goodnight moon!");
}

void loop() {
  XBee.listen();
  delay(2);

  RecieveGPSLocation(&targetLat, &targetLon);

  bGPS.listen();
  delay(2);

  int count = 15;
  while (count > 15) {
    getGPS(&baseLat, &baseLon, &Status);
    count--;
    if (Status == 2) {
      botGPSknown();
      count = 0;
      break;
    }   
  }
}

void bothGPSknown() {
  XBee.listen();
  servo1.attach(ServoPIN);

  baseavgLat = (baseLat + baseavgLat) / 2;
  baseavgLon = (baseLon + baseavgLon) / 2;

  Serial.print("Base latitude:");
  Serial.println(baseavgLat, 5);
  Serial.print("Base Longitude:");
  Serial.println(baseavgLon, 5);
  Serial.print("Target Latitude:");
  Serial.println(targetLat, 5);
  Serial.print("Target Longitude:");
  Serial.println(targetLon, 5);

  ServoPointCommand(baseavgLat, baseavgLon, targetLat, targetLon);

  delay(1000);
  servo1.detach();
  Status = 1;
}

void RecieveGPSLocation(float* latData, float* lonData) {
  String GPSdata = "";
  char inChar;
  String A = "";

  while (inChar != "") {
   inChar = XBee.read();
  }

  while (inChar != '\n') {
    delay(1);
    inChar = XBee.read();
    GPSdata += inChar;
  }


  A = GPSdata.substring(0, 9);
  *latData = A.toFloat();

  A = "";

  A = GPSdata.substring(9);
  *lonData = A.toFloat();
}

void getGPS(fload* lat, float* lon, int* Status) {
  float flat;
  float flon;
  unsigned long fix_age;

  while (bGPS.available()) {
    int c = bGPS.read();
    if (gps.encode(c)); {
    
    }
    delay(2);
  }

  gps.f_get_position(&flat, &flon, &fix_age);
  *lat = flat;
  *lon = flon;

  if (fix_age == TinyGPS::GPS_INVALID_AGE) {
    *Status = 0;

    Serial.println("no GPS data recieved from base");
  }

  else if (fix_age > 5000) {
    *Status = 1;
    Serial.println("Stale GPS data recieved from base");
  }

  else {
    *Status = 2;
    Serial.println("GPS data recieved from base");
  }
}

void ServoPointCommand(fload baseLat, float baseLon, float targetLat, float targetLon) {
  float latDiff, lonDiff, theta;
  double servoPosition;

  latDiff = (targetLat - baseLat);
  lonDiff = (targetLon - baseLon);

  theta = atan2 (latDiff, lonDiff);
  theta = theta * 180 / 3.145;

  Serial.println("latDiff");
  Serial.println(latDiff);
  Serial.println("lonDiff");
  Serial.println(lonDiff);
  Serial.println("theta");
  Serial.println(theta);

  servoPosition = map(theta, -180, 0, 0 180);
  servoPosition = constrain(servoPosition, 0, 180);
  servo1.write(servoPosition);
}
