#include <TinyGPS.h>
#include <SoftwareSerial.h>

#define gpsRXPIN 5
#define gpsTXPIN 4
#define XbeeRXPIN 2
#define XbeeTXPIN 3

SoftwareSerial tGPS (gpsRXPIN, gpsTXPIN);
SoftwareSerial XBee (XbeeRXPIN, XbeeTXPIN);
TinyGPS gps;

float TargetLat;
float TargetLon;
int Status = 0;

void SendGpsLocation(float Lat, float Lon, int Status);
void getGPS(float* lat, float* lon, int* Status);

void setup() {
  pinMode(gpsRXPIN, INPUT);
  pinMode(XbeeTXPIN, OUTPUT);

  tGPS.begin(9600);
  Serial.begin(9600);
  XBee.begin(9600);

  while (!Serial) {}

  Serial.println("Hello");
}

void loop() {
  tGPS.listen();
  getGPS(&TargetLat, &TargetLon, &Status);

  Serial.println(Status);
  Serial.println(TargetLon);
  Serial.println(TargetLat);

  if (Status == 2) {
    XBee.listen();
    while (!XBee) {}

    SendGPSLocation(TargetLat, TargetLon, Status);

    delay(1000);
  }
}

void SendGPSLocation(float Lat, float Lon, int Status) {
  String latData = String(Lat, 5);
  String lonData = String(Lon, 5);

  String GPSdata = "$";

  GPSdata += latData;
  GPSdata += ",";
  GPSdata += lonData;

  XBee.println(GPSdata);

  Serial.println(GPSdata);
}

void getGPS(float* lat, float* lon, int* Status) {
  float flat;
  float flon;
  unsigned long fix_age;

  while (tGPS.available()) {
    int c = tGPS.read();
    if (gps.encode(c));
    {}
  }

  gps.g_get_position(&flat, &flon, &fix_age);
  *lat = flat;
  *lon = flon;

  if (fix_age == TinyGPS::GPS_INVALID_AGE) {
    *Status = 0;
  }

  else if (fix_age > 5000) {
    *Status = 1;
  }

  else {
    *Status = 2;
  }
}
