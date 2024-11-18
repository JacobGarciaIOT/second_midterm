// IMPORTANT: When including a library in a firmware app, a sub dir prefix is needed
// before the particular .h file.

#include "Adafruit_Sensor.h"
#include "Adafruit_BMP280.h"

// #define BMP_CS A2
// #define BMP_SCK A3
// #define BMP_MISO A4
// #define BMP_MOSI A5 

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);

Timer timer(5000, checkEnvironment);

// Update to reflect current pressure at sea level
float seaLevelhPa = 1035.7;

void setup() {

  Particle.publish("DEBUG", "starting...");

  if (bmp.begin()) {
    Particle.publish("DEBUG", "starting the environment timer...");
    timer.start();
  }
  else {
    Particle.publish("WARN", "Could not find a valid BMP280 sensor, check wiring!");
  }

  Particle.publish("DEBUG", "started!");
}

void loop() {

}

void checkEnvironment() {

  Particle.publish("DEBUG", "checking the environment...");
  Particle.publish("environment/temperature", String(cToF(bmp.readTemperature())));
  Particle.publish("environment/pressure", String(pToHg(bmp.readPressure())));
  Particle.publish("environment/altitude", String(bmp.readAltitude(seaLevelhPa)));
}

float cToF(float c) {
  return c * 9/5 + 32;
}

float pToHg(float p) {
  return p/3389.39;
}
