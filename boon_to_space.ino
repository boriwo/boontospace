/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor. Designed specifically to work with the Adafruit BMP388 Breakout ----> http://www.adafruit.com/products/3966
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required to interface. Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries. BSD license, all text above must be included in any redistribution
 ***************************************************************************/

// Complete project details: https://RandomNerdTutorials.com/arduino-bmp388/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#include <DHT.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

int cnt;

#define DHTPIN 7
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

void setup() {

  cnt = 0;
  
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // Initialize digital pin 14 (A0) as an output.
  pinMode(14, OUTPUT);

  dht.begin();
}

void loop() {

  cnt++;
  
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  
  Serial.print("BMP388 Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("BMP388 Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("BMP388 Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();

  if (bmp.temperature > 28) {
    // Set digital pin 14 (A0) to HIGH.
    digitalWrite(14, LOW);
  } else {
     // Set digital pin 14 (A0) to HIGH.
     digitalWrite(14, HIGH);
  }

  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  /*if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT11 sensor!"));
    return;
  }*/

  Serial.print("DHT11 Temperature = ");
  Serial.println(t);
  Serial.print("DHT11 Humidity = ");
  Serial.println(h);

  Serial.print("Counter: ");
  Serial.println(cnt);
}


