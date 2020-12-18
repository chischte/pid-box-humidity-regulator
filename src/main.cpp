/*
 * *****************************************************************************
 * THERMO-HYGROMETER
 * *****************************************************************************
 * Reads temperature and humidity values from a sensor
 * Displays the values on an LCD Display
 * *****************************************************************************
 * Michael Wettstein
 * December 2020, Zürich
 * *****************************************************************************
 * SENSOR:  DHT22
 * DISPLAY: 1602 LCD Display Module
 * *****************************************************************************
 */

// INCLUDES --------------------------------------------------------------------
#include <Arduino.h>
#include <DHT.h>
#include <Insomnia.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>


LiquidCrystal_I2C lcd(0x3F, 16, 2);

#define DHTPIN 2      // DHT sensor pin
#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  lcd.init();
  lcd.backlight();
  dht.begin();
}

void loop() {

  // Read sensor values:
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  humidity = 77.665;
  temperature = 18.776;

  lcd.setCursor(7, 0);
  lcd.print(temperature, 2);
  lcd.print(" ");
  lcd.print((char)0b11011111); // = "°"
  lcd.print("C");
  lcd.setCursor(7, 1);
  lcd.print(humidity, 2);
  lcd.print(" ");
  lcd.print("%rF");

  delay(2000); // Delay 2 sec.
}
