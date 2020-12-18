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

// DELAYS ----------------------------------------------------------------------
Insomnia print_delay;
Insomnia read_delay;
Insomnia log_delay;

// DISPLAY ---------------------------------------------------------------------
LiquidCrystal_I2C lcd(0x3F, 16, 2);

// SENSOR ----------------------------------------------------------------------
#define DHTPIN 2      // DHT sensor pin
#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

// FUNCTIONS *******************************************************************

void update_display(float humidity, float temperature,
                    float humidity_difference) {
  lcd.setCursor(0, 0);
  lcd.print(temperature, 2);
  lcd.print((char)0b11011111); // = "°"
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print(humidity, 2);
  lcd.print("%rF=>");
  lcd.print(humidity_difference, 1);
}

// -----------------------------------------------------------------------------

float get_humidity_difference(float current_humidity) {
  const int number_of_minute_logs = 5;
  static float humidity_log[number_of_minute_logs];
  float humidity_difference;

  if (log_delay.delay_time_is_up(60000)) {
    // Move values one index back
    for (int i = 0; i < (number_of_minute_logs - 2); i++) {
      humidity_log[i] = humidity_log[i + 1];
    }
    humidity_log[number_of_minute_logs] = current_humidity;
    humidity_difference = current_humidity - humidity_log[0];
  }
  return humidity_difference;
}
// SETUP ***********************************************************************
void setup() {
  lcd.init();
  lcd.backlight();
  dht.begin();
}
// LOOP ************************************************************************
void loop() {

  float humidity;
  float temperature;
  float humidity_difference;

  // Read sensor values:
  if (read_delay.delay_time_is_up(1000)) {
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
  }

  // Log humidity and get humidity difference:
  humidity_difference = get_humidity_difference(humidity);

  humidity = 77.665;
  temperature = 18.776;
  humidity_difference = -3.2;

  // Update Display:
  if (print_delay.delay_time_is_up(2000)) {
    update_display(humidity, temperature, humidity_difference);
  }
}
