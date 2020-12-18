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
const int SENSOR_5V_PIN = 3;
#define DHTPIN 2      // DHT sensor pin
#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

// FUNCTIONS *******************************************************************

void update_display(float humidity, float temperature,
                    float humidity_difference) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(temperature, 1);
  lcd.print((char)0b11011111); // = "°"
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print(humidity, 1);
  lcd.print("%rF=>");
  lcd.print(humidity_difference, 1);
}

// -----------------------------------------------------------------------------

float get_humidity_difference(float current_humidity) {

  float humidity_difference;

  // Log configutation:
  const int number_of_minutes = 5;
  const int actualization_rate = 10; // [s]
  const int number_of_values = number_of_minutes * 60 / actualization_rate;
  const unsigned long delayTime = actualization_rate * 1000;

  static float humidity_log[number_of_values];

  if (log_delay.delay_time_is_up(delayTime)) {

    // Move values one index back:
    for (int i = 0; i < (number_of_values - 1); i++) {
      humidity_log[i] = humidity_log[i + 1];
    }

    // Update latest value:
    humidity_log[number_of_values - 1] = current_humidity;
    humidity_difference = current_humidity - humidity_log[0];

    // Print for debugging:
    for (int j = 0; j < number_of_values; j++) {
      Serial.print(humidity_log[j]);
      Serial.print("|");
    }
    Serial.println(" ");
  }
  return humidity_difference;
}
// SETUP ***********************************************************************

void setup() {
  lcd.init();
  lcd.backlight();
  dht.begin();
  pinMode(SENSOR_5V_PIN, OUTPUT);
  digitalWrite(SENSOR_5V_PIN, HIGH);
  Serial.begin(9600);
}
// LOOP ************************************************************************

void loop() {

  static float humidity;
  static float temperature;
  static float humidity_difference;

  // Read sensor values:
  if (read_delay.delay_time_is_up(1000)) {
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
  }

  // Log humidity and get humidity difference:
  humidity_difference = get_humidity_difference(humidity);

  // Update Display:
  if (print_delay.delay_time_is_up(2000)) {
    update_display(humidity, temperature, humidity_difference);
  }
}
