/*
 * *****************************************************************************
 * THERMO-HYGROMETER
 * *****************************************************************************
 * Reads temperature and humidity values from a sensor
 * Calculates the value by which humidity has changed in 5 minutes
 * Displays the values on an LCD display
 * *****************************************************************************
 * Michael Wettstein
 * December 2020, Zürich
 * *****************************************************************************
 * SENSOR:  DHT22
 * DISPLAY: 1602 LCD Display Module
 * *****************************************************************************
 * TODO:
 * IMPLEMENT set of target values
 *
 * *****************************************************************************
 */

// GLOBAL VARIABLE TO MANAGE DISPLAY REFRESH -----------------------------------
bool display_refreshed = false;

// ENUM FOR OPERATION MODES ----------------------------------------------------
enum Operation_mode {
  standard = 0,
  set_temperature = 1,
  set_humidty = 2,
};
Operation_mode operation_mode;

// INCLUDES --------------------------------------------------------------------
#include <Arduino.h>
#include <DHT.h>
#include <Debounce.h>
#include <Encoder.h>
#include <Insomnia.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// NO SLEEP DELAYS -------------------------------------------------------------
Insomnia print_delay;
Insomnia read_delay;
Insomnia log_delay;

// ROTARY ENCODER --------------------------------------------------------------
Encoder encoder(2, 3);
const int ENCODER_5V_PIN = 4;
const int ENCODER_S1 = 2;
const int ENCODER_S2 = 3;
const int ENCODER_PUSH = A0;

// DEBOUNCE BUTTON -------------------------------------------------------------
Debounce encoder_button(ENCODER_PUSH);

// DISPLAY I2C------------------------------------------------------------------
// SDA @ PIN A4
// SCL @ PIN A5
LiquidCrystal_I2C lcd(0x3F, 16, 2);

// SENSOR AM2315
// ---------------------------------------------------------------------- SDA @
// PIN A4 SCL @ PIN A5
#define DHTPIN                                                                 \
  11 //......wrong PIN, AM2315 not implemented yet !!!  //......wrong PIN,
     // AM2315 not implemented yet !!!  //......wrong PIN, AM2315 not
     // implemented yet !!!
#define DHTTYPE                                                                \
  DHT22 // DHT 22  (AM2302), AM2321 //......wrong PIN, AM2315 not implemented
        // yet !!!  //......wrong PIN, AM2315 not implemented yet !!!
DHT dht(DHTPIN, DHTTYPE);

// FUNCTIONS *******************************************************************

void update_standard_display(float humidity, float temperature,
                             float humidity_difference) {

  static float prev_humidity = humidity;
  static float prev_temperature = temperature;
  static float prev_humidity_difference = humidity_difference;

  if (humidity != prev_humidity) {
    display_refreshed = false;
    prev_humidity = humidity;
  }
  if (temperature != prev_temperature) {
    display_refreshed = false;
    prev_temperature = temperature;
  }
  if (humidity_difference != prev_humidity_difference) {
    display_refreshed = false;
    prev_humidity_difference = humidity_difference;
  }

  if (!display_refreshed) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(temperature, 1);
    lcd.print((char)0b11011111); // = "°"
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print(humidity, 1);
    lcd.print("%rF=>");
    lcd.print(humidity_difference, 1);
    display_refreshed = true;
  }
}

float limit(float value, float min, float max) {
  if (value < min) {
    value = min;
  }
  if (value > max) {
    value = max;
  }
  return value;
}

float update_set_humidity_display(float humidity_setpoint) {
  static long prev_position = 0;
  long current_position = encoder.read();
  static int encoder_klicks = 4;

  if (current_position - prev_position >= encoder_klicks) {
    display_refreshed = false;
    prev_position = current_position;
    humidity_setpoint++;
    humidity_setpoint = limit(humidity_setpoint, 75, 99);
  }

  if (prev_position - current_position >= encoder_klicks) {
    display_refreshed = false;
    prev_position = current_position;
    humidity_setpoint--;
    humidity_setpoint = limit(humidity_setpoint, 75, 99);
  }

  if (!display_refreshed) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("humidity set:");
    lcd.setCursor(0, 1);
    lcd.print(humidity_setpoint, 0);
    lcd.print("%rF");
    display_refreshed = true;
  }
  return humidity_setpoint;
}

float update_set_temperature_display(float temperature_setpoint) {
  static long prev_position = 0;
  long current_position = encoder.read();
  static int encoder_klicks = 4;

  if (current_position - prev_position >= encoder_klicks) {
    display_refreshed = false;
    prev_position = current_position;
    temperature_setpoint++;
    temperature_setpoint = limit(temperature_setpoint, 18, 35);
  }

  if (prev_position - current_position >= encoder_klicks) {
    display_refreshed = false;
    prev_position = current_position;
    temperature_setpoint--;
    temperature_setpoint = limit(temperature_setpoint, 18, 35);
  }

  if (!display_refreshed) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("temperature set:");
    lcd.setCursor(0, 1);
    lcd.print(temperature_setpoint, 0);
    lcd.print((char)0b11011111); // = "°"
    lcd.print("C");
    display_refreshed = true;

    display_refreshed = true;
  }

  return temperature_setpoint;
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
  }
  return humidity_difference;
}
// SETUP ***********************************************************************

void setup() {
  lcd.init();
  lcd.backlight();
  dht.begin();
  pinMode(ENCODER_5V_PIN, OUTPUT);
  digitalWrite(ENCODER_5V_PIN, HIGH);
  Serial.begin(9600);
  operation_mode = standard;
}
// LOOP ************************************************************************

void loop() {
  static int current_mode = 0;

  static float humidity_setpoint = 0;
  static float temperature_setpoint = 0;

  if (encoder_button.switched_low()) {
    display_refreshed = false;
    current_mode++;
  }

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

  switch (current_mode) {

  case standard:
    update_standard_display(humidity, temperature, humidity_difference);
    break;

  case set_temperature:
    temperature_setpoint = update_set_temperature_display(temperature_setpoint);
    break;

  case set_humidty:
    humidity_setpoint = update_set_humidity_display(humidity_setpoint);
    break;

  default: // jump back to standard
    current_mode = standard;
    break;
  }
}
