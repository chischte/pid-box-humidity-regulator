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
 */

// ENUM FOR OPERATION MODES ----------------------------------------------------
enum Operation_mode {
  standard = 0,
  set_temperature = 1,
  set_humidty = 2,
  jump_back_to_standard = 3
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
Encoder myEnc(2, 3);
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

void update_set_humidity_display() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("humidity");
}

void update_set_temperature_display() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("temperature");
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
  pinMode(ENCODER_5V_PIN, OUTPUT);
  // pinMode(ENCODER_PUSH, INPUT);
  digitalWrite(ENCODER_5V_PIN, HIGH);
  Serial.begin(9600);
  operation_mode = standard;
}
// LOOP ************************************************************************

long oldPosition = -999;
int current_mode = 0;

void loop() {

  if (encoder_button.switched_low()) {
    Serial.println("Button Pushed");
    current_mode++;
    if (current_mode >= jump_back_to_standard) {
      current_mode = 0;
    }
    Serial.println(current_mode);
  }

  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    // Serial.println(newPosition);
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
    // Update display:
    if (print_delay.delay_time_is_up(2000)) {
      update_standard_display(humidity, temperature, humidity_difference);
    }
    break;

  case set_temperature:
    if (print_delay.delay_time_is_up(500)) {
      update_set_temperature_display();
    }
    break;

  case set_humidty:
    if (print_delay.delay_time_is_up(500)) {
      update_set_humidity_display();
    }
    break;

  case jump_back_to_standard:
    operation_mode = standard;
    break;

  default:
    operation_mode = standard;
    break;
  }
}
