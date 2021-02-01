/*
 * *****************************************************************************
 * PID BOX HUMIDITY CONTROLLER
 * *****************************************************************************
 * Reads temperature and humidity values from a sensor
 * Displays the values on an LCD display
 * Regulates the humidity inside a box using a PID regulation algorithm.
 * The humidity is regulated by changing the air temperature.
 * Heating up the are lets he humidity drop.
 * *****************************************************************************
 * Michael Wettstein
 * February 2020, Zürich
 * *****************************************************************************
 * SENSOR:  AM2315
 * DISPLAY: 1602 LCD Display Module
 * ROTARY ENCODER
 * L298N MOTOR DRIVER MODULE (to regulate the heating element)
 * *****************************************************************************
 */

// GLOBAL VARIABLES ------------------------------------------------------------
bool display_refreshed = false;
float humidity;
float humidity_setpoint;
float delta_rH_in_30_seconds;
float temperature;
float temperature_setpoint;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
float pid = 0;
float temperature_limit = 33.0; // [°C] to avoid overheating
static long encoder_prev_position = 0;

// ENUM FOR OPERATION MODES ----------------------------------------------------
enum Operation_mode
{
  standard = 0,
  set_humidty = 1,
  number_of_modes = 2
};
Operation_mode operation_mode;

// INCLUDES --------------------------------------------------------------------
#include <Arduino.h>
#include <DHT.h>
#include <Debounce.h>
#include <EEPROM_Counter.h>
#include <Encoder.h>
#include <Insomnia.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_AM2315.h>
#include <Wire.h>

// NO SLEEP DELAYS -------------------------------------------------------------
Insomnia read_delay;
Insomnia log_delay;
Insomnia serial_print_delay;

// EEPROM STORAGE --------------------------------------------------------------
EEPROM_Counter eeprom_storage;

int eepromMinAddress = 0;
int eepromMaxAddress = 1000; // EEPROM size Arduino Nano/Uno: 1024 bytes

enum counter
{
  eeprom_temp,
  eeprom_humidity,
  endOfEnum
};
int numberOfValues = endOfEnum;

// RELAYS ----------------------------------------------------------------------
const int HEATING_RELAY_PIN = 8;

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

// SENSOR AM2315 ---------------------------------------------------------------
// SDA @ PIN A4 // SCL @ PIN A5
Adafruit_AM2315 am2315;

// L298N MOTOR DRIVER MODULE ---------------------------------------------------
// The driver module is used to control th power of the air heating element
// Heating the box lets the humidity drop
// Connect IN2 to GND
// Use a PWM Signal on IN1 to control the heating power.
const int HEATING_CONTROL_PWM_PIN = 6;

// FUNCTIONS *******************************************************************
float limit(float value, float min, float max)
{
  if (value < min)
  {
    value = min;
  }
  if (value > max)
  {
    value = max;
  }
  return value;
}

void monitor_changed_values_standard_display()
{
  static float prev_humidity = humidity;
  static float prev_temperature = temperature;
  static float prev_humidity_difference = delta_rH_in_30_seconds;

  if (humidity != prev_humidity)
  {
    display_refreshed = false;
    prev_humidity = humidity;
  }
  if (temperature != prev_temperature)
  {
    display_refreshed = false;
    prev_temperature = temperature;
  }
  if (delta_rH_in_30_seconds != prev_humidity_difference)
  {
    display_refreshed = false;
    prev_humidity_difference = delta_rH_in_30_seconds;
  }
}

void update_standard_display()
{
  monitor_changed_values_standard_display();

  if (!display_refreshed)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(temperature, 1);
    lcd.print((char)0b11011111); // = "°"
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print(humidity, 1);
    lcd.print("%rF =>");
    lcd.print(delta_rH_in_30_seconds, 1);
    display_refreshed = true;
  }
}

void monitor_changed_values_humidity_display()
{
  long current_position = encoder.read();
  static int encoder_klicks = 4;

  if (current_position - encoder_prev_position >= encoder_klicks)
  {
    display_refreshed = false;
    encoder_prev_position = current_position;
    humidity_setpoint++;
    eeprom_storage.set_value(eeprom_humidity, long(humidity_setpoint));
    humidity_setpoint = limit(humidity_setpoint, 0, 99);
  }

  if (encoder_prev_position - current_position >= encoder_klicks)
  {
    display_refreshed = false;
    encoder_prev_position = current_position;
    humidity_setpoint--;
    eeprom_storage.set_value(eeprom_humidity, long(humidity_setpoint));
    humidity_setpoint = limit(humidity_setpoint, 0, 99);
  }
}

void update_set_humidity_display()
{
  monitor_changed_values_humidity_display();
  if (!display_refreshed)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("humidity set:");
    lcd.setCursor(0, 1);
    lcd.print(humidity_setpoint, 0);
    lcd.print("%rF");
    display_refreshed = true;
  }
}

void calculate_delta_rH_in_30_seconds()
{

  // Log configutation:
  const int number_of_seconds = 30;
  const int actualization_rate = 2; // [s]
  const int number_of_values = number_of_seconds / actualization_rate;
  const unsigned long delayTime = actualization_rate * 1000;

  static float humidity_log[number_of_values];

  if (log_delay.delay_time_is_up(delayTime))
  {
    // Move values one index back:
    for (int i = 0; i < (number_of_values - 1); i++)
    {
      humidity_log[i] = humidity_log[i + 1];
    }
    // Update latest value:
    humidity_log[number_of_values - 1] = humidity;
    if (humidity_log[0] != 0)
    {
      delta_rH_in_30_seconds = humidity - humidity_log[0];
    }
    else
    {
      delta_rH_in_30_seconds = 0;
    }
  }
}

void read_sensor_values()
{
  if (read_delay.delay_time_is_up(1000))
  {
    am2315.readTemperatureAndHumidity(&temperature, &humidity);
  }
}

int monitor_encoder_button(int current_mode)
{
  if (encoder_button.switched_low())
  {
    display_refreshed = false;
    current_mode++;
  }
  if (current_mode >= number_of_modes)
  {
    current_mode = 0;
  }
  return current_mode;
}

void display_current_mode(int current_mode)
{
  switch (current_mode)
  {
  case standard:
    update_standard_display();
    break;

  case set_humidty:
    update_set_humidity_display();
    break;
  }
}

void calculate_p()
{
  // p should be at 100% if humidity is 2%rH above setpoint
  float delta_humidity = humidity - humidity_setpoint;
  float humidity_diference_for_full_reaction = 2; //[%rH]
  pid_p = 100 * delta_humidity / humidity_diference_for_full_reaction;
  pid_p = limit(pid_p, -100, 100);
}

void calculate_i()
{
  // i should go 4% up every minute humidity is 1% above setpoint
  float i_factor = 4; // [%]
  static unsigned long previous_time = micros();
  unsigned long new_time = micros();
  unsigned long delta_t = new_time - previous_time;
  previous_time = new_time;
  float delta_humidity = humidity - humidity_setpoint;
  float micros_per_minute = 1000.f * 1000.f * 60.f;
  pid_i += i_factor * delta_humidity * delta_t / micros_per_minute;
  pid_i = limit(pid_i, 0, 100);
}

void calculate_d()
{
  // d should be at 100% if humidity rises 0.5% in 30 seconds
  float rH_difference_for_full_reaction = 0.5; //[%rh/5minutes]
  pid_d = 100 * delta_rH_in_30_seconds / rH_difference_for_full_reaction;
  pid_d = limit(pid_d, -100, 100);
}

void calculate_pid_air_heater()
{
  calculate_p();

  calculate_i();

  calculate_d();

  pid = pid_p + pid_i + pid_d;
  pid = limit(pid, 0, 100);
}

void switch_air_heater()
{
  if (temperature < temperature_limit)
  {
    int heating_power = map(pid, 0, 100, 0, 255);
    analogWrite(HEATING_CONTROL_PWM_PIN, heating_power); // 0-255
  }
  else
  {
    analogWrite(HEATING_CONTROL_PWM_PIN, 0);
  }
}

void print_serial_plot_chart()
{
  if (serial_print_delay.delay_time_is_up(7000))
  {
    Serial.print(-100);
    Serial.print(",");
    Serial.print(+100);
    Serial.print(",");
    Serial.print(0);
    Serial.print(",");
    Serial.print(pid_p);
    Serial.print(",");
    Serial.print(pid_i);
    Serial.print(",");
    Serial.print(pid_d);
    Serial.print(",");
    Serial.print(pid);
    Serial.print(",");
    Serial.print(temperature);
    Serial.print(",");
    Serial.print(humidity);
    Serial.print(",");
    Serial.print(humidity_setpoint);
    Serial.println();
  }
}

// SETUP ***********************************************************************
void setup()
{
  eeprom_storage.setup(eepromMinAddress, eepromMaxAddress, numberOfValues);
  temperature_setpoint = float(eeprom_storage.get_value(eeprom_temp));
  humidity_setpoint = float(eeprom_storage.get_value(eeprom_humidity));
  lcd.init();
  lcd.backlight();
  pinMode(HEATING_RELAY_PIN, OUTPUT);
  pinMode(ENCODER_5V_PIN, OUTPUT);
  digitalWrite(ENCODER_5V_PIN, HIGH);
  operation_mode = standard;
  Serial.begin(9600);
  Serial.println("EXIT SETUP");
}

// LOOP ************************************************************************
void loop()
{
  read_sensor_values();

  // Toggle display modes:
  static int current_mode = 0;
  current_mode = monitor_encoder_button(current_mode);
  display_current_mode(current_mode);

  // Log humidity and get humidity difference:
  calculate_delta_rH_in_30_seconds();

  calculate_pid_air_heater();

  switch_air_heater();

  print_serial_plot_chart();
}
