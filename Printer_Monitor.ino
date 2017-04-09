// Printer Monitor
// This is the code for my hobbled together printer enclosure monitor
// Use as you see fit


// Depends on the following extra Arduino libraries:
// - Adafruit Unified Sensor Library: https://github.com/adafruit/Adafruit_Sensor
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - NewLiquidCrystal: https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/ (I used 1.3.4)

// Hardware:
// AM2302 Temp/Humity Sensor - Can be any DHT base sensor
// Relays (I use a SeedStudio relay shield that I had to repair)
// 16x2 LCD with LCM1602 IIC V1 backpack


#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SimpleTimer.h>
#include "custom_chars.h"
#include <EEPROM.h>

#define DHTPIN            12         // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5
#define fan_relay 2
#define dehumidifier_relay 3

DHT_Unified dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

SimpleTimer timer;

float data_temperature;
float data_humidity;
float humidity_setpoint;
boolean dehumidifier_running;
boolean fan_running;
int address = 0;
int lcd_key;

ISR (PCINT0_vect) // handle pin change interrupt for A0 here
{
  lcd_key = read_lcd_buttons();
}
ISR (PCINT2_vect) // handle pin change interrupt for A0 here
{
  lcd_key = read_lcd_buttons();
}

void setup() {
  Serial.begin(57600);

  humidity_setpoint = EEPROM.read(address);

  pinMode(fan_relay, OUTPUT);
  pinMode(dehumidifier_relay, OUTPUT);

  dht.begin();
  lcd.begin(16, 2);
  lcd.clear();
  lcd.createChar(0, temperature_full);
  lcd.createChar(1, temperature_1);
  lcd.createChar(2, temperature_2);
  lcd.createChar(3, temperature_empty);
  lcd.createChar(4, humidity_full);
  lcd.createChar(5, humidity_1);
  lcd.createChar(6, humidity_2);
  lcd.createChar(7, humidity_empty);
  timer.setInterval(5000, sensor_get_data);

  pciSetup(4);
  pciSetup(5);
  pciSetup(6);
  pciSetup(7);
  pciSetup(8);
}

void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

int read_lcd_buttons()
{
  if (digitalRead(4) == LOW)  return btnRIGHT;
  if (digitalRead(5) == LOW)  return btnDOWN;
  if (digitalRead(6) == LOW)  return btnLEFT;
  if (digitalRead(7) == LOW)  return btnSELECT;
  if (digitalRead(8) == LOW)  return btnUP;

  return btnNONE;
}

float sensor_read_temperature() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    return event.temperature;
  }
}

float sensor_read_humidity() {
  sensors_event_t event;
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    return event.relative_humidity;
  }
}
void sensor_get_data() {
  sensors_event_t event;
  data_temperature = sensor_read_temperature();
  data_humidity = sensor_read_humidity();
}

void populate_display() {
  lcd.setCursor(1, 0);
  for (byte i = 0; i < 4; i++) {
    if (fan_running) {
      lcd.write(byte(i));
    }
    else {
      lcd.write(byte(1));
    }
    lcd.setCursor(2, 0);
    lcd.print(data_temperature);
    lcd.print("  ");
    if (dehumidifier_running) {
      byte j = i + 4;
      lcd.write(byte(j));
    }
    else {
      lcd.write(byte(7));
    }
    lcd.setCursor(10, 0);
    lcd.print(data_humidity);
    lcd.setCursor(0, 1);
    lcd_key = read_lcd_buttons();

    switch (lcd_key)
    {
      case btnUP:
        {
          lcd.setCursor(0, 1);
          humidity_setpoint = humidity_setpoint + 1.0;
          EEPROM.update(address, humidity_setpoint);
          lcd.print(humidity_setpoint);
          break;
        }
      case btnDOWN:
        {
          lcd.setCursor(0, 1);
          humidity_setpoint = humidity_setpoint - 1.0;
          EEPROM.update(address, humidity_setpoint);
          lcd.print(humidity_setpoint);
          break;
        }
      case btnSELECT:
        {
          lcd.setCursor(0, 1);
          lcd.print(humidity_setpoint);
          break;
        }
      case btnNONE:
        {
          lcd.setCursor(0, 1);
          lcd.print("                ");
          break;
        }
    }
    lcd.setCursor(1, 0);
    delay(250);
  }
}

void dehumidify() {
  if (data_humidity > humidity_setpoint && lcd_key == btnNONE) {
    dehumidifier_running = true;
    digitalWrite(dehumidifier_relay, HIGH);
  }
  else {
    digitalWrite(dehumidifier_relay, LOW);
    dehumidifier_running = false;
  }
}

void exhaust() {
  fan_running = false;
}

void loop() {
  timer.run();
  if (data_temperature == 0 or data_humidity == 0) {
    sensor_get_data();
    populate_display();
  }
  else {
    dehumidify();
    exhaust();
    populate_display();
  }
}
