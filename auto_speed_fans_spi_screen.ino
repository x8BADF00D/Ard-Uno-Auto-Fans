/*
 * Pot diabled
 * TCA used to mux 2 i2c TS
 * added SPI Screen
 * 
 * 
 * Arduino PWM fan control w/ potentiometer input
 * ==============================================
 * 
 * This sketch is derived from:
 * https://create.arduino.cc/projecthub/tylerpeppy/25-khz-4-pin-pwm-fan-control-with-arduino-uno-3005a1
 * 
 * It allows you to control a PWM 4-pin PC fan via your Arduino Uno.
 * 
 * The general idea is to make use of the 10 PWM phase-correct mode
 * on timer 1 OCR1A, available via pin 9. This allows for control
 * over the speed of the fan via setting a value in the range
 * 0 - 100.
 * 
 * Furthermore, input from a potentiometer hooked up to analog pin
 * 2 is used to drive the fan speed. If you do not have a "pot", you
 * should hardcode your expected speed by setting FORCE_SPEED to >= 0.
 * 
 * Debugging notes:
 * 
 * Q1. Fan doesn't spin up?
 * A1. Make sure you are using a 12V supply to your fan, and make sure 
 *     that the PWM_FREQ_HZ variable matches the specification for your
 *     fan model.
 *     
 * Q2. Compiler issues complaining about OCR1A etc.
 * A2. This sketch is specific to the Arduino Uno. You should find the
 *     corresponding names for your board.
 * 
 * Created by Matt Clarke (Matchstic), 2021.
 */
#include <SPI.h>
#include "Wire.h"
#include "Adafruit_SHTC3.h"
#include "SSD1306Ascii.h"
#include "SSD1306AsciiSpi.h"

// SPI Pins for OLED
#define CS_PIN 5
#define RST_PIN 7
#define DC_PIN 6

#define TCAADDR 0x77

SSD1306AsciiSpi oled;

Adafruit_SHTC3 ts0 = Adafruit_SHTC3();
Adafruit_SHTC3 ts1 = Adafruit_SHTC3();

float t0f, t1f;
int duty = 0;

void tcaselect(uint8_t ch) {
  if (ch > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

const byte POT_PIN = 2;
const byte OC1A_PIN = 9;

// Adjust this value to set the frequency to your fan's needs
const word PWM_FREQ_HZ = 25000;
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ);

const int FORCE_SPEED = -1;

void setup() {
  Serial.begin(9600);
  //while (!Serial);
  delay(100);

  Serial.println("Serial set up");

  Wire.begin();

  oled.begin(&Adafruit128x64, CS_PIN, DC_PIN, RST_PIN);
  oled.setFont(System5x7);
  oled.clear();
  oled.set2X();
  oled.print("Hello world!");

  Serial.println("\nTCA ready");

  tcaselect(0);
  Serial.println("tcasel 0");

  delay(100);

  if (!ts0.begin()) {
    Serial.println("Couldn't find ts0");
    while (1) delay(1);
  }
  Serial.println("Found ts0 sensor");

  tcaselect(1);
  Serial.println("tcasel 1");

  delay(100);

  if (!ts1.begin()) {
    Serial.println("Couldn't find ts1");
    while (1) delay(1);
  }
  Serial.println("Found ts1 sensor");

  pinMode(OC1A_PIN, OUTPUT);
  //Serial.begin(9600); // open the serial port at 9600 bps:

  // Clear Timer1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
  oled.clear();
}

void loop() {
  //const int rotation = analogRead(POT_PIN);
  //Serial.println(rotation);


  tcaselect(0);
  sensors_event_t humidity, temp;
  ts0.getEvent(&humidity, &temp);
  t0f = (temp.temperature * 1.8) + 32;
  Serial.print("TS0:");
  Serial.print(t0f);
  Serial.print("\t");

  tcaselect(1);
  ts1.getEvent(&humidity, &temp);
  t1f = (temp.temperature * 1.8) + 32;
  Serial.print("TS1:");
  Serial.print(t1f);
  Serial.print("\t");

  int delta = t1f - t0f;
  if (delta > 3) {
    duty = 0.5 * delta * delta;
    if (duty > 100) duty = 100;
  } else duty = 0;

  Serial.print("Delta:");
  Serial.print(delta);
  Serial.print("\t");

  Serial.print("Cycle:");
  Serial.println(duty);
  // Serial.print("\t");

  setPwmDuty(FORCE_SPEED > -1 ? FORCE_SPEED : duty);

  //using oled.clear() causes screen to strobe due to low speed. oled.setRow() overwrites old data, avoiding strobe
  oled.setRow(0);
  oled.print("TS0: ");
  oled.println(t0f);
  oled.print("TS1: ");
  oled.println(t1f);
  oled.print("Delta: ");
  oled.println(delta);
  oled.print("DC:  ");
  oled.print(duty);
  oled.println(" %");

  delay(10);
}

/**
 * Sets the speed of the fan
 * 
 * @param duty A value between 0 and 100
 */
void setPwmDuty(byte duty) {
  OCR1A = (word)(duty * TCNT1_TOP) / 100;
}
