Auto Speed Fan controller

HW:
  - Arduino Uno
  - TCA9548A I2C Switch
  - 2 SHTC3 I2C Temp/Humidity Sensors
  - SSD1306 128x64 OLED Display
  - Pot for manual speed input (currently disabled)

Functional Description:
Uses 2 temperature sensors as an input. Depending on the difference between them, will generate a PWM signal to control PWM Fan speed

POT can also be used to control fan speed.

SW: Uses Wire and SHTC3 libraries for temp sensor. Uses SSD1306Ascii library instead of Adafruit library due to space constraints.

Uses Code borrowed from Matt Clarke (Matchstic) for PWM.
