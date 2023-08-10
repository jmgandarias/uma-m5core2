/*
Example script to create an horizontal slide bar using the Slider.h library inside Hrii_M5Core2 library

Developer: Juan M. Gandarias (juan.gandarias@iit.it)
Data: 04/09/2022
*/

#include <Slider.h>
#include <M5Core2.h>

float slider_value;

Slider slider(40, LCD_Y_LENGTH / 2, LCD_X_LENGTH - 40 * 2, 0.5, 0, 10, 5, WHITE, BLACK);

void setup()
{
  M5.begin();
  M5.Lcd.clear();
  slider.printSlider();
}

void loop()
{
  M5.update();
  slider_value = slider.updateSlider();
  Serial.println(slider_value);
  M5.Axp.SetLcdVoltage(MIN_LCD_VOLTAGE + slider_value * (MAX_LCD_VOLTAGE - MIN_LCD_VOLTAGE));
  delay(1000/25); //runs at 25Hz (The display cannot handle more)
}
