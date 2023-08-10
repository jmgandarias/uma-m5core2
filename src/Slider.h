#ifndef __SLIDER_H__
#define __SLIDER_H__

#include <M5Core2.h>
#include <ros.h>
#include <std_msgs/Float32.h>

#define LCD_X_LENGTH 320
#define LCD_Y_LENGTH 240
#define MIN_LCD_VOLTAGE 2500
#define MAX_LCD_VOLTAGE 3300

class Slider
{
public:
  Slider(int16_t x_cord, int16_t y_cord, int16_t length, float initial_value,
         bool orientation, int8_t radius, int8_t tolerance, uint32_t color, uint32_t background_color)
  {
    xCord_ = x_cord;
    yCord_ = y_cord;
    length_ = length;
    initialValue_ = initial_value;
    orientation_ = orientation;
    radius_ = radius;
    tolerance_ = tolerance;
    color_ = color;
    backgroundColor_ = background_color;
  }

  void printSlider()
  {
    // Draw the line
    M5.Lcd.drawFastHLine(xCord_, yCord_, length_, color_);
    // Draw the circle
    M5.Lcd.fillCircle(xCord_ + initialValue_ * length_, yCord_, radius_, color_);
  }

  float updateSlider()
  {
    Zone zone(xCord_ - tolerance_, yCord_ - tolerance_, xCord_ + tolerance_ * 2 + length_, yCord_ + tolerance_ * 2);
    if (M5.Touch.ispressed())
    {
      TouchPoint_t pos = M5.Touch.getPressPoint();
      if (pos.in(zone))
      { // Check if the touched point is in the zone around the slider
        if (pos.x < xCord_)
        {
          sliderValue_ = 0;
        } // If the touched point is lower than the x_cord of the slider, then the value is 0
        else if (pos.x > (xCord_ + length_))
        {
          sliderValue_ = 1;
        } // If the touched point is higher than the x_cord of the slider, then the value is 1
        else
        {
          sliderValue_ = float((pos.x - xCord_)) / float((length_ + xCord_));
        } // Otherwise, it's a value between 0 and 1
        Serial.println(sliderValue_);
        M5.Lcd.fillCircle(xCord_ + initialValue_ * length_, yCord_, radius_, backgroundColor_); // Delete the previous circle
        initialValue_ = sliderValue_;                                                           // Re-set the initialValue_
        printSlider();                                                                          // Draw the new slider
      }
    }
    return sliderValue_; // Return the current value of the slider
  }

  int16_t xCord_ = 0, yCord_ = 0, length_ = 0;
  float initialValue_ = 0.5, sliderValue_ = 0.5;
  bool orientation_ = 0;
  int8_t radius_ = 10, tolerance_ = 5;
  uint32_t color_ = WHITE, backgroundColor_ = BLACK;


}; // class Slider

#endif // __SLIDER_H__