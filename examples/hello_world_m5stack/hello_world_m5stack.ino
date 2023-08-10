#include <M5Core2.h>

/* After M5Core2 is started or reset
the program in the setUp () function will be run, and this part will only be run once.
After M5Core2 is started or reset, the program in the setup() function will be executed, and this part will only be executed once. */
void setup()
{
  M5.begin(); // Init M5Core2. Initialize M5Core2
  /* Power chip connected to gpio21, gpio22, I2C device
    Set battery charging voltage and current
    If used battery, please call this function in your project */
  M5.Lcd.print("Hello World"); // Print text on the screen (string) Print text on the screen (string)
}

/* After the program in setup() runs, it runs the program in loop()
The loop() function is an infinite loop in which the program runs repeatedly
After the program in the setup() function is executed, the program in the loop() function will be executed
The loop() function is an endless loop, in which the program will continue to run repeatedly */
void loop()
{
}