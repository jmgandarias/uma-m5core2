/*
Example script to set multiple vibrations levels in the M5core2

Developer:  Juan M. Gandarias 
            https://jmgandarias.com 
            jmgandarias@uma.es

Data: 18/04/2024
*/

#include <M5Core2.h>

// Variables
int min_motor_voltage = 1800; // minimum motor voltage (mV)
int max_motor_voltage = 3300; // maximum motor voltage (mV)
int vibration_values = 16;    // The register to send the desired voltage of the motor has 4 bits (2^4=16 vibration values).
float motor_voltage_increment = 93.75;
/*
The increment of 93.75 mV comes from this equation:
motor_voltage_increment=(max_motor_voltage-min_motor_voltage)/vibration_values=(3300-1800)/16=93.75
*/
float voltage_commanded = min_motor_voltage;
int vibration_motor_pin = 3;

void setup()
{
  M5.begin(); // Init M5Core2
}

void loop()
{
  for (int i = 1; i <= vibration_values; i = i + 1)
  {
    M5.Axp.SetLDOEnable(vibration_motor_pin, true); // Open the vibration.
    voltage_commanded = 1800 + motor_voltage_increment * (i - 1);
    M5.Axp.SetLDOVoltage(vibration_motor_pin, voltage_commanded); // Set motor voltage
    M5.Lcd.println(voltage_commanded);
    delay(500);
  }
  M5.Axp.SetLDOEnable(vibration_motor_pin, false); // Close the vibration.
  M5.Lcd.println(0);
  delay(500);
}
