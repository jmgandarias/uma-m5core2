/*
Example script to send a vibration to the M5core2 via ROS
When an Empy msg is published in /toggle_led, the screen turns green and the device vibrates

Developer:  Juan M. Gandarias 
            https://jmgandarias.com 
            jmgandarias@uma.es

Data: 18/04/2024
*/

#include <ros.h>
#include <std_msgs/Empty.h>
#include <SPI.h>
#include <UMAM5Core2.h>

// WiFi Network
char ssidDesired[] = "your_network_name";      // wifi network name
char password[] = "your_network_password";         // wifi network password
IPAddress client_ip(192, 168, 0, 40); // M5 IP
IPAddress server_ip(192, 168, 0, 13); // Roscore IP

UMA::CommunicationHandler comm_handler(ssidDesired, password);

IPAddress UMA::CommunicationHandler::client_ip_ = client_ip;
IPAddress UMA::CommunicationHandler::server_ip_ = server_ip;

// M5 LCD
TFT_eSprite Img = TFT_eSprite(&M5.Lcd);

// Vibration variables
float voltage_commanded = 2000; // It has to be a value between 1800 and 3300 mV
int vibration_motor_pin = 3;
int vibration_time = 400; // 400 ms

// Auxiliar functions
void waitingScreen();

// Ros Node Handle
ros::NodeHandle_<WiFiHardware> nh;

// Message Callback
void MessageCb(const std_msgs::Empty &toggle_msg)
{
  M5.Axp.SetLDOEnable(vibration_motor_pin, true);               // Open the vibration.
  M5.Axp.SetLDOVoltage(vibration_motor_pin, voltage_commanded); // Set motor voltage
  M5.Lcd.println(voltage_commanded);
  M5.Lcd.fillScreen(TFT_GREEN);
  delay(vibration_time);
  M5.Axp.SetLDOEnable(vibration_motor_pin, false); // Close the vibration.

  waitingScreen();
}

// Subscribers
ros::Subscriber<std_msgs::Empty> empty_msg_sub("toggle_led", &MessageCb);

void setup()
{
  Serial.begin(115200);

  M5.begin();

  // Scan Wifi
  String ssid_list = comm_handler.scanWifi();
  M5.Lcd.println(ssid_list);
  comm_handler.setupWiFi();
  delay(500);

  M5.Lcd.fillScreen(TFT_BLUE);
  Img.setColorDepth(8);
  Img.createSprite(320, 240);
  Img.fillSprite(TFT_BLACK);

  nh.initNode();
  nh.subscribe(empty_msg_sub);
  waitingScreen();
}

void loop()
{
  nh.spinOnce();
  delay(10);
  M5.update();
}

void waitingScreen()
{
  M5.Lcd.setTextDatum(MC_DATUM); // Set text alignment to center-aligned
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.drawFastHLine(60, 70, 200, WHITE); // Draw a white horizontal line with a length of 255 at (3,100)
  M5.Lcd.drawString("Waiting", 160, 90, 2);
  M5.Lcd.drawString("for", 160, 120, 2);
  M5.Lcd.drawString("messages", 160, 150, 2);
  M5.Lcd.drawFastHLine(60, 170, 200, WHITE); // Draw a white horizontal line with a length of 255 at (3,100)
}
