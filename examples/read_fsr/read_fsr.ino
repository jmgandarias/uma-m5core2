/*
Example script to read from the Analog Fore Sensing Resistor (FSR)
and publish it in a ROS topic with the M5core2

-------------------------------
Voltage divider:
Vout = R2/(R1+R2)Vin

Vin = 3.3V
R1 = FSR
R2 = 3.3kOhm
Vout - G35(ADC) port of M5Core2
-------------------------------

Developer: Juan M. Gandarias (juan.gandarias@iit.it)
Data: 04/08/2022
*/
#include <ros.h>
#include <std_msgs/Int16.h>
#include <SPI.h>
#include <HriiM5Core2.h>

// WiFi Network
char ssidDesired[] = "HRII_LAB";      // wifi network name
char password[] = "hriilabo";         // wifi network password
IPAddress client_ip(192, 168, 0, 40); // M5 IP
IPAddress server_ip(192, 168, 0, 13); // Roscore IP

HRII::CommunicationHandler comm_handler(ssidDesired, password);

IPAddress HRII::CommunicationHandler::client_ip_ = client_ip;
IPAddress HRII::CommunicationHandler::server_ip_ = server_ip;

// FSR variables
int force_raw = 0;
int fsr_port = 35;

// Ros Node Handle
ros::NodeHandle_<WiFiHardware> nh;

// Publisher
std_msgs::Int16 force_data_msg;
ros::Publisher force_data_msg_pub("force_data", &force_data_msg);

void setup()
{
  Serial.begin(115200);

  M5.begin();

  // Scan Wifi
  String ssid_list = comm_handler.scanWifi();
  M5.Lcd.println(ssid_list);
  comm_handler.setupWiFi();
  delay(500);

  M5.Lcd.fillScreen(BLACK);          // Set the screen background color to black.
  M5.Lcd.setTextColor(WHITE, BLACK); // Sets the foreground color and background color of the displayed text.
  M5.Lcd.setTextSize(2);             // Set the font size.

  nh.initNode();
  nh.advertise(force_data_msg_pub);

  // setup G35 as input (Vout connected to G35)
  pinMode(fsr_port, INPUT);
}

void loop()
{

  publishFSRData();

  nh.spinOnce();
  M5.update();
  delay(10);
}

void publishFSRData()
{

  force_raw = analogRead(fsr_port);

  // Publish FSR data
  force_data_msg.data = force_raw;
  force_data_msg_pub.publish(&force_data_msg);

  // //Print FSR data
  M5.Lcd.setCursor(0, 70);
  M5.Lcd.print("Force");
  M5.Lcd.setCursor(0, 92);
  M5.Lcd.printf("%d  (raw)", force_raw);
}