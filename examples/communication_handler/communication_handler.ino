/*
Example script to test the communication handler
with ROS and the M5core2

Developer: Juan M. Gandarias, Mattia Leonori, Pietro Balatti
({juan.gandarias, mattia.leonori, pietro.balatti}@iit.it)
Data: 12/10/2022
*/

#include <M5Core2.h>
#include <ros.h>
#include <SPI.h>
#include <CommunicationHandler.h>

// Auxiliar functions
void waitingScreen();

// WiFi Network
char ssidDesired[] = "HRII_LAB";      // wifi network name
char password[] = "hriilabo";         // wifi network password
IPAddress client_ip(192, 168, 0, 40); // M5 IP
IPAddress server_ip(192, 168, 0, 13); // Roscore IP

HRII::CommunicationHandler comm_handler(ssidDesired, password);

IPAddress HRII::CommunicationHandler::client_ip_ = client_ip;
IPAddress HRII::CommunicationHandler::server_ip_ = server_ip;

// Ros Node Handle
ros::NodeHandle_<WiFiHardware> nh;

void setup()
{
  Serial.begin(115200);
  M5.begin();

  // Scan Wifi
  String ssid_list = comm_handler.scanWifi();
  M5.Lcd.println(ssid_list);
  comm_handler.setupWiFi();
  delay(500);

  nh.initNode();
}

void loop()
{
  M5.update();
  delay(10);
  nh.spinOnce();
}