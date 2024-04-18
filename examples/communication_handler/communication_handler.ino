/*
Example script to test the communication handler with ROS and the M5core2

Developer:  Juan M. Gandarias 
            https://jmgandarias.com 
            jmgandarias@uma.es

Former developers:  Mattia Leonori, 
                    Pietro Balatti
                    {mattia.leonori, pietro.balatti}@iit.it

Data: 18/04/2024
*/

#include <M5Core2.h>
#include <ros.h>
#include <SPI.h>
#include <CommunicationHandler.h>

// Auxiliar functions
void waitingScreen();

// WiFi Network
char ssidDesired[] = "your_network_name";      // wifi network name
char password[] = "your_network_password";         // wifi network password
IPAddress client_ip(192, 168, 0, 40); // M5 IP
IPAddress server_ip(192, 168, 0, 13); // Roscore IP

HRII::CommunicationHandler comm_handler(ssidDesired, password);

IPAddress UMA::CommunicationHandler::client_ip_ = client_ip;
IPAddress UMA::CommunicationHandler::server_ip_ = server_ip;

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