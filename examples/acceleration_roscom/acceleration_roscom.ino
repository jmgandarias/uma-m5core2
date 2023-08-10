/*
Example script to read from the Accelerometer
and publish it in a ROS topic with the M5core2

Developer: Juan M. Gandarias (juan.gandarias@iit.it)
Data: 04/07/2022
*/
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
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

// IMU variables
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

const float G = 9.80665;

// Ros Node Handle
ros::NodeHandle_<WiFiHardware> nh;

// Publishers
std_msgs::Float32MultiArray accel_data_msg;
ros::Publisher accel_data_msg_pub("accel_data", &accel_data_msg);

void setup()
{
  Serial.begin(115200);

  M5.begin();

  // Scan Wifi
  String ssid_list = comm_handler.scanWifi();
  M5.Lcd.println(ssid_list);
  comm_handler.setupWiFi();
  delay(500);

  M5.IMU.Init(); // Init IMU sensor.

  M5.Lcd.fillScreen(BLACK);          // Set the screen background color to black.
  M5.Lcd.setTextColor(WHITE, BLACK); // Sets the foreground color and background color of the displayed text.
  M5.Lcd.setTextSize(2);             // Set the font size.

  nh.initNode();
  nh.advertise(accel_data_msg_pub);
  accel_data_msg.data = (float *)malloc(sizeof(float) * 3);
  accel_data_msg.data_length = 3;
}

void loop()
{
  publishIMUData();

  nh.spinOnce();
  M5.update();
  delay(10);
}

void publishIMUData()
{
  // Get IMU Data
  M5.IMU.getAccelData(&accX, &accY, &accZ); // Stores the triaxial accelerometer.

  // Publish ACC data
  accel_data_msg.data[0] = accX * G;
  accel_data_msg.data[1] = accY * G;
  accel_data_msg.data[2] = accZ * G;
  accel_data_msg_pub.publish(&accel_data_msg);

  // //Print acc data
  M5.Lcd.setCursor(0, 70);
  M5.Lcd.print("accX,   accY,  accZ");
  M5.Lcd.setCursor(0, 92);
  M5.Lcd.printf("%5.2f  %5.2f  %5.2f G", accX, accY, accZ);
}