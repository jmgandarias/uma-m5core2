/*
Example script to read from the IMU
and publish the data in a ROS topic with the M5core2

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

// WiFi variables
String ssidList;
String ssidName;

// IMU variables
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll = 0.0F;
float yaw = 0.0F;

float temp = 0.0F;

const float G = 9.80665;

// Ros Node Handle
ros::NodeHandle_<WiFiHardware> nh;

// Publishers
std_msgs::Float32MultiArray accel_data_msg, orientation_data_msg;
ros::Publisher orientation_data_msg_pub("orientation_data", &orientation_data_msg);
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

  nh.advertise(orientation_data_msg_pub);
  orientation_data_msg.data = (float *)malloc(sizeof(float) * 3);
  orientation_data_msg.data_length = 3;
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
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ); // Stores the triaxial accelerometer.
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);  // Stores the inertial sensor attitude.
  M5.IMU.getTempData(&temp);                // Stores the inertial sensor temperature to temp.

  // Publish IMU data
  //  imu_data_msg.orientation.x=pitch;
  //  imu_data_msg.orientation.y=roll;
  //  imu_data_msg.orientation.z=yaw;
  //  imu_data_msg.angular_velocity.x=gyroX;
  //  imu_data_msg.angular_velocity.y=gyroY;
  //  imu_data_msg.angular_velocity.z=gyroZ;
  //  imu_data_msg.linear_acceleration.x=accX;
  //  imu_data_msg.linear_acceleration.y=accY;
  //  imu_data_msg.linear_acceleration.z=accZ;
  //  imu_data_msg_pub.publish(&imu_data_msg);

  // Publish ACC data
  accel_data_msg.data[0] = accX * G;
  accel_data_msg.data[1] = accY * G;
  accel_data_msg.data[2] = accZ * G;
  accel_data_msg_pub.publish(&accel_data_msg);

  // //Print gyro data
  // M5.Lcd.setCursor(0, 20);  //Move the cursor position to (x,y).
  // M5.Lcd.print("gyroX,  gyroY, gyroZ"); //Screen printing formatted string.
  // M5.Lcd.setCursor(0, 42);
  // M5.Lcd.printf("%6.2f %6.2f%6.2f o/s", gyroX, gyroY, gyroZ);

  // //Print acc data
  M5.Lcd.setCursor(0, 70);
  M5.Lcd.print("accX,   accY,  accZ");
  M5.Lcd.setCursor(0, 92);
  M5.Lcd.printf("%5.2f  %5.2f  %5.2f G", accX, accY, accZ);

  // //Print pose data
  M5.Lcd.setCursor(0, 120);
  M5.Lcd.print("pitch,  roll,  yaw");
  M5.Lcd.setCursor(0, 142);
  M5.Lcd.printf("%5.2f  %5.2f  %5.2f deg", pitch, roll, yaw);
  orientation_data_msg_pub.publish(&orientation_data_msg);

  // //Print intertial sensor temp
  // M5.Lcd.setCursor(0, 175);
  // M5.Lcd.printf("Temperature : %.2f C", temp);
}
