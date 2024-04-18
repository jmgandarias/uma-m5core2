/*
Example script of multitasking with ROS using the UMA_M5Core2 library.
The M5Core2 integrates an ESP32 micropocessor which has two cores.
This script runs the following two tasks exploiting the multicore option of the ESP32:
 - Task1: Communication. One core will be fully dedicated to keep the communication using the ros library
 - Task2: GUI. The second core will be on charge of managing the Graphical User Interface

For more info about multitaskng with M5Core2 and ESP32:
- https://github.com/m5stack/M5Core2/blob/master/examples/Advanced/MultiTask/MultiTask.ino
- https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html

Developer:  Juan M. Gandarias 
            https://jmgandarias.com 
            jmgandarias@uma.es

Data: 18/04/2024
*/

#include <ros.h>
#include <UMAM5Core2.h>
#include <std_msgs/Float32.h>


char ssidDesired[] = "your_network_name"; // wifi network name
char password[] = "your_network_password";          // wifi network password
IPAddress client_ip(192, 168, 0, 40);    // M5 IP
IPAddress server_ip(192, 168, 0, 157);   // Roscore IP

UMA::CommunicationHandler comm_handler(ssidDesired, password);

IPAddress UMA::CommunicationHandler::client_ip_ = client_ip;
IPAddress UMA::CommunicationHandler::server_ip_ = server_ip;

// Auxiliar functions
float publishSliderData(float prev_value, float current_value);

// Slider variables
float slider_value = 0.5; // Value between 0 and 1, initial value in the middle
int16_t slider_x_pos = 30;
int16_t slider_y_pos = LCD_Y_LENGTH / 2;
float slider_prev_value = 0;
// Slider(int16_t x_cord, int16_t y_cord, int16_t length, float initial_value, bool orientation, int8_t radius, int8_t tolerance, uint32_t color, uint32_t background_color)
Slider slider(slider_x_pos, slider_y_pos, LCD_X_LENGTH - 2 * slider_x_pos, slider_value, 0, 10, 5, WHITE, BLACK);

// Tasks variables
int communication_task_freq = 100;
int gui_task_freq = 25;
int communication_task_initialization_flag = 0;

// Ros Node Handle
ros::NodeHandle_<WiFiHardware> nh;

std_msgs::Float32 slider_msg;
ros::Publisher slider_msg_pub("slider_scaling", &slider_msg, 1);

void vCommunicationTask(void *pvParameters)
{ // Define the tasks to be executed in Core 0.
  int16_t t1 = 0;

  // Scan Wifi
  String ssid_list = comm_handler.scanWifi();
  M5.Lcd.println(ssid_list);
  comm_handler.setupWiFi();
  delay(500);
  M5.Lcd.clear();

  // Initialize the node
  nh.initNode();

  // Initialize Slider
  nh.advertise(slider_msg_pub);
  slider_msg.data = slider_value;
  slider_msg_pub.publish(&slider_msg);
  delay(500); // Wait to initialize the publishers
  communication_task_initialization_flag = 1;

  while (1)
  { // Keep the thread running.
    nh.spinOnce();
    M5.update();
    slider_prev_value = publishSliderData(slider_prev_value, slider_value);
    Serial.print("Communication Task Uptime (s): ");
    Serial.println(millis() - t1);
    t1 = millis();
    delay(1/communication_task_freq);
  }
}

void vGUITask(void *pvParameters)
{
  int16_t t2 = 0;

  Serial.begin(115200);
  M5.begin();

  M5.Lcd.println("waiting for communication task initialization");
  while(!communication_task_initialization_flag){
     //while until the communication task is initialized
     M5.Lcd.print(".");
     delay(200);
  }
 
  slider.printSlider();

  while (1)
  {
    nh.spinOnce();
    M5.update();
    slider_value = slider.updateSlider();
    M5.Axp.SetLcdVoltage(MIN_LCD_VOLTAGE + slider_value * (MAX_LCD_VOLTAGE - MIN_LCD_VOLTAGE));
    Serial.print("GUI Task Uptime (s): ");
    Serial.println(millis() - t2);
    t2 = millis();
    delay(gui_task_freq);
  }
}

void setup()
{
  /*
   xTaskCreatePinnedToCore expects the following parameters:
    - fuction to implement the task
    - name of the task (string)
    - number of bytes of the task
    - Pointer that will be used as the parameter of the task
    - Task handler
    - Core where the task will run (0 or 1)
  */
  // Task 1 (Core 0)
  xTaskCreatePinnedToCore(vCommunicationTask, "Communication Task", 4096, NULL, 1, NULL, 0);

  // Task 2 (Core 1)
  xTaskCreatePinnedToCore(vGUITask, "GUI Task", 4096, NULL, 2, NULL, 1);
}

void loop()
{
}

float publishSliderData(float prev_value, float current_value)
{
  if (current_value != prev_value)
  {
    slider_msg.data = current_value;
    slider_msg_pub.publish(&slider_msg);
  }
  return current_value;
}
