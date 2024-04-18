/*
Example script to communicate with the M5Core2 via ROS.

Developer:  Juan M. Gandarias 
            https://jmgandarias.com 
            jmgandarias@uma.es

Former developer: Mattia Leonori, 
                  mattia.leonori@iit.it
                    
Data: 18/04/2024
*/

#include <M5Core2.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <SPI.h>
#include <WiFi.h>

// WiFi Network
char ssidDesired[] = "your_network_name"; // wifi network name
char password[] = "your_network_password";    // wifi network password

WiFiClient wifi_client;

// IP Addresses
IPAddress server_ip(192, 168, 0, 13); // roscore ip adress
IPAddress client_ip(192, 168, 0, 40); // roscore Client adress (IP of the M5)

// WiFi variables
String ssidList;
String ssidName;

// M5 LCD
TFT_eSprite Img = TFT_eSprite(&M5.Lcd);

// Auxiliar functions
void waitingScreen();
void SetupWiFi();

class WiFiHardware
{
public:
  WiFiHardware(){};
  void init()
  {
    wifi_client.connect(server_ip, 11411);
  }

  int read()
  {
    return wifi_client.read();
  }

  void write(uint8_t *data, int length)
  {
    for (int i = 0; i < length; i++)
      wifi_client.write(data[i]);
  }

  unsigned long time()
  {
    return millis(); // Easy; Did This One For You
  }
};

// Ros Node Handle
ros::NodeHandle_<WiFiHardware> nh;

// Message Callback
void MessageCb(const std_msgs::Empty &toggle_msg)
{
  M5.Lcd.fillScreen(TFT_GREEN);
  delay(200);
  waitingScreen();
}

// Subscribers
ros::Subscriber<std_msgs::Empty> empty_msg_sub("toggle_led", &MessageCb, 1);

// Publishers
std_msgs::Empty empty_msg;
ros::Publisher button_msg_pub("toggle_button", &empty_msg);

void setup()
{
  Serial.begin(115200);

  M5.begin();

  int n = WiFi.scanNetworks(); // Store the number of wifi scanned into n.
  delay(100);
  M5.Lcd.println("");
  ssidList = "I found the network: ";
  for (int i = 0; i < n; ++i)
  { // Save each wifi name scanned to ssidList.
    ssidName = WiFi.SSID(i);
    //    ssidList += "<option value=\"";
    if (ssidName == ssidDesired)
    {
      ssidList += WiFi.SSID(i);
      ssidList += "\n";
    }
  }
  M5.Lcd.println(ssidList);
  delay(500);

  SetupWiFi();

  M5.Lcd.fillScreen(TFT_BLUE);
  Img.setColorDepth(8);
  Img.createSprite(320, 240);
  Img.fillSprite(TFT_BLACK);

  nh.initNode();
  nh.subscribe(empty_msg_sub);
  nh.advertise(button_msg_pub);
  button_msg_pub.publish(&empty_msg);

  waitingScreen();
}

void loop()
{
  if (M5.BtnB.isPressed())
  { // If the key is pressed.
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString("message_published", 160, 120, 2);
    button_msg_pub.publish(&empty_msg);
    delay(200);
    waitingScreen();
  }
  nh.spinOnce();
  delay(10);
  M5.update();
}

void SetupWiFi()
{
  //  WiFi.config(client_ip);
  WiFi.begin(ssidDesired, password);

  M5.Lcd.print("Connecting to: ");
  M5.Lcd.println(ssidDesired);

  uint8_t i = 0;

  while (!(WiFi.status() == WL_CONNECTED) && (i++ < 200))
  {
    M5.Lcd.print(".");
    delay(500);
  }

  M5.Lcd.print("\n");

  if (WiFi.status() != WL_CONNECTED)
  {
    M5.Lcd.println("Could Not Connect To");
    M5.Lcd.println(ssidDesired);

    while (1)
      delay(500);
  }

  M5.Lcd.println("Ready Use! Client IP address:");
  IPAddress ip = WiFi.localIP();
  M5.Lcd.println(ip);
  delay(1000);
}

void waitingScreen()
{
  M5.Lcd.setTextDatum(MC_DATUM); // Set text alignment to center-aligned
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.drawFastHLine(60, 50, 200, WHITE); // Draw a white horizontal line with a length of 255 at (3,100)
  M5.Lcd.drawString("Waiting", 160, 70, 2);
  M5.Lcd.drawString("for", 160, 100, 2);
  M5.Lcd.drawString("messages", 160, 130, 2);
  M5.Lcd.drawFastHLine(60, 150, 200, WHITE); // Draw a white horizontal line with a length of 255 at (3,100)

  M5.Lcd.drawString("(Press the middle button to publish an empty msg)", 160, 200, 2); // Draw a white horizontal line with a length of 255 at (3,100)
}
