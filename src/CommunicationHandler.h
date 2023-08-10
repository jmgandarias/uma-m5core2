#ifndef __HRII_COMMUNICATION_HANDLER_H__
#define __HRII_COMMUNICATION_HANDLER_H__

namespace HRII
{

  class CommunicationHandler
  {
  public:
    CommunicationHandler(char *ssid_desired, char *password) : gateway(192, 168, 0, 1), subnet(255, 255, 255, 0)
    {
      ssidDesired_ = ssid_desired; // wifi network name
      password_ = password;        // wifi network password
    }

    CommunicationHandler()
    {
    }

    void init()
    {
    }

    String scanWifi()
    {

      int n = WiFi.scanNetworks(); // Store the number of wifi scanned into n.
      delay(100);
      bool networkNotFound = true;
      for (int i = 0; i < n && networkNotFound; i++)
      { // Save each wifi name scanned to ssidList.
        if (i == n - 1)
        {
          M5.Lcd.println("Looking for network..");
          n = WiFi.scanNetworks(); // Updated number of scanned networks number
          i = 0;                   // Resetting i
        }
        ssidName_ = WiFi.SSID(i);
        if (ssidName_ == ssidDesired_)
        {
          M5.Lcd.println(ssidName_ + " network found!");
          networkNotFound = false;
        }
      }
      delay(500);

      return ssidList_;
    }

    // WiFi Network
    char *ssidDesired_;
    char *password_;

    WiFiClient wifi_client;

    // IP Addresses
    static IPAddress client_ip_; // roscore Client adress
    static IPAddress server_ip_; // roscore ip adress
    IPAddress gateway;
    IPAddress subnet;

    // WiFi variables
    String ssidList_;
    String ssidName_;

    void setupWiFi()
    {

      WiFi.begin(ssidDesired_, password_);
      WiFi.config(client_ip_, gateway, subnet);

      M5.Lcd.print("Connecting to: ");
      M5.Lcd.println(ssidDesired_);

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
        M5.Lcd.println(ssidDesired_);

        while (1)
          delay(500);
      }

      M5.Lcd.println("Ready Use! Client IP address:");
      IPAddress ip = WiFi.localIP();
      M5.Lcd.println(ip);
      delay(1000);
    }
  }; // class CommunicationHandler

} // namespace HRII

class WiFiHardware
{
public:
  WiFiHardware(){};
  void init()
  {
    comm_handler.wifi_client.connect(comm_handler.server_ip_, 11411);
  }

  int read()
  {
    return comm_handler.wifi_client.read();
  }

  void write(uint8_t *data, int length)
  {
    for (int i = 0; i < length; i++)
      comm_handler.wifi_client.write(data[i]);
  }

  unsigned long time()
  {
    return millis(); // Easy; Did This One For You
  }

public:
  HRII::CommunicationHandler comm_handler;

}; // class WiFiHardware

#endif // __HRII_COMMUNICATION_HANDLER_H__
