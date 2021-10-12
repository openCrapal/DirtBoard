#include <SPI.h>
#include <WiFiNINA.h>

#define SECRET_SSID "FRITZ!Box 7590 GJ"
#define SECRET_PASS "88361075116169088293"

char ssid[] = SECRET_SSID;     // the name of your network
char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS;     // the Wifi radio's status

byte mac[6];                     // the MAC address of your Wifi Module


void setup()
{
  Serial.begin(9600);

  while (status != WL_CONNECTED) {
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 1 seconds for connection:
    delay(1000);
    // if you are connected, print your MAC address:
  }

  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[0]);
  Serial.print(",");
  Serial.print(mac[1]);
  Serial.print(",");
  Serial.print(mac[2]);
  Serial.print(",");
  Serial.print(mac[3]);
  Serial.print(",");
  Serial.print(mac[4]);
  Serial.print(",");
  Serial.println(mac[5]);

}

void loop () {}
