
#define SECRET_SSID "FRITZ!Box 7590 GJ"
#define SECRET_PASS "88361075116169088293"


#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

int status = WL_IDLE_STATUS;
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key index number (needed only for WEP)

//char hostName[] = "motorx";
char hostNameServer[] = "server";
char hostNameMotor1[] = "motor1";
char hostNameMotor2[] = "motor2";

unsigned int localPort = 2390;      // local port to listen on

char packetBuffer[128]; //buffer to hold incoming packet

# include "globals.h"

byte MAC_Server[] = {10, 05, 0, 0, 0, 01};
byte MAC_Motor1[] = {56, 125, 134, 58, 125, 128};
byte MAC_Motor2[] = {116, 210, 134, 18, 207, 164};

IPAddress target1;
IPAddress target2;

WiFiUDP Udp;

bool connected_to_server = false;



/**************************************************************************/
/*
    Init
*/
/**************************************************************************/


int initUDP(buffer_data & buff_in, buffer_data & buff_out)
{
  byte mac[6];
  WiFi.macAddress(mac);


  if (compare(mac, 6, MAC_Server))  {
    Who_Am_I = 0;
    WiFi.setHostname(hostNameServer);

  }
  else if (compare(mac, 6, MAC_Motor1))  {
    Who_Am_I = 1;
    WiFi.setHostname(hostNameMotor1);
  }
  else if (compare(mac, 6, MAC_Motor2))  {
    Who_Am_I = 2;
    WiFi.setHostname(hostNameMotor2);
  }
  else
  {
    Serial.println("unbekannte mac adresse");
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

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 2 seconds for connection:
    delay(2000);

  }

  Serial.println("connected to Wlan");
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address self: ");
  Serial.println(ip);

  Udp.begin(localPort);


  // Swap and init memory buffer

  buff_in.l_data = uint16_t(1);
  buff_in.l_data_old = uint16_t(1);

  buff_in.l_time = uint16_t(millis());
  buff_in.l_time_old = uint16_t(0);

  buff_out.l_data = uint16_t(1);
  buff_out.l_data_old = uint16_t(1);

  buff_out.l_time = uint16_t(millis());
  buff_out.l_time_old = uint16_t(0);

  for ( int j = 0; j < N_CARACS; j++)
  {
    buff_in.f_data[j] = 1.0;
    buff_in.f_data_old[j] = 1.0;

    buff_out.f_data[j] = 1.0;
    buff_out.f_data_old[j] = 1.0;
  }


}

int reconnect()
{

  if (Who_Am_I == 1)
  {
    int err = WiFi.hostByName(hostNameMotor2, target1) ;
    if (err == 1) {
      Serial.print("Ip address target: ");
      Serial.println(target1);
      connected_to_server = true;
    } else {
      Serial.print("Error code: ");
      Serial.println(err);
    }

  }
  else if (Who_Am_I == 2)
  {
    int err = WiFi.hostByName(hostNameMotor1, target1) ;
    if (err == 1) {
      Serial.print("Ip address target: ");
      Serial.println(target1);
      connected_to_server = true;
    } else {
      Serial.print("Error code: ");
      Serial.println(err);
    }

  }




  /*
    Udp.beginPacket(Udp.remoteIP(), localPort);
    Udp.write(ReplyBuffer);
    Udp.endPacket();*/
}


/**************************************************************************/
/*
    Transfer Data
*/
/**************************************************************************/


int transferUDP(buffer_data &buff_in, buffer_data &buff_out)
{
  digitalWrite(ledGreen, LOW);
  if (connected_to_server)
  {
    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {

      for (int i = 0; i < N_CARACS; i++)
      {
        buff_in.f_data_old[i] = buff_in.f_data[i];
      }
      buff_in.l_data_old = buff_in.l_data;

      //Serial.print("Received packet of size ");
      //Serial.println(packetSize);
      //Serial.print("From ");
      IPAddress remoteIp = Udp.remoteIP();
      //Serial.print(remoteIp);
      //Serial.print(", port ");
      //Serial.println(Udp.remotePort());

      // read the packet into packetBufffer
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;
      }
      //Serial.println("Contents:");
      //Serial.println(packetBuffer);

      if (not char_buffer_to_data(packetBuffer, 127, buff_in))
      {
        Serial.println("invalid data");
        for (int i = 0; i < N_CARACS; i++)
        {
          buff_in.f_data[i] = buff_in.f_data_old[i];
        }
        buff_in.l_data = buff_in.l_data_old;
      }
      else
      {
        digitalWrite(ledGreen, HIGH);

        char_buffer_to_data(packetBuffer, 127, buff_in);


      }
    }


    // send a reply, to the IP address and port of the other motor
    Udp.beginPacket(target1, localPort);


    buff_out.l_time_old = buff_out.l_time;
    buff_out.l_time = millis();

    data_to_char_buffer(buff_out, 127, packetBuffer);

    Udp.write(packetBuffer);
    Udp.endPacket();

    Serial.print("DX: ");
    Serial.print(buff_out.f_data[0] - buff_in.f_data[0]);
    Serial.print(" DY: ");
    Serial.print(buff_out.f_data[1] - buff_in.f_data[1]);
    Serial.print(" DZ: ");
    Serial.println(buff_out.f_data[2] - buff_in.f_data[2]);

  }
  else
  {
    reconnect();
  }
}


/*
  int initLowCom(buffer_data & buff)
  {
  if (!BLE.begin()) {
    //Serial.println("starting BLE failed!");
    while (1);
  }
  s_address = BLE.address();
  s_address.toCharArray(c_address , 18);
  Serial.print("Local BLE address: ");
  Serial.println(c_address);


  // Use the addresses to define Who_Am_I
  for (int i = 0; i < 3; i++)
  {
    if (compare(c_address , 18, addrs[i]))
    {
      Who_Am_I = i;
    }
  }

  // Swap and init memory buffer
  for (int i = 0; i < N_DEVS; i++)
  {

    buff.l_data[i] = 0b0000000000000000;
    buff.l_data_old[i] = 0b0000000000000000;

    for ( int j = 0; j < N_CARACS; j++)
    {
      buff.f_data[i][j] = 0.;
      buff.f_data_old[i][j] = 0.;
    }
  }

  switch (Who_Am_I)
  {
    case ID_SERVER:
      {
        BLEService motorControlService("19B10000-E8F2-537E-5F6C-D1047ABA1214"); // BLE motorControl Service
        BLE.setAdvertisedService(motorControlService);

        float f_buff [N_CARACS];

        for (int i = 0; i < N_DEVS; i++)
        {

          for (int j = 0; j < N_CARACS; j++)
          {
            f_buff [j] = buff.f_data[i][j];
          }


          f_caracs[i] = new BLECharacteristic( f_uuids[i], BLERead | BLENotify | BLEWrite, sizeof(f_buff));
          f_caracs[i]->writeValue(buff.f_data[i], sizeof(f_buff));
          motorControlService.addCharacteristic(*f_caracs[i]);

          f_caracs[N_DEVS + i] = new BLEUnsignedIntCharacteristic( f_uuids[N_DEVS + i], BLERead | BLENotify | BLEWrite);
          f_caracs[N_DEVS + i]->writeValue(buff.l_data[i]);
          motorControlService.addCharacteristic(*f_caracs[N_DEVS + i]);

          Serial.println("added charac");

        }



        // add service
        BLE.addService(motorControlService);

        BLE.setEventHandler(BLEConnected, addDevice);
        BLE.setEventHandler(BLEDisconnected, remDevice);

        BLE.setLocalName("DirtBoardServer");
        BLE.advertise();
        break;
      }
    default:
      {
        //
        BLE.scanForAddress(addrs[0]);
        BLE.setLocalName("Motor");
        connect_to_server();
      }
  }

  return Who_Am_I;
  }
*/



/**************************************************************************/
/*
    Transfer Data
*/
/**************************************************************************/
