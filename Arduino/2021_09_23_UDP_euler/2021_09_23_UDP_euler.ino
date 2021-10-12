
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>


#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "globals.h"


buffer_data local_data;
buffer_data target_data;

const int ledYellow = 4; // pin to use for the LED
const int ledRed = 5; // pin to use for the LED
const int ledGreen = 3; // pin to use for the LED
const int pinButton = 0;
const int pinDeadMan = 7;

char sensor_quat[48];

#define CTIME 20

long tc = 0;
long compteur = 0;

bool notAus = true;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(pinButton, INPUT_PULLUP);
  pinMode(pinDeadMan, INPUT_PULLUP);

  Serial.begin(115200);    // initialize serial communication
  while (!Serial and not digitalRead(pinButton));
  delay(500);

  bno_connect();
  //displaySensorStatus();
  //displayCalStatus();



  initUDP(target_data, local_data);

}


void loop() {


  notAus = digitalRead(pinDeadMan);
  if (notAus)
  {
    //Serial.println("not aus!");
  }

  get_euler(local_data);
  transferUDP(target_data, local_data);


  while (tc + CTIME >= millis())           // lower limit of the cycle time
  {
    digitalWrite(ledYellow, LOW);
  }
  digitalWrite(ledYellow, HIGH);

  tc = millis();

  compteur += 1;
}  // END_LOOP
