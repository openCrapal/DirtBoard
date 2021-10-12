#include <RobotEQ.h>

#define CHANNEL_1 1
#define CHANNEL_2 2

long compteur = 0;
long increment = 10;

// Configure Motor Controllers
RobotEQ controller(&Serial1);

void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  pinMode(7, INPUT_PULLUP);
}

void loop()
{
  while (digitalRead(7))
  {
    
  }

  int voltage = 0;
  int amps = 0;

  compteur += increment;

  
  if (compteur > 900)
  {
    increment = -10;
    compteur = 900;
  }
  else if (compteur < -900)
  {
    increment = 10;
    compteur = -900;
  }

  Serial.println(controller.queryMotorVoltage());
  
  if (controller.isConnected() == 0) {
    voltage = controller.queryBatteryVoltage();
    amps = controller.queryBatteryAmps();

    controller.commandMotorPower(CHANNEL_2, compteur);
  }
  
  delay(10);
  
}
