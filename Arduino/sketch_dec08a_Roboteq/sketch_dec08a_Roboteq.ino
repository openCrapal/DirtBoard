#include <RobotEQ.h>

#define CHANNEL_1 1
#define CHANNEL_2 2

// Configure Motor Controllers
RobotEQ controller(&Serial1);

void setup() {
  pinMode(13, OUTPUT);
  Serial1.begin(115200);
  Serial.begin(115200);
  Serial.println("So weit...0");
}

void loop() {
  int voltage;
  int amps;
  int power;
  
  if (controller.isConnected()==0) {
    voltage = controller.queryBatteryVoltage();
    amps = controller.queryBatteryAmps(1);
    power = controller.queryMotorPower(2);
    Serial.print("V: ");
    Serial.print(voltage);
    Serial.print("    A: ");
    Serial.print(amps);
    Serial.print("    P: ");
    Serial.println(power);
    controller.commandMotorPower(CHANNEL_1, 400);
    controller.commandMotorPower(CHANNEL_2, 40);
    digitalWrite(13, true);
  }
  else
  {
    digitalWrite(13,false);
    Serial.println("is not connected");
  }
  delay(200);

  
}
