
/* Configuration Sensor BNO055 */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (200)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define CTIME (200)
long t0;

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

/* Configure Motor controller*/
#include <RobotEQ.h>
#define CHANNEL_1 1
#define CHANNEL_2 2

// Configure Motor Controllers
RobotEQ controller(&Serial1);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

void setup() {
  pinMode(13, OUTPUT);
  Serial1.begin(115200);
  Serial.begin(115200);
  
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  delay(200);
  /* Display some basic information on this sensor */
  displaySensorDetails();
  bno.setExtCrystalUse(true);
}

void loop() {
  t0 = millis();
/* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  float Soll = event.orientation.y;
  long IntSoll = round(Soll);
  
  Serial.print("X: ");
  Serial.print(Soll);
  Serial.print("\tY: ");
  Serial.print(IntSoll);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  displayCalStatus();
  
  int voltage;
  int amps;
  int power;
  
  if (controller.isConnected()==0) {
    voltage = controller.queryBatteryVoltage();
    amps = controller.queryBatteryAmps(1);
    power = controller.queryMotorPower(2);
    Serial.print("\tV: ");
    Serial.print(voltage);
    Serial.print("\tA: ");
    Serial.print(amps);
    Serial.print("\tP: ");
    Serial.println(power);
    controller.commandMotorPower(CHANNEL_1, Soll);
    controller.commandMotorPower(CHANNEL_2, Soll);
    digitalWrite(13, true);
  }
  else
  {
    digitalWrite(13,false);
    Serial.println("\t Controller is not connected");
  }
  while( millis() < t0 + CTIME)
  {
    
  }

  
}
