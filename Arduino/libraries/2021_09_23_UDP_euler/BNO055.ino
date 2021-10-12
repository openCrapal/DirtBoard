
/**************************************************************************/
/*
    Configure the fusion sensor BNO055
*/
/**************************************************************************/

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS (20);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensor_t sensor;

long debounce_t0 = -3000;

uint8_t systm, gyro, accel, mag;
uint8_t system_status, self_test_results, system_error;


//sensors_event_t event;
//imu::Quaternion quat_old = imu::Quaternion(1., 0., 0., 0.);
//imu::Quaternion quat = imu::Quaternion(1., 0., 0., 0.);
//imu::Quaternion ref_quat = imu::Quaternion(1., 0., 0., 0.);
//imu::Quaternion incoming_quat = imu::Quaternion(1., 0., 0., 0.);


imu::Vector<3> euler = imu::Vector<3>(0., 0., 0.);
imu::Vector<3> euler_old = imu::Vector<3>(0., 0., 0.);

/**************************************************************************/
/*
    Configure the Storage of the sensor calibration data in the Flash memory
*/
/**************************************************************************/

#include <FlashStorage.h>

FlashStorage(flash_calib_data, adafruit_bno055_offsets_t);
FlashStorage(flash_sensor_id, long);

void bno_connect() {
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  //Serial.print("Current Temperature: ");
  //Serial.print(temp);
  //Serial.println(" °C");
  //Serial.println("");

  //Serial.print("Connectet to BNO. Sensor iD: ");
  digitalWrite(ledRed, HIGH);

  bno.getSensor(&sensor);
  long sensor_id = sensor.sensor_id;

  if (sensor_id != flash_sensor_id.read())
  {
    //No calibration data
  }
  else
  {

    adafruit_bno055_offsets_t calibrationData = flash_calib_data.read();
    bno.setSensorOffsets(calibrationData);
    //displaySensorOffsets(calibrationData);
  }
  bno.setExtCrystalUse(true);
}


void save_calib_data()
{

  adafruit_bno055_offsets_t calibrationData;
  // Calib ok + press button => write calibration datao of th blo055 in the flash memory
  if ((debounce_t0 + 5000 < millis()))
  {
    debounce_t0 = millis();
    if (displayCalStatus())
    {
      bno.getSensorOffsets(calibrationData);
      digitalWrite(ledRed, LOW);
      flash_sensor_id.write(sensor.sensor_id);
      flash_calib_data.write(calibrationData);
    }
    else

    {
      Serial.println("Calib : nicht ausreichend ");
      digitalWrite(ledRed, HIGH);
      debounce_t0 = millis();
    }
  }
}


/*
  bool get_quat(buffer_data & buff)
  {
  save_calib_data();

  quat = bno.getQuat();
  if (quat.w() == quat_old.w() && quat.x() == quat_old.x())
  {
    return false;
  }
  else
  {
    quat_old = quat;
    buff.f_data[0] = quat.w();
    buff.f_data[1] = quat.x();
    buff.f_data[2] = quat.y();
    buff.f_data[3] = quat.z();

    //quat = imu::Quaternion(buff.f_data[0], buff.f_data[1], buff.f_data[2], buff.f_data[3]);
    //quat_to_char_array(quat, buff.data[Who_Am_I][2], buff.data[Who_Am_I][3], buff.data[Who_Am_I][4], buff.data[Who_Am_I][5]);

    return true;
  }

  }
*/

bool get_euler(buffer_data & buff)
{
  save_calib_data();

  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

//  Serial.print("X: ");
//  Serial.print(euler.x());
//  Serial.print(" Y: ");
//  Serial.print(euler.y());
//  Serial.print(" Z: ");
//  Serial.println(euler.z());

  if (euler.x() == euler_old.x() && euler.z() == euler_old.z())
  {
    return false;
  }
  else
  {
    euler_old = euler;
    buff.f_data[0] = euler.x();
    buff.f_data[1] = euler.y();
    buff.f_data[2] = euler.z();

    return true;
  }

}


/************************************************************************** /
  /*
    Utilities
*/
/**************************************************************************/
/*
  void quat_to_char_array(imu::Quaternion quat, char *buff0, char *buff1, char *buff2, char *buff3)
  {
  double_to_char_array(quat.w(), buff0);
  double_to_char_array(quat.x(), buff1);
  double_to_char_array(quat.y(), buff2);
  double_to_char_array(quat.z(), buff3);
  }

  void char_array_to_quat(char *my_char, imu::Quaternion &quat)
  {
  double w, x, y, z;
  char* pEnd;
  w = strtod(my_char, &pEnd);
  x = strtod(pEnd, &pEnd);
  y = strtod(pEnd, &pEnd);
  z = strtod(pEnd, NULL);

  quat = imu::Quaternion(w, x, y, z);

  }
*/
/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/



bool displayCalStatus(void)
{
  // Get the four calibration values (0..3)
  // 3 means 'fully calibrated'

  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  // Display the individual values
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);

  bool calib_ok = false;
  if (int(system) > 1)
  {
    calib_ok = true;
  }
  return calib_ok;
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
*/
/**************************************************************************/


void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
  Serial.print("Accelerometer: \t");
  Serial.print(calibData.accel_offset_x, 5); Serial.print("\t");
  Serial.print(calibData.accel_offset_y, 5); Serial.print(" \t");
  Serial.print(calibData.accel_offset_z, 5); Serial.print(" \t");

  Serial.print("\nGyro: \t");
  Serial.print(calibData.gyro_offset_x); Serial.print(" \t");
  Serial.print(calibData.gyro_offset_y); Serial.print(" \t");
  Serial.print(calibData.gyro_offset_z); Serial.print(" \t");

  Serial.print("\nMag: \t");
  Serial.print(calibData.mag_offset_x); Serial.print(" \t");
  Serial.print(calibData.mag_offset_y); Serial.print(" \t");
  Serial.print(calibData.mag_offset_z); Serial.print(" \t");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.println(calibData.mag_radius);
}
/*
  /**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/


void displaySensorStatus(void)
{
  //Get the system status values (mostly for debugging purposes)
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  //Display the results in the Serial Monitor
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}
