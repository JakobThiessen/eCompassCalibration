/*!
 * @file getCompassdata.ino
 * @brief Output the compass data
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      [dexian.huang](952838602@qq.com)
 * @version  V1.0
 * @date  2017-7-3
 * @url https://github.com/DFRobot/DFRobot_QMC5883
 */
#include <DFRobot_QMC5883.h>

DFRobot_QMC5883 compass(&Wire, /*I2C addr*/QMC5883_ADDRESS);

void setup()
{
  Serial.begin(115200);
  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }

  if(compass.isHMC())
  {
    Serial.println("Initialize HMC5883");

    //Set/get the compass signal gain range, default to be 1.3 Ga
    // compass.setRange(HMC5883L_RANGE_1_3GA);
    // Serial.print("compass range is:");
    // Serial.println(compass.getRange());

    //Set/get measurement mode
    // compass.setMeasurementMode(HMC5883L_CONTINOUS);
    // Serial.print("compass measurement mode is:");
    // Serial.println(compass.getMeasurementMode());

    //Set/get the data collection frequency of the sensor
    // compass.setDataRate(HMC5883L_DATARATE_15HZ);
    // Serial.print("compass data rate is:");
    // Serial.println(compass.getDataRate());

    //Get/get sensor status
    // compass.setSamples(HMC5883L_SAMPLES_8);
    // Serial.print("compass samples is:");
    // Serial.println(compass.getSamples());
  }
  else if(compass.isQMC())
  {
    Serial.println("Initialize QMC5883");
     compass.setRange(QMC5883_RANGE_2GA);
     Serial.print("compass range is:");
     Serial.println(compass.getRange());

     compass.setMeasurementMode(QMC5883_CONTINOUS);
     Serial.print("compass measurement mode is:");
     Serial.println(compass.getMeasurementMode());

     compass.setDataRate(QMC5883_DATARATE_50HZ);
     Serial.print("compass data rate is:");
     Serial.println(compass.getDataRate());

     compass.setSamples(QMC5883_SAMPLES_8);
     Serial.print("compass samples is:");
     Serial.println(compass.getSamples());
  }
  delay(1000);
}

uint8_t data[100];

void loop()
{
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  sprintf((char*)data, "RAW: %d,%d,%d,%d,%d,%d,%d,%d,%d\n\r", 0, 0, 0, 0, 0, 0, mag.XAxis, mag.YAxis, mag.ZAxis, 0 );
  Serial.print((char*)data);
  delay(20);
}
