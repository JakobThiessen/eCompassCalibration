#include <Wire.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_LSM303_Accel.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303DLH_Mag_Unified magnet = Adafruit_LSM303DLH_Mag_Unified(12345);
//Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

void displaySensorDetails_mag(void)
{
  sensor_t sensor;
  magnet.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorDetails_acc(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup()
{
  #ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(115200);

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  accel.setRange(LSM303_RANGE_2G);
  accel.setMode(LSM303_MODE_HIGH_RESOLUTION);
  
  /* Enable auto-gain */
  magnet.enableAutoRange(true);
  /* Initialise the sensor */
  if(!magnet.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    //while(1);
  }

  magnet.setMagGain(LSM303_MAGGAIN_2_5);
  magnet.setMagRate(LSM303_MAGRATE_75);
  
  /* Display some basic information on this sensor */
//  displaySensorDetails_acc();
//  displaySensorDetails_mag();

}

sensors_event_t event;
sensors_event_t eventMag;

void loop()
{
    /* Get a new sensor event */
  //gyro.getEvent(&event);
  accel.getEvent(&event);
  magnet.getEvent(&eventMag);
  
  Serial.print("Raw:");
  Serial.print(event.acceleration.x); Serial.print(",");
  Serial.print(event.acceleration.y); Serial.print(",");
  Serial.print(event.acceleration.z); Serial.print(",");
  Serial.print(event.gyro.x); Serial.print(",");
  Serial.print(event.gyro.y); Serial.print(",");
  Serial.print(event.gyro.z); Serial.print(",");
  Serial.print(eventMag.magnetic.x); Serial.print(",");
  Serial.print(eventMag.magnetic.y); Serial.print(",");
  Serial.print(eventMag.magnetic.z); Serial.println("");

/*
  // unified data
  Serial.print("Uni:");
  Serial.print(event.acceleration.x); Serial.print(",");
  Serial.print(event.acceleration.y); Serial.print(",");
  Serial.print(event.acceleration.z); Serial.print(",");
  Serial.print(event.gyro.x, 4); Serial.print(",");
  Serial.print(event.gyro.y, 4); Serial.print(",");
  Serial.print(event.gyro.z, 4); Serial.print(",");
  Serial.print(event.magnetic.x); Serial.print(",");
  Serial.print(event.magnetic.y); Serial.print(",");
  Serial.print(event.magnetic.z); Serial.println("");
*/
delay(50);
}
