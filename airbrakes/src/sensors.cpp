#include "main.h"
#include <Arduino.h>
//#include <Adafruit_Sensor_Calibration.h>
//include <Adafruit_Sensor_Calibration_SDFat.h>
//#include <BNO055.h>
#include <Adafruit_BNO055.h>

#define BNO055_SAMPLERATE_DELAY_MS 10

/*
bool initSensors(void) {
  if (!lsm6ds.begin_I2C(0x6B) || !lis3mdl.begin_I2C(0x1E) || !baro.begin() || !lps.begin_I2C()) {
    Serial.println(lsm6ds.begin_I2C(0x6B));
    Serial.println(lis3mdl.begin_I2C(0x1E));
    Serial.println(baro.begin());
    Serial.println(lps.begin_I2C());
    return false;
  } 
  accelerometer = lsm6ds.getAccelerometerSensor();
  gyroscope = lsm6ds.getGyroSensor();
  magnetometer = &lis3mdl;

  return true;
}
*/
//V3

bool initSensors(void) {
  bno055 = Adafruit_BNO055(19, 0x29);
  /*if (!bno055.begin() || !baro.begin()) {
    Serial.println(baro.begin());
    Serial.println(bno055.begin());
    return false;
  } */
 if (!bno055.begin() || !bmp_baro.begin_I2C(0x77))
  return true;
}


void setupSensors(void) {

/*
  // set lowest range
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

//  Serial.println("setup_sensors check 1");

  // set slightly above refresh rate
  lsm6ds.setAccelDataRate(LSM6DS_RATE_52_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_52_HZ);
  //lsm6ds.highPassFilter(true, LSM6DS_HPF_ODR_DIV_400);

 // Serial.println("setupSensors check 1.5");

  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  */

 // Serial.println("setup_sensors check 2");
/*
  baro.setSeaPressure(SEAPRESSURE);  
  baro.setMode(MPL3115A2_ALTIMETER);
  baro.startOneShot();
  */

 bmp_baro.readAltitude(rocketConfig.getPressure());
 bmp_baro.setOutputDataRate(BMP3_ODR_100_HZ);

  calibrateSensors();

 // Serial.println("setup_sensors check 3");
}


SdFile calfile;


void initCalibration(void){
  //if (!sd.begin(4, SPI_HALF_SPEED)) sd.initErrorHalt();

  if (!cal.begin("calibrat.dat")){
    Serial.println("Failed to initialize calibration helper");
    
  }else if (!cal.loadCalibrationFromFile()){
    Serial.println("No calibration loaded/found");
  }
 /*
  if (!cal.begin(0)){
    Serial.println("Failed to initialize calibration helper");
    while (1) { yield(); }
  }*/
  
 // cal.printSavedCalibration();
  calfile.close();

}

void calibrateSensors(void){
  cal.calibrate(accel);
  cal.calibrate(gyro);
  cal.calibrate(mag);
}