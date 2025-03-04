#include <Arduino.h>
// #include <Adafruit_Sensor_Calibration.h>
// #include <Adafruit_Sensor_Calibration_SDFat.h>
#include <Adafruit_ADXL343.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_HTS221.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_MLX90393.h>
#include <Adafruit_MMC56x3.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_MSA301.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <SensorFusion.h>
#include <Adafruit_BMP3XX.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <SdFat.h>

// #include"maths.h"
#include "main.h"
#include "sim.h"
#include "config.h"
#include "coms.h"

#define TEST_TIME 60.0f
#define START_TIME 20.0f

SdFile Config;

//Adafruit_MPL3115A2 baro;
Adafruit_BMP3XX bmp_baro;

// V1

Adafruit_LSM6DS33 lsm6ds;
Adafruit_LPS25 lps;
Adafruit_LIS3MDL lis3mdl;

Adafruit_BNO055 bno055;

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// Adafruit_Sensor_Calibration_SDFat cal; Broken AF
sensorCalibration cal;

SF sensor_filter;

state rocketState;

brakeState airBrakeState;

controller rocketControl;

status rocketStatus;

config rocketConfig;


// GLOBAL SENSOR VALUES

float MPL_PRESSURE;
float MPL_ALTI;
float MPL_TEMP;

float LPS_PRESSURE;
float LPS_TEMP;

float ACC_X = 0.0f;
float ACC_Y = 0.0f;
float ACC_Z = 0.0f;

float GYRO_X = 0.0f;
float GYRO_Y = 0.0f;
float GYRO_Z = 0.0f;

float MAG_X = 0.0f;
float MAG_Y = 0.0f;
float MAG_Z = 0.0f;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t tempp;

float t = 0.0f;
float t_start = 0.0f;
float t_launch = 0.0f;
float t_last = 0.0f;
float dt = 0;

float start_altitude;
float target_apogee = 0.0f;

float test_dt = 0.0f;
float test_dt_now = 0.0f;
float test_dt_last = 0.0f;

float *copyQuat; // float array to copy quaternion to RocketState

void initPins();


void setup()
{
  initPins();
  Wire.begin();
  Wire.setClock(400000UL);
  
  t_start = (float)micros() / 1000000.0f;

  Serial.begin(115200);
  //while (!Serial);
  // Serial.println("Airbrakes!");
  delay(100);

  if (!initSensors())
  {
    Serial.println("Failed to initialize sensors!");
  }
  else
  {
 //   Serial.println("Worked!");
  }

  rocketState.stateType = ROCKET;
//  Serial.println("State type established");
  // Initialize rocket state history

//  Serial.println("rocketStateHistory created");
  initSD();
 // Serial.println("SD initialized");

  initLogs(); // Initialize state history logs

 // Serial.println("Logs initialized");
  // initBT();

  //rocketControl.initBrake();
  //rocketControl.deployBrake(0);
  //Serial.println("brake deployed");
  // initFlash(); // Initialize system flash
  initConfig();
  rocketConfig.loadConfigFromFile();

  //Serial.println(rocketConfig.getTargetApogee());

  airBrakeState.loadConfig(rocketConfig);

  Serial.println("config finished");
 // Serial.println("Config loaded");
  initCalibration();

  Serial.println("Cal loaded");

  setupSensors(); // setup sensors
  // delay(10000);

  /*for (;;){
    brakeTest(); // Test airbrake
  }*/

  readSensors(); // Read sensors
  
  /*for (int i = 0; i < 10; i++) // try to get barometer to converge
    rocketState.setGroundAltitude(baro.getAltitude()); */

  for (int i = 0; i < 5; i++)
    rocketState.setGroundAltitude(bmp_baro.readAltitude(rocketConfig.getPressure()));

  for (int i = 0; i < 500; i++){
    readSensors();
    rocketState.stepTime();
  }
  readSensors();
  rocketState.updateState();
  //rocketState.setAltitude(baro.getAltitude() - rocketState.getGroundAltitude());

  Serial.println("set altitude");
  //delay(1000);
  //delay(0000);

  initSim();

  rocketStatus.use_lora = true;

  /*for (int i = 0; i < (int)(300.0/STEP_TIME*1000); i++){
    rocketState.stepTime();
  }*/


  /*while (1){

    //runTestSim();
  }*/
  // delay(10000000000000);
  // exit(0);
}

void loop()
{
  //Serial.println("beginning loop");
  rocketStatus.updateTime();
 // Serial.print("time loop");
  readSensors();
 // Serial.println("readSensors");
 // readSerial();

  // Serial.print(rocketState.getAZ());

  // Serial.println("readSensors complete");
  /*dt = (float)(micros())/1000000.0f - t;
  rocketState.delta_t = dt;
  rocketState.time = (float)micros()/1000000.0f;
*/
  //rocketState.updateDeltaT();
  if (rocketState.flightPhase != PAD && rocketState.flightPhase != LAUNCH){
    rocketState.updateState();
  }


  // If time is past start time (time for the rocket to get set up)

  if (rocketState.flightPhase == PAD)
  {
   // readSerial();
   if (digitalRead(USE_LORA_PIN) == HIGH) // LoRa telemetry turned on
      rocketStatus.use_lora = true;
    Serial.println("here we go!");
    if (rocketStatus.t > LAUNCH_DELAY)
      rocketState.flightPhase = LAUNCH;
  }


  if (rocketState.flightPhase == LAUNCH)
    {                // While rocket flight phase is LAUNCH, continue to read sensors until moving.
      //readSensors(); // get sensor input
      //updateSim();
      rocketState.globalizeAcceleration();
      digitalWrite(BUZZER_PIN, HIGH);

      if ((((rocketStatus.t * 1000000) / (LOG_TIME_STEP * 1000000)) - ((rocketStatus.t_last * 1000000) / (LOG_TIME_STEP * 1000000))) >= 1)
      {
        rocketStatus.t_last = rocketStatus.t;

        logRocketState();
        sendRocketTelemetry();
        // logSimState();
      }

     //rocketState.updateDeltaT();
      //rocketState.updateState();
      //rocketStatus.updateTime();

     // simState.time = (float)micros() / 1000000.0f;

       Serial.println("flightphase pad");
      if (rocketState.getAZ() > TRIGGER_ACCEL)
      { // If launch is detected
       Serial.println("flighphase ignition");
        rocketState.flightPhase = IGNITION;
        rocketState.t_launch = rocketStatus.t;
        digitalWrite(BUZZER_PIN, LOW);
        rocketState.updateState();
        //rocketControl.deployBrake(0);
        
      }
      //rocketState.stepTime();
    }
  
  if (rocketState.flightPhase == IGNITION)
  {
    //Serial.println("flightphase ignition");
    //Serial.println(rocketState.time);
    rocketState.updateState();
    if ((rocketState.time) > BURN_TIME){
      rocketState.flightPhase = COAST;
      Serial.println("flightphase coast");
    }

  }
  if (rocketState.flightPhase == COAST){
    rocketState.updateState();

    // Needless to say, this bit could use some work.
    if (rocketStatus.apogee >= target_apogee)
    {
      //rocketControl.deployBrake(100);
    }
    else
    {
      //rocketControl.deployBrake(0);
    }
    // Serial.println("logging");
    

    if (((rocketStatus.t * 1000000) / (LOG_TIME_STEP * 1000000) - ((rocketStatus.t_last * 1000000) / (LOG_TIME_STEP * 1000000))) >= 1)
    {
      rocketStatus.t_last = rocketStatus.t;
       //Serial.println("logging");
      //Serial.println(rocketState.time);
      logRocketState();
      sendRocketTelemetry();
    }
    if (rocketState.time > rocketConfig.getMaxTime())
    {
      rocketState.flightPhase = LAND;
     // rocketControl.deployBrake(1);
      writeRocketStateLog();
      closeLogs();
      digitalWrite(BUZZER_PIN, HIGH);
     /* while(1){
        digitalWrite(BUZZER_PIN, HIGH);
        delay(1000);
        digitalWrite(BUZZER_PIN, LOW);
        delay(500);
      }*/
     // Serial.println("closed logs");
      exit(0);
    }
  }
  if (rocketState.flightPhase == LAND)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    return;
  }

  if (rocketState.flightPhase != PAD && rocketState.flightPhase != LAUNCH){
    updateSim();
  }

  rocketState.stepTime();
  rocketStatus.updateTime();
}

// READ SENSORS

float filter_dt;

void readSensors()
{

  /*if (!baro.conversionComplete())
  {*/

    bno055.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno055.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno055.getEvent(&mag, Adafruit_BNO055::VECTOR_MAGNETOMETER);

    ACC_X = accel.acceleration.x; // float, m/s2
    ACC_Y = accel.acceleration.y;
    ACC_Z = accel.acceleration.z;

    GYRO_X = gyro.gyro.x; // float, rad/s
    GYRO_Y = gyro.gyro.y;
    GYRO_Z = gyro.gyro.z;

    MAG_X = mag.magnetic.x;
    MAG_Y = mag.magnetic.y;
    MAG_Z = mag.magnetic.z;

    calibrateSensors();
  /*}

  else
  {*/
    // Updated every ~1 second (1 hz)
    // calibrated altitude data
    /*
    MPL_PRESSURE = baro.getLastConversionResults(MPL3115A2_PRESSURE); // float, hPa
    MPL_ALTI = baro.getLastConversionResults(MPL3115A2_ALTITUDE);     // float, m
    MPL_TEMP = baro.getLastConversionResults(MPL3115A2_TEMPERATURE);
    baro.startOneShot();
    

    rocketState.setBaroAltitude(MPL_ALTI);
    rocketState.setBaroPressure(MPL_PRESSURE);
    rocketState.setBaroTemperature(MPL_TEMP);
    */
   //Serial.println(rocketConfig.getPressure());
   //Serial.println(bmp_baro.readPressure());
   //Serial.println(bmp_baro.readAltitude(rocketConfig.getPressure()));
   rocketState.setBaroAltitude(bmp_baro.readAltitude(rocketConfig.getPressure()));
   rocketState.setBaroPressure(bmp_baro.readPressure());
   

    calibrateSensors();

 // }

  rocketState.setAX_Local(ACC_X);
  rocketState.setAY_Local(ACC_Y);
  rocketState.setAZ_Local(ACC_Z);

  // Use Madgwick filter to update sensor filter

  filter_dt = sensor_filter.deltatUpdate();

  sensor_filter.MadgwickUpdate(GYRO_X, GYRO_Y, GYRO_Z, ACC_X, ACC_Y, ACC_Z, MAG_X, MAG_Y, MAG_Z, filter_dt); // Use Madgwick filter to update sensor filter

  // Set RocketState quaternion to sensor_filter quaternion
  copyQuat = sensor_filter.getQuat();

  rocketState.setQuatW(copyQuat[0]);
  rocketState.setQuatX(copyQuat[1]);
  rocketState.setQuatY(copyQuat[2]);
  rocketState.setQuatZ(copyQuat[3]);
}

void brakeTest()
{
  Serial.println("brakeTest check 1");
  retractBrake();
  delay(500);
  deployBrake();
  delay(500);
  retractBrake();
  delay(500);
  Serial.println("brakeTest check 2");
}

void state::updateDeltaT()
{
  now = micros();
  delta_t = (float)((now - last_time) / 1000000.0f);
  last_time = now;
}

void state::stepTime()
{
  delay(STEP_TIME);
}

void status::updateTime(){
  t = (float)(micros())/1000000.0f;
}

void initPins(){
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(USE_LORA_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}