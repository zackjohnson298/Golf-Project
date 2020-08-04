#include <TinyPICO.h>
#include <Adafruit_LSM6DSOX.h>
#include <SPI.h>
#include <time.h>
#include "BluetoothSerial.h"
   
BluetoothSerial SerialBT;
// For SPI mode, we need a CS pin
#define LSM_CS 5
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 18
#define LSM_MISO 19
#define LSM_MOSI 23

Adafruit_LSM6DSOX imu;

const int PERIOD = 4; // Seconds
const int FREQUENCY = 1800; // Hz
const float TIMESTEP  = 1000000.0/(float)FREQUENCY; // Microseconds
const int DATAPOINTS = PERIOD*FREQUENCY;
const int POINTS_PER_PACKET = 12;
const int PACKET_SIZE = 6;
const int ARRAY_SIZE = (int)(DATAPOINTS*PACKET_SIZE);
const int OUTPUT_PACKET_SIZE = 2*POINTS_PER_PACKET*PACKET_SIZE + 4; 
uint16_t outputArray[ARRAY_SIZE]; 
uint8_t outputPacket[OUTPUT_PACKET_SIZE];
int oldestPoint = 0;
float xOffset = 0.0;
float yOffset = 0.0;
float zOffset = 0.0;

char success = '1';
char fail = '0';

void SendIMUData();
void GetIMUData();
void GetSwingData();
void CalibrateGyro();

// Function to Initialize output buffer packet
void InitializeOutputPacket() {
  
  outputPacket[0] = '$';
  outputPacket[1] = 0x02;
  int ii;
  for (ii = 2; ii < (OUTPUT_PACKET_SIZE - 2); ii++){
    outputPacket[ii] = 0;
  }
  outputPacket[OUTPUT_PACKET_SIZE - 2] = '\r';
  outputPacket[OUTPUT_PACKET_SIZE - 1] = '\n';
}

// 

void setup() {
  // put your setup code here, to run once:

  // Initialize Output Packet
  InitializeOutputPacket();
  
  Serial.begin(115200);
  SerialBT.begin("esp32");
  delay(20);
  Serial.println("Timestep Test!");
  if (!imu.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("LSM6DSOX Found!");

  // Set IMU Full Scale Values
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  // Set IMU Data Rates
  imu.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  imu.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
  delay(50);
  CalibrateGyro();
  
  Serial.println("IMU Initialized");
  Serial.println("");
  Serial.println("Send any character to begin");
  while (SerialBT.available() && SerialBT.read()); // empty buffer
  while (!SerialBT.available());                   // wait for data
  while (SerialBT.available() && SerialBT.read()); // empty buffer again
  delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Ready to record Swings!");
  
  
//  Serial.println("Beginning data measurement");
//  GetIMUData();
//  Serial.println("Transmitting Data");
//  SendIMUData();
//  Serial.println("Finished Sending Data");
}

void CalibrateGyro(void) {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  float xSum = 0.0;
  float ySum = 0.0;
  float zSum = 0.0;
  int ii = 0;
  int lastMicros = micros();
  while (ii < 100) {
    if ((micros() - lastMicros) >= TIMESTEP) {
      lastMicros = micros();
      imu.getEvent(&accel, &gyro, &temp);
      xSum += gyro.gyro.x;
      ySum += gyro.gyro.y;
      zSum += gyro.gyro.z;
      ii++;
    }
  }
  xOffset = xSum/100.0;
  yOffset = ySum/100.0;
  zOffset = zSum/100.0;
}

// Constantly records data (fifo buffer)
// Waits until swing is detected (based on gyro mags)
// Loop continues for specified swing duration
// Loop exits
void GetSwingData(void) {
  bool stationary = 0;
  bool swinging = 0;
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  float gyroMag = 0.0;  
  float gyroThresh = 100.0;
  int timeThresh = 600000;
  
//  Serial.println(xOffset);
//  Serial.println(yOffset);
//  Serial.println(zOffset);
  
  int lastStatTime = micros();
  bool record = true;
  int swingStart = 0;
  int lastMicros = micros();
  int swingDuration = 1750000;
  int Time = micros();
  oldestPoint = 0;
  float gyroFS = 2000.0;
  float gyroX = 0.0;
  float gyroY = 0.0;
  float gyroZ = 0.0;
  
  while (record) {
    Time = micros();
    if ((Time - lastMicros) >= TIMESTEP) { 
      lastMicros = Time;
      imu.getEvent(&accel, &gyro, &temp);
      outputArray[oldestPoint] = (uint16_t)accel.acceleration.x;
      outputArray[oldestPoint + 1] = (uint16_t)accel.acceleration.y;
      outputArray[oldestPoint + 2] = (uint16_t)accel.acceleration.z;
      outputArray[oldestPoint + 3] = (uint16_t)gyro.gyro.x;
      outputArray[oldestPoint + 4] = (uint16_t)gyro.gyro.y;
      outputArray[oldestPoint + 5] = (uint16_t)gyro.gyro.z;

      // Checking if oldestPoint is at the end of array, meaning it needs to go back to the start
      if (oldestPoint >= (ARRAY_SIZE - PACKET_SIZE)) {
        oldestPoint = 0;
      } else {
        oldestPoint += PACKET_SIZE;
      }

      gyroX = gyro.gyro.x*gyroFS/(2.0*16384.0);
      gyroY = gyro.gyro.y*gyroFS/(2.0*16384.0);
      gyroZ = gyro.gyro.z*gyroFS/(2.0*16384.0);

      if (gyroX > gyroFS) {
        gyroX = gyroX - 2.0*gyroFS;
      }
      if (gyroY > gyroFS) {
        gyroY = gyroY - 2.0*gyroFS;
      }
      if (gyroZ > gyroFS) {
        gyroZ = gyroZ - 2.0*gyroFS;
      }
            
      gyroMag = sqrt((gyroX - xOffset)*(gyroX - xOffset) + (gyroY - yOffset)*(gyroY - yOffset) + (gyroZ - zOffset)*(gyroZ - zOffset));
//      Serial.print(gyroX);
//      Serial.print(" ");
//      Serial.print(gyroY);
//      Serial.print(" ");
//      Serial.println(gyroZ);
//      Serial.println(gyroMag);
      if (gyroMag > gyroThresh) {
        stationary = false;
        if (!swinging && ((Time - lastStatTime) > timeThresh)) {
          swinging = true;
          swingStart = Time - timeThresh;
        } 
      } else {
        stationary = true;
        lastStatTime = Time;
      }
      
      // Making sure the full output packet is recorded before exiting loop
      if (swinging && ((Time - swingStart) >= swingDuration) && (oldestPoint % PACKET_SIZE*POINTS_PER_PACKET == 0)) {
        record = false;
      }
    }
  }
}

void SendIMUData() {
  int ii;
  int jj;
  int maxAttempts = 20;
  int t1 = micros();
  for (ii = oldestPoint; ii < (ARRAY_SIZE - PACKET_SIZE*POINTS_PER_PACKET); ii += PACKET_SIZE*POINTS_PER_PACKET) {
    int input;
    int attempts = 1;
    for (jj = 0; jj < PACKET_SIZE*POINTS_PER_PACKET;jj++){
      outputPacket[2*jj+2] = (uint8_t)((outputArray[ii+jj] & 0xFF00) >> 8);
      outputPacket[2*jj+3] = (uint8_t)(outputArray[ii+jj] & 0x00FF);
    }
    SerialBT.write(outputPacket,OUTPUT_PACKET_SIZE);
    
    while (!SerialBT.available()){}
    while ((SerialBT.read() == fail) && attempts < maxAttempts) {
      while (SerialBT.available() && SerialBT.read()) {}; // empty buffer
      SerialBT.write(outputPacket,OUTPUT_PACKET_SIZE);
      attempts++;
      while (!SerialBT.available()){}
    }
  }
  for (ii = 0; ii < (oldestPoint - PACKET_SIZE*POINTS_PER_PACKET); ii += PACKET_SIZE*POINTS_PER_PACKET) {
    int input;
    int attempts = 1;
    for (jj = 0; jj < PACKET_SIZE*POINTS_PER_PACKET;jj++){
      outputPacket[2*jj+2] = (uint8_t)((outputArray[ii+jj] & 0xFF00) >> 8);
      outputPacket[2*jj+3] = (uint8_t)(outputArray[ii+jj] & 0x00FF);
    }
    SerialBT.write(outputPacket,OUTPUT_PACKET_SIZE);
    
    while (!SerialBT.available()){}
    while ((SerialBT.read() == fail) && attempts < maxAttempts) {
      while (SerialBT.available() && SerialBT.read()) {}; // empty buffer
      SerialBT.write(outputPacket,OUTPUT_PACKET_SIZE);
      attempts++;
      while (!SerialBT.available()){}
    }
  }
  int t2 = micros();
  Serial.println("Done Transmitting");
  Serial.print("Elapsed Time: ");
  Serial.println((float)(t2 - t1)/1000000.0);
}

void GetIMUData(uint16_t *outputArray) {
  int ii = 0;
  int t1 = micros();
  int lastMicros = micros();
  while (ii < ARRAY_SIZE) {
    if ((micros() - lastMicros) >= TIMESTEP) {
      lastMicros = micros();
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t temp;
      imu.getEvent(&accel, &gyro, &temp);
      outputArray[ii] = (uint16_t)accel.acceleration.x;
      outputArray[ii + 1] = (uint16_t)accel.acceleration.y;
      outputArray[ii + 2] = (uint16_t)accel.acceleration.z;
      outputArray[ii + 3] = (uint16_t)gyro.gyro.x;
      outputArray[ii + 4] = (uint16_t)gyro.gyro.y;
      outputArray[ii + 5] = (uint16_t)gyro.gyro.z;
      ii += PACKET_SIZE;
    }
  }
  int t2 = micros();
  Serial.println("Measurement Complete");
  Serial.print("Expected Measurement Time: ");
  Serial.println(PERIOD);
  Serial.print("Actual Measurement Time: ");
  Serial.println((float)((t2 - t1)/1000000));
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Waiting for swing detection");
  GetSwingData();
  Serial.println("Swing Detected");
  Serial.println("Transmitting Data");
  SendIMUData();
  Serial.println("Send any character to record new swing.");
  while (SerialBT.available() && SerialBT.read()); // empty buffer
  while (!SerialBT.available());                   // wait for data
  while (SerialBT.available() && SerialBT.read()); // empty buffer again
  delay(10); // will pause Zero, Leonardo, etc until serial console opens

}
