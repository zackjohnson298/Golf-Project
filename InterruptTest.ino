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
// Define interrupt pin
#define INT_PIN 22

Adafruit_LSM6DSOX imu;

// Define Constants
const int DATAPOINTS = 2*3333;
const int POINTS_PER_PACKET = 12;
const int PACKET_SIZE = 6;
const int ARRAY_SIZE = (int)(DATAPOINTS*PACKET_SIZE);
const int OUTPUT_PACKET_SIZE = 2*POINTS_PER_PACKET*PACKET_SIZE + 4; 

// Define output arrays
uint16_t outputArray[ARRAY_SIZE]; 
uint8_t outputPacket[OUTPUT_PACKET_SIZE];
//int8_t finalPacket[OUTPUT_PACKET_SIZE] = { '$', 0x02, '$','$','$','$','$','$','$','$','$','$','$','$', '\r', '\n' };

// Define Global Variables
char success = '1';
char fail = '0';
int points = 0;

void SendIMUData(int16_t* outputArray);
void IRAM_ATTR GetDataISR() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp);
  outputArray[points] = (uint16_t)accel.acceleration.x;
  outputArray[points + 1] = (uint16_t)accel.acceleration.y;
  outputArray[points + 2] = (uint16_t)accel.acceleration.z;
  outputArray[points + 3] = (uint16_t)gyro.gyro.x;
  outputArray[points + 4] = (uint16_t)gyro.gyro.y;
  outputArray[points + 5] = (uint16_t)gyro.gyro.z;
  points += PACKET_SIZE;
  Serial.println(points);
}

// Function to Initialize output buffer packet
void InitializeOutputPacket(uint8_t* outputPacket) {
  
  outputPacket[0] = '$';
  outputPacket[1] = 0x02;
  int ii;
  for (ii = 2; ii < (OUTPUT_PACKET_SIZE - 2); ii++){
    outputPacket[ii] = 0;
  }
  outputPacket[OUTPUT_PACKET_SIZE - 2] = '\r';
  outputPacket[OUTPUT_PACKET_SIZE - 1] = '\n';
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.begin("esp32");
  delay(20);

  Serial.println("Interrupt-Driven Data Collection test!");
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
  imu.setAccelDataRate(LSM6DS_RATE_12_5_HZ);//3_33K_HZ);
  imu.setGyroDataRate(LSM6DS_RATE_12_5_HZ);//3_33K_HZ);

  // Initialize Output Packet
  InitializeOutputPacket(outputPacket);

  Serial.println("IMU Initialized");
  Serial.println("");
  Serial.println("Send any character to begin");
  while (SerialBT.available() && SerialBT.read()); // empty buffer
  while (!SerialBT.available());                   // wait for data
  while (SerialBT.available() && SerialBT.read()); // empty buffer again
  delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Collecting Data...");
  imu.configIntOutputs(1,1);
  imu.configInt1(0,0,1,0,0);
  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(INT_PIN, GetDataISR, FALLING);
  int t1 = millis();
  Serial.println("hello");
  while (points < ARRAY_SIZE) {}
  detachInterrupt(INT_PIN);
  int t2 = millis();
  Serial.println("Data Collection Complete");
  Serial.print("Collected ");
  Serial.print((t2-t1)/1000);
  Serial.println(" Seconds of Data."); 
  Serial.println("Transmitting data...");
  SendIMUData(outputArray);
  Serial.println("Data Transmission Complete.");
}

void SendIMUData(uint16_t *outputArray) {
  int ii;
  int jj;
  int maxAttempts = 20;
  for (ii = 0; ii < (ARRAY_SIZE - PACKET_SIZE*POINTS_PER_PACKET); ii += PACKET_SIZE*POINTS_PER_PACKET) {
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
}

void loop() {
  // put your main code here, to run repeatedly:
}
