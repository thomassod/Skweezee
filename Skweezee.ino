/************************************************************
This is the Arduino Code for Skweezee technology
The 28 Skweezee data, 3 accelerometer, 3 gyroscope and 4 quaternion values are sent to the serialport.




*************************************************************/
#include <SparkFunMPU9250-DMP.h>
#include <Wire.h>
#define SerialPort Serial

// first define the functions of the digital I/O pins
// the following pins control the multiplexers (enable and select)
#define EN 13
#define S0_A 12
#define S1_A 11
#define S2_A 10
#define S0_B 9
#define S1_B 8
#define S2_B 7

MPU9250_DMP imu;

int val = 0; // variable to store the value read

long gyroX, gyroY, gyroZ; //raw data
float rotX, rotY, rotZ; //processed data (°/s)
int mappedAccelX, mappedAccelY, mappedAccelZ; //0-255 mapped data

long accelX, accelY, accelZ; //raw data
float gForceX, gForceY, gForceZ; //processed data (g)
int mappedGyroX, mappedGyroY, mappedGyroZ;//0-255 mapped data
int mappedquat0, mappedquat1, mappedquat2, mappedquat3;//0-255 mapped quaternions
double maxval = 0, minval = 0;


void setup() 
{
  SerialPort.begin(9600);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  Wire.begin();
  pinMode(EN, OUTPUT);
  pinMode(S0_A, OUTPUT);
  pinMode(S1_A, OUTPUT);
  pinMode(S2_A, OUTPUT);
  pinMode(S0_B, OUTPUT);
  pinMode(S1_B, OUTPUT);
  pinMode(S2_B, OUTPUT);
  setupMPU();
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              100); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
}

void loop() 
{

  //Skweezee part
  digitalWrite(EN, HIGH);
  writeSkweezeeData(); // 28 skweezee values
  // Now disable the multiplexer (not really necessary)
  digitalWrite(EN, LOW);
  
  //IMU part
  recordAccelRegisters();
  recordGyroRegisters();
  writeIMUdata(); // 3 accel values, 3 gyro values
  //writeIMUquat(); // 4 quaternion values
  
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      writeIMUquat();
    }
  }
  
  //2 flags for Unity
  Serial.write(10);
  Serial.write(11);
  delay(20);
  }

void writeIMUquat(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

  mappedquat0 = (q0 + 1) *128;
  mappedquat1 = (q1 + 1) *128;
  mappedquat2 = (q2 + 1) *128;
  mappedquat3 = (q3 + 1) *128;  

  Serial.write(mappedquat0);
  Serial.write(mappedquat1);
  Serial.write(mappedquat2);
  Serial.write(mappedquat0);
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0b00000000); //Setting the gyro to full scale +/- 500deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 4g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  //gForceX = gForceX - 0.03;//small correction factor
  
  gForceY = accelY / 16384.0; 
  
  gForceZ = accelZ / 16384.0;
  //gForceZ = gForceZ - 0.03;//small correction factor

  //map the 3 values to 0-255 for serial write
  mappedAccelX = (gForceX)*255 /4;
  mappedAccelY = (gForceY)*255 /4;
  mappedAccelZ = (gForceZ)*255 /4; 
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotX = rotX + 1.65;//small correction factor
  
  rotY = gyroY / 131.0; 
  rotY = rotY - 2.5;//small correction factor
  
  rotZ = gyroZ / 131.0;
  rotZ = rotZ + 0.9;//small correction factor

  //map the 3 values to 0-255 for serial write
  mappedGyroX = (rotX )*256 / 502;
  mappedGyroY = (rotY )*256 / 502;
  mappedGyroZ = (rotZ )*256 / 502;
}

void printIMU() {
  
  //print the 3 accelero values
  //Serial.println();
  
  Serial.print(gForceX);
  Serial.print(" ");
  Serial.print(gForceY);
  Serial.print(" ");
  Serial.print(gForceZ);
  Serial.print(" ");


  //print the 3 gyro values
  Serial.print(rotX);
  Serial.print(" ");
  Serial.print(rotY);
  Serial.print(" ");
  Serial.print(rotZ);
  Serial.print(" ");
 // Serial.println();
}

void writeIMUdata(){

  //send the 3 mapped accelero values
  Serial.write(mappedAccelX);
  Serial.write(mappedAccelY);
  Serial.write(mappedAccelZ);
  
  //send the 3 mapped gyro values
  Serial.write(mappedGyroX);
  Serial.write(mappedGyroY);
  Serial.write(mappedGyroZ);
}


void writeSkweezeeData(){
  
  // Loop through all the relevant combinations of the multiplexers
  for (int i=0; i<8; i++) {
     for (int j=i+1; j<8; j++) {
        digitalWrite(S2_A, bitRead(i,2) );
        digitalWrite(S1_A, bitRead(i,1) );
        digitalWrite(S0_A, bitRead(i,0) );
        digitalWrite(S2_B, bitRead(j,2) );
        digitalWrite(S1_B, bitRead(j,1) );
        digitalWrite(S0_B, bitRead(j,0) );
        //delayMicroseconds(30); // time needed to go from L to H
        val = analogRead(0)>>2;
        Serial.write(val);
     } 
  }
}



