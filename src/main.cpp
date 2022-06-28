#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>

Adafruit_BMP085 bmp;
float pressure, temperature, altitude, previousAltitude;

Servo servo;
int servoPin;

//modes: not ready, ready, ascent, recovery
void mode_1(), mode_2(), mode_3(), mode_4();
int mode = 1;
float lastTime, takeoffTime;
boolean ledState = false;
boolean gyroReliable = true; //gyro values stop being reliable after apogee() is called
boolean takeoff(), emergency(), apogee();

void calculate_IMU_error();

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

void gyro(), baro();
  
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  lastTime = millis();

  servoPin = 2;
  servo.attach(servoPin);
  servo.write(180);

  //Start the serial
  // Serial.begin(19200);
  // Serial.print("Serial communication estabilished");

  //MPU setup
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);
  calculate_IMU_error();
  // Serial.println("MPU connection successful");

  //BMP setup
  if (!bmp.begin()) {
	// Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  } // Serial.println("BMP connection successful");
  //Find ground level pressure
  pressure = 0;
  for(int i = 0; i < 10; i++) {
    pressure += bmp.readPressure();
  }
  pressure /= 10;

  delay(20);
}
  
void loop() {

  gyro();
  baro();

  switch (mode) {
    case 1:
      mode_1();
      break;
    case 2:
      mode_2();
      break;
    case 3:
      mode_3();
      break;
    case 4:
      mode_4();
      break;
    default:
      mode = 1;
  }
  
  // Debugging purposes:
  /*
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);

  Serial.print("Altitude: ");
  Serial.println(altitude);
*/
  // Serial.print("Mode: ");
  // Serial.println(mode);
  

}




void gyro() {
  //Start communication with MPU
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58)
  
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission();
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  //Error correction
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
}

void baro() {
  previousAltitude = altitude;
  altitude = bmp.readAltitude(pressure);
}




void calculate_IMU_error() {
  // Function to correct the error in MPU6050
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;

  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;

  // Print the error values on the Serial Monitor, debuging purposes only
  // Serial.print("AccErrorX: ");
  // Serial.println(AccErrorX);
  // Serial.print("AccErrorY: ");
  // Serial.println(AccErrorY);
  // Serial.print("GyroErrorX: ");
  // Serial.println(GyroErrorX);
  // Serial.print("GyroErrorY: ");
  // Serial.println(GyroErrorY);
  // Serial.print("GyroErrorZ: ");
  // Serial.println(GyroErrorZ);
}

//check consequent altitudes, if they keep increasing, return true
boolean takeoff() {
  for(int i = 0; i < 5; i++) {
    delay(200);

    gyro();

    baro();

    if((int) altitude < (int) previousAltitude) {
      return false;
    }
  }
  return true;
}

//checks consequent gyro angles
boolean emergency() {
  for(int i = 0; i < 5; i++) {
    delay(200);

    gyro();

    baro();

    if(roll > 90 || yaw > 90) {
      return true;
    }
  }
  return false;
}

//200ms, 5measurements, ints, no delay afterwards
boolean apogee() {
  gyroReliable = false;

  for(int i = 0; i < 5; i++) {
    delay(200);

    baro();

    if((int) altitude > (int) previousAltitude) {
      return false;
    }
  }
  return true;
}




//check the inclination, if it is less than 45°, switch to mode 2
//we care about roll and yaw, as the mpu will be turned upwards
void mode_1() {
  ledState = false;
  digitalWrite(LED_BUILTIN, ledState);
  if(abs(roll) < 45 && abs(yaw) < 45) {
    mode = 2;
  }
}

void mode_2() {
  if(currentTime - lastTime > 1000) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastTime = millis();
  }

  if(abs(roll) > 45 || abs(yaw) > 45) {
    mode = 1;
    return;
  }

  altitude = bmp.readAltitude(pressure);
  if(altitude >= 10 && takeoff()) {
    mode = 3;
    takeoffTime = millis();
    previousAltitude = altitude;
  }
}

void mode_3() {
  if(currentTime - lastTime > 100) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastTime = millis();
  }

  previousAltitude = altitude;
  altitude = bmp.readAltitude(pressure);

  //check for emergency deployment
  if(currentTime - takeoffTime > 15000 || (gyroReliable && (abs(roll) > 90 || abs(yaw) > 90) && emergency())) {
    mode = 4;
    return;
  }
  //check for apogee
  if(((int) altitude < (int) previousAltitude) && apogee()) {
    mode = 4;
    return;
  }

}

void mode_4() {
  ledState = true;
  digitalWrite(LED_BUILTIN, ledState);

  servo.write(90);
}