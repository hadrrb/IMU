/*
Advanced_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"
#include "SensorFusion.h"

SF fusion;
float deltat, pitch, yaw, roll, ax, ay, az;
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
    status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
Serial.println("Gyro calibration");
  status = IMU.calibrateGyro();
    if (status < 0) {
    Serial.println("Gyro calibration failed");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
 Serial.println("Accel calibration");
  status = IMU.calibrateAccel();
      if (status < 0) {
    Serial.println("Accel calibration failed");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
//  Serial.println("Mag calibration");
//  status = IMU.calibrateMag();
//      if (status < 0) {
//    Serial.println("Mag calibration failed");
//    Serial.print("Status: ");
//    Serial.println(status);
//    while(1) {}
//  }
}

void loop() {
  // read the sensor
  IMU.readSensor();
  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  //choose only one of these two:
  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  fusion.MadgwickUpdate(IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(), ax, ay, az, IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT(), deltat);  //else use the magwick, it is slower but more accurate

  roll = fusion.getRollRadians();
  pitch = fusion.getPitchRadians();
  yaw = fusion.getYawRadians();
  Serial.println(String(pitch)+":"+String(roll)+":"+String(yaw)+":"+String(ax)+":"+String(ay)+":"+String(az)+":"+String(deltat));
}
