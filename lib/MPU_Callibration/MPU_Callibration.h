#ifndef MPU_CALLIBRATION_H
#define MPU_CALLIBRATION_H

#include <Arduino.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

void callibrateMPU(Servo servoX, Servo servoZ, MPU6050 accelgyro);
void meansensors(MPU6050 accelgyro);
void calibration(MPU6050 accelgyro);


#endif
