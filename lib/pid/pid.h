#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <Servo.h>

void PID(Servo servoX, Servo servoZ, int16_t rotX[], int16_t rotZ[], float angX[], float angZ[], long tstep[], int dataSize);

#endif
