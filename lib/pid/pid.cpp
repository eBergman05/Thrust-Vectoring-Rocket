//  Simulation values (rad -> rad)
//  kP = -0.33; 
//  kI = -0.2;
//  kD = -0.1;
//
//  Using an F22J
//  For hold down test using an F15 multiply all values by 1.34
//
//  Real values (deg -> us)
//  deg * 17.86 = us
//  kP = -5.89
//  kI = -3.57
//  kD = -1.79
//  all inputs are in degrees, deg/s, or deg*s
#include "pid.h"


//object definition
PID::PID(Servo sX, Servo sZ, int pX, int pZ)
    :servoX(sX), 
    servoZ(sZ), 
    pinX(pX),
    pinZ(pZ),
    rotX(), 
    rotZ(), 
    angX(), 
    angY(),
    angZ(), 
    intX(), 
    intZ(), 
    tstep(), 
    posX(), 
    posZ(), 
    range(125), 
    mag(), 
    kPX(-5.89), 
    kIX(-3.57),
    kDX(-1.79), 
    kPZ(-5.89), 
    kIZ(-3.57),
    kDZ(-1.79) 
{
    servoX.attach(pinX);
    servoZ.attach(pinZ);

    servoX.writeMicroseconds(1500);
    servoZ.writeMicroseconds(1375);
}

float PID::getTPosX() {
    return tPosX/17.86;
}

float PID::getTPosZ() {
    return tPosZ/17.86;
}

float PID::getPosX() {
    return posX/17.86;
}

float PID::getPosZ() {
    return posZ/17.86;
}

void PID::control(float aY, float rX, float rZ, float aX, float aZ, float iX, float iZ) {
    rotX = rX;
    rotZ = rZ;
    angX = aX;
    angY = aY;
    angZ = aZ;
    intX = iX;
    intZ = iZ;
    
    tPosX = (kPX*angX + kIX*intX + kDX*rotX);
    tPosZ = (kPZ*angZ + kIZ*intZ + kDZ*rotZ);
    mag = pow(tPosX*tPosX + tPosZ*tPosZ, 0.5);

    if (mag > range) {
        tPosX = int(tPosX*range/mag);
        tPosZ = int(tPosZ*range/mag);
    }

    posX = (tPosX * cos(angY)) + (tPosZ * sin(angY));
    posZ = (tPosZ * cos(angY)) - (tPosX * sin(angY));

    servoX.writeMicroseconds(1500+posX);
    servoZ.writeMicroseconds(1375+posZ);
}
