//  Simulation values (rad -> rad)
//  kP = -0.33; 
//  kI = -0.2;
//  kD = -0.1;
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

    servoX.writeMicroseconds(1520);
    servoZ.writeMicroseconds(1425);
}

float PID::getPosX() {
    return posX/25.;
}

float PID::getPosZ() {
    return posZ/25.;
}

void PID::control(float rX, float rZ, float aX, float aZ, float iX, float iZ) {
    rotX = rX;
    rotZ = rZ;
    angX = aX;
    angZ = aZ;
    intX = iX;
    intZ = iZ;
    
    posX = (kPX*angX + kIX*intX + kDX*rotX);
    posZ = (kPZ*angZ + kIZ*intZ + kDZ*rotZ);
    mag = pow(posX*posX + posZ*posZ, 0.5);

    if (mag > range) {
        posX = int(posX*range/mag);
        posZ = int(posZ*range/mag);
    }

    //writeMicroseconds is more precise: 0 deg -> 1000 us, 180 deg -> 2000 us



    servoX.writeMicroseconds(1520+posX);
    servoZ.writeMicroseconds(1425+posZ);
}
