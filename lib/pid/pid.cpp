#include "pid.h"

void PID(Servo servoX, Servo servoZ, int16_t rotX[], int16_t rotZ[], float angX[], float angZ[], long tstep[], int dataSize)
{
    int posX;
    int posZ;
    double mag;
  
    float kPX = -5;
    float kIX = -0.00;
    float kDX = -0.01;
    
    float kPZ = -5;
    float kIZ = -0.00;
    float kDZ = -0.01;
    
    float propX = angX[dataSize-1];
    float inteX = 0;
    float deriX = rotX[dataSize-1];
    
    float propZ = angZ[dataSize-1];
    float inteZ = 0;
    float deriZ = rotZ[dataSize-1];

    
    for(int i = 0; i < 100; i++)
    {
        inteX += angX[i]*tstep[i];
        inteZ += angZ[i]*tstep[i];
    }
    
    //averaging filters for P and D
    /*
    for(int i = 98; i > 95; i--)
    {
        propX += angX[i];
        propZ += angZ[i];
    }
    propX/=4;
    propZ/=4;

    for(int i = 98; i > 95; i--)
    {
        deriX += rotX[i];
        deriZ += rotZ[i];
    }
    deriX/=4;
    deriZ/=4;
    */

    posX = 5*(kPX*propX + kIX*inteX + kDX*deriX);
    posZ = 5*(kPZ*propZ + kIZ*inteZ + kDZ*deriZ);
    mag = pow(posX*posX + posZ*posZ,0.5);

    if (mag > 160) {
        posX = int(posX*160/mag);
        posZ = int(posZ*160/mag);
    }

    //writeMicroseconds is more precise: 0 deg -> 1000 us, 180 deg -> 2000 us

    servoX.writeMicroseconds(1500-posX);
    servoZ.writeMicroseconds(1500-posZ);
}