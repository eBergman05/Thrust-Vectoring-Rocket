#include "pid.h"

void PID(Servo servoX, Servo servoZ, int16_t rotX[], int16_t rotZ[], float angX[], float angZ[], int tstep[])
{
    int posX;
    int posZ;
  
    float kPX = -0.5;
    float kIX = -0.005;
    float kDX = -0.005;
    
    float kPZ = -0.5;
    float kIZ = -0.005;
    float kDZ = -0.005;
    
    float propX = angX[9];
    float inteX = 0;
    float deriX = rotX[9];
    
    float propZ = angZ[9];
    float inteZ = 0;
    float deriZ = rotZ[9];

    
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

    posX = 90 + kPX*propX + kIX*inteX + kDX*deriX;
    posZ = 90 - kPZ*propZ - kIZ*inteZ - kDZ*deriZ;

    servoX.write(posX);
    servoZ.write(posZ);
    
}
