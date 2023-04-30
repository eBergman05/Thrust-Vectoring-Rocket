#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <Servo.h>
class PID {
    private:
        Servo servoX;
        Servo servoZ;
        int pinX;
        int pinZ;
        float rotX; // deg/s
        float rotZ; // deg/s
        float angX;   // deg
        float angZ;   // deg
        float intX;   // deg*s
        float intZ;   // deg*s
        long tstep;   // s
        int posX;
        int posZ;
        int range;
        double mag;
        float kPX;
        float kIX;
        float kDX;
        float kPZ;
        float kIZ;
        float kDZ;

    public:
        PID(Servo sX, Servo sZ, int pX, int pZ);
        float getPosX();
        float getPosZ();
        void control(float rX, float rZ, float aX, float aZ, float iX, float iZ);
};
#endif
