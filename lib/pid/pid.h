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
        float angX; // deg
        float angY; //radians
        float angZ; // deg
        float intX; // deg*s
        float intZ; // deg*s
        long tstep; // s
        int posX; //us - real X servo position
        int posZ; //us - real Z servo position
        int tPosX; //us - "true" X servo position
        int tPosZ; //us - "true" Z servo position
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
        float getTPosX();
        float getTPosZ();
        float getPosX();
        float getPosZ();
        void control(float aY, float rX, float rZ, float aX, float aZ, float iX, float iZ);
};
#endif
