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
        double rotX; // deg/s
        double rotZ; // deg/s
        double angX; // deg
        double angY; //radians
        double angZ; // deg
        double intX; // deg*s
        double intZ; // deg*s
        long tstep; // s
        int posX; //us - real X servo position
        int posZ; //us - real Z servo position
        int tPosX; //us - "true" X servo position
        int tPosZ; //us - "true" Z servo position
        int range;
        double mag;
        double kPX;
        double kIX;
        double kDX;
        double kPZ;
        double kIZ;
        double kDZ;

    public:
        PID(Servo sX, Servo sZ, int pX, int pZ);
        double getTPosX();
        double getTPosZ();
        double getPosX();
        double getPosZ();
        void control(double aY, double rX, double rZ, double aX, double aZ, double iX, double iZ);
};
#endif
