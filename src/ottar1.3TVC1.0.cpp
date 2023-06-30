//@author Ethan Bergman
//@version 2.0
//@date 4.20.2023
//
//Takes in data from MPU6050 (6 DOF gyro/accel) and uses a PID feedback loop 
//to manipulate TVC mount accordingly
//
//Uses PID class to create a controller object and passes in control parameters
//at around 600Hz
//
#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <list>
#include <iostream>
#include <algorithm>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "pid.h"
#include "MPU_Callibration.h"
#include "Adafruit_BMP280.h"
#include "UI.h"


//MPU6050 accelgyro(0x69); // <-- use for AD0 high
MPU6050 accelgyro;

//barometer&altimeter
Adafruit_BMP280 bmp;

//gyro data variables
int16_t aX, aY, aZ;
int16_t gX, gY, gZ;

//two times used to determine tstep
long time0;//us
long time1;//us
long tstep;//us
float tstepS;//s

//data lists
EXTMEM float uTimeList[5000];
EXTMEM float anglXList[5000];
EXTMEM float anglZList[5000];
EXTMEM float gyroXList[5000];
EXTMEM float gyroZList[5000];
EXTMEM float sPosXList[5000];
EXTMEM float sPosZList[5000];
EXTMEM float altitList[5000];
int count;
int maxCount;

//raw accelgyro data
float gyroXData;//deg/s - relative X rotation
float gyroYData;//radians/s - Y rotation
float gyroZData;//deg/s - relative Z rotation

//"true" angular position and velocity data
float angleX;//deg - true X angle
float angleY;//radians - Y angle
float angleZ;//deg - true Z angle
float rotatX;//deg/s - true X rotation
float rotatZ;//deg/s - true Z rotation

//integral of the position-time graph
float intX;//deg*s - integral of X angle
float intZ;//deg*s - integral of Z angle

float pressure;//barometer currently not working
float altitude;//barometer currently not working

//ejection charge trigger
int liftoffTime;//ms

//define servos
Servo servoX;
Servo servoZ; 

//starting servo position, more accurate zeroes in PID class
int pos = 90; 

//define pinout
#define LEDR 6
#define LEDG 7
#define LEDB 8
#define buzzer 37
#define solunoid 33

//UI class object
UI blinky(LEDR, LEDG, LEDB, buzzer);

//PID controller
PID TVC(servoX, servoZ, 4, 5);

File myFile;

void setup() {
    //charge capacitors
    delay(500);

            
    //setup pins
    pinMode(LEDR, OUTPUT);
    pinMode(LEDB, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(buzzer, OUTPUT);
    pinMode(solunoid, OUTPUT);

    blinky.startupNoise();
    delay(500);
    //initialize I2C communication
    Wire.begin();

    accelgyro.initialize();

    //to verify connection:
    if (!accelgyro.testConnection()){
        blinky.failNoise();
        while(1);
    }

    //set resolution level (see MPU6050.ccp for details)
    accelgyro.setFullScaleGyroRange(2); //32.8 LSB/deg/sec
    //apply Low Pass Filter (see MPU6050.ccp for details)
    accelgyro.setDLPFMode(6);

    //test SD card and file existance
    if (!SD.begin(BUILTIN_SDCARD)) {
        blinky.failNoise();
        while(1);
    }
    myFile = SD.open("data.txt", FILE_WRITE);
    // if the file opened okay, write to it:
    if (myFile) {
        myFile.println("Data: time, angleX, angleZ, gX, gZ, iX, iZ, posX, posZ");
        myFile.close();
    } else {
        blinky.failNoise();
    }

    // re-open the file for reading:
    myFile = SD.open("data.txt");
    if (myFile) {
        Serial.println("data.txt:");

        // read from the file until there's nothing else in it:
        while (myFile.available()) 
        {
            Serial.write(myFile.read());
        }
        // close the file:
        myFile.close();
        blinky.successNoise();
    } else {
        // if the file didn't open, print an error:
        Serial.println("error opening data.txt");
        blinky.failNoise();
    }

    callibrateMPU(accelgyro);
    /*  
    accelgyro.setXAccelOffset(-1915);
    accelgyro.setYAccelOffset(2399);
    accelgyro.setZAccelOffset(1240);
    accelgyro.setXGyroOffset(-90);
    accelgyro.setYGyroOffset(43);
    accelgyro.setZGyroOffset(27);
    */

    accelgyro.getMotion6(&aX, &aY, &aZ, &gX, &gY, &gZ);

    gyroXData = 0;
    gyroYData = 0;
    gyroZData = 0;
    angleX = 0;
    angleY = 0;
    angleZ = 0;
    intX = 0;
    intZ = 0;

    TVC.control(angleY, gyroXData, gyroZData, angleX, angleZ, intX, intZ);

    delay(500);
    blinky.armedNoise();
    delay(500);

    //100 second countdown -- start apple timer for 1:40 between the first two beeps
    //ignition begins at T-00:01.50
    //last long beep is  T-00:01.00 to T-00:00.00
    //liftoff occurs at  T-00:00.00
    //ejection occurs at T+00:05.00 or if an abort is called from the OTTAr or mission control
    blinky.countDown();
    delay(70000);
    blinky.countDown();

    //throw away first 100 measurements - off for some reason
    //rocket inactive for first 155ms
    for(int n = 0; n < 100; n++) {

        accelgyro.getMotion6(&aX, &aY, &aZ, &gX, &gY, &gZ);  

        time0 = time1;
        time1 = micros();
        tstep = time1 - time0;
        tstepS = tstep/1000000.;

        gyroXData = gX/32.800;
        gyroYData = (gY/32.800) * 0.0174533; //convert degrees to radians
        gyroZData = gZ/32.800;
        angleY -= gyroYData*tstepS;
        rotatX = (gyroXData * cos(angleY)) + (gyroZData * sin(angleY));
        rotatZ = (gyroZData * cos(angleY)) + (gyroXData * sin(angleY));
        angleX += tstepS*rotatX;
        angleZ += tstepS*rotatZ;
        intX += angleX*tstepS;
        intZ += angleZ*tstepS;
    }

    //set cumulatives back to zero
    angleX = 0;
    angleY = 0;
    angleZ = 0;
    intX = 0;
    intZ = 0;

    count = 0;
    liftoffTime = millis();
    time1 = micros();

    //feedback loop
    while(1) {
        //reduce the amount of times the checks are called
        for (int i = 0; i < 10; i++) {
            //read motion data
            accelgyro.getMotion6(&aX, &aY, &aZ, &gX, &gY, &gZ);   

            //record time in us, convert tstep to s
            time0 = time1;
            time1 = micros();
            tstep = time1 - time0;
            tstepS = tstep/1000000.;

            //pass in data, convert to deg/s, deg, and deg*s
            gyroXData = gX/32.800;
            gyroYData = (gY/32.800) * 0.0174533; //convert degrees to radians
            gyroZData = gZ/32.800;
            angleY -= gyroYData*tstepS;
            rotatX = (gyroXData * cos(angleY)) - (gyroZData * sin(angleY));
            rotatZ = (gyroZData * cos(angleY)) + (gyroXData * sin(angleY));
            angleX += tstepS*rotatX;
            angleZ += tstepS*rotatZ;
            intX += angleX*tstepS;
            intZ += angleZ*tstepS;

            //feedback loop
            TVC.control(angleY, rotatX, rotatZ, angleX, angleZ, intX, intZ);
            
            //record every other data point set
            if(i%2 == 0 && count < 5000) {
                uTimeList[count] = tstep;
                anglXList[count] = angleX;
                anglZList[count] = angleZ;
                gyroXList[count] = gyroYData/0.0174533;
                gyroZList[count] = angleY/0.0174533;
                sPosXList[count] = TVC.getPosX();
                sPosZList[count] = TVC.getPosZ();

                count++;
            }
        }

        //check if the rocket has reached apogee or turned too far.
        //commented out for static tests with no ejection charge
        //5s to apogee
        if ((millis()-liftoffTime > 7000) || (abs(angleX) > 45) || (abs(angleZ) > 45)) {

            //open valve for 0.5s
            delay(100);
            digitalWrite(solunoid, HIGH);
            delay(500);
            digitalWrite(solunoid, LOW);

            //indicate activity
            digitalWrite(LEDB, HIGH);

            maxCount = count;
            for(count = 0; count < maxCount; count ++) {
                myFile = SD.open("data.txt", FILE_WRITE);
                myFile.print(uTimeList[count]);
                myFile.print("\t");
                myFile.print(anglXList[count]);
                myFile.print("\t");
                myFile.print(anglZList[count]);
                myFile.print("\t");
                myFile.print(gyroXList[count]);
                myFile.print("\t");
                myFile.print(gyroZList[count]);
                myFile.print("\t");
                myFile.print(sPosXList[count]);
                myFile.print("\t");
                myFile.print(sPosZList[count]);
                myFile.println();
                myFile.close();
            }
            
            myFile = SD.open("data.txt", FILE_WRITE);
            myFile.println(maxCount);
            myFile.close();

            digitalWrite(LEDB, LOW);
            //turn off activity light and begin ping
            blinky.completeNoise();
        }
    }
}
void loop(){}