//@author Ethan Bergman
//@version 2.0
//@date 4.20.2023
//
//Takes in data from MPU6050 (6 DOF gyro/accel) and uses a PID feedback loop 
//to manipulate TVC mount accordingly
//
//Uses PID class to create a controller object and passes in control parameters
//at around 250Hz
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
std::list<float>uTimeList;
std::list<float>anglXList;
std::list<float>anglZList;
std::list<float>gyroXList;
std::list<float>gyroZList;
std::list<float>sPosXList;
std::list<float>sPosZList;

//rotational data
float gyroXData;//deg/s
float gyroYData;//deg/s
float gyroZData;//deg/s

//angular position data
float angleX;//deg;
float angleZ;//deg;

//integral of the position-time graph
float intX;//deg*s
float intZ;//deg*s

float pressure;
float altitude;

//ejection charge triggers
bool liftoff = false;
bool apogee = false;
int16_t yCache;
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
    delay(2500);

    //setup pins
    pinMode(LEDR, OUTPUT);
    pinMode(LEDB, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(buzzer, OUTPUT);
    pinMode(solunoid, OUTPUT);

    blinky.startupNoise();

    //initialize I2C communication
    Wire.begin();

    accelgyro.initialize();

    //to verify connection:
    //accelgyro.testConnection();

    //set resolution level (see MPU6050.ccp for details)
    accelgyro.setFullScaleGyroRange(2); //32.8 LSB/deg/sec
    //apply Low Pass Filter (see MPU6050.ccp for details)
    accelgyro.setDLPFMode(6);

/*
    //attach to servos to pins 4 and 5
    servoX.attach(4);
    servoZ.attach(5);

    servoX.write(pos);
    servoZ.write(pos);
*/

    //test SD card and file existance
    if (!SD.begin(BUILTIN_SDCARD)) 
    {
        blinky.failNoise();
        while(1);
    }
    myFile = SD.open("data.txt", FILE_WRITE);
    // if the file opened okay, write to it:
    if (myFile) 
    {
        myFile.println("Data: time, angleX, angleZ, gX, gZ, iX, iZ, posX, posZ");
        myFile.close();
        blinky.successNoise();
    } else {
        blinky.failNoise();
    }

/*
    // re-open the file for reading:
    myFile = SD.open("data.txt");
    if (myFile) 
    {
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
*/ 

    callibrateMPU(accelgyro);
    accelgyro.getMotion6(&aX, &aY, &aZ, &gX, &gY, &gZ);
    yCache = aY; //to check takeoff

    gyroXData = 0;
    gyroZData = 0;
    angleX = 0;
    angleZ = 0;
    intX = 0;
    intZ = 0;

    TVC.control(gyroXData, gyroZData, angleX, angleZ, intX, intZ);

    blinky.armedNoise();
}

void loop() {
    //throw away first 100 measurements - off for some reason
    for(int n = 0; n < 100; n++) {

        accelgyro.getMotion6(&aX, &aY, &aZ, &gX, &gY, &gZ);  

        time0 = time1;
        time1 = micros();
        tstep = time1 - time0;
        tstepS = tstep/1000000.;

        gyroXData = gX/32.800;
        gyroZData = gZ/32.800;
        angleX += tstepS*gyroXData;
        angleZ += tstepS*gyroZData;
        intX += angleX*tstepS;
        intZ += angleZ*tstepS;
    }

    //set cumulatives back to zero
    angleX = 0;
    angleZ = 0;
    intX = 0;
    intZ = 0;


    //for static fire - time ignition
    blinky.ignitionNoise();
    liftoff = true;
    liftoffTime = millis();
    time1 = micros();

    myFile = SD.open("data.txt", FILE_WRITE);
    myFile.println("<<IGNITION>>");
    myFile.close();

    //feedback loop one: don't starting adding integral until liftoff
    while (!liftoff) {
        accelgyro.getMotion6(&aX, &aY, &aZ, &gX, &gY, &gZ);        
            
            //record time in us, convert tstep to s
            time0 = time1;
            time1 = micros();
            tstep = time1 - time0;
            tstepS = tstep/1000000.;
            
            //pass in data, convert to deg/s, deg, and deg*s
            gyroXData = gX/32.800;
            gyroZData = gZ/32.800;
            angleX += tstepS*gyroXData;
            angleZ += tstepS*gyroZData;

            //feedback loop
            TVC.control(gyroXData, gyroZData, angleX, angleZ, intX, intZ);

            if ((abs(aY - yCache) > 1000)){
                liftoff = true;
                liftoffTime = millis();
            }

            //write data to SD card
            myFile = SD.open("data.txt", FILE_WRITE);
            myFile.print(time1);
            myFile.print("\t");
            myFile.print(tstep);
            myFile.print("\t");
            myFile.print(angleX);
            myFile.print("\t");
            myFile.print(angleZ);
            myFile.print("\t");
            myFile.print(gyroXData);
            myFile.print("\t");
            myFile.print(gyroZData);
            myFile.print("\t");
            myFile.print(intX);
            myFile.print("\t");
            myFile.print(intZ);
            myFile.print("\t");
            myFile.print(TVC.getPosX());
            myFile.print("\t");
            myFile.print(TVC.getPosZ());
            myFile.println();
            if (liftoff) myFile.println("<<LIFTOFF>>");
            myFile.close();
    }

    //feedback loop
    while(1) {

        //check if the rocket has reached apogee or turned too far.
        //commented out for static tests with no ejection charge
        if ((millis()-liftoffTime > 10000) || (abs(angleX) > 45) || (abs(angleZ) > 45)) {

            //open valve for 0.5s
            delay(100);
            digitalWrite(solunoid, HIGH);
            delay(500);
            digitalWrite(solunoid, LOW);

            //indicate activity
            digitalWrite(LEDB, HIGH);

            for(std::list<float>::iterator it1=uTimeList.begin(), it2=anglXList.begin(), it3=anglZList.begin(), it4=gyroXList.begin(), it5=gyroZList.begin(), it6 = sPosXList.begin(), it7 = sPosZList.begin();
            (it1!=uTimeList.end());
            ++it1, ++it2, ++it3, ++it4, ++it5, ++it6, ++it7) 
            {
                myFile = SD.open("data.txt", FILE_WRITE);
                myFile.print(*it1);
                myFile.print("\t");
                myFile.print(*it2);
                myFile.print("\t");
                myFile.print(*it3);
                myFile.print("\t");
                myFile.print(*it4);
                myFile.print("\t");
                myFile.print(*it5);
                myFile.print("\t");
                myFile.print(*it6);
                myFile.print("\t");
                myFile.print(*it7);
                myFile.println();
                myFile.close();
            }
            
            digitalWrite(LEDB, LOW);
            //turn off activity light and shut down
            blinky.completeNoise();
        }

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
            gyroZData = gZ/32.800;
            angleX += tstepS*gyroXData;
            angleZ += tstepS*gyroZData;
            intX += angleX*tstepS;
            intZ += angleZ*tstepS;

            //feedback loop
            TVC.control(gyroXData, gyroZData, angleX, angleZ, intX, intZ);
            
            if(i%3 == 0) {
                uTimeList.push_back(tstep);
                anglXList.push_back(angleX);
                anglZList.push_back(angleZ);
                gyroXList.push_back(gyroXData);
                gyroZList.push_back(gyroZData);
                sPosXList.push_back(TVC.getPosX());
                sPosZList.push_back(TVC.getPosZ());
            }
        }
    }
}