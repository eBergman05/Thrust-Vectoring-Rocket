//@author Ethan Bergman
//@version 1.0
//@date 11.23.2022
//
//Takes in data from MPU6050 (6 DOF gyro/accel) and uses a PID feedback loop to manipulate TVC mount accordingly
//
#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "pid.h"
#include "MPU_Callibration.h"
#include "FlashSst26.h"
#include "PrintSerialFlashSst26.h"
#include "Adafruit_BMP280.h"
#include "UI.h"


//MPU6050 accelgyro(0x69); // <-- use for AD0 high
MPU6050 accelgyro;

Adafruit_BMP280 bmp;

//FlashSst26 flash;
//PrintSerialFlashSst26 printFlash;

int16_t aX, aY, aZ;
int16_t gX, gY, gZ;

const int SIZE = 100;

long time0;
long time1;
long tstep[SIZE];

int16_t gyroXData[SIZE];
int16_t gyroYData[SIZE];
int16_t gyroZData[SIZE];

float angleX[SIZE];
float angleZ[SIZE];

float pressure;
float altitude;

double data[5];

int count = 0;

bool liftoff = false;
bool apogee = false;
int16_t yCache;
int liftoffTime;

// uncomment "OUTPUT_READABLE_ACCELgyrO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELgyrO

// uncomment "OUTPUT_BINARY_ACCELgyrO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELgyrO

//define servos
Servo servoX;
Servo servoZ; 
Servo servoE; //ejection servo

int pos = 90; //starting servo position

#define LEDR 9
#define LEDB 8
#define LEDG 7
#define buzzer 37

UI blinky(LEDR, LEDG, LEDB, buzzer);

File myFile;

bool blinkState = false;

void setup() {

    pinMode(LEDR, OUTPUT);
    pinMode(LEDB, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(buzzer, OUTPUT);
    
    blinky.startupNoise();

    Wire.begin();

    //Serial.begin(38400);

    //initialize device
    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    //verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection());

    //set resolution level (see MPU6050.ccp for details)
    accelgyro.setFullScaleGyroRange(2); //32.8 LSB/deg/sec
    //apply Low Pass Filter (see MPU6050.ccp for details)
    accelgyro.setDLPFMode(6);

    //attach to servos to pins 4 and 5
    servoX.attach(4);
    servoZ.attach(5);
    servoE.attach(6);

    servoX.write(pos);
    servoZ.write(pos);
    servoE.write(0);


    //Serial.print("Initializing SD card...");

    if (!SD.begin(BUILTIN_SDCARD)) 
    {
        //Serial.println("initialization failed!");
        blinky.failNoise();
        while(1);
    }
    //Serial.println("initializationdone.");

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    myFile = SD.open("data.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (myFile) 
    {
        //Serial.print("Writing to test.txt...");
        myFile.println("Data: angleX, angleZ, pressure, altitued, time");
        // close the file:
        myFile.close();
        //Serial.println("done.");
        blinky.successNoise();
    } else {
        // if the file didn't open, print an error:
        //Serial.println("error opening data.txt");
        blinky.failNoise();
    }

    // re-open the file for reading:
    /*
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
    yCache = aY;
    blinky.armedNoise();
}

void loop() {
// read raw accel/gyro measurements from device

    accelgyro.getMotion6(&aX, &aY, &aZ, &gX, &gY, &gZ);

    /*
    #ifdef OUTPUT_READABLE_ACCELgyrO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(aX); Serial.print("\t");
        Serial.print(aY); Serial.print("\t");
        Serial.print(aZ); Serial.print("\t");
        Serial.print(gX); Serial.print("\t");
        Serial.print(gY); Serial.print("\t");
        Serial.println(gZ);
    #endif

    #ifdef OUTPUT_BINARY_ACCELgyrO
        Serial.write((uint8_t)(aX >> 8)); Serial.write((uint8_t)(aX & 0xFF));
        Serial.write((uint8_t)(aY >> 8)); Serial.write((uint8_t)(aY & 0xFF));
        Serial.write((uint8_t)(aZ >> 8)); Serial.write((uint8_t)(aZ & 0xFF));
        Serial.write((uint8_t)(gX >> 8)); Serial.write((uint8_t)(gX & 0xFF));
        Serial.write((uint8_t)(gY >> 8)); Serial.write((uint8_t)(gY & 0xFF));
        Serial.write((uint8_t)(gZ >> 8)); Serial.write((uint8_t)(gZ & 0xFF));
    #endif
    */
    
    if (!liftoff && (abs(aY - yCache) > 100)){
        liftoff = true;
        liftoffTime = millis();
    }
    if ((liftoff && (millis()-liftoffTime > 4000)) || (abs(angleX[99]) > 45) || (abs(angleZ[99]) > 45)){
        servoE.writeMicroseconds(2000);
        //might be wrong direction
    }

    for(int i = 0; i < 99; i++){
        gyroXData[i] = gyroXData[i+1];
        gyroYData[i] = gyroYData[i+1];
        gyroZData[i] = gyroZData[i+1];
        angleX[i] = angleX[i+1];
        angleZ[i] = angleZ[i+1];
        tstep[i] = tstep[i+1];
    }

    time0 = time1;
    time1 = millis();
    tstep[99] = time1 - time0;
    
    gyroXData[99] = gX;
    gyroYData[99] = gY;
    gyroZData[99] = gZ;
    angleX[99] = angleX[98] + tstep[99]*gX/32800.;
    angleZ[99] = angleZ[98] + tstep[99]*gZ/32800.;

    count++;
    if (count > 100)
    {
        PID(servoX, servoZ, gyroXData, gyroZData, angleX, angleZ, tstep, SIZE);
    }

    data[0] = angleX[99];
    data[1] = angleZ[99];
    data[2] = bmp.readPressure();
    data[3] = bmp.readAltitude(1050.35);
    data[4] = millis();

    //Serial.println(data[3]);

    myFile = SD.open("data.txt", FILE_WRITE);
    for (int n = 0; n < 5; n++)
    {
        myFile.print(data[n]);
        myFile.print("\t");
    }
    myFile.println();
    myFile.close();
        
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LEDB, blinkState);
}