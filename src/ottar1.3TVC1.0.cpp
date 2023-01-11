//@author Ethan Bergman
//@version 1.0
//@date 11.23.2022
//
//Takes in data from MPU6050 (6 DOF gyro/accel) and uses a PID feedback loop to manipulate TVC mount accordingly
//
#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "pid.h"
#include "MPU_Callibration.h"
#include "FlashSst26.h"
#include "Adafruit_BMP280.h"

MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
Adafruit_BMP280 bmp;

int16_t aX, aY, aZ;
int16_t gX, gY, gZ;

int time0;
int time1;
int tstep[100];

int16_t gyroXData[100];
int16_t gyroYData[100];
int16_t gyroZData[100];

float angleX[100];
float angleZ[100];

float pressure;
float altitude;

double data[5];

int count = 0;

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

int pos = 90; //starting servo position

#define LEDR 9
#define LEDB 8
#define LEDG 7
#define buzzer 37

bool blinkState = false;

void setup() {
    Wire.begin();

    Serial.begin(38400);

    //initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    //verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection());

    //set resolution level (see MPU6050.ccp for details)
    accelgyro.setFullScaleGyroRange(2); //32.8 LSB/deg/sec
    //apply Low Pass Filter (see MPU6050.ccp for details)
    accelgyro.setDLPFMode(6);

    //attach to servos to pins 4 and 5
    servoX.attach(4);
    servoZ.attach(5);

    servoX.write(pos);
    servoZ.write(pos);

    
    pinMode(LEDR, OUTPUT);
    pinMode(LEDB, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(buzzer, OUTPUT);

    digitalWrite(LEDB, HIGH);
    delay(1000);
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, LOW);

    callibrateMPU(servoX, servoZ, accelgyro);

    digitalWrite(LEDG, HIGH);
    tone(buzzer, 440);
    delay(500);
    noTone(buzzer);
    delay(500);
    tone(buzzer, 440);
    delay(500);
    noTone(buzzer);

}

void loop() {
// read raw accel/gyro measurements from device

    accelgyro.getMotion6(&aX, &aY, &aZ, &gX, &gY, &gZ);

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

    
    for(int i = 0; i < 99; i++)
    {
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
    //angleX[99] = 360*asin(aX/16384.)/(2*M_PI);
    //angleZ[99] = 360*asin(aZ/16384.)/(2*M_PI);
    angleX[99] = angleX[98] + tstep[99]*gX/32800.;
    angleZ[99] = angleZ[98] + tstep[99]*gZ/32800.;
    

    Serial.println(angleX[99],10);
    Serial.println(angleZ[99],10);
    
    
    count++;
    if (count > 100)
    {
        PID(servoX, servoZ, gyroXData, gyroZData, angleX, angleZ, tstep);
    }

    data[0] = angleX[99];
    data[1] = angleZ[99];
    data[2] = bmp.readPressure();
    data[3] = bmp.readAltitude(1050.35);
    data[4] = millis();

    Serial.println(data[3]);


    //delay(5);
    
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LEDG, blinkState);
}
