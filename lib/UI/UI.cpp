#include "UI.h"
#include <Arduino.h>

UI::UI(int LEDR, int LEDG, int LEDB, int buzzer)
    :R(LEDR),G(LEDG),B(LEDB),buzz(buzzer)
{}

void UI::armedNoise()
{
    digitalWrite(B, LOW);
    digitalWrite(R, LOW);
    
    digitalWrite(G, HIGH);
    tone(buzz, 440);
    delay(250);
    digitalWrite(G, LOW);
    noTone(buzz);
    delay(250);
    digitalWrite(G, HIGH);
    tone(buzz, 440);
    delay(250);
    digitalWrite(G, LOW);
    noTone(buzz);
    delay(250);
    digitalWrite(G, HIGH);
    tone(buzz, 440);
    delay(250);
    digitalWrite(G, LOW);
    noTone(buzz);
    delay(500);
    digitalWrite(G, HIGH);

    tone(buzz, 440);
    delay(250);
    noTone(buzz);
    delay(250);
    tone(buzz, 494);
    delay(500);
    tone(buzz, 659);
    delay(500);
    noTone(buzz);
    digitalWrite(G, LOW);
}

void UI::failNoise() 
{
    digitalWrite(B, LOW);
    digitalWrite(G, LOW);
    digitalWrite(R, HIGH);
    tone(buzz, 440);
    delay(500);
    tone(buzz, 293.66);
    delay(500);
    noTone(buzz);
}

void UI::startupNoise()
{
    digitalWrite(G, HIGH);
    tone(buzz, 440);
    delay(250);
    digitalWrite(G, LOW);
    noTone(buzz);
    delay(250);
    digitalWrite(G, HIGH);
    tone(buzz, 494);
    delay(250);
    digitalWrite(G, LOW);
    noTone(buzz);
    delay(250);
    digitalWrite(G, HIGH);
    tone(buzz, 659);
    delay(250);
    digitalWrite(G, LOW);
    noTone(buzz);
    delay(500);

    digitalWrite(B, HIGH);
    tone(buzz, 494);
    delay(500);
    noTone(buzz);
}

void UI::successNoise()
{
    digitalWrite(B, LOW);
    digitalWrite(R, LOW);
    digitalWrite(G, HIGH);
    tone(buzz, 293.66);
    delay(500);
    tone(buzz, 440);
    delay(500);
    noTone(buzz); 
    digitalWrite(G, LOW);
    digitalWrite(B, HIGH);
}

void UI::countDown() {
    digitalWrite(R, LOW);
    digitalWrite(G, LOW);
    for(int i = 0; i < 10; i++) {
        if (i%2 == 0) {
            digitalWrite(B, HIGH);
        } else {
            digitalWrite(B, LOW);
        }
        tone(buzz, 440);
        delay(200);
        noTone(buzz);
        delay(800);
    }
    for(int i = 0; i < 4; i++) {
        if (i%2 == 0) {
            digitalWrite(G, HIGH);
        } else {
            digitalWrite(G, LOW);
        }
        tone(buzz, 494);
        delay(200);
        noTone(buzz);
        delay(800);
    }
    digitalWrite(G, HIGH);
    digitalWrite(B, HIGH);
    digitalWrite(R, HIGH);
    tone(buzz, 659);
    delay(1000);
    digitalWrite(G, LOW);
    digitalWrite(B, LOW);
    digitalWrite(R, LOW);
    noTone(buzz);
}

void UI::completeNoise() {
    int i = 0;
    digitalWrite(B, LOW);
    digitalWrite(G, LOW);
    while (1) {
        if(i%2 == 0) digitalWrite(R, HIGH);
        else digitalWrite(R, LOW);
        tone(buzz, 494);
        delay(200);
        noTone(buzz);
        delay(1800);
        i++;
    }
}