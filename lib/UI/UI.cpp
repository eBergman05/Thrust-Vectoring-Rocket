#include "UI.h"
#include <Arduino.h>

UI::UI(int LEDR, int LEDG, int LEDB, int buzzer)
    :R(LEDR),G(LEDG),B(LEDB),buzz(buzzer)
{}

void UI::armedNoise()
{
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
