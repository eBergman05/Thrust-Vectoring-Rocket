#ifndef UI_H
#define UI_H

#include <Arduino.h>

class UI
{
    private:
        int R;
        int G;
        int B;
        int buzz;

    public:
        UI(int LEDR, int LEDG, int LEDB, int buzzer);

        void armedNoise();        
        void failNoise();
        void startupNoise();
        void successNoise();

};

#endif