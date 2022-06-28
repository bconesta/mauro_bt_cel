#include <Arduino.h>
#include <analogWrite.h>
class Motor
{
    private:
        int IN1_pin, IN2_pin, PWM_pin, PWM_rg;
    public:
        Motor();
        void attach(int IN1_pin, int IN2_pin, int PWM_pin);
        void write(bool IN1, bool IN2, int PWM);
        void printPWM(void);

};

