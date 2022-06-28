#include "MotorDriver.h"

Motor::Motor(){

}

void Motor::attach(int _IN1_pin, int _IN2_pin, int _PWM_pin){
    IN1_pin = _IN1_pin;
    IN2_pin = _IN2_pin;
    PWM_pin = _PWM_pin;
    
    pinMode(IN1_pin, OUTPUT);
    pinMode(IN2_pin, OUTPUT);
    pinMode(PWM_pin, OUTPUT);
}

void Motor::write(bool IN1, bool IN2, int PWM){
    digitalWrite(IN1_pin, IN1);
    digitalWrite(IN2_pin, IN2);
    analogWrite(PWM_pin, PWM);
    if(!IN1 && IN2) PWM_rg = -PWM;
    else PWM_rg = PWM;
}

void Motor::printPWM(){
    Serial.print(PWM_rg);
}