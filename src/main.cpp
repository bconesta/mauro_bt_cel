#include <Arduino.h>
#include <BluetoothSerial.h>
#include <MotorDriver.h>

BluetoothSerial bt;

//Definición de pines a utilizar por el puente H (L298N)
#define EN_1 13
#define IN1 14
#define IN2 12
#define EN_2 25
#define IN3 26
#define IN4 27

//Definición de pines a utilizar por sensores
#define TRIG_A 5
#define ECHO_A 18

#define TRIG_D 2
#define ECHO_D 4

#define TRIG_I 19
#define ECHO_I 21

//#define IR_F 35
#define IR_D 33
#define IR_I 32

//MODOS
#define SUMO 0
#define DESPEJE 1
#define LINEA_I 2
#define LINEA_D 3

int modo = 5;

#define VEL 255
#define VEL_LINEA 180
#define DIST_MAX_DESP 100.00
#define DIST_MAX_LINEA 60.00
const unsigned long t_max_desp = (DIST_MAX_DESP/0.03433)*2.0;
const unsigned long t_max_linea = (DIST_MAX_LINEA/0.03433)*2.0;
unsigned long distancia; 

bool sentido = false;

Motor motorL;
Motor motorR;

char inst;

void setup() {
  bt.begin("Mauro");
  bt.setPin("1922");

  Serial.begin(115200);
 
  motorL.attach(IN1,IN2,EN_1);
  motorR.attach(IN3,IN4,EN_2);
 
  pinMode(TRIG_A, OUTPUT);
  pinMode(TRIG_D, OUTPUT);
  pinMode(TRIG_I, OUTPUT);
  pinMode(ECHO_A, INPUT);
  pinMode(ECHO_D, INPUT);
  pinMode(ECHO_I, INPUT);

  pinMode(IR_D, INPUT);
  pinMode(IR_I, INPUT);

  while(modo == 5){
    if(bt.available()) inst = bt.read();

    if(inst == 'F') modo = SUMO;
    if(inst == 'B') modo = DESPEJE;
    if(inst == 'L') modo = LINEA_I;
    if(inst == 'R') modo = LINEA_D;
  }
}

void loop() {
  if(bt.available()) inst = bt.read();
  if(inst == 'S') ESP.restart();

  switch(modo){
    case SUMO:
      if(inst == 'F'){
        //Avanza
        motorL.write(1,0,VEL);
        motorR.write(1,0,VEL);
      }
      else if(inst == 'B'){
        //Retrocede
        motorL.write(0,1,VEL);
        motorR.write(0,1,VEL);
      } 
      else if(inst == 'L'){
        //Izquierda
        motorL.write(0,1,VEL);
        motorR.write(1,0,VEL);
      }
      else if(inst == 'R'){
        //Derecha
        motorL.write(1,0,VEL);
        motorR.write(0,1,VEL);
      }
      else if(inst == 'G'){
        //Avanza izq
        motorL.write(0,0,VEL);
        motorR.write(1,0,VEL);
      }
      else if(inst == 'I'){
        //Avanza der
        motorL.write(1,0,VEL);
        motorR.write(0,0,VEL);
      }
    break;
    /////////////////////////////////////
    case DESPEJE:
      if(digitalRead(IR_I) || digitalRead(IR_D)){
          motorL.write(1,0,180);
          motorR.write(1,0,180);
          delay(1000);
        }
        digitalWrite(TRIG_A, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_A, LOW);
        distancia = pulseIn(ECHO_A, HIGH, t_max_desp);
        Serial.print("Distancia atras: ");
        Serial.println(distancia);
        if(distancia){
          //Si sensa en el frente
          motorL.write(0,1,200);
          motorR.write(0,1,200);
        }
        else{
          digitalWrite(TRIG_D, HIGH);
          delayMicroseconds(10);
          digitalWrite(TRIG_D, LOW);
          distancia = pulseIn(ECHO_D, HIGH, t_max_desp);
          if(distancia){
            //Si sensa a la derecha
            Serial.print("Distancia derecha: ");
            Serial.println(distancia);
            sentido = true;
          }
          else{
            digitalWrite(TRIG_I, HIGH);
            delayMicroseconds(10);
            digitalWrite(TRIG_I, LOW);
            distancia = pulseIn(ECHO_I, HIGH, t_max_desp);
            if(distancia){
              Serial.print("Distancia izq: ");
              Serial.println(distancia);
              sentido = false;
            }
            else{
              //Si no sensó nada
            }
          }

          motorL.write(0,sentido,190);
          motorR.write(0,!sentido,190);
       
        }
    break;
    /////////////////////////////////////
    case LINEA_I:
      if(digitalRead(IR_I)){
        motorR.write(0,1,VEL_LINEA);
        motorL.write(0,0,0);
      }
      else{
        motorL.write(0,1,VEL_LINEA);
        motorR.write(0,0,0);
      }
    break; 
    ////////////////////////////////////
    case LINEA_D:
      if(digitalRead(IR_D)){
        motorL.write(0,1,VEL_LINEA);
        motorR.write(0,0,0);
      }
      else{
        motorR.write(0,1,VEL_LINEA);
        motorL.write(0,0,0);
      }
    break;
  }
}