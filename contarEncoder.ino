
#include <math.h>
#include "filtroEMA.h"
//----------------parametros RObot-------------------
#define diametro 6.7
      //     ---------------motor setup---------------*/
const int pinENA = 12;//señal de PWM
const int pinIN1 = 7;//indica sentido de giro
const int pinIN2 = 8;//indica sentido de giro

const int pinIN3 = 9;//indica sentido de giro
const int pinIN4 = 10;
const int pinENB = 11;//Señal de PWM

const int pinMotorD[3] = { pinENA, pinIN1, pinIN2 };
const int pinMotorI[3] = { pinENB, pinIN3, pinIN4 };


//----contador encoder---------
const int         N=              20;//Resolucion encoder       
const int         encoderD =      15;//pin de entrada de encoder derecha
const int         encoderI=       19;//pin de entrada de encoder izquierda
volatile unsigned encoder_countD=  0;//cuenta los pulsos de encoder derecha
int               encoder_countD_after=0;
int               encoder_countI_after=0;
int               dif_encoderD=0;
int               dif_encoderI=0;
volatile unsigned encoder_countI=  0;//cuenta los pulsos de encoder izquierda
volatile int      vueltaD=         0;//cuenta las vueltas que ha dado la rueda derecha
volatile int      vueltaI=         0;//cuenta las vueltas que ha dado la rueda izquierda
int               valorD=               0;
int               valorI=               0;
//----------variables tiempo muestreo----------//
volatile unsigned long startTimeI=0;
volatile unsigned long timeAfterI=0;
volatile unsigned deltaTimeI=0;

volatile unsigned long startTimeD=0;
volatile unsigned long timeAfterD=0;
volatile unsigned deltaTimeD=0;

volatile unsigned long timeStopD=0;
volatile unsigned long timeStopI=0;
volatile unsigned long deltaTimeStopD=0;
volatile unsigned long deltaTimeStopI=0;
//-----------debounce---------------
//Se usara para evitar que cuenten pulsos debido a rebotes o ruido del sistema.
# define TIMEDEBOUNCE 400 //(us) es el tiempo minimo entre pulsos
volatile unsigned long timeAfterDebounceD=0;
volatile unsigned long timeBeforeDebounceD=0;
volatile unsigned long deltaDebounceD=0;

volatile unsigned long timeAfterDebounceI=0;
volatile unsigned long timeBeforeDebounceI=0;
volatile unsigned long deltaDebounceI=0;
//---------------variables odometria-----------
double distanciaI=0;
double distanciaD=0;
 double radsI=0,radsD=0;
//------------------variables control rueda
// variables internas del controlador
//rueda derecha
unsigned long currentTimeD, previousTimeD;
double elapsedTimeD;
double errorD, lastErrorD, cumErrorD, rateErrorD;
// rueda izquierda
unsigned long currentTimeI, previousTimeI;
double elapsedTimeI;
double errorI, lastErrorI, cumErrorI, rateErrorI;
// Constantes del controlador
double kp=0.8, Ki=0.002, Kd=0.4;
// variables externas del controlador
double                   Input, Output, Setpoint;
int                      PWM_D=0;
int                      PWM_I=0;

//--funciones prototipo-----//
void motorSetup();//inicializa las variables para controlar los motores
void isrD();//fucnion de interrupcion para contar pulsos de encoder derecha
void isrI();//funcion de interrupcion para contar pulsos de encoder izquierda
void moveForward(const int pinMotor[3], int speed);//funcion que ordena mover hacia delante el robot
double odometria();
int pidD(double rads);
int pidI(double rads);


EMA media(0.5);
void setup() {
  Serial.begin(9600);
  delay(1000);
  motorSetup();
  pinMode(encoderD,INPUT_PULLUP);
  pinMode(encoderI,INPUT_PULLUP);
 

   attachInterrupt(digitalPinToInterrupt(encoderI), isrI, RISING);
    
    attachInterrupt(digitalPinToInterrupt(encoderD), isrD, RISING);
    Setpoint=15;
    fullStop(pinMotorI);
    fullStop(pinMotorD);
    PWM_D=50;
    PWM_I=80;
   moveForward(pinMotorD,PWM_D);
  moveForward(pinMotorI,PWM_I);
    double valorD=0,valorI=0; 
   
}

void loop() {
  
  timeStopD=millis();
  timeStopI=millis();
  

 deltaTimeStopD=timeStopD-timeAfterD;
 deltaTimeStopI=timeStopI-timeAfterI;
 if(deltaTimeStopD>100){
  deltaTimeD=100000000000;
  
 }
 if(deltaTimeStopI>100){
  deltaTimeI=100000000000;
  
 }
 
    
    
      valorD=media.media(deltaTimeD);
      valorI=media.media(deltaTimeI);
    
    noInterrupts();
    double fD=(double)1000/(valorD*N);
    double fI=(double)1000/(deltaTimeI*N);
    interrupts();
    radsD=2*3.14*fD;
    radsI=2*3.14*fI;
    

 
    
   Serial.print(deltaTimeD);
   Serial.print(":");
    Serial.print(deltaTimeI);
  
   Serial.print(",");
    Serial.print(valorI);
    Serial.print(",");
    Serial.println(radsI);
     
    /*int tiempoahora=millis();
    int espera=0;
    while(espera<150){
      
      espera=millis()-tiempoahora;
    }
    espera=0;
    PWM_D=PWM_D+pidD(radsD);;
    PWM_I=PWM_I+pidI(radsI);
    if(PWM_D>255 ){
      PWM_D=255;
      
    }
    if(PWM_I>255){
      PWM_I=255;
    }*/
    //moveForward(pinMotorD,0);
    //moveForward(pinMotorI,PWM_I);
   // PWM_D=PWM_D+5;
   // PWM_I=PWM_I+5;
    
    
}


 void motorSetup(){

   pinMode(pinIN1, OUTPUT);
   pinMode(pinIN2, OUTPUT);
   pinMode(pinENA, OUTPUT);
   pinMode(pinIN3, OUTPUT);
   pinMode(pinIN4, OUTPUT);
   pinMode(pinENB, OUTPUT);
}

void moveForward(const int pinMotor[3], int speed)
{
   digitalWrite(pinMotor[1], HIGH);
   digitalWrite(pinMotor[2], LOW);

   analogWrite(pinMotor[0], speed);
}
void fullStop(const int pinMotor[3])
{
   digitalWrite(pinMotor[1], LOW);
   digitalWrite(pinMotor[2], LOW);

   analogWrite(pinMotor[0], 0);
}

double odometria(){
  dif_encoderD=encoder_countD-encoder_countD_after;
  dif_encoderI=encoder_countI-encoder_countI_after;

  distanciaD=PI*diametro*((double)dif_encoderD)/20;
  distanciaI=PI*diametro*((double)dif_encoderI)/20;

  encoder_countD_after=encoder_countD;
  encoder_countI_after=encoder_countI;
}

void isrD(){
  timeBeforeDebounceD=micros();
  deltaDebounceD=timeBeforeDebounceD-timeAfterDebounceD;
  if(deltaDebounceD>TIMEDEBOUNCE){
    startTimeD=millis();
    deltaTimeD=(double)startTimeD-timeAfterD;
    encoder_countD++;
    timeAfterD=millis();
  }
  timeAfterDebounceD=micros();
  


}

void isrI(){
  timeBeforeDebounceI=micros();
  deltaDebounceI=timeBeforeDebounceI-timeAfterDebounceI;
  if(deltaDebounceI>TIMEDEBOUNCE){
    startTimeI=millis();
    deltaTimeI=(double)startTimeI-timeAfterI;
    encoder_countI++;
    timeAfterI=millis();
    
  }
  timeAfterDebounceI=micros();

}
int pidD(double rads){
  currentTimeD = millis();                               // obtener el tiempo actual
  elapsedTimeD = (double)(currentTimeD - previousTimeD);     // calcular el tiempo transcurrido
  errorD = Setpoint - rads;                               // determinar el error entre la consigna y la medición
  cumErrorD += errorD * elapsedTimeD;                      // calcular la integral del error
  rateErrorD = (errorD - lastErrorD) / elapsedTimeD;         // calcular la derivada del error

  int output = (int)round(kp*errorD  + Kd*rateErrorD);//+Ki*cumErrorD);     // calcular la salida del PID0.0001*cumError
 
  lastErrorD = errorD;                                      // almacenar error anterior
  previousTimeD = currentTimeD;                             // almacenar el tiempo anterior

  return output;
}

int pidI(double rads){
  currentTimeI = millis();                               // obtener el tiempo actual
  elapsedTimeI = (double)(currentTimeI - previousTimeI);     // calcular el tiempo transcurrido
  errorI = Setpoint - rads;                               // determinar el error entre la consigna y la medición
  cumErrorI += errorI * elapsedTimeI;                      // calcular la integral del error
  rateErrorI = (errorI - lastErrorI) / elapsedTimeI;         // calcular la derivada del error

  int output = (int)round(kp*errorI  + Kd*rateErrorI);//+Ki*cumErrorI);     // calcular la salida del PID 0.0001*cumError
 
  lastErrorI = errorI;                                      // almacenar error anterior
  previousTimeI = currentTimeI;                             // almacenar el tiempo anterior

  return output;
}
