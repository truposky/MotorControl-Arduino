
#include <math.h>
#include "MeanFilterLib.h"
#include <Arduino_LSM6DS3.h>
//----------------parametros RObot-------------------
#define D 6.7//cm
#define R 3.2 //aprox
#define L 9.5 // cm distancia entre ruedas
      //     ---------------motor setup---------------*/
const int pinENA = 12;//señal de PWM
const int pinIN1 = 7;//indica sentido de giro
const int pinIN2 = 8;//indica sentido de giro

const int pinIN3 = 9;//indica sentido de giro
const int pinIN4 = 10;
const int pinENB = 11;//Señal de PWM

const int pinMotorD[3] = { pinENA, pinIN1, pinIN2 };
const int pinMotorI[3] = { pinENB, pinIN3, pinIN4 };


//-----------------contador encoder------------------------------------------------------------------------------------
const int             N=              20;//Resolucion encoder       
const int             encoderD =      15;//pin de entrada de encoder derecha
const int             encoderI=       13;//pin de entrada de encoder izquierda
volatile unsigned     encoder_countD=  0;//cuenta los pulsos de encoder derecha
int                   encoder_countD_after=0;
int                   encoder_countI_after=0;
int                   dif_encoderD=0;
int                   dif_encoderI=0;
volatile unsigned     encoder_countI=  0;//cuenta los pulsos de encoder izquierda
volatile int          vueltaD=         0;//cuenta las vueltas que ha dado la rueda derecha
volatile int          vueltaI=         0;//cuenta las vueltas que ha dado la rueda izquierda
int                   valorD=               0;//media de tiempo
int nD=0;
int nI=0;
//----------T-------IEMPO ENTRE INTERRUPCIONES-----------------------------------------------------------------------//
volatile unsigned long startTimeI=0;
volatile unsigned long timeAfterI=0;
volatile unsigned      deltaTimeI;//diferencia de tiempo entre una interrupcion y otra

volatile unsigned long startTimeD=0;
volatile unsigned long timeAfterD=0;
volatile unsigned      deltaTimeD;//diferencia de tiempo entre una interrupcion y otra

volatile unsigned long timeStopD=0;//se usa para poner a cero los radianes cuando el tiempo es muy grande
volatile unsigned long timeStopI=0;//se usa para poner a cero los radianes cuando el tiempo es muy grande
volatile unsigned long deltaTimeStopD;
volatile unsigned long deltaTimeStopI;
//------------------------debounce-----------------------------------------------------------------------------------
//Se usara para evitar que cuenten pulsos debido a rebotes o ruido del sistema.
# define               TIMEDEBOUNCE 8 //(ms) es el tiempo minimo entre pulsos
volatile unsigned long timeAfterDebounceD=0;
volatile unsigned long timeBeforeDebounceD=0;
volatile unsigned long deltaDebounceD=0;

volatile unsigned long timeAfterDebounceI=0;
volatile unsigned long timeBeforeDebounceI=0;
volatile unsigned long deltaDebounceI=0;
//---------------------variables odometria------------------------------------------------------------------------------------
double                 distanciaI=0;
double                 distanciaD=0;

//---------------------VARIABLES DEL CONTROLADOR-----------------------------------------------------------------------------
//+++++++++++ variables internas del controlador++++++++++++++
double                   k=1;//se mide en milisefundos y sirve para establecer un tiempo de muestreo discreto
//++++++++++++++++++rueda derecha+++++++++++++++++
unsigned long         currentTimeD, previousTimeD=0;;
double                elapsedTimeD;
double                errorD=0, lastErrorD=0, cumErrorD=0, rateErrorD;
// ++++++++++rueda izquierda++++++++++++
unsigned long         currentTimeI, previousTimeI=0;;
double                elapsedTimeI;
double                errorI=0, lastErrorI=0, cumErrorI=0, rateErrorI;
// +++++++++++++++  ++++Constantes del controlador+++++++++++++++
double                kp=0, Ki=0.00007, Kd=0;
// ++++++++++++++++++variables externas del controlador++++++++++++++++++
double                Input, output;
//double                Setpoint;//se usa para indicar el valor deseado unidades en rad/s

//-------------------------GIROSCOPIO------------------------------------------------------------------------------------------------//Ç
float x, y, z;
const float ERR_GIROSCOPE=3.05;
//-------------------------FUNCIONES PROTOTIPO---------------------------/------------------------------------------------------------/
void motorSetup();//inicializa las variables para controlar los motores
void isrD();//fucnion de interrupcion para contar pulsos de encoder derecha
void isrI();//funcion de interrupcion para contar pulsos de encoder izquierda
void moveForward(const int pinMotor[3], int speed);//funcion que ordena mover hacia delante el robot
void moveBackward(const int pinMotor[3], int speed);//funcion para mover hacia atras la rueda
void fullStop(const int pinMotor[3]);//fucnion que para la rueda
double odometria();
int pidD(double wD,double Setpoint);//controlador PID de rueda derecha
int pidI(double wI,double Setpoint);//controlador PID de rueda Izquierda
void feedForward(double Setpoint,int &PWM_D,int&PWM_I);
void ajusteGyroscope(float z,float setpoint);
void TimeWait();//tiempo de espera entre cambio y cambio de PWM 
//void moveBackward(const int pinMotor[3], int speed);
//--------------------------------------------filtro de media movil simple para estabilizar la lectura de rad/s---------------------------------------
MeanFilter<long> meanFilterD(7);
MeanFilter<long> meanFilterI(7);



void setup() {
  Serial.begin(9600);
  delay(5000);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }
  motorSetup();
  pinMode(encoderD,INPUT_PULLUP);
  pinMode(encoderI,INPUT_PULLUP);
 

  attachInterrupt(digitalPinToInterrupt(encoderI), isrI, RISING);//prepara la entrada del encoder como interrupcion
    
  attachInterrupt(digitalPinToInterrupt(encoderD), isrD, RISING);
  
  
  cumErrorI=0;
  cumErrorD=0;
  fullStop( pinMotorD);
  fullStop( pinMotorI);
  delay(300);
  /*moveForward(pinMotorD,74);
  moveForward(pinMotorI,78);
  delay(300);
   moveForward(pinMotorD,75);
  moveForward(pinMotorI,79);
  delay(300);
   moveForward(pinMotorD,76);
  moveForward(pinMotorI,79);
  delay(300);
    PWM_D=round((Setpoint-6.93)/0.144);
    PWM_I=round((Setpoint-7.86)/0.155);
    PWM_D=76*(PWM_D<75)+PWM_D*(PWM_D>=75);
    PWM_I=75*(PWM_I<74)+PWM_I*(PWM_I>=75);
  moveForward(pinMotorD,PWM_D);
  moveForward(pinMotorI,PWM_I);
  delay(500);  */
   
}
int    PWM_D;
int    PWM_I;
double Setpoint=19,SetpointAnterior;//se usa para indicar el valor de referncia es temporal se debera usar uno para cada rueda
double errorGyroscope=0;
void loop() {
    
    double wI,wD;
    //setpoint sera indicada por el ordenador central
    
    if(Setpoint != SetpointAnterior){//solo se accede si el Setpoint ha cambiado
      feedForward(Setpoint, PWM_D,PWM_I);
      SetpointAnterior=Setpoint;
    
    }
      Serial.print("deltaTimeI: ");
      Serial.print(deltaTimeI);
      Serial.print("\n");
    //se define un tiempo maximo de espera para establecer una velocidad igual a cero
    double fD=(double)1/(deltaTimeD*N)*1000000;
    double fI=(double)1/(deltaTimeI*N)*1000000;
    //condicion para que no supere linealidad y se sature.
    if(fD<31/2/3.14){
      wD=2*3.14*fD;
     
    }
    if(fI<31/2/3.14 ){
       wI=2*3.14*fI;
    }
 
    
    //Se compara con el valor del giroscopio
    /* (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(x, y, z);
      errorGyroscope=ajusteGyroscope((double)z,wD,wI,Setpoint);
     }
     if(errorGyroscope>0){
      PWM_D=PWM_D+errorGyroscope*0.1;
     }
     else{
      PWM_I=PWM_I+errorGyroscope*-0.1;
     }*/
     
    //se calcula el valor para el controlador pid
    int ajusteD=pidD(wD,25.8);
    int ajusteI=pidI(wI,18); 
    PWM_D=PWM_D+ajusteD;
    PWM_I=PWM_I+ajusteI;
   /*
    PWM_D=74*(PWM_D<75)+PWM_D*(PWM_D>=75);
    PWM_I=75*(PWM_I<74)+PWM_I*(PWM_I>=74);
    if(PWM_D>255 ){
      PWM_D=255;
      
    }
    if(PWM_I>255){
      PWM_I=255;
    }
     if(PWM_D<0 ){
      PWM_D=0;
      
    }
    if(PWM_I<0){
      PWM_I=0;
    }*/
   /* Serial.print("Derecha:");
    Serial.print(PWM_D);
    Serial.print(",");
    Serial.print(wD);
    Serial.print(","); 
     Serial.print(ajusteD);
    Serial.print(",");   
    Serial.print("izquierda:");
    Serial.print(PWM_I);
    
    Serial.print(",");
     Serial.print(wI);
    Serial.print(",");
    Serial.println(ajusteI);
    */
    Serial.println(encoder_countD);
    //fullStop( pinMotorI);
    moveForward(pinMotorD,PWM_D);
    moveForward(pinMotorI,PWM_I);
    //se espera un tiempo antes de cambiar  PWM
    //no se usa delay opara evitar interferir con las interruociones.
    TimeWait();//se establece 300 milisegundos tiempo suficente para que el PWM cambie sin afectara a los motores
    
}

void TimeWait(){
   int ahora=millis();
    int resta=0;
    int despues=0;
  while(resta<250){
      despues=millis();
      resta=despues-ahora;
      deltaTimeD=meanFilterD.AddValue(deltaTimeD);
      deltaTimeI=meanFilterI.AddValue(deltaTimeI);
    }
}


float angularVelocitiMeasure(){
  
  
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
void moveBackward(const int pinMotor[3], int speed)
{
   digitalWrite(pinMotor[1], LOW);
   digitalWrite(pinMotor[2], HIGH);

   analogWrite(pinMotor[0], speed);
}
void fullStop(const int pinMotor[3])
{
   digitalWrite(pinMotor[1], LOW);
   digitalWrite(pinMotor[2], LOW);

   analogWrite(pinMotor[0], 0);
}
/*
double odometria(){
  dif_encoderD=encoder_countD-encoder_countD_after;
  dif_encoderI=encoder_countI-encoder_countI_after;

  distanciaD=PI*diametro*((double)dif_encoderD)/20;
  distanciaI=PI*diametro*((double)dif_encoderI)/20;

  encoder_countD_after=encoder_countD;
  encoder_countI_after=encoder_countI;
}*/

void isrD(){
  
  timeBeforeDebounceD=millis();//tiempo para evira rebotes
  deltaDebounceD=timeBeforeDebounceD-timeAfterDebounceD;// tiempo que ha pasdo entre interrupcion he interrupcion
  if(deltaDebounceD>TIMEDEBOUNCE){//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.
    startTimeD=micros();
    deltaTimeD=startTimeD-timeAfterD;
    encoder_countD++;
    timeAfterD=micros();
  }
  timeAfterDebounceD=millis();

}

void isrI(){
    
    timeBeforeDebounceI=millis();//tiempo para evira rebotes
    deltaDebounceI=timeBeforeDebounceI-timeAfterDebounceI;// tiempo que ha pasdo entre interrupcion he interrupcion
    if(deltaDebounceI>TIMEDEBOUNCE){//condicion para evitar rebotes
      //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.
      startTimeI=micros();
      deltaTimeI=startTimeI-timeAfterI;
      encoder_countI++;//se cuenta los pasos de encoder
      timeAfterI=micros();
      
    }
    timeAfterDebounceI=millis();  
  
}
int pidD(double wD,double Setpoint){
  int output=0;
  currentTimeD = millis();                               // obtener el tiempo actual
  elapsedTimeD = (double)(currentTimeD - previousTimeD); // calcular el tiempo transcurrido
  if(elapsedTimeD>=k){//se asegura un tiempo de muestreo
    errorD = Setpoint - wD;
    if(errorD>=0.6|| errorD<(-0.6)){
        cumErrorD += errorD * elapsedTimeD;                      // calcular la integral del error
        if(lastErrorD>0 && errorD<0)cumErrorD=errorD;
        if(lastErrorD<0 && errorD>0)cumErrorD=errorD;
        if(cumErrorD>35000 || cumErrorD<-35000) cumErrorD=0;                         //se resetea el error acumulativo
        rateErrorD = (errorD - lastErrorD) / elapsedTimeD;         // calcular la derivada del error
      
      output =(int) round(kp*errorD +Ki*cumErrorD + Kd*rateErrorD );     // calcular la salida del PID     0.0001*cumError  + Kd*rateErrorD
        lastErrorD = errorD;                                      // almacenar error anterior
    }
    previousTimeD = currentTimeD;                             // almacenar el tiempo anterior
  }
  return output;
}


int pidI(double wI,double Setpoint){
  int output=0;
  currentTimeI = millis();                               // obtener el tiempo actual
  elapsedTimeI = (double)(currentTimeI - previousTimeI);     // calcular el tiempo transcurrido
  if(elapsedTimeD>=k){//se asegura un tiempo de muestreo
    errorI = Setpoint - wI;   
    Serial.print("\nerrorI:");
    Serial.print(errorI);
    Serial.print("\t");
                    
    if(errorI>=0.6|| errorI<-0.6){
      cumErrorI += errorI * elapsedTimeI;  //pasado un tiempo se tiene que borrrar cumerror                    // calcular la integral del error
   
      if(lastErrorI>0 && errorI<0){
        cumErrorI=errorI;
      }
      if(lastErrorI<0 && errorI>0){
        cumErrorI=errorI;
      }
      if(cumErrorI>35000||cumErrorI<-35000) cumErrorI=0;     //se resetea el error acumulativo 
      rateErrorI = (errorI - lastErrorI) /elapsedTimeI;         // calcular la derivada del error
    
      output = (int)round(kp*errorI  +Ki*cumErrorI + Kd*rateErrorI);     // calcular la salida del PID 0.0001*cumError  + Kd*rateErrorI
     
      lastErrorI = errorI; 
          Serial.print("cumerrorI:");
    Serial.print(cumErrorI);
    Serial.print("\n");// almacenar error anterior
    }
    previousTimeI = currentTimeI;                             // almacenar el tiempo anterior
  }
  return output;
  
}

double ajusteGyroscope(double z,double &wD,double &wI,double Setpoint){//Setpoint en rad/s
  double W=D/2*(Setpoint-Setpoint)/L;// se calcula la velocidad angular del centro de masas del robot
   z=z+ERR_GIROSCOPE-0.3;
   double error=W-z;
   if(z>-0.15 && z<0.20){
    z=0;
   }
   /*Serial.print(W);
   Serial.print(",");
   Serial.print(z);
   Serial.print(",");
   */
   Serial.print(z);
   Serial.print(",");
   Serial.print(error);
   Serial.print("\n");
   
}


void feedForward(double Setpoint,int &PWM_D,int&PWM_I){
    PWM_D=round((Setpoint-7.887)/0.138);
    PWM_I=round((Setpoint-5.479)/0.1989);
    PWM_D=76*(PWM_D<75)+PWM_D*(PWM_D>=75);//Condiciones de linealidad
    PWM_I=75*(PWM_I<74)+PWM_I*(PWM_I>=75);//condiciones de linealidad
    moveForward(pinMotorD,74);
  moveForward(pinMotorI,78);
  delay(150);
   moveForward(pinMotorD,75);
  moveForward(pinMotorI,79);
  delay(150);
   moveForward(pinMotorD,76);
  moveForward(pinMotorI,79);
  delay(150);
    moveForward(pinMotorD,PWM_D);
    moveForward(pinMotorI,PWM_I);
    TimeWait();
}
