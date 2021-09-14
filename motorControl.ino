
const double pi = 3.141516;
const double diametro = 6;
/*-----------------motor setup---------------*/
const int pinENA = 12;
const int pinIN1 = 7;
const int pinIN2 = 8;

const int pinIN3 = 9;
const int pinIN4 = 10;
const int pinENB = 11;

const int waitTime = 2000;   //espera entre fases
const int speed = 200;      //velocidad de giro

const int pinMotorI[3] = { pinENA, pinIN1, pinIN2 };
const int pinMotorD[3] = { pinENB, pinIN3, pinIN4 };
const int encoderD = 2;
const int encoderI=3;

//----------------VARIABLES ENCODER-----------
int N=20;//ranuras de encoder
volatile double wr=0; //velocidad angular
volatile double frecuenciaR =0;//frecuencia de interrupcion;
volatile double wI=0; //velocidad angular
volatile double frecuenciaI =0;//frecuencia de interrupcion;
volatile double vr=0;
volatile double vI=0;
//----contador encoder---------
volatile int encoder_countD;
volatile int encoder_countI;
volatile int vueltaD;
volatile int vueltaI;

//---- variables para medir el tiempo----------
//----tiempo de muestreo-----//
unsigned tinicial=1;

volatile unsigned tiempoInterrupcionActualR=0; //tiempo de Interrupcion 
volatile unsigned tiempoInterrupcionAnteriorR=0;
volatile unsigned deltaTiempoInterrupcionR=0;

volatile unsigned tiempoInterrupcionActualI=0; //tiempo de Interrupcion 
volatile unsigned tiempoInterrupcionAnteriorI=0;
volatile unsigned deltaTiempoInterrupcionI=0;

volatile double deltaMuestreo=0;
//------filtro promedio------//
float measureL[5]={0};
float measureR[5]={0};

//--funciones prototipo-----//
void motorSetup();
void isrD();
void isrI();
void moveForward(const int pinMotor[3], int speed);

void setup() {

  motorSetup();
  pinMode(encoderD,INPUT_PULLUP);
  pinMode(encoderI,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderD),isrD,FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderI),isrI,FALLING);
    Serial.begin(9600);
    fullStop(pinMotorI);
    fullStop(pinMotorD);
}

void loop() {
  tiempoInterrupcionActualR=micros();
  tiempoInterrupcionActualI=micros();
  
  wr= frecuenciaR*2*pi/N;
  vr= wr*diametro/2;
  wI= frecuenciaI/N*2*pi;
  vI= wI*diametro/2;  
  Serial.print(frecuenciaR);
  Serial.print(',');
  Serial.println(frecuenciaI);
 

  
  moveForward(pinMotorI,90);
  moveForward(pinMotorD,90);
  
  

  
  
  /*Serial.println("VUELTAS:");
  Serial.println(vueltaD);
  */
}

void isrD(){
  encoder_countD++;
 // if (encoder_countD==3){
    deltaTiempoInterrupcionR=tiempoInterrupcionActualR - tiempoInterrupcionAnteriorR;
    tiempoInterrupcionAnteriorR=tiempoInterrupcionActualR;//es necesario saber el tiempo en el que se hizo la interrupcion
    frecuenciaR=(1000000)/(double)(deltaTiempoInterrupcionR); //frecuencia en Hz
        
 // }
 }
 
 void isrI(){
  encoder_countI++;
  //if (encoder_countI==3){
    deltaTiempoInterrupcionI=tiempoInterrupcionActualI-tiempoInterrupcionAnteriorI;
    tiempoInterrupcionAnteriorI=tiempoInterrupcionActualI;//es necesario saber el tiempo en el que se hizo la interrupcion
    frecuenciaI=(1000000)/(double)deltaTiempoInterrupcionI; //frecuencia en Hz
  //}
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
