/*-----------------motor setup---------------*/
const int pinENA = 12;
const int pinIN1 = 7;
const int pinIN2 = 8;

const int pinIN3 = 9;
const int pinIN4 = 10;
const int pinENB = 11;

const int waitTime = 2000;   //espera entre fases
const int speed = 200;      //velocidad de giro

const int pinMotorD[3] = { pinENA, pinIN1, pinIN2 };
const int pinMotorI[3] = { pinENB, pinIN3, pinIN4 };
int encoderD = 15;
int encoderI=19;

//----contador encoder---------
volatile unsigned encoder_countD=0;
volatile unsigned encoder_countI=0;
volatile int vueltaD;
volatile int vueltaI;

//--funciones prototipo-----//
void motorSetup();
void isrD();
void isrI();
void moveForward(const int pinMotor[3], int speed);


void setup() {
  motorSetup();
  pinMode(encoderD,INPUT_PULLUP);
  pinMode(encoderI,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderD),isrD,LOW);
  attachInterrupt(digitalPinToInterrupt(encoderI),isrI,LOW);
   Serial.begin(9600);
    fullStop(pinMotorI);
    fullStop(pinMotorD);
}

void loop() {
  Serial.print(encoder_countD);
  Serial.print(',');
  Serial.println(encoder_countI);

  
  moveForward(pinMotorI,55);
  moveForward(pinMotorD,55);

}


void isrD(){
  encoder_countD++;
 
 }
 
 void isrI(){
  encoder_countI++;
 
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
