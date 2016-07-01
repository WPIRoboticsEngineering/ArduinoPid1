#include <Servo.h>
#include <Encoder.h>


Encoder myEnc(2, 3);
const int Pot1 = A0;
const int Pot2 = A1;
const int Pot3 = A2;
double lastError = 0;
double errorSum = 0;
double motor_input; 
double act_pos = 0;
const int theta_des = 50;
int kp = 0;
int ki = 0;
int kd = 0;
int errordiff = 0;
int motor_init = 90;
int error1 = 0; 
long oldPosition  = 0;
long unsigned int newtime;
long unsigned int oldtime;
int motorvar = 90;
int degree = 0;
int minServo = 30;
int maxServo = 150;
long unsigned int lastTime = 0;
double totalerror=0;
Servo myservo;

void setup() {
  // put your setup code here, to run once:
    pinMode(Pot1, INPUT);
    pinMode(Pot2, INPUT);
    Serial.begin(115200);
    myservo.attach(11,1000,2000);
    myEnc.write(0);
}

void loop() {
    long newPosition = myEnc.read();
    if (newPosition > oldPosition+10) {
      oldPosition = newPosition;
      degree++;
     // Serial.println(degree);
    }
    if (newPosition <oldPosition-10){
      oldPosition = newPosition;
      degree--;
      //Serial.println(degree);
    }
    
    
//    newtime = millis();
//    if (newtime>oldtime+1000){
//      if (motorvar == 116){ 
//        motorvar = 65;}
//      else{ 
//        motorvar = 116;}
//      oldtime = newtime;
//    
//      }
//    myservo.write(motorvar);
    
    

  if((millis()-lastTime)>=10){
  pidControl();
  Serial.print(degree);
  Serial.print(",");
  Serial.print(theta_des);
  Serial.print(",");
  Serial.print(totalerror);
  Serial.print(",");
  Serial.print("p;");
  Serial.print(ReadPot1());
  Serial.print(",");
  Serial.print("i;");
  Serial.print(ReadPot2());
  Serial.print(",");
  Serial.print("d;");
  Serial.print(ReadPot3());
  Serial.print("\n");
  lastTime=millis();
  }
}

void pidControl(){
    kp = ReadPot1();
    ki = ReadPot2();
    kd = ReadPot3();
    
    error1 = theta_des - degree;
    errordiff = degree - lastError;
    
    errorSum+=errordiff;
    
    if(errordiff<0.5){
    errordiff=0;
    }
    
    totalerror = motor_init + (kp * error1) + (kd * errordiff) + (ki*errorSum);
    motor_input = totalerror;
    lastError = degree;

//  if (degree<0){//t
//  Serial.println("here");
//  motor_input = constrain(motor_input,90-(degree)*kp-(kd * errordiff),maxServo);
//  motor_input = constrain(motor_input,minServo,maxServo);
//  }else 
//  if (degree<15){//t
//  motor_input = constrain(motor_input,90,maxServo);
//  } else if (degree>85){//t
//  motor_input = constrain(motor_input,minServo,90);
//  } else {
//  motor_input = constrain(motor_input,minServo,maxServo);
//  }

  motor_input = constrain(motor_input,75,105);


  myservo.write(motor_input);
}

float ReadPot1(){
  float val = analogRead(Pot1);
  return val;
}

float ReadPot2(){
  float val = analogRead(Pot2);
  return val;
}

float ReadPot3(){
  float val = analogRead(Pot3);
  return val;
}
