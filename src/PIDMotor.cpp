#include "PIDMotor.h"
#include "Arduino.h"
#include "atomic.h"



PIDMotor::PIDMotor(int enca, int encb, int pwm, int in1, int in2){ 
      _enca = enca;
      _encb = encb;
      _pwm = pwm;
      _in1 = in1;
      _in2 = in2;
      init();
}
  
void PIDMotor::setGainValues(float kpIn, float kdIn, float kiIn){
    kp = kpIn; 
    kd = kdIn; 
    ki = kiIn; 
}
void PIDMotor::setMaxPwr(int maxpwr){
    umax = maxpwr;
}
void PIDMotor::setMaxPos(int maxpos){
    maxPos = maxpos;
}
void PIDMotor::setMaxErr(int maxerr){
    maxErr = maxerr;
}
void PIDMotor::setTarget(int target){
    this->target = constrain(target, 0, maxPos);
}
int PIDMotor::getTarget(){
    return this->target;
}
void PIDMotor::move(){
    evalu();
    setMotor();
     if(abs(target-pos)<maxErr){
        digitalWrite(_in1,LOW);
        digitalWrite(_in2,LOW);
    }
}

void PIDMotor::init(){
      pinMode(_enca, INPUT);
      pinMode(_encb, INPUT);
      pinMode(_pwm, OUTPUT);
      pinMode(_in1, OUTPUT);
      pinMode(_in2, OUTPUT);

}


void PIDMotor::evalu(){ // A function to compute the control signal

    // read time

    currT = micros();
    deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

   
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){  pos = posi; } // read position in atomic block
   
    int e = target - pos;  // error

    
    float dedt = (e-eprev)/(deltaT); // derivative

   
    eintegral = eintegral + e*deltaT;  // integral

  
    float u = (kp*e) + (kd*dedt) + (ki*eintegral);   // control signal

    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax){
        pwr = umax;
    }
    // motor direction
    dir = 1;
    if(u<0){
        dir = -1;
    }
   
    eprev = e;  // store previous error
}
void PIDMotor::setMotor(){

    if(dir == 1){
        digitalWrite(_in1,HIGH);
        digitalWrite(_in2,LOW);
        analogWrite(_pwm,pwr);
    }
    else if(dir == -1){
        digitalWrite(_in1,LOW);
        digitalWrite(_in2,HIGH);
        analogWrite(_pwm,pwr);
    }
    else{
        digitalWrite(_in1,LOW);
        digitalWrite(_in2,LOW);
    }  
    
}
void PIDMotor::readEncoder(){
    if(digitalRead(_enca) == digitalRead(_encb)){
        this->posi++;
    }
    else{ 
        this->posi--;
    }
}





