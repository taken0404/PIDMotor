// author:  Ahmad Hasan
// Arduino library for communicating and controlling DC motors with encoders using PID control loops and pwm signals
// requires a L298N H-bridge
// library inspired by: https://github.com/curiores/ArduinoTutorials & https://github.com/natnqweb/Motor_PID



#ifndef PIDMotor_h
#define PIDMotor_h

#include <Arduino.h>

class PIDMotor
{
    public:
        
        int pos;
        int _enca, _encb; // encoder pins on arduino  _enca should be on an Arduino interrupt pin
        
        PIDMotor(int enca, int encb, int pwm, int in1, int in2 );    // Constructor
        void setGainValues(float kpIn, float kdIn, float kiIn);     // A function to set the parameters
        void setMaxPwr(int maxpwr); // set max pwm power
        void setTarget(int target); // function to set the target position
        int getTarget(); //funciton to get the target
        void goTo(int targetPos); // go to target position
        void move();  
        void readEncoder();
        void setMaxPos(int maxpos);
        void setMaxErr(int maxerr);
        

    private:

        int  _in1, _in2; // dc motor pins on arduino
        int _pwm; // pwm signal pin on arduino; should be a pwm pin on Arduino
        

        float kp, kd, ki, umax; // PID Parameters
        float eprev, eintegral; // PID Storage

        volatile int posi; // position; updated by encoders
        int target; // target position
        int maxPos; // max pos of axis
        int maxErr; // maximum allowable error incase target is not reached
        
        long currT; // current time
        long prevT = 0; // previous time
        float deltaT; // delta time

        int pwr, dir; // power and direction values
        
        void evalu(); // function to compute the control signal
        void setMotor(); // funciton to set the motor 
        void init(); // initialize the pins  
        
  
};



#endif





