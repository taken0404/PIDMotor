#include <PIDMotor.h>



// create a PIDMotor object
//            (enca, encb, pwn, in1, in2)
PIDMotor x = PIDMotor(3,5,11,9,10);


void setup() {
    Serial.begin(115200);

    // set PID gain values
    //              kp,  kd,   ki
    x.setGainValues(2, 0.05, 0.001);

    // set max motor position
    x.setMaxPos(3400);

    // set max acceptable error
    x.setMaxErr(50);
    // attach interrupt to call the function you created
    attachInterrupt(digitalPinToInterrupt(x._enca), xInterrupt, CHANGE);
   
}



void loop() {
    if (Serial.available() > 0) {
        x.setTarget(Serial.parseInt());
      
    }
  
    x.move();
    Serial.println(x.posi);
   
}

// for each motor, you need to create a function that calls your motor's readEncoder() method
void xInterrupt(){ 
    x.readEncoder();
}



