#include <SimpleTimer.h>

#define _DEBUG true

SimpleTimer timer;
const int motor1 = 10;
const int motor2 = 11;
int countEncoder1 = 0;
int countEncoder2 = 0;
int cmd1 = 0;
int cmd2 = 0;

const float controlEngineeringFrequency = 50.0; //chosen by the developer
const int gearRatio = 100;//cf. motor's doc
const int encoderCPR = 32; //cause we only use channel A output, it must be 32 (cf.doc)

float spinPerSecWanted = 0.5; //perimeter's wheel of 48,5cm 

float sumError1 = 0;
float sumError2 = 0;
float precError1 = spinPerSecWanted;
float precError2 = spinPerSecWanted;


void setup(){
  Serial.begin(38400);//38400 bauds because it will be the case when integrating with PixyController
  
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  //analogWrite(motor1, 255); //Motor output at 0
  //analogWrite(motor2, 255); //Motor output at 0
  delay(5000);
  
  attachInterrupt(0, incrCountEncoder1, CHANGE); //Interruption 0 is on pin2 for motor1
  attachInterrupt(1, incrCountEncoder2, CHANGE); //Interruption 1 is on pin3 for motor2
  timer.setInterval(1000/controlEngineeringFrequency, PIDController); //every 20 ms, the function is called
} 

void loop(){
  timer.run();
  delay(10);
}

void incrCountEncoder1(){
 countEncoder1++; 
}  

void incrCountEncoder2(){
 countEncoder2++;
}

void PIDController(){
  unsigned long time1 = millis();
  
  //The following coefficients are set by Ziegler-Nichols method
  float kp1 = 200; // kps must be fixed by testing, for a Proportional control
  float kp2 = 200;
  
  float ki1 = 2.5;
  float ki2 = 2.9; //kis must be fixed by testing, for an Integral control
  
  float kd1 = 10666; //kds must be fixed by testing, for a Derivative control
  float kd2 = 9333;

  float encoderFrequency1 = controlEngineeringFrequency*(float)countEncoder1;
  float encoderFrequency2 = controlEngineeringFrequency*(float)countEncoder2;
  
  float spinPerSec1 = encoderFrequency1/(float)encoderCPR/(float)gearRatio;
  float spinPerSec2 = encoderFrequency2/(float)encoderCPR/(float)gearRatio;
  float error1 = spinPerSecWanted - spinPerSec1;
  float error2 = spinPerSecWanted - spinPerSec2;
  
  countEncoder1 = 0;
  countEncoder2 = 0;
  
  sumError1 += error1;
  sumError2 += error2;
  float deltaError1 = error1 - precError1;
  float deltaError2 = error2 - precError2;
  precError1 = error1;
  precError2 = error2;
  
  cmd1 = kp1*error1 + kd1*deltaError1 + ki1*sumError1; 
  cmd2 = kp2*error2 + kd2*deltaError2 + ki2*sumError2;
  
  // if cmd equals 0, motor runs, if cmd equals 105, motor stops
  if(cmd1 < 0) {
    cmd1 = 0;
  }
  else if (cmd1 > 105) {
    cmd1 = 105;
  }
  
  if(cmd2 < 0) {
    cmd2 = 0;
  }
  else if (cmd2 > 105) {
    cmd2 = 105;
  }
  
  analogWrite(motor1, 105 - cmd1);
  analogWrite(motor2, 105 - cmd2);
  
  if (_DEBUG) {
    Serial.print(" Tour par sec 1 :");
    Serial.println(spinPerSec1, 8);
    Serial.print(" Tour par sec 2 :");
    Serial.println(spinPerSec2, 8);
  }
  unsigned long time2 = millis();
    
  if (time2-time1 < 10) {
      delay(10 - time2 + time1); // to always wait 10ms if is less, so the speed control is properly done
  }
}  
