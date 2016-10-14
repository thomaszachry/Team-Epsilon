#include <AccelStepper.h>
#include <Stepper.h>

#define HWSERIAL Serial1
//pins 0-23 have interupt capability!
//define basemtr + pin (motor controller)
//define basemtr - pin motor controller)
//define arm + pin motor controller)
//define arm - pin motor controller)
//define actuator + pin motor controller)
//define actuator - pin motor controller)
//define pump + pin motor controller)
//define pump - pin motor controller)
//define selonoid + pin
//define selonoid - pin
//define base encoder + pin
//define base encoder - pin
//define arm encoder + pin
//define arm encoder - pin
//define actuator encoder + pin
//define actuator encoder - pin

//define steps per mtr here...
//define range of motion for actuator. 

//base height = 
//arm seg 1 length =
//arm seg 2 length = 
//linear actuator length = 

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
        HWSERIAL.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
     int incomingByte;
        
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.print("USB received: ");
    Serial.println(incomingByte, DEC);
                HWSERIAL.print("USB received:");
                HWSERIAL.println(incomingByte, DEC);
  }
  if (HWSERIAL.available() > 0) {
    incomingByte = HWSERIAL.read();
    Serial.print("UART received: ");
    Serial.println(incomingByte, DEC);
                HWSERIAL.print("UART received:");
                HWSERIAL.println(incomingByte, DEC);
  }

}

void mov_baseMtr(int startPos, int endPos, int steps, bool sucess)
{

 
}
void mov_armMtr(int startPos, int endPos, int steps, bool sucess)
{
  
}
void mov_endEft(int startPos, int endPos, int steps, bool sucess)
{
  
}
void suction() //electronic switch for air supply suction or blowing
{
  
}
void airMtr() //air pump motor. 
{
  
}

int pos_baseMtr() //encoder for base motor
{
  int position;
  /* Rotary encoder with attachInterrupt

Counts pulses from an incremental encoder and put the result in variable counter. 
Taking also into account the direction and counts down when the rotor rotates in 
the other direction.

This code is used attachInterrupt 0 and 1 which are pins 2 and 3 moust Arduino.
For more information about attachInterrupt see:
http://arduino.cc/en/Reference/AttachInterrupt
 
created 2014
by Ben-Tommy Eriksen

https://github.com/BenTommyE/BenRotaryEncoder
 
*/

 //Encoder connect to digitalpin 2 and 3 on the Arduino.
/*
volatile unsigned int counter = 0;  //This variable will increase or decrease depending on the rotation of encoder

void setup() {
  Serial.begin (9600);
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
  
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);
}

void loop() {
  // Send the value of counter
  Serial.println (counter);
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    counter++;
  }else{
    counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    counter--;
  }else{
    counter++;
  }
}

*/


  return position;
  
}

int pos_armMtr() //encoder for arm motor
{
 int position;
 
 /* Rotary encoder with attachInterrupt

Counts pulses from an incremental encoder and put the result in variable counter. 
Taking also into account the direction and counts down when the rotor rotates in 
the other direction.

This code is used attachInterrupt 0 and 1 which are pins 2 and 3 moust Arduino.
For more information about attachInterrupt see:
http://arduino.cc/en/Reference/AttachInterrupt
 
created 2014
by Ben-Tommy Eriksen

https://github.com/BenTommyE/BenRotaryEncoder
 
*/

 //Encoder connect to digitalpin 2 and 3 on the Arduino.
/*
volatile unsigned int counter = 0;  //This variable will increase or decrease depending on the rotation of encoder

void setup() {
  Serial.begin (9600);
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
  
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);
}

void loop() {
  // Send the value of counter
  Serial.println (counter);
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    counter++;
  }else{
    counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    counter--;
  }else{
    counter++;
  }
}

*/
  return position; 
}
int msgSend()
{
  int success;
  //see sketch example softwareSerial/twoPortRecieve for multi port serial comunication
  
  return success;
  
}
int msgRcv()
{
  int success;

  return sucess;
}

