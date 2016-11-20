#include <AccelStepper.h>
#include <Stepper.h>

#define HWSERIAL Serial1
#define pin1 8
#define pin2 9

AccelStepper stepper(2,pin1,pin2);
// define GPIO pins
const int LED_PIN = 13;
const int LED_ON = HIGH;
const int LED_OFF = LOW;

// define serial communication parameters
const unsigned long BAUD_RATE = 9600;

// define packet parameters
const byte PACKET_START_BYTE = 0xAA;
const unsigned int PACKET_OVERHEAD_BYTES = 3;
const unsigned int PACKET_MIN_BYTES = PACKET_OVERHEAD_BYTES + 1;
const unsigned int PACKET_MAX_BYTES = 255;

//pins 0-23 have interupt capability!
//define basemtr pins (motor controller)
//base mtr pin1 = 23, pin2 = 22, pin3 = 21, pin4 = 20

//arm mtr pin1 = 19, pin2 = 18, pin3 = 17, pin4 = 16

//base pos pin1 = 2, pin2 = 3

//define arm pos + pin motor controller)
// arm pos pin1 = 4, pin2 = 5

//define actuator + pin motor controller)

//define actuator encoder + pin

//define pump + pin motor controller)

//define selonoid + pin





//define steps per mtr here...
//define range of motion for actuator. 

//base height = 
//arm seg 1 length =
//arm seg 2 length = 
//linear actuator length = 

void setup() {
  //Check stepper spec sheet for valid values!!!
 /*  stepper.setMaxSpeed(400);
   stepper.setAcceleration(400);
   stepper.setSpeed(200);
   */
 
         // initialize the IO pins
    pinMode(LED_PIN, OUTPUT);

    // initialize the serial port
    Serial.begin(BAUD_RATE);

    // flash the LED state
    for(int i = 0; i < 25; i++)
    {
        digitalWrite(LED_PIN, LED_ON);
        delay(50);
        digitalWrite(LED_PIN, LED_OFF);
        delay(50);
}
}

void loop() {
  // put your main code here, to run repeatedly:
  // define control variables
    boolean isRunning = true;
    boolean ledState = false;

    // create the serial packet receive buffer
    static byte buffer[PACKET_MAX_BYTES];
    int count = 0;
    int packetSize = PACKET_MIN_BYTES;

    // continuously check for received packets
    while(isRunning)
    {
        // check to see if serial byte is available
        if(Serial.available())
        {
            // get the byte
            byte b = Serial.read();

            // handle the byte according to the current count
            if(count == 0 && b == PACKET_START_BYTE)
            {
                // this byte signals the beginning of a new packet
                buffer[count] = b;
                count++;
                continue;
            }
            else if(count == 0)
            {
                // the first byte is not valid, ignore it and continue
                continue;
            }
            else if(count == 1)
            {
                // this byte contains the overall packet length
                buffer[count] = b;

                // reset the count if the packet length is not in range
                if(packetSize < PACKET_MIN_BYTES || packetSize > PACKET_MAX_BYTES)
                {
                    count = 0;
                }
                else
                {
                    packetSize = b;
                    count++;
                }
                continue;
            }
            else if(count < packetSize)
            {
                // store the byte
                buffer[count] = b;
                count++;
            }

            // check to see if we have acquired enough bytes for a full packet
            if(count >= packetSize)
            {
                // validate the packet
                if(validatePacket(packetSize, buffer))
                {
                    // change the LED state if the packet is valid
                    ledState = !ledState;
                    digitalWrite(LED_PIN, ledState);
                    
                    // echo back the received packet payload
                    sendPacket(packetSize - PACKET_OVERHEAD_BYTES, buffer + 2);
                }

                // reset the count
                count = 0;
            }
        }
  }
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
boolean sendPacket(unsigned int payloadSize, byte *payload)
{
    // check for max payload size
    unsigned int packetSize = payloadSize + PACKET_OVERHEAD_BYTES;
    if(packetSize > PACKET_MAX_BYTES)
    {
        return false;
    }

    // create the serial packet transmit buffer
    static byte packet[PACKET_MAX_BYTES];

    // populate the overhead fields
    packet[0] = PACKET_START_BYTE;
    packet[1] = packetSize;
    byte checkSum = packet[0] ^ packet[1];

    // populate the packet payload while computing the checksum
    for(int i = 0; i < payloadSize; i++)
    {
        packet[i + 2] = payload[i];
        checkSum = checkSum ^ packet[i + 2];
    }

    // store the checksum
    packet[packetSize - 1] = checkSum;

    // send the packet
    Serial.write(packet, packetSize);
    Serial.flush();
    return true;
}
boolean validatePacket(unsigned int packetSize, byte *packet)
{
    // check the packet size
    if(packetSize < PACKET_MIN_BYTES || packetSize > PACKET_MAX_BYTES)
    {
        return false;
    }

    // check the start byte
    if(packet[0] != PACKET_START_BYTE)
    {
        return false;
    }

    // check the length byte
    if(packet[1] != packetSize)
    {
        return false;
    }

    // compute the checksum
    byte checksum = 0x00;
    for(int i = 0; i < packetSize - 1; i++)
    {
        checksum = checksum ^ packet[i];
    }

    // check to see if the computed checksum and packet checksum are equal
    if(packet[packetSize - 1] != checksum)
    {
        return false;
    }

    // all validation checks passed, the packet is valid
    return true;
}
void mov_baseMtr( int endPos)
{
  stepper.runToNewPosition(endPos);
  stepper.stop();
 
}
void mov_armMtr(int endPos)
{
  stepper.runToNewPosition(180);
  stepper.stop();
}
void mov_endEft(int startPos, int endPos, int steps, bool sucess)
{
  
}
void pressure()//electronic switch for air supply suction or blowing
{
  digitalWrite(pressurePin, HIGH);
}
void suction() //electronic switch for air supply suction or blowing
{
  digitalWrite(suctionPin, HIGH);
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

 //Encoder connect to digitalpin 4 and 5 on the Arduino.
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

