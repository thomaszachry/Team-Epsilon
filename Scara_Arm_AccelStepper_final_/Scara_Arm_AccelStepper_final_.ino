/***********************************************************************************************************************
 * @FILE serial_accelstepper_control.ino
 * @BRIEF An example Arduino sketch showing USB-serial communications with the Teensy microcontroller for stepper control
 * using the AccelStepper library
 *
 * This program provides an example of USB-serial communications with the Teensy 3.1 microcontroller. The communication 
 * is based on variable width byte packets containing an error checksum. The packet structure is defined as follows:
 *
 * packet[0] = PACKET_START_BYTE (0xAA)
 * packet[1] = PACKET_SIZE (total number of bytes including overhead and payload)
 * packet[n+2] = payload byte n -> [0, PAYLOAD_SIZE - 1]
 * packet[PACKET_SIZE - 1] = packet checksum
 *
 * The checksum is computed as the XOR chain of each byte in the packet before the checksum:
 * packet[0] XOR packet[1] XOR ... XOR packet[PACKET_SIZE - 2]
 *
 * The program handles the following packets:
 * 1. Immediate step command    [0xAA][17][0x0A][motor_index][acceleration][velocity][num steps][checksum]
 *    The acceleration, velocity, and num steps fields are 4 byte each, MSB first
 * 2. Deferred step command     [0xAA][17][0x0B][motor_index][acceleration][velocity][num steps][checksum]
 *    The acceleration, velocity, and num steps fields are 4 byte each, MSB first
 * 3. Execute deferred commands [0xAA][4][0x0C][checksum]
 *    Executes all deferred commands simultaneously
 *
 * @AUTHOR Christopher D. McMurrough
 * 
 * Stepper Deffinitions:
 * Base Stepper = baseMtr
 * pins: 
 * Pull+ = 12
 * Dir+ = 11
 * Enable = 10
 * Arm Stepper = armMtr
 * pins: 
 * Pull+ = 9
 * Dir+ = 8
 * Enable = 7
 * 
 * Pin Deffinitions
 * Selonoid Relay_ Activate = 2
 * Air Pump_mtr_PWM = 3
 * Air Pump Mtr In_A = 4
 * Air Pump Mtr IN_B = 5
 * Linear Actuator_Activate = 6
 * 
 * Base Encoder In_A = 14
 * Base Encoder In_B = 15
 * Arm Encoder In_A = 16
 * Arm Encoder In_B = 17
 * Limit Switch Base = 18
 * Limit Switch Arm Left = 19
 * Limit Switch Arm Right = 20
 * 
 * 
 * 
 * 
 **********************************************************************************************************************/

// inlcude necessary header files
#include <Encoder.h>
#include <AccelStepper.h>
#include <Servo.h>
#define STROKE_MAX_MM 200

String incomingByte;



// define GPIO pins
const int LED_PIN = 13;
const int LED_ON = HIGH;
const int LED_OFF = LOW;
const int STEPPER_ENABLED = HIGH;
const int STEPPER_DISABLED = LOW;


const int BASE_STEPPER_STEP_PIN = 12;
const int BASE_STEPPER_DIR_PIN = 11;
const int BASE_STEPPER_ENABLE_PIN = 10;

const int ARM_STEPPER_STEP_PIN = 9;
const int ARM_STEPPER_DIR_PIN = 8;
const int ARM_STEPPER_ENABLE_PIN =7;

//const int LINEAR_ACTUATOR_FEEDBACK = 17;
const int LINEAR_ACTUATOR_PWM = 6;
//const int LINEAR_ACTUATOR_Activate = 15;

const int AIR_SOLENOID = 2;

const int AIR_PUMP_PWM = 3;
const int AIR_PUMP_INA = 4;
const int AIR_PUMP_INB = 5;
const int AIR_PUMP_EN = 3;

const int BASE_ENCODER_IN_A = 14;
const int BASE_ENCODER_IN_B = 15;

int baseEncPos; //Holds position for base encoder

const int ARM_ENCODER_IN_A = 16;
const int ARM_ENCODER_IN_B = 17;

int armEncPos; // Holds position for arm encoder

const int BASE_LIMIT_SWITCH = 18;
const int ARM_RIGHT_LIMIT_SWITCH = 20;
const int ARM_LEFT_LIMIT_SWITCH = 19;


 
// define serial communication parameters
const unsigned long BAUD_RATE = 9600;

// define packet parameters
const byte PACKET_START_BYTE = 0xAA;
const unsigned int PACKET_OVERHEAD_BYTES = 3;
const unsigned int PACKET_MIN_BYTES = PACKET_OVERHEAD_BYTES + 1;
const unsigned int PACKET_MAX_BYTES = 255;

// define special packets
const byte STEP_COMMAND_IMMEDIATE = 0x0A;
const unsigned int STEP_COMMAND_IMMEDIATE_LENGTH = 17;
const byte STEP_COMMAND_DEFERRED = 0x0B;
const unsigned int STEP_COMMAND_DEFERRED_LENGTH = 17;
const byte STEP_COMMAND_EXECUTE = 0x0C;
const unsigned int STEP_COMMAND_EXECUTE_LENGTH = 4;

// define the stepper objects
//Base_Mtr
AccelStepper Stepper_1(1, BASE_STEPPER_STEP_PIN, BASE_STEPPER_DIR_PIN);
//Arm_Mtr
AccelStepper Stepper_2(1, ARM_STEPPER_STEP_PIN, ARM_STEPPER_DIR_PIN);

Servo myServo;
/*
 
AccelStepper Stepper_6(1, STEPPER_6_STEP_PIN, STEPPER_6_DIR_PIN);
*/


int base_reading;
int left_arm_reading;
int right_arm_reading;

int base_state;
int right_arm_state;
int left_arm_state; 

int base_total_steps; 
int arm_total_steps;

long time = 0;
long debounce = 200;

//Encoder arm_encoder(ARM_ENCODER_INA,ARM_ENCODER_INB);
//Encoder base_encoder(BASE_ENCODER_INA,BASE_ENCODER_INA);


/***********************************************************************************************************************
 * @BRIEF perform initial setup of the microcontroller
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
void setup()
{
  
// initialize the LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

// initialize the stepper motors
  Stepper_1.setEnablePin(BASE_STEPPER_ENABLE_PIN);
  Stepper_2.setEnablePin(ARM_STEPPER_ENABLE_PIN);
// activate the stepper motors
   Stepper_1.enableOutputs();
  Stepper_2.enableOutputs();

  myServo.attach(LINEAR_ACTUATOR_PWM);

//initialize the limit switches
 pinMode(BASE_LIMIT_SWITCH, INPUT);
 pinMode(ARM_LEFT_LIMIT_SWITCH, INPUT);
 pinMode(ARM_RIGHT_LIMIT_SWITCH, INPUT);
 
  //initialize air pump
  pinMode(AIR_PUMP_PWM, OUTPUT); 
 pinMode(AIR_PUMP_INA, OUTPUT); 
 pinMode(AIR_PUMP_INB, OUTPUT);
  pinMode(AIR_PUMP_EN,OUTPUT);
  
  //initialize relay module for solenoid
pinMode(AIR_SOLENOID, OUTPUT);
digitalWrite(AIR_SOLENOID, HIGH);
  //initialize encoder pins
pinMode(BASE_ENCODER_IN_A, INPUT); 
pinMode(BASE_ENCODER_IN_B, INPUT);

pinMode(ARM_ENCODER_IN_A, INPUT); 
pinMode(ARM_ENCODER_IN_B, INPUT); 

 attachInterrupt(0, doEncoderA, CHANGE);
 attachInterrupt(1, doEncoderB, CHANGE);

  //initialize linearactuator 
  
//pinMode(LINEAR_ACTUATOR_PWM, OUTPUT); 

 
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

/***********************************************************************************************************************
 * @BRIEF assembles and transmits a serial packet containing the given payload
 * @PARAM[in] payloadSize the size of the given payload in bytes
 * @PARAM[in] payload pointer to the data payload array
 * @RETURN true if the packet was transmitted successfully
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
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

/***********************************************************************************************************************
 * @BRIEF checks to see if the given packet is complete and valid
 * @PARAM[in] packetSize the size of the given packet buffer in bytes
 * @PARAM[in] packet pointer to the packet buffer
 * @RETURN true if the packet is valid
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
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

void robot_setup()
{
  boolean isDone = false;

 int steps;
 int steps_left;
 int steps_right;
  
  int limit = -99;

  base_reading = digitalRead(BASE_LIMIT_SWITCH);
  left_arm_reading = digitalRead(ARM_LEFT_LIMIT_SWITCH);
  right_arm_reading = digitalRead(ARM_RIGHT_LIMIT_SWITCH);

Stepper_1.setAcceleration((float) 1000*1000); // steps per second squared
       Stepper_1.setMaxSpeed((float) 1000); // steps per second
       
while(!isDone)
{       
  Stepper_1.runToNewPosition((int) 1);
              
  if(base_reading == HIGH && millis() - time > debounce)
  {
    steps ++;
    base_state = limit;
    isDone = true;
    break;
  }
  if(left_arm_reading == HIGH && millis() - time > debounce)
  {
    steps ++;
    left_arm_state = limit;
    isDone = true;
    break;
  }
  if(right_arm_reading == HIGH && millis() - time > debounce)
   {
   steps ++;
    right_arm_state = limit;
    isDone = true;
    break;
   }
  
 }
}
/***********************************************************************************************************************
 * ENCODER DECLERATIONS
 * BASE_ENCODER_IN_A = 14;
 * BASE_ENCODER_IN_B = 15;
 * baseEncPos - to hold numerical values for encoder movement
 * ARM_ENCODER_IN_A = 16;
 * ARM_ENCODER_IN_B = 17;
 * armEncPos - to hold numerical values for encoder movement
 *   
 * @BRIEF Functions to monitor Encoder movement for the base and arm
 * 
 **********************************************************************************************************************/
void doEncoderA(){ //Base encoder
  if (digitalRead(BASE_ENCODER_IN_A) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(BASE_ENCODER_IN_B) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      baseEncPos = baseEncPos - 1;         // CCW
    } 
    else {
      baseEncPos = baseEncPos + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(BASE_ENCODER_IN_B) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      baseEncPos = baseEncPos + 1;          // CW
    } 
    else {
      baseEncPos = baseEncPos - 1;          // CCW
    }

  }
  //Serial.println (baseEncPos, DEC);          // debug - remember to comment out
                                              // before final program run
  // you don't want serial slowing down your program if not needed
}
void doEncoderB(){ // Arm Encoder
  if (digitalRead(ARM_ENCODER_IN_A) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(ARM_ENCODER_IN_B) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      armEncPos = armEncPos - 1;         // CCW
    } 
    else {
      armEncPos = armEncPos + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(ARM_ENCODER_IN_B) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      armEncPos = armEncPos + 1;          // CW
    } 
    else {
      armEncPos = armEncPos - 1;          // CCW
    }

  }
  //Serial.println (armEncPos, DEC);          // debug - remember to comment out
                                              // before final program run
  // you don't want serial slowing down your program if not needed
}
/***********************************************************************************************************************
   * AIR_PUMP_PWM = 3;
   * const int AIR_PUMP_INA = 4;
   * const int AIR_PUMP_INB = 5;
   * @BRIEF Functions for controlling DC Air Pump Motor
   *  Air Pump provides both positive and negative pressure. 
   *    *****MOTOR WILL ONLY RUN IN ONE DIRECTION - DO NOT RUN IN REVERSE!!!!**************
   * @PARAM[1]: PWM_VAL = pulse width modulation value for the output of the motor. 
   * @AUTHOR Thomas Zachry
   *
**********************************************************************************************************************/
void motorForward(int PWM_val)  {
 digitalWrite(AIR_PUMP_INA, HIGH);
 digitalWrite(AIR_PUMP_INB, LOW);
 digitalWrite(AIR_PUMP_EN, HIGH);
 analogWrite(AIR_PUMP_PWM, PWM_val);
}
void motorStop()  {
 analogWrite(AIR_PUMP_PWM, 0);
 digitalWrite(AIR_PUMP_INA, LOW);
 digitalWrite(AIR_PUMP_INB, LOW);
 digitalWrite(AIR_PUMP_EN,LOW);
}
/***********************************************************************************************************************
 * @BRIEF Function for controlling Linear Actuator
 * 
 * @PARAM[1]: StrokePercentage = calculated percentage of required length to extend. 
 *  Calculated by setStrokeMM
 * @AUTHOR Thomas Zachry
 **********************************************************************************************************************/
void setStrokePerc(float strokePercentage)
{
  if ( strokePercentage >= 1.0 && strokePercentage <= 99.0 )
  {
    int usec = 1000 + strokePercentage * ( 2000 - 1000 ) / 100.0 ;
    myServo.writeMicroseconds( usec );
  }
}
/***********************************************************************************************************************
 * @BRIEF Calculates the percentage of available stroke to extend
 * @PARAM[1]: Requested stroke required
 * @RETURN: percentage of available stroke needed to extend. 
 * @AUTHOR Thomas Zachry
 **********************************************************************************************************************/
void setStrokeMM(int strokeReq)
{
  SetStrokePerc( ((float)strokeReq) / STROKE_MAX_MM );
}
/***********************************************************************************************************************
 * @BRIEF Runs the pick up procedure for the linear actuator
 * 
 * @AUTHOR Thomas Zachry
 **********************************************************************************************************************/

void performPickUp()
{
//move linear actuator 3/4
setStrokeMM(75);
//blow up baloon for 2seconds
 motorForward(127);                        //PWM strength values (25%=64; 50%=127; 100%=255)
 delay(2000);
 motorStop();
 
 setStrokeMM(99);

 //suction baloon
 digitalWrite(AIR_SOLENOID,LOW); //switch relay module to solenoid for suction!!!!!
 motorForward(127);                        //PWM strength values (25%=64; 50%=127; 100%=255)
 delay(2000);
 //motorStop();

   setStroke(1);
//move to sort target

   //perform put down
}

/***********************************************************************************************************************
 * @BRIEF main program loop
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
void loop()
{
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
          // change the LED state
          ledState = !ledState;
          digitalWrite(LED_PIN, ledState);

          // echo back the packet payload
          sendPacket(packetSize - PACKET_OVERHEAD_BYTES, buffer + 2);

          // handle any defined packets
          if(buffer[2] == STEP_COMMAND_IMMEDIATE && packetSize == STEP_COMMAND_IMMEDIATE_LENGTH)
          {
            // parse an immediate step command
            byte stepperID = buffer[3];
            long stepAcc = (long) buffer[4] * 16777216 + (long) buffer[5] * 65536 + (long) buffer[6] * 256 + (long) buffer[7];
            long stepVel = (long) buffer[8] * 16777216 + (long) buffer[9] * 65536 + (long) buffer[10] * 256 + (long) buffer[11];
            long stepPos = (long) buffer[12] * 16777216 + (long) buffer[13] * 65536 + (long) buffer[14] * 256 + (long) buffer[15];

            // update the target stepper motor
            if(stepperID == 0)
            {
              Stepper_1.setCurrentPosition(0); // reset current position
              Stepper_1.setAcceleration((float) stepAcc); // steps per second squared
              Stepper_1.setMaxSpeed((float) stepVel); // steps per second
              Stepper_1.runToNewPosition((float) stepPos); // steps per second
            }
            if(stepperID == 1)
            {
              Stepper_2.setCurrentPosition(0); // reset current position
              Stepper_2.setAcceleration((float) stepAcc); // steps per second squared
              Stepper_2.setMaxSpeed((float) stepVel); // steps per second
              Stepper_2.runToNewPosition((float) stepPos); // steps per second
            }
           
          }
          else if(buffer[2] == STEP_COMMAND_DEFERRED && packetSize == STEP_COMMAND_DEFERRED_LENGTH)
          {
            // parse a deferred step command
            byte stepperID = buffer[3];
            long stepAcc = (long) buffer[4] * 16777216 + (long) buffer[5] * 65536 + (long) buffer[6] * 256 + (long) buffer[7];
            long stepVel = (long) buffer[8] * 16777216 + (long) buffer[9] * 65536 + (long) buffer[10] * 256 + (long) buffer[11];
            long stepPos = (long) buffer[12] * 16777216 + (long) buffer[13] * 65536 + (long) buffer[14] * 256 + (long) buffer[15];

            // update the target stepper motor
            if(stepperID == 0)
            {
              Stepper_1.setCurrentPosition(0); // reset current position
              Stepper_1.setAcceleration((float) stepAcc); // steps per second squared
              Stepper_1.setMaxSpeed((float) stepVel); // steps per second
              Stepper_1.moveTo((float) stepPos); // steps per second
            }
            else if(stepperID == 1)
            {
              Stepper_2.setCurrentPosition(0); // reset current position
              Stepper_2.setAcceleration((float) stepAcc); // steps per second squared
              Stepper_2.setMaxSpeed((float) stepVel); // steps per second
              Stepper_2.moveTo((float) stepPos); // steps per second
            }
            
          }
          else if(buffer[2] == STEP_COMMAND_EXECUTE && packetSize == STEP_COMMAND_EXECUTE_LENGTH)
          {
            // execute all deferred motion commands until completion
            while(Stepper_1.distanceToGo() != 0 || Stepper_2.distanceToGo() != 0)
            {
              Stepper_1.run();
              Stepper_2.run();
              
            }

            performPickUp();
          }
        }

        // reset the count
        count = 0;
      }
    }
  }
}




