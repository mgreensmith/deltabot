/*
 * deltabot_servos.pde
 * Created by Matt Greensmith, 25 May 2011
 * version 0.2
 * Last edit: 25 November 2011
 * This is a modified version of:
 *
 * MultipleSerialServoControl.pde
 * Created:  23 December 2009
 * Author:   Brian D. Wendt
 *   (http://principialabs.com/)
 * Version:  1.1
 * License:  GPLv3
 *   (http://www.fsf.org/licensing/)
 *
 */
 
// Import the Arduino Servo library
#include <Servo.h> 

// Physical pins where servos will be attached.
int SERVO_1_PIN = 9;
int SERVO_2_PIN = 10;
int SERVO_3_PIN = 11;

// Create a Servo object for each servo
Servo servo1;
Servo servo2;
Servo servo3;

// Common servo setup values
int minPulse = 600;   // minimum servo position, us (microseconds)
int maxPulse = 2400;  // maximum servo position, us
boolean servosEnabled = false;  //servo power enabled

// User input for servo and position
int userInput[3];    // raw input from serial buffer, 3 bytes
int startbyte;       // start byte, begin reading input
int servo;           // which servo to pulse?
int pos;             // servo angle 0-180
int i;               // iterator

// LED on Pin 13 to indicate if servos have power
int ledPin = 13;
int ledPinState = LOW;

void setup() 
{ 
  pinMode(ledPin, OUTPUT);

  // Open the serial connection, 9600 baud
  Serial.begin(9600);
} 

void loop() 
{ 
  // Wait for serial input (min 3 bytes in buffer)
  if (Serial.available() > 2) {
    // Read the first byte
    startbyte = Serial.read();
    // If it's really the startbyte (255) ...
    if (startbyte == 255) {
      // ... then get the next two bytes
      for (i=0;i<2;i++) {
        userInput[i] = Serial.read();
      }
      // First byte = servo to move?
      servo = userInput[0];
      // Second byte = which position?
      pos = userInput[1];
      // Packet error checking and recovery
      if (pos == 255) { servo = 255; }

      // Assign new position to appropriate servo
      switch (servo) {
        case 1:
          servo1.write(pos);    // move servo1 to 'pos'
          break;
        case 2:
          servo2.write(pos);
          break;
        case 3:
          servo3.write(pos);
          break;
        

        // Enable or disable servos, display status on LED
        case 99:
          if (pos == 180) {  //enable
            if ( !servosEnabled ) {

              attachServos();
            }
            ledPinState = HIGH;
          }
          if (pos == 0) {    //disable
            
            detachServos();
            ledPinState = LOW;
          }
          digitalWrite(ledPin, ledPinState);
          break;
      }
    }
  }
}

void attachServos() {
  // Attach each Servo object to a digital pin
  servo1.attach(SERVO_1_PIN, minPulse, maxPulse);
  servo2.attach(SERVO_2_PIN, minPulse, maxPulse);
  servo3.attach(SERVO_3_PIN, minPulse, maxPulse);
  servosEnabled = true;
}

void detachServos() {
  servo1.detach();
  servo2.detach();
  servo3.detach();
  servosEnabled = false;
}
  

  

