#include "SerialTransfer.h"
#include <AccelStepper.h>

struct STRUCT {
  float temp1;
  float temp2;
  float temp3;
  float temp4;
  float temp5;
} data;

SerialTransfer teensyToArduinoTransfer;

//Nema 23
const int dirPin1 = 2;
const int stepPin1 = 3;
AccelStepper linearStepper(AccelStepper::DRIVER, stepPin1, dirPin1); //controlled by driver board, 3 = step, 2 = direction

bool motorTurning = false;


void setup() {
  SerialUSB.begin(9600);
  Serial1.begin(9600);
  teensyToArduinoTransfer.begin(Serial1);

  linearStepper.setMaxSpeed(200); //max speed set in pul/rev
  linearStepper.setSpeed(200); //speed in steps per second
  linearStepper.setCurrentPosition(0);
  linearStepper.setAcceleration(200);
  linearStepper.runToNewPosition(200*100);

  delay(500);
}

void loop() {

  SerialUSB.println("Starting Loop");

  // linearStepper.runSpeed();

  if(teensyToArduinoTransfer.available()) {

    SerialUSB.println("Got new data.");

    uint16_t recSize = 0;
    recSize = teensyToArduinoTransfer.rxObj(data, recSize);
    
    SerialUSB.println(data.temp1);
    SerialUSB.println(data.temp2);
    SerialUSB.println(data.temp3);
    SerialUSB.println(data.temp4);
    SerialUSB.println(data.temp5);

    ////// Choose what temperature we want to send to the temperature controller


    ////// Send data to the temperature controller



  } else if(teensyToArduinoTransfer.status < 0) {

    SerialUSB.print("ERROR: ");

    if(teensyToArduinoTransfer.status == -1)
      SerialUSB.println(F("CRC_ERROR"));
    else if(teensyToArduinoTransfer.status == -2)
      SerialUSB.println(F("PAYLOAD_ERROR"));
    else if(teensyToArduinoTransfer.status == -3)
      SerialUSB.println(F("STOP_BYTE_ERROR"));
  }

  delay(1000);
}
