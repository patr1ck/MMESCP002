#include "SerialTransfer.h"
#include <AccelStepper.h>

//Notes
 //********************************************************************************************************** 
 //Any data that you're writing to the Serial port using Serial.println() 
 //or Serial.print() in your sketch will be graphed in real time in the Serial Plotter. 
 //********************************************************************************************************** 
 //To convert the time from milliseconds to seconds, minutes, or hours, you can use the following equations: 
 //Seconds: time (in seconds) = millis() / 1000 
 //Minutes: time (in minutes) = millis() / (1000 * 60) 
 //Hours: time (in hours) = millis() / (1000 * 60 * 60) 
 //***********************************************************************************************************


struct STRUCT {
  float temp1;
  float temp2;
  float temp3;
  float temp4;
  float temp5;
} data;

SerialTransfer teensyToArduinoTransfer;

const int dirPin1 = 2;
const int stepPin1 = 3;
AccelStepper linearStepper(AccelStepper::DRIVER, stepPin1, dirPin1);

void setup() {
  SerialUSB.begin(9600);
  Serial1.begin(9600);
  teensyToArduinoTransfer.begin(Serial1);

  linearStepper.setMaxSpeed(800); 
  linearStepper.setSpeed(800);
  linearStepper.setCurrentPosition(0);
  linearStepper.setAcceleration(300);
  linearStepper.moveTo(((200*100)/4)*300); 

  SerialUSB.println("Time(s),Temp1,Temp2,Temp3,Temp4,Temp5,StepperPosition,StepperSpeed,StepperAcceleration,StepperDistanceToGo,StepperDirection,StepperTargetPosition");
  delay(500);
}

void loop() {
  linearStepper.run(); // Stepper motor run command moved here

  if(teensyToArduinoTransfer.available()) {
    uint16_t recSize = 0;
    recSize = teensyToArduinoTransfer.rxObj(data, recSize);
    
    SerialUSB.print(millis()/1000.0); # time printed in seconds
    SerialUSB.print(',');
    SerialUSB.print(data.temp1);
    SerialUSB.print(',');
    SerialUSB.print(data.temp2);
    SerialUSB.print(',');
    SerialUSB.print(data.temp3);
    SerialUSB.print(',');
    SerialUSB.print(data.temp4);
    SerialUSB.print(',');
    SerialUSB.print(data.temp5);
    SerialUSB.print(',');

    SerialUSB.print(linearStepper.currentPosition());
    SerialUSB.print(',');
    SerialUSB.print(linearStepper.speed());
    SerialUSB.print(',');
    SerialUSB.print(linearStepper.acceleration());
    SerialUSB.print(',');
    SerialUSB.print(linearStepper.distanceToGo());
    SerialUSB.print(',');
    SerialUSB.println(linearStepper.targetPosition());


  } else if(teensyToArduinoTransfer.status < 0) {
    SerialUSB.print("ERROR: ");
    if(teensyToArduinoTransfer.status == -1)
      SerialUSB.println(F("CRC_ERROR"));
    else if(teensyToArduinoTransfer.status == -2)
      SerialUSB.println(F("PAYLOAD_ERROR"));
    else if(teensyToArduinoTransfer.status == -3)
      SerialUSB.println(F("STOP_BYTE_ERROR"));
  }

  // delay(1000); // Commented this out to make stepper motor run more smoothly
}