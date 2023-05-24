#include "SerialTransfer.h"
#include <AccelStepper.h>
#include <ModbusMaster.h>

#define MAX485_DE 7         //DE Pin of Max485
#define MAX485_RE_NEG 6     //RE Pin of Max485

ModbusMaster tempControllerMM;
int tempControllerValue;

void fnExecRead() {

  int addressToRead = 0x3E8; // The address of Present Value

  // SerialUSB.println("calling readInputRegisters...");
  uint8_t ReadResult = tempControllerMM.readInputRegisters(addressToRead, 1);
  // SerialUSB.println("returned from readInputRegisters...");
  int dataNya;

  delay(600); //Give the Arduino time to finish the reading

  if (ReadResult == tempControllerMM.ku8MBSuccess) { //Check whether Max485 successfuly reads the device
    dataNya = tempControllerMM.getResponseBuffer(0); //Get the data from Max485 Response Buffer
    char StrBufReadResult[10];

    itoa( dataNya, StrBufReadResult, 10 ); //Convert the data to string
    char StrReadRow[45]; //Variable to store read result per row

    strcat(StrReadRow, "Read temperature value: ");
    strcat(StrReadRow, StrBufReadResult );

    // Increment the temp controller (just a test for now)
    tempControllerValue = atoi(StrBufReadResult) + 1;

    SerialUSB.println(StrReadRow); //for debugging purpose, write to Serial
  } else { //Check whether Max485 cannot reads the device and an error has arise
    SerialUSB.print("Error = "); //for debugging purpose, write to Serial
    SerialUSB.println(ReadResult); //for debugging purpose, write to Serial
  }
}

void fnExecWrite() {
  uint8_t WriteResult; //Variable to get Read result from Max485
  int dataTulisNya; //Variable to save the value written to device

  WriteResult = tempControllerMM.writeSingleRegister(0, tempControllerValue);

  delay(600); //Give Arduino some time to finish write command properly

  if (WriteResult == tempControllerMM.ku8MBSuccess) { //Check whether Max485 successfuly write to the device
    dataTulisNya = tempControllerMM.getResponseBuffer(0); //Get the data from Max485 Response Buffer
    SerialUSB.print("Successfully wrote new target value temp: ");
    SerialUSB.println(tempControllerValue);
  }

  delay (500);
}

void preTransmission() { //Function for setting stste of Pins DE & RE of RS-485 to HIGH
  delay(500);
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() { //Function for setting stste of Pins DE & RE of RS-485 to LOW
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  delay(500);
}



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
  linearStepper.setAcceleration(0);
  // linearStepper.moveTo(((200*100)/4)*300); 

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  Serial3.begin(9600, SERIAL_8E1); // 8 bits, even parity, 1 stop bit
  while (!Serial3) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  tempControllerMM.begin(1, Serial3);
  tempControllerMM.preTransmission(preTransmission);
  tempControllerMM.postTransmission(postTransmission);

  SerialUSB.println("Time(s),Temp1,Temp2,Temp3,Temp4,Temp5,StepperPosition,StepperSpeed,StepperAcceleration,StepperDistanceToGo,StepperDirection,StepperTargetPosition");
  delay(2000);
}

void loop() {
  linearStepper.run(); // Stepper motor run command moved here

  if(teensyToArduinoTransfer.available()) {
    uint16_t recSize = 0;
    recSize = teensyToArduinoTransfer.rxObj(data, recSize);
    
    SerialUSB.print(millis()/1000.0); // time printed in seconds
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

  SerialUSB.println("calling fnExecRead...");
  fnExecRead(); //Read from Max485
  fnExecWrite();

  delay(1000); // Commented this out to make stepper motor run more smoothly
}