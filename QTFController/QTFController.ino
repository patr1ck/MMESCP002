#include "SerialTransfer.h"
#include <Scheduler.h>
#include <AccelStepper.h>
#include <EasyButton.h>
#include <ModbusMaster.h>


// ------------- Configuration Section -------------

int targetTemperature = 20;   // In degrees C. Min is 0, Max is 700.
int runTime = 100;            // In minutes.

// ------------ End Configuration Section ----------



#define MAX485_DE 7         //DE Pin of Max485
#define MAX485_RE_NEG 6     //RE Pin of Max485
#define GREEN_BUTTON_PIN 22
#define RED_BUTTON_PIN 23
#define LIMIT_SWITCH_START_SIDE 24
#define LIMIT_SWITCH_END_SIDE 25

#define MySerial Serial3
#define PresentValueAddress 0x3E8

int debounce = 50;
bool pullup = true;
bool invert = true;
EasyButton startButton(GREEN_BUTTON_PIN, debounce, pullup, invert);
EasyButton stopButton(RED_BUTTON_PIN, debounce, pullup, invert);
EasyButton limitSwitchStart(LIMIT_SWITCH_START_SIDE, debounce, pullup, invert);
EasyButton limitSwitchEnd(LIMIT_SWITCH_END_SIDE, debounce, pullup, invert);


ModbusMaster tempControllerMM;
// ModbusSerial tempControllerMM(MySerial, 1, MAX485_DE);
int presentValue = 0;

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

int desiredPosition = -1; // User defined desired position


void readPresentValue() {

  int addressToRead = PresentValueAddress; // The address of Present Value (See Section 2.3 "Read Input Registers" of TK Series User Manual for Communication)

  uint8_t ReadResult = tempControllerMM.readInputRegisters(addressToRead, 1);
  int dataNya;

  delay(600); //Give the Arduino time to finish the reading

  if (ReadResult == tempControllerMM.ku8MBSuccess) { //Check whether Max485 successfuly reads the device
    dataNya = tempControllerMM.getResponseBuffer(0); //Get the data from Max485 Response Buffer
    char StrBufReadResult[10];

    itoa( dataNya, StrBufReadResult, 10 ); //Convert the data to string
    char StrReadRow[45]; //Variable to store read result per row

    // Set the global variable
    presentValue = atoi(StrBufReadResult);

    // Log the result
    strcat(StrReadRow, "Read present value: ");
    strcat(StrReadRow, StrBufReadResult);
    SerialUSB.println(StrReadRow); 
  } else { // There was some kind of error, so log it
    SerialUSB.print("Error in readPresentValue = ");
    SerialUSB.println(ReadResult);
  }
}

void writeTargetTemperature(int targetTemp) {
  uint8_t WriteResult; //Variable to get Read result from Max485
  int dataTulisNya; //Variable to save the value written to device

  WriteResult = tempControllerMM.writeSingleRegister(0, targetTemp);

  delay(600); //Give Arduino some time to finish write command properly

  if (WriteResult == tempControllerMM.ku8MBSuccess) { //Check whether Max485 successfuly write to the device
    dataTulisNya = tempControllerMM.getResponseBuffer(0); //Get the data from Max485 Response Buffer
    SerialUSB.print("Successfully wrote new target value temp: ");
    SerialUSB.println(targetTemp);
  } else {
    SerialUSB.print("Error in writeTargetTemperature = ");
    SerialUSB.println(WriteResult);
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

void setup() {
  SerialUSB.begin(9600);
  Serial1.begin(9600);
  teensyToArduinoTransfer.begin(Serial1);

  linearStepper.setMaxSpeed(800); 
  linearStepper.setSpeed(800);
  linearStepper.setCurrentPosition(0);
  linearStepper.setAcceleration(300);

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  Serial3.begin(9600, SERIAL_8E1); // 8 bits, even parity, 1 stop bit
  while (!Serial3); // Wait for Serial3 to be ready.

  tempControllerMM.begin(1, Serial3);
  tempControllerMM.preTransmission(preTransmission);
  tempControllerMM.postTransmission(postTransmission);
  delay(1000);

  startButton.begin();
  startButton.onPressed(startPressed);

  stopButton.begin();
  stopButton.onPressed(stopPressed);

  limitSwitchStart.begin();
  limitSwitchStart.onPressed(limitStartHit);
  
  limitSwitchEnd.begin();
  limitSwitchEnd.onPressed(limitEndHit);
  
  Scheduler.startLoop(loop2);

  SerialUSB.println("Enter the desired position when ready:");
}

void loop() {
  if(desiredPosition == -1 && SerialUSB.available() > 0) {
    desiredPosition = SerialUSB.parseInt();
    linearStepper.moveTo(((200*100)/4)*desiredPosition); 
    SerialUSB.println("Time(s),Temp1,Temp2,Temp3,Temp4,Temp5,StepperPosition,DesiredPosition,TimeRemaining");
    delay(500);
  }
  
  linearStepper.run(); // Stepper motor run command moved here

  if(teensyToArduinoTransfer.available()) {
    uint16_t recSize = 0;
    recSize = teensyToArduinoTransfer.rxObj(data, recSize);
    
    SerialUSB.print(millis()/1000.0);
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
    SerialUSB.print(desiredPosition);
    SerialUSB.print(',');
    int timeRemaining = desiredPosition - linearStepper.currentPosition();
    SerialUSB.print(timeRemaining < 0 ? 0 : timeRemaining);
    SerialUSB.print(',');
    SerialUSB.println(presentValue);

  } else if(teensyToArduinoTransfer.status < 0) {
    SerialUSB.print("ERROR: ");
    if(teensyToArduinoTransfer.status == -1)
      SerialUSB.println(F("CRC_ERROR"));
    else if(teensyToArduinoTransfer.status == -2)
      SerialUSB.println(F("PAYLOAD_ERROR"));
    else if(teensyToArduinoTransfer.status == -3)
      SerialUSB.println(F("STOP_BYTE_ERROR"));
  }

  yield();
}

void startPressed() {
  SerialUSB.println("Start button has been pressed!");
  readPresentValue(); // Debug Only
}

void stopPressed() {
  SerialUSB.println("Stop button has been pressed!");
}

void limitStartHit() {
  SerialUSB.println("Limit Start button has been pressed!");
}

void limitEndHit() {
  SerialUSB.println("Limit End button has been pressed!");
}

void loop2() {
  startButton.read();
  stopButton.read();
  limitSwitchStart.read();
  limitSwitchEnd.read();
  yield();
}