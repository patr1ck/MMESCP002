#include "SerialTransfer.h"
#include <Scheduler.h>
#include <AccelStepper.h>
#include <EasyButton.h>
#include <ModbusMaster.h>


// ------------- Configuration Section -------------

const int targetTemperature = 20;   // In degrees C. Min is 0, Max is 700.
const float DesiredPosition1 = -5.0; // Milimeters from current position

// ------------ End Configuration Section ----------


// Buttons
#define GREEN_BUTTON_PIN 22
#define RED_BUTTON_PIN 23
#define LIMIT_SWITCH_START_SIDE 24
#define LIMIT_SWITCH_END_SIDE 25
int debounce = 50;
bool pullup = true;
bool invert = true;
EasyButton startButton(GREEN_BUTTON_PIN, debounce, pullup, false);
bool startWasPressed = false;
EasyButton stopButton(RED_BUTTON_PIN, debounce, pullup, false);
bool stopWasPressed = false;
EasyButton limitSwitchStart(LIMIT_SWITCH_START_SIDE, debounce, pullup, invert);
EasyButton limitSwitchEnd(LIMIT_SWITCH_END_SIDE, debounce, pullup, invert);

// Temp Controller
#define MAX485_DE 7         //DE Pin of Max485
#define MAX485_RE_NEG 6     //RE Pin of Max485
#define PresentValueAddress 0x3E8
ModbusMaster tempControllerMM;
int presentValue = 0;

// Teensy Serial Communication
struct STRUCT {
  float temp1;
  float temp2;
  float temp3;
  float temp4;
  float temp5;
} data;
SerialTransfer teensyToArduinoTransfer;

// Stepper Motor Driver
const int dirPin1 = 2;
const int stepPin1 = 3;
AccelStepper linearStepper(AccelStepper::DRIVER, stepPin1, dirPin1);
int desiredPosition = -1; // User defined desired position
const int stepsPerRotation = 200;
const int gearboxRatio = 100;
const float leadOfScrew = 4.0;
const float estimatedSpeed = 800.0; // Estimated speed in steps per second
const float estimatedAcceleration = 300.0; // Estimated acceleration in steps per second squared
unsigned long estimatedRunTime = 0; // Global variable for estimated run time
unsigned long lastUpdateTime = 0; // Last time the estimated run time was updated
const unsigned long updateInterval = 1000; // Update interval for estimated run time in milliseconds
const float stepSize = 1.0 / (stepsPerRotation * gearboxRatio / leadOfScrew); // Step size in millimeters


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
  int desiredPosition = ((stepsPerRotation * gearboxRatio) / leadOfScrew) * DesiredPosition1; // Calculate desired position in steps
  linearStepper.moveTo(desiredPosition); // Move the stepper motor to the desired position

  lastUpdateTime = millis(); // Initialize the last update time
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

  SerialUSB.println("Time(s),Temp1,Temp2,Temp3,Temp4,Temp5,StepperPosition(mm),StepperSpeed(mm/s),StepperAcceleration(mm/s^2),StepperDistanceToGo(mm),EstimatedRunTime(s)");
}

void loop() {  

  if (stopWasPressed == false && startWasPressed == true) {
    linearStepper.run(); // Stepper motor run command moved here
  }

  // Update the estimated run time at the specified interval
  if (millis() - lastUpdateTime >= updateInterval) {
    // Calculate estimated run time
    float distanceToTravel = abs(linearStepper.distanceToGo());
    float timeToReachPosition = sqrt(2.0 * distanceToTravel / estimatedAcceleration) + (distanceToTravel / estimatedSpeed);
    estimatedRunTime = (unsigned long)(timeToReachPosition * 1000.0); // Convert to milliseconds

    lastUpdateTime = millis(); // Update the last update time
  }

  // If we are able to get temperature data from the teensy thermocouple controller...
  if(teensyToArduinoTransfer.available()) {
    uint16_t recSize = 0;
    recSize = teensyToArduinoTransfer.rxObj(data, recSize);
    
       // Print the data values
    SerialUSB.print(millis() / 1000.0, 2); // Print current time in seconds
    SerialUSB.print(',');
    SerialUSB.print(data.temp1, 2); // Print temperature 1
    SerialUSB.print(',');
    SerialUSB.print(data.temp2, 2); // Print temperature 2
    SerialUSB.print(',');
    SerialUSB.print(data.temp3, 2); // Print temperature 3
    SerialUSB.print(',');
    SerialUSB.print(data.temp4, 2); // Print temperature 4
    SerialUSB.print(',');
    SerialUSB.print(data.temp5, 2); // Print temperature 5
    SerialUSB.print(',');
    SerialUSB.print(linearStepper.currentPosition() * stepSize, 3); // Print stepper position in millimeters
    SerialUSB.print(',');
    SerialUSB.print(linearStepper.speed() * stepSize, 2); // Print stepper speed in millimeters per second
    SerialUSB.print(',');
    SerialUSB.print(linearStepper.acceleration() * stepSize, 2); // Print stepper acceleration in millimeters per second squared
    SerialUSB.print(',');
    SerialUSB.print(linearStepper.distanceToGo() * stepSize, 3); // Print stepper distance to go in millimeters
    SerialUSB.print(',');
    SerialUSB.print(estimatedRunTime / 1000.0, 3); // Print estimated run time in seconds
    SerialUSB.println();

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
  startWasPressed = true;
  stopWasPressed = false;
}

void stopPressed() {
  SerialUSB.println("Stop button has been pressed!");
  stopWasPressed = true;
}

void limitStartHit() {
  SerialUSB.println("Limit Start button has been pressed!");
  stopWasPressed = true;
}

void limitEndHit() {
  SerialUSB.println("Limit End button has been pressed!");
  stopWasPressed = true;
}

void loop2() {
  startButton.read();
  stopButton.read();
  limitSwitchStart.read();
  limitSwitchEnd.read();
  yield();
}
// Hello world