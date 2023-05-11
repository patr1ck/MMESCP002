// MMESCP-002: ThermocoupleController
// May 11 2023

// This file handles reading temperature data from the MAX6675 amplifiers
// and communicating that data back to the main controller. It should be run
// only on the Teensy (4.1) device.

#include <GyverMAX6675.h>
#include "SerialTransfer.h"

#define CLK_PIN 13  // CLK
#define DATA_PIN 12 // SO

// Data pins
GyverMAX6675<CLK_PIN, DATA_PIN, 6> sens1;
GyverMAX6675<CLK_PIN, DATA_PIN, 7> sens2;
GyverMAX6675<CLK_PIN, DATA_PIN, 8> sens3;
GyverMAX6675<CLK_PIN, DATA_PIN, 9> sens4;
GyverMAX6675<CLK_PIN, DATA_PIN, 10> sens5;

struct STRUCT {
  float temp1;
  float temp2;
  float temp3;
  float temp4;
  float temp5;
} data;

SerialTransfer teensyToArduinoTransfer;

void setup() {
  //Setup logging
  Serial.begin(9600);

  //Setup the Teensy -> Arduino connection
  Serial1.begin(9600);
  teensyToArduinoTransfer.begin(Serial1);
  delay(500);
}

void loop() {

  if (sens1.readTemp()) { // Try to read temperature
    Serial.print("Temp 1: ");
    data.temp1 = sens1.getTemp();
    Serial.print(data.temp1);
    Serial.println(" *C");
  } else {
    Serial.println("Error Reading Temp 1"); // Log error if can't read
    data.temp1 = 0;
  }

  if (sens2.readTemp()) { // Try to read temperature
    Serial.print("Temp 2: ");
    data.temp2 = sens2.getTemp();
    Serial.print(data.temp2);
    Serial.println(" *C");
  } else {
    Serial.println("Error Reading Temp 2"); // Log error if can't read
    data.temp2 = 0;
  }

  if (sens3.readTemp()) { // Try to read temperature
    Serial.print("Temp 3: ");
    data.temp3 = sens3.getTemp();
    Serial.print(data.temp3);
    Serial.println(" *C");
  } else {
    Serial.println("Error Reading Temp 3"); // Log error if can't read
    data.temp3 = 0;
  }

    if (sens4.readTemp()) { // Try to read temperature
    Serial.print("Temp 4: ");
    data.temp4 = sens4.getTemp();
    Serial.print(data.temp4);
    Serial.println(" *C");
  } else {
    Serial.println("Error Reading Temp 4"); // Log error if can't read
    data.temp4 = 0;
  }

  if (sens5.readTemp()) { // Try to read temperature
    Serial.print("Temp 5: ");
    data.temp5 = sens5.getTemp();
    Serial.print(data.temp5);
    Serial.println(" *C");
  } else {
    Serial.println("Error Reading Temp 5"); // Log error if can't read
    data.temp5 = 0;
  }

  // All the temps have been read. Send the data.
  uint16_t sendSize = 0;
  sendSize = teensyToArduinoTransfer.txObj(data, sendSize);
  teensyToArduinoTransfer.sendData(sendSize);

  delay(1000); // Can only refresh temps once per second
}