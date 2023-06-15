# MMESCP002
Code for the 2022-2023 PSU Capstone Project MMESCP-002

The most up-to-date versions of this code can be found at https://github.com/patr1ck/MMESCP002

If you are the new owner of this project, please contact @patr1ck through PSU channels (pg9@pdx.edu) and he will transfer this project to you.

There are two Arduino files in this repository:
	* ThermocoupleController is meant to run on the Teensy which is attatched to the TTCHost board, and communicated temparetunes back to the primary QTFController.
	* QTFController is responsible for moving the linear actuator and modifying the temperature of the furnace as needed. It should be run on the primary Arduino Due.
	
# How to Pyrolyze

## Arduino Setup

### QTF Controller (Main System Controller)

To do development on the main system controller, you'll need to install several libraries, all of which can be found in the Arduino Library Manager:

* SerialTransfer (Used to read from TTCHost over serial)
* Scheduler (Used to run multiple loops)
* EasyButton (Used to receive push button callbacks)
* ModbusMaster (Used to communicate with the Autonics Temperature Controller)
* AccelStepper (Used to control the stepper motor driver)

Once you have these installed, you can build/upload the QTFController code found in /QTFController


### Thermocouple Controller (TTCHost)

To do development on the TTCHost board, you will need the Teensy board manager. Follow the instructions at the top of the page here (Add the board manager URL to your Arudino app, and then install the Teensy board manager):

https://www.pjrc.com/teensy/td_download.html

You will also need two libraries: GyverMAX6675 and SerialTransfer. Install them from the Arduino Library Manager.

You should then be able to build/upload the ThermocoupleController code found in /ThermocoupleController.


## Running the Quartz Tube Furnace

At the top of the QTFController file, you'll find the "Configuration Section" (Line 8), which allows you to set up parameters for running the device.

`resetMode`: When set to true, will run the device back to the starting position. This should be set to false for experiment runs.

`targetTemperature`: Sets the goal temperature on the inside of the quartz tube.

`runTime`: Sets the running time of the experiment. This is the length of time it will take for the furnace to pass over the length of the biomass sled.

Aside from the Arduino configuration, the device has a control panel with three buttons:

* Green button: Starts the linear actuator.
* Red button: Stops the linear actuator.
* Emergency stop button (large red button): Shuts down the entire system.

Performing a basic experiment is done like this:

1. On a computer, install and setup the Arduino for the QTF Controller as listed above (only needs to be done once). Make sure this computer is plugged into the Arduino Due – it's the only USB cable coming from the controls box.
1. Make sure the device is reset: Set resetMode = true in the QTFController, upload, press the green button to start the linear actuator movement, and wait until it stops back in the starting position.
1. Set your temperature and run time in the configuration section of the QTFController.ino file. Set resetMode back to false.
1. Upload that file to the Arduino.
1. The heater will begin heating. You can monitor the temperature in the Arduino output and on the front of the heater monitor.
1. When heater reaches the desired temperature (which may take a while...), press the green button to start the linear actuator's motion.


## Areas of Improvement

Because of the timing limitations of the capstone project and the long duration of tests, the code has not been run to completion very many times, and may contain issues. Some of the areas for improvement include:

* Temperature Interpolation: Currently, the temperature is read from the TC nearest to the heater. This could be improved by taking a weighted average of the two nearest TCs instead.
* Bias calculations: We presently have a hard-coded bias calculation, but determining one dynamically would be better. Doing so would require essentially reimplementing PID control in the Arduino code.
* Auto-start: Having the experiment start automatically once the appropriate temperature is reached, instead of having to press the green start button.

