# MMESCP002
Code for the 2022-2023 PSU Capstone Project MMESCP-002

The most up-to-date versions of this code can be found at https://github.com/patr1ck/MMESCP002

There are two Arduino files in this repository:
	* ThermocoupleController is meant to run on the Teensy which is attatched to the TTCHost board, and communicated temparetunes back to the primary QTFController.
	* QTFController is responsible for moving the linear actuator and modifying the temperature of the furnace as needed. It should be run on the primary Arduino Due.