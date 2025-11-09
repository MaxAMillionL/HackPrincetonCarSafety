### ClearDrive ###
## Setup ##
To use this program, you need to install the following programs:

* CARLA's open source simulator: https://carla.readthedocs.io/en/latest/download/
* The Carla library from python
* The required dependencies outlines in the CARLA download
* The serial library for communication between the code and hardware

You must also have the following physical items:

* Arduino
* Air Quality Sensor
* Means of connecting (i.e., wires, shield, USB-C, etc.)

## The Process ##
To run this program, first, open up the CarlaUE4.exe file to boot up the simulation. 
Then, run the python file given in the github directory. This will open up a window
Indicating the current air quality if all is correct. If it does not display a number
other than 0, check your connection or change your COM port to the arduino.

You now must tab into the air quality window to drive the car. once the air quality
sensor detects bad air, the car will enter self-driving mode and you will completed a
loop of the simulation. Happy coding!
