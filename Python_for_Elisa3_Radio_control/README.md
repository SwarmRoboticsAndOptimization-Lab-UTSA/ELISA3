# elisa3_remote_library_python
This library simplify the implementation of applications that need to control one or more robots remotely thorugh the radio base-station connected to the computer. The library provides functions to set all actuators and receive sensors data from the robots.

Requirements:
* tested on Python 3
* pip3 install libusb_package
* pip3 install pyusb

## Lattice Formation Folder:
    * This contains the working demo of a lattice formation using elisa 3 robots
    * To run this code make sure you update the list of robots that you will use on elisa3_LatticeFormation.py
    * e.g. robotAddr = [3901,4469,4021,3823,4060,3890,4101,3829,3904,3869]
    * elisa3.py and utils.py are files with dependencies used to controll and calculate different aspects of their location and movement
    * This code uses libusb to communicate with the elisa usb, in order to run open a terminal in the location of the files and execute:
    - sudo ../../../ElisaEnv/bin/python3 elisa3_LatticeFormation.py
    * this is to call the python environment that has all the dependencies in sudo mode so that we can have the permissions to control the robots using a usb

## UTSA Formation Folder:
    * This contains the working demo of a U.T.S.A formation using elisa 3 robots
    * To run this code make sure you update the list of robots that you will use on elisa3_utsaFormation.py
    * e.g. robotAddr = [3901,4469,4021,3823,4060,3890,4101,3829,3904,3869]
    * elisa3.py and NewObstacleAvoidanceUtils.py  are files with dependencies used to control and calculate different aspects of their location and movement.
    * This code uses libusb to communicate with the elisa usb, in order to run open a terminal in the location of the files and execute
    - sudo ../../../ElisaEnv/bin/python3 elisa3_utsaFormation.py
    * this is to call the python environment that has all the dependencies in sudo mode so that we can have the permissions to control the robots using a usb

## How to know what robots I will use?
    * There is a script called TestQr_Codes.py
    * When executed the camera will be oppened and will display what robots are detected, it also includes bars to adjust camera values, this are helpful when camera configuration is required to improve detection on the qr codes. If changes are made make sure to save the values so that it can be updated on the intialize_camera function on the utils of each project.

## apriltag-gen.py
    * This script is to generate new qr codes to be printed
    * Update the qr codes that you want to print
    * Run the code, it will save them on a file
    * to display them use the following command
    * Use the displayed image to copy and paste on word, it usually gets the minimum size that works on elisa3 robots

## Elisa3 Ready to Use.txt
    * This is a list of all the elisas that has been opened and put the selector in position F, this is required by  the robots to correctly work with the radio signals.

## elisa3_basic_example.py
    * This is a script provided by GCtronics, where they show how the communication is handled and what information can be extracted from the robots using the radio signal. This is a good example to test functionalities, and a good starting point when creating new code for the elisas.
    * Make sure to have a copy of the original.

## Elisa3_Testing_MaxSwarm.py
    * this script has a simple controll command send to the robots on the list, it is used to test the communication with increased number of robots, up to now it has been tested with 50 at the same time without showing delay.
