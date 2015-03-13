# Introduction #

This package provides a c++ wrapper for phidgets.

# C++ bindings #
  * To simply use the C++ bindings it is all in one header file.
  * Currently supported are RFID, LCD and Servo Phidgets more are on their way.
  * To use the bindings simply copy phidgetspp.hh from the phidgetspp directory and :
```
#include "phidgetspp.hh"
```

# Example Programs #

## rfid\_demo ##

How to run the demo:
  1. Make sure that the RFID phidget is plugged in
  1. Make sure that the permissions have been set see [PhidgetsSetup](PhidgetsSetup.md)
  1. execute the following `rosrun phidgetspp rfid_demo`
    * The program will flash the onboard LED and the two output LEDs
    * It will provide show some example accessors
    * It will state the last known RFID
    * It will notify on detecting an RFID and losing contact.
    * It's output will look like this:
```
Opening
Attaching . . . timeout in 30 seconds
Attach handler ran!
Success Ataching
Phidget type: PhidgetRFID
Phidget label: PhidgetRFID
Phidget name: Phidget RFID 2-output
Phidget Version: 206
Phidget Serial number: 56461
Library Version: Phidget21 - Version 2.1.6 - Built Oct 27 2009 19:30:35
Turning ANtenna On
Number of outputs is: 2
Output states 1: 0 2:0
Output states 1: 1 2:1
antenna is on: 1
Blinking LED and waiting until user termination
Last Tag seen was: 3218669571
Last Tag seen was: 3218669571
Last Tag seen was: 3218669571
Default gotTag response! 1351812609
Default lostTag response! 1351812609
Last Tag seen was: 1351812609
Last Tag seen was: 1351812609
Default gotTag response! 1351812609
Last Tag seen was: 1351812609
Last Tag seen was: 1351812609
Default lostTag response! 1351812609
Last Tag seen was: 1351812609
Last Tag seen was: 1351812609
```

## lcd\_demo ##
How to run the demo:
  1. Make sure that the RFID phidget is plugged in
  1. Make sure that the permissions have been set see [PhidgetsSetup](PhidgetsSetup.md)
  1. execute the following `rosrun phidgetspp lcd_demo`
    * You will see the following while the display shows off the capabilities.
```
connecting to textLCDPhidget type: PhidgetTextLCD
Phidget label: PhidgetTextLCD
Phidget name: Phidget TextLCD
Phidget Version: 123
Phidget Serial number: 67516
Library Version: Phidget21 - Version 2.1.6 - Built Oct 28 2009 22:35:14
Row count is 2
Column count is 20
Displaying all characters TOP and BOTTOM
Flashing backlight
Adjusting contrast
```