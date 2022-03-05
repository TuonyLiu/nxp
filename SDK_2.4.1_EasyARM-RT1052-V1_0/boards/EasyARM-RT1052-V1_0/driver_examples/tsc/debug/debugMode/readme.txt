Overview
========
The fourWireModeInterrupt example shows how to use TSC driver in 4-wire debug modes:
The touch screen controller needs to co-work with ADC and TSC analogue.

In this example , we make use of the available electrodes on board to show driver usage.
1. Firstly, before TSC starts work, ADC driver configure ADC_HCx;
2. Then, we configure TSC module to work in debug mode.
3. Then, we configure six states(1st-pre-charge, 1st-detect, x-measure, y-measure, and 
   2nd-pre-charge, 2nd-detect) to simulate non-debug(hardware) mode.
4. Lastly, if measure data is valid, serial terminal prints x/y-coordinate information.


Toolchain supported
===================
- IAR embedded Workbench 7.80.4
- GCC ARM Embedded 2016-5.4-q3

Hardware requirements
=====================
- Micro USB cable
- EasyARM-RT1052  board
- 5V power supply
- TFT 4.3A
- Personal Computer

Board settings
==============
No special settings are required.

Prepare the Demo
================
1.  Connect 5V power supply and JLink Plus to the board, switch SW2001 to power on the board.
2.  Connect a USB cable between the host PC and the J1901 USB port on the target board.
3.  Connect LCD8000-43T to the J901 LCD expansion port connector.
4.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Download the program to the target board.
5.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
================
When running successfully, the log output in terminal shall be similar as below:

TSC DebugMode Example Start!
ADC_DoAntoCalibration() Done.
Please touch screen!
x = 22, y = 4075
x = 24, y = 4082
x = 24, y = 4083
x = 25, y = 4082
x = 24, y = 4084
x = 21, y = 4075
x = 21, y = 4080
...

Customization options
=====================

