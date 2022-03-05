Overview
========
The CSI RGB565 project shows how to receive the camera data using CSI driver.
In this example, you will see the camera input image shown in the LCD. Please
note that the camera input image resolution might be smaller than the LCD
panel, so part of the LCD panel might be blank.

Toolchain supported
===================
- Keil MDK 5.24a
- IAR embedded Workbench 8.22.2
- GCC ARM Embedded 7-2017-q4-major
- MCUXpresso10.2.0

Hardware requirements
=====================
- Mini/micro USB cable
- EasyARM-RT1052 board
- Personal Computer
- TFT-4.3 CAP LCD board
- MT9M114 module
This example supports OV7725 & MT9M114, if OV7725 is used, then MT9M114 is not necessary.

Board settings
==============
1. Connect the TFT-4.3 CAP board.
2. Connect the MT9M114 or OV7725 camera module.

Prepare the Demo
================
1.  Connect a USB cable between the host PC and the OpenSDA USB port on the target board. 
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Download the program to the target board.
5.  Launch the debugger in your IDE to begin running the demo.

Note: If OV7725 is used, change the macro APP_CAMERA_TYPE to APP_CAMERA_OV7725 in
qr_decode.c.
      If MT9M114 is used, change the macro APP_CAMERA_TYPE to APP_CAMERA_MT9M114 in
qr_decode.c.

Running the demo
================
When the demo runs successfully, the camera received pictures are shown in the LCD.

These instructions are displayed/shown on the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

QR Decoder example start...
Success!
Format: QR_CODE
http://ucmp.sf-express.com/service/weixin/activity/wx_b2sf_order?p1=071340041973
Success!
Format: QR_CODE
http://ucmp.sf-express.com/service/weixin/activity/wx_b2sf_order?p1=071340041973

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

=====================

