Open Source Sparta ION
=============
There's a bunch of (Dutch) blog posts that go with this: (http://infant.tweakblogs.net/blog/cat/2875) (Awesome work InfantEudora)
Also check out the Discord: (https://discord.gg/anZees2)

This repository contains the following:

 folder       |  descr
--------------|-------------------------------------------------------------
 eagle        | All relevant eagle files, BOM lists.
 ion_firmware | AVR Studio projects for the different kinds of firmware.
 ion_software | Software for testing, or uploading hex files.
 lib          | Files used in either firmware or software.
 lib_ion      | Files used for Sparta ION specific code.

All files in this repository are released under GNU GPLv3 (https://www.gnu.org/copyleft/gpl.html)

Software used:
----------------
 - Atmel Studio 7.0.2397 (https://www.microchip.com/mplab/avr-support/atmel-studio-7)
 - GCC 9.3.0
 - Windows: Cygwin(https://cygwin.com/install.html)
 - Eagle 8.6.3 (http://www.cadsoftusa.com/)

Firmware:
----------------

The firmware running your bike's motor, consists of three parts:
 - A bootloader, started on powerup.
 - The actual firmware, called application executed by the bootloader.
 - Configuration memory, stored in ROM.

The bootloader uses two blocks to identify the harware it's running on, and what firmware should be loaded.
The firware uses one block to read/write settings, which are currently the strain gauge calibration.

If you wan't to build a control PCB, use the latest revision of 3phasecntrl.

New Hardare Revision Wishlist:
Motor Driver:
 - Modify 3phasecntrl for XHP motors.
BMS:
 - Modifiy BMS for Li-Ion/Li-Po support.

New Software Revision Wishlist:
 - Enhance CU3 Display support
 - Write BMS code
 - Modify Bootloader for BMS

