# ADF5355_frequency_generator

ADF5355 Bluepill Frequency Generator

Code to use an ADF5355 as a signal generator from 52 to 13600 MHz

Based on that of DD7LP and GM8BJF (See below) with minor 
modifications by GI1MIC to run on a STM32F103CB/C8 Bluepill.

The code changes were originally required because the original code
kept locking up on SPI writes.
 
The following code is based on using the onboard ADF5355 25Mhz clock

Since the 25Mhz clock on my board was closer to 24.99395Mhz I added
a calibration feature. The current calibration offset is shown
during start-up.

To change the offset, press the frequency switch during power on and
release when the display becomes active. Then adjust as required -
a minus offset is subtracted from your crystal while a positive offset
is added. 

The offset is stored in internal eeprom.

Calibration can be accomplished  against a known receiver (higher the
known RX frequency the better)

The board I use was programmed with the maple DFU
bootloader for ease of programming.

The Arduino IDE (V1.8.13) was configured for a generic SMT32F1xx

Blue pill Pins:
Encoder 1 Clk - PB11, DT - PB10, SWT - PB1, + - 3.3V, GND - GND
Encoder 2 Clk - PA3, DT - PA2, SWT - PA1, + - 3.3V, GND - GND
I2C OLED  SDA - PB7, SCL - PB6, + - 3.3V, GND - GND
ADF5355  CLK-PA5, MUX-PB0, LE-PA4, DAT-PA7, 3.3V - 3.3V, GND - GND, Barrel Jack - 6V (or 5V on the Bluepill)



