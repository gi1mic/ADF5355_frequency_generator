# ADF5355_frequency_generator

ADF5355 Bluepill Frequency Generator

![alt text](https://github.com/gi1mic/ADF5355_frequency_generator/blob/main/images/screenShot.jpg?raw=true)

This is code that allows an ADF5355 module to be used as a signal generator from 52Mhz to 1,3600MHz 

The code is based on that of GM8BJF (https://github.com/gm8bjf/ADF5355_sig_gen) which in-turn was based on code from DD7LP.

I originally took on the task of modifying GM8BJF's code because it did not work on my board. His code kept locking up on SPI writes even though it appeared correct (fake STM32?). I also wanted to use a different, larger,  display plus the on-board ADF5355 25Mhz clock rather than an external clock.

The code now runs on a generic STM32F103CB/C8 "Bluepill" with a 1.3" 128x64 OLED display (SH1106 chip set). The code changes
involved moving to the u8g2 library which makes it easier to use alternative displays and changing the encoders inteface to use interrupts so they have smoother control.

Once using the generator I found the ADF5355 on-board 25Mhz XTAL was actually closer to 24.99395Mhz. So to get around this problem I added a calibration feature allowing an offset to be set and stored in flash. The active calibration offset is shown during start-up.

To change the offset, press the frequency switch during power on and release when the display becomes active. Then adjust the offset as required. A minus offset is subtracted from your crystal while a positive offset is added. The offset is stored in internal EEPROM. Calibration can be accomplished  against a known receiver (higher the RX frequency the better)

In use one rotary encoder changes the frequency up/down while pressing it jumps to preset frequencies. The other sets tuning step size while pressing it sets output level between -5dBm to +2dBm.

At the request of VE7XDT I added a digital input (PB5) to control an Int/Ext message on the display to signify where the CLK source is coming from.

I also added "long press" features to the encoders to toggle the output state of two digital outputs (PB3/PB4). These can be used to select a clock source or to control a relay. 

The Bluepill board I have was programmed with a maple DFU boot-loader to simplify programming directly from the Arduino IDE.

The Arduino IDE (V1.8.13) was configured for a generic SMT32F1xx, C8 128K, and the upload method set to "Maple DFU Bootloader 2.0".

## Blue pill Pins:
**Encoder 1**

-     Clk - PB11 
-     DT  - PB10
-     SWT - PB1
-     +   - 3.3V
-     GND - GND

**Encoder 2**
-     Clk - PA3
-     DT  - PA2
-     SWT - PA1
-     +   - 3.3V
-     GND - GND

**1.3" 128x64 SH1103 I2C OLED (Other 128x64 displays can be used check the code)**
-     SDA - PB7
-     SCL - PB6
-     +   - 3.3V
-     GND - GND
    
**ADF5355**
-     CLK - PA5
-     MUX - PB0
-     LE  - PA4
-     DAT - PA7
-     3.3V - 3.3V
-     GND - GND
-     Barrel Jack - 6V or 5V from the Bluepill

**Digital Outs**
-     Long Press 1 - PB4
-     Long press 2 - PB5


**Ext/Int CLK display**
-     Internal/External input - PB5





