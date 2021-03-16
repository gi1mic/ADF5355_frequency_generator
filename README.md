# ADF5355_frequency_generator

ADF5355 Bluepill Frequency Generator

Code to use an ADF5355 as a signal generator from 52 to 13600 MHz 

The code is based on that of GM8BJF (https://github.com/gm8bjf/ADF5355_sig_gen) which in-turn was based on code from DD7LP.

Updated by me (GI1MIC) to run on a STM32F103CB/C8 Bluepill with a 1.3" 128x64 OLED display (SH1106 chip-set). Since the code
has been adapted to use the u8g2 library it is a relatively simple matter to use alternative displays. 

I originally took on the task of modifying GM8BJF's code because it did not work on my board. His code kept locking up on SPI writes even though it appeared correct (fake STM32?). I also wanted to use a different, larger,  display plus the on-board ADF5355 25Mhz clock rather than an external clock.

Once using the generator I found the ADF5355 on-board 25Mhz XTAL was actually closer to 24.99395Mhz. So to get around this problem I added a calibration feature allowing an offset to be specified in code. The current calibration offset is shown during start-up.

To change the offset, press the frequency switch during power on and release when the display becomes active. Then adjust as required. A minus offset is subtracted from your crystal while a positive offset is added. The offset is stored in internal EEPROM.


Calibration can be accomplished  against a known receiver (higher the RX frequency the better)

Other than that one rotary encoder changes the frequency up/down while pressing it jumps to preset frequencies. The other sets tuning step size while pressing it sets output level between -5dBm to +2dBm.

The Bluepill board I use is preprogrammed with the maple DFU boot-loader which greatly simplifies programming from the Arduino IDE.

The Arduino IDE (V1.8.13) was configured for a generic SMT32F1xx and the upload method set to "Maple DFU Bootloader 2.0".

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



