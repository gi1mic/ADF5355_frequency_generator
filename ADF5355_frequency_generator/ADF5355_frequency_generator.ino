// Code to use an ADF5355 as a signal generator from 52 to 13600 MHz
//
// Based on that of DD7LP and GM8BJF (See below) with minor
// modifications by GI1MIC to run on a STM32F103CB/C8 Bluepill.
//
// The code changes originally required because the base code
// kept locking up on SPI writes.
//
// The following is based on using the onboard ADF5355 25Mhz clock
//
// Since my onboard 25Mhz clock was closer to 24.99395Mhz I added
// a calibration feature. The current stored calibration offset is shown
// during start-up.
//
// To change the offset, press the frequency switch during power on and
// release when the display becomes active. Then adjust as required -
// a minus offset is subtracted from your crystal while a positive offset
// is added.
//
// The offset is stored in internal eeprom.
//
// Calibration can be accomplished  against a known receiver (higher the known RX frequency the better)
//
// The board I use has been programmed with a maple DFU bootloader for ease of programming.
//
// Debounce capacitors, 10nf - 100nf, may be requred between the encoder/buttton inputs to ground.
// See https://github.com/enjoyneering/RotaryEncoder for a nice graphic of how to connect them to the encoder.
//
// The Arduino IDE (V1.8.13) was configured for a generic SMT32F1xx (BluePill F103CB 128K)
//
// Blue pill Pins:
// Encoder 1 Clk - PB11, DT - PB10, SWT - PB1, + - 3.3V, GND - GND
// Encoder 2 Clk - PA3, DT - PA2, SWT - PA1, + - 3.3V, GND - GND
// I2C OLED  SDA - PB7, SCL - PB6, + - 3.3V, GND - GND
// ADF5355  CLK-PA5, MUX-PB0, LE-PA4, DAT-PA7, 3.3V - 3.3V, GND - GND, Barrel Jack - 6V (5V from the Bluepill seems to work)
//
//
// Version 2.6 Simplified the code and switched to using interrupts for the encoders.
//
// Version 2.5 Added support for steps in Hz. Changed internal code frequencies to be in Hz rather than the Hz/10 of the original code.
//
// Version 2.4 Simplified more code and tidy up of on-screen formatting
//
// Version 2.3 Simplified dbm and preset frequency selection code
//
// Version 2.2 Simplified frequency step code to fix 1Khz selection
//
// Version 2.1 At the request of VE7XDT I added a long press option to the frequency adjustment encoder.
//             This toggles output pin B4 to select between an internal or external clock source using external hardware.
//
// Version 2.0 Updated code to use U8g2lib display library. This supports a greater range of displays.
//             Current code supports a 1.3" 64x128 oled display and a SH1106 chip
//
// Version 1.0 used a 0.98" 64x128 oled display with a SSD1306 chip


//***********************************************************************************************************************************************
//*******Code to use an ADF5355 as a signal generator from 52 to 13600 MHz based on that of DD7LP Christian Petersen for the ADF4351, (See ******
//*******credits below). The code has been ported to run on a 32 bit STM32F103CB ARM Cortex-M3 board. This was necessary to take *****
//*******advantage of the frequency resolution capabilities of the ADF5355. The display is an I2C OLED type. Brian Flynn GM8BJF. 14 June 2019 ***
//***********************************************************************************************************************************************
//***********************************************************************************************************************************************
// ****** Modifikation der Routinen vorgesehen für ADF4351 DDS bearbeitet von DD7LP Christian Petersen www.darc-husum.de im Februar    2017 *****
// *******Wesentliche Programmteile, Funktionen und Änderungen programmiert von DJ7OO Klaus Hirschelmann http://www.kh-gps.de/ im Februar 2017 **
//  ******Programmroutinen für das Ansprechen des ADF4351 aus einer Software von OE6OCG  Richard Posch 8302 Nestelbach 8452. Austria ************
//  ******Modifikation bezüglich des Kanalrasters. Diese wurden neu programmiert von OM Peter, DF6YL                     im Juli 2017************
//  ******Danke an OM Peter für diese Arbeit:                                 Anschrift Peter Beutel Verdistr. 2 59227 Ahlen         ************
// Achtung, kommerzielle Verwertung diese Software ist nicht gestattet bedarf der schriftlichen Zustimmmmung der Autoren, bzw Programmierer *****
//***********************************************************************************************************************************************

#include <RotaryEncoder.h>   // Install mathertel's library using the library manager or https://github.com/mathertel/RotaryEncoder)
#include <U8g2lib.h>         // *** Install from library manager *****//
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include <EEPROM.h>

// Change the following for your display. See the u8g2 library wiki oe examples for options.
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, PB6, PB7);

#define START_FREQ 145400000  // Start-up frequency
#define INIT_STEP       1000000 // Initial channel step
#define CHAN_STEP_SIZE       10 // Chan step multiplier (up and down)
#define MIN_STEP              1 // Minimum  step frequency
#define MAX_STEP     1000000000 // Max  step frequency
#define MIN_ADF5355    54000000 // Lowest ADF5355 freq
#define MAX_ADF5355  1360000000 // Highest ADF5355 freq


unsigned long startTime = millis(); // Grap the current system clock time during startup

const int pin_encoderFreqA = PB11;  // pin 0
const int pin_encoderFreqB = PB10;  // pin 1
unsigned char encoder_Freq_prev = 0;

const int pin_stepA = PA3;  // pin 8
const int pin_stepB = PA2;  // pin 9
unsigned char encoderStep_prev = 0;

// Note: You may need to change the last parm from FOUR3 to TWO03 depending on the encoder steps per latch 
RotaryEncoder encoderFreq(pin_encoderFreqA, pin_encoderFreqB, RotaryEncoder::LatchMode::FOUR3);
RotaryEncoder encoderStep(pin_stepA,        pin_stepB,        RotaryEncoder::LatchMode::FOUR3);

const int switchFreq = PB1;
const int switchStep = PA1;

const int encoderFrequencyButton = PB4;      // Output for long press on rotary encoder
const int encoderStepButton = PB3;      // Output for long press on rotary encoder
const int clockSource = PB5;      // Output for long press on rotary encoder

const int slaveSelectPin = PA4;  // SPI-SS bzw. enable ADF4350  wurde von mir von pin 3 auf pin 10 geändert
const int ADF5355_MUX = PB0;

boolean mrk1, mrk1_old = 0;

int cnt_fix = 4;
int dbm = 0;

double Freq = START_FREQ;
long refin =  2500000;      // XTAL freq (corrected for my on board 25Mhz xtal)
long xtalOffset = 0;
long ChanStep = INIT_STEP;
unsigned long Reg[13];      // ADF5355 Reg's

const double freqPresets[] = {   // Array to hold the preset frequencies add or remove frequencies as required
  52000000, // 52.0 MHz
  70100000, // 70.1 MHz
  144200000, // 144.2 MHz
  145400000, // 145.4 MHz
  432900000, // 432.9 MHz
  1296900000, // 1296.9 MHz
  2320900000, // 2320.9 MHz
  3456100000, // 3456.1 MHz
  5760500000, // 5760.5 MHz
  10368100000  // 10368.1  MHz
};

//////////////////////// Encoder Interrupt routine  ////////////////////////////////////////////////
void checkPositionFreq()
{
  encoderFreq.tick(); // just call tick() to check the state.
}

//////////////////////// Encoder Interrupt routine  ////////////////////////////////////////////////
void checkPositionStep()
{
  encoderStep.tick(); // just call tick() to check the state.
}

//////////////////////////////////////////////////////////////////////////////
//                                      Setup                               //
//////////////////////////////////////////////////////////////////////////////
void setup() {

  u8g2.begin();

  // Screen mask - static text
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_crox5hb_tr);
  u8g2.setCursor(20, 32);;
  u8g2.print("GI1MIC");    // Change as required
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.sendBuffer();
  delay(2000);
  // Draw lines
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.drawLine(0, 14, 128, 14);
  u8g2.drawLine(0, 38, 128, 38);

  SPI.begin();  // Start SPI for ADF5355

  attachInterrupt(digitalPinToInterrupt(pin_encoderFreqA), checkPositionFreq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_encoderFreqB), checkPositionFreq, CHANGE);

  attachInterrupt(digitalPinToInterrupt(pin_stepA), checkPositionStep, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_stepB), checkPositionStep, CHANGE);

  pinMode(encoderFrequencyButton, OUTPUT);
  digitalWrite(encoderFrequencyButton, LOW);

  pinMode(encoderStepButton, OUTPUT);
  digitalWrite(encoderStepButton, LOW);

  pinMode (slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, LOW);

  pinMode(clockSource, INPUT_PULLUP);

  pinMode(switchFreq, INPUT_PULLUP);     // 2 fix channel select
  pinMode(switchStep, INPUT_PULLUP);     // 10 power select

  pinMode(ADF5355_MUX, INPUT_PULLUP);    // lock/unlock
  pinMode(14, INPUT_PULLUP);    // intref/extref via #A0  *************************Pin für int/ext Referenz *******************
  pinMode(16, INPUT_PULLUP);    // intref/extref via #A7 !!!! NB DIFFERENT FOR DUE !!!!

  // If freq button pressed during boot allow calibration data to be changed
  // else display what was saved and adjust 'refin' as needed
  EEPROM.get(0, xtalOffset);

  if (digitalRead(switchFreq) == LOW) {
    calibration();
  } else {
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, 15, 128, 16);
    u8g2.setDrawColor(1);
    u8g2.setCursor(5, 30);
    u8g2.print("Xtal Offset: ");
    u8g2.print(xtalOffset, DEC);
    u8g2.print(" Khz");
    u8g2.sendBuffer();
    delay(3000);
  }
  refin -= xtalOffset;
  updateDisplay();

}



////////////////////////////////////////////////////////////////////////
//                      MAIN PROGRAM LOOP                        //
////////////////////////////////////////////////////////////////////////
void loop()
{
  
  // Process frequency encoder
  if (rotary_encFreq()) {
    long f = Freq;
    SetFreq();
    updateDisplay();
  }

  // Process step encoder
  if (rotary_encStep()) {
    updateDisplay();
  }

  // Process freq button
  if (fixfrq_select()) {
    updateDisplay();
  }

  // Process power button
  if (  pwr_select()) {
    updateDisplay();
    long f = Freq;
    SetFreq();
  }

  // Update lock/unlock status
  if (digitalRead(ADF5355_MUX) == HIGH) {
    mrk1 = 1;
  } else {
    mrk1 = 0;
  }

  if (mrk1 != mrk1_old) {
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, 0, 64, 12);
    u8g2.setDrawColor(1);
    u8g2.setCursor( 0, 7);
    if (digitalRead(ADF5355_MUX) == HIGH)
      u8g2.print("Locked");
    else
      u8g2.print("Unlocked");
    u8g2.sendBuffer();
  }
  mrk1_old = mrk1;

  // Update display
  u8g2.setDrawColor(0);
  u8g2.drawBox(64, 0, 128, 12);
  u8g2.setDrawColor(1);
  u8g2.setCursor( 100, 7);
  if (digitalRead(clockSource)) {
    u8g2.print("Int");
  } else {
    u8g2.print("Ext");
  }
  u8g2.sendBuffer();
}

//*************************** End of main loop  ********************************************

///////////////////////// Subroutine: Set Frequency ADF5355 ///////////////////////////
void SetFreq()
{
  //Initialization sequence
  ConvertFreq(Reg);
  Write_ADF_Reg(12);
  Write_ADF_Reg(11);
  Write_ADF_Reg(10);
  Write_ADF_Reg(9);
  Write_ADF_Reg(8);
  Write_ADF_Reg(7);
  Write_ADF_Reg(6);
  Write_ADF_Reg(5);
  Write_ADF_Reg(4);
  Write_ADF_Reg(3);
  Write_ADF_Reg(2);
  Write_ADF_Reg(1);
  Write_ADF_Reg(0);
}

////////////////////////// ADF5355 write register ////////////////////////////
void Write_ADF_Reg(int idx)
{ // make 4 byte from integer for SPI-Transfer
  byte buf[4];
  for (int i = 0; i < 4; i++)
    buf[i] = (byte)(Reg[idx] >> (i * 8));

  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));   //  Speed (e.g.10000000=10MHz), BitOrder (MSBFIRST), Mode (SPI_MODE0 - SPI_MODE3
  delayMicroseconds(1);
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(1);
  SPI.transfer((byte) buf[3]);
  SPI.transfer((byte) buf[2]);
  SPI.transfer((byte) buf[1]);
  SPI.transfer((byte) buf[0]);

  delayMicroseconds(1);
  digitalWrite(slaveSelectPin, HIGH);
  SPI.endTransaction();
  delayMicroseconds(10);
}


////////////////////////////// Calculate ADF5355 registers based on requested frequency  //////////////////////////
void ConvertFreq(unsigned long R[])
////  Declare variables for registers/////
{
  // PLL-Reg-R0 = 32bit
  //  int N_Int = 92;       // 16bit
  int Prescal = 0;          // 1bit geht nicht ??? it does not work
  int Autocal = 1;          //1 bit
  //  reserved              // 10bit

  // PLL-Reg-R1 = 32bit
  // int FRAC1 = 10;       // 24 bit
  // reserved              // 4bit

  // PLL-Reg-R2         =  32bit
  int M_Mod2 = 16383;            // 14 bit
  // int Frac2 = 0;            // 14 bit

  // PLL-Reg-R3         =  32bit - FIXED !
  // Fixed value to be written = 0x3 =3

  // PLL-Reg-R4 =  32bit
  int U1_CountRes = 0;     // 1bit
  int U2_Cp3state = 0;     // 1bit
  int U3_PwrDown = 0;      // 1bit
  int U4_PDpola = 1;       // 1bit
  int U5_MuxLog = 1;       // 1bit
  int U6_RefMode = 1;      // 1bit
  //  int U5_LPD = 0;      // 1bit
  //  int U6_LPF = 1;      // 1bit 1=Integer, 0=Frac not spported yet
  int CP_ChgPump = 9;      // 4bit
  int D1_DoublBuf = 0;     // 1bit
  int R_Counter = 1;       // 10bit
  int RD1_Rdiv2 = 0;       // 1bit
  int RD2refdoubl = 0;     // 1bit
  int M_Muxout = 6;        // 3bit
  // reserved              // 2bit

  // PLL-Reg-R5 = 32bit
  // Phase Select: Not of particular interest in Amateur radio applications. Leave at a string of zeros.

  // PLL-Reg-R6 = 32bit
  // Variable value to be written!!!
  int D_out_PWR = dbm;      // 2bit  OutPwr 0-3 3= +5dBm   Power out 1
  int D_RF_ena = 1;         // 1bit  OutPwr 1=on           0 = off  Outport Null freischalten
  int Reserved  = 0;        // 3bit
  int D_RFoutB = 1;         // 1bit  aux OutSel
  int D_MTLD = 0;           // 1bit
  int CPBleed = 126;        // 8bit
  int D_RfDivSel = 3;       // 3bit 3=70cm 4=2m    lokale Variable
  int D_FeedBack = 1;       // 1bit
  // reserved               // 7bit

  // PLL-Reg-R7 = 32bit
  // Fixed value to be written = 0x120000E7 = 301990119 (dec)

  // PLL-Reg-R8 = 32bit
  // Fixed value to be written = 0x102D0428 = 271385640 (dec)

  // PLL-Reg-R9 = 32bit
  // Fixed value to be written = 0x5047CC9 = 84180169 (dec)

  // PLL-Reg-R10 =  32bit
  // Fixed value to be written = 0xC0067A = 12584570 9dec)

  // PLL-Reg-R11 =  32bit
  // Fixed value to be written = 0x61300B = 6369291 (dec)

  // PLL-Reg-R12 =  32bit
  // Fixed value to be written = 0x1041C = 66588 (dec)

  // Referenz Freg Calc

  // int F4_BandSel = 10.0 * B_BandSelClk / PFDFreq;
  double RFout = Freq / 10;       // VCO-Frequency  144200000

  // calc bandselect und RF-div
  float outdiv = 1;
  if (RFout >= 680000000) {
    outdiv = 0.5;
    D_RfDivSel = 0;
    D_RFoutB = 0;
    D_RF_ena = 0;
  }
  if (RFout < 680000000) {
    outdiv = 1;
    D_RfDivSel = 0;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  if (RFout < 340000000) {
    outdiv = 2;
    D_RfDivSel = 1;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  if (RFout < 170000000) {
    outdiv = 4;
    D_RfDivSel = 2;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  if (RFout < 85000000) {
    outdiv = 8;
    D_RfDivSel = 3;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  if (RFout < 42500000) {
    outdiv = 16;
    D_RfDivSel = 4;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  if (RFout < 21250000) {
    outdiv = 32;
    D_RfDivSel = 5;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  if (RFout < 10625000) {
    outdiv = 64;
    D_RfDivSel = 6;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }

  /////////////////////////////////////////////////////////////////////////////
  //////////////////////// N and Frac1 and Frac2 calculations /////////////////
  //////////////////////// Done using double precision 64 bit /////////////////
  //////////////////////// Results agree exactly with AD demo /////////////////
  /////////////////////////////////////////////////////////////////////////////

  double PFDFreq = refin * ((1.0 + RD2refdoubl) / (R_Counter * (1.0 + RD1_Rdiv2))); //Phase detector frequency
  double N = ((RFout) * outdiv) / PFDFreq;   // Calculate N
  int N_Int = N;   // N= 50 for 5 GHz   // Turn N into integer
  double F_Frac1x = (N - N_Int) * pow(2, 24);   // Calculate Frac1 (N remainder * 2^24)
  int F_FracN = F_Frac1x;  // turn Frac1 into an integer
  double F_Frac2x = ((F_Frac1x - F_FracN)) * pow(2, 14);  // Claculate Frac2 (F_FracN remainder * 2^14)
  int F_Frac1 =   F_Frac1x;  // turn Frac1 into integer
  int F_Frac2 =   F_Frac2x;  // turn Frac2 into integer

  ////////////////// Set 32 bit register values R0 to R12 ///////////////////////////

  R[0]  = (unsigned long)(0 + N_Int * pow(2, 4) + Prescal * pow(2, 20) + Autocal * pow(2, 21)); // R0 für Startfrequenz ok
  R[1]  = (unsigned long)(1 + F_Frac1 * pow(2, 4));
  R[2]  = (unsigned long)(2 + M_Mod2 * pow(2, 4) + F_Frac2 * pow(2, 18)); //
  R[3]  = (unsigned long)(0x3);  //Fixed value (Phase control not needed)
  R[4]  = (unsigned long)(4 + U1_CountRes * pow(2, 4) + U2_Cp3state * pow(2, 5) + U3_PwrDown * pow(2, 6) + U4_PDpola * pow(2, 7) + U5_MuxLog * pow(2, 8) + U6_RefMode * pow(2, 9) + CP_ChgPump * pow(2, 10) + D1_DoublBuf * pow(2, 14) + R_Counter * pow(2, 15) + RD1_Rdiv2 * pow(2, 25) + RD2refdoubl * pow(2, 26) + M_Muxout * pow(2, 27));
  R[5]  = (unsigned long) (0x800025); // Fixed (Reserved)
  R[6]  = (unsigned long)(6 + D_out_PWR * pow(2, 4) + D_RF_ena * pow(2, 6) + Reserved * pow(2, 7) + D_RFoutB * pow(2, 10) + D_MTLD * pow(2, 11) + Reserved * pow(2, 12) + CPBleed * pow(2, 13) +  D_RfDivSel * pow(2, 21) + D_FeedBack * pow(2, 24) + 10 * pow(2, 25));
  R[7]  = (unsigned long) (0x120000E7);
  R[8]  = (unsigned long) (0x102D0428);
  R[9]  = (unsigned long) (0x2A29FCC9);
  R[10] = (unsigned long) (0xC0043A);
  R[11] = (unsigned long) (0x61300B);
  R[12] = (unsigned long) (0x1041C);
}

//////////////////////// update Display  ////////////////////////////////////////////////
void updateDisplay() {
  // Display dBm
  u8g2.setDrawColor(0);
  u8g2.drawBox(8, 40, 128, 12);
  u8g2.setDrawColor(1);
  u8g2.setCursor(8, 50);
  u8g2.print( "Power out = ");
  if (dbm == 0) {
    u8g2.print( "-4");
  } else if (dbm == 1) {
    u8g2.print("-1");
  } else if (dbm == 2) {
    u8g2.print("+2");
  } else if (dbm == 3) {
    u8g2.print("-5");
  }
  u8g2.print( " dBm");

  // Display tuning step size
  u8g2.setDrawColor(1);
  u8g2.setCursor( 8, 62);
  u8g2.print("Step Inc = ");
  u8g2.setDrawColor(0);
  u8g2.drawBox(70, 53, 100, 16);
  u8g2.setDrawColor(1);
  u8g2.setCursor( 70, 62);
  if (ChanStep < 1000) {
    u8g2.printf("%4u Hz", ChanStep );
  } else if (ChanStep < 1000000) {
    u8g2.printf("%4u KHz", ChanStep / 1000);
  } else {
    u8g2.printf("%4u MHz", ChanStep / 1000000);
  }

  // Display frequency
  double locFreq = Freq;
  locFreq = locFreq / 1000000;

  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 15, 128, 16);
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_helvB10_tr);
  u8g2.setCursor( 00, 31);
  char freqString [25];
  floatToString(freqString, locFreq , 6, 13);
  u8g2.print(freqString);
  u8g2.print("MHz");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.sendBuffer();
}

//////////////////////// Calibration  ////////////////////////////////////////////////
void calibration()
{
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 15, 128, 16);
  u8g2.setDrawColor(1);
  u8g2.setCursor( 5, 30);
  u8g2.print("Set offset: ");
  u8g2.print(xtalOffset, DEC);
  u8g2.print(" KHz");
  u8g2.sendBuffer();
  delay(1000);

  while (digitalRead(switchFreq) == HIGH) {
    unsigned long currentTime = millis();
    if (currentTime >= (startTime + 2)) {
      int encoder = encoderFreq.getPosition();
      if (encoder_Freq_prev != encoder) {
        int dir = (int)encoderFreq.getDirection();
        if ( dir == 1)
          ++xtalOffset;
        if ( dir == -1) {
          --xtalOffset;
        }
        if (xtalOffset > 10000) {
          xtalOffset = 0;
        }
        if (xtalOffset < -10000) {
          xtalOffset = 0;
        }
        encoder_Freq_prev = encoder;     // Store value of A for next time
      }
      u8g2.setDrawColor(0);
      u8g2.drawBox(0, 15, 128, 16);
      u8g2.setDrawColor(1);
      u8g2.setCursor( 5, 30);
      u8g2.print("Set offset: ");
      u8g2.print(xtalOffset, DEC);
      u8g2.print(" KHz");
      u8g2.sendBuffer();
    }
  }
  EEPROM.put(0, xtalOffset);
}


/////////////////////////////// Frequency select /////////////////////////////////
bool rotary_encFreq()
{
  int encoder = encoderFreq.getPosition();

  if (encoder_Freq_prev != encoder) {
    int dir = (int)encoderFreq.getDirection();
    if ( dir == 1) {
      Freq = Freq + ChanStep;
    }
    if ( dir == -1) {
      Freq = Freq - ChanStep;
    }
    if (Freq > MAX_ADF5355) {
      Freq = MIN_ADF5355;
    }
    if (Freq < MIN_ADF5355) {
      Freq = MAX_ADF5355;
    }
    encoder_Freq_prev = encoderFreq.getPosition();     // Store for next time
    return true;
  }
  return false;
}

/////////////////////////////// Step select ////////////////////////////////
bool rotary_encStep()
{
  int encoder = encoderStep.getPosition();

  if (encoderStep_prev != encoder) {
    int dir = (int)encoderStep.getDirection();
    if ( dir == 1) {
      ChanStep = ChanStep * CHAN_STEP_SIZE;
    } // Ignore 0 case
    if (dir == -1) {
      ChanStep = ChanStep / CHAN_STEP_SIZE;
    }
    if (ChanStep > MAX_STEP) {
      ChanStep = MIN_STEP;
    }
    if (ChanStep < MIN_STEP ) {
      ChanStep = MAX_STEP;
    }
    encoderStep_prev = encoderStep.getPosition();     // Store for next time
    return true;
  }
  return false;
}

/////////////////////////// Fixed frequency select ////////////////////////////
bool fixfrq_select()
{
  unsigned long pressedTime;

  if (digitalRead(switchFreq) == LOW)
  { pressedTime = millis();
    while (digitalRead(switchFreq) == LOW) {
      delay(10);
    }; // Wait for release
    if ((millis() - pressedTime) > 500) {                  // Long press delay
      digitalWrite(encoderFrequencyButton, !digitalRead(encoderFrequencyButton));;     // Invert output
    } else {
      Freq = freqPresets[cnt_fix++];
      if (cnt_fix >= (sizeof(freqPresets) / sizeof(freqPresets[0]))) {
        cnt_fix = 0 ;
      }
      delay(300);
    }
    return true;
  }
  return false;
}

////////////////////////////////// Power select ///////////////////////////////
bool pwr_select()
{
  unsigned long pressedTime;

  if (digitalRead(switchStep) == LOW)
  { pressedTime = millis();
    while (digitalRead(switchStep) == LOW) {
      delay(10);
    };      // Wait for release
    if ((millis() - pressedTime) > 500) {                  // Long press delay
      digitalWrite(encoderStepButton, !digitalRead(encoderStepButton));;     // Invert output
    } else {
      dbm++;
      if (dbm >= 4) {
        dbm = 0 ;
      }
      delay(300);
    }
    return true;
  }
  return false;
}


//////////////////////////// Float to string //////////////////////////////////////////////////////////
// See https://forum.arduino.cc/index.php/topic,37391.0.html#12
char * floatToString(char * outstr, double val, byte precision, byte widthp) {
  char temp[16];
  byte i;

  // compute the rounding factor and fractional multiplier
  double roundingFactor = 0.5;
  unsigned long mult = 1;
  for (i = 0; i < precision; i++)
  {
    roundingFactor /= 10.0;
    mult *= 10;
  }

  temp[0] = '\0';
  outstr[0] = '\0';

  if (val < 0.0) {
    strcpy(outstr, "-\0");
    val = -val;
  }

  val += roundingFactor;

  strcat(outstr, itoa(int(val), temp, 10)); //prints the int part
  if ( precision > 0) {
    strcat(outstr, ".\0"); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision - 1;
    while (precision--)
      mult *= 10;

    if (val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val) - val ) * mult;
    unsigned long frac1 = frac;

    while (frac1 /= 10)
      padding--;

    while (padding--)
      strcat(outstr, "0\0");

    strcat(outstr, itoa(frac, temp, 10));
  }

  // generate space padding
  if ((widthp != 0) && (widthp >= strlen(outstr))) {
    byte J = 0;
    J = widthp - strlen(outstr);

    for (i = 0; i < J; i++) {
      temp[i] = ' ';
    }

    temp[i++] = '\0';
    strcat(temp, outstr);
    strcpy(outstr, temp);
  }
  return outstr;
}
