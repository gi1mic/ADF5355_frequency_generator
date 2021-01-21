// Code to use an ADF5355 as a signal generator from 52 to 13600 MHz
//
// Based on that of DD7LP and GM8BJF (See below) with minor 
// modifications by GI1MIC to run on a STM32F103CB/C8 Bluepill.
//
// The code changes were originally required because the original code
// kept locking up on SPI writes.
// 
// The following code is based on using the onboard ADF5355 25Mhz clock
//
// Since the 25Mhz clock on my board was closer to 24.99395Mhz I added
// a calibration feature. The current calibration offset is shown
// during start-up.
//
// To change the offset, press the frequency switch during power on and
// release when the display becomes active. Then adjust as required -
// a minus offset is subtracted from your crystal while a positive offset
// is added. 
//
// The offset is stored in internal eeprom.
//
// Calibration can be accomplished  against a known receiver (higher the
// known RX frequency the better)
//
// The board I use was programmed with the maple DFU
// bootloader for ease of programming.
//
// The Arduino IDE (V1.8.13) was configured for a generic SMT32F1xx
//
// Blue pill Pins:
// Encoder 1 Clk - PB11, DT - PB10, SWT - PB1, + - 3.3V, GND - GND
// Encoder 2 Clk - PA3, DT - PA2, SWT - PA1, + - 3.3V, GND - GND
// I2C OLED  SDA - PB7, SCL - PB6, + - 3.3V, GND - GND
// ADF5355  CLK-PA5, MUX-PB0, LE-PA4, DAT-PA7, 3.3V - 3.3V, GND - GND, Barrel Jack - 6V (or 5V on the Bluepill)
//
//
//

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

#include <SPI.h>
#include <RotaryEncoder.h>   //************https://github.com/mathertel/RotaryEncoder/blob/master/RotaryEncoder.h
#include <Wire.h>            // I2C library //
#include <Adafruit_SSD1306.h>  // device driver for 128x64 SPI/I2C
#include <Adafruit_GFX.h>               //************** https://github.com/adafruit/Adafruit-GFX-Library
#include <EEPROM.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

unsigned long currentTime;
unsigned long loopTime;
const int pin_A = PB11;  // pin 0
const int pin_B = PB10;  // pin 1
unsigned char encoder_A;
unsigned char encoder_B;
unsigned char encoder_A_prev = 0;

unsigned long loopTime2;
const int pin_A2 = PA3;  // pin 8
const int pin_B2 = PA2;  // pin 9
unsigned char encoder_A2;
unsigned char encoder_B2;
unsigned char encoder_A2_prev = 0;

const int switch1 = PB1;
const int switch2 = PA1;

boolean mrk1, mrk1_old, mrk2, mrk2_old;

int press = 0;
int cnt_step = 6;
int cnt_step_old;
int cnt_fix = 4;
int cnt_fix_old;
int cnt_pwr = 1;
int cnt_pwr_old;
int mdbm = 0;

const int slaveSelectPin = PA4;  // SPI-SS bzw. enable ADF4350  wurde von mir von pin 3 auf pin 10 geändert
const int ADF5355_MUX = PB0;


long Freq = 14520000;  // Start-up frequency
long Freq_Old;
//long refin =  2499395;      // XTAL freq (corrected for my on board 25Mhz xtal)
long refin =  2500000;      // XTAL freq (corrected for my on board 25Mhz xtal)
long xtalOffset = 0;
long ChanStep = 10000;      // Initial channel step = 100.0 Khz
unsigned long Reg[13];      // ADF5355 Reg's
int pegelZ = 0;            // Zähler konstante für Ausgangspegel-Zähler
int outPegel = mdbm;       // Ausgangspegel beim start = -4 dBm

///////////////////////// Subroutine: Set Frequency ADF5355 ///////////////////////////
void SetFreq(long Frequ)     // Freq hier lokal oben global
{
  //Initialisation sequence
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

////////////////////////// Part-Subroutine ADF5355 ////////////////////////////
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



////////////////////////////// Sub-Subroutine ADF5355 //////////////////////////
void ConvertFreq(unsigned long R[])
////  Declare variables for registers/////
{
  // PLL-Reg-R0         =  32bit
  //   Registerselect        4bit
  //  int N_Int = 92;       // 16bit
  int Prescal = 0;         // 1bit geht nicht ??? it does not work
  int Autocal = 1;          //1 bit
  //  reserved           // 10bit

  // PLL-Reg-R1         =  32bit
  //   Registerselect        4bit
  //   int FRAC1 = 10;       // 24 bit
  //   reserved              // 4bit

  // PLL-Reg-R2         =  32bit
  //    Registerselect        4bit
  int M_Mod2 = 16383;            // 14 bit
  //    int Frac2 = 0;            // 14 bit


  // PLL-Reg-R3         =  32bit - FIXED !
  // Registerselect        4bit
  //Fixed value to be written = 0x3 =3


  // PLL-Reg-R4         =  32bit
  // Registerselect        4bit
  int U1_CountRes = 0;     // 1bit
  int U2_Cp3state = 0;     // 1bit
  int U3_PwrDown = 0;      // 1bit
  int U4_PDpola = 1;       // 1bit
  int U5_MuxLog = 1;          // 1bit
  int U6_RefMode = 1;          // 1bit
  //  int U5_LPD = 0;          // 1bit
  //  int U6_LPF = 1;          // 1bit 1=Integer, 0=Frac not spported yet
  int CP_ChgPump = 9;      // 4bit
  int D1_DoublBuf = 0;     // 1bit
  int R_Counter = 1;       // 10bit
  int RD1_Rdiv2 = 0;       // 1bit
  int RD2refdoubl = 0;     // 1bit
  int M_Muxout = 6;        // 3bit
  // reserved              // 2bit



  // PLL-Reg-R5         =  32bit
  // Registerselect        // 4bit
  // Phase Select: Not of partcular interst in Amatuer radio applications. Leave at a string of zeros.

  // PLL-Reg-R6         =  32bit
  // Registerselect        // 4bit
  //Variable value to be written!!!
  int D_out_PWR = mdbm;      // 2bit  OutPwr 0-3 3= +5dBm   Power out 1
  int D_RF_ena = 1;            // 1bit  OutPwr 1=on           0 = off  Outport Null freischalten
  int Reserved  = 0;                 // 3bit
  int D_RFoutB = 1;         // 1bit  aux OutSel
  int D_MTLD = 0;              // 1bit
  int CPBleed = 126;   // 8bit
  int D_RfDivSel = 3;      // 3bit 3=70cm 4=2m    lokale Variable
  int D_FeedBack = 1;       // 1bit
  // reserved              // 7bit

  // PLL-Reg-R7         =  32bit
  // Registerselect        // 4bit
  //Fixed value to be written = 0x120000E7 = 301990119 (dec)

  // PLL-Reg-R8         =  32bit
  // Registerselect        // 4bit
  //Fixed value to be written = 0x102D0428 = 271385640 (dec)

  // PLL-Reg-R9         =  32bit
  // Registerselect        // 4bit
  //Fixed value to be written = 0x5047CC9 = 84180169 (dec)

  // PLL-Reg-R10         =  32bit
  // Registerselect        // 4bit
  //Fixed value to be written = 0xC0067A = 12584570 9dec)

  // PLL-Reg-R11         =  32bit
  // Registerselect        // 4bit
  //Fixed value to be written = 0x61300B = 6369291 (dec)

  // PLL-Reg-R12         =  32bit
  // Registerselect        // 4bit
  //Fixed value to be written = 0x1041C = 66588 (dec)

  // Referenz Freg Calc

  // int F4_BandSel = 10.0 * B_BandSelClk / PFDFreq;
  double RFout = Freq;       // VCO-Frequenz  144200000  Freq ist global, RFout ist lokal

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

  R[0] = (unsigned long)(0 + N_Int * pow(2, 4) + Prescal * pow(2, 20) + Autocal * pow(2, 21)); // R0 für Startfrequenz ok
  R[1] = (unsigned long)(1 + F_Frac1 * pow(2, 4));
  R[2] = (unsigned long)(2 + M_Mod2 * pow(2, 4) + F_Frac2 * pow(2, 18)); //
  R[3] = (unsigned long)(0x3);  //Fixed value (Phase control not needed)
  R[4] = (unsigned long)(4 + U1_CountRes * pow(2, 4) + U2_Cp3state * pow(2, 5) + U3_PwrDown * pow(2, 6) + U4_PDpola * pow(2, 7) + U5_MuxLog * pow(2, 8) + U6_RefMode * pow(2, 9) + CP_ChgPump * pow(2, 10) + D1_DoublBuf * pow(2, 14) + R_Counter * pow(2, 15) + RD1_Rdiv2 * pow(2, 25) + RD2refdoubl * pow(2, 26) + M_Muxout * pow(2, 27));
  R[5] = (unsigned long) (0x800025); // Fixed (Reserved)
  R[6] = (unsigned long)(6 + D_out_PWR * pow(2, 4) + D_RF_ena * pow(2, 6) + Reserved * pow(2, 7) + D_RFoutB * pow(2, 10) + D_MTLD * pow(2, 11) + Reserved * pow(2, 12) + CPBleed * pow(2, 13) +  D_RfDivSel * pow(2, 21) + D_FeedBack * pow(2, 24) + 10 * pow(2, 25));
  R[7] = (unsigned long) (0x120000E7);
  R[8] = (unsigned long) (0x102D0428);
  R[9] = (unsigned long) (0x2A29FCC9);
  R[10] = (unsigned long) (0xC0043A);
  R[11] = (unsigned long) (0x61300B);
  R[12] = (unsigned long) (0x1041C);
}

//////////////////////////////////////////////////////////////////////////////
//                                      Setup                               //
//////////////////////////////////////////////////////////////////////////////
void setup() {

  Wire.begin();

  // ******************Screen mask static text*****************
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // enable internal HV supply for display and set I2C address
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.drawFastHLine(0,  14, 128, WHITE);
  display.drawFastHLine(0,  34, 128, WHITE);
  display.display();
  display.fillRect(85, 0, 43, 12, BLACK);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(85, 0);
  display.print("GI1MIC");
  display.display();

  SPI.begin();  // Start SPI for ADF5355
  pinMode (slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, LOW);

  pinMode(pin_A, INPUT_PULLUP);
  pinMode(pin_B, INPUT_PULLUP);
  pinMode(pin_A2, INPUT_PULLUP);
  pinMode(pin_B2, INPUT_PULLUP);

  pinMode(switch1, INPUT_PULLUP);     // 2 fix channel select
  pinMode(switch2, INPUT_PULLUP);     // 10 power select
  pinMode(ADF5355_MUX, INPUT_PULLUP);    // lock/unlock
  pinMode(14, INPUT_PULLUP);    // intref/extref via #A0  *************************Pin für int/ext Referenz *******************
  pinMode(16, INPUT_PULLUP);    // intref/extref via #A7 !!!! NB DIFFERENT FOR DUE !!!!

  currentTime = millis();
  loopTime = currentTime;
  loopTime2 = currentTime;


// If freq button pressed dring boot
// allow the calibration data to be changed
// else display what was saved and adjust
// the refin freq as required
  EEPROM.get(0,xtalOffset);
//  xtalOffset = 605;
  if (digitalRead(switch1) == LOW) {
    calibrate();
  } else {
    display.fillRect(0, 15, 128, 16, BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(5, 0);
    display.print("Xtal Offset");
    display.setCursor(20 , 20);
    display.print(xtalOffset, DEC);
    display.print(" Khz");
    display.display();
    delay(2000);
  }
  refin -= xtalOffset;
}

// *********************** Subroutine: update Display  **************************
void updateDisplay() {

  // I2C display update routine.
  if (mdbm == 0) {
    delayMicroseconds(100);
    display.fillRect(8, 40, 128, 12, BLACK);
    delayMicroseconds(100);
    display.setTextColor(WHITE);
    delayMicroseconds(100);
    display.setTextSize(1);
    delayMicroseconds(100);
    display.setCursor(8, 40);
    delayMicroseconds(25);
    display.print("Power out = -4 dBm");
    delayMicroseconds(25000);
  }
  else if (mdbm == 1) {
    display.fillRect(8, 40, 128, 12, BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(8, 40);
    display.print("Power out = -1 dBm");
  }
  else if (mdbm == 2) {
    display.fillRect(8, 40, 128, 12, BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(8, 40);
    display.print("Power out = +2 dBm");
  }
  else if (mdbm == 3) {
    display.fillRect(8, 40, 128, 12, BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(8, 40);
    display.print("Power out = +5 dBm");
  }
  display.display();

  //*********************Diplaying tuning step size *************************
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(8 , 53);
  display.print("Step = ");
  float ChanStep2 = ChanStep;
  if (ChanStep2 < 100)
  { ChanStep2 = ChanStep2 / 0.1;
    display.fillRect(40, 53, 100, 16, BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(60 , 53);
    display.print(ChanStep2, 0);
    display.print(" Hz");
    display.display();
  }
  else if (ChanStep2 < 100000)
  { ChanStep2 = ChanStep2 / 100;
    display.fillRect(40, 53, 100, 16, BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(60 , 53);
    display.print(ChanStep2, 0);
    display.print(" KHz");
    display.display();
  }
  else
  { ChanStep2 = ChanStep2 / 100000;
    display.fillRect(40, 53, 100, 16, BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(60 , 53);
    display.print(ChanStep2, 0);
    display.print(" MHz");
  }

  //**********************Displaying frequency***************************

  double Freq2;
  Freq2 = Freq;
  Freq2 = Freq2 / 100000;

  if (Freq2 < 1000) {
    display.fillRect(0, 15, 128, 16, BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(20 , 20);
    display.print(Freq2, 6 );
    display.print(" MHz");
  }
  else
  {
    display.fillRect(0, 15, 128, 16, BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(20 , 20);
    display.print(Freq2, 6);
    display.print(" MHz");
  }
}


void calibrate()
{

  display.fillRect(0, 15, 128, 16, BLACK);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(5, 0);
  display.print("Offset");
  display.setCursor(20 , 20);
  delay(2000);
  display.print(xtalOffset, DEC);
  display.print(" Khz");
  display.display();

  while (digitalRead(switch1) == HIGH) {
    currentTime = millis();
    if (currentTime >= (loopTime + 2)) {
      encoder_A = digitalRead(pin_A);
      encoder_B = digitalRead(pin_B);
      if ((!encoder_A) && (encoder_A_prev)) {
        if (encoder_B) {
          if (++xtalOffset > 10000) {
            xtalOffset = 0;
          }
        }
        else {
          if (--xtalOffset < -10000) {
            xtalOffset = 0;
          }
        }
      }
      encoder_A_prev = encoder_A;     // Store value of A for next time
      loopTime = currentTime;         // Updates loopTime
        display.fillRect(0, 15, 128, 16, BLACK);
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(20 , 20);
        display.print(xtalOffset, DEC);
        display.print(" Khz");
        display.display();
    }
  }

  EEPROM.put(0,xtalOffset);
}

////////////////////////////////////////////////////////////////////////
//                      MAIN PROGRAM LOOP                        //
////////////////////////////////////////////////////////////////////////
void loop()
{
  rotary_enc2();
  if (cnt_step != cnt_step_old) {
    updateDisplay();
    //  delayMicroseconds(250);
    updateDisplay();   // needs second update to stop encoders interacting ???///
    cnt_step_old = cnt_step;
  }

  fixfrq_select();
  if (cnt_fix != cnt_fix_old) {
    updateDisplay();
    cnt_fix_old = cnt_fix;
  }

  pwr_select();
  if (cnt_pwr != cnt_pwr_old) {
    updateDisplay();
    cnt_pwr_old = cnt_pwr;
    SetFreq(Freq);
  }
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);


  if (digitalRead(ADF5355_MUX) == HIGH)   // select lock/unlock
  {
    mrk1 = 1;
  } else {
    mrk1 = 0;
  }
  //*************************

  if (mrk1 != mrk1_old) {

    if (digitalRead(ADF5355_MUX) == HIGH)
    {
      display.fillRect(0, 0, 80, 12, BLACK);
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print("Locked");
      display.display();
    }
    else
    {
      display.fillRect(0, 0, 80, 12, BLACK);
      display.setTextColor(WHITE);
      display.setTextSize(0.5);
      display.setCursor(0, 0);
      display.print("Unlocked");
      display.display();
    }
  }

  mrk1_old = mrk1;

  rotary_enc();
  if (Freq != Freq_Old) {
    updateDisplay();   //
    // delayMicroseconds(250);
    SetFreq(Freq);
    updateDisplay();   // // needs second update to stop encoders interacting ???///
    Freq_Old = Freq;
  }
}

//*************************** End of main loop  ********************************************

/////////////////////////////// Subroutine: Frequency select /////////////////////////////////
void rotary_enc()
{
  currentTime = millis();
  if (currentTime >= (loopTime + 2)) {
    encoder_A = digitalRead(pin_A);
    encoder_B = digitalRead(pin_B);
    if ((!encoder_A) && (encoder_A_prev)) {
      if (encoder_B) {
        Freq = Freq + ChanStep;
        if (Freq > 1360000000) {
          Freq = 5400000;
        }
      }
      else {
        Freq = Freq - ChanStep;
        if (Freq < 5400000) {
          Freq = 1360000000;
        }
      }
    }
    encoder_A_prev = encoder_A;     // Store value of A for next time
    loopTime = currentTime;         // Updates loopTime
  }
}

/////////////////////////////// Subroutine: Step select ////////////////////////////////
void rotary_enc2()
{
  currentTime = millis();
  if (currentTime >= (loopTime2 + 2)) {
    encoder_A2 = digitalRead(pin_A2);
    encoder_B2 = digitalRead(pin_B2);
    if ((!encoder_A2) && (encoder_A2_prev)) {
      if (encoder_B2) {
        cnt_step = cnt_step + 1;
        if (cnt_step > 9) {
          cnt_step = 1;
        }
      }
      else {
        cnt_step = cnt_step - 1;
        if (cnt_step < 1) {
          cnt_step = 9;
        }
      }
    }
    encoder_A2_prev = encoder_A2;     // Store value of A for next time
    loopTime2 = currentTime;         // Updates loopTime
  }
  // Serial.println(cnt_step);
  if (cnt_step == 0) {
    ChanStep = 0.1;  // 1 Hz
  }
  else if (cnt_step == 1) {
    ChanStep = 1;  // 10 Hz
  }
  else if (cnt_step == 2) {
    ChanStep = 10;  // 100 Hz
  }
  else if (cnt_step == 3) {
    ChanStep = 100;  // 1 KHz
  }
  else if (cnt_step == 4) {
    ChanStep = 1000;  // 10 KHz
  }
  else if (cnt_step == 5) {
    ChanStep = 10000;  // 100KHz
  }
  else if (cnt_step == 6) {
    ChanStep = 100000;  // 1 MHz
  }
  else if (cnt_step == 7) {
    ChanStep = 1000000;  // 10 MHz
  }
  else if (cnt_step == 8) {
    ChanStep = 10000000;  // 100 MHz
  }
  else if (cnt_step == 9) {
    ChanStep = 100000000;  // 1000 MHz
  }
}

/////////////////////////// Subroutine: Fixfrequencyselect ////////////////////////////
void fixfrq_select()
{
  press = digitalRead(switch1);
  if (press == LOW)
  {

    if (cnt_fix == 0) {
      Freq = 5200000;  // 52.0 MHz
    }
    else if (cnt_fix == 1) {
      Freq = 7010000;  // 70.1 MHz
    }
    else if (cnt_fix == 2) {
      Freq = 14420000;  // 144.2 MHz
    }
    else if (cnt_fix == 3) {
      Freq = 43290000;  // 432.9 MHz
    }
    else if (cnt_fix == 4) {
      Freq = 129690000;  // 1296.9 MHz
    }
    else if (cnt_fix == 5) {
      Freq = 232090000;  // 2320.9 MHz
    }
    else if (cnt_fix == 6) {
      Freq = 345610000;  // 3456.1 MHz
    }
    else if (cnt_fix == 7) {
      Freq = 576050000;  // 5760.5 MHz
    }
    else if (cnt_fix == 8) {
      Freq = 1036810000;  // 10368.1  MHz
    }
    cnt_fix = cnt_fix + 1;
    if (cnt_fix == 9) {
      cnt_fix = 0 ;
    }
    delay(300);
  }
}

////////////////////////////////// Subroutine: Power select ///////////////////////////////
void pwr_select()
{
  press = digitalRead(switch2);
  if (press == LOW)
  {
    if (cnt_pwr == 0) {
      mdbm = 0;  // -4dBm
    }
    else if (cnt_pwr == 1) {
      mdbm = 1;  // -1dBm
    }
    else if (cnt_pwr == 2) {
      mdbm = 2;  // +2dBm
    }
    else if (cnt_pwr == 3) {
      mdbm = 3;  // +5dBm
    }
    cnt_pwr = cnt_pwr + 1;
    if (cnt_pwr == 4) {
      cnt_pwr = 0 ;
    }
    delay(300);
  }
}
//////////////////////////////////////////////////////////////////////////////////////
