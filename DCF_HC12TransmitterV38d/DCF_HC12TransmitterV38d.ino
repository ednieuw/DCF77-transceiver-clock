// =============================================================================================================================
/* 
This Arduino code works on a ATMEGA328 ( Nano Uno), ATMEGA1284 chip and Nano Every
This source contains code for the following modules:  
- RTC DS3231 clock module
- LDR light sensor 5528
- HC-12 Long Range Wireless Communication Module
- DCF77 module DCF-2
The DCF77 module reads the time and date from the German longwave DCF-77 time signal
The time and date are transmitted as ASCII string as hhmmss. (031500 = 15 minutes past 3)
The date is hourly transmitted with a prefic D followed by ddmmyyyy ( D01042020 - 1 april 2020)
Comments are transmitted with a prefix @ (@ 10:07:38 09-01-2020 DCF:16)
V01 - V28 developing 
V029 Added Station and Version number as define. **To be done Added CRC in @ HC12 string 
V030 MAX72191 displays added 
V31 Added support for 24 Ws2812 ring for small transmitter S001. Added Dimleds()
V32 Stable version
V33 HC12 transceiver added
V34 Failed introduction of time functions
V35 Optimizing, efficiency in MAX7219
V36 Optimizing and refining, 
V37 stable version.  Added Interval measurements in DCFtiny, Store settings in EEPROM, added menu entries, 
    Corrected drifting and enhanced efficiency from 75% to >90% by changing > 999 in >= 1000 msec in DCFtine loop
V38a->z Optimized for BT communication on Nano every. second serial port does not receive? on Nano Every
V38b Minor changes
V38c Failure . HC12 and BT can not be used together. BT works when HC12 is removed. softwareserial lib?. Serial1 will not initialize
V38d https://mcudude.github.io/MegaCoreX/package_MCUdude_MegaCoreX_index.json


Ed Nieuwenhuys 2020
 */
// ===============================================================================================================================

//--------------------------------------------
// ARDUINO Definition of installed modules
// If a module is not present remove it from compilation 
// by adding the comment slashes, //, before the #define
//--------------------------------------------
#define DCFMOD           // Use the Arduino DCF77 library with interrupts
#define DCFTINY          // Use the Tiny DCF algorithm in this program

#define HC12MOD          // Use HC12 time transreceiver Long Range Wireless Communication Module
#define BLUETOOTHMOD     // Use  this define if Bluetooth needs other pins than pin 0 and pin 1

//#define LCDMOD           // LCD 2 x 16 installed 
#define TM1637tijd       // 4-digit time display installed
#define MAX7219info      // Two MAX7219 displays

#define MOD_DS3231       // DS3231 RTC module installed

//#define LED2812
#define LED6812         // choose between LED type RGB=LED2812 or RGBW=LED6812
//#define LEDs24RingInstalled

char STATION[] = "L002";  
char VERSION[] = "V038d";
//--------------------------------------------
// ARDUINO Includes defines and initialysations
//--------------------------------------------
                      #if defined LED2812 || defined LED6812
#define LEDsInstalled
                      #endif LED2812 || LED6812

#include <Wire.h>
#include "RTClib.h"
#include "Time.h"
#include <TimeLib.h>             // For time management 
                     #if defined LEDsInstalled || defined LEDs24RingInstalled
#include <Adafruit_NeoPixel.h>
                     #endif LEDsInstalled
#include <EEPROM.h>
                     #ifdef LCDMOD
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
                     #endif LCDMOD
                     #ifdef DCFMOD
#include "DCF77.h"
                     #endif DCFMOD
                     #ifdef BLUETOOTHMOD
#include <SoftwareSerial.h>        // Arduino      // for Bluetooth communication
                     #endif BLUETOOTHMOD
                     #ifdef HC12MOD
#include <SoftwareSerial.h>              // For HC12
                     #endif HC12MOD 
                     #ifdef TM1637tijd 
#include <TM1637Display.h>
                     #endif TM1637tijd 
                     #ifdef MAX7219info
#include "LedControl.h" 

// pin 10 is connected to the CLK 
// pin 11 is connected to CS 
// pin 12 is connected to the DataIn 
 LedControl lc= LedControl(12,10,11,2);
                      #endif MAX7219info 
//--------------------------------------------
// PIN Assigments
//--------------------------------------------                      
enum DigitalPinAssignments {      // Digital hardware constants ATMEGA 328 ----
 RX           = 1,                // Connects to TX
 TX           = 0,                // Connects to RX
 DCF_PIN      = 2,                // DCFPulse on interrupt pin
 CLK_TM1637   = 3,                // PIN 3 TM1637Display
 DIO_TM1637   = 4,                // PIN 4 TM1637Display
 LED_PIN      = 5,                // LED pin PIN 5
 BT_RX        = 7,                // Tx-pin on BT-mod
 BT_TX        = 6,                // Rx-pin on BT-mod 
 HC_12TX      = 1,                // HC-12 TX Pin
 HC_12RX      = 0,                // HC-12 RX Pin  
 PIN08        = 8,                // PIN 8 
 secondsPin   = 8,                // Seconds
 LED09        = 9,                // PIN 9 
 DCFgood      = 9,                // DCF-signal > 50
 PIN11        = 11,               // PIN 11
 PIN12        = 12,               // PIN 12 
 DCF_LED_Pin  = 13,               // Show DCF-signal
 };
 
enum AnaloguePinAssignments {     // Analogue hardware constants ----
 EmptyA0      = 0,                // A0
 EmptyA1      = 1,                // A1
 PhotoCellPin = 2,                // A2
 EmptyA3      = 3,                // A3
 SDA_pin      = 4,                // SDA pin
 SCL_pin      = 5};               // SCL pin

//--------------------------------------------
// COLOURS
//--------------------------------------------   
                         #ifdef LED2812
const uint32_t white     = 0x00FFFFFF;                   // white is R, G and B on WS2812                
                         #endif LED2812
                         #ifdef LED6812    
const uint32_t white     = 0xFF000000;                   // The SK6812 LED has a white LED that is pure white
                         #endif LED6812  
const uint32_t black  = 0x000000, red   = 0xFF0000, orange = 0xFF7000;
const uint32_t yellow = 0xFFFF00, apple = 0x80FF00, brown  = 0x503000;
const uint32_t green  = 0x00FF00, grass = 0x00FF80, sky    = 0x00FFFF;
const uint32_t marine = 0x0080FF, blue  = 0x0000FF, pink   = 0xFF0080; 
const uint32_t purple = 0xFF00FF, softwhite = 0x606060;

//--------------------------------------------
// LED
//--------------------------------------------

int Previous_LDR_read = 512;
                      #ifdef LEDsInstalled
                         #ifdef LED6812 
const byte NUM_LEDS  = 96;    // How many leds in  strip?   
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRBW + NEO_KHZ800);  //NEO_RGBW
                         #endif LED6812  
                         #ifdef LED2812
const byte NUM_LEDS  = 96;    // How many leds in  strip?
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);  //NEO_RGB NEO_GRB
                         #endif LED2812
                      #endif LEDsInstalled
                         #ifdef LEDs24RingInstalled
uint32_t BitColour   = red;  
const byte NUM_LEDS  = 24;    // How many leds in  strip?
const uint32_t white = 0x00FFFFFF;  
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);  //NEO_RGB NEO_GRB
                         #endif LEDs24RingInstalled                     

//--------------------------------------------
// HC-12 Long Range Wireless Communication Module
//--------------------------------------------
                         #ifdef HC12MOD
SoftwareSerial HC12(HC_12TX, HC_12RX); //rxPin, txPin, inverse_logic.
                                       //rxPin: the pin on which to receive serial data 
                                       //txPin: the pin on which to transmit serial data
                         #endif HC12MOD 
//--------------------------------------------
// BLUETOOTH
//--------------------------------------------                                     
                           #ifdef BLUETOOTHMOD               // Bluetooth ---------------------
SoftwareSerial Bluetooth(BT_RX, BT_TX);    // BT_RX <=> TXD on BT module, BT_TX <=> RXD on BT module
String  BluetoothString;
                           #endif BLUETOOTHMOD 
//--------------------------------------------
// CLOCK
//--------------------------------------------                                 
#define MAXTEXT 100                // The maximum length of sptext in de sprintf function
static  uint32_t msTick;           // The number of millisecond ticks since we last incremented the second counter
int     Isecond, Iminute, Ihour, Iday, Imonth, Iyear; 
int     Dsecond, Dminute, Dhour, Dday, Dmonth, Dyear, Dwday; 
byte    lastminute = 0, lasthour = 0, sayhour = 0;
byte    SecPulse  = 0;             // Give a pulse to the Isecond led

const byte MenuItems = 12;          // No of entries in menu
const char menu[MenuItems][50] PROGMEM = {
//const char menu[][MAXTEXT] PROGMEM = {
 "*** DCF HC12 Bluetooth Transmitter ***",
 "Enter time as:hhmm (1321) or hhmmss (132145)",
 "A   DCF Debug info On/Off", 
 "D   D15122017 for date 15 December 2017",
 "F   Toggle full or interval DCFtiny readings", 
 "G   (G20) DCF loop measures every (0-999) msec",
 "H   (H50) mesurements in DCF loop (0-999)",
 "Lnn (L5) Min light intensity ( 1-255)",
 "Mnn (M90)Max light intensity (1%-250%)",
 "I   for this info",
 "R   Reset",
 "    Ed Nieuwenhuys apr-2020" };

//--------------------------------------------
// LDR PHOTOCELL
//--------------------------------------------
const byte MAXBRIGHTNESS = 20;
const byte LOWBRIGHTNESS = 5;
byte  LightReducer    =  MAXBRIGHTNESS; // Factor (%) to dim LED intensity with. Between 1% and 250%
byte  LowerBrightness =  LOWBRIGHTNESS; // Lower limit of Brightness ( 0 - 255)  
int   OutPhotocell;                     // stores reading of photocell;
int   MinPhotocell    = 999;            // stores minimum reading of photocell;
int   MaxPhotocell    = 1;              // stores maximum reading of photocell;

//--------------------------------------------
// DS3231 CLOCK MODULE
//--------------------------------------------
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
        #ifdef MOD_DS3231
RTC_DS3231 RTCklok;                   // RTC_DS1307 RTC; 
        #else if
RTC_Millis RTCklok;                   // if no RTC clock connected run on processor time
        #endif
DateTime Inow;
                    #ifdef DCFMOD 
//--------------------------------------------
// DCF-2 DCF77 MODULE
//--------------------------------------------
                    #ifdef ARDUINO_AVR_NANO_EVERY
#define DCF_INTERRUPT 2              // DCF Interrupt number associated with DCF_PIN ( 2 Nano Every)
                    #else if
#define DCF_INTERRUPT 0              // Nano Uno etc 
                    #endif      
DCF77 DCF = DCF77(DCF_PIN,DCF_INTERRUPT,LOW); // HIGH (HKW) or LOW (Reichelt). Some DCF modules invert the signal
                    #endif DCFMOD 
enum LinearLedstrip  {               // Linear LED strip
         Minutemark = 84, MinPar, HourPar, YearPar, DCFgoodminutes, DCF_tijd, RTC_tijd,
         Reserveantenna, Onehourchange, Summertime, Wintertime, TimeDoubfull };       
byte     DCF_signal         = 50;    // is a proper time received? (0 - 100)
byte     CSummertime        = 52;    // If >50 times CSummertime is received Summertime = true
byte     CWintertime        = 52;    // If >50 times CWintertime is received Wintertime = true
//byte     SignalBin[10];

//--------------------------------------------
// DCF77 DCFtiny MODULE
//--------------------------------------------
                    #ifdef DCFTINY
static   uint32_t DCFmsTick;         // the number of millisecond ticks since we last incremented the second counter
bool     DCFEd              = false; // 
bool     DCFThijs           = false; //    // flag to detect if DCFtiny and DCF77 library received a good time
bool     DCFlocked          = false; // time received from DCF77
bool     DCF_ReadFulltime   = true;  // Read DCF constantly or in intervals
byte     StartOfEncodedTime =  0;    // Should be always one. Start if time signal
int      TotalEff;                   // Efficiency of DCF-signal (0-100%)
int      mSecInterval       = 10;    // if DCF_ReadFulltime = false this is time in msec between readings
int      DCFreadingsInLoop  = 20;    // and ths are the number of reading in a loop
uint32_t SumSecondSignal    =  0;    // sum of digital signals ( 0 or 1) in 1 second
uint32_t SumSignalCounts    =  0;    // Noof of counted signals
uint32_t SignalFaults       =  0;    // Counter for SignalFaults per hour  
uint32_t Ed                 =  0;    //  
uint32_t Ed1                =  0;    //  
uint32_t Thijs              =  0;    //  
uint32_t EdThijsGelijk      =  0;    // 
uint32_t MinutesSinceStart  =  0;
float    TellerEd           = 50;    // Initial count for efficiency DCF-signal 
float    TellerTh           = 50;    // Initial count for efficiency DCF-signal

                    #endif DCFTINY
                    #ifdef LCDMOD
//--------------------------------------------
// LCD Module
//--------------------------------------------
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack
                    #endif 
                              #ifdef TM1637tijd
//--------------------------------------------
// TM1637 display
//--------------------------------------------
TM1637Display Tijd_display = TM1637Display(CLK_TM1637, DIO_TM1637);
                              #endif TM1637tijd

//----------------------------------------
// Common
//----------------------------------------
byte PrintDebugInfo         = false;      // for showing debug info for DCFTINY
char sptext[MAXTEXT+2];               // for common print use    
int  MilliSecondValue       = 999;    // The duration of a second  minus 1 ms. Used in Demo mode

// --------------------------------------------------------------------------
// End Definitions                                                    
// --------------------------------------------------------------------------
//--------------------------------------------
// ARDUINO Loop
//--------------------------------------------
void loop(void)
{
 SerialCheck(); 
 EverySecondCheck();
 EveryMinuteUpdate();
                              #if defined DCFMOD || defined DCFTINY        
 DCF77Check();
                              #endif DCFMOD || defined DCFTINY
}  
//--------------------------------------------
// ARDUINO Setup
//--------------------------------------------
void setup()
{                                        
 Serial.begin(9600);                                                // Setup the serial port to 9600 baud                                                        // Start the RTC-module  
 while (!Serial);                                                    // wait for serial port to connect. Needed for native USB port only
 Wire.begin();                                                      // Start communication with I2C / TWI devices                            
 pinMode(DCF_LED_Pin, OUTPUT);                                      // For showing DCF-pulse or other purposes
 pinMode(DCF_PIN,     INPUT_PULLUP);
// pinMode(BT_State,  INPUT_PULLUP);
// pinMode(BT_Break,  OUTPUT);   
 pinMode(DCFgood,     OUTPUT );  
 pinMode(secondsPin,  OUTPUT );
                              #ifdef MOD_DS3231
 RTCklok.begin();                                                   // start the RTC-module
                              # else if 
 RTCklok.begin(DateTime(F(__DATE__), F(__TIME__)));                 // if no RTC module is installed use the ATMEGAchip clock
                              #endif MOD_DS3231  
                              #ifdef DCFMOD
 DCF.Start();                                                      // Start the DCF-module
 Tekstprintln("DCF enabled");
                              #endif DCFMOD
                              #ifdef LCDMOD
 lcd.begin (16,2); // for 16 x 2 LCD module                         // Activate LCD module
 lcd.setBacklightPin(3,POSITIVE);
 lcd.setBacklight(HIGH);
 Tekstprintln("LCD enabled");
                              #endif LCDMOD
                              #ifdef TM1637tijd                      // 4-digit time display installed
 Tijd_display.setBrightness(3);                                    // Set the display brightness (0-7):
 Tijd_display.clear();
 Tekstprintln("4-digit time display installed"); 
                              #endif TM1637tijd  
                              #ifdef ARDUINO_AVR_NANO_EVERY
 Tekstprintln("ARDUINO_AVR_NANO_EVERY enabled");
                              #endif

                              #ifdef LEDsInstalled
                                  // --- LED
 strip.begin();
 strip.setBrightness(LightReducer); //BRIGHTNESS);
 LedsOff();                                                         // Initialize all pixels to 'off' 
 ColorLeds("", 60, 84, purple);                                     // fill the middle ring with purple dots
 LedsOffOuterring();
 ShowLeds(); 
                              #ifdef LED6812    
 Tekstprintln("LEDs SK6812 enabled");
                              #endif LED6812  
                              #ifdef LED2812
 Tekstprintln("LEDs WS2812 enabled");
                              #endif LED2812
   
                                 // --- LED 
                              #endif LEDsInstalled  
                              #ifdef LEDs24RingInstalled
 strip.begin();
 strip.setBrightness(LightReducer); //BRIGHTNESS);
 LedsOff();                                                         // Initialize all pixels to 'off' 
 Tekstprintln("24 LED ring enabled"); 
                              #endif LEDs24RingInstalled
                              #ifdef BLUETOOTHMOD 
                              #if defined ARDUINO_SAMD_MKRWIFI1010 
                              // || defined ARDUINO_AVR_NANO_EVERY
 Serial1.begin(9600);                                               // Bluetooth connected to Serial1
                              #else if
 Bluetooth.begin(9600); 
 Tekstprintln("Bluetooth is enabled"); 
                              #endif ARDUINO_SAMD_MKRWIFI1010
 Tekstprintln("Bluetooth enabled");
                              #endif BLUETOOTHMOD
                              #ifdef HC12MOD
 HC12.begin(9600);               // Serial port to HC12
 Tekstprintln("HC-12 time sender enabled");
                             #endif HC12MOD 

                             #ifdef MAX7219info                     
  lc.shutdown(0,false);  /* Set the brightness to a medium values */
  lc.setIntensity(0,0);  /* and clear the display */
  lc.clearDisplay(0);      
  lc.shutdown(1,false);  /* Set the brightness to a medium values */  
  lc.setIntensity(1,0);  /* and clear the display */
  lc.clearDisplay(1);
//  sprintf(sptext, "0123456789 12345");
//  PrintStringToSegDisplay(sptext);
  
  Tekstprintln("MAX7219info enabled");     
                              #endif MAX7219info  
  
 DateTime now        = RTCklok.now();
 DateTime compiled   = DateTime(__DATE__, __TIME__);

 if (now.unixtime() < compiled.unixtime()) 
  {
   Serial.println(F("RTC is older than compile time! Updating"));   // Following line sets the RTC to the date & time this sketch was compiled
   RTCklok.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }

 msTick = millis();                                                 // Used in KY-040 rotary for debouncing and seconds check
 SWversion();                                                       // Display the version number of the software
 GetTijd(0);                                                        // Get the time and store it in the proper variables
 lasthour = Ihour;
 setTime(Ihour, Iminute, Isecond,Iyear, Imonth, Iday);
 DCFmsTick = millis();                                              // Start of DCF 1 second loop
 } 
//--------------------------------------------
// CLOCK Version info
//--------------------------------------------
void SWversion(void) 
{ 
 for (int n=0; n<52; n++) {Serial.print(F("_"));}                                       Serial.println();
 for (int i = 0; i <MenuItems; i++)   {strcpy_P(sptext, menu[i]);                       Tekstprintln(sptext);  }
 for (int n=0; n<52; n++) {Serial.print(F("_"));}                                       Serial.println();
 sprintf(sptext,"  Brightness Min: %3d bits Max: %3d%%",LowerBrightness, LightReducer); Tekstprintln(sptext);
 sprintf(sptext,"    LDR read Min: %3d bits Max: %3d bits",MinPhotocell, MaxPhotocell); Tekstprintln(sptext);
 sprintf(sptext,"  DCF loop every: %3d msec  for: %3d times",mSecInterval, DCFreadingsInLoop); Tekstprintln(sptext);
 sprintf(sptext,"Version: %s Station; %s",VERSION,STATION);                             Tekstprintln(sptext);   
 for (int n=0; n<52; n++) {Serial.print(F("_"));}                                       Serial.println();
}

//--------------------------------------------
// ARDUINO Reset to default settings
//--------------------------------------------
void Reset(void)
{
 LightReducer         = MAXBRIGHTNESS;             // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
 EEPROMwrite(0, LightReducer);                     // Default intensity for this clock
 LowerBrightness      = LOWBRIGHTNESS;             // Lower limit of Brightness ( 0 - 255)
 EEPROMwrite(1, LowerBrightness);                  // Default Lower Brightness for this clock
 WritemSecInterval(mSecInterval);                  // DCFtiny interval loop in EEPROM
 WriteDCFreadingsInLoop(DCFreadingsInLoop);        // DCFtiny interval loop in EEPROM
 DCF_ReadFulltime     = false;                     // Read DCF constantly or in intervals
 MinPhotocell         = 1024;                      // stores minimum reading of photocell;
 MaxPhotocell         = 1;                         // stores maximum reading of photocell;
 SWversion();                                      // Display the version number of the software
 GetTijd(1);                                       // Get the time and store it in the proper variables
}
//--------------------------------------------
// CLOCK common print routines
//--------------------------------------------
void Tekstprint(char tekst[])
{
 Serial.print(tekst);    
                          #ifdef BLUETOOTHMOD   

                          #if defined(ARDUINO_SAMD_MKRWIFI1010)
 Serial1.print(tekst);  
                          #else if
 Bluetooth.print(tekst);  
                          #endif  
                          #endif BLUETOOTHMOD
}

void Tekstprintln(char tekst[])
{
 Serial.println(tekst);    
                          #ifdef BLUETOOTHMOD
                          #if defined(ARDUINO_SAMD_MKRWIFI1010)
 Serial1.println(tekst);
                          #else if
 Bluetooth.print(tekst);  
                          #endif defined(ARDUINO_SAMD_MKRWIFI1010)   
                          #endif BLUETOOTHMOD
}
//--------------------------------------------
// CLOCK Update routine done every second
//--------------------------------------------
void EverySecondCheck(void)
{
 if( millis() - msTick > 50)   
   { digitalWrite(secondsPin,LOW); }            // Turn OFF the second on pin 13
 if( millis() - msTick > 999)                   // Flash the onboard Pin 13 Led so we know something is happening
   {    
    msTick = millis();                          // second++; 
    digitalWrite(secondsPin,HIGH);              // turn ON the second on pin 13
    ++SecPulse;                                 // second routine in function DimLeds
    GetTijd(0);                                 // synchronize time with RTC clock    
                           #if defined LEDsInstalled || defined LEDs24RingInstalled
    if(Isecond == 7)       DimLeds(true);       // Text LED intensity control + seconds tick print every 30 seconds   
    else                   DimLeds(false);      // every second an intensitiy check and update from LDR reading  
                           #endif LEDsInstalled || defined LEDs24RingInstalled
                           #ifdef LCDMOD
    Print_tijd_LCD();
                           #endif LCDMOD  
                           #ifdef HC12MOD 
 switch(Isecond)                                  // When HC12 sends it disturbs DCF-signal. In seconds 1 - 19 are whether info bits
  {
   case  2: sprintf(sptext,"@ %0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d DCFeff:%0.2d S:%s",
                    Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),
                    Inow.month(),Inow.year(),TotalEff, STATION);
            if (DCFlocked)  HC12.println(sptext);                      // Print the time only if DCF time is trusted 
                            Serial.println(sptext);            break;  
   case  3: sprintf(sptext,"T%0.2d%0.2d%0.2d ",Ihour,Iminute,Isecond); // Send the time as Thhmmss
            if (DCFlocked)  HC12.println(sptext);                      // Print the time only if DCF time is trusted
            else            HC12.println("@ DCF not locked ");
                            Serial.print (sptext);             break;  // In all other cases it is an error
   case  4: if(Iminute == 0)                                           // Send date once an hour as Dddmmyyyy
              {    
               sprintf(sptext,"D%0.2d%0.2d%0.4d",Iday,Imonth,Iyear);
               if (DCFlocked) HC12.println(sptext);
                              Serial.println(sptext);
              }                                                break; 
   }
                           #endif HC12MOD   
                           #ifdef LEDsInstalled || defined LEDs24RingInstalled
   ShowLeds();
                           #endif LEDsInstalled
  }
}
 
//--------------------------------------------
// CLOCK Update routine done every minute
//--------------------------------------------
 void EveryMinuteUpdate(void)
 {
 if (Iminute != lastminute)                     // Show time every minute
  { 
   lastminute = Iminute;
   Print_RTCtijd();
   MinutesSinceStart++;                        // counter for DCF efficiency calculation 
   if(MinutesSinceStart> 0X0FFFFFFF)           // Reset counters before overflow
     {
      Ed = Ed1 = Thijs = EdThijsGelijk = MinutesSinceStart = 0;
      TellerEd = TellerTh = 50;                // Initial count for efficiency DCF-signal
     }
   TellerEd -= (TellerEd / 100);              
   TellerTh -= (TellerTh / 100);
   TotalEff =  constrain((int)((100*(Ed+Thijs-EdThijsGelijk))/(MinutesSinceStart+1)),1,99);
   DCF_signal--; 
   DCF_signal = constrain( DCF_signal,1,99);
   analogWrite(DCFgood, DCF_signal);           // The LED intensity displays the signal quality
                            #ifdef MAX7219info                     
 // lc.shutdown(0,false);
 // lc.setIntensity(0,0);                      // Set the brightness to a medium values 
   lc.clearDisplay(0);                         // and clear the display */
//  lc.shutdown(1,false);
 // lc.setIntensity(1,0);                      // Set the brightness to a medium values
//  lc.clearDisplay(1);                        // and clear the display */
                              #endif MAX7219info     
  } 
 if (Ihour != lasthour) 
    {
     GetTijd(0);   // sometimes one of the LEDs is white because of wrong Ihour?? remove if in oddle ring white LEDs are at wrong hour
                             #ifdef LEDsInstalled
     ColorMiddleRing(lasthour , SignalFaults);  
                             #endif LEDsInstalled
     SignalFaults = 0;   
     DCFlocked = false;                        // it is time to receive a DCF-time code
     lasthour = Ihour;
    }
 }
 
//--------------------------------------------
// CLOCK check for DCF input
//--------------------------------------------
void DCF77Check(void)
{
 static int  lastsecond = 99; 
 digitalWrite(DCF_LED_Pin, !digitalRead(DCF_PIN));   
                             #ifdef DCFMOD
 time_t DCFtime = DCF.getTime();                // Check if new DCF77 time is available
 if (DCFtime!=0)
  {
//   GetTijd(0);
//   setTime(DCFtime);
//   sprintf(sptext,"  DCF77 Lib OK --> %0.2d:%0.2d %0.2d-%0.2d-%0.4d ",hour(), minute(), day(), month(), year()); 
//   Serial.println(sptext); 
//x   DateTime now = RTCklok.now();                                           // Needed to use unixtime.
//x   DateTime DCFtijd (year(), month(), day(), hour(), minute(), 0);         // Convert the time in the time_t DCFtime to Datetime so we can use .unixtime 
//x   if (abs(now.unixtime() - DCFtijd.unixtime()) > 2 )                      // > in Seconds. If difference between DCF and RTC time update the RTC
//x      {
//x       Serial.println(F("RTC is different than DCF77 time! Updating"));   // Following line sets the RTC to the date & time this sketch was compiled
//x       RTCklok.adjust(DCFtime); 
//x      }   
//x   DCFlocked = true;                            // DCF time stored (for one hour before unlock)
   DCF_signal+=2;   
   DCF_signal = constrain( DCF_signal,1,100);
   Thijs++;
   TellerTh++;
   DCFThijs = true;    
  }
                             #endif DCFMOD                             
                             #ifdef DCFTINY 
 static uint32_t IntervalCounter = 100;
 
 if(DCF_ReadFulltime)
   {
    SumSecondSignal += (1-digitalRead(DCF_PIN)); 
    SumSignalCounts++; 
   }
 else
   {
    if((millis() - IntervalCounter) >= mSecInterval)               // count ever 20 msec 50 readings for DCF-receiver 
      { 
       IntervalCounter = millis();      
       for (int n=0; n<DCFreadingsInLoop; n++) { SumSecondSignal += (1-digitalRead(DCF_PIN));  } 
       SumSignalCounts+=DCFreadingsInLoop;                             // Noof of counted signals  
      }
    }
 if( Isecond != lastsecond)    //           (millis() - DCFmsTick) >998) //= 1000)               // Compute every second the received DCF-bit to a time 
   { 
    lastsecond = Isecond;
//    DCFmsTick = millis(); 
    if(byte OKstatus = UpdateDCFclock())        // If after 60 sec a valid time is calculated, sent it to the HC-12 module
      {
      if(OKstatus == 2)                         // If time flag was OK and date flag was NOK
        {
         sprintf(sptext,"       TIME OK --> %0.2d:%0.2d  ",Dhour, Dminute); Serial.println(sptext);           
         //  RTCklok.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Dhour, Dminute, 0));
         Ed1++;
        }
      if(OKstatus == 1)                         // If time flag was OK and date flag also OK  (OKstatus == 1)
        {
//         sprintf(sptext,"TIME & DATE OK --> %0.2d:%0.2d %0.2d-%0.2d-%0.4d/%0.1d ",Dhour, Dminute, Dday, Dmonth, 2000+Dyear, Dwday );
//         Serial.println(sptext);
         DateTime now = RTCklok.now();
         DateTime DCFtijd (Dyear, Dmonth, Dday, Dhour, Dminute, 0);
         if(abs(now.unixtime() - DCFtijd.unixtime()) > 2 )                     // > in Seconds. If difference between dcf and rtc time update the RTC
           {
            Serial.println(F("RTC Updating"));   // Following line sets the RTC to the date & time this sketch was compiled
            RTCklok.adjust(DateTime(Dyear, Dmonth, Dday, Dhour, Dminute, 0));
            setTime(Dhour, Dminute, 0, Dyear, Dmonth, Dday);
            DCFmsTick = millis();                                              // Start counting again after time setting  nodig ???
           }
         DCFlocked = true;                       // DCF time stored (for one hour before unlock)
         DCF_signal += 2; 
         DCF_signal = constrain( DCF_signal,1,99);
         Ed++;
         TellerEd++;
         DCFEd = true;                           // turn off DCF received at the same minute
        }         
      }
   } 
//  if(LHbitmeet) SignalBin[(millis() - DCFmsTick)/100] = 1;                   // Storing the bits disrupts the signal too much
                             #endif DCFTINY               
} 
                             #ifdef DCFTINY
//--------------------------------------------
// CLOCK Make the time from the received DCF-bits
//--------------------------------------------
byte UpdateDCFclock(void)
{
 byte        TimeOK, Receivebit, Receivebitinfo;                               // Return flag is proper time is computed and received bit value
 byte        BMinutemark, BReserveantenna, BOnehourchange;
 byte        BSummertime, BWintertime    , BLeapsecond;
 bool        TimeSignaldoubtfull = false;                                      // If a bit length signal is not between 5% and 30%
 static byte Bitpos, Paritybit;                                                // Postion of the received second signal in the outer ring  
 static byte MinOK=2, HourOK=2, YearOK=2;                                      // static because the content must be remembered

 if (Bitpos >= 60) 
     {
      Bitpos = TimeOK = TimeSignaldoubtfull = Dsecond = Dminute = Dhour = Dday = Dwday = Dmonth = Dyear = 0;
      MinOK = HourOK = YearOK = 2;
      if (DCFEd && DCFThijs) EdThijsGelijk++;
      DCFEd = DCFThijs = false;                                                 // turn off DCF received at the same minute
                           #ifdef LEDsInstalled
      LedsOffOuterring();
      ColorLeds("", MinPar,YearPar, orange);                                    // Colour MinPar, HourPar and YearPar orange
                           #endif LEDsInstalled                          
      } 
 int msec  = (int)(1000 * SumSecondSignal / SumSignalCounts);                   // This are roughly the milliseconds times 10 of the signal length
 switch(msec)
  {
   case   0 ...  10: if (SumSignalCounts > 50)                                  // 
                       {Bitpos = 59; Receivebitinfo = 2; Receivebit = 0; }      // If enough signals and one second no signal found. This is position zero
                              else { Receivebitinfo = 9; Receivebit = 0; }      // If not it is a bad signal probably a zero bit
                                                                        break;  // If signal is less than 0.05 sec long than it is the start of minute signal
   case  11 ...  50: Receivebitinfo = 8; Receivebit = 0;                break;  // In all other cases it is an error
   case  51 ... 160: Receivebitinfo = 0; Receivebit = 0;                break;  // If signal is 0.1 sec long than it is a 0 
   case 161 ... 280: Receivebitinfo = 1; Receivebit = 1;                break;  // If signal is 0.2 sec long than it is a 1 
   default:          Receivebitinfo = 9; Receivebit = 1;                break;  // In all other cases it is an error probably a one bit
  }
                            #ifdef LEDs24RingInstalled
 if (Bitpos ==  0) { BitColour = yellow; LedsOff(); } 
 if (Bitpos == 20) { BitColour = blue;   LedsOff(); }
 if (Bitpos == 36) { BitColour = green;  LedsOff(); }

 ColorLeds("", Bitpos % 24, Bitpos % 24 , Receivebit<2 ? (Receivebit == 0 ? BitColour : red ) : (Receivebit == 9 ? white : green) );
 ShowLeds();
                           #endif LEDs24RingInstalled 
 if(Receivebitinfo > 2)  SignalFaults++;                                        // Count the number of faults per hour
 switch (Bitpos)                                                                // Decode the 59 bits to a time and date.
  {                                                                             // It is not binary but "Binary-coded decimal". Blocks are checked with an even parity bit. 
   case   0: BMinutemark = Receivebit;
                             #ifdef LEDsInstalled 
             ColorLed(Minutemark,(Receivebit ? white : green));
                             #endif LEDsInstalled
                                                                        break;  // Bit must always be 0
   case 1 ... 14:                                                       break;  // reserved for wheather info we will not use
   case  15: BReserveantenna = Receivebit;
                             #ifdef LEDsInstalled
             ColorLed(Reserveantenna, (Receivebit ? red : white));
                             #endif LEDsInstalled
                                                                        break;  // 1 if reserve antenna is in use
   case  16: BOnehourchange  = Receivebit;
                             #ifdef LEDsInstalled  
             ColorLed(Onehourchange, (Receivebit ? red : white));
                             #endif LEDsInstalled
                                                                        break;  // Has value of 1 one hour before change summer/winter time
   case  17: BSummertime = Receivebit;
             if(Receivebit) CSummertime++; 
             else           CSummertime--;
             CSummertime = constrain(CSummertime,5,60);                
                             #ifdef LEDsInstalled
             ColorLed(Summertime, (CSummertime>50 ? orange : white));
                             #endif LEDsInstalled
                                          break;                                // 1 if summer time 
   case  18: BWintertime = Receivebit; 
             if(Receivebit) CWintertime++;
             else           CWintertime--;
             CWintertime = constrain(CWintertime,5,60);
                             #ifdef LEDsInstalled
             ColorLed(Wintertime, (CWintertime>50 ? blue : white ));
                             #endif LEDsInstalled
                                                                        break;  // 1 if winter time
   case  19: BLeapsecond = Receivebit;  
                                                                        break;  // Announcement of leap second in time setting is coming up
   case  20: StartOfEncodedTime = Receivebit;                                   // Bit must always be 1                
             Paritybit = 0;                                             break;
   case  21: if(Receivebit) {Dminute  = 1;  Paritybit = 1 - Paritybit;} break;
   case  22: if(Receivebit) {Dminute += 2 ; Paritybit = 1 - Paritybit;} break;
   case  23: if(Receivebit) {Dminute += 4 ; Paritybit = 1 - Paritybit;} break;
   case  24: if(Receivebit) {Dminute += 8 ; Paritybit = 1 - Paritybit;} break;
   case  25: if(Receivebit) {Dminute += 10; Paritybit = 1 - Paritybit;} break;
   case  26: if(Receivebit) {Dminute += 20; Paritybit = 1 - Paritybit;} break;
   case  27: if(Receivebit) {Dminute += 40; Paritybit = 1 - Paritybit;} break;
   case  28: MinOK = (Receivebit==Paritybit);                                   // The minute parity is OK or NOK
                     #ifdef LEDsInstalled
             ColorLed( MinPar, MinOK ? green : red);
                     #endif LEDsInstalled
             Paritybit = 0;                                             break;    
   case  29: if(Receivebit) {Dhour   =  1; Paritybit = 1 - Paritybit;}  break;
   case  30: if(Receivebit) {Dhour  +=  2; Paritybit = 1 - Paritybit;}  break;
   case  31: if(Receivebit) {Dhour  +=  4; Paritybit = 1 - Paritybit;}  break;
   case  32: if(Receivebit) {Dhour  +=  8; Paritybit = 1 - Paritybit;}  break;
   case  33: if(Receivebit) {Dhour  += 10; Paritybit = 1 - Paritybit;}  break;
   case  34: if(Receivebit) {Dhour  += 20; Paritybit = 1 - Paritybit;}  break;
   case  35: HourOK = (Receivebit==Paritybit);                                  // The hour parity is OK or NOK
                     #ifdef LEDsInstalled
             ColorLed( HourPar, HourOK ? green : red);                                    
                     #endif LEDsInstalled             
             Paritybit = 0;                                             break;  
   case  36: if(Receivebit) {Dday    =  1; Paritybit = 1 - Paritybit;}  break;
   case  37: if(Receivebit) {Dday   +=  2; Paritybit = 1 - Paritybit;}  break;
   case  38: if(Receivebit) {Dday   +=  4; Paritybit = 1 - Paritybit;}  break;
   case  39: if(Receivebit) {Dday   +=  8; Paritybit = 1 - Paritybit;}  break;
   case  40: if(Receivebit) {Dday   += 10; Paritybit = 1 - Paritybit;}  break;
   case  41: if(Receivebit) {Dday   += 20; Paritybit = 1 - Paritybit;}  break;
   case  42: if(Receivebit) {Dwday   =  1; Paritybit = 1 - Paritybit;}  break;
   case  43: if(Receivebit) {Dwday  +=  2; Paritybit = 1 - Paritybit;}  break;
   case  44: if(Receivebit) {Dwday  +=  4; Paritybit = 1 - Paritybit;}  break;
   case  45: if(Receivebit) {Dmonth  =  1; Paritybit = 1 - Paritybit;}  break;
   case  46: if(Receivebit) {Dmonth +=  2; Paritybit = 1 - Paritybit;}  break;
   case  47: if(Receivebit) {Dmonth +=  4; Paritybit = 1 - Paritybit;}  break;
   case  48: if(Receivebit) {Dmonth +=  8; Paritybit = 1 - Paritybit;}  break;
   case  49: if(Receivebit) {Dmonth += 10; Paritybit = 1 - Paritybit;}  break;
   case  50: if(Receivebit) {Dyear   =  1; Paritybit = 1 - Paritybit;}  break;
   case  51: if(Receivebit) {Dyear  +=  2; Paritybit = 1 - Paritybit;}  break;
   case  52: if(Receivebit) {Dyear  +=  4; Paritybit = 1 - Paritybit;}  break;
   case  53: if(Receivebit) {Dyear  +=  8; Paritybit = 1 - Paritybit;}  break;
   case  54: if(Receivebit) {Dyear  += 10; Paritybit = 1 - Paritybit;}  break;
   case  55: if(Receivebit) {Dyear  += 20; Paritybit = 1 - Paritybit;}  break;
   case  56: if(Receivebit) {Dyear  += 40; Paritybit = 1 - Paritybit;}  break;
   case  57: if(Receivebit) {Dyear  += 80; Paritybit = 1 - Paritybit;}  break;
   case  58: YearOK = (Receivebit==Paritybit);                                    // The year month day parity is OK or NOK
                     #ifdef LEDsInstalled
              ColorLed(YearPar, YearPar ? green : red); 
                     #endif LEDsInstalled   
              Paritybit = 0;                                            break;  
   case  59: //Serial.print("silence");             
             TimeOK = 0;    
             StartOfEncodedTime = 0;            
             if( MinOK && HourOK && YearOK) 
               {  
                if(abs((((Dday*24+Dhour)*60)+Dminute)-(((Iday*24+Ihour)*60)+Iminute))<300){TimeOK = 1;} // < 300 seconds
               }              
             if( MinOK && HourOK && !YearOK) 
               { 
               if (abs((((Dday*24+Dhour)*60)+Dminute)-(((Iday*24+Ihour)*60)+Iminute))<300){TimeOK = 2;} 
               }                            
             if(Dmonth ==0 || Dmonth >12 || Dhour >23 || Dminute>59) TimeOK = 0; // If time is definitely wrong return Time is not OK 
             if(DCFEd>10 && abs(((Dyear*12+Dmonth)*31)-(((Iyear-2000)*12+Dmonth)*31))>400){TimeOK = 0;}  // if a few DCF signal are received valid time control will be tighter  
                                                                        break;
    default:  Serial.println(F("Default in BitPos loop. Impossible"));  break;
   }
  if((Bitpos>19 && Bitpos < 36) && Receivebitinfo > 2) TimeSignaldoubtfull = true;// If a date time bit is received and the received bit is not 0 or 1 then time becomes doubtfull
  if(PrintDebugInfo)
    {
     sprintf(sptext,"@ %s%s%s ", (MinOK<2?(MinOK?"M":"m"):"."),(HourOK<2?(HourOK?"H":"h"):"."),(YearOK<2?(YearOK?"Y":"y"):"."));      Serial.print(sptext);
     sprintf(sptext,"%4ld %5ld %2ld%%, Sec:%2d (%d) %0.2d:%0.2d %0.2d-%0.2d-%0.4d/%0.1d ?%d F%d",
                     SumSecondSignal, SumSignalCounts, 100*SumSecondSignal/SumSignalCounts, Bitpos, 
                     Receivebitinfo, Dhour, Dminute, Dday, Dmonth, 2000+Dyear, Dwday, TimeSignaldoubtfull, SignalFaults);             Serial.print(sptext); 
//     sprintf(sptext," Ed:%2d%% Th:%2d%% Both:%ld Ed:%ld Th:%ld",(int)(TellerEd), (int)(TellerTh),EdThijsGelijk, Ed, Thijs);           Serial.println(sptext); 
     sprintf(sptext," Ed:%ld Th:%ld  Both:%ld  Tot:%ld ",Ed, Thijs,EdThijsGelijk, MinutesSinceStart);           Serial.println(sptext); 
    }
                            #ifdef MAX7219info
 switch (Bitpos)                                                                // Decode the 59 bits to a time and date.
  {                                                                             // It is not binary but "Binary-coded decimal". Blocks are checked with an even parity bit. 
   case 0 ... 19: sprintf(sptext,"%0.3d %0.2d%0.2d",(int)(1000*SumSecondSignal/SumSignalCounts), TotalEff, (int)(100*Ed/MinutesSinceStart)); break;
        default:  sprintf(sptext,"%0.3d %0.2d%0.2d",(int)(1000*SumSecondSignal/SumSignalCounts),Dhour, Dminute);
  }
       
    PrintStringToSegDisplay(sptext); //   Serial.println(sptext);
                            #endif MAX7219info       
// sprintf(sptext,"\%d%d%d%d%d%d%d%d%d%d/", SignalBin[0],SignalBin[1],SignalBin[2],SignalBin[3],SignalBin[4], SignalBin[5],SignalBin[6],SignalBin[7],SignalBin[8],SignalBin[9]);  Serial.println(sptext);

                             #ifdef LEDsInstalled
 ColorLedOuterRing(Bitpos, Receivebitinfo<2 ? (Receivebitinfo == 0 ? blue : red ) : yellow) ; 
 ColorLed(DCFgoodminutes, SignalFaults<61 ? (SignalFaults<31 ? (SignalFaults <16 ? (SignalFaults <6 ? green : purple) : yellow ) : orange) : red);
 ColorLed(TimeDoubfull,  TimeSignaldoubtfull ? red : green);
 ColorLed(DCF_tijd,      DCFlocked ?  blue : white); 
 ColorLed(RTC_tijd,      DCFlocked ? white : red  );
                             #endif LEDsInstalled
 SumSecondSignal = SumSignalCounts = 0;
// for (int x=0; x<10; x++) SignalBin[x]=0;
 Bitpos++;
return(TimeOK);
}                 
                             #endif DCFTINY
//--------------------------------------------
// CLOCK check for serial input
//--------------------------------------------
void SerialCheck(void)
{
 String  SerialString; 
 while (Serial.available())
  {
   delay(3);  
   char c = Serial.read();
   if (c>31 && c<128) SerialString += c;       // allow input from Space - Del
  }
 if (SerialString.length()>0) 
         ReworkInputString(SerialString);      // Rework ReworkInputString();
 SerialString = "";
//                           #ifdef HC12MOD
// HC12Check();
//                           #endif HC12MOD 
                           #ifdef BLUETOOTHMOD   
 BluetoothCheck(); 
                           #endif BLUETOOTHMOD  
}
                           #ifdef HC12MOD
//--------------------------------------------
// CLOCK check for HC-12 input
//--------------------------------------------                            
void HC12Check(void)
{
 static bool DataInBuffer = false;
 String      HC12String; 
 static long HC12StartTime = millis(); 
 char        c;
 HC12String.reserve(64);
 HC12String ="";
 HC12.listen(); //When using two software serial ports, you have to switch ports by listen()ing on each one in turn.
 while (HC12.available()>0)                            // If HC-12 has data
   {       
    c = HC12.read();     
    if (c>31 && c<128)  HC12String += c;              // Allow only input from Space - Del
    else c = 0;
    delay(3);
   }
 DataInBuffer = false;
 if (HC12String.length()>0) 
   {
    Serial.print("Received HC-12: "); Serial.println(HC12String);
//     ReworkInputString(HC12String);
    DataInBuffer = false;  
    }                  
}                         
                           #endif HC12MOD
                           #ifdef BLUETOOTHMOD
//--------------------------------------------
// CLOCK check for Bluetooth input
//--------------------------------------------                           
void BluetoothCheck(void)
{ 
 Bluetooth.listen();  //  When using two software serial ports, you have to switch ports by listen()ing on each one in turn.
 while (Bluetooth.available()) 
  {
   delay(3); 
                           #if defined(ARDUINO_SAMD_MKRWIFI1010)
   char c = Serial1.read();
                           #else if
   char c = Bluetooth.read();
                           #endif   
   Serial.print(c);
   if (c>31 && c<128) BluetoothString += c;
   else c = 0;     // delete a CR
  }
 if (BluetoothString.length()>0)  
      ReworkInputString(BluetoothString);       // Rework ReworkInputString();
 BluetoothString = "";
}
                           #endif BLUETOOTHMOD  
  
//--------------------------------------------
// DS3231 Get time from DS3231
//--------------------------------------------
void GetTijd(byte printit)
{
 Inow    = RTCklok.now();
 Ihour   = Inow.hour();
 Iminute = Inow.minute();
 Isecond = Inow.second();
 Iday    = Inow.day();
 Imonth  = Inow.month();
 Iyear   = Inow.year();
// if (Ihour > 24) { Ihour = random(12)+1; Iminute = random(60)+1; Isecond = 30;}  // set a time if time module is absent or defect
 if (printit)  Print_RTCtijd(); 
}


//--------------------------------------------
// DS3231 utility function prints time to serial
//--------------------------------------------
void Print_RTCtijd(void)
{
 sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d DCFsignal:%0.2d",Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),Inow.month(),Inow.year(),DCF_signal);
 Tekstprintln(sptext);
                              #ifdef TM1637tijd
 Tijd_display.showNumberDecEx( ((Inow.hour() * 100) + Inow.minute()), 0b11100000, true);    // Display the current time in 24 hour format with leading zeros enabled and a center colon:
 //   Tijd_display.showNumberDec(displaytime, true); // Prints displaytime without center colon.  
                              #endif TM1637tijd  
}
                    #ifdef LCDMOD
//--------------------------------------------
// CLOCK Print time to LCD display
//--------------------------------------------
void Print_tijd_LCD(void)
{
 lcd.home (); // set cursor to 0,0
 sprintf(sptext,"%0.2d:%0.2d:%0.2d",Inow.hour(),Inow.minute(),Inow.second());   lcd.print(sptext);
 sprintf(sptext," LDR%d   ",analogRead(PhotoCellPin));                          lcd.print(sptext);
 lcd.setCursor (0,1);        // go to start of 2nd line
 sprintf(sptext,"%0.2d-%0.2d-%0.4d",Inow.day(),Inow.month(),Inow.year());       lcd.print(sptext);
 sprintf(sptext," DCF%d   ",DCF_signal);                                        lcd.print(sptext);  // kan ook nog variable TotalEff invullen ipv DCF_signal
}
                    #endif LCDMOD
//--------------------------------------------
// CLOCK utility function prints time to serial
//--------------------------------------------
void Print_tijd(void)
{
 sprintf(sptext,"%0.2d:%0.2d:%0.2d",Ihour,Iminute,Isecond);
// sprintf(sptext,"%0.2d:%0.2d:%0.2d",hour(),minute(),second());
 Tekstprintln(sptext);
}

//--------------------------------------------
// DS3231 Set time in module and print it
//--------------------------------------------
void SetRTCTime(void)
{ 
 Ihour   = constrain(Ihour  , 0,24);
 Iminute = constrain(Iminute, 0,59); 
 Isecond = constrain(Isecond, 0,59); 
 RTCklok.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Ihour, Iminute, Isecond));
 GetTijd(0);                                      // synchronize time with RTC clock
 Print_tijd();
}
//--------------------------------------------
// DS3231 Get temperature from module
//--------------------------------------------
int get3231Temp(void)
{
 byte tMSB, tLSB;
 int temp3231;
  
 Wire.beginTransmission(DS3231_I2C_ADDRESS);    //temp registers (11h-12h) get updated automatically every 64s
 Wire.write(0x11);
 Wire.endTransmission();
 Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
 if(Wire.available()) 
  {
    tMSB = Wire.read();                          // 2's complement int portion
    tLSB = Wire.read();                          // fraction portion 
    temp3231 = (tMSB & B01111111);               // do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ) + 0.5;    // only care about bits 7 & 8 and add 0.5 to round off to integer   
  }
 else {  temp3231 = -273; }   
 return (temp3231);
}

// ------------------- End  Time functions 
                     #if defined LEDsInstalled || defined LEDs24RingInstalled
// --------------------Colour Clock Light functions -----------------------------------
//--------------------------------------------
//  LED Set color for several LEDs
//--------------------------------------------
void ColorLeds(char* Tekst, int FirstLed, int LastLed, uint32_t RGBWColor)
{ 
 strip.fill(RGBWColor, FirstLed, ++LastLed - FirstLed );   
 // Serial.println(RGBWColor,HEX); 
 if (strlen(Tekst)>0 ) {sprintf(sptext,"%s ",Tekst); Tekstprint(sptext); }   // Print the Tekst  
}
//--------------------------------------------
//  LED Set color for one LED
//--------------------------------------------
void ColorLed(int LedNr, uint32_t RGBWColor)
{ 
 strip.setPixelColor(LedNr, RGBWColor);   // for (int n = FirstLed; n <= LastLed; n++)  strip.setPixelColor(n,RGBWColor  );
// Serial.print(LedNr); Serial.println(RGBWColor,HEX);  
}
//--------------------------------------------
//  LED Set color for  LED in outer ring 
//--------------------------------------------
void ColorLedOuterRing(int LedNr, uint32_t RGBWColor)
{
 strip.setPixelColor(LedNr, RGBWColor); 
}

//--------------------------------------------
//  LED color middle ring. 
//  Faults are no of signals not 0.1 or 0.2 seconds long 
//--------------------------------------------
void ColorMiddleRing(byte Fhour ,uint32_t Nooffaults)
{
  switch(Nooffaults)
  {
   case   0 ...   5:    ColorLed(60 + Fhour,  green ); break;
   case   6 ...  15:    ColorLed(60 + Fhour,  apple ); break; 
   case  16 ...  30:    ColorLed(60 + Fhour,  marine); break;  
   case  31 ...  60:    ColorLed(60 + Fhour,  blue  ); break; 
   case  61 ... 120:    ColorLed(60 + Fhour,  orange); break; 
   case 121 ... 0xFFFE: ColorLed(60 + Fhour,  red   ); break; 
   case 0xFFFF:         ColorLeds("", 60, 95, purple);          // Reset and color the middle ring purple
   default:                                             break; 
  }
 if (Fhour==23) Fhour = 0;
 ColorLed(61 + Fhour, white);                                   // colour the next hour white
}
//--------------------------------------------
//  LED Clear display settings of the LED's
//--------------------------------------------
void LedsOff(void) 
{ 
 strip.fill(0, 0, NUM_LEDS);      // Turn all LEDs off 
}

void LedsOffOuterring(void) 
{ 
 strip.fill(0x000032 , 0, 60);    // Color the outer seconds info ring LEDs faint blue
}
void LedsOffMiddlering(void) 
{ 
 strip.fill(0, 60, 24);           // Turn LEDs of hour quality middle ring off
}
void LedsOffInnerring(void) 
{ 
 strip.fill(0, 84, 12);           // Turn LEDs of information middle ring off 
}
//--------------------------------------------
//  LED Push data in LED strip to commit the changes
//--------------------------------------------
void ShowLeds(void)
{
 strip.show();                   // Send the signal to the LEDs
}
//--------------------------------------------
//  LED Set brighness of LEDs
//--------------------------------------------  
void SetBrightnessLeds( byte Bright)
{
 strip.setBrightness(Bright);    // Brighness of the LEDs
 ShowLeds();
}
//--------------------------------------------
//  LED Dim the leds by PWM measured by the LDR and print values
//--------------------------------------------
void DimLeds(byte print) 
{                                                                                          
 if (SecPulse)   // if a second has passed 
 {
  int BrCalc, Temp;
  int LDR_read = (4 * Previous_LDR_read + analogRead(PhotoCellPin)) / 5;   // Read lightsensor 
  Previous_LDR_read = LDR_read;
  OutPhotocell = (int) (((float)LightReducer/100) * sqrt( (float) 63.5 * (float) constrain(LDR_read,1,1023))); // Linear --> hyperbolic with sqrt
  MinPhotocell = MinPhotocell > LDR_read ? LDR_read : MinPhotocell;
  MaxPhotocell = MaxPhotocell < LDR_read ? LDR_read : MaxPhotocell;
 // sprintf(sptext,"LDR read: %d MinPhotocell %d\n",LDR_read, MinPhotocell);   Tekstprint(sptext);
  byte BrightnessCalcFromLDR = constrain(OutPhotocell, LowerBrightness , 255);  // filter out of strange results
  BrCalc = (int) (BrightnessCalcFromLDR/2.55);
  if(print)
  {
   sprintf(sptext,"LDR:%d=%d%% ",LDR_read, BrCalc);  //  T=%dC ,Get3231Temp()-2);
   Tekstprint(sptext);
                              #if defined DCFMOD || defined DCFTINY 
   sprintf(sptext,"DCF Ed:%ld Th:%ld  Both:%ld  Tot:%ld ",Ed, Thijs,EdThijsGelijk, MinutesSinceStart); 
   Tekstprint(sptext);                                                         //   sprintf(sptext," DCF:%d%% ",DCF_signal);   Tekstprint(sptext);
                              #endif DCFMOD || DCFTINY                            
   Print_tijd();
  }
  SetBrightnessLeds(BrightnessCalcFromLDR);                                      
 }
 SecPulse = 0;
}

//--------------------------------------------
//  LED function to make RGBW color
//-------------------------------------------- 
uint32_t FuncCRGBW( uint32_t Red, uint32_t Green, uint32_t Blue, uint32_t White)
{ 
 return ( (White<<24) + (Red << 16) + (Green << 8) + Blue );
}
//--------------------------------------------
//  LED functions to extract RGBW colors
//-------------------------------------------- 
 uint8_t Cwhite(uint32_t c) { return (c >> 24);}
 uint8_t Cred(  uint32_t c) { return (c >> 16);}
 uint8_t Cgreen(uint32_t c) { return (c >> 8); }
 uint8_t Cblue( uint32_t c) { return (c);      }

                     #endif LEDsInstalled || defined LEDs24RingInstalled
//--------------------------------------------
//  MAX7219 Print in MAX7219 with 16 digits
//--------------------------------------------
                     #ifdef MAX7219info
void PrintStringToSegDisplay(char *text)
{
 for (int n = 0; n<16;n++)
  {
   int d = text[n] - '0';  
   lc.setDigit((n/8), 7-n%8,d ,false);
  }
}
                     #endif MAX7219info 

//--------------------------------------------
//  LED In- or decrease light intensity value
//--------------------------------------------
void WriteLightReducer(float amount)
{
 LightReducer += amount; 
 WriteLightReducerEeprom(LightReducer);
}

//--------------------------------------------
//  LED Write light intensity to EEPROM
//--------------------------------------------
void WriteLightReducerEeprom(byte waarde)
{
 LightReducer = constrain (waarde, 0 , 255);  // May not be larger than 255
 EEPROMwrite(0, LightReducer);                // Store the value (0-250) in permanent EEPROM memory at address 0
 sprintf(sptext,"Max brightness: %3d%%",LightReducer);
 Tekstprintln(sptext);
// Serial.print(millis() - RotaryPressTimer); Serial.print(" msec ------- ");
// Serial.print(F("LightReducer: ")); Serial.print(LightReducer * 100); Serial.println("%");
}

//--------------------------------------------
//  LED Write lowest allowable light intensity to EEPROM
//--------------------------------------------
void WriteLowerBrightness(byte waarde)
{
 LowerBrightness = constrain (waarde, 0 , 150);      // Range between 1 and 100
 EEPROMwrite(1, LowerBrightness);                    // Default Lower Brightness for this clock
 sprintf(sptext,"Lower brightness: %3ld bits",(long) LowerBrightness);
 Tekstprintln(sptext);
}
//--------------------------------------------
//  LED Write DCF loop values to EEPROM
//--------------------------------------------
void WritemSecInterval(int waarde)
{
EEPROMwrite(2, waarde & 0x00FF);
EEPROMwrite(3, waarde & 0xFF00);  
}
//--------------------------------------------
void WriteDCFreadingsInLoop(int waarde)
{
EEPROMwrite(4,waarde & 0x00FF);
EEPROMwrite(5,waarde & 0xFF00);
}
//--------------------------------------------
//  LED Read DCF loop values from EEPROM
//--------------------------------------------
int ReadSecInterval(void)
{
return EEPROMread(2) + EEPROMread(3)<<8;  
}
//--------------------------------------------
int ReadDCFreadingsInLoop(void)
{
return EEPROMread(4) + EEPROMread(5)*256;
}
//--------------------------------------------
//  Write to EEPROM
//--------------------------------------------
void EEPROMwrite(byte pos, byte waarde)
{ 
  EEPROM.write(pos, waarde);     
                     #if defined(ARDUINO_SAMD_MKRWIFI1010)
  EEPROM.commit();
                     #endif 
}
//--------------------------------------------
//  Read  EEPROM
//--------------------------------------------
byte EEPROMread(byte pos)  
{ 
  return EEPROM.read(pos);
}

//--------------------------------------------
//  CLOCK Input from Bluetooth or Serial
//--------------------------------------------
void ReworkInputString(String InputString)
{
 String temp;
 InputString.toCharArray(sptext, MAXTEXT-1);

 if ( InputString[0] > 64 )                                           // Does the string start with a letter?
  {
  int val = InputString[0];
  int FMfreq;
  
  Tekstprintln(sptext);
  switch (val)
   {
    case 'A':
    case 'a':
             PrintDebugInfo = 1 - PrintDebugInfo;
             break;  
    case 'D':
    case 'd':  
             if (InputString.length() == 9 )
              {
               int Jaar;
               temp   = InputString.substring(1,3);     Iday = (byte) temp.toInt(); 
               temp   = InputString.substring(3,5);   Imonth = (byte) temp.toInt(); 
               temp   = InputString.substring(5,9);     Jaar =  temp.toInt(); 
               Iday   = constrain(Iday  , 0, 31);
               Imonth = constrain(Imonth, 0, 12); 
               Jaar   = constrain(Jaar , 1000, 9999); 
               RTCklok.adjust(DateTime(Jaar, Imonth, Iday, Inow.hour(), Inow.minute(), Inow.second()));
//               sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d",Inow.hour(),Inow.minute(),Inow.second(),Iday,Imonth,Jaar);
//               Tekstprintln(sptext);
              }
              else Tekstprintln("**** Length fault. Enter ddmmyyyy ****");
             break;       
    case 'F':
    case 'f':   
             DCF_ReadFulltime   = 1 - DCF_ReadFulltime;               // Toggle DCF77 Full to interval readings
             sprintf(sptext,"DCF readings changed to: %s ", DCF_ReadFulltime ? "Constant reading" : "Interval reading" );
             Tekstprintln(sptext);
             break;       
    case 'G':                                                         // Change the DCF loop interval time
    case 'g':    
             temp = InputString.substring(1);
             mSecInterval   = temp.toInt();
             mSecInterval   = constrain(mSecInterval  , 0, 1000);
             WritemSecInterval(mSecInterval);
             sprintf(sptext,"Interval changed to: %d msec",mSecInterval);
             Tekstprintln(sptext);
             DCF_ReadFulltime     = false;
             break;
    case 'H':                                                         // Change the DCF no of loops in an interval time 
    case 'h':    
             temp = InputString.substring(1);
             DCFreadingsInLoop   = temp.toInt();
             DCFreadingsInLoop   = constrain(DCFreadingsInLoop  , 1, 1000);
             WriteDCFreadingsInLoop(DCFreadingsInLoop);
             sprintf(sptext,"DCF readings In Loop changed to: %d reads / loop",DCFreadingsInLoop);
             Tekstprintln(sptext);
             DCF_ReadFulltime     = false;
             break;
    case 'I':
    case 'i':   
             SWversion();
             break;      
    case 'L':                                                         // Lowest value for Brightness
    case 'l':    
             temp = InputString.substring(1);
             LowerBrightness   = temp.toInt();
             LowerBrightness   = constrain(LowerBrightness  , 1, 255);
             WriteLowerBrightness(LowerBrightness);
             sprintf(sptext,"Lower brightness changed to: %d bits",LowerBrightness);
             Tekstprintln(sptext);
             break;
    case 'M':                                                         // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'm':    
             temp = InputString.substring(1);
             byte Intensityfactor = temp.toInt();
             Intensityfactor      = constrain(Intensityfactor, 1, 255);
             WriteLightReducerEeprom(Intensityfactor);
             sprintf(sptext,"Max brightness changed to: %d%%",Intensityfactor);
             Tekstprintln(sptext);
             break; 
    case 'R':
    case 'r':
            if (InputString.length() == 1)
              {
               Reset();                                                 // Reset all settings 
               Tekstprintln("**** Resetted to default settings ****"); 
              }
            break; 
    default:
             break;
    }
  }
   else if (InputString.length() > 3 && InputString.length() <7 )
      {
       temp = InputString.substring(0,2);   
       Ihour = temp.toInt(); 
       if (InputString.length() > 3) { temp = InputString.substring(2,4); Iminute = temp.toInt(); }
       if (InputString.length() > 5) { temp = InputString.substring(4,6); Isecond = temp.toInt(); }
       SetRTCTime();
      }
 InputString = "";
 temp = "";
}
