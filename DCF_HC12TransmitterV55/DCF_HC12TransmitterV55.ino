// =============================================================================================================================
/* 
This Arduino code works on a ATMEGA328 ( Nano Uno), ATMEGA1284 chip and Nano Every
This source contains code for the following modules:  
- RTC DS3231 clock module
- LDR light sensor 5528
- HC-12 Long Range Wireless Communication Module
- DCF77 module DCF-2
The DCF77 module reads the time and date from the German longwave DCF-77 time signal
The time and date are transmitted as ASCII-string as hhmmss. (031500 = 15 minutes past 3)
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
V38b Minor changes  Ed:3429 Th:3520  Both:2490  Tot:5518 Last stable version
V38c Failure . HC12 and BT can not be used together. BT works when HC12 is removed. softwareserial lib?. Serial1 will not initialize
V38d https://mcudude.github.io/MegaCoreX/package_MCUdude_MegaCoreX_index.json
V38f Added ValidTimes
V38g Bluetooth on pin 6,7 HC12 on 8 and 12, pin 11 gets every second interfering false data 
V39  Stable version. moved Bluetooth- and Serialcheck to Everysecond subroutine. Readings went up from 10000 to 35000/s!
V40a Moved time calculation from Bitpos 59 to bitpos 0. Added statistics Ed:1052 Th:959 Both:901 Valid:1109 Tot:1173 
V40b Added EEPROM storage of Statistics  
V40c Tighter struct mem
V40d Get data from EEPROM at startup
V40e debugging   Ed:857 Th:1147 Min:1175
V41  >999msec changed to  if( Inow.second() != lastsecond)   . changed it back was very bad if((millis() - DCFmsTick) >999)
V41a to Reset MAX7219 added InitialyseMAX7219.
v42  Added menu entries 
V43  Succes! added at line 731; while (1-digitalRead(DCF_PIN)); --> Ed:609 Th:608 Min:614 
V44  In DCFclock L001 with one 7219 display 10-05-2020
V45  Optimizing 
V46  Added software reset from menu. Solved MegaCoreX compiler problem with DCF77. Added LEDs are off at night
V47  Added interrupt to pin0 and disabled it. Connect a wire between DCF pin 2 and pin 0 to use it. Minor changes in DCFlock and DCF signal use 
V48  Stable version
V49  Cleanup after extracting the code for DCF77-NoIntV01.ino
V50  Optimized time decoding from DCF   Identical to DCF77-NoIntV03. Added short Debuginfo possibility as menu entry B
V51  Corrected overflow in statistics counters
V52  Added HT16K33tijd
V53  Cleaned up, Changed and corrected DCFtiny counters
V53a LDR reading in display
V54  Final upload in L001 station
V55  Store valid time counters in EEPROM    

Ed Nieuwenhuys June 2020
 */
// ===============================================================================================================================

//--------------------------------------------
// ARDUINO Definition of installed modules
// If a module is not present remove it from compilation 
// by adding the comment slashes, //, before the #define
//--------------------------------------------
#define DCFMOD             // Use the Arduino DCF77 library with interrupts
#define DCFTINY            // Use the Tiny DCF algorithm in this program

#define HC12MOD            // Use HC12 time transreceiver Long Range Wireless Communication Module
#define BLUETOOTHMOD       // Use  this define if Bluetooth needs other pins than pin 0 and pin 1

//#define LCDMOD           // LCD 2 x 16 installed 
#define HT16K33tijd        // 4-digit HT16K33 time display installed https://www.adafruit.com/product/879 Adafruit GFX library 
#define TM1637tijd         // 4-digit TM1637 time display installed   TM1637 by Avishorp
#define MAX7219info        // MAX7219 display

#define MOD_DS3231         // DS3231 RTC module installed

//#define LED6812          // choose between LED type RGB=LED2812 or RGBW=LED6812
#define LED2812
//#define LEDs24RingInstalled

const byte NUMBEROF7219DISPLAYS = 2;       // 1 or 2. Second display chained with first one
char STATION[] = "L001";  
char VERSION[] = "V055";
//--------------------------------------------
// ARDUINO Includes defines and initialysations
//--------------------------------------------
                      #if defined LED2812 || defined LED6812
#define LEDsInstalled
                      #endif LED2812 || LED6812

#include <Wire.h>
#include <RTClib.h>
#include <avr/wdt.h>
#include <TimeLib.h>                             // For time management 
                     #if defined LEDsInstalled || defined LEDs24RingInstalled
#include <Adafruit_NeoPixel.h>
                     #endif LEDsInstalled
#include <EEPROM.h>
                     #ifdef LCDMOD
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
                     #endif LCDMOD
                     #ifdef DCFMOD
#include <DCF77.h>
                     #endif DCFMOD
                     #ifdef BLUETOOTHMOD
#include <SoftwareSerial.h>        // Arduino     // for Bluetooth communication
                     #endif BLUETOOTHMOD
                     #ifdef HC12MOD
#include <SoftwareSerial.h>                       // For HC12
                     #endif HC12MOD 
                     #ifdef TM1637tijd 
#include <TM1637Display.h>
                     #endif TM1637tijd 
                     #ifdef HT16K33tijd
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"                //https://learn.adafruit.com/adafruit-led-backpack/overview?view=all
                     #endif HT16K33tijd
                     #ifdef MAX7219info
#include <LedControl.h> 
                      #endif MAX7219info 
//--------------------------------------------
// PIN Assigments
//--------------------------------------------                      
enum DigitalPinAssignments {      // Digital hardware constants ATMEGA 328 ----

 DCFgood      = 1,                 // DCF-signal > 50
 secondsPin   = 1,                 // Seconds
 Interrupt0   = 0,                 // Interrupt0
 DCF_PIN      = 2,                 // DCFPulse on interrupt pin
 CLK_TM1637   = 3,                 // PIN 3 TM1637Display
 DIO_TM1637   = 4,                 // PIN 4 TM1637Display
 LED_PIN      = 5,                 // LED rings on pin PIN 5
 BT_TX        = 6,                 // Rx-pin on BT-mod to TX-pin on Arduino
 BT_RX        = 7,                 // Tx-pin on BT-mod to RX-pin on Arduino   
 HC_12TX      = 8,                 // RXD on HC-12 to TX Pin on Arduino  
 HC_12RX      = 9,                 // TXD on HC-12 to RX Pin on Arduino
 MAX7219CLK   = 10,                // MAX7219CLK
 MAX7219CS    = 11,                // MAX7219CS 
 MAX7219DataIn= 12,                // MAX7219DataIn
 DCF_LED_Pin  = 13                 // Show DCF-signal
 };
 
enum AnalogPinAssignments {     // Analog hardware constants ----
 EmptyA0      = 0,                // A0
 EmptyA1      = 1,                // A1
 PhotoCellPin = 2,                // A2
 EmptyA3      = 3,                // A3
 SDA_pin      = 4,                // SDA pin
 SCL_pin      = 5,                // SCL pin
 EmptyA6      = 6,                // Empty
 EmptyA7      = 7};               // Empty

//--------------------------------------------
// COLOURS
//--------------------------------------------   
                         #ifdef LED2812
const uint32_t white     = 0x00FFFFFF;                     // White is R, G and B on WS2812                
                         #endif LED2812
                         #ifdef LED6812    
const uint32_t white     = 0xFF000000;                     // The SK6812 LED has a white LED that is pure white
                         #endif LED6812  
const uint32_t black  = 0x000000, red   = 0xFF0000, orange = 0xFF7000;
const uint32_t yellow = 0xFFFF00, apple = 0x80FF00, brown  = 0x503000;
const uint32_t green  = 0x00FF00, grass = 0x00FF80, sky    = 0x00FFFF;
const uint32_t marine = 0x0080FF, blue  = 0x0000FF, pink   = 0xFF0080; 
const uint32_t purple = 0xFF00FF, softwhite = 0x606060;

//--------------------------------------------
// LED
//--------------------------------------------
bool LEDsAreOff = false;                                   // If true LEDs are off except time display
                      #ifdef LEDsInstalled
                         #ifdef LED6812 
const byte NUM_LEDS  = 96;                                 // How many leds in  strip?   
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRBW + NEO_KHZ800);  //NEO_RGBW
                         #endif LED6812  
                         #ifdef LED2812
const byte NUM_LEDS  = 96;                                 // How many leds in  strip?
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);  //NEO_RGB NEO_GRB
                         #endif LED2812
                      #endif LEDsInstalled
                         #ifdef LEDs24RingInstalled
const byte NUM_LEDS  = 24;                                 // How many leds in  strip?
const uint32_t white = 0x00FFFFFF;  
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);  //NEO_RGB NEO_GRB
                         #endif LEDs24RingInstalled                     

//--------------------------------------------
// HC-12 Long Range Wireless Communication Module
//--------------------------------------------
                         #ifdef HC12MOD
SoftwareSerial HC12(HC_12RX, HC_12TX); 
bool HC12transmit = true;                                   // If false HC12 does not transmits
                         #endif HC12MOD 
//--------------------------------------------
// BLUETOOTH
//--------------------------------------------                                     
                           #ifdef BLUETOOTHMOD
SoftwareSerial Bluetooth(BT_RX, BT_TX);                     // BT_RX <=> TXD on BT module, BT_TX <=> RXD on BT module
bool    BluetoothConnected = false;
String  BluetoothString;
                           #endif BLUETOOTHMOD 
//--------------------------------------------
// CLOCK
//--------------------------------------------                                 
#define MAXTEXT 110                                         // The maximum length of sptext in de sprintf function
static  uint32_t msTick;                                    // The number of millisecond ticks since we last incremented the second counter
int       Isecond, Iminute, Ihour, Iday, Imonth, Iyear; 
int       Dsecond, Dminute, Dhour, Dday, Dmonth, Dyear, Dwday; 
byte      lastminute, lasthour, lastday, lastmonth, lastyear, sayhour = 0;
byte      SecPulse  = 0;                                    // Give a pulse to the Isecond led

const byte MenuItems = 18;                                  // No of entries in menu
const char menu[][50] PROGMEM = {
 "*** DCF HC12 Bluetooth Transmitter ***",
 "Enter time as: Thhmmss (T132145)",
 "A   DCF debug info On/Off",
 "B   Short DCF debug info On/Off",
 "C   Clear statistics",  
// "D   D15122017 for date 15 December 2017",
 "E   Reset MAX7219",
 "F   Toggle full or interval DCFtiny readings", 
 "G   (G20) DCF loop measures every (0-999) msec",
 "H   (H50) mesurements in DCF loop (0-999)",
 "I   For this info",
 "K   Toggle HC12 transmission ON/OFF",
 "Lnn (L5) Min light intensity ( 1-255)",
 "Mnn (M90)Max light intensity (1%-250%)",
 "N   (N2208)Turn On/OFF LEDs between Nhhhh",
 "O   Turn Display ON/OFF",
 "R   Reset",
 "S   Print statistics",
// "T   T091512 is time 09:15:12",
  "    Ed Nieuwenhuys sep-2020" };

//--------------------------------------------
// LDR PHOTOCELL
//--------------------------------------------
const byte MAXBRIGHTNESS    = 15;
const byte LOWBRIGHTNESS    = 4;
byte  LightReducer          =  MAXBRIGHTNESS;              // Factor (%) to dim LED intensity with. Between 1% and 250%
byte  LowerBrightness       =  LOWBRIGHTNESS;              // Lower limit of Brightness ( 0 - 255)
byte  BrightnessCalcFromLDR = 64;                          // Corrected brighness (0-255) 
int   MinPhotocell          = 999;                         // stores minimum reading of photocell;
int   MaxPhotocell          = 1;                           // stores maximum reading of photocell;
int   LDRvalue;

//--------------------------------------------
// DS3231 CLOCK MODULE
//--------------------------------------------
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
        #ifdef MOD_DS3231
RTC_DS3231 RTCklok;                                        // RTC_DS1307 RTC; 
        #else if
RTC_Millis RTCklok;                                        // if no RTC clock connected run on processor time
        #endif
DateTime Inow;
                    #ifdef DCFMOD 
//--------------------------------------------
// DCF-2 DCF77 MODULE
//--------------------------------------------
//                    #ifdef ARDUINO_AVR_NANO_EVERY || defined NANO_EVERY_PINOUT
#define DCF_INTERRUPT 2                                   // DCF Interrupt number associated with DCF_PIN ( 2 Nano Every)
//                    #else if
//#define DCF_INTERRUPT 0                                 // Nano Uno etc 
//                    #endif      
DCF77 DCF = DCF77(DCF_PIN,DCF_INTERRUPT,LOW); // HIGH (HKW) or LOW (Reichelt). Some DCF modules invert the signal
                    #endif DCFMOD 
enum LinearLedstrip  {                                    // Linear LED strip
         Minutemark = 84, MinPar, HourPar, YearPar, 
         DCFgoodminutes, DCF_tijd, RTC_tijd,
         Reserveantenna, Onehourchange, Summertime, 
         Wintertime, TimeDoubfull };       
byte     DCF_signal         = 40;                         // is a proper time received? (0 - 100)
byte     CSummertime        = 30;                         // If >50 times CSummertime is received Summertime = true
byte     CWintertime        = 30;                         // If >50 times CWintertime is received Wintertime = true
byte     COnehourchange     = 10;
//byte     SignalBin[10];

//--------------------------------------------
// DCF77 DCFtiny MODULE
//--------------------------------------------
                    #ifdef DCFTINY
const byte MSECINT          =  5;                         // if DCF_ReadFulltime = false this is time in msec between readings
const byte DCFREADS         = 25;                         // and this are the number of reading in a loop
uint32_t DCFmsTick          = 0;                          // the number of millisecond ticks since we last incremented the second counter
byte     EdMin              = 0;                          // Counter of valid times per minute for DCFtiny
byte     ThMin              = 0;                          // Counter of valid times per minute for DCF77
bool     DCFEd              = false;                      // Flag to detect if DCFtiny library received a good time
bool     DCFTh              = false;                      // Flag to detect if DCF77 library received a good time
bool     DCFlocked          = false;                      // Time received from DCF77
bool     DCF_ReadFulltime   = true;                       // Read DCF constantly or in intervals
byte     StartOfEncodedTime =  0;                         // Should be always one. Start if time signal
int      TotalEff;                                        // Efficiency of DCF-signal (0-100%)
int      mSecInterval       = MSECINT;                    // If DCF_ReadFulltime = false this is time in msec between readings
int      DCFreadsInLoop     = DCFREADS;                   // and this are the number of reading in a loop
int      Interruptmsec[2];
uint32_t SumSecondSignal    =  0;                         // Sum of digital signals ( 0 or 1) in 1 second
uint32_t SumSignalCounts    =  0;                         // Noof of counted signals
uint32_t SignalFaults       =  0;                         // Counter for SignalFaults per hour  
uint32_t Ed                 =  0;                         // Counter of total valid times since start for DCFtiny 
uint32_t Thijs              =  0;                         // Counter of totalvalid times since start for DCF77
uint32_t EdTh               =  0;                         // Valid times per minute counters for DCFtiny and DCF77
uint32_t ValidTimes         =  0;                         // Counter of total recorded valid times
uint32_t MinutesSinceStart  =  0;                         // Counter of recorded valid and not valid times
                    #endif DCFTINY
                    #ifdef LCDMOD
//--------------------------------------------
// LCD Module
//--------------------------------------------
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);                // 0x27 is the I2C bus address for an unmodified backpack
                    #endif 
                              #ifdef TM1637tijd
//--------------------------------------------
// TM1637 display
//--------------------------------------------
TM1637Display Tijd_display = TM1637Display(CLK_TM1637, DIO_TM1637);
                              #endif TM1637tijd

                              #ifdef HT16K33tijd
//--------------------------------------------
// HT16K33 IC2 display
//--------------------------------------------
Adafruit_7segment Tijd_displayIC2 = Adafruit_7segment();
                     #endif HT16K33tijd

//--------------------------------------------
// MAX7219 display
//--------------------------------------------
                     
                     #ifdef MAX7219info
//const byte NUMBEROF7219DISPLAYS = 1;                       // 1 or 2. Second chained with first one
// pin 10 is connected to the MAX7219CLK 
// pin 11 is connected to MAX7219CS 
// pin 12 is connected to the MAX7219DataIn 
 LedControl lc= LedControl(MAX7219DataIn,MAX7219CLK,MAX7219CS,NUMBEROF7219DISPLAYS);
                      #endif MAX7219info 
//----------------------------------------
// Common
//----------------------------------------
bool   LastsNotSet            = false;                      // Used to set lasthour, lastday, etc
byte   PrintDebugInfo         = false;                      // For showing debug info for DCFTINY
byte   PrintDebugInfoShort    = false;                      // For showing debug info for DCFTINY
char   sptext[MAXTEXT+2];                                   // For common print use    
struct RAMstorage {                                         // 246 bytes long
  byte HourEff[3][24]= {0};
} RMem;
struct EEPROMstorage {                                      // 174 bytes long
  byte LightReducer;
  byte LowerBrightness;
  byte TurnOffLEDsAtHH;
  byte TurnOnLEDsAtHH;
  byte DayEff[3][31];
  byte MonthEff[3][12];
  byte YearEff[3][10];
  int  mSecInterval;
  int  DCFreadsInLoop;
  uint32_t Ed;                                              // Counter of total valid times since start for DCFtiny 
  uint32_t Thijs;                                           // Counter of totalvalid times since start for DCF77
  uint32_t EdTh;                                            // Valid times per minute counters for DCFtiny and DCF77
  uint32_t ValidTimes;                                      // Counter of total recorded valid times
  uint32_t MinutesSinceStart;                               // Counter of recorded valid and not valid times
} Mem;
 
// --------------------------------------------------------------------------
// End Definitions                                                    
// --------------------------------------------------------------------------
//--------------------------------------------
// ARDUINO Loop
//--------------------------------------------
void loop(void)
{ 
 EverySecondCheck();
                              #if defined DCFMOD || defined DCFTINY        
 DCF77Check();
                              #endif DCFMOD || defined DCFTINY
}  
//--------------------------------------------
// ARDUINO Setup
//--------------------------------------------
void setup()
{                                        
 Serial.begin(9600);                                                          // Setup the serial port to 9600 baud                                                        // Start the RTC-module  
 while (!Serial);                                                             // wait for serial port to connect. Needed for native USB port only
 Tekstprintln("\n*********\nSerial started"); 
                              #ifdef ARDUINO_AVR_NANO_EVERY
 Tekstprintln("Compiled for ARDUINO_AVR_NANO_EVERY");
                              #endif
                              #if defined(ARDUINO_SAMD_MKRWIFI1010)
 Tekstprintln("Compiled for ARDUINO_SAMD_MKRWIFI1010");                             
                              #endif 
 Wire.begin();                                                                // Start communication with I2C / TWI devices                            
 pinMode(DCF_LED_Pin, OUTPUT);                                                // For showing DCF-pulse or other purposes
 pinMode(DCF_PIN,     INPUT_PULLUP);
 pinMode(Interrupt0,  INPUT_PULLUP);
 pinMode(DCFgood,     OUTPUT );  
 pinMode(secondsPin,  OUTPUT );
                              #ifdef MOD_DS3231
 RTCklok.begin();                                                             // start the RTC-module
                              # else if 
 RTCklok.begin(DateTime(F(__DATE__), F(__TIME__)));                           // if no RTC module is installed use the ATMEGAchip clock
                              #endif MOD_DS3231  
                              #ifdef DCFMOD
 DCF.Start();                                                                 // Start the DCF-module
 Tekstprintln("DCF77 enabled");
                              #endif DCFMOD
                              #ifdef LCDMOD
 lcd.begin (16,2); // for 16 x 2 LCD module                                   // Activate LCD module
 lcd.setBacklightPin(3,POSITIVE);
 lcd.setBacklight(HIGH);
 Tekstprintln("LCD enabled");
                              #endif LCDMOD
                              #ifdef TM1637tijd                               // 4-digit time display installed
 Tijd_display.setBrightness(3);                                               // Set the display brightness (0-7):
 Tijd_display.clear();
 Tekstprintln("4-digit time TM1637 display installed"); 
                              #endif TM1637tijd  
                              #ifdef HT16K33tijd
 Tijd_displayIC2.begin(0x70);
 Tijd_displayIC2.setBrightness(3);                                            // Set the display brightness (0-7):
 Tekstprintln("4-digit time HT16K33 display installed"); 
                              #endif HT16K33tijd
                              #ifdef LEDsInstalled
                                  // --- LED
 strip.begin();
 strip.setBrightness(LightReducer); //BRIGHTNESS);
 LedsOff();                                                                   // Initialize all pixels to 'off' 
 ColorLeds("", 60, 84, purple);                                               // fill the middle ring with purple dots
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
 LedsOff();                                                                   // Initialize all pixels to 'off' 
 Tekstprintln("24 LED ring enabled"); 
                              #endif LEDs24RingInstalled
                              #ifdef BLUETOOTHMOD 
                              #if defined(ARDUINO_SAMD_MKRWIFI1010)           // || defined (ARDUINO_AVR_NANO_EVERY)  
 Serial1.begin(9600); 
 Tekstprintln("Bluetooth MKR enabled"); // Bluetooth connected to Serial1
                              #else if
 Bluetooth.begin(9600); 
 Tekstprintln("Bluetooth enabled"); 
                              #endif ARDUINO_SAMD_MKRWIFI1010
                              #endif BLUETOOTHMOD
                              #ifdef HC12MOD
 HC12.begin(9600);               // Serial port to HC12
 Tekstprintln("HC-12 time sender enabled");
                             #endif HC12MOD 
                             #ifdef MAX7219info                     
 InitialyseMAX7219();
  sprintf(sptext, "123 567887654321");  PrintStringToSegDisplay(sptext);         
 Tekstprintln("MAX7219info enabled");     
 lc.clearDisplay(0);
 lc.clearDisplay(1); 
                              #endif MAX7219info    
 DateTime now        = RTCklok.now();
 DateTime compiled   = DateTime(__DATE__, __TIME__);
 if (now.unixtime() < compiled.unixtime()) 
  {
   Serial.println(F("RTC is older than compile time! Updating"));             // Following line sets the RTC to the date & time this sketch was compiled
   RTCklok.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }
 EEPROM.get(0,Mem);                                                          // Get the data from EEPROM
                                                                             // Store the EEPROM values in work variables
 Ed                = Mem.Ed;                                                 // Counter of total valid times since start for DCFtiny 
 Thijs             = Mem.Thijs;                                              // Counter of totalvalid times since start for DCF77
 EdTh              = Mem.EdTh;                                               // Valid times per minute counters for DCFtiny and DCF77
 ValidTimes        = Mem.ValidTimes;                                         // Counter of total recorded valid times
 MinutesSinceStart = Mem.MinutesSinceStart;                                  // Counter of recorded valid and not valid times
 SWversion();                                                                // Display the version number of the software
 GetTijd(0);                                                                 // Get the time and store it in the proper variables (Ihour,Iday etc.)
 setTime(Ihour, Iminute, Isecond,Iyear, Imonth, Iday);
 lasthour  = Ihour;
 lastday   = Iday;
 lastmonth = Imonth;
 lastyear  = Iyear;
 //attachInterrupt(digitalPinToInterrupt(Interrupt0), FuncInterrupt0, CHANGE);
 DCFmsTick = millis();                                                        // Start of DCF 1 second loop
 msTick = millis();                                                           // Used in KY-040 rotary for debouncing and seconds check

 } 
//--------------------------------------------
// CLOCK Version info
//--------------------------------------------
void SWversion(void) 
{ 
 for (int n=0; n<52; n++) Serial.print("_");                                                Serial.println();
 for (int i = 0; i <MenuItems; i++)   {strcpy_P(sptext, menu[i]);                           Tekstprintln(sptext);  }
 for (int n=0; n<52; n++) {Serial.print(F("_"));}                                           Serial.println();
 sprintf(sptext,"  Brightness Min: %3d bits Max: %3d%%",LowerBrightness, LightReducer);     Tekstprintln(sptext);
 sprintf(sptext,"    LDR read Min: %3d bits Max: %3d bits",MinPhotocell, MaxPhotocell);     Tekstprintln(sptext);
 sprintf(sptext,"  DCF loop every: %3d msec  for: %3d times",mSecInterval, DCFreadsInLoop); Tekstprintln(sptext);
 sprintf(sptext,"LEDs off between: %0.2d - %0.2d",Mem.TurnOffLEDsAtHH, Mem.TurnOnLEDsAtHH); Tekstprintln(sptext);
 sprintf(sptext,"Software version: %s Station: %s",VERSION,STATION);                        Tekstprintln(sptext); 
 GetTijd(1);  
 for (int n=0; n<52; n++) Serial.print("_");                                                Serial.println();
}

//--------------------------------------------
// ARDUINO Reset to default settings
//--------------------------------------------
void FuncInterrupt0()
{ 
  static uint32_t LastIntmsec = 0;
  int diff = millis()-LastIntmsec;
  if(diff>20)
  {
    if (diff<500)
       { 
         Interruptmsec[0] = Interruptmsec[1]=0;
         Interruptmsec[0] = diff;
       }
    else Interruptmsec[1] = diff;
//    { sprintf(sptext,"%3d msec %3d msec %3d msec",Interruptmsec[0],Interruptmsec[1],Interruptmsec[0]+Interruptmsec[1]); Tekstprintln(sptext);}
    LastIntmsec = millis();
  }  
}

//--------------------------------------------
// ARDUINO Reset function
//--------------------------------------------
void SoftwareReset( uint8_t prescaler) 
{
 wdt_enable( prescaler);    // start watchdog with the provided prescaler
 while(1) {}
}
//--------------------------------------------
// ARDUINO Reset to default settings
//--------------------------------------------
void Reset(void)
{
 SoftwareReset( WDTO_60MS); //WDTO_15MS, WDTO_30MS, WDTO_60MS, WDTO_120MS, WDTO_250MS, WDTO_500MS, WDTO_1S, WDTO_2S, WDTO_4S, and WDTO_8S
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
                          #if defined(ARDUINO_SAMD_MKRWIFI1010)  //|| defined (ARDUINO_AVR_NANO_EVERY)
 Serial1.println(tekst);
                          #else if
 Bluetooth.println(tekst);  
                          #endif defined(ARDUINO_SAMD_MKRWIFI1010)   
                          #endif BLUETOOTHMOD
}
//--------------------------------------------
// CLOCK Update routine done every second
//--------------------------------------------
void EverySecondCheck(void)
{
 if( millis() - msTick == 50 ) analogWrite(secondsPin,LOW);                   // Turn OFF the second on pin secondsPin
 if( millis() - msTick > 999)                                                 // Flash the onboard Pin 13 Led so we know something is happening
   {    
    msTick = millis();                                                        // second++; 
    analogWrite(secondsPin,BrightnessCalcFromLDR);                            // Turn ON the second on pin 13
    SerialCheck();
    SecPulse++;                                                               // second routine in function DimLeds
    GetTijd(0);                                                               // synchronize time with RTC clock    
                           #if defined LEDsInstalled || defined LEDs24RingInstalled               
   if ( LEDsAreOff) LedsOff();                                                // If LEDs are off no need to measure LDR
   else DimLeds(false);                                                       // every second an intensitiy check and update from LDR reading  
                           #endif LEDsInstalled || defined LEDs24RingInstalled
                           #ifdef LCDMOD
    Print_tijd_LCD();
                           #endif LCDMOD 
                           #ifdef HC12MOD
 if(HC12transmit)                                                             // if HC12 is allowed to transmit
  {
   switch(Isecond)                                                            // When HC12 sends it disturbs DCF-signal. In seconds 1 - 19 are whether info bits
    {
   case  1: sprintf(sptext,"@%s|%0.2d:%0.2d:%0.2d ",STATION,Inow.hour(),Inow.minute(),Inow.second());
            if (DCFlocked)  HC12.print(sptext);                               // Print the time only if DCF time is trusted 
                            Tekstprint(sptext);     
            sprintf(sptext,"%0.2d-%0.2d-%0.4d|",Inow.day(),Inow.month(),Inow.year());
            if (DCFlocked)  HC12.print(sptext);                               // Print the time only if DCF time is trusted
                            Tekstprint(sptext);
            sprintf(sptext,"DCF-eff:%0.2d%% Signal:%0.2d |%s",TotalEff,DCF_signal, DCFlocked ? "DCF" : "RTC");
            if (DCFlocked)  HC12.println(sptext);                             // Print the time only if DCF time is trusted
                            Tekstprintln(sptext); 
                                                               break;  
   case  2: sprintf(sptext,"T%0.2d%0.2d%0.2d ",Ihour,Iminute,Isecond);        // Send the time as Thhmmss
            if (DCFlocked) {HC12.println(sptext); Tekstprintln(sptext); }     // Print the time only if DCF time is trusted
                                                              break;          // In all other cases it is an error
   case  5: if(Iminute == 0)                                                  // Send date once an hour as Dddmmyyyy
              {    
               sprintf(sptext,"D%0.2d%0.2d%0.4d",Iday,Imonth,Iyear);
               if (DCFlocked) { HC12.println(sptext); Tekstprintln(sptext);}
              }                                                break; 
     }
   }
                           #endif HC12MOD 
                           #ifdef BLUETOOTHMOD   
   BluetoothCheck();
                           #endif BLUETOOTHMOD   
                           #ifdef LEDsInstalled || defined LEDs24RingInstalled
   ShowLeds();
                           #endif LEDsInstalled
  }
 if(Iminute != lastminute) EveryMinuteUpdate();                               // Every minute routine
}
 
//--------------------------------------------
// CLOCK Update routine done every minute
//--------------------------------------------
void EveryMinuteUpdate(void)
{ 
 lastminute = Iminute;

                           #ifdef TM1637tijd
 Tijd_display.showNumberDecEx(((Inow.hour() * 100) + Inow.minute()), 0b11100000, true);    // Display the current time in 24 hour format with leading zeros enabled and a center colon:
                                              // Tijd_display.showNumberDec(displaytime, true); // Prints displaytime without center colon.  
                           #endif TM1637tijd  
                           #ifdef HT16K33tijd
 sprintf(sptext,"%0.2d:%0.2d",Inow.hour(),Inow.minute());
 for (int i=0; i<5; i++)  Tijd_displayIC2.writeDigitNum(i, sptext[i]- '0');
 Tijd_displayIC2.drawColon(true); 
// Tijd_displayIC2.print( 100 * Inow.hour()+Inow.minute());
// if (Inow.hour() == 0) 
//      {
//        Tijd_displayIC2.writeDigitNum(1, 0);      // Also pad when the 10's minute is 0 and should be padded.
//        if (Inow.minute() < 10) Tijd_displayIC2.writeDigitNum(2, 0);
//      }
// if (Inow.hour() < 10) Tijd_displayIC2.writeDigitNum(0, 0);  // also pad the first hour digit with zero
 Tijd_displayIC2.writeDisplay();
                           #endif HT16K33tijd
   
 if(MinutesSinceStart> 0X0FFFFFF0)                                            // Reset counters before overflow
    { Ed = Thijs = EdTh = ValidTimes = MinutesSinceStart = 0; }
 TotalEff =  (int) constrain( (  (double)ValidTimes / (((double)MinutesSinceStart)/100)  ),1,99);  
 analogWrite(DCFgood, DCF_signal);           // The LED intensity displays the signal quality
                           #ifdef MAX7219info                     
 lc.shutdown(0,false); // lc.clearDisplay(0);                                 // and clear the display 1
 lc.shutdown(1,false); // lc.clearDisplay(1);                                 // and clear the display 2
                           #endif MAX7219info 
 if (LastsNotSet && DCFlocked && MinutesSinceStart > 10)
  {                                                                           // A proper time is received and these variables can be set truely
   lastday     = Iday;
   lastmonth   = Imonth;
   lastyear    = Iyear;
   LastsNotSet = true;
  }
                           #ifdef LEDsInstalled                         
 ColorMiddleRing(lasthour);  
 ColorLed(DCF_tijd,  DCFlocked  ?  blue : white); 
 ColorLed(RTC_tijd,  DCFlocked  ? white : red  );
                           #endif LEDsInstalled  
 if (Ihour != lasthour) EveryHourUpdate();
}

//--------------------------------------------
// CLOCK Update routine done every hour
//--------------------------------------------
void EveryHourUpdate(void)
{
 GetTijd(0);                                                                  // Sometimes one of the LEDs is white because of wrong Ihour?? remove if in middle ring white LEDs are at wrong hour
 RMem.HourEff[0][lasthour] = EdMin;                                            // Enter the noof valid minutes for the last hour for DCFtiny
 RMem.HourEff[1][lasthour] = ThMin;                                            // Enter the noof valid minutes for the last hour for DCF77
 RMem.HourEff[2][lasthour] = constrain(SignalFaults,0,99);                     // Enter the noof SignalFaults last hour, >99 disrupt lay-out of statistics 
 DCFlocked = false;                                                           // It is time to receive a DCF-time code  
 sprintf(sptext,"MemHour DCFtiny:%0.2d DCF77:%0.2d Faults: %d",
   RMem.HourEff[0][lasthour], RMem.HourEff[1][lasthour], RMem.HourEff[2][lasthour]); 
 Tekstprintln(sptext);
 if(Ihour == Mem.TurnOffLEDsAtHH) LEDsAreOff = true;                          // is it time to turn off the LEDs?
 if(Ihour == Mem.TurnOnLEDsAtHH)  LEDsAreOff = false;                         // or on?
 EdMin = ThMin = SignalFaults = 0;                                  // Reset the minute counters
 lasthour = constrain(Ihour,0,23);
 if (Iday != lastday) EveryDayUpdate();
}
//--------------------------------------------
// CLOCK Update routine done every day
//--------------------------------------------
void EveryDayUpdate(void)
{
 int i,n;
 int numerator, denominator;
 GetTijd(0);
 for(i=0;i<3;i++)                                                             // i = 0,1,2 -> DCFtiny, DCF77,Both
  {                                                                           // Calculate average valid minutes cq Nooffaults/hour over one day 
   numerator = denominator = 0;
   for(n=0;n<24;n++) 
     {
      if(RMem.HourEff[i][n]>0) 
       { 
        numerator += RMem.HourEff[i][n]; 
        denominator++;
       } 
     }
   if(denominator<1) denominator=1;
   Mem.DayEff[i][lastday] = (byte)(numerator / denominator);
  }
 for(i=0;i<3;i++) {for(n=lastday+1;n<32;n++) {Mem.DayEff[i][n] = 0;}}        // Set following days of the month to zero
  sprintf(sptext,"MemDay DCFtiny:%0.2d DCF77:%0.2d SignalFaults: %d",Mem.DayEff[0][lastday], 
         Mem.DayEff[1][lastday], Mem.DayEff[2][lastday]);     
 Tekstprintln(sptext); 
 Mem.Ed                = Ed;                                                 // Counter of total valid times since start for DCFtiny 
 Mem.Thijs             = Thijs;                                              // Counter of totalvalid times since start for DCF77
 Mem.EdTh              = EdTh;                                               // Valid times per minute counters for DCFtiny and DCF77
 Mem.ValidTimes        = ValidTimes;                                         // Counter of total recorded valid times
 Mem.MinutesSinceStart = MinutesSinceStart;                                  // Counter of recorded valid and not valid times
 lastday = constrain(Iday,1,31);
 if (Imonth != lastmonth) EveryMonthUpdate();
 EEPROM.put(0,Mem);                                                          // Store the Mem. data in EEPROM only once a day
}

//--------------------------------------------
// CLOCK Update routine done every month
//--------------------------------------------
 void EveryMonthUpdate(void)
 {
  int i,n;
  int numerator, denominator;
  GetTijd(0);
  for(i=0;i<3;i++)  // i = 0,1,2 -> DCFtiny, DCF77,Both
   {                                                                          // Calculate average valid minutes cq Nooffaults/hour over one month 
    numerator = denominator = 0;
    for(n=0;n<32;n++) 
      {
        if(Mem.DayEff[i][n]>0) 
          { 
           numerator += Mem.DayEff[i][n]; 
           denominator++;
          } 
      }
    if(denominator<1) denominator=1;
    Mem.MonthEff[i][lastmonth] = (byte)(numerator / denominator);
   }
  sprintf(sptext,"MemMonth DCFtiny:%0.2d DCF77:%0.2d SignalFaults: %d", 
      Mem.MonthEff[0][lastmonth],Mem.MonthEff[1][lastmonth], Mem.MonthEff[2][lastmonth]);     
  Tekstprintln(sptext); 
  lastmonth = constrain(Imonth,1,12);
  if (Iyear != lastyear) EveryYearUpdate();
 }
//--------------------------------------------
// CLOCK Update routine done every year
//--------------------------------------------
 void EveryYearUpdate(void)
 {
  int i,n;
  int numerator, denominator;
  GetTijd(0);
  for(i=0;i<3;i++)  // i = 0,1,2 -> DCFtiny, DCF77,Both
    {                                                                         // Calculate average valid minutes cq Nooffaults/hour over one year 
     numerator = denominator = 0;
     for(n=0;n<13;n++) 
       { 
        if(Mem.MonthEff[i][n]>0) 
          { 
           numerator += Mem.MonthEff[i][n]; 
           denominator++;
          } 
        }
     if(denominator<1) denominator=1;
     Mem.YearEff[i][lastyear] = (byte)(numerator / denominator);
    }
  sprintf(sptext,"MemYear DCFtiny:%0.2d DCF77:%0.2d SignalFaults: %d",
      Mem.YearEff[0][lastyear], Mem.YearEff[1][lastyear], Mem.YearEff[2][lastyear]);     
  Tekstprintln(sptext);   
  lastyear = Iyear%10;
  if(lastyear>10) lastyear=0;   // you'll never know
 }

//--------------------------------------------
// CLOCK check for DCF input
//--------------------------------------------
void DCF77Check(void)
{ 
 digitalWrite(DCF_LED_Pin, !digitalRead(DCF_PIN)); 
                              #ifdef DCFMOD
 time_t DCFtime = DCF.getTime();                                              // Check if new DCF77 time is available
 if (DCFtime!=0)
  {
   tmElements_t tm;
   breakTime(DCFtime, tm);                                                    // Store the DCFtime in struct tm
//   setTime(DCFtime);      // stores the time in processor. disrupts DCFtiny millis()?
   sprintf(sptext,"  DCF77 Lib OK --> %0.2d:%0.2d %0.2d-%0.2d-%0.4d ",tm.Hour, tm.Minute, tm.Day, tm.Month, tm.Year+1970); 
   Tekstprintln(sptext); 
   DateTime now = RTCklok.now();                                              // Needed to use unixtime.
   DateTime DCFtijd (1970+tm.Year, tm.Month, tm.Day, tm.Hour, tm.Minute, 0);  // Convert the time in the time_t DCFtime to Datetime so we can use .unixtime 
   uint32_t TimeDiff = (abs(now.unixtime() - DCFtijd.unixtime()));
//   sprintf(sptext,"RTC is %d sec different from DCF77 time! RTC:%0.2d:%0.2d:%0.2d DCF:%0.2d:%0.2d:%0.2d ",
//              TimeDiff,Inow.hour(), Inow.minute(),Inow.second(), tm.Hour,tm.Minute, tm.Second); 
//   Tekstprintln(sptext); 
   if(TimeDiff > 2  && DCFlocked)                                             // > in Seconds. If difference between DCF and RTC time update the RTC
      {
        sprintf(sptext,"RTC differs %ld sec from DCF77. RTC:%0.2d:%0.2d:%0.2d DCF:%0.2d:%0.2d:%0.2d ",
                        TimeDiff,Inow.hour(), Inow.minute(),Inow.second(), tm.Hour,tm.Minute, tm.Second); 
        Tekstprintln(sptext);       
//       RTCklok.adjust(DCFtime); 
      }   
   Thijs++;
   ThMin++;
   DCFTh = true;                                                            // Used to check if DCF77 and DEtiny receive both a good time    
  }
                             #endif DCFMOD    
                             #ifdef DCFTINY 
 if(DCF_ReadFulltime)                                                         // If continios measuring is selected
   {
    SumSecondSignal += (!digitalRead(DCF_PIN));                               // invert 0 to 1 and 1 to 0
    SumSignalCounts++; 
   }
 else                                                                         // if the pulses are measured in time intervals
   {
    static uint32_t IntervalCounter = 100;      
    if((millis() - IntervalCounter) >= mSecInterval)                          // count ever .. msec .. readings for DCF-receiver 
      { 
       IntervalCounter = millis();      
       for (int n=0; n<DCFreadsInLoop; n++) { SumSecondSignal += (1-digitalRead(DCF_PIN)); } 
       SumSignalCounts+=DCFreadsInLoop;                                      // Noof of counted signals  
      }
    }
  if((millis() - DCFmsTick) >995) //= 1000)                                   // Compute every second the received DCF-bit to a time 
   { 
    while (1-digitalRead(DCF_PIN))                                            // Avoid splitting a positive signal 
       {
        SumSignalCounts++;
        SumSecondSignal++;
       }
    DCFmsTick = millis();                                                     // Set one second counter
    SumSignalCounts*=1.10;                                                    // Correction to correct pulses to an average of 100 and 200 msecs
    if(byte OKstatus = UpdateDCFclock())                                      // If after 60 sec a valid time is calculated, sent it to the HC-12 module
      {
      if(OKstatus == 2)                                                       // If time flag was OK and date flag was NOK
        {
        sprintf(sptext,"       TIME OK --> %0.2d:%0.2d  ",Dhour, Dminute); Tekstprintln(sptext);           
    //  RTCklok.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Dhour, Dminute, 0));
         Ed++;                                                                // Count the valid times
         EdMin++;                                                             // Count the valid times
        }
      if(OKstatus == 1)                                                       // If time flag was OK and date flag also OK  (OKstatus == 1)
        {
         sprintf(sptext,"TIME & DATE OK --> %0.2d:%0.2d %0.2d-%0.2d-%0.4d ",Dhour, Dminute, Dday, Dmonth, 2000+Dyear);
         Tekstprintln(sptext);
         DateTime now = RTCklok.now();
         DateTime DCFtijd (Dyear, Dmonth, Dday, Dhour, Dminute, 0);
         uint32_t TimeDiff = (abs(now.unixtime() - DCFtijd.unixtime()));
         if(TimeDiff > 2)//  && DCFlocked)                                    // > in Seconds. If difference between DCF and RTC time update the RTC
           {
            sprintf(sptext,"RTC differs %ld sec from DCFtiny! Updating. RTC:%0.2d:%0.2d:%0.2d DCF:%0.2d:%0.2d:%0.2d ",
                               TimeDiff,Inow.hour(), Inow.minute(),Inow.second(),Dhour,Dminute,0); 
            Tekstprintln(sptext);   
            RTCklok.adjust(DateTime(Dyear, Dmonth, Dday, Dhour, Dminute, 1));
            setTime(Dhour, Dminute, 1, Dyear, Dmonth, Dday);
           }
         DCFlocked = true;                                                    // DCF time stored (for one hour before unlock)
    //      if(DCF_signal<40) DCFlocked = false; 
         Ed++;                                                                // Count the valid times
         EdMin++;                                                             // Count the valid times
         DCFEd = true;                                                        // Used to check if DCF77 and DEtiny receive both a good time
        }         
      }
   } 
                             #endif DCFTINY
} 
                             #ifdef DCFTINY
//--------------------------------------------
// DCFtiny Make the time from the received DCF-bits
//--------------------------------------------
byte UpdateDCFclock(void)
{
 byte            TimeOK     , Receivebit     , Receivebitinfo;                 // Return flag is proper time is computed and received bit value
 byte            BMinutemark, BReserveantenna, BOnehourchange;
 byte            BSummertime, BWintertime    , BLeapsecond;
 bool            TimeSignaldoubtfull = false;                                  // If a bit length signal is not between 5% and 30%
 static byte     Bitpos,  Paritybit;                                           // Postion of the received second signal in the outer ring  
 static byte     MinOK=2, HourOK=2, YearOK=2;                                  // 2 means not set. static because the content must be remembered
 static int      TimeMinutes,     LastTimeMinutes;                             // Used to check if minute and days 
 static int      TimeMinutesDiff, DateDaysDiff;                                // do not differ much from previous recorded day and minute 
 static uint32_t DateDays,        LastDateDays;
               
 if (Bitpos >= 60) 
     {
      Bitpos = 0;
      MinutesSinceStart++;                                                     // Counter for DCF efficiency calculation 
      if (DCFEd && DCFTh) { EdTh++;    DCFlocked = true;}                   // Count the times both DCFalgorithm received a good time. Lock DCFtime
      if (DCFEd || DCFTh) { ValidTimes++; DCF_signal++; }
      else  DCF_signal--;                                            
      DCF_signal = constrain( DCF_signal,1,99);
      DCFEd = DCFTh = false;                                                   // Turn off good DCF-times received
//                           #ifdef LEDsInstalled *************************************************************************************
//      LedsOffOuterring();
//      ColorLeds("", Minutemark,YearPar, orange);                               // Colour Minutemark, MinPar, HourPar and YearPar orange
//                           #endif LEDsInstalled                          
      } 
 int msec  = (int)(1000 * SumSecondSignal / SumSignalCounts);                  // This are roughly the milliseconds times 10 of the signal length
 switch(msec)
  {
   case   0 ...  10: if (SumSignalCounts > 50)                                 // Check if there were enough signals
                       {Bitpos = 59; Receivebitinfo = 2; Receivebit = 0; }     // If enough signals and one second no signal found. This is position zero
                              else { Receivebitinfo = 9; Receivebit = 0; }     // If not it is a bad signal probably a zero bit
                                                                        break; // If signal is less than 0.05 sec long than it is the start of minute signal
   case  11 ...  50: Receivebitinfo = 8; Receivebit = 0;                break; // In all other cases it is an error
   case  51 ... 160: Receivebitinfo = 0; Receivebit = 0;                break; // If signal is 0.1 sec long than it is a 0 
   case 161 ... 320: Receivebitinfo = 1; Receivebit = 1;                break; // If signal is 0.2 sec long than it is a 1 
   default:          Receivebitinfo = 9; Receivebit = 1;                break; // In all other cases it is an error probably a one bit
  } 
 if(Receivebitinfo > 2)  SignalFaults++;                                       // Count the number of faults per hour
 TimeOK = 0;                                                                   // Set Time return code to false
 switch (Bitpos)                                                               // Decode the 59 bits to a time and date.
  {                                                                            // It is not binary but "Binary-coded decimal". Blocks are checked with an even parity bit. 
    case 0: BMinutemark     = Receivebit;                                      // Blocks are checked with an even parity bit. 
            TimeMinutes     = (Dhour*60 + Dminute);
            DateDays        = (uint32_t)(((Dyear*12 + Dmonth)*31) + Dday);
            TimeMinutesDiff = abs(TimeMinutes - LastTimeMinutes);
            DateDaysDiff    = (int) abs(DateDays - LastDateDays);
            if((MinOK && HourOK && YearOK) && 
              (TimeMinutesDiff < 2) && (DateDaysDiff < 2 ) )      TimeOK = 1;  // All OK  
            if(MinOK && HourOK && !YearOK && (TimeMinutesDiff<2)) TimeOK = 2;  // If a date difference time remains usable
            if(TimeSignaldoubtfull)                               TimeOK = 5;  // If a signal lenght was too short or long this last minute time is not OK
            if(Dmonth==0 || Dmonth>12 || Dhour>23 || Dminute>59 ) TimeOK = 0;  // If time is definitely wrong return Time is not OK
                             #ifdef LEDsInstalled
            LedsOffOuterring();
            ColorLeds("", Minutemark,YearPar, orange);                         // Colour Minutemark, MinPar, HourPar and YearPar orange 
            ColorLed(Minutemark,Receivebit ? red: green);
                             #endif LEDsInstalled
                                                                        break; // Bit must always be 0
   case   1: TimeSignaldoubtfull = 0;                                          // Start a fresh minute
             LastTimeMinutes = TimeMinutes; 
             LastDateDays    = DateDays;
             MinOK = HourOK = YearOK = 2;
             Dminute = Dhour = Dday = Dwday = Dmonth = Dyear = 0;  
   case   2 ... 14:                                                     break; // Reserved for wheather info we will not use
   case  15: BReserveantenna = Receivebit;
                             #ifdef LEDsInstalled
             ColorLed(Reserveantenna, (Receivebit ? red : white));
                             #endif LEDsInstalled
                                                                        break; // 1 if reserve antenna is in use
   case  16: BOnehourchange  = Receivebit;
             if(Receivebit) COnehourchange++;
             else           COnehourchange--;
             COnehourchange = constrain(COnehourchange,5,15);
                             #ifdef LEDsInstalled  
             ColorLed(Onehourchange, (COnehourchange>10 ? yellow : white));
                             #endif LEDsInstalled
             if(COnehourchange>10) DCFlocked = false;                          // Unlock if one hour change comes up 
                                                                        break; // Has value of 1 one hour before change summer/winter time
   case  17: BSummertime = Receivebit;
             if(Receivebit) CSummertime++; 
             else           CSummertime--;
             CSummertime = constrain(CSummertime,5,60);                
                             #ifdef LEDsInstalled
             ColorLed(Summertime, (CSummertime>50 ? purple : white));
                             #endif LEDsInstalled
                                          break;                               // 1 if summer time 
   case  18: BWintertime = Receivebit; 
             if(Receivebit) CWintertime++;
             else           CWintertime--;
             CWintertime = constrain(CWintertime,5,60);
                             #ifdef LEDsInstalled
             ColorLed(Wintertime, (CWintertime>50 ? blue : white ));
                             #endif LEDsInstalled
                                                                        break; // 1 if winter time
   case  19: BLeapsecond = Receivebit;                                  break; // Announcement of leap second in time setting is coming up
   case  20: StartOfEncodedTime = Receivebit; Paritybit = 0;            break; // Bit must always be 1 
   case  21: if(Receivebit) {Dminute  = 1;  Paritybit = 1 - Paritybit;} break;
   case  22: if(Receivebit) {Dminute += 2 ; Paritybit = 1 - Paritybit;} break;
   case  23: if(Receivebit) {Dminute += 4 ; Paritybit = 1 - Paritybit;} break;
   case  24: if(Receivebit) {Dminute += 8 ; Paritybit = 1 - Paritybit;} break;
   case  25: if(Receivebit) {Dminute += 10; Paritybit = 1 - Paritybit;} break;
   case  26: if(Receivebit) {Dminute += 20; Paritybit = 1 - Paritybit;} break;
   case  27: if(Receivebit) {Dminute += 40; Paritybit = 1 - Paritybit;} break;
   case  28: MinOK = (Receivebit==Paritybit);                                  // The minute parity is OK or NOK
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
   case  35: HourOK = (Receivebit==Paritybit);                                 // The hour parity is OK or NOK
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
   case  58: YearOK = (Receivebit==Paritybit);                                 // The year month day parity is OK or NOK
                     #ifdef LEDsInstalled
              ColorLed(YearPar, YearPar ? green : red); 
                     #endif LEDsInstalled   
              Paritybit = 0;                                            break;  
   case  59: //Serial.println("silence"); 
                                                                        break;
    default:  Serial.println(F("Default in BitPos loop. Impossible"));  break;
   }

  if((Bitpos>19 && Bitpos<36) && Receivebitinfo>2) TimeSignaldoubtfull = true; // If a date time bit is received and the received bit is not 0 or 1 then time becomes doubtfull
  if(PrintDebugInfo)
    {
     sprintf(sptext,"@%s%s%s", (MinOK<2?(MinOK?"M":"m"):"."),(HourOK<2?(HourOK?"H":"h"):"."),(YearOK<2?(YearOK?"Y":"y"):"."));  Tekstprint(sptext);
     if (PrintDebugInfoShort)
        {
         sprintf(sptext,"%5ld %5ld %2ld%%(%d)%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.2d",
                     SumSecondSignal, SumSignalCounts, 100*SumSecondSignal/SumSignalCounts,  
                     Receivebitinfo, Dhour, Dminute,Bitpos, Dday, Dmonth, Dyear); 
         Tekstprintln(sptext);
        }
      else
        {
         sprintf(sptext,"%5ld %5ld %2ld%% (%d) %0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d/%0.1d F%d",
                     SumSecondSignal, SumSignalCounts, 100*SumSecondSignal/SumSignalCounts,  
                     Receivebitinfo, Dhour, Dminute, Bitpos, Dday, Dmonth, 2000+Dyear, Dwday, SignalFaults); 
         Tekstprint(sptext); 
         sprintf(sptext," Ed:%ld Th:%ld Min:%ld OK:%d L:%s",Ed, Thijs, MinutesSinceStart,TimeOK,DCFlocked?"Y":"N"); 
         Tekstprintln(sptext);         
        }
//     sprintf(sptext," %0.3d:%0.3d :%+0.2d",msec, Interruptmsec[0],msec-Interruptmsec[0]); Tekstprintln(sptext);
    }
                            #ifdef MAX7219info
 if(NUMBEROF7219DISPLAYS==1)
 {
  switch (Bitpos)                                                                // Decode the 59 bits to a time and date.
   {                                                                             // It is not binary but "Binary-coded decimal". Blocks are checked with an even parity bit. 
    case 0 ... 19: sprintf(sptext,"%0.3d_%0.2d%0.2d%0.2d_%0.2d_%0.2d",(int)
                   (1000*SumSecondSignal/SumSignalCounts),Iday,Imonth,Bitpos,TotalEff,SignalFaults);    break;
          default: sprintf(sptext,"%0.3d %0.2d%0.2d",(int)(1000*SumSecondSignal/SumSignalCounts),Dhour, Dminute);
   }
 }
 else 
  { 
   switch (Bitpos)                                                                // Print reading values in the displays.
    {
     case 0 ...  5: sprintf(sptext,"%0.8ld%0.8ld",ValidTimes, MinutesSinceStart);                        break;
     case        6: lc.clearDisplay(0);  lc.clearDisplay(1);
     case 7 ... 19: sprintf(sptext,"%0.3d_%0.2d%0.2d%0.2d_%0.2d_%0.2d",(int)
                    (1000*SumSecondSignal/SumSignalCounts),Iday,Imonth,Bitpos,TotalEff,SignalFaults);     break;
     case       20: lc.clearDisplay(1);
           default: sprintf(sptext,"%0.3d_%0.2d%0.2d%0.2d__%0.4d",(int)
                    (1000*SumSecondSignal/SumSignalCounts),Dhour,Dminute,Bitpos,LDRvalue); //DCF_signal,SignalFaults);
    }
   }
 PrintStringToSegDisplay(sptext); 
                            #endif MAX7219info       
// sprintf(sptext,"\%d%d%d%d%d%d%d%d%d%d/", SignalBin[0],SignalBin[1],SignalBin[2],SignalBin[3],SignalBin[4], SignalBin[5],SignalBin[6],SignalBin[7],SignalBin[8],SignalBin[9]);  Serial.println(sptext);

                            #ifdef LEDs24RingInstalled
 uint32_t BitColour = red;  
 if (Bitpos ==  0) { BitColour = yellow; LedsOff(); } 
 if (Bitpos == 20) { BitColour = blue;   LedsOff(); }
 if (Bitpos == 36) { BitColour = green;  LedsOff(); }
 ColorLeds("", Bitpos % 24, Bitpos % 24 , Receivebit<2 ? (Receivebit == 0 ? BitColour : red ) : (Receivebit == 9 ? white : green) );
 ShowLeds();
                           #endif LEDs24RingInstalled 
                           #ifdef LEDsInstalled
 ColorLedOuterRing(Bitpos, Receivebitinfo<2 ? (Receivebitinfo == 0 ? blue : red ) : yellow) ; 
 ColorLed(DCFgoodminutes, SignalFaults<61 ? (SignalFaults<31 ? (SignalFaults <16 ? (SignalFaults <3 ? green : purple) : yellow ) : orange) : red);
 ColorLed(TimeDoubfull,  TimeSignaldoubtfull ? red : green);
                           #endif LEDsInstalled
 SumSecondSignal = SumSignalCounts = 0;
// for (int x=0; x<10; x++) SignalBin[x]=0;
 Dsecond = Bitpos;
 Bitpos++; 
return(TimeOK);
}  
//--------------------------------------------
// DCFtiny Print statistics
//--------------------------------------------
void PrintStats(void)
{
                              #if defined DCFMOD || defined DCFTINY 
 sprintf(sptext,"DCFEd:%ld\nDCF77:%ld\nBoth:%ld\nValid times:%ld\nTotal minutes:%ld\n",Ed, Thijs, EdTh, ValidTimes, MinutesSinceStart);
 Tekstprint(sptext);
 sprintf(sptext,"DCF Eff:%0.2d%% Signal:%0.2d %s",TotalEff,DCF_signal, DCFlocked ? "DCF" : "RTC");
 Tekstprintln(sptext); 
 int i,n;
// hour
 sprintf(sptext,"0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 "); Tekstprintln(sptext);   
 for(i=0;i<3;i++)  // i = 0,1,2 -> DCFtiny, DCF77,Both
   {                                         
    for(n=0;n<24;n++) { sprintf(sptext,"%0.2d ",RMem.HourEff[i][n]); Tekstprint(sptext);}
        Tekstprintln(""); 
    }

//day
 sprintf(sptext,"1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31"); Tekstprintln(sptext);   
 for(i=0;i<3;i++)  // i = 0,1,2 -> DCFtiny, DCF77,Both
   {                                         
    for(n=1;n<32;n++) { sprintf(sptext,"%0.2d ",Mem.DayEff[i][n]); Tekstprint(sptext);}
    Tekstprintln(""); 
    }
      
//month
 sprintf(sptext,"1  2  3  4  5  6  7  8  9  10 11 12"); Tekstprintln(sptext);   
 for(i=0;i<3;i++)  // i = 0,1,2 -> DCFtiny, DCF77,Both
   {                                         
    for(n=1;n<13;n++) { sprintf(sptext,"%0.2d ",Mem.MonthEff[i][n]); Tekstprint(sptext);}
    Tekstprintln(""); 
    }
 
//year
  sprintf(sptext," 0  1  2  3  4  5  6  7  8  9    Last 10 years (year modulo 10)"); Tekstprintln(sptext);   
 for(i=0;i<3;i++)  // i = 0,1,2 -> DCFtiny, DCF77,Both
   {                                         
    for(n=0;n<10;n++) { sprintf(sptext,"%0.2d ",Mem.YearEff[i][n]); Tekstprint(sptext);}
    Tekstprintln(""); 
    }    
 SumSignalCounts=9999;                                              // Avoid resetting to 0 seconds. Printing takes a long time
 SumSecondSignal=6666;                                              // Printing misses many counts in the DCFtiny receiver
}
                           #endif DCFMOD || DCFTINY                
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
   if (c>31 && c<128) SerialString += c;                           // allow input from Space - Del
  }
 if (SerialString.length()>0) 
         ReworkInputString(SerialString);                          // Rework ReworkInputString();
 SerialString = ""; 
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
 while (HC12.available()>0)                                        // If HC-12 has data
   {       
    c = HC12.read();     
    if (c>31 && c<128)  HC12String += c;                           // Allow only input from Space - Del

    delay(3);
   }
 HC12String += "\n";
 DataInBuffer = false;
 if (HC12String.length()>0) 
   {
    Serial.print("Received HC-12: "); Serial.println(HC12String);
    ReworkInputString(HC12String);
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
                           #if defined(ARDUINO_SAMD_MKRWIFI1010)   //|| defined (ARDUINO_AVR_NANO_EVERY)  
 while (Serial1.available()) 
  {
//   delay(3); 
   char c = Serial1.read();
   Serial.print(c);
   if (c>31 && c<128) BluetoothString += c;
   else c = 0;     // delete a CR
  }
                           #else if
 Bluetooth.listen();  //  When using two software serial ports, you have to switch ports by listen()ing on each one in turn.
 while (Bluetooth.available()) 
  {
//   delay(3); 
   char c = Bluetooth.read();
   Serial.print(c);
   if (c>31 && c<128) BluetoothString += c;
   else c = 0;     // delete a CR
  }
                           #endif   
 if (BluetoothString.length()>0)  
    ReworkInputString(BluetoothString);                            // Rework ReworkInputString();
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
 sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d",Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),Inow.month(),Inow.year());
 Tekstprintln(sptext);
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
 Tekstprintln(sptext);
}

//--------------------------------------------
// DS3231 Set time in module and print it
//--------------------------------------------
void SetRTCTime(void)
{ 
 Ihour   = constrain(Ihour  ,0,24);
 Iminute = constrain(Iminute,0,59); 
 Isecond = constrain(Isecond,0,59); 
 RTCklok.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Ihour, Iminute, Isecond));
 GetTijd(0);                      // synchronize time with RTC clock
 Print_tijd();
}
// ------------------- End  Time functions 
                     #if defined LEDsInstalled || defined LEDs24RingInstalled
// --------------------Colour Clock Light functions -----------------------------------
//--------------------------------------------
//  LED Set color for several LEDs
//--------------------------------------------
void ColorLeds(char* Tekst, int FirstLed, int LastLed, uint32_t RGBWColor)
{ 
 strip.fill(RGBWColor, FirstLed, ++LastLed - FirstLed );                     // Serial.println(RGBWColor,HEX); 
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
//  Faults are no of signals that are not 0.1 or 0.2 seconds long 
//--------------------------------------------
void ColorMiddleRing(byte Fhour)
{
 for (int n=0; n<24;n++) 
  {  
   switch(RMem.HourEff[2][n])
   {
     case   0 ...   5:    ColorLed(60 + n,  green ); break;
     case   6 ...  15:    ColorLed(60 + n,  apple ); break; 
     case  16 ...  30:    ColorLed(60 + n,  marine); break;  
     case  31 ...  60:    ColorLed(60 + n,  blue  ); break; 
     case  61 ... 120:    ColorLed(60 + n,  orange); break; 
     case 121 ... 0xFD:   ColorLed(60 + n,  red   ); break; 
     case 0xFE:           ColorLeds("", 60, 95, purple);        // Reset and color the middle ring purple
     default:                                            break; 
    }
  }
 if (Fhour==23) Fhour = 0;
 ColorLed(61 + Fhour, white);                                   // colour the next hour white
}
//--------------------------------------------
//  LED Clear display settings of the LED's
//--------------------------------------------
void LedsOff(void) 
{ 
 strip.fill(0, 0, NUM_LEDS);                    // Turn all LEDs off 
}

void LedsOffOuterring(void) 
{ 
 strip.fill(0X808000 , 0, 60);    // Color the outer seconds info ring LEDs faint yellow
}
void LedsOffMiddlering(void) 
{ 
 strip.fill(0, 60, 24);                        // Turn LEDs of hour quality middle ring off
}
void LedsOffInnerring(void) 
{ 
 strip.fill(0, 84, 12);                       // Turn LEDs of information middle ring off 
}
//--------------------------------------------
//  LED Push data in LED strip to commit the changes
//--------------------------------------------
void ShowLeds(void)
{
 strip.show();                                // Send the signal to the LEDs
}
//--------------------------------------------
//  LED Set brighness of LEDs
//--------------------------------------------  
void SetBrightnessLeds( byte Bright)
{
 strip.setBrightness(Bright);                 // Brighness of the LEDs
 ShowLeds();
}

//--------------------------------------------
//  LED Dim the leds by PWM measured by the LDR and print values
//--------------------------------------------
void DimLeds(byte print) 
{                                                                                          
 if (SecPulse)   // if a second has passed 
 {
  static int Previous_LDR_read = 512;
  int OutPhotocell, BrCalc, Temp;
  int LDR_read = (4 * Previous_LDR_read + analogRead(PhotoCellPin)) / 5;   // Read lightsensor 
  LDRvalue = Previous_LDR_read = LDR_read;
  OutPhotocell = (int) (((float)LightReducer/100) * sqrt( (float) 63.5 * (float) constrain(LDR_read,1,1023))); // Linear --> hyperbolic with sqrt
  MinPhotocell = MinPhotocell > LDR_read ? ((LDR_read+9*MinPhotocell))/10 : MinPhotocell;
  MaxPhotocell = MaxPhotocell < LDR_read ? ((LDR_read+9*MaxPhotocell))/10 : MaxPhotocell;        // sprintf(sptext,"LDR read: %d MinPhotocell %d\n",LDR_read, MinPhotocell);   Tekstprint(sptext);
  BrightnessCalcFromLDR = constrain(OutPhotocell, LowerBrightness , 255);  // filter out of strange results
  BrCalc = (int) (BrightnessCalcFromLDR/2.55);
  if(print)
    {
     sprintf(sptext,"LDR:%d=%d%% ",LDR_read, BrCalc);  //  T=%dC ,Get3231Temp()-2);
     Tekstprint(sptext);                          
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
if(LEDsAreOff) {lc.shutdown(0,false); lc.clearDisplay(0);  lc.shutdown(1,false); lc.clearDisplay(1); return; }
if(Mem.TurnOffLEDsAtHH!=Mem.TurnOnLEDsAtHH)
    if(Ihour >= Mem.TurnOffLEDsAtHH && Ihour<24)
      if(Ihour>=0 && Ihour<Mem.TurnOnLEDsAtHH) 
        { lc.shutdown(0,false); lc.clearDisplay(0);  lc.shutdown(1,false); lc.clearDisplay(1);     return;  }  
 for (int n = 0; n<16;n++)
    {
     int d = text[n] - '0';  
     lc.setDigit(n/8, 7-n%8, d ,false);
     }
}
//--------------------------------------------
//  MAX7219 Initialyse MAX7219 with two units
//--------------------------------------------
void InitialyseMAX7219(void)
{
 lc.shutdown(0,false); 
 lc.setIntensity(0,0);                                               // Set the brightness 0 - 15
 lc.clearDisplay(0);                                                 // and clear the display
 lc.shutdown(1,false);   
 lc.setIntensity(1,0);                                               // Set the brightness 0 - 15
 lc.clearDisplay(1);                                                 // and clear the display
}
                     #endif MAX7219info 

//--------------------------------------------
//  LED In- or decrease light intensity value
//--------------------------------------------
void WriteLightReducer(float amount)
{
 LightReducer = amount; 
 StoreLightReducer(LightReducer);
}

//--------------------------------------------
//  LED Write light intensity to EEPROM
//--------------------------------------------
void StoreLightReducer(byte waarde)
{
 Mem.LightReducer = constrain (waarde,0,250);      // Range between 1 and 250
// In UpdateEveryDay() --> EEPROM.put(0,Mem);     // Store the value (0-250) in permanent EEPROM memory 
 sprintf(sptext,"Max brightness: %3d%%",LightReducer);
 Tekstprintln(sptext);
// Serial.print(millis() - RotaryPressTimer); Serial.print(" msec ------- ");
// Serial.print(F("LightReducer: ")); Serial.print(LightReducer * 100); Serial.println("%");
}

//--------------------------------------------
//  LED Write lowest allowable light intensity to EEPROM
//--------------------------------------------
void StoreLowerBrightness(byte waarde)
{
 Mem.LowerBrightness = constrain (waarde, 0 ,250);     // Range between 1 and 100
// In UpdateEveryDay() --> EEPROM.put(0,Mem);        // Store the value (0-250) in permanent EEPROM memory 
 sprintf(sptext,"Lower brightness: %3ld bits",(long) LowerBrightness);
 Tekstprintln(sptext);
}
//--------------------------------------------
//  LED Write DCF loop values to EEPROM
//--------------------------------------------
void WritemSecInterval(int waarde){ EEPROM.update(0,Mem.mSecInterval);}

//--------------------------------------------
//  LED Write DCF reads in loop values to EEPROM
//--------------------------------------------
void WriteDCFreadsInLoop(int waarde)
{ 
 Mem.DCFreadsInLoop =  constrain (waarde, 0 ,1000); 
// In UpdateEveryDay() --> EEPROM.put(0,Mem);
}

//--------------------------------------------
//  LED Read DCF loop values from EEPROM
//--------------------------------------------
int ReadSecInterval(void)
{ return EEPROM.get(0,Mem.mSecInterval);}
//--------------------------------------------
int ReadDCFreadsInLoop(void)
 {return EEPROM.get(0,Mem.DCFreadsInLoop); }
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
{ return EEPROM.read(pos); }

//--------------------------------------------
//  CLOCK Input from Bluetooth or Serial
//--------------------------------------------
void ReworkInputString(String InputString)
{
 String temp;
 InputString.toCharArray(sptext, MAXTEXT-1);

 if ( InputString[0] > 64)                                           // Does the string start with a letter?
  {
  int val = InputString[0];
  Tekstprintln(sptext);
  switch (val)
   {
    case 'A':
    case 'a':
             PrintDebugInfo = 1 - PrintDebugInfo;
             PrintDebugInfoShort = false;
             break;  
    case 'B':
    case 'b':
             PrintDebugInfo = 1 - PrintDebugInfo;
             PrintDebugInfoShort = true;
             break; 
    case 'C':
    case 'c':
             int i,n;         // i = 0,1,2 -> DCFtiny, DCF77,Both
             if(InputString.length() == 1)
               {
                for(i=0;i<3;i++) {for(n=0;n<24;n++) { RMem.HourEff[i][n] = 0; } }
                for(i=0;i<3;i++) {for(n=1;n<32;n++) { Mem.DayEff[i][n]  = 0; } }
                for(i=0;i<3;i++) {for(n=1;n<13;n++) { Mem.MonthEff[i][n]= 0; } } 
                for(i=0;i<3;i++) {for(n=0;n<10;n++) { Mem.YearEff[i][n] = 0; } }
                Mem.Ed = Mem.Thijs = Mem.EdTh = Mem.ValidTimes = Mem.MinutesSinceStart = 0;
                Ed = Thijs = EdTh = ValidTimes = MinutesSinceStart = 0;             
                Tekstprintln("Statistics data erased. EEPROM data will be cleared at midnight"); 
                PrintStats();
               }
              if(InputString.length() == 3)
               {
                for (i=0 ; i<EEPROM.length(); i++) { EEPROM.write(i, 0); }
                Tekstprintln("EEPROM data were erased"); 
                PrintStats();
               }
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
               sprintf(sptext,"Date changed");   Tekstprintln(sptext);  
               DCFlocked = false;                                          // It is time to receive a DCF-time code   
               DCF_signal = 30; 
               sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d",Inow.hour(),Inow.minute(),Inow.second(),Iday,Imonth,Jaar);
               Tekstprintln(sptext);
              }
              else Tekstprintln("**** Length fault. Enter ddmmyyyy ****");
                                                                   break;
    case 'E':
    case 'e':  
                                  #ifdef MAX7219info 
             if(InputString.length() == 1)
               {
                InitialyseMAX7219();                                            // Reset the displays or start them 
                Tekstprintln("MAX7219 Initialysed"); 
               }
                                   #endif MAX7219info 
                                                                    break;
    case 'F':
    case 'f':   
             DCF_ReadFulltime   = 1 - DCF_ReadFulltime;               // Toggle DCF77 Full to interval readings
             sprintf(sptext,"DCF readings changed to: %s ", DCF_ReadFulltime ? "Constant reading" : "Interval reading" );
             Tekstprintln(sptext);
             break;       
    case 'G':                                                         // Change the DCF loop interval time
    case 'g':    
             if(InputString.length() > 1)
               {
               temp = InputString.substring(1);
               mSecInterval   = temp.toInt();
               mSecInterval   = constrain(mSecInterval  , 0, 1000);
               WritemSecInterval(mSecInterval);
               sprintf(sptext,"Interval changed to: %d msec",mSecInterval);
               DCF_ReadFulltime     = false;
               }
             else Tekstprintln("**** Length fault. Enter Thhmmss ****");
             Tekstprintln(sptext); 
                                                                   break;
    case 'H':                                                         // Change the DCF no of loops in an interval time 
    case 'h':    
             if(InputString.length() > 1)
               {  
                temp = InputString.substring(1);
                DCFreadsInLoop   = temp.toInt();
                DCFreadsInLoop   = constrain(DCFreadsInLoop  , 1, 1000);
                WriteDCFreadsInLoop(DCFreadsInLoop);
                sprintf(sptext,"DCF readings In Loop changed to: %d reads / loop",DCFreadsInLoop);
                DCF_ReadFulltime     = false;
               }
             else Tekstprintln("**** Length fault. Enter Thhmmss ****");
             Tekstprintln(sptext); 
                                                                   break;
    case 'I':
    case 'i':   
             SWversion();
                                                                   break;                                                                  
    case 'K':
    case 'k':
                                #ifdef HC12MOD    
             HC12transmit   = 1 - HC12transmit;                         // Toggle HC12 transmitting
             sprintf(sptext,"HC12transmit changed to: %s ", HC12transmit ? "HC12 transmits" : "HC12 stops transmits" );
             Tekstprintln(sptext);
                                 #endif HC12MOD 
                                                                   break;
    case 'L':                                                         // Lowest value for Brightness
    case 'l':
             if(InputString.length() > 1)
              {    
               temp = InputString.substring(1);
               LowerBrightness   = constrain(temp.toInt()  , 1, 255);
               StoreLowerBrightness(LowerBrightness);
               sprintf(sptext,"Lower brightness changed to: %d bits",LowerBrightness);
              }
              else Tekstprintln("**** Length fault. Enter Thhmmss ****");
              Tekstprintln(sptext); 
                                                                   break;
    case 'M':                                                         // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'm':
             if(InputString.length() > 1)
              {       
               temp = InputString.substring(1);
               byte Intensityfactor = constrain(temp.toInt(), 1, 255);
               WriteLightReducer(Intensityfactor);
               sprintf(sptext,"Max brightness changed to: %d%%",Intensityfactor);
               Tekstprintln(sptext);
              }
              else Tekstprintln("**** Length fault. Enter Thhmmss ****");
              Tekstprintln(sptext);               
                                                                   break; 
    case 'N':
    case 'n':
             if (InputString.length() == 1 )
              {
                Mem.TurnOffLEDsAtHH = 0;
                Mem.TurnOnLEDsAtHH  = 0;
              }
             if (InputString.length() == 5 )
              {
               temp   = InputString.substring(1,3);   Mem.TurnOffLEDsAtHH = (byte) temp.toInt(); 
               temp   = InputString.substring(3,5);   Mem.TurnOnLEDsAtHH = (byte) temp.toInt(); 
              }
             Mem.TurnOffLEDsAtHH = constrain(Mem.TurnOffLEDsAtHH, 0, 23);
             Mem.TurnOnLEDsAtHH  = constrain(Mem.TurnOnLEDsAtHH,  0, 23); 
             sprintf(sptext,"LEDs are OFF between %2d:00 and %2d:00", Mem.TurnOffLEDsAtHH,Mem.TurnOnLEDsAtHH );
             Tekstprintln(sptext); 
                                                                   break;
    case 'O':
    case 'o':
             if(InputString.length() == 1)
               {
                LEDsAreOff = !LEDsAreOff;
                sprintf(sptext,"LEDs are %s", LEDsAreOff?"OFF":"ON" );
                Tekstprintln(sptext); 
               }
                                                                  break;
    case 'R':
    case 'r':
            if(InputString.length() == 1)
              {
               Reset();                                                 // Reset all settings 
               Tekstprintln("**** Resetted to default settings ****"); 
              }
                                                                  break;
    case 'S':
    case 's':
             if(InputString.length() == 1)
               {
                Tekstprintln("----> Statistics -----");
                PrintStats();                                            // Print statistics  
               }
                                                                  break;        
    case 'T':
    case 't':
             if(InputString.length() >= 7)  // T125500
              {
               temp = InputString.substring(1,3);   
               if(temp.toInt() <24) Ihour = temp.toInt();
               else break; 
               temp = InputString.substring(3,5);   
               if(temp.toInt() <60) Iminute = temp.toInt();
               else break;                
               temp = InputString.substring(5,7);   
               if(temp.toInt() <60) Isecond = temp.toInt();
               else break;
               SetRTCTime();
               DCFlocked = false;                                          // It is time to receive a DCF-time code 
               DCF_signal = 30;
               sprintf(sptext,"Time changed");  Tekstprintln(sptext);       
              }
              else Tekstprintln("**** Length fault. Enter Thhmmss ****");
             
                                                                    break;           
    default:
                                                                    break;
    }
  } 
 InputString = "";
 temp = "";
}
