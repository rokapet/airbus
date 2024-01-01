// *****************************************************************************************
// Include the required libraries
// *****************************************************************************************
// XPLDirect plugin for the communication with X-Plane by https://www.patreon.com/curiosityworkshop
#include <arduino.h>
#include <XPLDirect.h>                // include file for the X-plane direct interface
XPLDirect Xinterface(&Serial);        // create an instance of it

// TM1637 library by Rob Tillaart - from TM1637 demo
// #include "TM1637.h"

// CD74HC4067 for 16-channel input/output multiplexers by Patrick Wasp
// https://github.com/waspinator/CD74HC4067
// Included in Arduino IDE by default, but need to install it first
#include <CD74HC4067.h>

// TM1637TinyDisplay for 7-segment displays by Jason Cox
// https://github.com/jasonacox/TM1637TinyDisplay
// Included in Arduino IDE by default, but need to install it first
// #include <TM1637TinyDisplay.h> // 4-digit displays
// #include <TM1637TinyDisplay6.h> // 6-digit displays

// U8G2 for monochrome displays by Oliver Kraus (olikraus)
// https://github.com/olikraus/u8g2
// Included in Arduino IDE by default, but need to install it first
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// Shifty for bit shifters, like the 16-channel DM13A or the 8-channel 74HC595 by Johnathan Bartlett
// https://github.com/johnnyb/Shifty
// Included in Arduino IDE by default, but need to install it first
#include <Shifty.h>

// EncoderTool for rotary encoders by luni64
// https://github.com/luni64/EncoderTool
// Included in Arduino IDE by default, but need to install it first
#include <EncoderTool.h>
using namespace EncoderTool;


// *****************************************************************************************
// Define required variables
// *****************************************************************************************
long int startTime;                   // for error checking - from TM1637 demo
// float compassHeading;                 // name for item - from TM1637 demo

// AUTOLAND warning action function
// unsigned long ALprevMillis = 0;


// *****************************************************************************************
// Define X-Plane datarefs
// *****************************************************************************************
long int beacon;                      // holds status of beacon light - remove

/// Side panel CAPT
// Autoland Led
long int master_warning;              // Master_warning Led
long int master_caution;              // Master_caution Led
long int CaptStickInputOverride;      // Capt Side Stick Priority arrow Led
// Capt Side Stick Priority chars Led

/*
/// Side panel CO
long int CoStickInputOverride;        // FO Side Stick Priority arrow Led
// FO Side Stick Priority chars Led
*/

/// EFIS CAPT
// QNH Led
long int BaroStdCapt;                 // Baro setting on CPT side is "STD"
// QFE Led (not implemented in Toliss)
long int FD1Engage;                   // FD Led
long int ILSonCapt;                   // LS Led
long int NDShowCSTRCapt;              // CSTR Led
long int NDShowWPTCapt;               // WPT Led
long int NDShowVORDCapt;              // VOR.D Led
long int NDShowNDBCapt;               // NDB Led
long int NDShowARPTCapt;              // ARPT Led
long int NDmodeCapt;                  // ND Nav mode
long int NDrangeCapt;                 // ND Nav range

/*
/// EFIS CO
long int BaroStdFO;                   // Baro setting on FO side is "STD"
// QFE Led (not implemented in Toliss)
long int FD2Engage;                   // FD Led
long int ILSonCo;                     // LS Led
long int NDShowCSTRFO;                // CSTR Led
long int NDShowWPTFO;                 // WPT Led
long int NDShowVORDFO;                // VOR.D Led
long int NDShowNDBFO;                 // NDB Led
long int NDShowARPTFO;                // ARPT Led
long int NDmodeFO;                    // ND Nav mode
long int NDrangeFO;                   // ND Nav range
*/

/// FCU
long int airspeed_dial_kts_mach;      // AP target speed on display, knots or mach, depends on "sim/cockpit2/autopilot/airspeed_is_mach" (float RW knots/mach)
long int airspeed_is_mach;            // Speed mode - 0: knots, 1: mach (int RW bool)
long int SPDmanaged;                  // Dot on display next to speed (int RO bool)
long int SPDdashed;                   // Speed is in managed mode (---) (int RO bool)
// Heading (degrees)
long int HDGTRKmode;                  // HDG mode - HDG-VS or TRK-FPA (modify dataref to change - no command)
long int HDGmanaged;                  // Dot on display next to heading (int RO bool)
long int HDGdashed;                   // HDG is in managed mode (---) (int RO bool)
long int altitude_hold_ft;            // AP target altitude on display 
long int ALTmanaged;                  // Dot on display next to altitude
long int ALTdashed;                   // ALT is in managed mode (---)
// V/S speed
long int VSdashed;                    // Vertical speed is in managed mode (---)
long int LOCilluminated;              // LOC button Led
long int AP1Engage;                   // Engage AP1 (modify dataref to change - no command) - or "airbus_qpac/ap1_push"?
long int ATHRmode;                    // A/THR button Led
long int AP2Engage;                   // Engage AP2 (modify dataref to change - no command) - or "airbus_qpac/ap2_push"?
long int APVerticalMode;              // EXPED button Led (if value of dataref is >110)
long int APPRilluminated;             // APPR button Led
long int FCUAltKnobRotation;          // the actual rotary knob position in clicks with 20 clicks being one full turn. (Dataref values 0-19 with wrap-around.)
long int FCUHeadingKnobRotation;      // the actual rotary knob position in clicks with 20 clicks being one full turn. (Dataref values 0-19 with wrap-around.)
long int FCUSpeedKnobRotation;        // the actual rotary knob position in clicks with 20 clicks being one full turn. (Dataref values 0-19 with wrap-around.)
long int FCUVSKnobRotation;           // the actual rotary knob position in clicks with 20 clicks being one full turn. (Dataref values 0-19 with wrap-around.)


// *****************************************************************************************
// Define X-Plane commands
// *****************************************************************************************
// Side panel CAPT
int CaptChronoButton;                 // Push Chrono button

/*
// Side panel FO
int CoChronoButton;                   // Push Chrono button
*/

// FCU
int PushAltitude;                     // Push ALT knob
int PullAltitude;                     // Pull ALT knob
int PushVSSel;                        // Push V/S knob
int PullVSSel;                        // Pull V/S knob
int PushHDGSel;                       // Push HDG knob
int PullHDGSel;                       // Pull HDG knob
int PushSPDSel;                       // Push SPD knob
int PullSPDSel;                       // Pull SPD knob
int LOCbutton;                        // Push LOC button
int ATHRbutton;                       // Push A/THR button
int EXPEDbutton;                      // Push EXPED button
int APPRbutton;                       // Push APPR button

// *****************************************************************************************
// Define the required HW units
// *****************************************************************************************
// TM1637 display1;                      // define display - from TM1637 demo

/*
//// CD74HC4067 16-channel input/output multiplexer
// Input pins of input boards
const int sig_pin_in[] = {18,19,20};

// Output pin of output board
const int sig_pin_out = 21;

// The four control pins for all the boards
CD74HC4067 mux(22, 23, 24, 25);


//// TM1637 7-segment display
#define TM1637_DIN 29 // Common DIN pin

// Initialize displays on the output multiplexer
// Clock pin will be "sig_pin_out" of the output multiplexer, as the TM1367 displays will be connected to it
TM1637TinyDisplay display(sig_pin_out, TM1637_DIN); // 4-Digit Display Class
TM1637TinyDisplay6 display6(sig_pin_out, TM1637_DIN); // 6-Digit Display Class

*/
//// U8G2 displays
// U8g2 Contructor List (Frame Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
//
// Q: How can I use multiple SPI Displays?
// A: For each additional display, separate CS (Chip select) and RST (Reset) lines are required.
U8G2_SSD1309_128X64_NONAME0_F_4W_SW_SPI efis_left(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8); 

/*
//// Shifty for bit shifters, like the 16-channel DM13A or the 8-channel 74HC595
// Clock pin will be "sig_pin_out" of the output multiplexer, as the DM13A LED drivers will be connected to it
#define DM13A_DATA 28 // DATA pin
#define DM13A_LATCH 27 // LATCH pin
Shifty LED_Drivers; // Declare the shift register


//// Rotary encoders
constexpr unsigned encoderCount = 16; // number of attached encoders
constexpr unsigned S0 = 12;    // address pin 0
constexpr unsigned S1 = 13;    //...
constexpr unsigned S2 = 14;    //...
constexpr unsigned S3 = 15;    // address pin 3
constexpr unsigned SIG_A = 16; // output pin SIG of multiplexer A
constexpr unsigned SIG_B = 17; // output pin SIG of multiplexer B
// Initialize multiplexed encoders - name(encoderCount, S0, S1, S2, S3, SIG_A, SIG_B);
EncPlex4067 encoders(encoderCount, S0, S1, S2, S3, SIG_A, SIG_B);
*/

// *****************************************************************************************
// End of init section
// *****************************************************************************************


// *****************************************************************************************
// Start of setup section
// *****************************************************************************************
void setup() {
  // put your setup code here, to run once:

  pinMode(LED_BUILTIN, OUTPUT);                             // built in LED on arduino board for debug and demonstration purposes

  Serial.begin(XPLDIRECT_BAUDRATE);                         // start serial interface.  Baudrate is specified in the header, dont change   
  Xinterface.begin("Xplane TM1637 Demo");                   // needed for initialization.  Send a texual identifier of your device as a parameter.
 
  ErrorBlink(LED_BUILTIN, 5);       // let everyone know we are awake, not really necessary.

  // Initialize serial and wait for port to open:
  while (!Serial)
  {
    ErrorBlink(LED_BUILTIN, 2);
    delay(300);
  }
  digitalWrite(LED_BUILTIN, LOW);

  // *****************************************************************************************
  // Register XPLDireact datarefs
  // *****************************************************************************************
  // Parameters are:  registerDataRef(  char *        dataref name, a c style string that identifies the dataref
  //                                    int           read mode, can be XPL_READ or XPL_WRITE or XPL_READWRITE
  //                                    unsigned int  update rate.  0 for constant or specify milliseconds to wait before update from xplane.  currently only implemented for sending values.
  //                                    int           resolution divider.  Use this to reduce traffic.  For instance x-plane sends engine RPM as a constantly changing float value.  Use a divider of 10 to reduce resolution to tens of RPMs ie 1523.34 RPM becomes 1520 RPM
  //                                    long int * or float *   the value that will be sent or received from xplane
  //                            
  //                                    optionally, the last value can be an array and element pointer for xplane datarefs that return arrays:
  //                                    long int *, int index   *or*   float *, int index

  // Xinterface.registerDataRef("sim/cockpit2/gauges/indicators/compass_heading_deg_mag", XPL_READ, 100, 1, &compassHeading);                // From TM1637 demo

  Xinterface.registerDataRef("sim/cockpit2/switches/beacon_on", XPL_READ, 100, 0, &beacon);                                               // Use beacon light dataref for reading

  /// Side panel CAPT
  // Autoland Led
  Xinterface.registerDataRef("AirbusFBW/CaptStickInputOverride", XPL_READ, 100, 0, &CaptStickInputOverride);
  Xinterface.registerDataRef("sim/cockpit2/annunciators/master_warning", XPL_READ, 100, 0, &master_warning);                              // Master Warning Led (int RO bool)
  Xinterface.registerDataRef("sim/cockpit2/annunciators/master_caution", XPL_READ, 100, 0, &master_caution);                              // Master Caution Led  (int RO bool)
  // Capt Side Stick Priority chars Led

  /*
  /// Side panel FO
  Xinterface.registerDataRef("AirbusFBW/CoStickInputOverride", XPL_READ, 100, 0, &CoStickInputOverride);
  // FO Side Stick Priority chars Led
  */

  /// EFIS CAPT
  // QNH Led
  Xinterface.registerDataRef("AirbusFBW/BaroStdCapt", XPL_READWRITE, 100, 0, &BaroStdCapt);                                               // Baro setting on CPT side is "STD" -> turn off QNH Led if 1(int RW bool)
  // QFE Led (not implemented in Toliss)
  Xinterface.registerDataRef("AirbusFBW/FD1Engage", XPL_READWRITE, 100, 0, &FD1Engage);                                                   // FD Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/ILSonCapt", XPL_READWRITE, 100, 0, &ILSonCapt);                                                   // LS Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDShowCSTRCapt", XPL_READWRITE, 100, 0, &NDShowCSTRCapt);                                         // CSTR Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDShowWPTCapt", XPL_READWRITE, 100, 0, &NDShowWPTCapt);                                           // WPT Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDShowVORDCapt", XPL_READWRITE, 100, 0, &NDShowVORDCapt);                                         // VOR.D Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDShowNDBCapt", XPL_READWRITE, 100, 0, &NDShowNDBCapt);                                           // NDB Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDShowARPTCapt", XPL_READWRITE, 100, 0, &NDShowARPTCapt);                                         // ARPT Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDmodeCapt", XPL_READWRITE, 100, 0, &NDmodeCapt);                                                 // ND Nav mode (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDrangeCapt", XPL_READWRITE, 100, 0, &NDrangeCapt);                                               // ND Nav range (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)

  /*
  /// EFIS CO
  Xinterface.registerDataRef("AirbusFBW/BaroStdFO", XPL_READWRITE, 100, 0, &BaroStdCapt);                                                 // Baro setting on FO side is "STD" -> turn off QNH Led if 1(int RW bool)
  // QFE Led (not implemented in Toliss)
  Xinterface.registerDataRef("AirbusFBW/FD2Engage", XPL_READWRITE, 100, 0, &FD2Engage);                                                   // FD Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/ILSonFO", XPL_READWRITE, 100, 0, &ILSonFO);                                                       // LS Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDShowCSTRFO", XPL_READWRITE, 100, 0, &NDShowCSTRFO);                                             // CSTR Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDShowWPTFO", XPL_READWRITE, 100, 0, &NDShowWPTFO);                                               // WPT Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDShowVORDFO", XPL_READWRITE, 100, 0, &NDShowVORDFO);                                             // VOR.D Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDShowNDBFO", XPL_READWRITE, 100, 0, &NDShowNDBFO);                                               // NDB Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDShowARPTFO", XPL_READWRITE, 100, 0, &NDShowARPTFO);                                             // ARPT Led (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDmodeFO", XPL_READWRITE, 100, 0, &NDmodeFO);                                                     // ND Nav mode (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/NDrangeFO", XPL_READWRITE, 100, 0, &NDrangeFO);                                                   // ND Nav range (EFIS button + status): 0: Not engaged, 1: FD 1 active" (int RW bool)
  */


  /// FCU
  Xinterface.registerDataRef("sim/cockpit2/autopilot/airspeed_dial_kts_mach", XPL_READWRITE, 100, 0, &airspeed_dial_kts_mach);            // AP target speed on display, knots or mach, depends on "sim/cockpit2/autopilot/airspeed_is_mach" (float RW knots/mach)
  Xinterface.registerDataRef("sim/cockpit2/autopilot/airspeed_is_mach", XPL_READWRITE, 100, 0, &airspeed_is_mach);                        // Speed mode - 0: knots, 1: mach (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/SPDmanaged", XPL_READ, 100, 0, &SPDmanaged);                                                      // Dot on display next to speed  0: no, 1: yes (int RO bool)
  Xinterface.registerDataRef("AirbusFBW/SPDdashed", XPL_READ, 100, 0, &SPDdashed);                                                        // Speed is in managed mode (---)  0: no, 1: yes (int RO bool)
  // Heading (degrees)
  Xinterface.registerDataRef("AirbusFBW/HDGTRKmode", XPL_READWRITE, 100, 0, &HDGTRKmode);                                                 // HDG mode - HDG-VS or TRK-FPA (modify dataref to change - no command) 0: HDG/VS; 1: TRK/FPA (int RW enum)
  Xinterface.registerDataRef("AirbusFBW/HDGmanaged", XPL_READ, 100, 0, &HDGmanaged);                                                      // Dot on display next to heading (int RO bool)
  Xinterface.registerDataRef("AirbusFBW/HDGdashed", XPL_READ, 100, 0, &HDGdashed);                                                        // HDG is in managed mode (---)  0: no, 1: yes (int RO bool)
  Xinterface.registerDataRef("sim/cockpit2/autopilot/altitude_hold_ft", XPL_READ, 100, 0, &altitude_hold_ft);                             // AP target altitude on display (float RO feet)
  Xinterface.registerDataRef("AirbusFBW/ALTmanaged", XPL_READ, 100, 0, &ALTmanaged);                                                      // Dot on display next to altitude (int RO bool)
  Xinterface.registerDataRef("AirbusFBW/ALTdashed", XPL_READ, 100, 0, &ALTdashed);                                                        // ALT is in managed mode (---) 0: no, 1: yes (int RO bool)
  // V/S speed
  Xinterface.registerDataRef("AirbusFBW/VSdashed", XPL_READ, 100, 0, &VSdashed);                                                          // Vertical speed is in managed mode (---)  0: no, 1: yes (int RO bool)
  Xinterface.registerDataRef("AirbusFBW/LOCilluminated", XPL_READ, 100, 0, &LOCilluminated);                                              // LOC button Led (int RO bool)
  Xinterface.registerDataRef("AirbusFBW/AP1Engage", XPL_READWRITE, 100, 0, &AP1Engage);                                                   // Engage AP1 (modify dataref to change - no command) 0: AP not engaged, 1: AP engaged (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/ATHRmode", XPL_READ, 100, 0, &ATHRmode);                                                          // A/THR button Led 0: disengaged, 1: armed, 2: engaged/active (int RO enum)
  Xinterface.registerDataRef("AirbusFBW/AP2Engage", XPL_READWRITE, 100, 0, &AP2Engage);                                                   // Engage AP2 (modify dataref to change - no command)0: AP not engaged, 1: AP engaged (int RW bool)
  Xinterface.registerDataRef("AirbusFBW/APVerticalMode", XPL_READ, 100, 0, &APVerticalMode);                                              // EXPED button Led (on if value of dataref is >110) Vertical AP mode: 0=SRS, 1=CLB, 2=DES, 3=ALT CST*, 4=ALT CST, 6=G/S*, 7=G/S, 8=FINAL, 10=FLARE, 11=LAND; 101=OP CLB, 102=OP DES, 103=ALT*, 104=ALT, 105: ALT CRZ, 107=V/S or FPA, 112: EXP CLB, 113: EXP DES" (int RO enum)
  Xinterface.registerDataRef("AirbusFBW/APPRilluminated", XPL_READ, 100, 0, &APPRilluminated);                                            // APPR button Led (int RO bool)
  Xinterface.registerDataRef("AirbusFBW/FCUAltKnobRotation", XPL_READWRITE, 100, 0, &FCUAltKnobRotation);                                 // The actual rotary knob position in clicks with 20 clicks being one full turn. (Dataref values 0-19 with wrap-around.)
  Xinterface.registerDataRef("AirbusFBW/FCUHeadingKnobRotation", XPL_READWRITE, 100, 0, &FCUHeadingKnobRotation);                         // The actual rotary knob position in clicks with 20 clicks being one full turn. (Dataref values 0-19 with wrap-around.)
  Xinterface.registerDataRef("AirbusFBW/FCUSpeedKnobRotation", XPL_READWRITE, 100, 0, &FCUSpeedKnobRotation);                             // The actual rotary knob position in clicks with 20 clicks being one full turn. (Dataref values 0-19 with wrap-around.)
  Xinterface.registerDataRef("AirbusFBW/FCUVSKnobRotation", XPL_READWRITE, 100, 0, &FCUVSKnobRotation);                                   // The actual rotary knob position in clicks with 20 clicks being one full turn. (Dataref values 0-19 with wrap-around.)



  // *****************************************************************************************
  // Register XPLDireact commands
  // *****************************************************************************************
  // Side panel CAPT
  Xinterface.registerCommand("AirbusFBW/CaptChronoButton", &CaptChronoButton);

  //! Side panel FO
  // Xinterface.registerCommand("AirbusFBW/CoChronoButton", &CoChronoButton);

  /// EFIS CAPT

  /// EFIS FO

  //// FCU
  Xinterface.registerCommand("AirbusFBW/LOCbutton", &LOCbutton);
  Xinterface.registerCommand("AirbusFBW/ATHRbutton", &ATHRbutton);
  Xinterface.registerCommand("AirbusFBW/EXPEDbutton", &EXPEDbutton);
  Xinterface.registerCommand("AirbusFBW/APPRbutton", &APPRbutton);
  Xinterface.registerCommand("AirbusFBW/PushAltitude", &PushAltitude);
  Xinterface.registerCommand("AirbusFBW/PullAltitude", &PullAltitude);
  Xinterface.registerCommand("AirbusFBW/PushVSSel", &PushVSSel);
  Xinterface.registerCommand("AirbusFBW/PullVSSel", &PullVSSel);
  Xinterface.registerCommand("AirbusFBW/PushHDGSel", &PushHDGSel);
  Xinterface.registerCommand("AirbusFBW/PullHDGSel", &PullHDGSel);
  Xinterface.registerCommand("AirbusFBW/PushSPDSel", &PushSPDSel);
  Xinterface.registerCommand("AirbusFBW/PullSPDSelitude", &PullSPDSel);


  // *****************************************************************************************
  // Configure HW units
  // *****************************************************************************************
  // display1.begin(4,5);             //  clockpin, datapin - from TM1637 demo
  // display1.displayInt(888888);
  
  /*
    //// CD74HC4067 16-channel input/output multiplexer
  // Set the initial mode of the input pins
  for (int control : sig_pin_in) {
    pinMode(control, INPUT);
  }
  
  // Set the initial mode of the output pin
  pinMode(sig_pin_out, OUTPUT);


  //// TM1637 7-segment display
  // Set brightness
  display.setBrightness(0x0f);
  display6.setBrightness(0x0f);
  
  */
  //// U8G2 displays
  efis_left.begin();

  /*
  //// Shifty for bit shifters, like the 16-channel DM13A or the 8-channel 74HC595
  LED_Drivers.setBitCount(16); // Set the number of bits you have (multiples of 8)
  LED_Drivers.setPins(sig_pin_out, DM13A_DATA, DM13A_LATCH); // Set the clock, data, and latch pins you are using. This also sets the pinMode for these pins.


  //// Rotary encoders
  encoders.begin();


  //// XPLDirect - initialize
  pinMode(LED_BUILTIN, OUTPUT);     // amber LED on arduino board for debug and demonstration purposes
  Serial.begin(XPLDIRECT_BAUDRATE); // start serial interface.  Baudrate is specified in the header, dont change   
  Xinterface.begin("Xplane Mega2560"); // needed for initialization.  Send a textual identifier of your device as a parameter.
  ErrorBlink(LED_BUILTIN, 3);       // let everyone know we are awake, blink onboard amber LED
  */

}
// *****************************************************************************************
// End of setup section
// *****************************************************************************************

// *****************************************************************************************
// Start of main loop section
// *****************************************************************************************
void loop() {
  // put your main code here, to run repeatedly:

  Xinterface.xloop();  //  needs to run every cycle

  // *****************************************************************************************
  // everything after the next line will only occur every 300ms
  // *****************************************************************************************
  if (millis() - startTime > 1000) startTime = millis();   else return;                         
  if (!Xinterface.allDataRefsRegistered()) return;

  // *****************************************************************************************
  // Configure HW (buttons, switches, displays, etc.) here below:
  // *****************************************************************************************

  // display1.displayInt(compassHeading);    // from TM1637 demo

  /*
    // XPLDirect - Query dataref values
  // Need to insert XPLDirect functions into appropriate positions above for buttons, LEDs, switchws, rotaries, etc...
  if (beacon) digitalWrite(LED_BUILTIN, HIGH);        // if beacon is on set the builtin led on
  else        digitalWrite(LED_BUILTIN, LOW);
  
  //// CD74HC4067 16-channel input multiplexers
  byte mux_input; // Declare variable for reading the channels
  
  // Loop through the channels of the input multiplexer boards
  for (int i = 0; i < 16; i++) { // Loop through the 16 channels - main multiplexer loop
    mux.channel(i); // Switch to the selected channel

    switch (i) { // Switch-case for different actions on different channels
      case 0: // action on channel #0
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
          mux_input = digitalRead(input); // Read value from the actual channel on the selected input pin
    
          // Write status of the selected input pin on the actual channel to Serial Monitor
          Serial.print("Push button on input pin ");
          Serial.print(input);
          Serial.print(", on channel ");
          Serial.print(i);
          Serial.print(" is ");
    
          if (mux_input == HIGH) {
            Serial.println("not pressed!");
          } else if (mux_input == LOW) {
            Serial.println("pressed!");
          }
    
          // delay(100);
        }
        // end of action on channel #0

      case 1: // action on channel #1
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        } 

        // The TM1637 displays are in the main multiplexer loop (same control pins as the input multiplexer boards), but connected to the dedicated output multiplexer board (separate data pin on the Arduino).
        // Show something on a 4-digit display
        display.showNumber(100, false, 3, 0);    // Number, bool leading zero, maxdigits=3, position=0 (left)
        // or something else on a 6-digit display
        //display6.showString("\xB0", 1, 3);        // Degree Mark, length=1, position=3 (right)
        delay(500);
        display.clear(); // Clear Screen
      // end of action on channel #1

      case 2: // action on channel #2
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }

        // Rotary encoders are in the main multiplexer loop, but on different channel pins and data pins (wiring-wise completely separated), so the main loop only gives the channel ID, everything else is handled by the encoder library
        encoders.tick();
        if (encoders[i].valueChanged())
        {
          Serial.print("Encoder:");
          Serial.print(i);
          Serial.print(" value:");
          Serial.println(encoders[i].getValue());
        }
        // end of action on channel #2

      case 3: // action on channel #3
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #3

      case 4: // action on channel #4
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #4

      case 5: // action on channel #5
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #5

      case 6: // action on channel #6
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #6

      case 7: // action on channel #7
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #7

      case 8: // action on channel #8
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #8

      case 9: // action on channel #9
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #9

      case 10: // action on channel #10
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #10

      case 11: // action on channel #11
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #11

      case 12: // action on channel #12
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #12

      case 13: // action on channel #13
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #13

      case 14: // action on channel #14
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #14

      case 15: // action on channel #15
        // Input buttons and switches are in the main multiplexer loop, connected to the input multiplexer boards
        for (int input : sig_pin_in) { // Loop through input pins
        }
        // end of action on channel #15

    } // End of switch-case

  } // End of multiplexer input/output loop
  */

  efis_left.clearBuffer();					// clear the internal memory
  efis_left.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font
  efis_left.drawStr(0,10,"Hello World!");	// write something to the internal memory
  efis_left.sendBuffer();					// transfer internal memory to the display
  delay(1000);  
}
// *****************************************************************************************
// End of main loop section
// *****************************************************************************************


// *****************************************************************************************
// Function declarations
// *****************************************************************************************
void ErrorBlink(int pin, int count)
{
  for (int i = 0; i< count; i++)
  { 
   digitalWrite(pin, HIGH);
   delay(200);
   digitalWrite(pin, LOW);
   delay(100);
  }
}
