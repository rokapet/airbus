#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//// U8G2 displays
// U8g2 Contructor List (Frame Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
// cs=SS  

U8G2_SSD1309_128X64_NONAME0_F_4W_SW_SPI efis_left(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);  
//U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI efis_left(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);  
//U8G2_SSD1309_128X64_NONAME2_F_4W_SW_SPI efis_left(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);  
//U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI efis_left(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8); 

#define LCDWidth                        efis_left.getDisplayWidth()
#define LCDHeight                       efis_left.getDisplayHeight()
// int QNH
// int QNH_w
// int QNH_x

void setup() {
  // put your setup code here, to run once:
  //// U8G2 displays
  efis_left.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  efis_left.clearBuffer();					// clear the internal memory
  efis_left.setFont(u8g2_font_lubB14_tr);	// choose a suitable font
  efis_left.drawStr(10,22,"QFE");	// write something to the internal memory
  efis_left.drawStr(74,22,"QNH");	// write something to the internal memory  

  efis_left.setFont(u8g2_font_logisoso34_tr);	// choose a suitable font
  int QNH = 1013;
  int QNH_w = efis_left.getStrWidth(QNH);      // returns string width - not workig on an int, need to find out how to convert to string
  int QNH_x = ((LCDWidth - QNH_w) / 2);        // should work, but QNH_w does not
  efis_left.setCursor(20, LCDHeight-2);        // should use QNH_x
  efis_left.print(String(QNH));
  efis_left.sendBuffer();					// transfer internal memory to the display
  delay(1000);  
}
