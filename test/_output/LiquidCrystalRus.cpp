#include "LiquidCrystalRus.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <avr/pgmspace.h>

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif



const PROGMEM uint8_t utf_recode[] = 
       { 0x41,0xa0,0x42,0xa1,0xe0,0x45,0xa3,0xa4,0xa5,0xa6,0x4b,0xa7,0x4d,0x48,0x4f,
         0xa8,0x50,0x43,0x54,0xa9,0xaa,0x58,0xe1,0xab,0xac,0xe2,0xad,0xae,0x62,0xaf,0xb0,0xb1,
         0x61,0xb2,0xb3,0xb4,0xe3,0x65,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0x6f,
         0xbe,0x70,0x63,0xbf,0x79,0xe4,0x78,0xe5,0xc0,0xc1,0xe6,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7
        };     
























LiquidCrystalRus::LiquidCrystalRus(uint8_t rs, uint8_t rw, uint8_t enable,
			     uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
			     uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
  init(0, rs, rw, enable, d0, d1, d2, d3, d4, d5, d6, d7);
}

LiquidCrystalRus::LiquidCrystalRus(uint8_t rs, uint8_t enable,
			     uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
			     uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
  init(0, rs, 255, enable, d0, d1, d2, d3, d4, d5, d6, d7);
}

LiquidCrystalRus::LiquidCrystalRus(uint8_t rs, uint8_t rw, uint8_t enable,
			     uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
  init(1, rs, rw, enable, d0, d1, d2, d3, 0, 0, 0, 0);
}

LiquidCrystalRus::LiquidCrystalRus(uint8_t rs,  uint8_t enable,
			     uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
  init(1, rs, 255, enable, d0, d1, d2, d3, 0, 0, 0, 0);
}

void LiquidCrystalRus::init(uint8_t fourbitmode, uint8_t rs, uint8_t rw, uint8_t enable,
			 uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
			 uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
  _rs_pin = rs;
  _rw_pin = rw;
  _enable_pin = enable;
  
  _data_pins[0] = d0;
  _data_pins[1] = d1;
  _data_pins[2] = d2;
  _data_pins[3] = d3; 
  _data_pins[4] = d4;
  _data_pins[5] = d5;
  _data_pins[6] = d6;
  _data_pins[7] = d7; 

  pinMode(_rs_pin, OUTPUT);
  
  if (_rw_pin != 255) { 
    pinMode(_rw_pin, OUTPUT);
  }
  pinMode(_enable_pin, OUTPUT);
  
  if (fourbitmode)
    _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  else 
    _displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;
  
  begin(16, 1);  
}

void LiquidCrystalRus::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
  if (lines > 1) {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = lines;
  _currline = 0;

  
  if ((dotsize != 0) && (lines == 1)) {
    _displayfunction |= LCD_5x10DOTS;
  }

  
  
  
  delayMicroseconds(50000); 
  
  digitalWrite(_rs_pin, LOW);
  digitalWrite(_enable_pin, LOW);
  if (_rw_pin != 255) { 
    digitalWrite(_rw_pin, LOW);
  }
  
  
  if (! (_displayfunction & LCD_8BITMODE)) {
    
    

    
    writeNbits(0x03,4);
    delayMicroseconds(4500); 

    
    writeNbits(0x03,4);
    delayMicroseconds(4500); 
    
    
    writeNbits(0x03,4); 
    delayMicroseconds(150);

    
    writeNbits(0x02,4); 
  } else {
    
    

    
    command(LCD_FUNCTIONSET | _displayfunction);
    delayMicroseconds(4500);  

    
    command(LCD_FUNCTIONSET | _displayfunction);
    delayMicroseconds(150);

    
    command(LCD_FUNCTIONSET | _displayfunction);
  }

  
  command(LCD_FUNCTIONSET | _displayfunction);  

  
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
  display();

  
  clear();

  
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  
  command(LCD_ENTRYMODESET | _displaymode);

}

void LiquidCrystalRus::setDRAMModel(uint8_t model) {
  _dram_model = model;
}


void LiquidCrystalRus::clear()
{
  command(LCD_CLEARDISPLAY);  
  delayMicroseconds(2000);  
}

void LiquidCrystalRus::home()
{
  command(LCD_RETURNHOME);  
  delayMicroseconds(2000);  
}

void LiquidCrystalRus::setCursor(uint8_t col, uint8_t row)
{
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row >= _numlines ) {
    row = _numlines-1;    
  }
  
  command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}


void LiquidCrystalRus::noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystalRus::display() {
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}


void LiquidCrystalRus::noCursor() {
  _displaycontrol &= ~LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystalRus::cursor() {
  _displaycontrol |= LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}


void LiquidCrystalRus::noBlink() {
  _displaycontrol &= ~LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystalRus::blink() {
  _displaycontrol |= LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}


void LiquidCrystalRus::scrollDisplayLeft(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void LiquidCrystalRus::scrollDisplayRight(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}


void LiquidCrystalRus::leftToRight(void) {
  _displaymode |= LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}


void LiquidCrystalRus::rightToLeft(void) {
  _displaymode &= ~LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}


void LiquidCrystalRus::autoscroll(void) {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}


void LiquidCrystalRus::noAutoscroll(void) {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}



void LiquidCrystalRus::createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; 
  command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    write(charmap[i]);
  }
}



inline void LiquidCrystalRus::command(uint8_t value) {
  send(value, LOW);
}

#if defined(ARDUINO) && ARDUINO >= 100
  size_t LiquidCrystalRus::write(uint8_t value)
#else
  void   LiquidCrystalRus::write(uint8_t value)
#endif
{
  uint8_t out_char=value;

  if (_dram_model == LCD_DRAM_WH1601) {  
    uint8_t ac=recv(LOW) & 0x7f;
    if (ac>7 && ac<0x14) command(LCD_SETDDRAMADDR | (0x40+ac-8));
  }

  if (value>=0x80) { 
    if (value >= 0xc0) {
      utf_hi_char = value - 0xd0;
    } else {
      value &= 0x3f;
      if (!utf_hi_char && (value == 1)) 
        send(0xa2,HIGH); 
      else if ((utf_hi_char == 1) && (value == 0x11)) 
        send(0xb5,HIGH); 
      else 
        send(pgm_read_byte_near(utf_recode + value + (utf_hi_char<<6) - 0x10), HIGH);
    }    
  } else send(out_char, HIGH);
#if defined(ARDUINO) && ARDUINO >= 100
  return 1; 
#endif
}




void LiquidCrystalRus::send(uint8_t value, uint8_t mode) {
  digitalWrite(_rs_pin, mode);

  
  if (_rw_pin != 255) { 
    digitalWrite(_rw_pin, LOW);
  }
  
  if (_displayfunction & LCD_8BITMODE) {
    writeNbits(value,8); 
  } else {
    writeNbits(value>>4,4);
    writeNbits(value,4);
  }
}


uint8_t LiquidCrystalRus::recv(uint8_t mode) {
  uint8_t retval;
  digitalWrite(_rs_pin, mode);

  
  if (_rw_pin != 255) { 
    digitalWrite(_rw_pin, HIGH);
  }
  
  if (_displayfunction & LCD_8BITMODE) {
    retval = readNbits(8); 
  } else {
    retval = readNbits(4) << 4;
    retval |= readNbits(4);
  }
  return retval;
}
void LiquidCrystalRus::pulseEnable() {
  digitalWrite(_enable_pin, LOW);
  delayMicroseconds(1);    
  digitalWrite(_enable_pin, HIGH);
  delayMicroseconds(1);    
  digitalWrite(_enable_pin, LOW);
  delayMicroseconds(100);   
}

void LiquidCrystalRus::writeNbits(uint8_t value, uint8_t n) {
  for (int i = 0; i < n; i++) {
    pinMode(_data_pins[i], OUTPUT);
    digitalWrite(_data_pins[i], (value >> i) & 0x01);
  }

  pulseEnable();
}

uint8_t LiquidCrystalRus::readNbits(uint8_t n) {
  uint8_t retval=0;
  for (int i = 0; i < n; i++) {
    pinMode(_data_pins[i], INPUT);
  }

  digitalWrite(_enable_pin, LOW);
  delayMicroseconds(1);    
  digitalWrite(_enable_pin, HIGH);
  delayMicroseconds(1);    
  
  for (int i = 0; i < n; i++) {
    retval |= (digitalRead(_data_pins[i]) == HIGH)?(1 << i):0;
  }

  digitalWrite(_enable_pin, LOW);

  return retval;
}

