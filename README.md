# RTC-embedded-project

// ---------------------------------------------------------------------------
// GLOBAL DEFINES
#define F_CPU 16000000L // run CPU at 16 MHz
#define LED 5 // Boarduino LED on PB5
#define ClearBit(x,y) x &= ~_BV(y) // equivalent to cbi(x,y)
#define SetBit(x,y) x |= _BV(y) // equivalent to sbi(x,y)
// ---------------------------------------------------------------------------
// INCLUDES
#include <avr/io.h> // deal with port registers
#include <util/delay.h> // used for _delay_ms function
#include <string.h> // string manipulation routines
#include <stdlib.h>
// ---------------------------------------------------------------------------
// TYPEDEFS
typedef uint8_t byte; // I just like byte & sbyte better
typedef int8_t sbyte;
// ---------------------------------------------------------------------------
// MISC ROUTINES
void InitAVR()
{
DDRB = 0x3F; // 0011.1111; set B0-B5 as outputs
DDRC = 0x00; // 0000.0000; set PORTC as inputs
}
void msDelay(int delay) // put into a routine
{ // to remove code inlining
for (int i=0;i<delay;i++) // at cost of timing accuracy
_delay_ms(1);
}
void FlashLED()
{
SetBit(PORTB,LED);
msDelay(250);
ClearBit(PORTB,LED);
msDelay(250);
}
// ---------------------------------------------------------------------------
// HD44780-LCD DRIVER ROUTINES
//
// Routines:
// LCD_Init initializes the LCD controller
// LCD_Cmd sends LCD controller command
// LCD_Char sends single ascii character to display
// LCD_Clear clears the LCD display & homes cursor
// LCD_Home homes the LCD cursor
// LCD_Goto puts cursor at position (x,y)
// LCD_Line puts cursor at start of line (x)
// LCD_Hex displays a hexadecimal value
// LCD_Integer displays an integer value
// LCD_String displays a string
//
// The LCD module requires 6 I/O pins: 2 control lines & 4 data lines.
// PortB is used for data communications with the HD44780-controlled LCD.
// The following defines specify which port pins connect to the controller:
#define LCD_RS 0 // pin for LCD R/S (eg PB0)
#define LCD_E 1 // pin for LCD enable
#define DAT4 2 // pin for d4
#define DAT5 3 // pin for d5
#define DAT6 4 // pin for d6
#define DAT7 5 // pin for d7
// The following defines are HD44780 controller commands
#define CLEARDISPLAY 0x01
#define SETCURSOR 0x80
void PulseEnableLine ()
{
SetBit(PORTB,LCD_E); // take LCD enable line high
_delay_us(40); // wait 40 microseconds
ClearBit(PORTB,LCD_E); // take LCD enable line low
}
void SendNibble(byte data)
{
PORTB &= 0xC3; // 1100.0011 = clear 4 data lines
if (data & _BV(4)) SetBit(PORTB,DAT4);
if (data & _BV(5)) SetBit(PORTB,DAT5);
if (data & _BV(6)) SetBit(PORTB,DAT6);
if (data & _BV(7)) SetBit(PORTB,DAT7);
PulseEnableLine(); // clock 4 bits into controller
}
void SendByte (byte data)
{
SendNibble(data); // send upper 4 bits
SendNibble(data<<4); // send lower 4 bits
ClearBit(PORTB,5); // turn off boarduino LED
}
void LCD_Cmd (byte cmd)
{
ClearBit(PORTB,LCD_RS); // R/S line 0 = command data
SendByte(cmd); // send it
}
void LCD_Char (byte ch)
{
SetBit(PORTB,LCD_RS); // R/S line 1 = character data
SendByte(ch); // send it
}
void LCD_Init()
{
LCD_Cmd(0x33); // initialize controller
LCD_Cmd(0x32); // set to 4-bit input mode
LCD_Cmd(0x28); // 2 line, 5x7 matrix
LCD_Cmd(0x0C); // turn cursor off (0x0E to enable)
LCD_Cmd(0x06); // cursor direction = right
LCD_Cmd(0x01); // start with clear display
msDelay(3); // wait for LCD to initialize
}
void LCD_Clear() // clear the LCD display
{
LCD_Cmd(CLEARDISPLAY);
msDelay(3); // wait for LCD to process command
}
void LCD_Home() // home LCD cursor (without clearing)
{
LCD_Cmd(SETCURSOR);
}
void LCD_Goto(byte x, byte y) // put LCD cursor on specified line
{
byte addr = 0; // line 0 begins at addr 0x00
switch (y)
{
case 1: addr = 0x40; break; // line 1 begins at addr 0x40
case 2: addr = 0x14; break;
case 3: addr = 0x54; break;
}
LCD_Cmd(SETCURSOR+addr+x); // update cursor with x,y position
}
void LCD_Line(byte row) // put cursor on specified line
{
LCD_Goto(0,row);
}
void LCD_String(const char *text) // display string on LCD
{
while (*text) // do until /0 character
LCD_Char(*text++); // send char & update char pointer
}
void LCD_Hex(int data)
// displays the hex value of DATA at current LCD cursor position
{
char st[8] = ""; // save enough space for result
itoa(data,st,16); // convert to ascii hex
//LCD_Message("0x"); // add prefix "0x" if desired
LCD_String(st); // display it on LCD
}
void LCD_Integer(int data)
// displays the integer value of DATA at current LCD cursor position
{
char st[8] = ""; // save enough space for result
itoa(data,st,10); // convert to ascii
LCD_String(st); // display in on LCD
}
// ---------------------------------------------------------------------------
// I2C (TWI) ROUTINES
//
// On the AVRmega series, PA4 is the data line (SDA) and PA5 is the clock (SCL
// The standard clock rate is 100 KHz, and set by I2C_Init. It depends on the AVR osc. freq.
#define F_SCL 100000L // I2C clock speed 100 KHz
#define READ 1
#define TW_START 0xA4 // send start condition (TWINT,TWSTA,TWEN)
#define TW_STOP 0x94 // send stop condition (TWINT,TWSTO,TWEN)
#define TW_ACK 0xC4 // return ACK to slave
#define TW_NACK 0x84 // don't return ACK to slave
#define TW_SEND 0x84 // send data (TWINT,TWEN)
#define TW_READY (TWCR & 0x80) // ready when TWINT returns to logic 1.
#define TW_STATUS (TWSR & 0xF8) // returns value of status register
#define I2C_Stop() TWCR = TW_STOP // inline macro for stop condition
void I2C_Init()
// at 16 MHz, the SCL frequency will be 16/(16+2(TWBR)), assuming prescalar of 0.
// so for 100KHz SCL, TWBR = ((F_CPU/F_SCL)-16)/2 = ((16/0.1)-16)/2 = 144/2 = 72.
{
TWSR = 0; // set prescalar to zero
TWBR = ((F_CPU/F_SCL)-16)/2; // set SCL frequency in TWI bit register
}
byte I2C_Detect(byte addr)
// look for device at specified address; return 1=found, 0=not found
{
TWCR = TW_START; // send start condition
while (!TW_READY); // wait
TWDR = addr; // load device's bus address
TWCR = TW_SEND; // and send it
while (!TW_READY); // wait
return (TW_STATUS==0x18); // return 1 if found; 0 otherwise
}
byte I2C_FindDevice(byte start)
// returns with address of first device found; 0=not found
{
for (byte addr=start;addr<0xFF;addr++) // search all 256 addresses
{
if (I2C_Detect(addr)) // I2C detected?
return addr; // leave as soon as one is found
}
return 0; // none detected, so return 0.
}
void I2C_Start (byte slaveAddr)
{
I2C_Detect(slaveAddr);
}
byte I2C_Write (byte data) // sends a data byte to slave
{
TWDR = data; // load data to be sent
TWCR = TW_SEND; // and send it
while (!TW_READY); // wait
return (TW_STATUS!=0x28);
}
byte I2C_ReadACK () // reads a data byte from slave
{
TWCR = TW_ACK; // ack = will read more data
while (!TW_READY); // wait
return TWDR;
//return (TW_STATUS!=0x28);
}
byte I2C_ReadNACK () // reads a data byte from slave
{
TWCR = TW_NACK; // nack = not reading more data
while (!TW_READY); // wait
return TWDR;
//return (TW_STATUS!=0x28);
}
void I2C_WriteByte(byte busAddr, byte data)
{
I2C_Start(busAddr); // send bus address
I2C_Write(data); // then send the data byte
I2C_Stop();
}
void I2C_WriteRegister(byte busAddr, byte deviceRegister, byte data)
{
I2C_Start(busAddr); // send bus address
I2C_Write(deviceRegister); // first byte = device register address
I2C_Write(data); // second byte = data for device register
I2C_Stop();
}
byte I2C_ReadRegister(byte busAddr, byte deviceRegister)
{
byte data = 0;
I2C_Start(busAddr); // send device address
I2C_Write(deviceRegister); // set register pointer
I2C_Start(busAddr+READ); // restart as a read operation
data = I2C_ReadNACK(); // read the register data
I2C_Stop(); // stop
return data;
}
// ---------------------------------------------------------------------------
// DS1307 RTC ROUTINES
#define DS1307 0xD0 // I2C bus address of DS1307 RTC
#define SECONDS_REGISTER 0x00
#define MINUTES_REGISTER 0x01
#define HOURS_REGISTER 0x02
#define DAYOFWK_REGISTER 0x03
#define DAYS_REGISTER 0x04
#define MONTHS_REGISTER 0x05
#define YEARS_REGISTER 0x06
#define CONTROL_REGISTER 0x07
#define RAM_BEGIN 0x08
#define RAM_END 0x3F
void DS1307_GetTime(byte *hours, byte *minutes, byte *seconds)
// returns hours, minutes, and seconds in BCD format
{
*hours = I2C_ReadRegister(DS1307,HOURS_REGISTER);
*minutes = I2C_ReadRegister(DS1307,MINUTES_REGISTER);
*seconds = I2C_ReadRegister(DS1307,SECONDS_REGISTER);
if (*hours & 0x40) // 12hr mode:
*hours &= 0x1F; // use bottom 5 bits (pm bit = temp & 0x20)
else *hours &= 0x3F; // 24hr mode: use bottom 6 bits
}
void DS1307_GetDate(byte *months, byte *days, byte *years)
// returns months, days, and years in BCD format
{
*months = I2C_ReadRegister(DS1307,MONTHS_REGISTER);
*days = I2C_ReadRegister(DS1307,DAYS_REGISTER);
*years = I2C_ReadRegister(DS1307,YEARS_REGISTER);
}
void SetTimeDate()
// simple, hard-coded way to set the date.
{
I2C_WriteRegister(DS1307,MONTHS_REGISTER, 0x08);
I2C_WriteRegister(DS1307,DAYS_REGISTER, 0x31);
I2C_WriteRegister(DS1307,YEARS_REGISTER, 0x13);
I2C_WriteRegister(DS1307,HOURS_REGISTER, 0x08+0x40); // add 0x40 for PM
I2C_WriteRegister(DS1307,MINUTES_REGISTER, 0x51);
I2C_WriteRegister(DS1307,SECONDS_REGISTER, 0x00);
}
// ---------------------------------------------------------------------------
// APPLICATION ROUTINES
void ShowDevices()
// Scan I2C addresses and display addresses of all devices found
{
LCD_Line(1); LCD_String("Found:");
byte addr = 1;
while (addr>0)
{
LCD_Char(' ');
addr = I2C_FindDevice(addr);
if (addr>0) LCD_Hex(addr++);
}
}
void LCD_TwoDigits(byte data)
// helper function for WriteDate()
// input is two digits in BCD format
// output is to LCD display at current cursor position
{
byte temp = data>>4;
LCD_Char(temp+'0');
data &= 0x0F;
LCD_Char(data+'0');
}
void WriteDate()
{
byte months, days, years;
DS1307_GetDate(&months,&days,&years);
LCD_TwoDigits(months);
LCD_Char('/');
LCD_TwoDigits(days);
LCD_Char('/');
LCD_TwoDigits(years);
}
void WriteTime()
{
byte hours, minutes, seconds;
DS1307_GetTime(&hours,&minutes,&seconds);
LCD_TwoDigits(hours);
LCD_Char(':');
LCD_TwoDigits(minutes);
LCD_Char(':');
LCD_TwoDigits(seconds);
}
void LCD_TimeDate()
{
LCD_Line(0); WriteTime();
LCD_Line(1); WriteDate();
}
// ---------------------------------------------------------------------------
// PROGRAM LOOP
void MainLoop()
{
while(1)
{
LCD_TimeDate(); // put time & date on LCD
msDelay(1000); // one second between updates
}
}
// ---------------------------------------------------------------------------
// MAIN PROGRAM
int main(void)
{
InitAVR(); // set port direction
LCD_Init(); // initialize HD44780 LCD controller
I2C_Init(); // set I2C clock frequency
LCD_String("Ready.");
ShowDevices(); // show that I2C is working OK
msDelay(4000);
LCD_Clear();
MainLoop(); // display time
}
