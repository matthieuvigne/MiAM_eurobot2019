#include "MiAMEurobot/drivers/LCDDriver.h"
#include <stdio.h>
#include <unistd.h>

// Define registers and command list.

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// Constant definition
int rs_pin = 15;
int rw_pin = 14;
int enable_pin = 13;
int data_pins[4] = {12, 11, 10, 9};

LCD::LCD()
{
}


bool LCD::init(I2CAdapter *adapter, int const& address)
{
    // Init MPC chip.
    if(mpc_init(&mpc_, adapter, address) == false)
        return false;
    // Configure MPC I/O.
    mpc_pinMode(mpc_, 8, MPC_OUTPUT);
    mpc_pinMode(mpc_, 7, MPC_OUTPUT);
    mpc_pinMode(mpc_, 6, MPC_OUTPUT);

    mpc_pinMode(mpc_, rs_pin, MPC_OUTPUT);
    mpc_pinMode(mpc_, rw_pin, MPC_OUTPUT);
    mpc_pinMode(mpc_, enable_pin, MPC_OUTPUT);

    for(int x=0; x < 4; x++)
        mpc_pinMode(mpc_, data_pins[x], MPC_OUTPUT);

    // Turn off all legs, set all the other lines to low.
    unsigned int pinValue = 0;
    pinValue = pinValue | (0b111 << 6);
    mpc_writeAll(mpc_, pinValue);

    // Reset LCD into 4 bit mode : see HD44780 datasheet, figure 24 pg. 46.
    sendCommand(0x33);
    sendCommand(0x32);

    // Set display mode.
    sendCommand(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    // Turn on display, disable cursor and blink.
    sendCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    // Set the entry mode
    sendCommand(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);

    // Clear LCD display.
    clear();
    return true;
}

void LCD::clear()
{
    sendCommand(LCD_CLEARDISPLAY);
}

void LCD::setText(std::string const& text, int line)
{
    mutex_.lock();
    // If line is not zero, set offset to write to line 1.
    if(line != 0)
        line  = 0x40;
    // Go to beginning of line.
    sendCommand(LCD_SETDDRAMADDR | line);
    int stringLength = text.length();
    if (stringLength > 16)
        stringLength = 16;
    for(int x = 0; x < stringLength; x++)
        sendChar(text[x]);
    // Pad string with spaces if needed.
    for(int x = stringLength; x < 16; x++)
        sendChar(' ');
    mutex_.unlock();
}

void LCD::setChar(char text, int line, int column)
{
    mutex_.lock();
    usleep(200);
    int offset = 0;
    // If line is not zero, set offset to write to line 1.
    if(line != 0)
        offset  = 0x40;
    if(column < 0)
        column = 0;
    if(column > 15)
        column = 15;
    offset += column;
    // Go to beginning of line.
    sendCommand(LCD_SETDDRAMADDR | offset);
    sendChar(text);
    mutex_.unlock();
}


void LCD::setTextCentered(std::string const& text, int line)
{
    // Compute number of space needed - on odd number the string will be left-aligned.
    int numberOfSpaces = (16 - int(text.length())) / 2;
    std::string centeredText = text;
    if(numberOfSpaces > 0)
        centeredText = std::string(numberOfSpaces, ' ') +  text;
    setText(centeredText, line);
}


void LCD::setBacklight(bool red, bool green, bool blue)
{
    mpc_digitalWrite(mpc_, 6, ~(red ? 1 : 0) & 0x1);
    mpc_digitalWrite(mpc_, 7, ~(green ? 1 : 0) & 0x1);
    mpc_digitalWrite(mpc_, 8, ~(blue ? 1 : 0) & 0x1);
}

bool LCD::isButtonPressed(LCDButton button)
{
    if(mpc_digitalRead(mpc_, button) == 0)
        return true;
    return false;
}


void LCD::pulseEnable()
{
    mpc_digitalWrite(mpc_, enable_pin, 1);
    usleep(1);    // enable pulse must be >450ns
    mpc_digitalWrite(mpc_, enable_pin, 0);
    usleep(40);   // commands need > 37us to settle
}


// Send data over 4bits interface, with specific rs pin value.
void LCD::sendData(unsigned char const& data, unsigned char const& rsValue)
{
    unsigned int portValue = mpc_readAll(mpc_);

    // Set RS, RW and enable to 0
    portValue &= ~(1 << rs_pin);
    portValue &= ~(1 << rw_pin);
    portValue &= ~(1 << enable_pin);

    // Set RS pin to right value
    portValue |= (rsValue << rs_pin);

    // Set data pin : first 4 bits.
    for (int x = 0; x < 4; x++)
    {
        portValue &= ~(1 << data_pins[x]);
        portValue |= (((data >> (4 + x)) & 0x01) << data_pins[x]);
    }
    mpc_writeAll(mpc_, portValue);
    pulseEnable();

    // Second 4 bits.
    for (int x = 0; x < 4; x++)
    {
        portValue &= ~(1 << data_pins[x]);
        portValue |= (((data >> (x)) & 0x01) << data_pins[x]);
    }
    mpc_writeAll(mpc_, portValue);
    pulseEnable();
}


// Send a command to the LCD screen, in 4 bits mode
void LCD::sendCommand(unsigned char value)
{
    sendData(value, 0);
}


// Send a character to the LCD screen, in 4 bits mode
void LCD::sendChar(unsigned char value)
{
    sendData(value, 1);
}
