// SmartLCDandIOexpander_I2C interface library.
// This library communicates with a Arduino programmed as
// a SmartLCDandIOexpander_I2C LCD Display backpack.
//
// I2C clock and data pins are of both Arduino's are
// connected together as a bus.
// The I2C bus clock and data pins need to
// have pull-up resistors. 10K ohms to either
// 5V or 3.3V work. Use 3.3V if you have
// other parts on the bus that run on 3.3V.
// This works without level shifters.
//
// It would be preferred that rather than
// branching of numerous versions of this
// library, that improvements and additions
// be contributed back to this library.
//
// However, for those who want the minimal interface,
// it is actually easy to create the commands by hand.
// You could cut and paste just what you need,
// such as the rotate code, into your code.

// Digital write a 1 to pin 13 would be:
//   "xxD13DW01\n"
// Analog write 57% duty cycle to pin 10 would be:
//   "xxD10AWxx0.57\n"
// Digital read from pin 12 would be:
//   "xxD12DR\n"
// Analog read pin A3 would be:
//   "xxA03AR\n"
// Set pin D10 to output mode would be:
//   "xxD10PMOT"
//
// The trick is the bits in each character need
// to be rotated left by 2, so the extended commands
// can work using the never used
// LCD R/W pin as a selector to route
// the extended commands to the extended IO code.
//
// The read commands need to be followed up with
// read routine using Wire.requestFrom().
// The return data does not need to be rotated.
//
// The command structure looks like this.
//
//"iipppccssvvvvv\n\0";
//
// ii       = command ID.
// ppp      = pin number.
// cc       = primary command.
// ss       = sub command.
// vvvvv... = variable length data (up to buffer size - 11).
// \n command termination.
// \0 Standard C string termination.
//
// Note: cID can be ignored in
// simple straight line code projects.
//
// This interface provides error status check functions.
// If in simple projects you choose to ignore errors,
// then the this library will behave in a pseudo wires
// fell off mode. The digital pins will read high
// and the analog voltage readings will go to 2.0 volts
//
// Author: Tom Stall, WB6B <mtm<@>mountaintom<.>com>
// Copyright: 2018 Tom Stall
// Version: 0.1.1
// Date: 20180702
// License: GNU General Public License

/**
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

#include "Arduino.h"
#include <Wire.h>

#ifndef SmartLCDandIOexpander_I2C_H
#define SmartLCDandIOexpander_I2C_H

#define I2C_ADDRESS 0x27

#define VDATA_SIZE 32

enum E_ERRORS {
  E_OK = 0x00,
  E_ERROR = 0x01,       // Generic error code
  E_NORDATA = 0x03,     // No data was received
  E_NOSDATA = 0x05,     // No data was sent
  E_CIDERROR = 0x07,    // cID of received data did not match
  E_RBOVERFLOW = 0x09,  // Received data overflowed buffer
  E_RTYPE = 0X0B,       // Return did not match expected type
  E_COMFAIL = 0x0D,     // Bus Failure
  E_DERRER = 0x0F,      // Display adapter reported error
};

class SmartLcdIO {
 public:
  SmartLcdIO(byte i2c_addr);
  SmartLcdIO(void);

  // Extended I/O methods

  int eError(void);

  char eDisplayError(void);

  void eDigitalWrite(char* pin, bool level);

  bool eDigitalRead(char* pin);

  void eAnalogWrite(char* pin, float dutyCycle);

  float eAnalogRead(char* pin);

  void eAnaloglStart(char* pin, bool on, float filterFactor);

  void ePinMode(char* pin, char* mode);

  // Send command to custom commands.
  // Custom commands generally calculate something and
  // either return the result or directly send it to
  // the LCD display.
  //
  // You will need to convert your send data to a string
  // and send it in vData. This is up to you because
  // custom commands can interpret vData however
  // they need to.
  char* eCustomCommand(char* pin, char* customCommand, char* vData);

  // Send a debug message over the display's serial port.
  void eDebugMessage(char* pin, bool on, float floatValue);

  // LCD Display methods.
  // Except as noted, these function the
  // same as the LiquidCrystal library.
  void eDisplayBrightness(float brightness);  // Custom
  void eDisplayLedOnOff(bool level);          // Custom

  void ePrint(char* printData);
  void eClear(void);
  void eHome(void);
  void eSetCursor(byte column, byte row);
  void eCursor(void);
  void eNoCursor(void);
  void eBlink(void);
  void eNoBlink(void);
  void eDisplay(void);
  void eNoDisplay(void);
  void eScrollDisplayLeft(void);
  void eScrollDisplayRight(void);
  void eAutoscroll(void);
  void eNoAutoscroll(void);
  void eLeftToRight(void);
  void eRightToLeft(void);

 private:
  char* eCommandSend(char* pin, char* pCmd, char* sCmd, char* vData);
  char* eGetReturnedData(char* pin, char* pCmd);
  int eReceiveData(void);
};

#endif
