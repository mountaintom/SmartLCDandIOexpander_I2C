// This is a Smart I2C LCD Display backpack adapter and I/O expander.
//
// This is intended to replace the common PCF8571 based I2C backpack
// adapters used with inexpensive (2 x 16 and 4 x 20) parallel LCD displays.
// This adapter code can be programmed into a Nano and mounted
// to the back of the LCD display.
// This adapter has extended commands that allow the adapter to
// read and write I/O pins on the Nano, in addition to the
// adapt or functioning as a generic I2C LCD display adapter.
//
// This I2C Display adapter code should handle IC2 LCD display data
// (from a normal IC2 LCD display library) and Extended I/O commands without
// coordination or interference with each other.
//
// This is a general use smart display adapter. However,
// it was inspired by the desire to create a simple way to
// expand the available I/O ports with the uBITX radio transceiver.
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

// I2C clock and data pins are of both Arduino's
// are connected together as a bus.
// The I2C bus clock and data pins need to
// have pull-up resistors. 10K ohms to either
// 5V or 3.3V work. Use 3.3V if you have
// other parts on the bus that run on 3.3V.
// This works without level shifters.
//
// It would be preferred that rather than
// branching of numerous versions of the
// interface library, that improvements and additions
// be contributed back to this library.
//
// However, for those who want the minimal interface,
// it is actually easy to create the commands by hand.
// You could cut and paste just what you need from the included
// interface library code, such as the rotate code, into your code.

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
// This the included interface library provides
// error status check functions.
// If in simple projects you choose to ignore errors,
// then the the library will behave in a pseudo "wires
// fell off" mode. The digital pins will read high
// and the analog voltage readings will go to 2.0 volts
//

#include <stdio.h>
#include <string.h>
// ----

// These libraries can be installed using
// Sketch -> Include Library -> Manage Libraries.
#include <SoftTimer.h>  //https://github.com/prampec/arduino-softtimer
// Also requires: https://github.com/prampec/arduino-pcimanager

// Standard Libraries
#include <LiquidCrystal.h>
#include <Wire.h>

#define I2C_ADDRESS 0x27
#define I2C_ADDRESS_ALT 0x3F

#define LCD_COLUMNS 16
#define LCD_ROWS 2

#define LCD_D4_PIN 5
#define LCD_D5_PIN 6
#define LCD_D6_PIN 7
#define LCD_D7_PIN 8

#define LCD_RS_PIN 2
#define LCD_RW_PIN 3
#define LCD_E_PIN 4
#define LCD_LED_PIN 9

#define LCD_D4_IDATA 4
#define LCD_D5_IDATA 5
#define LCD_D6_IDATA 6
#define LCD_D7_IDATA 7

#define LCD_RS_IDATA 0
#define LCD_RW_IDATA 1
#define LCD_E_IDATA 2
#define LCD_LED_IDATA 3

#define DEFAULT_LCD_LED_BRIGHTNESS 255
#define DEFAULT_FILTER_VALUE 0.1

#define EXTENDED_COMMAND_MAX_LENGTH 32

// Shameless globals
int _lcdLedBrightness;
bool _lcdLedOn = false;
bool _extendedCommandsOnly = false;
String _extendedCommand;
bool _extendedCommandReady = false;

// If these numbers are incrementing you may wish to explore why.
long _extendedCommandInvalidCnt = 0;     // Debug counter
long _extendedCommandOverflowCnt = 0;    // Debug counter
long _extendedCommandNotReadCnt = 0;     // Debug counter
long _extendedCommandNoSendDataCnt = 0;  // Debug counter
// How many microseconds latest analogScan was blocking;
long _analogMicros;  // Debug/Monitoring

String _sendData;
bool _sendDataReady;

float _analogValue[8];
float _analogFilterValue[8];
unsigned int _analogSamplesSinceRead[8];
unsigned int _analogSamplesSinceStart[8];

float _analogValueMax[8];

bool _analogActive[] = {false, false, false, false, false, false, false, false};

const float analogScalingFactor = 1.10 / 1023.0;

float _chipTemp = 0.0;

LiquidCrystal lcd(LCD_RS_PIN, LCD_E_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN,
                  LCD_D7_PIN);

void scanAnalog(Task* me);
void scanAnalogMax(Task* me);
void serialDebugMessage(Task* me);

Task taskScanAnalog(3, scanAnalog);
Task taskScanAnalogMax(5, scanAnalogMax);
Task taskSerialDebugMessage(10000, serialDebugMessage);

void setup() {
  pinMode(3, INPUT_PULLUP);
  delay(25);
  // Place a 1K pulldown resistor on pin D3 to use alternate I2C address
  int i2c_address = (digitalRead(3)) ? I2C_ADDRESS : I2C_ADDRESS_ALT;

  initPinsDefault();

  initLCD();
  initLiguidCrystal();
  initAnalog();

  Wire.begin(i2c_address);  // join i2c
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.begin(9600);  // start serial for output

  SoftTimer.add(&taskScanAnalog);
  SoftTimer.add(&taskScanAnalogMax);
  SoftTimer.add(&taskSerialDebugMessage);

  // Semi random seed. Naturally will center around common temperatures.
  randomSeed(abs(GetTemp() * (2147483647L / 150)));
}

void initPinsDefault() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);  // Default to OUTPUT. LED (L) may be turned on and off.
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT);  // I2C bus. Must be INPUT
  pinMode(A5, INPUT);  // I2C bus. Must be INPUT
  pinMode(A6, INPUT_PULLUP);
  pinMode(A7, INPUT_PULLUP);

  // Note: The I2C bus needs to be pulled up by external resistors. In the case
  // of the uBITX transceiver the I2C bus is shared with 3.3V parts. In this
  // case, the clock and data pins are pulled up with 10K ohm resistors to the
  // Arduino Nano 3.3V pin. This works without level shifters. Although, just
  // barely in spec. Do not accidentally set the I2C pins to outputs as this
  // could damage other ICs connected to the bus.
}

void initLCD() {
  pinMode(LCD_RS_PIN, OUTPUT);
  pinMode(LCD_RW_PIN, OUTPUT);
  pinMode(LCD_E_PIN, OUTPUT);
  pinMode(LCD_LED_PIN, OUTPUT);

  pinMode(LCD_D4_PIN, OUTPUT);
  pinMode(LCD_D5_PIN, OUTPUT);
  pinMode(LCD_D6_PIN, OUTPUT);
  pinMode(LCD_D7_PIN, OUTPUT);

  digitalWrite(LCD_RW_PIN, 0);
  digitalWrite(LCD_E_PIN, 0);
  setLcdLedBrightness(DEFAULT_LCD_LED_BRIGHTNESS);
  setLcdLedOn();
}

void initLiguidCrystal() {
  // set up the LCD's number of columns and rows:
  lcd.begin(LCD_COLUMNS, LCD_ROWS);

  lcd.print("B6B Display I/O");
  lcd.setCursor(0, 1);
  lcd.print("V 0.1.1");
}

void initAnalog() {
  // This adapter requires this
  // to be configured for internal.
  analogReference(INTERNAL);
  // Set pins A6 and A7 to actively reading
  // analog by default.
  pinMode(A6, INPUT);  // Analog input
  pinMode(A7, INPUT);  // Analog input
  analogStart("xx", "A6", "01", "");
  analogStart("xx", "A7", "01", "");
  // Let settings settle down some.
  delay(50);
}

void setLcdLedBrightness(int brightness) {
  analogWrite(LCD_LED_PIN, brightness);
  _lcdLedBrightness = brightness;
  _lcdLedOn = true;
}

void setLcdLedOn() {
  if (!_lcdLedOn) {
    analogWrite(LCD_LED_PIN, _lcdLedBrightness);
    _lcdLedOn = true;
  }
}

void setLcdLedOff() {
  analogWrite(LCD_LED_PIN, 0);
  _lcdLedOn = false;
}

void writeLCD(byte dataLCD) {
  digitalWrite(LCD_RW_PIN, 0);
  digitalWrite(LCD_RS_PIN, (dataLCD >> LCD_RS_IDATA) & 0x01);
  digitalWrite(LCD_D4_PIN, (dataLCD >> LCD_D4_IDATA) & 0x01);
  digitalWrite(LCD_D5_PIN, (dataLCD >> LCD_D5_IDATA) & 0x01);
  digitalWrite(LCD_D6_PIN, (dataLCD >> LCD_D6_IDATA) & 0x01);
  digitalWrite(LCD_D7_PIN, (dataLCD >> LCD_D7_IDATA) & 0x01);
  digitalWrite(LCD_E_PIN, (dataLCD >> LCD_E_IDATA) & 0x01);

  if (((dataLCD >> LCD_LED_IDATA) & 0x01) == 0) {
    setLcdLedOff();
  } else {
    setLcdLedOn();
  }
}

void scanAnalog(Task* me) {
  // The analog reads will block for around 150 microseconds each.

  long startedTask = micros();

  // Force this to be sure it is correct
  analogReference(INTERNAL);

  for (int pin = 0; pin < 8; pin++) {
    if (_analogActive[pin]) {
      float scaledReading = analogScalingFactor * (float)analogRead(pin);
      _analogValue[pin] +=
        (scaledReading - _analogValue[pin]) * _analogFilterValue[pin];
      // These peg the counters to the max value for the data type once a
      // counter wraps around.
      _analogSamplesSinceRead[pin] =
        max(_analogSamplesSinceRead[pin], _analogSamplesSinceRead[pin] + 1);
      _analogSamplesSinceStart[pin] =
        max(_analogSamplesSinceStart[pin], _analogSamplesSinceStart[pin] + 1);
    }
  }
  _chipTemp += (GetTemp() - _chipTemp) * 0.05;

  _analogMicros = micros() - startedTask;
}

void scanAnalogMax(Task* me) {
  // Capture the peak analog values with a slow decay time.
  // The decay time is a starting point. May want to adjust it to taste.
  float decayValue = 0.001;

  for (int pin = 0; pin < 8; pin++) {
    if (_analogActive[pin]) {
      float analogValue = _analogValue[pin];

      _analogValueMax[pin] = max(_analogValueMax[pin], analogValue);
      _analogValueMax[pin] += (analogValue - _analogValueMax[pin]) * decayValue;

    } else {
      _analogValueMax[pin] = 0.0;
    }
  }
}

void serialDebugMessage(Task* me) {
  Serial.println(F(" == Periodic Debug Message =="));
  printDebugInfo();
}

void printDebugInfo() {
  Serial.print(F("Overflow Count: "));
  Serial.println(_extendedCommandOverflowCnt);
  Serial.print(F("Invalid Count: "));
  Serial.println(_extendedCommandInvalidCnt);
  Serial.print(F("Not Read Count: "));
  Serial.println(_extendedCommandNotReadCnt);
  Serial.print(F("No Send Data Count: "));
  Serial.println(_extendedCommandNoSendDataCnt);
  Serial.print(F("Current Command Buffer: "));
  Serial.println(_extendedCommand);
  Serial.print(F("Current Send Buffer: "));
  Serial.println(_sendData);

  for (int i = 0; i < 8; i++) {
    Serial.print(F("Analog Value "));
    Serial.print(i);
    Serial.print(": ");
    Serial.println(_analogValue[i], 4);
  }
  Serial.print(F("Analog Scan time in microseconds: "));
  Serial.println(_analogMicros);
  Serial.print(F("Internal chip temperature: "));
  Serial.println(_chipTemp, 4);
  Serial.print(F("Alt Internal chip temperature: "));
  Serial.println(GetTemp(), 4);
  Serial.print(F("Digital Pin D12 level: "));
  Serial.println(digitalRead(12));
  Serial.print(F("Digital Pin D13 level: "));
  Serial.println(digitalRead(13));
  Serial.print(F("Random Number: "));
  Serial.println(random());
}

double GetTemp(void) {
  // This routine is from:
https:  // playground.arduino.cc/Main/InternalTemperatureSensor

  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  // Reference should already be at 1.1V, so delay unnecessary
  // and would cause blocking issue.
  // delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA, ADSC))
    ;

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31) / 1.22;

  // The returned temperature is in degrees Celsius.
  return (t);
}

void gatherExtendedIoCommands(byte iData) {
  // Collect data with Extended IO selector set high. Set command ready
  // flag true when valid looking command if found.
  // Routine is agnostic whether or not command was sent in one buffer or
  // one byte at a time. This is so this routine could be use by other pure
  // serial connections or the user for some reason does not use a buffer to
  // send the command all at once. The extended commands should work anyway.
  //
  // Data was rotated right two places to allow data to work around
  // the RW bit being in the second bit position as used by
  // the i2c LCD display library. This allows the RW bit (as it is otherwise
  // never used) to become the selector bit for i2c LCD emulation and extended
  // IO commands.

  // These funny shifts emulate a rotate to the right.
  byte rData = (iData >> LCD_RW_IDATA + 1 | iData << 8 - (LCD_RW_IDATA + 1));
  rData &= 0x7F;  // Mask out extended IO select bit, now shifted to D7.

  // Let c strings be received without harm. Just ignore zero terminators.
  if (rData == 0x00) {
    return;
  }

  // Serial.print((char) rData);
  // Serial.print(" -- ");
  // Serial.print(rData, HEX);
  // Serial.print(" --- ");
  // Serial.println(iData, HEX);
  if (_extendedCommandReady == true) {
    _extendedCommandReady = false;
    _extendedCommand = "";
  }

  if (_extendedCommand.length() <= EXTENDED_COMMAND_MAX_LENGTH) {
    if (rData == '\n') {
      _extendedCommandReady = true;
      Serial.print(" +++ Cmd +++ ");
      Serial.println(_extendedCommand);
    } else {
      _extendedCommand.concat((char)rData);
    }
  } else {
    _extendedCommandOverflowCnt++;

    // Purge buffer when end of command is received so
    // buffer max will remain as indicator of Overflow condition.
    if (rData == '\n') {
      _extendedCommand = "";
    }
  }
}

void processCommand(String eCommand) {
  // Command structure is a fixed uniform structure for simple
  // generation/parsing.

  String commandID =
    "xx";  // Simple validity check that read data is for read request.
  String iPin = "x00";           // Target of command. Generally a pin ID.
  String primaryCommand = "xx";  // The main extended command.
  String subCommand =
    "xx";  // Secondary command or Pin command high/low as 01/00;
  String variableLengthData = "";  // Floating point or string data.

  // Note: commandID is normally a random number sent by client. The purpose is
  // to flag software issues where the user has different parts of their code
  // unintentionally sending read requests. Without this ID, diagnosing the
  // strange behavior this could produce would be difficult. If you get any
  // mismatches between the sent ID and the received ID, you have something to
  // look at to fix in your code. For simple clients used in very simple
  // straight line code, you may ignore the ID.
  //
  // This I2C Display adapter code should handle IC2 LCD display data
  // (from a normal IC2 LCD display library) and Extended I/O commands without
  // coordination or interference with each other.

  int commandLength = eCommand.length();

  if (commandLength >= 2) {
    commandID = eCommand.substring(0, 2);
  }
  if (commandLength >= 5) {
    iPin = eCommand.substring(2, 5);
  }
  if (commandLength >= 7) {
    primaryCommand = eCommand.substring(5, 7);
  }
  if (commandLength >= 9) {
    subCommand = eCommand.substring(7, 9);
  }
  if (commandLength >= 10) {
    variableLengthData = eCommand.substring(9);
  }

  bool cmdFound = false;

  // Function call.
  // void function (String CommandID, String iPin, String subCommand, String
  // variableLengthData);

  // Shortened names used in functions.
  // void function (String cID, String iPin String sCmd, String vData);

  // Block all attempts to change the I2C pins.
  if (iPin.equalsIgnoreCase("A04") || iPin.equalsIgnoreCase("A05")) {
    return;
  }
  if (iPin.equalsIgnoreCase("D18") || iPin.equalsIgnoreCase("D19")) {
    return;
  }

  typedef void (*cmd_func)(String commandID, String iPin, String subCommand,
                           String variableLengthData);

  struct commands {
    String command;
    cmd_func function;
  };

  // Process commands in a loop.
  // Only optimization is
  // to put most used commands first.
  static commands command_list[] = {{String("DR"), readPin},
    {String("DW"), writePin},
    {String("AR"), readAnalog},
    {String("AW"), writeAnalog},
    {String("AS"), analogStart},
    {String("PM"), modePin},
    {String("LP"), displayPrint},
    {String("LC"), displayCommand},
    {String("LB"), displayBrightness},
    {String("LO"), displayLedOnOff},
    {String("EO"), extendedCommandsOnly},
    {String("XC"), calculatedValue},
    {String("DB"), printSerialDebugMessage}
  };

  cmdFound = false;
  for (int i = 0; i < sizeof(command_list); i++) {
    if (primaryCommand.equals(command_list[i].command)) {
      cmdFound = true;
      command_list[i].function(commandID, iPin, subCommand, variableLengthData);
      break;
    }
  }
  if (cmdFound == false) {
    _extendedCommandInvalidCnt++;
  }
}

void receiveEvent(int howMany) {
  // Serial.println(Wire.available()); //Debug
  while (1 <= Wire.available()) {
    byte iData = (byte)Wire.read();  // receive byte
    // Serial.print(c);         // Debug - print the character
    if (((iData >> LCD_RW_IDATA) & 0x01) == 0) {
      // RW bit is set low, so pass direct to LCD.
      if (_extendedCommandsOnly == false) {
        writeLCD(iData);
      }
    } else {
      // RW bit is high so this is part of
      // an extended command.
      gatherExtendedIoCommands(iData);
      if (_extendedCommandReady == true) {
        processCommand(_extendedCommand);
      }
    }
  }
}

void requestEvent() {
  if (_sendDataReady && (_sendData.length() >= 4)) {
    // Serial.print("Send Data: "); // Debug
    // Serial.println(_sendData); // Debug

    if (_sendData.length() >= EXTENDED_COMMAND_MAX_LENGTH - 1) {
      _extendedCommandNoSendDataCnt++;
      Wire.write("ExxF00\n");  // Buffer Overflow
      return;
    }

    _sendData.concat('\n');  // Append end of data token
    Wire.write(_sendData.c_str(), _sendData.length());
    _sendDataReady = false;

  } else {
    _extendedCommandNoSendDataCnt++;
    Wire.write("Exx300\n");  // No data to send
  }
}

int pinToInt(String pin) {
  // Used by digital read/write and pin mode.
  if (pin.substring(0, 1).equals("D")) {
    return pin.substring(1, 3).toInt();
  } else if (pin.equals("A00")) {
    // These return the integer values for the AXX address aliases
    return A0;
  } else if (pin.equals("A01")) {
    return A1;
  } else if (pin.equals("A02")) {
    return A2;
  } else if (pin.equals("A03")) {
    return A3;
  } else if (pin.equals("A04")) {
    return A4;
  } else if (pin.equals("A05")) {
    return A5;
  } else if (pin.equals("A06")) {
    return A6;
  } else if (pin.equals("A07")) {
    return A7;
  }
  _extendedCommandInvalidCnt++;
  return -1;
}

void modePin(String cID, String iPin, String sCmd, String vData) {
  String pmode = sCmd;
  int pin = pinToInt(iPin);

  // Set mode if pinToInt thinks in is valid
  // Mostly concerned about analog pins as they are used as
  // array indexes.
  if (pin >= 0) {
    if (pmode.equals("OT")) {
      pinMode(pin, OUTPUT);

    } else if (pmode.equals("IN")) {
      pinMode(pin, INPUT);

    } else if (pmode.equals("IP")) {
      pinMode(pin, INPUT_PULLUP);

    } else {
      _extendedCommandInvalidCnt++;
    }

    // If analog pin is set as digital, then disable the analogScan and reset
    // values.
    if (iPin.substring(0, 1).equals("A")) {
      int a_pin = iPin.substring(1, 3).toInt();

      // This is a redundant range check, as pinToInt checked analog pins.
      // But going to leave it in for consistency with analogStart routine.
      if (a_pin >= 0 && a_pin <= 7) {
        _analogActive[a_pin] = false;
        _analogSamplesSinceRead[a_pin] = 0;
        _analogSamplesSinceStart[a_pin] = 0;
        _analogValue[a_pin] = 0.0;
      }
    }
  }
}

void readPin(String cID, String iPin, String sCmd, String vData) {
  // Return data format:
  // tiiep
  //   t = type of return message
  //     (P = Digital pin, A = Analog, U = user, E = Error with no data)
  //   ii = Command ID for simple validity check.
  //   e  = error code. Odd codes are errors, even are warnings.
  //   p = Pin level (1/0).

  int pin = pinToInt(iPin);
  bool level = digitalRead(pin);

  _sendData = "P";
  _sendData += cID;  // Return command validity check.

  if (_sendDataReady == true) {
    _extendedCommandNotReadCnt++;
    _sendData += "B";  // Error code
  } else {
    _sendData += "0";  // No error
  }
  _sendData += (level) ? '1' : '0';

  _sendDataReady = true;
}

void writePin(String cID, String iPin, String sCmd, String vData) {
  bool level = (sCmd.toInt() == 1);
  int pin = pinToInt(iPin);
  digitalWrite(pin, level);
}

void analogStart(String cID, String iPin, String sCmd, String vData) {
  float filterValue = vData.toFloat();
  int pin = pinToInt(iPin);  // Translate to alias pin values for A0-A7

  bool runStop;

  if (sCmd.toInt() == 1) {
    runStop = true;
    ;
  } else {
    runStop = false;
  }

  // Only do this for analog input pins
  if (iPin.substring(0, 1).equals("A")) {
    int a_pin = iPin.substring(1, 3).toInt();

    if (a_pin >= 0 && a_pin <= 7) {
      pinMode(pin, INPUT);
      _analogActive[a_pin] = runStop;
      _analogSamplesSinceRead[a_pin] = 0;
      _analogSamplesSinceStart[a_pin] = 0;
      _analogValue[a_pin] = 0.0;

      if (filterValue > 0) {
        _analogFilterValue[a_pin] = filterValue;
      } else {
        _analogFilterValue[a_pin] = DEFAULT_FILTER_VALUE;
      }
    } else {
      _extendedCommandInvalidCnt++;  // Outside of allowed pin range
    }
  } else {
    _extendedCommandInvalidCnt++;  // Not an analog pin
  }
}

void readAnalog(String cID, String iPin, String sCmd, String vData) {
  // Gets values from memory buffers.
  // Return data format:
  // tiiensvv...
  //   t = type of return message
  //     (P = Digital pin, A = Analog, U = user, E = Error with no data)
  //   ii = Command ID for simple validity check.
  //   e  = error code. Odd codes are errors, even are warnings.
  //   n = new sample since last read.
  //   s = LOG of number of samples since analog read started
  //   vvvv = floating point value of analog voltage read.(Variable length)
  // The command ID is returned to handle cases where the user may attempt a
  // multitasking interface to this device and send more than one type of read
  // request before the actual bus read takes place.

  // Error Codes:
  // Use an error if the receiving end should drop the returned data.
  // Use a warning if the receiving end can use the data, but
  // debugging or more sophisticated receiving software might make use of.
  // Simplest clients will simply error on odd codes and ignore even codes.
  // Some codes will only make sense as warnings and vise versa.
  // Where one error is the cause of another, send the root cause error.
  // Invalid received commands may be silently ignored.
  //
  // 0 = no errors
  // 1 = generic error code.
  // 2 = Read request without new send data as warning
  // 3 = Read request without new send data as error
  // 4 = Previous send buffer was overwritten and never sent as warning
  // 5 = Previous send buffer was overwritten and never sent as error
  // 6 = Invalid command as warning
  // 7 = Invalid command as error
  // 8 = Pin number out of range as warning
  // 9 = Pin number out of range as error
  // B = Input buffer overflow as warning
  // C = Input buffer overflow as error
  // E = Return buffer overflow as warning
  // F = Return buffer overflow as error

  // Only do this for analog input pins
  if (iPin.substring(0, 1).equals("A")) {
    int a_pin = iPin.substring(1, 3).toInt();

    String errorCode;

    if (_sendDataReady == true) {
      _extendedCommandNotReadCnt++;
      errorCode = "B";  // Previous data unread
    } else {
      errorCode = "0";  // No error
    }

    if (a_pin >= 0 && a_pin <= 7) {
      if (_analogActive[a_pin] == true) {
        // Send back command ID so receiver can validate the data
        // received is for request they made.
        _sendData = "A";
        _sendData.concat(cID);
        _sendData.concat(errorCode);  // Error code
        // Some indicators of settling time of analog values.
        _sendData.concat(min(_analogSamplesSinceRead[a_pin], 9));
        _sendData.concat(min((int)log(_analogSamplesSinceStart[a_pin]), 9));
        // Filtered analog value.
        _sendData.concat(String(_analogValue[a_pin], 4));
        _sendDataReady = true;
        _analogSamplesSinceRead[a_pin] = 0;
      }

    } else {
      _extendedCommandInvalidCnt++;  // Outside of allowed pin range
    }
  } else {
    _extendedCommandInvalidCnt++;  // Not an analog pin
  }
}

void writeAnalog(String cID, String iPin, String sCmd, String vData) {
  float dutyCycle = vData.toFloat();

  if (iPin.substring(0, 1).equals("D")) {
    int pin = iPin.substring(1, 3).toInt();

    analogWrite(pin, constrain(dutyCycle * 255.0, 0, 255));

  } else {
    _extendedCommandInvalidCnt++;
  }
}

void displayPrint(String cID, String iPin, String sCmd, String vData) {
  lcd.print(vData);
}

void displayCommand(String cID, String iPin, String sCmd, String vData) {
  String lCommand = sCmd;
  int column = vData.substring(0, 3).toInt();
  int row = vData.substring(3, 6).toInt();

  if (String("CL").equals(lCommand)) {
    lcd.clear();
  } else if (String("HM").equals(lCommand)) {
    lcd.home();
  } else if (String("SC").equals(lCommand)) {
    lcd.setCursor(column, row);
  } else if (String("CO").equals(lCommand)) {
    lcd.cursor();
  } else if (String("CN").equals(lCommand)) {
    lcd.noCursor();
  } else if (String("BO").equals(lCommand)) {
    lcd.blink();
  } else if (String("BN").equals(lCommand)) {
    lcd.noBlink();
  } else if (String("DO").equals(lCommand)) {
    lcd.display();
  } else if (String("DN").equals(lCommand)) {
    lcd.noDisplay();
  } else if (String("SL").equals(lCommand)) {
    lcd.scrollDisplayLeft();
  } else if (String("SR").equals(lCommand)) {
    lcd.scrollDisplayRight();
  } else if (String("AS").equals(lCommand)) {
    lcd.autoscroll();
  } else if (String("AN").equals(lCommand)) {
    lcd.noAutoscroll();
  } else if (String("LR").equals(lCommand)) {
    lcd.leftToRight();
  } else if (String("RL").equals(lCommand)) {
    lcd.rightToLeft();
  } else {
    _extendedCommandInvalidCnt++;
  }
}

void calculatedValue(String cID, String iPin, String sCmd, String vData) {
  // Place code for custom calculated results you would
  // like to be returned via the I2C bus or sent directly to the LCD display.
  //
  // Included is a sample calculation used with transmitters and antennas.
  //
  // Note: If you are sending to the LCD display and it is also being written
  // to, independently, by another uncoordinated process, the results
  // may not be what you want.

  String cCommand = sCmd;
  String userData = vData;

  String result;

  if (String("PF").equals(cCommand)) {
    // "Forward" Power.
    // Based on 1VDC is 100 RMS RF volts (141 V peak)
    // into a 50 load. So 1 volt DC would be 200 watts.
    // If you are using a directional coupler / SWR bridge
    // adjust the values as needed.

    result = String(sq((_analogValueMax[6] * 100.0)) / 50.0);
  } else if (String("PR").equals(cCommand)) {
    // "Reverse" Power.

    result = String(sq((_analogValueMax[7] * 100.0)) / 50.0);

  } else if (String("SW").equals(cCommand)) {
    // "SWR"

    float vf = _analogValueMax[6];
    float vr = _analogValueMax[7];
    float vd = (vf - vr);
    vd = min(vd, 0.002);  // Limit max SWR reading before divide by zero
    // Calculate SWR from directional coupler voltages.
    result = String((vf + vr) / vd);

  } else if (String("TC").equals(cCommand)) {
    // The internal chip temp is only approximate.
    result = String(_chipTemp);
  } else if (String("TF").equals(cCommand)) {
    result = String((_chipTemp * 9.0 / 5.0) + 32);
  } else if (String("RN").equals(cCommand)) {
    result = String(random());
  } else if (String("XC").equals(cCommand)) {
    // Debug display command.
    result = ":ID:" + cID + ":Pn:" + iPin + ":vD:" + vData;
  } else {
    _extendedCommandInvalidCnt++;
    return;
  }

  if (iPin.equals("XDS")) {
    lcd.print(result);
  } else {
    String errorCode;

    if (_sendDataReady == true) {
      _extendedCommandNotReadCnt++;
      errorCode = "B";  // Previous data unread
    } else {
      errorCode = "0";  // No error
    }

    // Send back command ID so receiver can validate the data
    // received is for request they made.
    _sendData = "U";
    _sendData.concat(cID);
    _sendData.concat(errorCode);  // Error code
    _sendData.concat(result);
    _sendDataReady = true;
  }
}

void displayBrightness(String cID, String iPin, String sCmd, String vData) {
  float brightness = vData.toFloat();

  setLcdLedBrightness(constrain(brightness * 255.0, 0, 255));
}

void displayLedOnOff(String cID, String iPin, String sCmd, String vData) {
  if (sCmd.toInt() == 1) {
    setLcdLedOn();
  } else {
    setLcdLedOff();
  }
}

void extendedCommandsOnly(String cID, String iPin, String sCmd, String vData) {
  if (sCmd.toInt() == 1) {
    _extendedCommandsOnly = true;
  } else {
    _extendedCommandsOnly = false;
  }
}

void printSerialDebugMessage(String cID, String iPin, String sCmd,
                             String vData) {
  Serial.println(" == Debug Command output:");

  printDebugInfo();

  Serial.println(" == More Debug Info:");

  Serial.print("cID: ");
  Serial.println(cID);

  Serial.print("Pin: ");
  Serial.println(iPin);

  Serial.print("Sub command: ");
  Serial.println(sCmd);

  Serial.print("vData: ");
  Serial.println(vData);
}
