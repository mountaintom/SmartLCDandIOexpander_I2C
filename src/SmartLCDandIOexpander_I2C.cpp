// SmartLCDandIOexpander_I2C interface library.
// This library communicates with a Arduino programmed as
// a SmartLCDandIOexpander_I2C LCD Display backpack.
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

#include "SmartLCDandIOexpander_I2C.h"

const int vData_Size = VDATA_SIZE + 3;

byte _i2cAddr;

// If these numbers are incrementing you may wish to explore why.
long _eReturnDataOverflowCnt = 0;  // Debug/Monitoring
long _eReturnDataNoDataCnt = 0;    // Debug/Monitoring
long _badCIDCnt = 0;               // Debug/Monitoring

char _lastCID[3];

E_ERRORS _lastError = E_OK;

char _lastDisplayError = '0';  // Last error sent by display

struct eCommand {
  char cID[2];
  char pin[3];
  char pCmd[2];
  char sCmd[2];
  char vData[vData_Size];
} eCBuf;
struct eCommand* eCBuf_p = &eCBuf;
char* eCBuf_as_char_p = (char*)eCBuf_p;

struct eReturnData {
  char rType[1];
  char cID[2];
  char error[1];
  char vData[33];
} eRBuf;
struct eReturnData* eRBuf_p = &eRBuf;

struct digitalReturnData {
  char rType[1];
  char cID[2];
  char error[1];
  char level[2];
};

struct analogReturnData {
  char rType[1];
  char cID[2];
  char error[1];
  char readSamples[1];
  char startSamples[1];
  char value[30];
};

SmartLcdIO::SmartLcdIO(byte i2c_addr) { _i2cAddr = i2c_addr; }

SmartLcdIO::SmartLcdIO(void) { _i2cAddr = 0x27; }

int SmartLcdIO::eError(void) { return (_lastError); }

char SmartLcdIO::eDisplayError(void) { return (_lastDisplayError); }

void SmartLcdIO::eDigitalWrite(char* pin, bool level) {
  eCommandSend(pin, "DW", (level) ? "01" : "00", "\0");
}
bool SmartLcdIO::eDigitalRead(char* pin) {
  char* rData = eCommandSend(pin, "DR", "\0\0", "\0");

  // If error, float level to one
  // Like if the wire fell off the pin
  if (_lastError != E_OK) {
    return (true);
  }

  return (rData[0] == '0') ? false : true;
}

void SmartLcdIO::eAnalogWrite(char* pin, float dutyCycle) {
  char sBuf[9];

  dtostrf(dutyCycle, 6, 5, sBuf);

  eCommandSend(pin, "AW", "xx", sBuf);
}

float SmartLcdIO::eAnalogRead(char* pin) {
  // Returns the analog voltage read by the pin in volts.
  // Range is from 0 to 1.1 volts.

  char* rData = eCommandSend(pin, "AR", "\0\0", "\0");

  // If there is an error, float voltage up to two volts.
  if (_lastError != E_OK) {
    rData = "2.00";
  }

  return (atof(rData));
}

void SmartLcdIO::eAnaloglStart(char* pin, bool on, float filterFactor) {
  // Activate scanning of an analog pin. If a filterFactor is
  // included it will be used in the smoothing filter.
  // Otherwise the default value of 0.10 will be used.

  char sBuf[8] = {0};

  if (filterFactor > 0) {
    dtostrf(filterFactor, 6, 4, sBuf);
  }

  eCommandSend(pin, "AS", (on) ? "01" : "00", sBuf);
}

void SmartLcdIO::ePinMode(char* pin, char* mode) {
  eCommandSend(pin, "PM", mode, "\0");
}

void SmartLcdIO::eDisplayBrightness(float brightness) {
  char sBuf[8];

  dtostrf(brightness, 6, 4, sBuf);

  eCommandSend("xxx", "LB", "xx", sBuf);
}

void SmartLcdIO::eDisplayLedOnOff(bool level) {
  eCommandSend("xxx", "LO", (level) ? "01" : "00", "\0");
}

char* SmartLcdIO::eCustomCommand(char* pin, char* customCommand, char* vData) {
  // Send command to custom commands.
  // Custom commands generally calculate something and
  // either return the result or directly send it to
  // the LCD display.
  //
  // You will need to convert your send data to a string
  // and send it in vData. This is up to you because
  // custom commands can interpret vData however
  // they need to.

  char* rData = eCommandSend(pin, "XC", customCommand, vData);

  return (rData);
}

void SmartLcdIO::eDebugMessage(char* pin, bool on, float floatValue) {
  // Send a debug message back over the display's serial port.
  char sBuf[8] = {0};

  dtostrf(floatValue, 6, 5, sBuf);

  eCommandSend(pin, "DB", (on) ? "01" : "00", sBuf);
}

char* SmartLcdIO::eCommandSend(char* pin, char* pCmd, char* sCmd, char* vData) {
  _lastError = E_OK;

  memset(eCBuf_p, 0, sizeof(*eCBuf_p));

  bool pinShortLength = (strlen(pin) < 3);

  // Pad pin id when, for example, "A2" is specified
  // rather than "A02".
  eCBuf_p->pin[0] = pin[0];
  eCBuf_p->pin[1] = (pinShortLength) ? '0' : pin[1];
  eCBuf_p->pin[2] = (pinShortLength) ? pin[1] : pin[2];

  strncpy(eCBuf_p->pCmd, pCmd, 2);
  strncpy(eCBuf_p->sCmd, sCmd, 2);
  strncpy(eCBuf_p->vData, vData, sizeof(eCBuf_p->vData) - 1);

  // Likely the random numbers are not correctly seeded,
  // but it makes no difference for this use.
  // This is mainly a canary sentinel function.
  eCBuf_p->cID[0] = random(97, 123);
  eCBuf_p->cID[1] = random(97, 123);

  strncpy(_lastCID, eCBuf_p->cID, 2);

  // Tack the "\n" terminator on end of command.
  strncat(eCBuf_as_char_p, "\n", sizeof(*eCBuf_p) - 1);

  Serial.print("Command Buffer: ");
  Serial.println(eCBuf_as_char_p);

  // Rotate, in place, the bytes of the command string.
  char* p = eCBuf_as_char_p;
  for (int i = 0; i < sizeof(*eCBuf_p); i++, p++) {
    if (*p == '\0') {
      break;
    }
    // These shifts emulate a by 2 left rotate.
    *p = (*p << 2 | *p >> 6);
    *p |= 0x02;  // Set "RW" high so this will be an extended command
  }

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(eCBuf_as_char_p);
  int error = Wire.endTransmission();
  if (error != 0) {
    _lastError = E_COMFAIL;
  }

  char* rData = eGetReturnedData(pin, pCmd);

  return (rData);
}

char* SmartLcdIO::eGetReturnedData(char* pin, char* pCmd) {
  if (_lastError != E_OK) {
    return ('\0');
  }

  E_ERRORS errorCode = E_OK;

  if (strncmp(pCmd, "DR", 2) == 0) {
    // Interpret receive buffer as Digital pin read

    errorCode = eReceiveData();

    if (errorCode != E_OK) {
      _lastError = errorCode;
      return ('\0');
    }

    struct digitalReturnData* DD_p;
    DD_p = (digitalReturnData*)eRBuf_p;
    _lastDisplayError = *DD_p->error;

    // Odd display status codes are errors
    if (_lastDisplayError % 2 == 1) {
      _lastError = E_DERRER;
    }
    return (DD_p->level);
  }

  if (strncmp(pCmd, "AR", 2) == 0) {
    // Interpret receive buffer as Analog pin read

    errorCode = eReceiveData();

    if (errorCode != E_OK) {
      _lastError = errorCode;
      return ('\0');
    }

    struct analogReturnData* AD_p;
    AD_p = (analogReturnData*)eRBuf_p;
    
    _lastDisplayError = *AD_p->error;

    // Odd display status codes are errors
    if (_lastDisplayError % 2 == 1) {
      _lastError = E_DERRER;
    }

    return (AD_p->value);
  }

  if ((strncmp(pCmd, "XC", 2) == 0) && (strncmp(pin, "XDS", 3) != 0)) {
    errorCode = eReceiveData();

    if (errorCode != E_OK) {
      _lastError = errorCode;
      return ('\0');
    }

    struct eReturnData* CD_p;
    CD_p = (eReturnData*)eRBuf_p;

    _lastDisplayError = *CD_p->error;

    // Odd display status codes are errors
    if (_lastDisplayError % 2 == 1) {
      _lastError = E_DERRER;
    }

    return (CD_p->vData);
  }

  return ('\0');
}

int SmartLcdIO::eReceiveData() {
  // Received data will be returned in the
  // shared received data buffer.

  E_ERRORS eReturnDataError = E_OK;

  memset(eRBuf_p, 0, sizeof(*eRBuf_p));

  Wire.requestFrom(I2C_ADDRESS, sizeof(*eRBuf_p) - 1);
  char* p = (char*)eRBuf_p;  // Do a char cast of receive buffer
  int rCount = 0;

  while (Wire.available()) {
    char c = Wire.read();
    // Serial.print("--Received Char: ");
    // Serial.print((char) c);
    // Serial.print(" -- ");
    // Serial.println((unsigned char) c, HEX);

    if (rCount < sizeof(*eRBuf_p) - 2) {
      rCount++;
      *p++ = (c == '\n') ? '\0'
                         : c;  // Replace the end token with C string end token
    } else {
      _eReturnDataOverflowCnt++;
      eReturnDataError = E_RBOVERFLOW;
    }
  }  // End of while loop

  if (rCount >= 4) {
    if (strncmp(eRBuf_p->cID, _lastCID, 2) != 0) {
      _badCIDCnt++;
      // Returned data does not match data requested
      eReturnDataError = E_CIDERROR;
    }
  } else {
    _eReturnDataNoDataCnt++;
    // No data was ready for request.
    eReturnDataError = E_NORDATA;
  }

  // Serial.println("*** Receive Buffer ***");
  // Serial.println((char *) eRBuf_p);

  return (eReturnDataError);
}

// The following ate LCD display commands
void SmartLcdIO::ePrint(char* printData) {
  eCommandSend("xxx", "LP", "xx", printData);
}

void SmartLcdIO::eClear(void) { eCommandSend("xxx", "LC", "CL", "\0"); }

void SmartLcdIO::eHome(void) { eCommandSend("xxx", "LC", "HM", "\0"); }

void SmartLcdIO::eSetCursor(byte column, byte row) {
  char clro[8];
  char sBuf[6];

  // Convert column, forcing leading zeros
  itoa(column + 1000, sBuf, 10);
  strncpy(&clro[0], sBuf + 1, 4);

  // Convert row, forcing leading zeros
  itoa(row + 1000, sBuf, 10);
  strncpy(&clro[3], sBuf + 1, 4);

  eCommandSend("xxx", "LC", "SC", clro);
}

void SmartLcdIO::eCursor(void) { eCommandSend("xxx", "LC", "CO", "\0"); }

void SmartLcdIO::eNoCursor(void) { eCommandSend("xxx", "LC", "CN", "\0"); }

void SmartLcdIO::eBlink(void) { eCommandSend("xxx", "LC", "BO", "\0"); }

void SmartLcdIO::eNoBlink(void) { eCommandSend("xxx", "LC", "BN", "\0"); }

void SmartLcdIO::eDisplay(void) { eCommandSend("xxx", "LC", "DO", "\0"); }

void SmartLcdIO::eNoDisplay(void) { eCommandSend("xxx", "LC", "DN", "\0"); }

void SmartLcdIO::eScrollDisplayLeft(void) {
  eCommandSend("xxx", "LC", "SL", "\0");
}

void SmartLcdIO::eScrollDisplayRight(void) {
  eCommandSend("xxx", "LC", "SR", "\0");
}

void SmartLcdIO::eAutoscroll(void) { eCommandSend("xxx", "LC", "AS", "\0"); }

void SmartLcdIO::eNoAutoscroll(void) { eCommandSend("xxx", "LC", "AN", "\0"); }

void SmartLcdIO::eLeftToRight(void) { eCommandSend("xxx", "LC", "LR", "\0"); }

void SmartLcdIO::eRightToLeft(void) { eCommandSend("xxx", "LC", "RL", "\0"); }
