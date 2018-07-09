#include <LiquidCrystal_PCF8574.h>
#include <SmartLCDandIOexpander_I2C.h>

#define I2C_ADDRESS 0x27

#define LCD_COLUMNS 16
#define LCD_ROWS 2

//#define LCD_COLUMNS 20
//#define LCD_ROWS    4

LiquidCrystal_PCF8574 lcd(I2C_ADDRESS);
SmartLcdIO eLcd(I2C_ADDRESS);

void setup() {
  int error;

  Serial.begin(9600);

  while (!Serial);

  Serial.println("I2C LCD Display and I/O check starting");

  Wire.begin();
  Wire.beginTransmission(I2C_ADDRESS);
  error = Wire.endTransmission();
  Serial.print("Error: ");
  Serial.print(error);

  if (error == 0) {
    Serial.println(": LCD found.");

  } else {
    Serial.println(": LCD not found.");
  }

  lcd.begin(LCD_COLUMNS, LCD_ROWS);
  
  lcd.setBacklight(255);
  lcd.print("Smart LCD Test");
  lcd.setCursor(0, 1);
  lcd.print("V 0.1.1");
  
  // Send debug messages to serial console
  eLcd.eSerialDebugEnable(true);
  
  eLcd.ePinMode("D11", "OT");
  eLcd.ePinMode("D12", "IP");

  eLcd.eAnaloglStart("A03", "01", 0);
}

void loop() {
  delay(2000);
  Serial.println("Looping---");
  lcd.begin(LCD_COLUMNS, LCD_ROWS);

  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();

  lcd.print("* LCD PCF8574 mode");
  delay(1000);
  lcd.clear();
  lcd.print("Print using I2C");
  lcd.setCursor(0, 1);
  lcd.print("standard library");
  delay(2000);

  lcd.clear();
  lcd.print("Using Extended");
  lcd.setCursor(0, 1);
  lcd.print("IO commands");
  delay(2000);

  lcd.clear();
  lcd.print("Chip Temp");
  Serial.println("Displaying Chip Temp");
  delay(1000);

  // The temperature can change
  // between the two readings
  lcd.clear();
  eLcd.eCustomCommand("XDS", "TC", "");
  lcd.print(" Centigrade");
  lcd.setCursor(0, 1);
  eLcd.eCustomCommand("XDS", "TF", "");
  lcd.print(" Fahrenheit");
  delay(1000);

  char* TF = eLcd.eCustomCommand("xxx", "TF", "");
  Serial.print("Reading extended command: TF: ");
  Serial.println(TF);
  Serial.println("------");

  float av = eLcd.eAnalogRead("A03");
  Serial.print("Read Analog: ");
  Serial.println(av);

  lcd.clear();
  lcd.print("Analog Pin A03");
  lcd.setCursor(0, 1);
  lcd.print(av);
  lcd.print(" Volts");
  delay(1000);

  Serial.println("Write pin D11 and D13: ");
  eLcd.eDigitalWrite("D13", 1);
  eLcd.eDigitalWrite("D11", 0);
  bool level = eLcd.eDigitalRead("D13");
  Serial.print("Read pin D13: ");
  Serial.println(level);
  delay(500);

  eLcd.eDigitalWrite("D13", 0);
  eLcd.eDigitalWrite("D11", 1);
  level = eLcd.eDigitalRead("D13");
  Serial.print("Read pin D13: ");
  Serial.println(level);

  level = eLcd.eDigitalRead("D12");
  Serial.print("Read pin D12: ");
  Serial.println(level);
  lcd.clear();
  lcd.print("Digital Pin D12");
  lcd.setCursor(0, 1);
  lcd.print(level);
  delay(1000);

  eLcd.eClear();
  eLcd.ePrint("LCD Extended mode");
  delay(1000);
  eLcd.eClear();
  eLcd.ePrint("Print using");
  eLcd.eSetCursor(0, 1);
  eLcd.ePrint("Extended commands");
  delay(2000);

  eLcd.eClear();
  eLcd.ePrint("Display Off");
  delay(1000);
  eLcd.eNoDisplay();
  delay(1000);

  eLcd.eDisplay();
  eLcd.eClear();
  eLcd.ePrint("Display On");
  delay(1000);

  eLcd.eClear();
  eLcd.ePrint("Backlight Off");
  delay(1000);
  eLcd.eDisplayLedOnOff(false);
  delay(1000);

  eLcd.eClear();
  eLcd.ePrint("Backlight On");
  eLcd.eDisplayLedOnOff(true);
  delay(1000);

  eLcd.eClear();
  eLcd.ePrint("Dimming Display");
  Serial.println("Dimming Display *");

  float brightness;

  for (int i = 100; i >= 0; i -= 2) {
    brightness = i / 100.0;
    eLcd.eDisplayBrightness(brightness);
    delay(50);
  }
  for (int i = 0; i <= 100; i += 5) {
    brightness = i / 100.0;
    eLcd.eDisplayBrightness(brightness);
    delay(10);
  }
  delay(1000);
}
