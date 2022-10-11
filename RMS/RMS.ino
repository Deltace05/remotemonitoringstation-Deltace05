#include "sensitiveInformation.h"
#define FORMAT_SPIFFS_IF_FAILED true

// Wifi & Webserver
#include "WiFi.h"
#include "SPIFFS.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

//Temperature & TFT Screen
#include <Wire.h>
#include "Adafruit_ADT7410.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include "Adafruit_miniTFTWing.h"
#include <Adafruit_MotorShield.h>

// ESP32Servo Start
#include <ESP32Servo.h>
Servo myservo;  // create servo object to control a servo
int servoPin = 12;
boolean blindsOpen = false;
// ESP32Servo End

//RTC
#include "RTClib.h"
AsyncWebServer server(80);
Adafruit_miniTFTWing ss;
#define TFT_RST    -1    // we use the seesaw for resetting to save a pin
#define TFT_CS   14
#define TFT_DC   32

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Create the ADT7410 temperature sensor object
Adafruit_ADT7410 tempsensor = Adafruit_ADT7410();

RTC_PCF8523 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

boolean LEDOn = false; // State of Built-in LED true=on, false=off.

#define LOOPDELAY 100
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *myMotor = AFMS.getMotor(4);

#define LEDRed 27
#define LEDGreen 33
// RFID Start

#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN  21  // ES32 Feather
#define RST_PIN 17 // esp32 Feather - SCL pin. Could be others.

MFRC522 rfid(SS_PIN, RST_PIN);
bool safeLocked = true;
int ledState = LOW;             // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 10000;           // interval at which to blink (milliseconds)


// RFID End

void setup() {
  Serial.begin(9600);
#ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
#endif
  while (!Serial) {
    delay(10);
  }
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x49) for example
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find ADT7410!");
    while (1);
  }

  if (!ss.begin()) {
    Serial.println("seesaw init error!");
    while (1);
  }
  else Serial.println("seesaw started");

  ss.tftReset();
  ss.setBacklight(0x0); //set the backlight fully on

  // Use this initializer (uncomment) if you're using a 0.96" 180x60 TFT
  tft.initR(INITR_MINI160x80);   // initialize a ST7735S chip, mini display

  tft.setRotation(3);

  tft.fillScreen(ST77XX_BLACK);

  // sensor takes 250 ms to get first readings
  delay(250);

  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    // Follow instructions in README and install
    // https://github.com/me-no-dev/arduino-esp32fs-plugin
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  //Wifi Configuration
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    tftDrawText("Connecting to WiFi..", ST77XX_RED);
  }
  tft.fillScreen(ST77XX_BLACK);

  Serial.println();
  Serial.print("Connected to the Internet");
  Serial.print("IP address: ");
  String ip = WiFi.localIP().toString();
  Serial.println(ip);

  // Display IP on TFT
  tft.setCursor(0, 60);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextWrap(true);
  tft.print(ip);

  routesConfiguration(); // Reads routes from routesManagement
  server.begin();

  // RTC
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    //    abort();
  }

  rtc.start();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  float drift = 43; // seconds plus or minus over oservation period - set to 0 to cancel previous calibration.
  float period_sec = (7 * 86400);  // total obsevation period in seconds (86400 = seconds in 1 day:  7 days = (7 * 86400) seconds )
  float deviation_ppm = (drift / period_sec * 1000000); //  deviation in parts per million (μs)
  float drift_unit = 4.34; // use with offset mode PCF8523_TwoHours
  // float drift_unit = 4.069; //For corrections every min the drift_unit is 4.069 ppm (use with offset mode PCF8523_OneMinute)
  int offset = round(deviation_ppm / drift_unit);
  // rtc.calibrate(PCF8523_TwoHours, offset); // Un-comment to perform calibration once drift (seconds) and observation period (seconds) are correct
  // rtc.calibrate(PCF8523_TwoHours, 0); // Un-comment to cancel previous calibration
  Serial.print("Offset is "); Serial.println(offset); // Print to control offset
  // The following line can be uncommented if the time needs to be reset.
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  pinMode(LED_BUILTIN, OUTPUT);

  AFMS.begin();  // create with the default frequency 1.6KHz

  // ESP32Servo Start
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 12 to the servo object
  // ESP32Servo End

  // RFID Start
  SPI.begin(); // init SPI bus
  rfid.PCD_Init(); // init MFRC522
  // RFID End
  pinMode(LEDRed, OUTPUT);
  pinMode(LEDGreen, OUTPUT);
  digitalWrite(LEDRed, LOW);
  digitalWrite(LEDGreen, LOW);

}

void loop() {

  builtinLED();
  updateTemperature();
  //adaLoggerRTC();
  automaticFan(30.00);
  windowBlinds();
  readRFID();
  safeStatusDisplay();
  delay(LOOPDELAY); // To allow time to publish new code.
}

//built in LED function, allows for the LED on the ESP32 board to be turned on and off by the website
void builtinLED() {
  if (LEDOn) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

//Takes temperature from ADT7410 temperature sensor and outputs it as a string
void updateTemperature() {
  // Read and print out the temperature, then convert to *F
  float c = tempsensor.readTempC();
  //Serial.print("Temp: "); Serial.print(c); Serial.print("*C\t");
  String tempInC = String(c);
  tftDrawText(tempInC, ST77XX_BLUE);
  //delay(1000);
}

//Checks temperature from temperature variable and if temperature passes a certain threshold will turn on the DC motor (fan)
void automaticFan(float temperatureThreshold) {
  float c = tempsensor.readTempC();
  myMotor->setSpeed(100);
  if (c < temperatureThreshold) {
    myMotor->run(RELEASE);
  } else {
    myMotor->run(FORWARD);
    logEvent("Fan Started");
  }
}

//Prints current temperature to the top of the TFT screen live.
void tftDrawText(String text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextSize(3);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setTextWrap(true);
  tft.print(text);
}

//If the A button on the TFT wing is pressed, will rotate a servo from 0 to 180 degrees and back, simulates opening a blind
void windowBlinds() {
  uint32_t buttons = ss.readButtons();
  if (! (buttons & TFTWING_BUTTON_A)) {
    if (blindsOpen) {
      myservo.write(0);
      logEvent("Blinds Activated");
    } else {
      myservo.write(180);
      logEvent("Blinds Activated");
    }
    blindsOpen = !blindsOpen;
  }
}


//Reads the ID of the tag presented to the scanner, and if it is the correct ID number, will unlock the safe, if the tag is incorrect, will lock the safe
void readRFID() {

  String uidOfCardRead = "";
  String validCardUID = "76 012 54 73";

  if (rfid.PICC_IsNewCardPresent()) { // new tag is available
    if (rfid.PICC_ReadCardSerial()) { // NUID has been readed
      MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
      for (int i = 0; i < rfid.uid.size; i++) {
        uidOfCardRead += rfid.uid.uidByte[i] < 0x10 ? " 0" : " ";
        uidOfCardRead += rfid.uid.uidByte[i];
      }
      //Serial.println(uidOfCardRead);

      rfid.PICC_HaltA(); // halt PICC
      rfid.PCD_StopCrypto1(); // stop encryption on PCD
      uidOfCardRead.trim();
      if (uidOfCardRead == validCardUID) {
        safeLocked = false;
        logEvent("Safe Unlocked");
      } else {
        safeLocked = true;
        logEvent("Safe Locked");
      }
    }
  }
}

//if the red LED is on, safe is locked
//if green LED is on, the safe is unlocked
//This serves to show if the safe is unlocked as without it it is impossible to tell what state the safe is in.
void safeStatusDisplay() {
  /*
     Outputs the status of the Safe Lock to the LEDS
     Red LED = Locked
     Green LED = Unlocked.
  */
  if (safeLocked) {
    digitalWrite(LEDRed, HIGH);
    digitalWrite(LEDGreen, LOW);
  } else {
    digitalWrite(LEDRed, LOW);
    digitalWrite(LEDGreen, HIGH);
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      safeLocked = true;
      logEvent("Safe Locked");
    }
  }
}

//not in use, simply prints date and time to serial monitor.
void adaLoggerRTC () {
  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  //delay(3000);
}

//If function is called, will log the data inside the brackets to the SPIFFS partition
void logEvent(String dataToLog) {
  /*
     Log entries to a file stored in SPIFFS partition on the ESP32.
  */
  // Get the updated/current time
  DateTime rightNow = rtc.now();
  char csvReadableDate[25];
  sprintf(csvReadableDate, "%02d,%02d,%02d,%02d,%02d,%02d,",  rightNow.year(), rightNow.month(), rightNow.day(), rightNow.hour(), rightNow.minute(), rightNow.second());

  String logTemp = csvReadableDate + dataToLog + "\n"; // Add the data to log onto the end of the date/time

  const char * logEntry = logTemp.c_str(); //convert the logtemp to a char * variable

  //Add the log entry to the end of logevents.csv
  appendFile(SPIFFS, "/logEvents.csv", logEntry);

  // Output the logEvents - FOR DEBUG ONLY. Comment out to avoid spamming the serial monitor.
  //  readFile(SPIFFS, "/logEvents.csv");

  Serial.print("\nEvent Logged: ");
  Serial.println(logEntry);
}
