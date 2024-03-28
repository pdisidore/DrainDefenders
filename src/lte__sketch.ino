/*  This code is apart of the drain defenders project. The main funciton of this code is to use 
  the adafruit dotletics SIM 7000 board to cellularly connect to our MQTT server. This code reads
  the turbidity from pin 24 and then out puts via the cerial port, dweet, and mqtt connection. 

    Note: this code is specifically meant for AVR microcontrollers (Arduino Uno, Mega, Leonardo, etc)
    However, if you are using an ESP8266 please make the minor modifications mentioned below in the
    comments for the pin definitions and software serial initialization.

    Author: Phillip Disidore
    Last Updated: 3/28/2024
*/

#include "BotleticsSIM7000.h"  // https://github.com/botletics/Botletics-SIM7000/tree/main/src
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/******* ORIGINAL ADAFRUIT FONA LIBRARY TEXT *******/
/***************************************************
  This is an example for our Adafruit FONA Cellular Module

  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542

  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#define SIMCOM_7000

// For botletics SIM7000 shield
//#define DTR 8 // Connect with solder jumper
//#define RI 9 // Need to enable via AT commands
//#define T_ALERT 12 // Connect with solder jumper
#define PWRKEY 6
#define RST 7
#define TX 0  // Microcontroller RX
#define RX 1  // Microcontroller TX
#define MQTT_SERVER "144.126.217.239"
#define MQTT_SERVERPORT 1883
#define MQTT_USERNAME ""
#define MQTT_KEY ""
#define TURB_TOPIC "/Turbidity"

// this is a large buffer for replies
char replybuffer[255];
unsigned long lastMsg = 0;
const int filter_length = 32;

uint16_t VbatBuffer[filter_length];
uint32_t VbatFor32Readings = 0;
unsigned int i = 0;

unsigned long AvgTurbArr[filter_length];
unsigned long TurbSum = 0;

int right = 0;

// Hardware serial is also possible!
HardwareSerial *modemSerial = &Serial1;

Botletics_modem_LTE modem = Botletics_modem_LTE();

void ConnectToMQTT();
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = { 0 };  // MUST use a 16 character buffer for IMEI!

void setup() {

  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);  // Default state

  pinMode(PWRKEY, OUTPUT);

  // Turn on the module by pulsing PWRKEY low for a little bit
  // This amount of time depends on the specific module that's used
  modem.powerOn(PWRKEY);  // Power on the module

  Serial.begin(9600);
  Serial.println(F("Modem basic test"));
  Serial.println(F("Initializing....(May take several seconds)"));

  // Hardware serial:

  modemSerial->begin(115200);  // Default SIM7000 baud rate

  if (!modem.begin(*modemSerial)) {
    DEBUG_PRINTLN(F("Couldn't find SIM7000"));
  }

  type = modem.type();
  Serial.println(F("Modem is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case SIM7000:  //The one we have
      Serial.println(F("SIM7000"));
      break;
    default:
      Serial.println(F("SIM is no longer SIM7000?"));
      break;
  }

  // Print module IMEI number.
  uint8_t imeiLen = modem.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: ");
    Serial.println(imei);
  }

  // Set modem to full functionality
  modem.setFunctionality(1);                // AT+CFUN=1
  modem.setNetworkSettings(F("hologram"));  // For Hologram SIM card

  if (!modem.enableGPRS(true))
    Serial.println(F("Failed to turn on"));

  ////ADDED SINCE PUSH
  // Open wireless connection if not already activated
  if (!modem.wirelessConnStatus()) {
    while (!modem.openWirelessConnection(true)) {
      Serial.println(F("Failed to enable connection, retrying..."));
      delay(2000);  // Retry every 2s
    }
    Serial.println(F("Enabled data!"));

    /*******************mqtt connection********************/
    //https://github.com/botletics/SIM7000-LTE-Shield/wiki/Library-Functions#modemmqttconnect
    ConnectToMQTT();
    //LAST OF ADDED SINCE PUSH
  }
}

void loop() {
  char command;
  //testing automating commands
  //every 5 seconds, print battery
  unsigned long now = millis();

  uint64_t timeInMilli = (now - lastMsg + 0xFFFF) % 0xFFFF;
  if (timeInMilli >= 1000) {
    uint16_t vbat;

    lastMsg = now;
    if (i < filter_length) {  //could update to moving average instead of constant averaging

      AvgTurbArr[i] = analogRead(24);
      TurbSum += analogRead(24);
      i++;
    } else {

      ConnectToMQTT();

      uint16_t turbLevel = TurbSum >> 5;

      Serial.print(F("Turb = "));
      Serial.println(turbLevel);

      // Post data to website via 2G or LTE CAT-M/NB-IoT
      //float turb = 730 - analogRead(24);
      float turb = analogRead(24);

      // Create char buffers for the floating point numbers for sprintf
      // Make sure these buffers are long enough for your request URL
      char URL[150];
      char body[100];
      char turbSendBuff[16];

      // Format the floating point numbers as needed
      dtostrf(turb, 1, 2, turbSendBuff);  // float_val, min_width, digits_after_decimal, char_buffer
      sprintf(URL, "dweet.io/dweet/for/%s?turb=%s&batt", imei, turbSendBuff);

      if (!modem.postData("GET", URL)) {
        Serial.println(F("Failed to complete HTTP GET..."));
      } else {
        Serial.println(F("Complete HTTP GET..."));
      }

      if (!modem.MQTT_publish(TURB_TOPIC, turbSendBuff, strlen(turbSendBuff), 1, 0)) {
        Serial.println(F("Failed to publish!"));
      }  // Send Turbidity
      else {
        Serial.println(F("Published data!"));
      }

      VbatFor32Readings = 0;
      TurbSum = 0;
      i = 0;
    }
  }
}

void ConnectToMQTT() {
  while (!modem.MQTT_connectionStatus()) {
    // Set up MQTT parameters (see MQTT app note for explanation of parameter values)
    modem.MQTT_setParameter("URL", MQTT_SERVER, MQTT_SERVERPORT);
    // Set up MQTT username and password if necessary
    // modem.MQTT_setParameter("USERNAME", MQTT_USERNAME);
    // modem.MQTT_setParameter("PASSWORD", MQTT_PASSWORD);
    //    modem.MQTTsetParameter("KEEPTIME", 30); // Time to connect to server, 60s by default

    Serial.print(F("Connecting to MQTT broker...    "));
    if (!modem.MQTT_connect(true)) {
      Serial.println(F("Failed to connect to broker!"));
    } else Serial.println(F("Connected to MQTT broker!"));
  }
}

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

char readBlocking() {
  while (!Serial.available())
    ;
  return Serial.read();
}
uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (!isdigit(c = readBlocking())) {
    //Serial.print(c);
  }
  Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    Serial.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //Serial.println(F("SPACE"));
      break;
    }

    while (Serial.available()) {
      char c = Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)  // the first 0x0A is ignored
          continue;

        timeout = 0;  // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}
