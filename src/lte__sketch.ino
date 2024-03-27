/*  This is an example sketch to test the core functionalities of SIMCom-based cellular modules.
    This code supports the SIM7000-series modules (LTE CAT-M/NB-IoT shields) for low-power IoT devices!

    Note: this code is specifically meant for AVR microcontrollers (Arduino Uno, Mega, Leonardo, etc)
    However, if you are using an ESP8266 please make the minor modifications mentioned below in the
    comments for the pin definitions and software serial initialization.

    For ESP32 please use the ESP32_LTE_Demo instead: https://github.com/botletics/SIM7000-LTE-Shield/blob/master/Code/examples/ESP32_LTE_Demo/ESP32_LTE_Demo.ino

    Author: Timothy Woo (www.botletics.com)
    Github: https://github.com/botletics/SIM7000-LTE-Shield
    Last Updated: 11/22/2022
    License: GNU GPL v3.0
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

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = { 0 };  // MUST use a 16 character buffer for IMEI!

void printMenu(void) {
  Serial.println(F("-------------------------------------"));
  // General
  Serial.println(F("[?] Print this menu"));
  Serial.println(F("[a] Read the ADC; 2.8V max for SIM800/808, 0V-VBAT for SIM7000 shield"));
  Serial.println(F("[b] Read supply voltage"));  // Will also give battery % charged for most modules
  Serial.println(F("[C] Read the SIM CCID"));
  Serial.println(F("[U] Unlock SIM with PIN code"));
  Serial.println(F("[i] Read signal strength (RSSI)"));
  Serial.println(F("[n] Get network status"));
  Serial.println(F("[1] Get network connection info"));  // See what connection type and band you're on!

  // Time
  Serial.println(F("[y] Enable local time stamp (SIM800/808/70X0)"));
  Serial.println(F("[Y] Enable NTP time sync (SIM800/808/70X0)"));  // Need to use "G" command first!
  Serial.println(F("[t] Get network time"));                        // Works just by being connected to network

  // Data Connection
  Serial.println(F("[G] Enable cellular data"));
  Serial.println(F("[g] Disable cellular data"));
  Serial.println(F("[l] Query GSMLOC (2G)"));
#if !defined(SIMCOM_3G) && !defined(SIMCOM_7500) && !defined(SIMCOM_7600)
  Serial.println(F("[w] Read webpage"));
  Serial.println(F("[W] Post to website"));
#endif
  // The following option below posts dummy data to dweet.io for demonstration purposes. See the
  // IoT_example sketch for an actual application of this function!
  Serial.println(F("[2] Post to dweet.io - 2G / LTE CAT-M / NB-IoT"));  // SIM800/808/900/7000/7070
  Serial.println(F("[3] Post to dweet.io - 3G / 4G LTE"));              // SIM5320/7500/7600

  // GPS
  if (type >= SIM808_V1) {
    Serial.println(F("[O] Turn GPS on (SIM808/5320/7XX0)"));
    Serial.println(F("[o] Turn GPS off (SIM808/5320/7XX0)"));
    Serial.println(F("[L] Query GPS location (SIM808/5320/7XX0)"));
    if (type == SIM808_V1) {
      Serial.println(F("[x] GPS fix status (SIM808 v1 only)"));
    }
    Serial.println(F("[E] Raw NMEA out (SIM808)"));
  }

  Serial.println(F("[S] Create serial passthru tunnel"));
  Serial.println(F("-------------------------------------"));
  Serial.println(F(""));
}
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

  //Didn't compile
  // while (!netStatus()) {
  //   Serial.println(F("Failed to connect to cell network, retrying..."));
  //   delay(2000);  // Retry every 2s
  // }
  // Serial.println(F("Connected to cell network!"));

  // Disable data just to make sure it was actually off so that we can turn it on
  //  modem.openWirelessConnection(false);

  // Open wireless connection if not already activated
  if (!modem.wirelessConnStatus()) {
    while (!modem.openWirelessConnection(true)) {
      Serial.println(F("Failed to enable connection, retrying..."));
      delay(2000);  // Retry every 2s
    }
    Serial.println(F("Enabled data!"));

    /*******************mqtt connection********************/
    // MQTTconnect(const char *protocol, const char *clientID, const char *username, const char *password)
    // Description
    // Send the connect packet to the MQTT broker.
    // Note: Must have data enabled and must connect to the server via TCP/UDP before using this command.

    // Syntax
    // modem.MQTTconnect(protocol, clientID, username, password)

    // Parameters
    // protocol: string defining the MQTT protocol. Ex: "MQTT" or "MQIsdp" (for CloudMQTT specifically)
    // clientID: string containing the client ID. This can be the device's IMEI number, custom name, etc.
    // username: string containing the username
    // password: string containing the password
    // if (!modem.TCPconnect(MQTT_SERVER, MQTT_SERVERPORT)) Serial.println(F("Failed to connect to TCP/IP!"));
    // // CloudMQTT requires "MQIsdp" instead of "MQTT"
    // if (!modem.MQTTconnect("MQTT", MQTT_CLIENT, MQTT_USERNAME, MQTT_KEY)) Serial.println(F("Failed to connect to MQTT broker!"));

    //https://github.com/botletics/SIM7000-LTE-Shield/wiki/Library-Functions#modemmqttconnect
    //Attempt 2 at connection
    // while (!modem.MQTT_connectionStatus()) {
    //   Serial.print("Attempting MQTT connection...");
    //   // Create a random client ID
    //   String clientId = "ESP8266Client-";
    //   clientId += String(random(0xffff), HEX);
    //   if (!modem.TCPconnect(MQTT_SERVER, MQTT_SERVERPORT)) Serial.println(F("Failed to connect to TCP/IP!"));
    //   if (!modem.MQTTconnect("MQTT", MQTT_CLIENT, MQTT_USERNAME, MQTT_KEY)) Serial.println(F("Failed to connect to MQTT broker!"));

    //   delay(2000);
    // }
    // Serial.println(F("Connected to TCP/IP!"));
    // Serial.println(F("Connected to MQTT broker!"));

    // Attempt 1
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
      }
    }
    Serial.println(F("Connected to MQTT broker!"));

    //LAST OF ADDED SINCE PUSH
    printMenu();
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
        }
        else Serial.println(F("Connected to MQTT broker!"));
      }

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

// command = Serial.read();
// Serial.println(command);


//   switch (command) {
//     case '?':
//       {
//         printMenu();
//         break;
//       }

//     case 'a':
//       {
//         // read the ADC
//         uint16_t adc;
//         if (!modem.getADCVoltage(&adc)) {
//           Serial.println(F("Failed to read ADC"));
//         } else {
//           Serial.print(F("ADC = "));
//           Serial.print(adc);
//           Serial.println(F(" mV"));
//         }
//         break;
//       }

//     case 'b':
//       {
//         // read the battery voltage and percentage
//         uint16_t vbat;
//         if (!modem.getBattVoltage(&vbat)) {
//           Serial.println(F("Failed to read Batt"));
//         } else {
//           Serial.print(F("VBat = "));
//           Serial.print(vbat);
//           Serial.println(F(" mV"));
//         }

//         if (type != SIM7500 && type != SIM7600) {
//           if (!modem.getBattPercent(&vbat)) {
//             Serial.println(F("Failed to read Batt"));
//           } else {
//             Serial.print(F("VPct = "));
//             Serial.print(vbat);
//             Serial.println(F("%"));
//           }
//         }

//         break;
//       }

//     case 'U':
//       {
//         // Unlock the SIM with a PIN code
//         char PIN[5];
//         flushSerial();
//         Serial.println(F("Enter 4-digit PIN"));
//         readline(PIN, 3);
//         Serial.println(PIN);
//         Serial.print(F("Unlocking SIM card: "));
//         if (!modem.unlockSIM(PIN)) {
//           Serial.println(F("Failed"));
//         } else {
//           Serial.println(F("OK!"));
//         }
//         break;
//       }

//     case 'C':
//       {
//         // read the CCID
//         modem.getSIMCCID(replybuffer);  // make sure replybuffer is at least 21 bytes!
//         Serial.print(F("SIM CCID = "));
//         Serial.println(replybuffer);
//         break;
//       }

//     case 'i':
//       {
//         // read the RSSI
//         uint8_t n = modem.getRSSI();
//         int8_t r;

//         Serial.print(F("RSSI = "));
//         Serial.print(n);
//         Serial.print(": ");
//         if (n == 0) r = -115;
//         if (n == 1) r = -111;
//         if (n == 31) r = -52;
//         if ((n >= 2) && (n <= 30)) {
//           r = map(n, 2, 30, -110, -54);
//         }
//         Serial.print(r);
//         Serial.println(F(" dBm"));

//         break;
//       }

//     case 'n':
//       {
//         // read the network/cellular status
//         uint8_t n = modem.getNetworkStatus();
//         Serial.print(F("Network status "));
//         Serial.print(n);
//         Serial.print(F(": "));
//         if (n == 0) Serial.println(F("Not registered"));
//         if (n == 1) Serial.println(F("Registered (home)"));
//         if (n == 2) Serial.println(F("Not registered (searching)"));
//         if (n == 3) Serial.println(F("Denied"));
//         if (n == 4) Serial.println(F("Unknown"));
//         if (n == 5) Serial.println(F("Registered roaming"));
//         break;
//       }
//     case '1':
//       {
//         // Get connection type, cellular band, carrier name, etc.
//         modem.getNetworkInfo();
//         break;
//       }

//       /*** Time ***/

//     case 'y':
//       {
//         // enable network time sync
//         if (!modem.enableRTC(true))
//           Serial.println(F("Failed to enable"));
//         break;
//       }

//     case 'Y':
//       {
//         // enable NTP time sync
//         if (!modem.enableNTPTimeSync(true, F("pool.ntp.org")))
//           Serial.println(F("Failed to enable"));
//         break;
//       }

//     case 't':
//       {
//         // read the time
//         char buffer[23];

//         modem.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
//         Serial.print(F("Time = "));
//         Serial.println(buffer);
//         break;
//       }


//       /*********************************** GPS */

//     case 'o':
//       {
//         // turn GPS off
//         if (!modem.enableGPS(false))
//           Serial.println(F("Failed to turn off"));
//         break;
//       }
//     case 'O':
//       {
//         // turn GPS on
//         if (!modem.enableGPS(true))
//           Serial.println(F("Failed to turn on"));
//         break;
//       }
//     case 'x':
//       {
//         int8_t stat;
//         // check GPS fix
//         stat = modem.GPSstatus();
//         if (stat < 0)
//           Serial.println(F("Failed to query"));
//         if (stat == 0) Serial.println(F("GPS off"));
//         if (stat == 1) Serial.println(F("No fix"));
//         if (stat == 2) Serial.println(F("2D fix"));
//         if (stat == 3) Serial.println(F("3D fix"));
//         break;
//       }

//     case 'L':
//       {
//         /*
//         // Uncomment this block if all you want to see is the AT command response
//         // check for GPS location
//         char gpsdata[120];
//         modem.getGPS(0, gpsdata, 120);
//         if (type == SIM808_V1)
//           Serial.println(F("Reply in format: mode,longitude,latitude,altitude,utctime(yyyymmddHHMMSS),ttff,satellites,speed,course"));
//         else if ( (type == SIM5320A) || (type == SIM5320E) || (type == SIM7500) || (type == SIM7600) )
//           Serial.println(F("Reply in format: [<lat>],[<N/S>],[<lon>],[<E/W>],[<date>],[<UTC time>(yyyymmddHHMMSS)],[<alt>],[<speed>],[<course>]"));
//         else
//           Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));

//         Serial.println(gpsdata);

//         break;
//         */

//         float latitude, longitude, speed_kph, heading, altitude;

//         // Use the top line if you want to parse UTC time data as well, the line below it if you don't care
//         //        if (modem.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude, &year, &month, &day, &hour, &minute, &second)) {
//         if (modem.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {  // Use this line instead if you don't want UTC time
//           Serial.println(F("---------------------"));
//           Serial.print(F("Latitude: "));
//           Serial.println(latitude, 6);
//           Serial.print(F("Longitude: "));
//           Serial.println(longitude, 6);
//           Serial.print(F("Speed: "));
//           Serial.println(speed_kph);
//           Serial.print(F("Heading: "));
//           Serial.println(heading);
//           Serial.print(F("Altitude: "));
//           Serial.println(altitude);
//           // Comment out the stuff below if you don't care about UTC time
//           /*
//           Serial.print(F("Year: ")); Serial.println(year);
//           Serial.print(F("Month: ")); Serial.println(month);
//           Serial.print(F("Day: ")); Serial.println(day);
//           Serial.print(F("Hour: ")); Serial.println(hour);
//           Serial.print(F("Minute: ")); Serial.println(minute);
//           Serial.print(F("Second: ")); Serial.println(second);
//           Serial.println(F("---------------------"));
//           */
//         }

//         break;
//       }

//     case 'E':
//       {
//         flushSerial();
//         if (type == SIM808_V1) {
//           Serial.print(F("GPS NMEA output sentences (0 = off, 34 = RMC+GGA, 255 = all)"));
//         } else {
//           Serial.print(F("On (1) or Off (0)? "));
//         }
//         uint8_t nmeaout = readnumber();

//         // turn on NMEA output
//         modem.enableGPSNMEA(nmeaout);

//         break;
//       }

//       /*********************************** GPRS */

//     case 'g':
//       {
//         // disable data
//         if (!modem.enableGPRS(false))
//           Serial.println(F("Failed to turn off"));
//         break;
//       }
//     case 'G':
//       {
// // turn GPRS off first for SIM7500
// #if defined(SIMCOM_7500) || defined(SIMCOM_7600)
//         modem.enableGPRS(false);
// #endif

//         // enable data
//         if (!modem.enableGPRS(true))
//           Serial.println(F("Failed to turn on"));
//         break;
//       }
//     case 'l':
//       {
//         // check for GSMLOC (requires GPRS)
//         uint16_t returncode;

//         if (!modem.getGSMLoc(&returncode, replybuffer, 250))
//           Serial.println(F("Failed!"));
//         if (returncode == 0) {
//           Serial.println(replybuffer);
//         } else {
//           Serial.print(F("Fail code #"));
//           Serial.println(returncode);
//         }

//         break;
//       }

// #if !defined(SIMCOM_3G) && !defined(SIMCOM_7500) && !defined(SIMCOM_7600)
//     // The code below was written by Adafruit and only works on some modules
//     case 'w':
//       {
//         // read website URL
//         uint16_t statuscode;
//         int16_t length;
//         char url[80];

//         flushSerial();
//         Serial.println(F("URL to read (e.g. dweet.io/get/latest/dweet/for/sim7500test123):"));
//         Serial.print(F("http://"));
//         readline(url, 79);
//         Serial.println(url);

//         Serial.println(F("****"));
//         if (!modem.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) {
//           Serial.println("Failed!");
//           break;
//         }
//         while (length > 0) {
//           while (modem.available()) {
//             char c = modem.read();

//             // Serial.write is too slow, we'll write directly to Serial register!
// #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//             loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
//             UDR0 = c;
// #else
//             Serial.write(c);
// #endif
//             length--;
//             if (!length) break;
//           }
//         }
//         Serial.println(F("\n****"));
//         modem.HTTP_GET_end();
//         break;
//       }

//     case 'W':
//       {
//         // Post data to website
//         uint16_t statuscode;
//         int16_t length;
//         char url[80];
//         char data[80];

//         flushSerial();
//         Serial.println(F("NOTE: in beta! Use simple websites to post!"));
//         Serial.println(F("URL to post (e.g. httpbin.org/post):"));
//         Serial.print(F("http://"));
//         readline(url, 79);
//         Serial.println(url);
//         Serial.println(F("Data to post (e.g. \"foo\" or \"{\"simple\":\"json\"}\"):"));
//         readline(data, 79);
//         Serial.println(data);

//         Serial.println(F("****"));
//         if (!modem.HTTP_POST_start(url, F("text/plain"), (uint8_t *)data, strlen(data), &statuscode, (uint16_t *)&length)) {
//           Serial.println("Failed!");
//           break;
//         }
//         while (length > 0) {
//           while (modem.available()) {
//             char c = modem.read();

// #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//             loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
//             UDR0 = c;
// #else
//             Serial.write(c);
// #endif

//             length--;
//             if (!length) break;
//           }
//         }
//         Serial.println(F("\n****"));
//         modem.HTTP_POST_end();
//         break;
//       }
// #endif

// #if defined(SIMCOM_2G) || defined(SIMCOM_7000) || defined(SIMCOM_7070)
//     case '2':
//       {
//         // Post data to website via 2G or LTE CAT-M/NB-IoT

//         float temperature = analogRead(A0) * 1.23;  // Change this to suit your needs

//         uint16_t battLevel;
//         if (!modem.getBattVoltage(&battLevel)) battLevel = 3800;  // Use dummy voltage if can't read

//         // Create char buffers for the floating point numbers for sprintf
//         // Make sure these buffers are long enough for your request URL
//         char URL[150];
//         char body[100];
//         char tempBuff[16];

//         // Format the floating point numbers as needed
//         dtostrf(temperature, 1, 2, tempBuff);  // float_val, min_width, digits_after_decimal, char_buffer

// #ifdef SIMCOM_7070  // Use this line if you have the SIM7000G because the 1529B01SIM7000G firmware doesn't seem to run the commands below well \
//                     // #if defined(SIMCOM_7000) || defined(SIMCOM_7070) // Use this if you have SIM7000A, especially with SSL
//         // Add headers as needed
//         // modem.HTTP_addHeader("User-Agent", "SIM7070", 7);
//         // modem.HTTP_addHeader("Cache-control", "no-cache", 8);
//         // modem.HTTP_addHeader("Connection", "keep-alive", 10);
//         // modem.HTTP_addHeader("Accept", "*/*, 3);

//         // Connect to server
//         // If https:// is used, #define BOTLETICS_SSL 1 in Botletics_modem.h
//         if (!modem.HTTP_connect("http://dweet.io")) {
//           Serial.println(F("Failed to connect to server..."));
//           break;
//         }

//         // GET request
//         // Format URI with GET request query string
//         sprintf(URL, "/dweet/for/%s?temp=%s&batt=%i", imei, tempBuff, battLevel);
//         modem.HTTP_GET(URL);

//         // POST request
//         /*
//             sprintf(URL, "/dweet/for/%s", imei); // Format URI

//             // Format JSON body for POST request
//             // Example JSON body: "{\"temp\":\"22.3\",\"batt\":\"3800\"}"
// //            sprintf(body, "{\"temp\":\"%s\",\"batt\":\"%i\"}", tempBuff, battLevel); // construct JSON body

// //            modem.HTTP_addHeader("Content-Type", "application/json", 16);
//             modem.HTTP_addPara("temp", "23.4", 5); // Test value
//             modem.HTTP_addPara("batt", "4120", 5); // Test value
//             modem.HTTP_POST(URL, body, strlen(body));
//             */

// #else
//           // Construct the appropriate URL's and body, depending on request type
//         // Use IMEI as device ID for this example

//         // GET request
//         sprintf(URL, "dweet.io/dweet/for/%s?temp=%s&batt=%i", imei, tempBuff, battLevel);  // No need to specify http:// or https://
//                                                                                            //        sprintf(URL, "http://dweet.io/dweet/for/%s?temp=%s&batt=%i", imei, tempBuff, battLevel); // But this works too

//         if (!modem.postData("GET", URL))
//           Serial.println(F("Failed to complete HTTP GET..."));

//           // POST request
//           /*
//             sprintf(URL, "http://dweet.io/dweet/for/%s", imei);
//             sprintf(body, "{\"temp\":%s,\"batt\":%i}", tempBuff, battLevel);

//             if (!modem.postData("POST", URL, body)) // Can also add authorization token parameter!
//               Serial.println(F("Failed to complete HTTP POST..."));
//             */

// #endif

//         break;
//       }
// #endif

// #if defined(SIMCOM_3G) || defined(SIMCOM_7500) || defined(SIMCOM_7600)
//     case '3':
//       {
//         // Post data to website via 3G or 4G LTE
//         float temperature = analogRead(A0) * 1.23;  // Change this to suit your needs

//         uint16_t battLevel;
//         if (!modem.getBattVoltage(&battLevel)) battLevel = 3800;  // Use dummy voltage if can't read

//         // Create char buffers for the floating point numbers for sprintf
//         // Make sure these buffers are long enough for your request URL
//         char URL[150];
//         char tempBuff[16];

//         // Format the floating point numbers as needed
//         dtostrf(temperature, 1, 2, tempBuff);  // float_val, min_width, digits_after_decimal, char_buffer

//         // Construct the appropriate URL's and body, depending on request type
//         // Use IMEI as device ID for this example

//         // GET request
//         sprintf(URL, "GET /dweet/for/%s?temp=%s&batt=%i HTTP/1.1\r\nHost: dweet.io\r\n\r\n", imei, tempBuff, battLevel);

//         if (!modem.postData("www.dweet.io", 443, "HTTPS", URL))  // Server, port, connection type, URL
//           Serial.println(F("Failed to complete HTTP/HTTPS request..."));

//         break;
//       }
// #endif
//       /*****************************************/

//     case 'S':
//       {
//         Serial.println(F("Creating SERIAL TUBE"));
//         while (1) {
//           while (Serial.available()) {
//             delay(1);
//             modem.write(Serial.read());
//           }
//           if (modem.available()) {
//             Serial.write(modem.read());
//           }
//         }
//         break;
//       }

//     default:
//       {
//         // Serial.println(F("Unknown command"));
//         // printMenu();
//         break;
//       }
//   }
//   // flush input
//   flushSerial();
//   while (modem.available()) {
//     Serial.write(modem.read());
//   }
// }

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
