// BMx280_I2C.ino
//
// shows how to use the BMx280 library with the sensor connected using I2C.
//
// Copyright (c) 2018 Gregor Christandl

/***************************************************************************
* Example sketch for the ADS1115_WE library
*
* This sketch shows how to use the ADS1115 in single shot mode.
*
* Further information can be found on:
* https://wolles-elektronikkiste.de/ads1115 (German)
* https://wolles-elektronikkiste.de/en/ads1115-a-d-converter-with-amplifier (English)
*
***************************************************************************/
//
// ESP-SolMon
//
// Esp-12E weather monitor featuring bme280 for climate measurements, ads1115
// to measure analog readings from the amazing open hardware
// https://github.com/fadushin/solar-esp32/ solar power LiFePo4 battery charge supervisor.
//
// Copyright (c) 2021 Winford (Uncle Grumpy)
//

#include <Arduino.h>
#include <Wire.h>
#include<ADS1115_WE.h>
#include <BMx280I2C.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Hash.h>
#include <include/WiFiState.h>
#include <coredecls.h>         // crc32()
#include <ArduinoJson.h>
#include "conf.h"             // create this file and #define SSID, PASSWORD, MQTT_IP... 
                              // or comment out the above line and fill in below.

const char* ssid = SSID;
const char* password = PASSWORD;
const char* mqtt_server = MQTT_IP;

#define BMP_ADDRESS 0x76
#define ADC_ADDRESS 0x48
#define SDA 2
#define SCL 14

//create an BMP280 object using the I2C interface
BMx280I2C bmx280(BMP_ADDRESS);

//create an ADS1115 object using the I2C interface
ADS1115_WE adc = ADS1115_WE(ADC_ADDRESS);

WiFiClient espClient;
PubSubClient client(espClient);
float espVoltage;   //BAD global! this is my lazy way of reding the voltage before wifi is active
ADC_MODE(ADC_VCC);

struct nv_s {     // this trick comes from the esp8268/LowPowerDemo example sketch.
  WiFiState wss; // core's WiFi save state

  struct {
    uint32_t crc32;
    uint32_t rstCount;  // stores the Deep Sleep reset count
    // you can add anything else here that you want to save, must be 4-byte aligned
  } rtcData;
};

static nv_s* nv = (nv_s*)RTC_USER_MEM; // user RTC RAM area
uint32_t resetCount = 0;  // keeps track of the number of Deep Sleep tests / resets


void setup() {
  resetCount = 0;
  String resetCause = ESP.getResetReason();
    // Read previous resets (Deep Sleeps) from RTC memory, if any
  uint32_t crcOfData = crc32((uint8_t*) &nv->rtcData.rstCount, sizeof(nv->rtcData.rstCount));
  if ((crcOfData = nv->rtcData.crc32) && (resetCause == "Deep-Sleep Wake")) {
    resetCount = nv->rtcData.rstCount;  // read the previous reset count
    resetCount++;
  }
  nv->rtcData.rstCount = resetCount; // update the reset count & CRC
  updateRTCcrc();

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println();
  Serial.print("Connecting WiFi.");

  // Turn on power to sensors.
  pinMode(4, OUTPUT);       // base of 2N2222 peripheral power control.
  digitalWrite(4, HIGH);    // Power on peripherals.

  Wire.begin(SDA,SCL);

  /* SETUP BMP */
  //begin() checks the Interface, reads the sensor ID (to differentiate between BMP280 and BME280)
  //and reads compensation parameters.
  if (!bmx280.begin())
  {
    Serial.println("begin() failed. check your BMx280 Interface and I2C Address.");
    while (1);
  }

  //reset sensor to default parameters.
  bmx280.resetToDefaults();

  //by default sensing is disabled and must be enabled by setting a non-zero
  //oversampling setting.
  //set an oversampling setting for pressure and temperature measurements.
  bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
  bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);

  //if sensor is a BME280, set an oversampling setting for humidity measurements.
  if (bmx280.isBME280())
    bmx280.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);

  /* SETUP ADC */
 if(!adc.init()){
    Serial.println("ADS1115 not connected!");
  }

  /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   *
   * ADS1115_RANGE_6144  ->  +/- 6144 mV
   * ADS1115_RANGE_4096  ->  +/- 4096 mV
   * ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1115_RANGE_1024  ->  +/- 1024 mV
   * ADS1115_RANGE_0512  ->  +/- 512 mV
   * ADS1115_RANGE_0256  ->  +/- 256 mV
   */
  adc.setVoltageRange_mV(ADS1115_RANGE_2048); //comment line/change parameter to change range

  /* Set the inputs to be compared */
  adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.setCompareChannels(ADS1115_COMP_1_GND);
  //adc.setCompareChannels(ADS1115_COMP_2_GND);   /* uncomment to enable */
  //adc.setCompareChannels(ADS1115_COMP_3_GND);   /* uncomment to enable */

  /*
   * Alert pin is NOT connected!!!
   *    Do not enable it!!!
   */
  adc.setAlertPinMode(ADS1115_DISABLE_ALERT);

  /* Set the conversion rate in SPS (samples per second)
   * Options should be self-explaining:
   *
   *  ADS1115_8_SPS
   *  ADS1115_16_SPS
   *  ADS1115_32_SPS
   *  ADS1115_64_SPS
   *  ADS1115_128_SPS (default)
   *  ADS1115_250_SPS
   *  ADS1115_475_SPS
   *  ADS1115_860_SPS
   */
  adc.setConvRate(ADS1115_860_SPS); //uncomment if you want to change the default

  /* Set continuous or single shot mode:
   *
   *  ADS1115_CONTINUOUS  ->  continuous mode
   *  ADS1115_SINGLE     ->  single shot mode (default)
   */
  //adc.setMeasureMode(ADS1115_CONTINUOUS); //uncomment if you want to change the default

  /* ESP ADC */
  // measute the internal voltage before we turn on wifi, it will be reported latter.
  espVoltage = ESP.getVcc();

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

 client.setServer(mqtt_server, 1883);

}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  reportClimate();
  reportPower();
  reportVoltage();
  digitalWrite(4, LOW);    // Power off peripherals.
  reportWaketime();        // report time spent awake.
  ESP.deepSleep(60e6);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    char message[64];
    char counter[9];
    itoa ((int)resetCount,counter,10);
    StaticJsonDocument<48> doc;
    doc["count"] = counter;
    serializeJson(doc, message);
    Serial.println();
    Serial.print("Connection to MQTT broker ");
    // Attempt to connect
    if (client.connect("SolMon")) {
      Serial.println("established.");
      // Once connected, publish an announcement...
      while (!client.publish("SolMon/run", message, true)) {
        delay(5);
      }
      Serial.print(F("\nReset count = "));
      Serial.println(resetCount);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 second");
      // Wait 1/2 second before retrying
      delay(500);
    }
  }
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){}
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

void reportClimate() {
  /* MEASURE MBP280 */
  float pascals = 0.0;
  char message[24];
  char value[12];
  StaticJsonDocument<48> doc;
  
  //start a measurement
  if (!bmx280.measure())
  {
    Serial.println("could not start measurement, is a measurement already running?");
    StaticJsonDocument<48> doc;
    doc["ERROR"] = "BMx280 Failure.";
    serializeJson(doc, message);
    client.publish("SolMon/console", message, true);
    return;
  }

  //wait for the measurement to finish
  //important: measurement data is read from the sensor in function hasValue() only.
  //make sure to call get*() functions only after hasValue() has returned true.
  do
  {
    delay(5);
  } while (!bmx280.hasValue());

  Serial.print("Pressure: "); Serial.println(bmx280.getPressure64());
  Serial.print("Temperature: "); Serial.println(bmx280.getTemperature());

  // Publish to mqtt
  pascals = bmx280.getPressure64();
  float mbars = pascals / 100;
  dtostrf(mbars, 7, 2, value);  // convert mbars to 7 character value[] - rounding at 2 decimal spaces (i.e. "1013.37")
  doc.clear();
  doc["mbars"] = value;
  serializeJson(doc, message);
  while (!client.publish("sensors/climate/outdoor/millibars", message, true)) {
    delay(5);
  }
  float inHg = mbars / 33.8639;
  dtostrf(inHg, 5, 2, value);  // convert inHg to 5 character value[] - rounding at 2 decimal spaces (i.e. "29.37")
  doc.clear();
  doc["inHg"] = value;
  serializeJson(doc, message);
  while (!client.publish("sensors/climate/outdoor/inHg", message, true)) {
    delay(5);
  }
  float celsius = bmx280.getTemperature();
  dtostrf(celsius, 5, 2, value);  // convert celsius to 5 character value[] - rounding at 2 decimal spaces (i.e. "48.00")
  doc.clear();
  doc["C"] = value;
  serializeJson(doc, message);
  while (!client.publish("sensors/climate/outdoor/temp/c", message, true)) {
    delay(5);
  }
  float tempF = (celsius * 1.8) + 32;
  dtostrf(tempF, 5, 2, value);  // convert tempF to 6 character value[] - rounding at 2 decimal spaces (i.e. "100.00")
  doc.clear();
  doc["F"] = value;
  serializeJson(doc, message);
  while (!client.publish("sensors/climate/outdoor/temp/f", message, true)) {
    delay(5);
  }

  if (bmx280.isBME280())
  {
    float humid = bmx280.getHumidity();
    dtostrf(humid, 5, 2, value);  // convert humid to 5 character value[] - rounding at 2 decimal spaces (i.e. "100.00")
    doc.clear();
    doc["percent"] = value;
    serializeJson(doc, message);
    Serial.print("Humidity: ");
    while (!client.publish("sensors/climate/outdoor/humidity", message, true)) {
      delay(5);
    }
    Serial.println(humid);
  }
}

void reportPower() {
  /* MEASURE ADS1115 */
  float voltage = 0.0;
  //float volts = 0.0;
  char message[24];
  char value[6];
  
  Serial.print("Battery: ");
  voltage = (readChannel(ADS1115_COMP_0_GND) * 3.26);
  dtostrf(voltage, 4, 2, value);  // convert volts to 4 character value[] - rounding at 2 decimal spaces
  StaticJsonDocument<48> doc;
  doc["volts"] = value;
  serializeJson(doc, message);
  // Publish a message to mqtt
  while (!client.publish("SolMon/battery", message, true)) {
    delay(5);
  }
  Serial.println(value);

  Serial.print("Panel:   ");
  voltage = (readChannel(ADS1115_COMP_1_GND) * 4);
  dtostrf(voltage, 4, 2, value);  // convert volts to 4 character value[] - rounding at 2 decimal spaces
  doc.clear();
  doc["volts"] = value;
  serializeJson(doc, message);
  // Publish a message to mqtt
  while (!client.publish("SolMon/panel", message, true)) {
    delay(5);
  }
  Serial.println(value);
}


void reportVoltage() {            // read internal VCC
  static float volts = ((espVoltage - 220) / 1000);
  char message[24];
  char value[6];
  dtostrf(volts, 4, 2, value);  // convert volts to 4 character value[] - rounding at 2 decimal spaces
  StaticJsonDocument<48> doc;
  doc.clear();
  doc["volts"] = value;
  serializeJson(doc, message);
  // Publish a message to mqtt
  while (!client.publish("SolMon/vcc", message, true)) {
    delay(5);
  }
  strcat(value, "\0");
  Serial.print("ESP Vcc: ");
  Serial.print(value);
  Serial.println(" volts");
}

void reportWaketime() {
  Serial.print("Execution time = ");
  Serial.println(millis());
  Serial.flush();
  int wakeTime = millis();
  char message[64];
  char timer[9];
  itoa ((int)wakeTime,timer,10);
  StaticJsonDocument<48> doc;
  doc["milliseconds"] = timer;
  serializeJson(doc, message);
  while (!client.publish("SolMon/duration", message, true)) {
    delay(5);
  }
  client.disconnect();
}

void updateRTCcrc() {  // updates the reset count CRC
  nv->rtcData.crc32 = crc32((uint8_t*) &nv->rtcData.rstCount, sizeof(nv->rtcData.rstCount));
}
