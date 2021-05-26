// BMx280_I2C.ino
//
// shows how to use the BMP280 / BMx280 library with the sensor connected using I2C.
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


const char* ssid = "SSID;
const char* password = "PASSWD";
const char* mqtt_server = "IP address";

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

struct nv_s {
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
  readVoltage();
  handleOTA();
  digitalWrite(4, LOW);    // Power off peripherals.
  reportWaketime();        // report time spent awake.
  ESP.deepSleep(60e6);
}

void reconnect() {
  // Loop until we're reconnected
  using namespace std;
  while (!client.connected()) {
    Serial.println();
    Serial.print("Connection to MQTT broker ");
    // Attempt to connect
    if (client.connect("SolMon")) {
      Serial.println("established.");
      //Serial.println();
      // Once connected, publish an announcement...
      client.publish("SolMon/run", String(resetCount).c_str());
      Serial.print(F("\nReset count = "));
      Serial.println(resetCount);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 second");
      // Wait 1 second before retrying
      delay(1000);
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

void setupOTA() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("solmon");

  // No authentication by default
  //ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  ArduinoOTA.setPasswordHash("B43B1C8BA7E211D86B90C354A60D96C3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
    client.publish("SolMon/OTA", String(type).c_str());
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    client.publish("SolMon/OTA", "COMPLETE");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    client.publish("SolMon/OTA", String(error).c_str());
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  client.publish("SolMon/OTA", "READY");
}

void handleOTA() {
  setupOTA();
  ArduinoOTA.handle();
}

void reportClimate() {
  /* MEASURE MBP280 */
  float pascals = 0.0;
  float celsius = 0.0;
  
  //start a measurement
  if (!bmx280.measure())
  {
    Serial.println("could not start measurement, is a measurement already running?");
    client.publish("SolMon/console", "could not start measurement, is a BMx280 request already running?");
    return;
  }

  //wait for the measurement to finish
  //important: measurement data is read from the sensor in function hasValue() only.
  //make sure to call get*() functions only after hasValue() has returned true.
  do
  {
    delay(10);
  } while (!bmx280.hasValue());

  Serial.print("Pressure: "); Serial.println(bmx280.getPressure64());
  Serial.print("Temperature: "); Serial.println(bmx280.getTemperature());
  
  // Publish to mqtt
  pascals = bmx280.getPressure64();
  float mbars = pascals / 100;
  client.publish("climate/outdoor/millibars", String(mbars).c_str());
  float inHg =   roundf((pascals / 3386.39) * 100) / 100;
  client.publish("climate/outdoor/inHg", String(inHg).c_str());
  celsius = bmx280.getTemperature();
  float tempC = roundf(celsius * 100) / 100;
  client.publish("climate/outdoor/temp/c", String(tempC).c_str());
  float tempF = roundf(((celsius * 1.8) + 32 ) * 100 ) /100;
  client.publish("climate/outdoor/temp/f", String(tempF).c_str());

  if (bmx280.isBME280())
  {
    float humid = bmx280.getHumidity();
    Serial.print("Humidity: "); 
    //Serial.println(humid);
    //float humidity = roundf(humid * 100) / 100;
    client.publish("climate/outdoor/humidity", String(humid).c_str());
    Serial.println(humid);
  }
}

void reportPower() {
  /* MEASURE ADS1115 */
  float voltage = 0.0;
  float volts = 0.0;
  
  Serial.print("Battery: ");
  voltage = readChannel(ADS1115_COMP_0_GND);
  //Serial.println(voltage);
  volts = roundf(voltage * 326) / 100;
  // Publish a message to mqtt
  client.publish("SolMon/battery", String(volts).c_str());
  Serial.println(volts);

  Serial.print("Panel:   ");
  voltage = readChannel(ADS1115_COMP_1_GND);
  //Serial.println(voltage);
  volts = roundf(voltage * 400) / 100;
  // Publish a message to mqtt
  client.publish("SolMon/panel", String(volts).c_str());
  Serial.println(volts);
}


void readVoltage() { // read internal VCC
  //float voltage = ESP.getVcc();
  //Serial.printf("The internal VCC reads %1.2f volts\n", voltage / 1000);
  float volts = roundf(espVoltage * .1) / 100;
  // Publish a message to mqtt
  client.publish("SolMon/vcc", String(volts).c_str());
  Serial.printf("ESP Vcc:  %1.2f volts\n", volts);

}

void reportWaketime() {
  Serial.print("Execution time = ");
  Serial.println(millis());
  Serial.flush();
  int wakeTime = millis();
  while (!client.publish("SolMon/duration", String(wakeTime).c_str())) {
    delay(10);
  }  
}

void updateRTCcrc() {  // updates the reset count CRC
  nv->rtcData.crc32 = crc32((uint8_t*) &nv->rtcData.rstCount, sizeof(nv->rtcData.rstCount));
}
