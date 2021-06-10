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
//#include <WiFiUdp.h>
//#include <ArduinoOTA.h>
#include <include/WiFiState.h>
#include <coredecls.h>         // crc32()
#include "conf.h"    // IMPORTANT: copy conf.h.example to conf.h and edit.

const char* ssid = SSID;            // defined in conf.h
const char* password = PASSWORD;    // defined in conf.h
const char* mqtt_server = MQTT_IP;  // defined in conf.h

#define BMP_ADDRESS 0x76
#define ADC_ADDRESS 0x48
//#define SDA 2
//#define SCL 14

//create an BMP280 object using the I2C interface
BMx280I2C bmx280(BMP_ADDRESS);

//create an ADS1115 object using the I2C interface
ADS1115_WE adc = ADS1115_WE(ADC_ADDRESS);

WiFiClient espClient;

#if defined(OUTPUT_HA)
#include <ArduinoHA.h>
// setup device instance
HADevice device;
HAMqtt mqtt(espClient, device);
// define sensors/topics
HASensor sm_run("runs");
HASensor sm_vcc("Vcc");
HASensor sm_time("Time-Awake");
HASensor sm_bat("Battery-Voltage");
HASensor sm_sol("Panel-Voltage");
HASensor sm_wifi("WiFi-dB");
HASensor sm_log("Log");
HASensor sm_mbar("millibars");
HASensor sm_mercury("inHg");
HASensor sm_c("C");
HASensor sm_f("F");
HASensor sm_humid("Humidity");
#endif

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

//
/*
 * Results in:
 *    $<base-topic>/sensors/$<device-UUID>/$<HASensor>/state
 *    the "state" topic will contain the sensor value
 */

void setup() {
  resetCount = 0;
  String resetCause = ESP.getResetReason();
    // Read previous resets (Deep Sleeps) from RTC memory, if any
  uint32_t crcOfData = crc32((uint8_t*) &nv->rtcData.rstCount, sizeof(nv->rtcData.rstCount));
  if ((crcOfData = nv->rtcData.crc32) && (resetCause == "Deep-Sleep Wake")) {
    resetCount = nv->rtcData.rstCount;  // read the previous reset count
    resetCount++;
  }
  nv->rtcData.rstCount = resetCount; // update the reset count and save to rtc
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

  #if defined(OUTPUT_HA)
  // Unique ID must be set!
  byte mac[WL_MAC_ADDR_LENGTH];
  WiFi.macAddress(mac);
  #endif
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print("."); 
  }
  
  #if defined(OUTPUT_HA)
  device.setUniqueId(mac, sizeof(mac));
  // set device's details
  device.setName("SolMon");
  device.setSoftwareVersion(FW_VERSION);
  device.setManufacturer(MANUFACTURER);
  device.setModel(MODEL_NO);
  // additional sensor parameters (optional)
  sm_run.setName("Wakes without reset");
  sm_run.setIcon("mdi:run");
  sm_time.setName("Time Awake");
  sm_time.setIcon("mdi:timer");
  sm_time.setUnitOfMeasurement("ms");
  sm_vcc.setName("SolMon Vcc");
  sm_vcc.setIcon("mdi:current-dc");
  sm_vcc.setDeviceClass("voltage");
  sm_vcc.setUnitOfMeasurement("V");
  sm_bat.setName("SolMon Battery");
  sm_bat.setIcon("mdi:battery-plus");
  sm_bat.setDeviceClass("voltage");
  sm_bat.setUnitOfMeasurement("V"); 
  sm_sol.setName("Solar Panel");
  sm_sol.setIcon("mdi:solar-power");
  sm_sol.setDeviceClass("voltage");
  sm_sol.setUnitOfMeasurement("V");
  sm_wifi.setName("ESP WiFi Signal");
  sm_wifi.setIcon("mdi:wifi");
  sm_wifi.setDeviceClass("signal_strength");
  sm_wifi.setUnitOfMeasurement("dB");
  sm_log.setName("SolMon Log");
  sm_log.setIcon("mdi:file-alert");
  sm_mercury.setName("Air Pressure");
  sm_mercury.setIcon("mdi:gauge-low");
  sm_mercury.setDeviceClass("pressure");
  sm_mbar.setName("Outdoor Air Pressure");
  sm_mbar.setIcon("mdi:gauge-low");
  sm_mbar.setDeviceClass("pressure");
  sm_mbar.setUnitOfMeasurement("mbar");
  sm_c.setName("Outdoor Temperature");
  sm_c.setIcon("mdi:thermometer");
  sm_c.setDeviceClass("temperature");
  sm_c.setUnitOfMeasurement("°C");
  sm_f.setName("Outdoor Temperature");
  sm_f.setIcon("mdi:thermometer");
  sm_f.setDeviceClass("temperature");
  sm_f.setUnitOfMeasurement("°F");
  sm_humid.setName("Outdoor Humidity");
  sm_humid.setIcon("mdi:gauge");
  sm_humid.setDeviceClass("humidity");
  sm_humid.setUnitOfMeasurement("%");

  mqtt.begin(mqtt_server);
  #endif

}

void loop() {

  mqtt.loop();
  reportStatus();
  reportClimate();
  reportVoltage();
  digitalWrite(4, LOW);    // Power off peripherals.
  reportWaketime();        // report time spent awake.
  ESP.deepSleep(60e6);
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
  
  //start a measurement
  if (!bmx280.measure())
  {
    Serial.println("could not start measurement, is a measurement already running?");
    #if defined(OUTPUT_HA)
    sm_log.setValue("BMx280 Failure.");
    #endif  
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
  #if defined(OUTPUT_HA)
  sm_mbar.setValue(mbars);
  #endif

  float inHg = mbars / 33.8639;
  #if defined(OUTPUT_HA)
  sm_mercury.setValue(inHg);
  #endif

  float celsius = bmx280.getTemperature();
  #if defined(OUTPUT_HA)
  sm_c.setValue(celsius);
  #endif

  float tempF = (celsius * 1.8) + 32;
  #if defined(OUTPUT_HA)
  sm_f.setValue(tempF);
  #endif

  if (bmx280.isBME280())
  {
    float humid = bmx280.getHumidity();
    Serial.print("Humidity: ");
    #if defined(OUTPUT_HA)
    sm_humid.setValue(humid);
    #endif 
    Serial.println(humid);
  }
}

void reportStatus() {
  static float BATvolts = (readChannel(ADS1115_COMP_0_GND) * 3.26);
  static float SOLvolts = (readChannel(ADS1115_COMP_1_GND) * 4);
  static float sigLevel = WiFi.RSSI();
  static float INPUTvolts = ((espVoltage - 220) / 1000);
  //char value[6];

  //dtostrf(INPUTvolts, 4, 2, value);  // convert volts to 4 character value[] - rounding at 2 decimal spaces
  //strcat(value, "\0");
  Serial.print("ESP Vcc: ");
  #if defined(OUTPUT_HA)
  sm_vcc.setValue(INPUTvolts);
  #endif 
  Serial.print(INPUTvolts);
  Serial.println(" volts");
  
  //uint32_t resetCount -- global
  //BATvolts = (readChannel(ADS1115_COMP_0_GND) * 3.26);
  //SOLvolts = (readChannel(ADS1115_COMP_1_GND) * 4);

  Serial.print(F("\nReset count = "));
  #if defined(OUTPUT_HA)
  sm_run.setValue(resetCount);
  #endif 
  Serial.println(resetCount);
  
  Serial.print("Battery: ");
  #if defined(OUTPUT_HA)
  sm_bat.setValue(BATvolts);
  #endif 
  Serial.println(BATvolts);

  Serial.print("Panel:   ");
  #if defined(OUTPUT_HA)
  sm_sol.setValue(SOLvolts);
  #endif 
  Serial.println(SOLvolts);

  Serial.print("WiFi dB: ");
  #if defined(OUTPUT_HA)
  sm_wifi.setValue(sigLevel);
  #endif 
  Serial.println(sigLevel);
}

void reportVoltage() {            // read internal VCC
  static float INPUTvolts = ((espVoltage - 220) / 1000);
  char value[6];
  dtostrf(INPUTvolts, 4, 2, value);  // convert volts to 4 character value[] - rounding at 2 decimal spaces
  strcat(value, "\0");
  Serial.print("ESP Vcc: ");
  #if defined(OUTPUT_HA)
  sm_vcc.setValue(INPUTvolts);
  #endif 
  Serial.print(value);
  Serial.println(" volts");
}

void reportWaketime() {
  Serial.print("Execution time = ");
  #if defined(OUTPUT_HA)
  int timer = millis();
  sm_time.setValue(timer);
  #endif
  Serial.println(millis());
  Serial.flush();
}

void updateRTCcrc() {  // updates the reset count CRC
  nv->rtcData.crc32 = crc32((uint8_t*) &nv->rtcData.rstCount, sizeof(nv->rtcData.rstCount));
}

void onBeforeSwitchStateChanged(bool state, HASwitch* s)
{
    // this callback will be called before publishing new state to HA
    // in some cases there may be delay before onStateChanged is called due to network latency
}
