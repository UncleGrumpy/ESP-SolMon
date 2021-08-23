/* BMx280_I2C.ino
  //
  // shows how to use the BMx280 library with the sensor connected using I2C.
  //
  // Copyright (c) 2018 Gregor Christandl
*/
/***************************************************************************
  Example sketch for the ADS1115_WE library

  This sketch shows how to use the ADS1115 in single shot mode.

  Further information can be found on:
  https://wolles-elektronikkiste.de/ads1115 (German)
  https://wolles-elektronikkiste.de/en/ads1115-a-d-converter-with-amplifier (English)

***************************************************************************/
//
// ESP-SolMon
//
// Esp-12E module weather monitor featuring bme280 for climate measurements, ads1115
// to measure analog readings from the amazing open hardware
// https://github.com/fadushin/solar-esp32/ solar power LiFePo4 battery charge supervisor.
// Should work with any ESP8266. Also ESP32 with a few changes to the includes, and other
// minor adjusments.
//
// Copyright (c) 2021 Winford (Uncle Grumpy)
//

#include <Arduino.h>
#include <LittleFS.h>
#include <Wire.h>
#include <ADS1115_WE.h>
#include <BMx280I2C.h>
#include <BH1750.h>
#include <ESP8266WiFi.h>
//#include <WiFiUdp.h>
//#include <ArduinoOTA.h>
//#include <include/WiFiState.h>
#include <coredecls.h>         // crc32()
#include "conf.h"    // IMPORTANT: copy conf.h.example to conf.h and edit.

const char* ssid = SSID;            // defined in conf.h
const char* password = PASSWORD;    // defined in conf.h
const char* mqtt_server = MQTT_IP;  // defined in conf.h

// create a BMP280 object.
BMx280I2C bmx280(BME_ADDRESS);
// create an ADS1115 object using the I2C interface
ADS1115_WE adc = ADS1115_WE(ADC_ADDRESS);
// create a BH1750 object.
BH1750 luxMeter;

WiFiClient espClient;

ADC_MODE(ADC_VCC);
static float espVoltage = ESP.getVcc();   //BAD global! this is my way of reading the voltage before wifi is active
// NOTE: The subtratction of 218 is to correct for a value that always mesures ~218mV above the true value on my ESP-12F.
// This is a known flaw of the esp8266 when measuring the input voltage with the internal ADC.
// Each module is different, but the good news is, that however wrong it is the ammount stays consistent. You can
// measure the true reading with a volt meter once and always use the same value to correct for that module.

struct nv_s {     // this trick borrowed from the esp8268/LowPowerDemo example sketch.
  uint32_t crc;  // =) Stored outside of the rtcData struct so we don't have to wory about offset when we calculate crc32 of the data.
  struct {
    // Add anything here that you want to save in RTC_USER_MEM. MUST be 4-byte aligned for crc to work!
    uint32_t rstCount;  // stores the Deep Sleep reset count
    uint32_t noWifi;     // stores the number of consecutive missed connections
    uint32_t channel;    // stores the wifi channel for faster no-scan connetion
    uint32_t bssid[6];   // stores mac address of AP for fast no-san connection
    //uint32_t crc;       // crc of rtcData
  } rtcData;
};

static nv_s* nv = (nv_s*)RTC_USER_MEM; // user RTC RAM area
uint32_t resetCount = 0;      // keeps track of the number of Deep Sleep tests / resets
uint32_t wifiMissed;       // keeps track of the number of consecutive missed wifi conections
uint32_t wifiChan;         // wifi channel needed for fast reconnect.
uint8_t wifiID[6];       // wifi bssid for fast reconnect.
 
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
HASensor sm_lux("Illuminance");
#endif

#if defined(OUTPUT_INFLUX)
#include <InfluxDbClient.h>
// InfluxDB client instance for InfluxDB 1
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DB_NAME);
// Uncomment below and to use InfluxDB 1 authentication, set influx user/pass in conf.h, and comment out/remove above line.
//client.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME, INFLUXDB_USER, INFLUXDB_PASSWORD);

// Data point
Point wifi_sensor("wifi_signal");
Point run_sensor("run_counter");
Point timer_sensor("time_awake");
Point voltage_sensor("voltage");
Point log_sensor("console_log");
Point pressure_sensor("air_pressure");
Point temp_sensor("temperature");
Point humid_sensor("humidity");
Point lux_sensor("illuminance");
#endif

void setup() {
  uint32_t setupStart = millis();
  Serial.begin(74880);  // Native Serial speed for ESP12-F so we can get boot messages too.
  delay(250);
  Serial.println();
  delay(50);
  Serial.println("boot time (ms): " + setupStart);
  delay(1);
  bool useRTC;
  String resetCause = ESP.getResetReason();
  Serial.println(resetCause);
  delay(1);
  // Read previous resets (Deep Sleeps) from RTC memory, if any
  //bool goodCRC = rtcValid();
  if ((rtcValid()) && (resetCause == "Deep-Sleep Wake")) {
    resetCount = nv->rtcData.rstCount;  // read the previous reset count
    resetCount++;
    nv->rtcData.rstCount = resetCount; // update the reset count and save to rtc
    wifiMissed = nv->rtcData.noWifi;
    for (int mem = 0; mem < 6; mem++) {
      wifiID[mem] = nv->rtcData.bssid[mem];
    } 
    useRTC = true;
  } else {
    if (rtcValid()) {
      // we were reset by some condition other than "Deep-Sleep Wake" so don't try fast reconnect.
      resetCount = nv->rtcData.rstCount;  // read the previous reset count
      resetCount++;
      nv->rtcData.rstCount = resetCount; // update the reset count and save to rtc
      wifiMissed = nv->rtcData.noWifi;
      useRTC = false;
    } else {
      resetCount++;
      nv->rtcData.rstCount = resetCount; // update the reset count and save to rtc
      useRTC = false;
    }
  }
  updateRTCcrc();
    
  if (!LittleFS.begin()) {
    Serial.println("Unable to start filesystem. Formatting.");
    LittleFS.format();
    delay(100);
    Serial.println("Done. Retrying to start LittleFS.");
    if (!LittleFS.begin()) {
      Serial.println("Unable to start filesystem. Giving up. Good Luck!");
    }
  }

  // WiFiInit() starts the WiFi connection in the background. It will return the time that the WiFi 
  // setup began, this start time is given to WiFiTimeout() as an argument to track the total
  // connection time. Most of the delay() and yield()in the following setup is to give time to
  // negotiate the connection before we call WiFiTimeout() and wait for the connection to finish
  // (or reach 10 seconds and abort.) The rest of setup() after WiFiTimeout requires an active connection.
  uint32_t startWifi = WiFiInit(useRTC);
  
  // Turn on power to sensors.
  pinMode(4, OUTPUT);       // base of 2N2222 peripheral power control.
  digitalWrite(4, HIGH);    // Power on peripherals.

  Wire.begin(SDA, SCL);
  delay(5);

  /* SETUP BMP */
  //begin() checks the Interface, reads the sensor ID (to differentiate between BMP280 and BME280)
  //and reads compensation parameters.
  if (!bmx280.begin())
  {
    char err[] = "BMx280 setup failed. Check I2C pins and address.\0";
    logError(err);
    Serial.println(err);
    delay(1);
  }

  //reset sensor to default parameters.
  bmx280.resetToDefaults();
  delay(1);

  //by default sensing is disabled and must be enabled by setting a non-zero
  //oversampling setting.
  //set an oversampling setting for pressure and temperature measurements.
  bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
  bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);
  delay(1);

  //if sensor is a BME280, set an oversampling setting for humidity measurements.
  if (bmx280.isBME280()) {
    bmx280.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);
    delay(1);
  }

  /* SETUP ADC */
  if (!adc.init()) {
    char err[] = "ADS1115 not connected!\0";
    logError(err);
    delay(1);
  }

  /* Set the voltage range of the ADC to adjust the gain
     Please note that you must not apply more than VDD + 0.3V to the input pins!

     ADS1115_RANGE_6144  ->  +/- 6144 mV
     ADS1115_RANGE_4096  ->  +/- 4096 mV
     ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
     ADS1115_RANGE_1024  ->  +/- 1024 mV
     ADS1115_RANGE_0512  ->  +/- 512 mV
     ADS1115_RANGE_0256  ->  +/- 256 mV
  */
  adc.setVoltageRange_mV(ADS1115_RANGE_2048); //comment line/change parameter to change range
  delay(1);
  
  /* Set the inputs to be compared */
  adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.setCompareChannels(ADS1115_COMP_1_GND);
  //adc.setCompareChannels(ADS1115_COMP_2_GND);   /* uncomment to enable */
  //adc.setCompareChannels(ADS1115_COMP_3_GND);   /* uncomment to enable */
  delay(1);
  
  // Alert pin is NOT connected!!! Do not enable it!!!
  adc.setAlertPinMode(ADS1115_DISABLE_ALERT);
  delay(1);
  
  /* Set the conversion rate in SPS (samples per second)
     Options should be self-explaining:

      ADS1115_8_SPS
      ADS1115_16_SPS
      ADS1115_32_SPS
      ADS1115_64_SPS
      ADS1115_128_SPS (default)
      ADS1115_250_SPS
      ADS1115_475_SPS
      ADS1115_860_SPS
  */
  adc.setConvRate(ADS1115_64_SPS); // taking a slightly longer measurment than default. hopfully increases accuracy.
  delay(1);
  
  /* Set continuous or single shot mode:

      ADS1115_CONTINUOUS  ->  continuous mode
      ADS1115_SINGLE     ->  single shot mode (default)
  */
  //adc.setMeasureMode(ADS1115_CONTINUOUS); //uncomment if you want to change the default

  /* SETUP BH1750 */
  if (!luxMeter.begin(BH1750::ONE_TIME_LOW_RES_MODE)) {
    char err[] = "Error initialising BH1750.";
    logError(err);
    Serial.println(err);
    delay(1);
  } else {
    delay(1);
    luxMeter.setMTreg(32);  // assume full sun for calibration.
    delay(5);
  }

#if defined(OUTPUT_HA)
  // Unique ID must be set!
  byte mac[WL_MAC_ADDR_LENGTH];
  WiFi.macAddress(mac);
  delay(1);
#endif

  // Need to make sure wifi finishes connecting before we can go on...
  WiFiTimeout(startWifi);

#if defined(OUTPUT_HA)  // Setup Home Assistant Device.
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
  sm_wifi.setName("SolMon WiFi Signal");
  sm_wifi.setIcon("mdi:wifi");
  sm_wifi.setDeviceClass("signal_strength");
  sm_wifi.setUnitOfMeasurement("dB");
  sm_log.setName("SolMon Log");
  sm_log.setIcon("mdi:file-alert");
  sm_mercury.setName("Air Pressure");
  sm_mercury.setIcon("mdi:gauge-low");
  sm_mercury.setUnitOfMeasurement("in Hg");
  sm_mbar.setName("Outdoor Air Pressure");
  sm_mbar.setIcon("mdi:gauge-low");
  sm_mbar.setDeviceClass("pressure");
  sm_mbar.setUnitOfMeasurement("mbar");
  sm_c.setName("Outdoor Temperature");
  sm_c.setIcon("mdi:thermometer");
  sm_c.setDeviceClass("temperature");
  sm_c.setUnitOfMeasurement("C");
  sm_f.setName("Outdoor Temperature");
  sm_f.setIcon("mdi:thermometer");
  sm_f.setDeviceClass("temperature");
  sm_f.setUnitOfMeasurement("F");
  sm_humid.setName("Outdoor Humidity");
  sm_humid.setIcon("mdi:gauge");
  sm_humid.setDeviceClass("humidity");
  sm_humid.setUnitOfMeasurement("%");
  sm_lux.setName("Solar Illuminance");
  sm_lux.setIcon("mdi:weather-sunny");
  sm_lux.setDeviceClass("illuminance");
  sm_lux.setUnitOfMeasurement("lx");

  mqtt.begin(mqtt_server);
#endif

#if defined(OUTPUT_INFLUX)  // Setup InfluxDB input.
  // Sensor parameters
  wifi_sensor.addTag("device", DEVICE_NAME);
  wifi_sensor.addTag("type", "wifi_signal");
  wifi_sensor.addTag("unit", "dB");
  wifi_sensor.addTag("location", "front yard");
  run_sensor.addTag("device", DEVICE_NAME);
  run_sensor.addTag("sensor", "wakeup_counter");
  run_sensor.addTag("type", "counter");
  timer_sensor.addTag("device", DEVICE_NAME);
  timer_sensor.addTag("type", "timer");
  log_sensor.addTag("device", DEVICE_NAME);
  log_sensor.addTag("type", "log");
  voltage_sensor.addTag("device", DEVICE_NAME);
  voltage_sensor.addTag("battery_sensor", "ads1115");
  voltage_sensor.addTag("panel_sensor", "ads1115");
  voltage_sensor.addTag("vcc_sensor", "esp_adc");
  voltage_sensor.addTag("type", "adc_voltage");
  voltage_sensor.addTag("unit", "Volts");
  voltage_sensor.addTag("location", "front yard");
  pressure_sensor.addTag("device", DEVICE_NAME);
  pressure_sensor.addTag("sensor", "bme280");
  pressure_sensor.addTag("type", "air_pressure");
  pressure_sensor.addTag("location", "front yard");
  temp_sensor.addTag("device", DEVICE_NAME);
  temp_sensor.addTag("sensor", "bme280");
  temp_sensor.addTag("type", "temperature");
  temp_sensor.addTag("location", "front yard");
  humid_sensor.addTag("device", DEVICE_NAME);
  humid_sensor.addTag("sensor", "bme280");
  humid_sensor.addTag("type", "humidity");
  humid_sensor.addTag("unit", "%");
  humid_sensor.addTag("location", "front yard");
  lux_sensor.addTag("device", DEVICE_NAME);
  lux_sensor.addTag("sensor", "bh1750");
  lux_sensor.addTag("type", "illuminance");
  lux_sensor.addTag("unit", "lx");
  lux_sensor.addTag("location", "front yard");
#endif

  // Upload logs if previous failed wifi connections.
  logCheck();

  uint32_t setupDone = millis();
  uint32_t setupTime = setupDone - setupStart;
  Serial.println("setup time (ms): " +  setupTime );
  Serial.println("Total init time (ms): " +  setupDone );
}

void loop() {
#if defined(OUTPUT_HA)
  mqtt.loop();
#endif
  reportStatus();
  reportLux();
  reportClimate();
  digitalWrite(4, LOW);    // Power off peripherals.
  reportWaketime();        // report time spent awake.
  delay(50);
  WiFi.disconnect( true );
  delay( 1 );
  WiFi.mode( WIFI_OFF );
  ESP.deepSleep(60e6, RF_DISABLED);
  delay(1);
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while (adc.isBusy()) {}
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

void reportClimate() {
  /* MEASURE BME280 */
  //start a measurement
  if (!bmx280.measure())
  {
    Serial.println("could not start measurement, is a measurement already running?");
#if defined(OUTPUT_HA)
    sm_log.setValue("BMx280 Failure.");
#endif
#if defined(OUTPUT_INFLUX)
    log_sensor.clearFields();
    log_sensor.addField("log", "BMx280 Failure");
    if (!client.writePoint(log_sensor)) {
      Serial.print("InfluxDB write failed: ");
      Serial.println(client.getLastErrorMessage());
    }
#endif
  }

  // wait for the measurement to finish
  // important: measurement data is read from the sensor in function hasValue() only.
  // make sure to call get*() functions only after hasValue() has returned true.
  do
  {
    delay(5);
  } while (!bmx280.hasValue());

  // Publish to mqtt
  float pascals = 0.0;
  pascals = bmx280.getPressure64();
  float mbars = pascals / 100;
  float inHg = mbars / 33.8639;
#if defined(OUTPUT_HA)
  sm_mbar.setValue(mbars);
  sm_mercury.setValue(inHg);
#endif
#if defined(OUTPUT_INFLUX)
  pressure_sensor.clearFields();
  pressure_sensor.addField("millibars", mbars);
  pressure_sensor.addField("pascals", pascals);
  pressure_sensor.addField("inches_mercury", inHg );
  if (!client.writePoint(pressure_sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
#endif
  Serial.print("Pressure: "); Serial.println(bmx280.getPressure64());

  float celsius = bmx280.getTemperature();
  float tempF = (celsius * 1.8) + 32;
#if defined(OUTPUT_HA)
  sm_c.setValue(celsius);
  sm_f.setValue(tempF);
#endif
#if defined(OUTPUT_INFLUX)
  temp_sensor.clearFields();
  temp_sensor.addField("outdoor_C", celsius);
  temp_sensor.addField("outdoor_F", tempF);
  if (!client.writePoint(temp_sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
#endif
  Serial.print("Temperature: "); Serial.println(bmx280.getTemperature());

  if (bmx280.isBME280())
  {
    float humid = bmx280.getHumidity();
    Serial.print("Humidity: ");
#if defined(OUTPUT_HA)
    sm_humid.setValue(humid);
#endif
    Serial.println(humid);
#if defined(OUTPUT_INFLUX)
    humid_sensor.clearFields();
    humid_sensor.addField("outdoor_humidity", humid);
    if (!client.writePoint(humid_sensor)) {
      Serial.print("InfluxDB write failed: ");
      Serial.println(client.getLastErrorMessage());
    }
#endif

  } else {
    Serial.println("could not measure humidity, is this a bmp280?");
#if defined(OUTPUT_HA)
    sm_log.setValue("BMP: No humidity");
#endif
#if defined(OUTPUT_INFLUX)
    log_sensor.clearFields();
    log_sensor.addField("log", "BMP: No humidity");
    if (!client.writePoint(log_sensor)) {
      Serial.print("InfluxDB write failed: ");
      Serial.println(client.getLastErrorMessage());
    }
#endif
  }
}

void reportLux() {
  // First measurement (still in low rez mode) is for calibration.
  while (!luxMeter.measurementReady(true)) {
    delay(10);
    Serial.print("#");
  }
  float cal = luxMeter.readLightLevel();
  // Setup next measurement in high resolution at optimal measument time.
  luxMeter.begin(BH1750::ONE_TIME_LOW_RES_MODE);
  if (cal < 0) {
    Serial.println("BH1750 Error condition detected");
#if defined(OUTPUT_HA)
    sm_log.setValue("BH1750 Failure.");
#endif
#if defined(OUTPUT_INFLUX)
    log_sensor.clearFields();
    log_sensor.addField("log", "BH1750 Failure");
    if (!client.writePoint(log_sensor)) {
#if defined(OUTPUT_HA)
      sm_log.setValue("InfluxDB write failed.");
#endif
      Serial.print("InfluxDB write failed: ");
      Serial.println(client.getLastErrorMessage());
    }
#endif
  } else {
    if (cal > 40000.0) {
      // Sunshine.
      // reduce measurement time - needed in direct sun light
      if (luxMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)) {
        if (luxMeter.setMTreg(32)) {
          Serial.println("Setting MTReg to low value for high light environment");
        } else {
            Serial.println("Error setting MTReg to low value for high light environment");
          }
        } else {
          Serial.println("Error setting up BH1750 high rez.");
        }
      } else {
        if (cal > 10.0) {
          // typical light environment
          if (luxMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE_2)) {
            if (luxMeter.setMTreg(69)) {
              Serial.println("Setting MTReg to default value for normal light environment (high rez)");
            } else {
              Serial.println("Error setting MTReg to default value for normal light environment (hugh rez)");
            }
          } else {
            Serial.println("Error setting up BH1750 high rez high def.");
          }
        } else {
          if (cal <= 10.0) {
            //very low light environment
            if (luxMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE_2)) {
              if (luxMeter.setMTreg(138)) {
                Serial.println("Setting MTReg to high value for low light environment (high rez)");
              } else {
                Serial.println("Error setting MTReg to high value for low light environment (high rez)");
              }
            } else {
              Serial.println("Error setting up BH1750 high rez high def.");
            }
          }
        }
      }
    }
    Serial.print("lux: ");
    while (!luxMeter.measurementReady(true)) {
      delay(1);
      //Serial.print("#");
    }
    float lux = luxMeter.readLightLevel();

#if defined(OUTPUT_HA)
  sm_lux.setValue(lux);
#endif
#if defined(OUTPUT_INFLUX)
  lux_sensor.clearFields();
  lux_sensor.addField("solar_illuminance", lux);
  if (!client.writePoint(lux_sensor)) {
#if defined(OUTPUT_HA)
    sm_log.setValue("InfluxDB write failed.");
#endif
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
#endif
  Serial.print(lux);
  Serial.println(" lx");
}

void reportStatus() {
  /* Report wake count */
  Serial.print("\nReset count = ");
#if defined(OUTPUT_HA)
  sm_run.setValue(resetCount);
  sm_log.setValue("");
#endif
  Serial.println(resetCount);
#if defined(OUTPUT_INFLUX)
  run_sensor.clearFields();
  run_sensor.addField("wake_counter", resetCount);
  if (!client.writePoint(run_sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
#endif

  static float BATvolts = (readChannel(ADS1115_COMP_0_GND) * 3.26);
  static float SOLvolts = (readChannel(ADS1115_COMP_1_GND) * 4);
  static float sigLevel = WiFi.RSSI();
  static float INPUTvolts = (espVoltage / 1024.00f );

  Serial.print("ESP Vcc: ");
  Serial.print(INPUTvolts);
  Serial.println(" volts");
#if defined(OUTPUT_HA)
  sm_vcc.setValue(INPUTvolts);
  sm_bat.setValue(BATvolts);
  sm_sol.setValue(SOLvolts);
#endif
#if defined(OUTPUT_INFLUX)
  voltage_sensor.clearFields();
  voltage_sensor.addField("SolMon_Vcc", INPUTvolts);
  voltage_sensor.addField("SolMon_Battery", BATvolts);
  voltage_sensor.addField("SolMon_Panel", SOLvolts);
  if (!client.writePoint(voltage_sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
#endif
  Serial.print("Battery: ");
  Serial.println(BATvolts);
  Serial.print("Panel:   ");
  Serial.println(SOLvolts);

  Serial.print("WiFi dB: ");
#if defined(OUTPUT_HA)
  sm_wifi.setValue(sigLevel);
#endif
  Serial.println(sigLevel);
#if defined(OUTPUT_INFLUX)
  wifi_sensor.clearFields();
  wifi_sensor.addField("SolMon", sigLevel);
  if (!client.writePoint(wifi_sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
#endif
}

void reportWaketime() {
  if (nv->crc != crc32((uint8_t*) &nv->rtcData, sizeof(nv->rtcData))) {
    updateRTCcrc();
    if (!rtcValid()) {
      Serial.println("Fatal flaw in RTC RAM detected!");
    }
  }
  Serial.print("Execution time = ");
#if defined(OUTPUT_HA)
  uint32_t timer = millis() + 165;         // adding 165 to account for the upcomming delays and WiFiShutdown =)
  sm_time.setValue(timer);
#endif
#if defined(OUTPUT_INFLUX)
  timer_sensor.clearFields();
  timer_sensor.addField("millisconds", timer);
  if (!client.writePoint(timer_sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
#endif
  Serial.println(millis());
  Serial.flush();
  WiFi.disconnect(true);
  delay(1);
}

void updateRTCcrc() {  // updates the reset count CRC
  nv->crc = crc32((uint8_t*)&nv->rtcData, sizeof(nv->rtcData));
  if (!rtcValid()){
    Serial.println("Failed to update RTC RAM");
  }
}

bool rtcValid() {
  bool valid;
  // Calculate the CRC of what we just read from RTC memory
  uint32_t crc = crc32((uint8_t*)&nv->rtcData, sizeof(nv->rtcData));
  if( crc != nv->crc ) {
    Serial.println("WARNING: rtcRAM failed crc validation!");
    valid = false;
  } else {
    valid = true;
  }
  return valid;
}

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

void logError(char text[]) {
  File logFile = LittleFS.open("/error.txt", "a");
  if (!logFile) {
    File logFile = LittleFS.open("/error.txt", "w");
  }
  logFile.println(text);
  delay(1);
  logFile.close();
  delay(1);
}

void logCheck() {
  File logFile = LittleFS.open("/error.txt", "r");
  if ((logFile.available()) && (sizeof(logFile) > 0 )) {
    char msg[48] = "Missed wireless connections: ";
    char num[18] = {0};
    itoa ((int)wifiMissed, num, 10);
    strcat(msg, num);
    Serial.println(msg);
    sm_log.setValue(msg);
    delay(1);
    while(logFile.available()) {
      Serial.write(logFile.read());
    }
    logFile.close();
    LittleFS.remove("/error.txt");
    delay(1);
    Serial.println();
  } else {    
    char msg[4] = {0};
    strcat(msg, "Ok.");
    Serial.println(msg);
    sm_log.setValue(msg);
    delay(1);
  }
  wifiMissed = 0;
  nv->rtcData.noWifi = wifiMissed; // reset the missed connection count and save to rtc
  updateRTCcrc();
  delay(1);
  for (int mem = 0; mem < 6; mem++ ) {
      Serial.print(nv->rtcData.bssid[mem], HEX);
  }
  Serial.println();
  LittleFS.end();
  delay(1);
}

uint32_t WiFiInit( bool rtcOK ) {
  uint32_t startWifi = millis();
#if defined(OUTPUT_HA)
  // Unique ID must be set!
  byte mac[WL_MAC_ADDR_LENGTH];
  WiFi.macAddress(mac);
  delay(1);
#endif
  // Optional for faster connetions.
  IPAddress staticIP(192, 168, 12, 15); // parameters below are for your static IP address, if used
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 0, 0);
  IPAddress dns1(192, 168, 0, 1);
  //IPAddress dns2(0, 0, 0, 0);
  WiFi.forceSleepWake();
  delay(1);
  WiFi.mode(WIFI_STA);
  //WiFi.setOutputPower(10);
  WiFi.persistent(false);   // Dont's save WiFiState to flash we will store it in RTC RAM later.
  WiFi.config(staticIP, gateway, subnet);
  if ((rtcOK) && (wifiMissed == 0)) {
    Serial.print("rtcOK = ");
    Serial.println(rtcOK);
    WiFi.begin( ssid, password, nv->rtcData.channel, wifiID, true );
    Serial.println();
    Serial.print("Using Channel ");
    Serial.println(nv->rtcData.channel);
    Serial.print("Reconnecting to previous network ");
    for (int mem = 0; mem < 6; mem++ ) {
      Serial.print(wifiID[mem], HEX);
    }
     //Serial.println();
  } else { 
    Serial.print("rtcOK = ");
    Serial.println(rtcOK);
    Serial.print("wifiMissed = ");
    Serial.println(wifiMissed);
    WiFi.begin(ssid, password);
    Serial.println();
    Serial.print("Connecting to network");
  }
  return startWifi;
}

void WiFiTimeout(uint32_t wifiTime) {
  uint32_t GiveUp = millis() + 10000;   // 10 seconds max before giving up.
  while ( (WiFi.status() != WL_CONNECTED) && (millis() < GiveUp) ) {
    delay(50);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    char err[] = "WiFi connect failed. Retry in 60 Seconds.\0";
    logError(err);
    delay(1);
    wifiMissed = nv->rtcData.noWifi;  // read the previous wifi fail count
    delay(1);
    wifiMissed++;
    Serial.print("SAVING WIFI MISSED #");
    Serial.println(wifiMissed);
    delay(1);
    nv->rtcData.noWifi = wifiMissed; // update the missed connection count and save to rtc
    delay(5);
    digitalWrite(4, LOW);   // Power off peripherals.
    Serial.println();
    Serial.println(err);
    updateRTCcrc();
    delay(5);
    WiFi.disconnect( true );
    delay( 1 );
    WiFi.mode( WIFI_OFF );
    ESP.deepSleep(60e6, RF_DISABLED);    // Try again in 60 seconds.
  } else {
    Serial.println();
    Serial.print("Wifi connect took (ms): ");
    Serial.println(millis() - wifiTime);
    nv->rtcData.channel = WiFi.channel();
    Serial.print("Wrote channel #");
    Serial.println(nv->rtcData.channel);
    delay(1);
    nv->rtcData.noWifi = 0;   // reset missed connection counter.
    delay(1);
    uint8_t* bss_id = WiFi.BSSID();
    Serial.print("Wrote network bssid > ");
    for (unsigned int len = 0; len < WL_MAC_ADDR_LENGTH; len++ ) {
      nv->rtcData.bssid[len] = bss_id[len];
      Serial.print(nv->rtcData.bssid[len], HEX);
    }
    Serial.println();
    updateRTCcrc();
    Serial.print("Connected to ");
    Serial.print(WiFi.BSSIDstr());
    Serial.print("  -- Channel ");
    Serial.println(WiFi.channel());
    delay(1);
    //WiFi.setAutoReconnect(true);
  }
}


/*
// modified WiFi.BSSIDstr() from lib.
String myBSSID(void) {
  struct station_config conf;
  char mac[18] = { 0 };
  wifi_station_get_config(&conf);
  sprintf(mac, "%02X,%02X,%02X,%02X,%02X,%02X", conf.bssid[0], conf.bssid[1], conf.bssid[2], conf.bssid[3], conf.bssid[4], conf.bssid[5]);
  return String(mac);
}
*/

/*
  void onBeforeSwitchStateChanged(bool state, HASwitch* s)
  {
    // this callback will be called before publishing new state to HA
    // in some cases there may be delay before onStateChanged is called due to network latency
  }
*/
