# ESP-SolMon
HomeAssistant integrated Weather station for ESP8266 powered by fadushin/solar-esp32 solar power charge supervisor,
with optional direct to InfluxDB output.

This is a work in progress. This code is meant for a solar power weather station. Mine is built with an ESP-12E module,
but any ESP8266 will work. With very few modifications it shold work on and ESP32 aws well. The board is powered by a
LiFePo4 battery and an amazing little (1 inch square) open hardware solar charge module that you can build yourself. See 
https://github.com/fadushin/solar-esp32 for more details on the solar charger. I am currently using mine with a very small 
36x68mm solar panel and it keeps the battery near full all the time, and it recovers to a full charge even on cloudy days.

A BME280 is used to collect temperature, air pressure, and humidity. An ADS1115 is used to collect battery and panel
voltage readings from the solar-esp32 module. A 2N2222 transistor is used to cut power to the sensor modules during sleep,
assuring as minimal of a power drain as possible. A lux meter will be added soon. I also have plans to add a rain intensity
sensor and an aneometer.

Copy conf.h.example to conf.h and edit to set the device details, slect Home Assistant and/or InfluxDB output,
server addresses, and passwords.
