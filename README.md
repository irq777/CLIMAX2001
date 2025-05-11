# CLIMAX2001
*An Arduino UNO based room-climate logger with SD-card*

- [Adafruit Arduino datalogging shield with RTC](https://learn.adafruit.com/adafruit-data-logger-shield/overview) writing measured data in CSV format to a SD-Card
- [CJMCU-811 CO2/TVOC sensor](https://learn.adafruit.com/adafruit-ccs811-air-quality-sensor/arduino-wiring-test) (disabled and replaced by SCD30)
- [SCD30 Sensirion CO2/temperature/humidity sensor](https://www.sensirion.com/products/catalog/SCD30/) module
- [BME280 humidity/temperature/barometer sensor](https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout/arduino-test) (used only for pressure since SCD30 was added but currently disabled due to issue #6)
- [HM3301 Seeed PM2.5 dust detection laser sensor](https://wiki.seeedstudio.com/Grove-Laser_PM2.5_Sensor-HM3301/)
- LDR light sensor
- SSD1306 128x64 OLED Display 0,96"
- Push button to toggle display on/off
- Everything put together into and onto a nice yellow EURO-BOX housing
- [DCF77](https://en.wikipedia.org/wiki/DCF77) time signal (currently deactivated)
- RGB LED indicating the CO2 level:
  - <600 green
  - <1000 light green
  - <1500 orange 
  - \>=1500 red
   
![Climax2001_front](https://user-images.githubusercontent.com/52123868/236780165-3b59ef0f-1c78-4aa1-a62c-2104123e7b28.JPG)

## Known Issues
- The temperature-measurement is not very linear regardless an individual offset-correection. See https://github.com/irq777/CLIMAX2001/issues/4.

## Exel CSV Import
It's a bit painful to insert comma-separated values from a CSV file - especially if you are in a country where your PC and your software (like MS Office) is not running under US locals. 
Below is way to fix that problem. The easiest but more radical solution is to just to set your "Region" to e.g. English (United States) in Windows.
1. Excel > Data > From Text/CSV
2. Import "LOGGERnn.CSV"
3. The preview shows that the commas are missing e.g. in values of temperature, humidity or pressure if your office is not running under "US-locals"
4. To fix that go to Import dialog window and choose "Transform Data" to open the "Power Query Editor" 
5. In "Power Query Editor" window set locals to "English (United States)" via "File > Options and Settings > Query Options > Regional Settings"
6. Close "Power Query Editor" and proceed with step 1. (I don't know why but I have to change the locals in "Power Query Editor" every time I start Excel)
7. The preview shows now the commas correctly 
8. Click "Close & Load"
9. Your values appear now properly in Excel including commas for temperature, humidity and pressure

## Draw Exel diagrams
1. Mark colum A (datetime) and select "Format cells"
2. Choose "Time" and select "13:30:55" (if you keep the date, drawing the diagrams will not work)
3. Mark e.g colum A (datetime) first(!) and colum C (temp) second(!) and go to Insert > Insert Line or Area Chart
4. A nice diagram with the name of the selected second colum is drawn over time
