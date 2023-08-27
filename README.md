# CLIMAX2001
*An Arduino UNO based room-climate logger*

- Adafruit Arduino datalogging shield (SD-Card + RTC)
- CJMCU-811 CO2/TVOC sensor (disabled)
- SCD30 Sensirion CO2/temperature/humidity sensor module
- BME280 humidity/temperature/barometer Sensor (used only for pressure since SCD30 was added)
- HM3301 Seeed PM2.5 Dust Detection Sensor
- LDR Light Sensor
- SSD1306 OLED Display 0,96"
- Push button to toggle display on/off
- Everything put together into and onto an EURO-BOX housing
![Climax2001_front](https://user-images.githubusercontent.com/52123868/236780165-3b59ef0f-1c78-4aa1-a62c-2104123e7b28.JPG)

## Exel CSV Import
It's a bit painful to insert comma-separated values from a CSV file - especially if you are in a country where your PC and your software (like MS Office) is not running under US locals. Here is way to fix that problem:
1. Excel > Data > From Text/CSV
2. Import "LOGGERnn.CSV"
3. The preview shows that the commas are missing e.g. in values of temperature, humidity or pressure if your office is not running under "US-locals"
4. To fix that go to Import dialog window and choose "Transform Data" to open the "Power Query Editor" 
5. In "Power Query Editor" window set locals to "English (United States)" via "File > Options and Settings > Query Options > Regional Settings"
6. The preview shows now the commas correctly 
7. Click "Close & Load"
8. Your values appear now properly in Excel including commas for temperature, humidity and pressure

## Draw Exel diagrams
1. Mark colum A (datetime) and select "Format cells"
2. Choose "Time" and select "13:30:55" (if you keep the date, drawing the diagrams will not work)
3. Mark e.g colum A (datetime) first(!) and colum C (temp) second(!) and go to Insert > Insert Line or Area Chart
4. A nice diagram with the name of the selected second colum is drawn over time
