// Rational:
// This code measures time, temperature, CO2 (ppm) and tvoc (ppb) every minute and log it to a SD-Card in CSV format

// Sources:
// Dallas DS1820 Sensor: https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806 / https://www.arduino.cc/reference/en/libraries/onewire/
// Adafruit Arduino Datalogging Shield: https://learn.adafruit.com/adafruit-data-logger-shield/overview
// Light & Temp Logger: https://github.com/adafruit/Light-and-Temp-logger
// CJMCU-811 CO2 Sensor board with CCS811 air quality sensor: https://iotspace.dev/arduino-co2-sensor-im-eigenbau-ccs811-sensor/
// BME280 Humidity/Temp/Barometer Sensor https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout/arduino-test
// SSD1306 OLED Display 0,96" 5V I2C 128x64Pixel  https://learn.adafruit.com/monochrome-oled-breakouts/arduino-library-and-examples 
//                                                https://draeger-it.blog/fehler-ssd1306-allocation-failed-am-oled-display-beheben/

// Notes:
// - Run CCS811 for 20 minutes, before accurate readings are generated
// - The equivalent CO2 (eCO2) output range for CCS811 is from 400ppm to 8192ppm. Values outside this range are clipped.
// - Total Volatile Organic Compound (TVOC) output range for CCS811 is from 0ppb to 1187ppb. Values outside this range are clipped.

/********************************************************************/
// First we include the libraries
#include <OneWire.h> //Dallas DS1820 Sensor https://github.com/PaulStoffregen/OneWire 
#include <DallasTemperature.h> //https://github.com/milesburton/Arduino-Temperature-Control-Library/blob/master/DallasTemperature.h
//#include <SPI.h>
#include <SD.h> //https://github.com/arduino-libraries/SD/blob/master/src/SD.h
//#include <Wire.h>
#include "RTClib.h" //https://github.com/adafruit/RTClib/blob/master/src/RTClib.h
#include "Adafruit_CCS811.h" //https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library/blob/master/src/SparkFunCCS811.h
#include "Adafruit_BME280.h" //https://github.com/adafruit/Adafruit_BME280_Library
#include "SSD1306Ascii.h" //https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiAvrI2c.h" //https://github.com/greiman/SSD1306Ascii

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL 1000 //60000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 60000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

// the digital pins that connect to the LEDs
#define LEDMODE 0
#if LEDMODE
 #define redLEDpin 2
 #define greenLEDpin 3
#endif //LEDMODE

// The light sensor
#define photocellPin 0 // analog 0

// SSD1306 Display
#define OLED_ON 1 // eneable / disable display support
#if OLED_ON
 #define I2C_ADDRESS 0x3C
 #define RST_PIN -1
 SSD1306AsciiAvrI2c oled;
#endif // OLED_ON

// Setup logging shield (RTC, SDCARD)
RTC_DS1307 RTC; // define the Real Time Clock object
const int chipSelect = 10;// for the data logging shield, we use digital pin 10 for the SD cs line
File logfile; // the logging file

#define DS1820_ON 0 //enable disable Dallas oneWire temperature sensor
#if DS1820_ON
 /********************************************************************/
 // Data wire is plugged into pin 2 on the Arduino 
 #define ONE_WIRE_BUS 9
 /********************************************************************/
 // Setup a oneWire instance to communicate with any OneWire devices  
 // (not just Maxim/Dallas temperature ICs)  
 OneWire oneWire(ONE_WIRE_BUS); 
 /********************************************************************/
 // Pass our oneWire reference to Dallas Temperature. 
 DallasTemperature sensors(&oneWire);
 /********************************************************************/ 
#endif // DS1820_ON

//CCS811 air quality sensor (CO2/TVOC)
Adafruit_CCS811 ccs; // I2C

//BME280 Humidity/Temp/Barometer Sensor
#define BME280_ON 1
#if BME280_ON
 //BME280 Humidity/Temp/Barometer Sensor
 Adafruit_BME280 bme; // I2C
#endif //BME280_ON

struct sensor_data{
  DateTime now;
  float humid;
  float temp;
  float press;
  int co2;
  int tvoc;
  int light;
#if DS1820_ON  
  float temperatureC; 
#endif // DS1820_ON 
};

sensor_data data = {};

unsigned long lastLogMillis = 0;
unsigned long lastSyncMillis = 0;

void setup(void) 
{ 
  // start serial port 
  Serial.begin(9600); 
  Serial.println("Bootup...");

#if OLED_ON
  //Init SSD1306 Display
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.println("Bootup...");
#endif // OLED_ON

  // use debugging LEDs
#if LEDMODE  
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
#endif //LEDMODE
  
#if WAIT_TO_START
  Serial.println(F("Type any character to start"));
  while (!Serial.available());
#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println(F("card initialized."));

  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }

  if (! logfile) {
    error("couldnt create file");
  }
  Serial.print(F("Logging to: "));
  Serial.println(filename);

#if DS1820_ON
 // Start up the DS1820 OneWire library for the DS1820 temperature sensor 
  sensors.begin(); 
#endif // DS1820_ON

  //Init CCS811 via I2C
  Serial.print("Initializing CCS811 sensor...");
  if(!ccs.begin(0x5A)){
   Serial.println(F("Could not start sensor. Pleas check wiring!"));
   while(1);
  }
  Serial.println(F("sensor initilized."));

#if BME280_ON  
  //Init BME280 via I2C
  if (!bme.begin(0x76)) {  
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1);
  } 
#endif // BME280_ON  

  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.println(F("RTC failed"));
#if ECHO_TO_SERIAL
    Serial.println(F("RTC failed"));
#endif  //ECHO_TO_SERIAL
  }

  // set RTC
  if (! RTC.isrunning()) {
    Serial.println(F("RTC is NOT running!"));
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

#if BME280_ON
  logfile.println(F("datetime,light,temp,humid,pressure,co2,tvoc"));  
#if ECHO_TO_SERIAL
  Serial.println(F("datetime,light,temp,humid,pressure,co2,tvoc"));
#endif //ECHO_TO_SERIAL
#else //BME280_ON
  logfile.println(F("datetime,light,co2,tvoc"));  
#if ECHO_TO_SERIAL
  Serial.println(F("datetime,light,co2,tvoc"));
#endif //ECHO_TO_SERIAL
#endif //BME280_ON
  
#if OLED_ON
  delay(1000);
  oled.clear();
#endif // OLED_ON 
}

bool fetch_data(void)
{
  // fetch the time
  data.now = RTC.now();

  // fetch sensordata
#if DS1820_ON
  // Dallas DS1820 Temperature Sensor (request temperatures of all devices on the onewire-bus and get temperature of fist found sensor)
  sensors.requestTemperatures(); 
  data.temperatureC = sensors.getTempCByIndex(0); //currently 
#endif // DS1820_ON

  // Light intensity 
  data.light = analogRead(photocellPin); //read light intensity (LDR is nonlinear, so there is no utit for this)

#if BME280_ON
  data.humid = bme.readHumidity();
  data.temp = bme.readTemperature();
  data.press = bme.readPressure();
  ccs.setEnvironmentalData(data.humid,data.temp); //send enviromnental data to the CSS811 for better compensation
#else //BME280_ON
  ccs.setEnvironmentalData(45,18); //set dummy temp & humidity 
#endif //BME280_ON

  //read CO2 and TVOC of CCS811
  if(ccs.available()){
   //float temp = ccs.calculateTemperature(); //That stuff is not working/unsupported by lib somehow
   if(!ccs.readData()){
    data.co2 = ccs.geteCO2();
    data.tvoc = ccs.getTVOC();}
   else{
    Serial.println(F("ERROR!"));
    while(1);}
  } 
  return true;
}

bool log_data(void)
{
#if OLED_ON
  oled.setCursor(0,0);
  oled.println("--== CLIMAX 2000 ==--");
  oled.print("   ");
  oled.print(data.now.year(), DEC);  
  oled.print("/"); 
  oled.print(data.now.month(), DEC);
  oled.print("/");
  oled.print(data.now.day(), DEC);
  oled.print(" ");
  if (data.now.hour()<10){
    oled.print("0");
  }
  oled.print(data.now.hour(), DEC);
  oled.print(":");
  if (data.now.minute()<10){
    oled.print("0");
  }
  oled.print(data.now.minute(),DEC);
  /*oled.print(":");
  if (data.now.second()<10){
    oled.print("0");
  }
  oled.print(data.now.second(),DEC);*/ 
  oled.println();    
#endif // OLED_ON

  logfile.print('"');
  logfile.print(data.now.year(), DEC);
  logfile.print("/");
  logfile.print(data.now.month(), DEC);
  logfile.print("/");
  logfile.print(data.now.day(), DEC);
  logfile.print(" ");
  logfile.print(data.now.hour(), DEC);
  logfile.print(":");
  logfile.print(data.now.minute(), DEC);
  logfile.print(":");
  logfile.print(data.now.second(), DEC);
  logfile.print('"');

#if ECHO_TO_SERIAL
  Serial.print('"');
  Serial.print(data.now.year(), DEC);
  Serial.print("/");
  Serial.print(data.now.month(), DEC);
  Serial.print("/");
  Serial.print(data.now.day(), DEC);
  Serial.print(" ");
  Serial.print(data.now.hour(), DEC);
  Serial.print(":");
  Serial.print(data.now.minute(), DEC);
  Serial.print(":");
  Serial.print(data.now.second(), DEC);
  Serial.print('"');
#endif //ECHO_TO_SERIAL

  logfile.print(", ");    
  logfile.print(data.light);
  logfile.print(", ");   
#if BME280_ON 
  logfile.print(data.temp);
  logfile.print(", ");  
  logfile.print(data.humid);
  logfile.print(", ");  
  logfile.print(data.press/100);
  logfile.print(", ");      
#endif // BME280_ON  
  logfile.print(data.co2);
  logfile.print(", ");  
  logfile.print(data.tvoc);
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(data.light);
  Serial.print("light, ");    
#if BME280_ON   
  Serial.print(data.temp);
  Serial.print("°C, "); 
  Serial.print(data.humid);
  Serial.print("%, ");   
  Serial.print(data.press/100);
  Serial.print("hPa, "); 
#endif // BME280_ON    
  Serial.print(data.co2);
  Serial.print("ppm, "); 
  Serial.print(data.tvoc);
  Serial.print("ppb"); 
#endif //ECHO_TO_SERIAL
  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

#if OLED_ON
  oled.print("Temperature: ");
  oled.print(data.temp);  
  oled.print(" C   ");  
  oled.println();  
  oled.print("Humidity: ");  
  oled.print(data.humid);  
  oled.print(" %   ");
  oled.println(); 
  oled.print("Pressure: ");  
  oled.print(data.press/100);  
  oled.print(" hPA ");
  oled.println(); 
  oled.print("C02: ");  
  oled.print(data.co2);  
  oled.print(" ppm   ");
  oled.println(); 
  oled.print("TVOC: ");  
  oled.print(data.tvoc);  
  oled.print(" ppb   ");
  oled.println(); 
  oled.print("Light: ");  
  oled.print(data.light);  
  oled.print("   ");
  oled.println();   
#endif // OLED_ON  
}

void loop(void) 
{ 
  unsigned long currentMillis = millis();  //get the current time

  if(currentMillis-lastLogMillis >= LOG_INTERVAL){
    fetch_data(); 
    log_data(); 
    lastLogMillis = currentMillis;  
  }  
  
// Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time    
  if(currentMillis-lastSyncMillis >= SYNC_INTERVAL){
    Serial.println(F("log_data")); 
    logfile.flush();
    lastSyncMillis = currentMillis;  
  } 
} 

void error(char *str)
{
  Serial.print(F("error: "));
  Serial.println(str);
  
  // red LED indicates error
#if LEDMODE  
  digitalWrite(redLEDpin, HIGH);
#endif //LEDMODE

  while(1);
}