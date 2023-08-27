// Rational:
// This code measures time, temperature, CO2 (ppm) and tvoc (ppb) every minute and log it to a SD-Card in CSV format

// Sources:
// Dallas DS1820 sensor: https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806 / https://www.arduino.cc/reference/en/libraries/onewire/
// Adafruit Arduino datalogging shield: https://learn.adafruit.com/adafruit-data-logger-shield/overview
// Light & Temp logger: https://github.com/adafruit/Light-and-Temp-logger
// CJMCU-811 CO2/TVOC sensor board with CCS811 air quality sensor: https://iotspace.dev/arduino-co2-sensor-im-eigenbau-ccs811-sensor/
// BME280 Humidity/Temperature/Barometer sensor https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout/arduino-test
// SSD1306 OLED Display 0,96" 5V I2C 128x64Pixel  https://learn.adafruit.com/monochrome-oled-breakouts/arduino-library-and-examples 
//                                                https://draeger-it.blog/fehler-ssd1306-allocation-failed-am-oled-display-beheben/
// HM3301 Seeed PM2.5 dust detection sensor https://wiki.seeedstudio.com/Grove-Laser_PM2.5_Sensor-HM3301/
// SCD30 Sensirion CO2/Temperature/Humidity sensor module 

// Libraries:
// DallasTemperature (Miles Burton)
// OneWire (Jim Studt)
// OneButton (Matthias Hertel)
// RTClib (Adafruit)
// SSD1306Ascii (Bill Greiman)
// Adafruit CCS811 Library (Adafruit)
// Adafruit BME280 Library (Adafruit)
// Grove - Laser PM2.5 Sensor HM3301 (Seeed Studio)
// Adafruit SCD30 Library (Adafruit)

// Notes:
// - Run CCS811 for 20 minutes, before accurate readings are generated
// - The equivalent CO2 (eCO2) output range for CCS811 is from 400ppm to 8192ppm. Values outside this range are clipped.
// - Total Volatile Organic Compound (TVOC) output range for CCS811 is from 0ppb to 1187ppb. Values outside this range are clipped.
// - PM2.5 particle diameter 0,0025 mm or smaller (PM=particulate matter) 
// - PM10  particle diameter 0,0100 mm or smaller
// - PM1   particle diameter 0,0010 mm or smaller

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
#include <OneButton.h>
#include <Seeed_HM330X.h> //https://github.com/Seeed-Studio/Seeed_PM2_5_sensor_HM3301
#include <util/parity.h>  //https://wolles-elektronikkiste.de/dcf77-funkuhr
#include "Adafruit_SCD30.h" //https://github.com/adafruit/Adafruit_SCD30/
// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL 60000 // 60000 mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 60000 // 60000 mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()
#define CSVSEP ","

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

// the digital pins that connect to the LEDs if jumpers on board are set
#define LEDMODE 0
#if LEDMODE
 #define greenLedPin 4
 #define redLedPin 5
#endif //LEDMODE

//DCF77
#define DCF77 0
#if DCF77
 byte dcfInterruptPin=3;
 volatile unsigned long dcfLastInt = 0;
 volatile unsigned long long dcfCurrentBuf = 0;
 volatile byte dcfBufCounter;
#endif //DCF77

// The light sensor
#define photocellPin 0 // analog 0

// SSD1306 Display
#define OLED_ON 1 // eneable / disable display support
#if OLED_ON
 #define I2C_ADDRESS 0x3C 
 #define RST_PIN -1
 #define OLED_TIMEOUT 12000
 SSD1306AsciiAvrI2c oled; //I2C 0x3C
 bool permaOled = false;
#endif // OLED_ON

// Setup logging shield (RTC, SDCARD)
RTC_DS1307 RTC; // define the Real Time Clock object
const int chipSelect = 10;// for the data logging shield, we use digital pin 10 for the SD cs line
File logfile; // the logging file

#define DS1820_ON 0 //enable disable Dallas oneWire temperature sensor (currently unused coz onewire takes too much mem)
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

#define CCS811_ON 0
#if CCS811_ON
//CCS811 air quality sensor (CO2/TVOC)
Adafruit_CCS811 ccs; // I2C 0x5A
#endif //CCS811_ON

//BME280 Humidity/Temp/Barometer Sensor
#define BME280_ON 1
#if BME280_ON
  //BME280 Humidity/Temp/Barometer Sensor
  Adafruit_BME280 bme; // I2C 0x76
  #define BME_TEMP_OFFSET -0.3 //temperature offset correction
#endif //BME280_ON

#define SCD30_ON 1
#if SCD30_ON
// Sensirion SCD30 CO2/Temperature/Humidity
Adafruit_SCD30  scd30; // I2C 0x61
#endif //SCD30_ON

//Seeed PM2.5 Sensor HM3301
#define HM3301_ON 1
#if HM3301_ON
  HM330X hm3301; // I2C 0x40
#endif //HM3301_ON 

struct sensor_data{
  DateTime now;
  float humid;
  float temp;
  float press;
  int co2;
  int tvoc;
  int light;
#if HM3301_ON
  uint16_t sensnum;
  uint16_t pm1std;
  uint16_t pm25std;
  uint16_t pm10std;
  uint16_t pm1atm;
  uint16_t pm25atm;
  uint16_t pm10atm;
#endif //HM3301_ON
#if DS1820_ON  
  float temperatureC; 
#endif // DS1820_ON 
};

sensor_data data = {};

unsigned long currentMillis = 0;
unsigned long lastLogMillis = 0;
unsigned long lastSyncMillis = 0;
unsigned long lastOledMillis = 0;

OneButton button(2, true, true);

void setup(void) 
{ 
  // start serial port 
  Serial.begin(115200); 
  Serial.println(F("Bootup..."));

#if OLED_ON
  //Init SSD1306 Display
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.println(F("Bootup..."));
#endif // OLED_ON

  // use debugging LEDs
#if LEDMODE  
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
#endif //LEDMODE

#if DCF77
  pinMode(dcfInterruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(dcfInterruptPin), DCF77_ISR, CHANGE);
#endif //DCF77

#if WAIT_TO_START
  Serial.println(F("Type any character to start"));
  while (!Serial.available());
#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print(F("Initializing SD card..."));
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

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
  //need to correct date/time? set it below, uncomment and run it just once(!)
  //RTC.adjust(DateTime(2023, 3, 12, 11, 15, 0)); //e.g. set RTC to 2023-03-12 11:15:00 (am)

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println(F("card initialized."));

  // create a new file with ascending number 
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

  // create a new file using the current date & time
  /*DateTime timestamp = RTC.now();
  //Serial.println(String("DateTime::TIMESTAMP_FULL:\t")+data.now.timestamp(DateTime::TIMESTAMP_FULL));
  //Serial.println(String("DateTime::TIMESTAMP_FULL:\t")+timestamp(DateTime::TIMESTAMP_FULL));
  char filename[]="LOGGER00.CSV";
  //sprintf(filename, "%02d%02d%02d-%02d%02d%2d.csv", timestamp.hour(),timestamp.minute(),timestamp.second(),timestamp.month(),timestamp.day(),timestamp.year());
  //sprintf(filename, "12345678.csv");
  //sprintf(filename, "heinzis.csv");
  Serial.println(filename);
  if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
    }
*/
  if (! logfile) {
    error("couldnt create file");
  }
  Serial.print(F("Logging to: "));
  Serial.println(filename);

#if DS1820_ON
 // Start up the DS1820 OneWire library for the DS1820 temperature sensor 
  sensors.begin(); 
#endif // DS1820_ON

#if CCS811_ON
  //Init CCS811 (CO2) via I2C
  Serial.print(F("Initializing CCS811 sensor..."));
  if(!ccs.begin(0x5A)){
   Serial.println(F("Could not start CCS811 sensor, check wiring!"));
   while(1);
  }
  Serial.println(F("sensor initilized."));
#endif //CCS811_ON

#if BME280_ON  
  //Init BME280 (Humidity/Pressure/Temp) via I2C
  Serial.print(F("Initializing BME280 sensor..."));
  if (!bme.begin(0x76)) {  
    Serial.println(F("Could not find BME280 sensor, check wiring!"));
    while (1);
  } 
  Serial.println(F("sensor initilized."));
#endif // BME280_ON  

#if SCD30_ON
  //Init SCD30 (CO2/Temperature/Humidity)
  Serial.print(F("Initializing SCD30 sensor..."));
  if (!scd30.begin()) {
    Serial.println(F("Failed to find SCD30 chip, check wiring!"));
    while (1) { delay(10); }
  }
  Serial.println(F("sensor initilized."));
  Serial.print(F("Measurement Interval: "));
  Serial.print(scd30.getMeasurementInterval());
  Serial.println(F(" seconds"));
#endif //SCD30_ON

  //Write data-headers to CSV 
  logfile.print(F("datetime" CSVSEP "light" CSVSEP "temp" CSVSEP "humid" CSVSEP ));
#if BME280_ON
  logfile.print(F("pressure" CSVSEP ));
#endif //BME280_ON  
  logfile.print(F("co2" CSVSEP ));
#if CCS811_ON  
  logfile.print(F("tvoc" CSVSEP ));
#endif //CCS811_ON  
  logfile.print(F("pm1std" CSVSEP "pm25std" CSVSEP "pm10std" CSVSEP "pm1atm" CSVSEP "pm25atm" CSVSEP "pm10atm"));  
  logfile.println(F(""));

#if ECHO_TO_SERIAL
  Serial.print(F("datetime,light,temp,humid,"));
#if BME280_ON
  Serial.print(F("pressure,"));
#endif //BME280_ON
  Serial.print(F("co2,"));
#if CCS811_ON
  Serial.print(F("tvoc,"));
#endif //CCS811_ON
#if HM3301_ON
Serial.print(F("pm1std,pm25std,pm10std,pm1atm,pm25atm,pm10atm"));
#endif //HM3301
Serial.println(F(""));
#endif //ECHO_TO_SERIAL

#if HM3301_ON
  delay(100);
  if (hm3301.init()) {
     Serial.print(F("HM330X init failed!!"));
     while (1);
  }
#endif //HM3301_ON
#if OLED_ON 
  button.attachClick(singleClick);
  button.attachLongPressStart(longPress);
#endif //#if OLED_ON 

#if OLED_ON
  delay(2000); // wait a bit longer before fetching data and avoid zero measurements
  oled.clear();
#endif // OLED_ON 
  currentMillis,lastLogMillis,lastSyncMillis,lastSyncMillis = millis();
  fetch_data();
#if OLED_ON    
    if ((currentMillis-lastOledMillis <= OLED_TIMEOUT)||(permaOled==true)){
      write_oled();        
    }
    else{
      oled.clear();      
    }    
#endif // OLED_ON  
  log_data();
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
#if !SCD30_ON //use temp/humid of SCD30 if enabled 
  data.humid = bme.readHumidity();
  data.temp = bme.readTemperature() + BME_TEMP_OFFSET;
#endif //!SCD30_ON 
  data.press = bme.readPressure();
#if CCS811_ON
  ccs.setEnvironmentalData(data.humid,data.temp); //send enviromnental data to the CSS811 for better compensation
#endif //CCS811_ON
#else //BME280_ON
#if CCS811_ON
  ccs.setEnvironmentalData(45,18); //set dummy temp & humidity 
#endif //CCS811_ON
#endif //BME280_ON

#if CCS811_ON
  //read CO2 and TVOC of CCS811
  if(ccs.available()){
   //float temp = ccs.calculateTemperature(); //That stuff is not working/unsupported by lib somehow
   if(!ccs.readData()){
    data.co2 = ccs.geteCO2();
    data.tvoc = ccs.getTVOC();}
   else{
    Serial.println(F("CCS811 read failed!"));
    while(1);}
  } 
#endif //CCS811_ON

#if SCD30_ON
  //read CO2, Temperature and Humidity
  if (scd30.dataReady()){
   if (!scd30.read()){ Serial.println(F("Error reading SCD30 sensor data")); return; }
   data.co2 = scd30.CO2;
   data.temp = scd30.temperature;
   data.humid = scd30.relative_humidity;
  }
#endif //SCD30_ON

#if HM3301_ON
  //read HM3301  
  uint8_t hm3301buf[30];
  uint16_t hm3301value = 0;
  if (hm3301.read_sensor_value(hm3301buf, 29)) {
    Serial.println(F("HM3301 read failed!"));
  }
  if (NULL == hm3301buf) {
    Serial.println(F("HM3301 no data!"));
    return false;
  }

  for (int i = 1; i < 8; i++) {
    hm3301value = (uint16_t) hm3301buf[i * 2] << 8 | hm3301buf[i * 2 + 1];
    if (i==1)
    data.sensnum=hm3301value;
    if (i==2)
    data.pm1std=hm3301value;
    if (i==3)
    data.pm25std=hm3301value;
    if (i==4)
    data.pm10std=hm3301value;
    if (i==5)
    data.pm1atm=hm3301value;
    if (i==6)
    data.pm25atm=hm3301value;
    if (i==7)
    data.pm10atm=hm3301value;
  }
#endif //HM3301_ON    
  return true;
}

bool log_data(void)
{
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

  logfile.print(CSVSEP); //csv separator
  logfile.print(data.light);
  logfile.print(CSVSEP); //csv separator
#if BME280_ON || SCD30_ON 
  logfile.print(data.temp);
  logfile.print(CSVSEP); //csv separator 
  logfile.print(data.humid);
  logfile.print(CSVSEP); //csv separator 
#if BME280_ON 
  logfile.print(data.press/100);
  logfile.print(CSVSEP); //csv separator     
#endif // BME280_ON  
#endif // BME280_ON || SCD30_ON 
#if CCS811_ON || SCD30_ON
  logfile.print(data.co2);
  logfile.print(CSVSEP); //csv separator 
#endif // CCS811_ON || SCD30_ON
#if CCS811_ON
  logfile.print(data.tvoc);
  logfile.print(CSVSEP);
#endif //CCS811_ON 
#if HM3301_ON

  logfile.print(data.pm1std);
  logfile.print(CSVSEP); //csv separator 
  logfile.print(data.pm25std);
  logfile.print(CSVSEP); //csv separator
  logfile.print(data.pm10std);
  logfile.print(CSVSEP); //csv separator
  logfile.print(data.pm1atm);
  logfile.print(CSVSEP); //csv separator
  logfile.print(data.pm25atm);
  logfile.print(CSVSEP); //csv separator
  logfile.print(data.pm10atm);
  //logfile.print(CSVSEP); //csv separator
#endif //HM3301_ON
  logfile.println();
#if ECHO_TO_SERIAL
  Serial.print(": "); 
  Serial.print(data.light);
  Serial.print("LDR,");    
#if BME280_ON || SCD30_ON 
  Serial.print(data.temp);
  Serial.print("C,"); 
  Serial.print(data.humid);
  Serial.print("pt,");   
#endif BME280_ON || SCD30_ON 
#if BME280_ON 
  Serial.print(data.press/100);
  Serial.print("hPa,"); 
#endif // BME280_ON 
#if CCS811_ON || SCD30_ON  
  Serial.print(data.co2);
  Serial.print("ppm,"); 
#endif //CCS811_ON || SCD30_ON
#if CCS811_ON
  Serial.print(data.tvoc);
  Serial.print("ppb,"); 
#endif //CCS811_ON
#if HM3301_ON
  Serial.print(data.pm1std);
  Serial.print("ug/m3,");
  Serial.print(data.pm25std);
  Serial.print("ug/m3,");
  Serial.print(data.pm10std);
  Serial.print("ug/m3,");
  Serial.print(data.pm1atm);
  Serial.print("ug/m3,");
  Serial.print(data.pm25atm);
  Serial.print("ug/m3,");
  Serial.print(data.pm10atm);
  Serial.print("ug/m3");
#endif //HM3301_ON
  Serial.println();
#endif //ECHO_TO_SERIAL
}

void write_oled()
{
#if OLED_ON
  //oled.clear();
  oled.setCursor(0,0);
  oled.println("--== CLIMAX 2001 ==--");
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
  oled.print("Temperature: ");
  oled.print(data.temp);  
  oled.print(" C");
  oled.clearToEOL();
  oled.println();  
  oled.print("Humidity: ");  
  oled.print(data.humid);  
  oled.print(" %");
  oled.clearToEOL();  
  oled.println(); 
  oled.print("Pressure: ");  
  oled.print(data.press/100);  
  oled.print(" hPA");
  oled.clearToEOL();
  oled.println(); 
  oled.print("C02: ");  
  oled.print(data.co2);  
  oled.print(" ppm");
  oled.clearToEOL();
  oled.println(); 
#if CCS811_ON
  oled.print("TVOC: ");  
  oled.print(data.tvoc); 
  oled.print(" ppb   "); //weirdly in this line a "q" is showing up out of nowhere
#else
  // No CCS811 - no TVOC. Therefor show something else useful.
  /*oled.print("Light: ");  
  oled.print(data.light); */
  oled.print("PM1(atm): ");  
  oled.print(data.pm1atm); 
  oled.print(" ug/m3 ");  
#endif //CCS811_ON
  oled.clearToEOL();  
  oled.println(); 
  oled.print("PM2.5(atm): ");  
  oled.print(data.pm25atm); 
  oled.print(" ug/m3 ");  
  oled.clearToEOL();
  //oled.println();   
#endif // OLED_ON 
}

#if OLED_ON 
void singleClick()
{
  Serial.println(F("singleClick"));  
  if(permaOled == true){
    permaOled = false;
    oled.clear();
    return;  
  }
  currentMillis = millis();
  if ((currentMillis-lastOledMillis <= OLED_TIMEOUT)){
    lastOledMillis = OLED_TIMEOUT+1;
  }
  else{
    lastOledMillis = currentMillis; 
  }
} 

void longPress()
{
  Serial.println(F("longPress"));
  if(permaOled==true){
    permaOled = false;
    oled.clear(); 
  }
  else{
    permaOled = true;
  }
}
#endif // OLED_ON 

void loop(void) 
{ 
  button.tick();
  currentMillis = millis();  //get the current time
  if(currentMillis-lastLogMillis >= LOG_INTERVAL){
    fetch_data();   
    log_data(); 
    lastLogMillis = currentMillis;  
  }  
#if OLED_ON    
    if ((currentMillis-lastOledMillis <= OLED_TIMEOUT)||(permaOled==true)){
      write_oled();        
    }
    else{
      oled.clear();      
    }    
#endif // OLED_ON
  
// Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time    
  if(currentMillis-lastSyncMillis >= SYNC_INTERVAL){
#if ECHO_TO_SERIAL    
    Serial.println(F("log_data")); 
#endif //ECHO_TO_SERIAL
    logfile.flush();
    lastSyncMillis = currentMillis;  
  } 
} 

#if DCF77
void DCF77_ISR(){
  unsigned int dur = 0;
  dur = millis() - dcfLastInt; 
  
  if(digitalRead(dcfInterruptPin)){
    if(dur>1500){
      if(dcfBufCounter==59){
        dcfEvaluateSequence();
      }
      dcfBufCounter = 0;
      dcfCurrentBuf = 0;
    }
  }
  else{
#if LEDMODE    
    digitalWrite(redLedPin, !digitalRead(redLedPin)); // toogle LED whenever we receive an interrupt
#endif //LEDMODE    
    if(dur>150){
      dcfCurrentBuf |= ((unsigned long long)1<<dcfBufCounter);
    }
    dcfBufCounter++;
  }
  dcfLastInt = millis();
}

void dcfEvaluateSequence(){
  Serial.println(F("dcf_eval")); 
  byte dcf77Year = (dcfCurrentBuf>>50) & 0xFF;    // year = bit 50-57
  byte dcf77Month = (dcfCurrentBuf>>45) & 0x1F;       // month = bit 45-49
  byte dcf77DayOfWeek = (dcfCurrentBuf>>42) & 0x07;   // day of the week = bit 42-44
  byte dcf77DayOfMonth = (dcfCurrentBuf>>36) & 0x3F;  // day of the month = bit 36-41
  byte dcf77Hour = (dcfCurrentBuf>>29) & 0x3F;       // hour = bit 29-34
  byte dcf77Minute = (dcfCurrentBuf>>21) & 0x7F;     // minute = 21-27 
  bool parityBitMinute = (dcfCurrentBuf>>28) & 1;
  bool parityBitHour = (dcfCurrentBuf>>35) & 1;
  bool parityBitDate = (dcfCurrentBuf>>58) & 1;

  if((parity_even_bit(dcf77Minute)) == parityBitMinute){
    if((parity_even_bit(dcf77Hour)) == parityBitHour){
      if(((parity_even_bit(dcf77DayOfMonth) + parity_even_bit(dcf77DayOfWeek) 
           + parity_even_bit(dcf77Month) + parity_even_bit(dcf77Year))%2) == parityBitDate){
        RTC.adjust(DateTime(rawByteToInt(dcf77Year) + 2000, rawByteToInt(dcf77Month), 
            rawByteToInt(dcf77DayOfMonth), rawByteToInt(dcf77Hour), rawByteToInt(dcf77Minute), 0));
#if LEDMODE
        digitalWrite(greenLedPin, !digitalRead(greenLedPin)); // toogle LED whenever we set the time
#endif //LEDMODE
       }
    }
  }
}

unsigned int rawByteToInt(byte raw){
  return ((raw>>4)*10 + (raw & 0x0F));
}
#endif //DCF77

void error(char *str)
{
  Serial.print(F("error: "));
  Serial.println(str);
  
  // red LED indicates error
#if LEDMODE  
  digitalWrite(redLedPin, HIGH);
#endif //LEDMODE

  while(1);
}
