//==============================================================
//  mikeWeatherStation
//  Mike McPherson
//  November 2013
//==============================================================

//=================================
// GPRS
//=================================
#define SERVER   "wxftp.mmcpix.com"
#define USERNAME "wxd4t4"
#define PASSWORD "w34th3r"
#define FTPPATH  "/"
#define WX_ID    "Holmbeck"

//=================================
// barometer
//
// uses SCL/SDA pins
//=================================
#include "Barometer.h"
#include <Wire.h>
Barometer barometer;

//=================================
// wind
//=================================
#define WINDDIRPIN A1
#define WINDSPEEDPIN 3

//=================================
// ticks from anemometer
//=================================
int windTicks;

//=================================
// flag to show interrupt was from wind
//=================================
boolean windFlag;

//=================================
// temp & humidity
//=================================
#include <DHT22.h>
#define DHTPIN     A3
DHT22 dht(DHTPIN);

//=================================
// Grove power
//=================================
#define GROVEPOWERPIN    6

//=================================
// battery voltage
//=================================
#define BATVOLTPIN A7
#define BATVOLT_R1      10              // voltage divider R1 = 10M
#define BATVOLT_R2      2               // voltage divider R2 = 2M
#define ADC_AREF        3.3

//=================================
//RTC
//=================================
#include <Sodaq_DS3231.h>

//==============================================
// loop control
//
// every X seconds do wind direction calculation
// every Y seconds write data record to flash
// every Z seconds upload data to server
//==============================================
#define WIND_INTERVAL     1
#define RECORD_INTERVAL  10
#define UPLOAD_INTERVAL  60
uint32_t windTime;        // epoch time for next wind calculation
uint32_t recordTime;      // epoch time for next record write
uint32_t uploadTime;      // epoch time for next data upload
unsigned long startTime;  // used for two-second sensor warm-up time (non-blocking)
uint32_t epochRecord;     // epoch time at start of record processing

//=================================
// sleep info
//=================================
#include <avr/sleep.h>
#include <avr/wdt.h>

//=================================
// flag to show interrupts was from watchdog
//=================================
boolean wdtFlag;

//=================================
// diagnostics/debug support
// define to use software serial
// comment out to use "real" Serial
//=================================
#define SOFT_SERIAL 1

#ifdef SOFT_SERIAL
#define DIAGPORT_RX     4
#define DIAGPORT_TX     5
#include <SoftwareSerial.h>
SoftwareSerial diagport(DIAGPORT_RX, DIAGPORT_TX);
#else
#define diagport Serial
#endif

//=================================
// diagnostics/debug support
// define for diagnostics
// comment out for none
//=================================
#define ENABLE_DIAG 1

#ifdef ENABLE_DIAG
#define DIAGPRINT(...)          diagport.print(__VA_ARGS__)
#define DIAGPRINTLN(...)        diagport.println(__VA_ARGS__)
#else
#define DIAGPRINT(...)
#define DIAGPRINTLN(...)
#endif

//=================================
// data record for writing results to flash
// and uploading to server
//=================================
#include <Sodaq_dataflash.h>
#include "Diag.h"
#include "DataRecord.h"
myRecord dataRecord;

//=====================================================================================
// Arduino setup
//=====================================================================================
void setup() 
{
  // start the serial (debug) port
  //==============================
  initSerial();

  // annonunce self to world
  //========================
  DIAGPRINTLN();
  separator(40);
  DIAGPRINTLN(F("Sodaq weather station v1.01"));
  DIAGPRINT(F("initialising "));
  
  // initialize the Grove power pin
  //===============================
  pinMode(GROVEPOWERPIN, OUTPUT);     

  // set Grove power on
  //===================
  groveOn();

  // initialise RTC
  //===============
  initClock();
  
  // initialise humidity
  //====================
  humidityInit();

  // initialise barometer
  //=====================
  barometerInit();

  // initialise wind
  //================
  initWind();

  // initialise timers
  //==================
  initTimers();
  
  // show timestamp
  //===============
  DIAGPRINTLN();
  showTimestamp();
  
  // clear the interrupt flags
  //==========================
  wdtFlag = false;
  windFlag = false;
  startTime = 0;

  // set Grove power off
  //====================
  groveOff();
  
  // show amount of RAM free
  //========================
  showRAM();
  separator(40);

  // start the watchdog timer
  //=========================
  sleepInit();

  // ... and sleep
  //==============
  systemSleep();
}

//=====================================================================================
// main code loop
// we wake every four seconds
//
// do nothing for two seconds to give sensors time to warm up
//=====================================================================================
void loop()
{
  uint32_t epoch;
  
  // get time from RTC
  //==================
  epoch = rtc.now().getEpoch();
  
  if (epoch >= windTime)
  { 
    // time to do wind calculation
    //============================
    showTime();
    DIAGPRINTLN(F("wind"));
    windTime = epoch + WIND_INTERVAL;
  }
  
  if (epoch >= recordTime)
  { 
    if (startTime == 0)
    {
      startTime = millis();
      
      // set Grove power on
      //===================
      groveOn();

      // save the epoch time
      //====================
      epochRecord = epoch;
    }
    
    // pause while the sensors warm up
    //================================
    if ( (millis() - startTime) > 2000)
    {
      
      // time to write out a record
      //===========================
      showTime();
      DIAGPRINTLN(F("data record"));
      
      separator(40);
    
      showTimestamp();
      
      batteryGetVoltage();
      batteryShowVoltage();
      DIAGPRINTLN();
    
      barometerGetPressure();
      barometerShow();
      DIAGPRINTLN();
      
      showWind();
      DIAGPRINTLN();
      
      humidityGetHumidity();
      humidityShow();

      separator(40);
    
      // set Grove power off
      //====================
      groveOff();
      
      // write data record to flash memory
      //==================================
      writeDataRecord(epochRecord);
  
      // reset the timers
      //=================
      recordTime = epochRecord + RECORD_INTERVAL;
      startTime = 0;
    }
  }
  
  if (epoch >= uploadTime)
  { 
    // time to upload data to the server
    //==================================
    showTime();
    DIAGPRINTLN(F("upload"));
    uploadTime = epoch + UPLOAD_INTERVAL;
  }

  if (startTime == 0)
  {
    // we've finished with the sensor warm up
    // so now we can sleep again
    // watchDog timer will wake us up in a bit
    //========================================
    systemSleep();
  }
}

//=====================================================================================
// routine to initialise barometer sensor
//=====================================================================================
void barometerInit(void)
{
  barometer.init();
  DIAGPRINT("b");
}

//=====================================================================================
// routine to get pressure from barometer
//=====================================================================================
void barometerGetPressure(void)
{
  // get the pressure
  //=================
  dataRecord.pressure = barometer.bmp085GetPressure(barometer.bmp085ReadUP());
  
  // get the temperature
  //====================
  dataRecord.temperature = barometer.bmp085GetTemperature(barometer.bmp085ReadUT()); 
}

//=====================================================================================
// routine to show current Temperature and Pressure
//=====================================================================================
void barometerShow(void)
{
  float hPa;

  hPa = dataRecord.pressure / 100;
  
  DIAGPRINT(F("Temperature: "));
  DIAGPRINT(dataRecord.temperature, 2); //display 2 decimal places
  DIAGPRINTLN(F(" C"));

  DIAGPRINT(F("   Pressure: "));
  DIAGPRINT(hPa, 0); //whole number only.
  DIAGPRINTLN(F(" hPa"));
}

//=====================================================================================
// routine to get current Battery voltage
//=====================================================================================
void batteryGetVoltage(void)
{
  // read the raw voltage
  //=====================
  dataRecord.batteryVoltage = analogRead(BATVOLTPIN);
}

//=====================================================================================
// routine to show current Battery voltage
//=====================================================================================
void batteryShowVoltage(void)
{
  float batteryVoltage;
  
  //=============================================================
  // This pin is connected to the middle of a 10M and 2M resistor
  // that are between Vcc and GND.
  // So actual battery voltage is:
  //    (<adc value> * Aref / 1023) * (R1+R2) / R2
  //=============================================================
  batteryVoltage = (dataRecord.batteryVoltage * ADC_AREF / 1023) * (BATVOLT_R1+BATVOLT_R2) / BATVOLT_R2;
  DIAGPRINT(F("    Battery: "));
  DIAGPRINT(batteryVoltage);
  DIAGPRINTLN(F(" V"));
}

//=====================================================================================
// Returns the number of bytes currently free in RAM
//=====================================================================================
static int freeRAM(void) 
{
  extern int  __bss_end;
  extern int* __brkval;
  int free_memory;
  if (reinterpret_cast<int>(__brkval) == 0) 
  {
    // if no heap use from end of bss section
    //=======================================
    free_memory = reinterpret_cast<int>(&free_memory) - reinterpret_cast<int>(&__bss_end);
  } else 
  {
    // use from top of stack to heap
    //==============================
    free_memory = reinterpret_cast<int>(&free_memory) - reinterpret_cast<int>(__brkval);
  }
  return free_memory;
}

//=====================================================================================
// power down switchable Grove sockets
//=====================================================================================
void groveOff(void)
{
  // set Grove power off
  //====================
  digitalWrite(GROVEPOWERPIN, LOW);
}

//=====================================================================================
// power up switchable Grove sockets
//=====================================================================================
void groveOn(void)
{
  // set Grove power on
  //===================
  digitalWrite(GROVEPOWERPIN, HIGH);
}

//=====================================================================================
// interrupt handler for wind ticks
//=====================================================================================
void handleWindTick()
{
  // simply increment the counter
  //=============================
  windTicks++;
  windFlag = true;
}

//=====================================================================================
// routine to initialise humidity sensor
//=====================================================================================
void humidityInit(void)
{
  // initialise humidity
  // just needs 2 seconds to get warmed up!
  //=======================================
  delay(2000);
  DIAGPRINT("h");
}

//=====================================================================================
// routine to get humidity and temperature from humidity sensor
//=====================================================================================
void humidityGetHumidity(void)
{
  DHT22_ERROR_t errorCode;
  
  errorCode = dht.readData();
  switch(errorCode)
  {
    case DHT_ERROR_NONE:
      // get raw readings
      //=================
      dataRecord.humidity = dht.getHumidity();
      dataRecord.temperatureH = dht.getTemperatureC();
      break;
    case DHT_ERROR_CHECKSUM:
      DIAGPRINTLN(F("checksum error"));
      break;
    case DHT_BUS_HUNG:
      DIAGPRINTLN(F("BUS Hung"));
      break;
    case DHT_ERROR_NOT_PRESENT:
      DIAGPRINTLN(F("Not Present"));
      break;
    case DHT_ERROR_ACK_TOO_LONG:
      DIAGPRINTLN(F("ACK time out"));
      break;
    case DHT_ERROR_SYNC_TIMEOUT:
      DIAGPRINTLN(F("Sync Timeout"));
      break;
    case DHT_ERROR_DATA_TIMEOUT:
      DIAGPRINTLN(F("Data Timeout"));
      break;
    case DHT_ERROR_TOOQUICK:
      DIAGPRINTLN(F("Polled to quick"));
      break;
  }
}

//=====================================================================================
// routine to show current Humidity from humidity sensor
//=====================================================================================
void humidityShow()
{
  float dewPoint;
  int cloudBase;
        
  // calculate dew point
  //====================
  dewPoint = dataRecord.temperatureH - (100.0 - dataRecord.humidity) / 5.0;
  
  // calculate cloud base (to nearest 100')
  //==============================
  cloudBase = (dataRecord.temperatureH - dewPoint) * 5;
  cloudBase = cloudBase * 100;

  DIAGPRINT(F("Temperature: "));
  DIAGPRINT(dataRecord.temperatureH);
  DIAGPRINTLN(F(" C"));
  DIAGPRINT(F("   Humidity: "));
  DIAGPRINT(dataRecord.humidity);
  DIAGPRINTLN(F(" %"));
  DIAGPRINT(F("  Dew point: "));
  DIAGPRINT(dewPoint);
  DIAGPRINTLN(F(" C"));
  DIAGPRINT(F("  Cloudbase: "));
  DIAGPRINT(cloudBase);
  DIAGPRINTLN(F(" ft est"));
}

//=====================================================================================
// watchDogTimer interrupt
//=====================================================================================
ISR(WDT_vect)
{
  // just set the watchdog flag
  //===========================
  wdtFlag = true;
}

//=====================================================================================
// routine to initialise RTC (RealTime Clock)
//=====================================================================================
void initClock()
{
  // adjust this call to set your date/time/dayOfWeek
  // year, month, day, hour, minute, second, dayOfWeek
  //==================================================
  DateTime dt(2013, 11, 28, 10, 46, 0, 4);

  // initialise the clock  
  //=====================
  rtc.begin();

  // un-comment the next call to set the date/time
  //==============================================
  if (0)
    rtc.setDateTime(dt);

  DIAGPRINT("r");
}

//=====================================================================================
// routine to initialise serial port
//=====================================================================================
void initSerial(void)
{
#ifdef ENABLE_DIAG
  // initialise serial port for diagnostics
  //=======================================
  diagport.begin(19200);
#endif
}

void initTimers(void)
{
  uint32_t epoch;
  
  // initialise timers to current clock setting
  //===========================================
  epoch = rtc.now().getEpoch();
  windTime = epoch + WIND_INTERVAL;
  recordTime = epoch + RECORD_INTERVAL;
  uploadTime = epoch + UPLOAD_INTERVAL;
}

//=====================================================================================
// routine to initialise wind sensors
//=====================================================================================
void initWind(void)
{
  windTicks = 0;
  pinMode(WINDSPEEDPIN, INPUT_PULLUP);

  // Sodaq Moja INT0 => D2
  //======================
  attachInterrupt(0, handleWindTick, FALLING);          
  
  // ensure interrupts are enabled
  //==============================
  interrupts();

  DIAGPRINT("w");
}

void separator(int count)
{
  // print a separator
  //==================
  for (int i = 0; i<count; i++)
  DIAGPRINT("=");
  DIAGPRINTLN();
}

void showEpoch(uint32_t epoch)
{
//=====================================================================================
// routine to show time
//=====================================================================================
  DIAGPRINT(epoch);
  DIAGPRINT(" ");
}

//=====================================================================================
// show amount of RAM free
//=====================================================================================
void showRAM(void)
{
  DIAGPRINT(F("   free RAM: ")); 
  DIAGPRINT(freeRAM()); 
  DIAGPRINTLN(F(" bytes"));
}

//=====================================================================================
// routine to show time
//=====================================================================================
void showTime(void)
{
  int hour;
  int minute;
  int second;
  DateTime now;

  // show date/time
  //===============
  now = rtc.now();
  hour = now.hour();
  minute = now.minute();
  second = now.second();

  if (hour<10)
    DIAGPRINT("0");
  DIAGPRINT(hour, DEC);
  DIAGPRINT(':');
  if (minute<10)
    DIAGPRINT("0");
  DIAGPRINT(minute, DEC);
  DIAGPRINT(':');
  if (second<10)
    DIAGPRINT("0");
  DIAGPRINT(second, DEC);
  DIAGPRINT(' ');
}

//=====================================================================================
// routine to show timestamp and RTC temperature
//=====================================================================================
void showTimestamp(void)
{
  char months[][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  char wkdays[][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

  float temp;
  int day;
  int hour;
  int minute;
  int second;
  DateTime now;
    
  // show date/time
  //===============
  now = rtc.now();
  day = now.date();
  hour = now.hour();
  minute = now.minute();
  second = now.second();
  
  DIAGPRINT(F("  Timestamp: "));
  DIAGPRINT(wkdays[now.dayOfWeek()]);
  DIAGPRINT(' ');
  if (day<10)
    DIAGPRINT("0");
  DIAGPRINT(day, DEC);
  DIAGPRINT('-');
  DIAGPRINT(months[now.month()-1]);
  DIAGPRINT('-');
  DIAGPRINT(now.year(), DEC);
  DIAGPRINT(' ');
  
  showTime();
  DIAGPRINTLN();
  
  temp = rtc.getTemperature();
  DIAGPRINT(F("   RTC temp: "));
  DIAGPRINT(temp);
  DIAGPRINTLN(F(" C"));
}

//=====================================================================================
// routine to show current Wind settings
//=====================================================================================
void showWind(void)
{
  int windRaw;
  float windSpeed;
  int myTicks;
  
  int  direction[] = {    0,    74,    88,   110,   156,   215,  267,   349,   436,   533,   617,   669,   747,   809,   859,   918, 1024};
  char points[][4] = {"ESE", "ENE", "E  ", "SSE", "SE ", "SSW", "S  ", "NNE", "NE ", "WSW", "SW ", "NNW", "N  ", "WNW", "NW ", "W  "};
  int i;
  
  // read raw data
  //==============
  windRaw = analogRead(WINDDIRPIN);
  
  DIAGPRINT(F("   Wind dir: "));

  // convert to direction
  //=====================
  for (i=0; i<sizeof(direction); i++)
  {
    if (windRaw > direction[i] && windRaw < direction[i+1])
    {
        DIAGPRINT(points[i]);
        break;
    }
  }
  DIAGPRINT(F(" ("));
  DIAGPRINT(windRaw);
  DIAGPRINTLN(F(")"));
  
  // convert windTicks to wind speed
  //================================
  cli();
  myTicks = windTicks;
  windTicks = 0;
  sei();
  windSpeed = myTicks * 0.6;
  
  DIAGPRINT(F(" Wind speed: "));
  DIAGPRINT(windSpeed, 1);
  DIAGPRINT(F(" ("));
  DIAGPRINT(myTicks);
  DIAGPRINTLN(F(")"));
}

//=====================================================================================
// prepare the watch dog timer
//=====================================================================================
void sleepInit(void)
{
  // clear various "reset" flags
  //============================
  MCUSR = 0;     
    
  // allow changes, disable reset
  //=============================
  WDTCSR = bit (WDCE) | bit (WDE);
  
  // set interrupt mode and interval
  //
  // WDP3 WDP2 WDP1 WDP0 timer   ~seconds
  //    0    0    0    0   16 mS
  //    0    0    0    1   32 mS
  //    0    0    1    0   64 mS
  //    0    0    1    1  128 mS 0.125
  //    0    1    0    0  256 mS 0.25
  //    0    1    0    1  512 mS 0.5
  //    0    1    1    0 1024 mS 1
  //    0    1    1    1 2048 mS 2
  //    1    0    0    0 4096 mS 4
  //    1    0    0    1 8192 mS 8
  //
  //============================================
  WDTCSR = bit (WDIE) | bit (WDP2) | bit (WDP1);
  
  // pat the dog
  //============
  wdt_reset();

  // set sleep mode
  //===============
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
}

//=====================================================================================
// set up sleep mode and sleep
//=====================================================================================
void systemSleep()
{
  // disable ADC
  //============
  ADCSRA &= ~_BV(ADEN);
  
  // turn off brown-out enable in software
  //======================================
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS); 
  sleep_cpu();

  // sleep...
  //=========
  sleep_mode(); 

  // ... and wake-up
  //================

  // enable ADC
  //===========
  ADCSRA |= _BV(ADEN);
}

void writeDataRecord(uint32_t epoch)
{   
  // write data record to flash
  //===========================
}

