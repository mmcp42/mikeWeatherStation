//==============================================================
//  mikeWeatherStation
//  Mike McPherson
//  November 2013
//==============================================================

//=================================
// barometer
//=================================
#include "Barometer.h"
#include <Wire.h>
Barometer barometer;

//=================================
// wind
//=================================
#define WINDDIRPIN A0
#define WINDSPEEDPIN 3
volatile int windTicks;

//=================================
// temp & humidity
//=================================
#include <DHT22.h>
#define DHTPIN     4
DHT22 dht(DHTPIN);

//=================================
// flashing LED
//=================================
#define LEDPIN      6

//=================================
// battery voltage
//=================================
#define BATVOLTPIN A7
#define BATVOLT_R1      10              // in fact 10M
#define BATVOLT_R2      2               // in fact 2M
#define ADC_AREF        3.3

//=================================
//RTC
//=================================
#include <Sodaq_DS3231.h>

//=================================
// loop timer (in seconds)
//=================================
#define INTERVAL 10
unsigned long loopTimer;

//=====================================================================================
// Arduino setup
//=====================================================================================
void setup() 
{                
  // initialise serial port for diagnostics
  //=======================================
  Serial.begin(19200);
  Serial.println("Sodaq basic test v1.01");
  Serial.print("initialising ");
  
  // initialize the LED pin
  //=======================
  pinMode(LEDPIN, OUTPUT);     

  // initialise RTC
  //===============
  initClock();
  
  // initialise humidity
  //====================
  initHumidity();

  // initialise barometer
  //=====================
  initBarometer();

  // initialise wind
  //================
  initWind();

  // initialise loop timer
  //======================
  loopTimer = millis() - INTERVAL * 1000;
  
  Serial.println();
  Serial.println("ready");
}

//=====================================================================================
// main code loop
//=====================================================================================
void loop()
{
  if ((millis() - loopTimer) >= INTERVAL * 1000)
  {
    // time to check the sensors again
    //================================
    loopTimer = millis();

    // set the LED on
    //===============
    digitalWrite(LEDPIN, HIGH);
    
    Serial.println("=================================");
    showTimestamp();
        
    showBatteryVoltage();
  
    showHumidity();
  
    showPressure();
    
    showWind();
    
    // set the LED off
    //================
    digitalWrite(LEDPIN, LOW);    
  }
}

//=====================================================================================
// interrupt handler for wind ticks
//=====================================================================================
void handleWindTick()
{
  // simply increment the counter
  //=============================
  windTicks++;
}

//=====================================================================================
// routine to show initialise barometer sensor
//=====================================================================================
void initBarometer(void)
{
  barometer.init();
  Serial.print("b");
}

//=====================================================================================
// routine to show initialise RTC (RealTime Clock)
//=====================================================================================
void initClock()
{
  // adjust this call to set your date/time/dayOfWeek
  // year, month, day, hour, minute, second, dayOfWeek
  //==================================================
  DateTime dt(2013, 11, 22, 21, 22, 0, 5);

  // initialise the clock  
  //=====================
  rtc.begin();

  // un-comment the next call to set the date/time
  //==============================================
  if (0)
    rtc.setDateTime(dt);

  Serial.print("r");
}

//=====================================================================================
// routine to show initialise humidity sensor
//=====================================================================================
void initHumidity(void)
{
  // initialise humidity
  // just needs 2 seconds to get warmed up!
  //=======================================
  delay(2000);
  Serial.print("h");
}

//=====================================================================================
// routine to show initialise wind sensors
//=====================================================================================
void initWind(void)
{
  windTicks = 0;
  pinMode(WINDSPEEDPIN, INPUT_PULLUP);

  // Sodaq Moja INT1 => D3
  //======================
  attachInterrupt(1, handleWindTick, FALLING);          
  
  // ensure interrupts are enabled
  //==============================
  interrupts();

  Serial.print("w");
}

//=====================================================================================
// routine to show current Battery voltage
//=====================================================================================
void showBatteryVoltage(void)
{
  int adc;
  float batteryVoltage;
  
  /*
   * This pin is connected to the middle of a 10M and 2M resistor
   * that are between Vcc and GND.
   * So actual battery voltage is:
   *    (<adc value> * Aref / 1023) * (R1+R2) / R2
   */
  adc = analogRead(BATVOLTPIN);
  batteryVoltage = (adc * ADC_AREF / 1023) * (BATVOLT_R1+BATVOLT_R2) / BATVOLT_R2;
  Serial.print("    Battery: ");
  Serial.print(batteryVoltage);
  Serial.println(" v");
  Serial.println();
}

//=====================================================================================
// routine to show current Temperature and Humidity
//=====================================================================================
void showHumidity(void)
{
  DHT22_ERROR_t errorCode;
  float temperature;
  float humidity;
  float dewPoint;
  int cloudBase;
  
  // The sensor can only be read from every 1-2s, and requires a minimum
  // 2s warm-up after power-on.
  //====================================================================  
  errorCode = dht.readData();
  switch(errorCode)
  {
    case DHT_ERROR_NONE:
      // get raw readings
      //=================
      temperature = dht.getTemperatureC();
      humidity = dht.getHumidity();
      
      // calculate dew point
      //====================
      dewPoint = temperature - (100.0 - humidity) / 5.0;
      
      // claculTE CLOUD BASE (TO 100')
      //==============================
      cloudBase = (temperature - dewPoint) * 5;
      cloudBase = cloudBase * 100;
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" C");
      Serial.print("   Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
      Serial.print("  Dew point: ");
      Serial.print(dewPoint);
      Serial.println(" C");
      Serial.print("  Cloudbase: ");
      Serial.print(cloudBase);
      Serial.println(" ft");
      break;
    case DHT_ERROR_CHECKSUM:
      Serial.print("check sum error ");
      Serial.print(dht.getTemperatureC());
      Serial.print("C ");
      Serial.print(dht.getHumidity());
      Serial.println("%");
      break;
    case DHT_BUS_HUNG:
      Serial.println("BUS Hung ");
      break;
    case DHT_ERROR_NOT_PRESENT:
      Serial.println("Not Present ");
      break;
    case DHT_ERROR_ACK_TOO_LONG:
      Serial.println("ACK time out ");
      break;
    case DHT_ERROR_SYNC_TIMEOUT:
      Serial.println("Sync Timeout ");
      break;
    case DHT_ERROR_DATA_TIMEOUT:
      Serial.println("Data Timeout ");
      break;
    case DHT_ERROR_TOOQUICK:
      Serial.println("Polled to quick ");
      break;
  }
  Serial.println();
}

//=====================================================================================
// routine to show current Temperature and Pressure
//=====================================================================================
void showPressure(void)
{
  float temperature;
  float pressure;
  float hPa;

  //Get the temperature, bmp085ReadUT MUST be called first
  //======================================================
  temperature = barometer.bmp085GetTemperature(barometer.bmp085ReadUT()); 
  
  //Get the temperature
  //===================
  pressure = barometer.bmp085GetPressure(barometer.bmp085ReadUP());       
  hPa = pressure / 100;
  
  Serial.print("Temperature: ");
  Serial.print(temperature, 2); //display 2 decimal places
  Serial.println(" C");

  Serial.print("   Pressure: ");
  Serial.print(hPa, 0); //whole number only.
  Serial.println(" hPa");
  Serial.println();
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
  
  Serial.print("  Timestamp: ");
  Serial.print(wkdays[now.dayOfWeek()]);
  Serial.print(' ');
  if (day<10)
    Serial.print("0");
  Serial.print(day, DEC);
  Serial.print('-');
  Serial.print(months[now.month()-1]);
  Serial.print('-');
  Serial.print(now.year(), DEC);
  Serial.print(' ');
  if (hour<10)
    Serial.print("0");
  Serial.print(hour, DEC);
  Serial.print(':');
  if (minute<10)
    Serial.print("0");
  Serial.print(minute, DEC);
  Serial.print(':');
  if (second<10)
    Serial.print("0");
  Serial.print(second, DEC);
  Serial.println();
  
  temp = rtc.getTemperature();
  Serial.print("   RTC temp: ");
  Serial.print(temp);
  Serial.println(" C");
}

//=====================================================================================
// routine to show current Wind settings
//=====================================================================================
void showWind(void)
{
  int windRaw;
  
  windRaw = analogRead(WINDDIRPIN);
  
  Serial.print("   Wind dir: ");
  Serial.print(windRaw);
  Serial.println(" raw");

  Serial.print(" Wind speed: ");
  Serial.print(windTicks);
  Serial.println(" raw");
  windTicks = 0;
}

