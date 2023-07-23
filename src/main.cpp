#include <Arduino.h>

//* Temperature Lib
#include <OneWire.h>
#include <DallasTemperature.h>

//* TDS Lib
#include "GravityTDS.h"

//* RTC Lib
#include <SPI.h>
#include "RTClib.h"

//* TDS PIN
#define TDS_PIN 34

//* TimePoint and paralel time
unsigned long currentTime = millis();
long timePointLED = 1000; // Blink LED every 1s
unsigned long prevTimeLED = millis();

// LED BUILTIN State
int LED_Builtin_State = LOW;

// TDS Library Insatance
GravityTDS gravityTds;

// Temperature Library Instance
OneWire oneWire(33);
DallasTemperature DS18B20_Sensor(&oneWire);

// RTC Instance
RTC_DS1307 rtc_DS1307;

// Global Var TDS and Temperature
float tdsValue = 0;
float temperatureValue = 0;

//* FUnction declaration for readTDS
void readTDS();

void setup()
{
  Serial.begin(115200);

  if (!rtc_DS1307.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }

  //! Gravity TDS Setup
  gravityTds.setPin(TDS_PIN);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(4096);
  gravityTds.begin();

  //! Builtin LED Setup
  pinMode(BUILTIN_LED, OUTPUT);
}

void loop()
{
  // Blink Internal LED every 1s
  if (currentTime - prevTimeLED > timePointLED)
  {
    LED_Builtin_State = !LED_Builtin_State;
    digitalWrite(BUILTIN_LED, LED_Builtin_State);

    prevTimeLED = currentTime;
  }

  DateTime currentTime = rtc_DS1307.now();

  readTDS();
}

//* readTDS() function definition
void readTDS()
{
  DS18B20_Sensor.requestTemperatures();
  temperatureValue = DS18B20_Sensor.getTempCByIndex(0);
  gravityTds.setTemperature(temperatureValue);
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();
  Serial.printf("Temperature: %.2fC ", temperatureValue);
  Serial.printf("| TDS Value: %.2fppm \n", tdsValue);
}