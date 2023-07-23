//! Library Initialization Goes Here!
#include <Arduino.h>

//* Wifi Library
#include <WiFi.h>

//* Message Protocol Library
#include <PubSubClient.h>
#include <ArduinoJson.h>

//* Temperature Library
#include <OneWire.h>
#include <DallasTemperature.h>

//* TDS Library
#include "GravityTDS.h"

//* RTC Library
#include <SPI.h>
#include "RTClib.h"

//! Variable and PIN Initialization Goes Here!
//* MQTT Variable
#define MQTT_MSG_Buffer_Size (50)

//* TDS PIN
#define TDS_PIN 34

//* SSR PIN
#define SSR_PIN 26

//* Flow Meter Pin and Interrupt
#define sensorInterrupt 0
#define FLOW_PIN1 35
#define FLOW_PIN2 32

//* WiFi Initialization
const char *ssid = "Kuro";
const char *password = "kuro_1905";

//! Variable Instance Goes Here!
//* TimePoint and paralel time
long timePointLED = 1000; // Blink LED every 1s
unsigned long prevTimeLED = millis();

//* LED BUILTIN State
int LED_Builtin_State = LOW;

//* TDS Library Insatance
GravityTDS gravityTds;

//* Temperature Library Instance
OneWire oneWire(33);
DallasTemperature DS18B20_Sensor(&oneWire);

//* RTC Instance
RTC_DS1307 rtc_DS1307;

//* MQTT and WiFi Instance
WiFiClient espClient;
PubSubClient client(espClient);
const char *mqtt_server = "broker.emqx.io";
char msg[MQTT_MSG_Buffer_Size];
bool isSendToMqtt = false;

//* Global Variable TDS and Temperature
//* Initial Value for TDS and Temperature
float tdsValue = 0;
float temperatureValue = 0;
//* Caller Value for TDS and Temperature
float tdsValueResult;
float temperatureValueResult;

//* Flow Meter Counter Instance
volatile int flowMeterCount1;
volatile int flowMeterCount2;
//* Flow Meter Instance
//! You can change according to your datasheet
//! Flow pulse characteristics: F = (98*Q) ± 2% Q = L/min
float calibrationFactor = 98;
float flowRate1 = 0.0;
float flowRate2 = 0.0;
unsigned int flowMilliLitres1 = 0;
unsigned long totalMilliLitres1 = 0;
unsigned int flowMilliLitres2 = 0;
unsigned long totalMilliLitres2 = 0;
unsigned long flowMeterOldTime = 0;

//! Function Declaration Goes Here!
//* Function declaration for setupWifi
void setupWifi();

//* Function declaration for mqttReconnect
void mqttReconnect();

//* FUnction declaration for readTDS
void readTDS(float *tdsReadingResult, float *temperatureReadingResult);

//* Function declaration for sendToMqtt
void sendToMqtt();

//* Function declaration for IRAM_ATTR IRAMFlow
void IRAM_ATTR IRAMFlow1();
void IRAM_ATTR IRAMFlow2();

//* Function declaration for openSelenoidValve
void openSelenoidValve(int flowRate);

//* Function declaration for valveCallback
void valveCallback(char *topic, byte *message, unsigned int length);

//! MAIN PROGRAM START HERE!
void setup()
{
  Serial.begin(115200);

  if (!rtc_DS1307.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }

  //* WiFi Setup
  setupWifi();

  //* MQTT Setup
  client.setServer(mqtt_server, 1833);
  client.setCallback(valveCallback);

  //* Gravity TDS Setup
  gravityTds.setPin(TDS_PIN);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(4096);
  gravityTds.begin();

  //* Builtin LED Setup
  pinMode(BUILTIN_LED, OUTPUT);

  //* SSR Setup
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, HIGH);

  //* Flow Meter Setup
  pinMode(FLOW_PIN1, INPUT);
  pinMode(FLOW_PIN2, INPUT);
  attachInterrupt(FLOW_PIN1, IRAMFlow1, FALLING);
  attachInterrupt(FLOW_PIN2, IRAMFlow2, FALLING);
}

void loop()
{
  unsigned long millisCurrentTime = millis();

  // Blink Internal LED every 1s
  if (millisCurrentTime - prevTimeLED > timePointLED)
  {
    LED_Builtin_State = !LED_Builtin_State;
    digitalWrite(BUILTIN_LED, LED_Builtin_State);

    prevTimeLED = millisCurrentTime;
  }

  if (!client.connected())
  {
    mqttReconnect();
  }
  client.loop();

  DateTime rtcCurrentTime = rtc_DS1307.now();

  readTDS(&tdsValueResult, &temperatureValueResult);

  if (rtcCurrentTime.second() == 0 && !isSendToMqtt)
  {
    if (rtcCurrentTime.minute() % 1 == 0)
    {
      sendToMqtt();
      Serial.println("Send to mqtt");
      isSendToMqtt = true;
    }
  }
  else if (rtcCurrentTime.second() != 0)
  {
    isSendToMqtt = false;
  }
}
//! MAIN PROGRAM END HERE!

//! Function Definition Goes Here!
//* setupWifi() function definition
void setupWifi()
{
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

//* mqttReconnect() function definition
void mqttReconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection");
    String clientId = "ESP32Client-";
    clientId += String(random(0xfff), HEX);

    // Attemp to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println(" connected");

      //* client subscribe topic
      client.subscribe("heizou/valve/50");
      client.subscribe("heizou/valve/100");
      client.subscribe("heizou/valve/200");
    }
    else
    {
      Serial.print("failed, rc= ");
      Serial.print(client.state());
      Serial.println("retry in 5 second");
      delay(5000);
      mqttReconnect();
    }
  }
}

//* readTDS() function definition
void readTDS(float *tdsReadingResult, float *temperatureReadingResult)
{
  DS18B20_Sensor.requestTemperatures();
  temperatureValue = DS18B20_Sensor.getTempCByIndex(0);
  gravityTds.setTemperature(temperatureValue);
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();
  Serial.printf("Temperature: %.2fC ", temperatureValue);
  Serial.printf("| TDS Value: %.2fppm \n", tdsValue);

  *tdsReadingResult = tdsValue;
  *temperatureReadingResult = temperatureValue;
}

//* sendToMqtt() function definition
void sendToMqtt()
{
  DynamicJsonDocument doc(1024);
  doc["ppm"] = tdsValue;
  doc["temperature"] = temperatureValue;
  doc["source"] = "mqtt";

  char JSONMessageBuffer[1024];
  serializeJson(doc, JSONMessageBuffer);
  Serial.println("Sending message to MQTT tpic..");
  Serial.println(JSONMessageBuffer);

  if (client.publish("heizou/tdstemp", JSONMessageBuffer) == true)
  {
    Serial.println("Success sending message");
  }
  else
  {
    Serial.println("Error sending message");
  }
  Serial.println("-------------");
}

//* IRAM_ATTR IRAMFlow function definition
void IRAM_ATTR IRAMFlow1()
{
  flowMeterCount1++;
}

void IRAM_ATTR IRAMFlow2()
{
  flowMeterCount2++;
}

//* openSelenoidValve() function definition
void openSelenoidValve(int flowRate)
{
  digitalWrite(SSR_PIN, LOW);

  while (totalMilliLitres1 <= flowRate || totalMilliLitres2 <= flowRate)
  {
    // only process counters once per second
    if ((millis() - flowMeterOldTime) > 1000)
    {
      // Disable the interrupt while calculating flow rate and sending the value to the host
      detachInterrupt(FLOW_PIN1);
      detachInterrupt(FLOW_PIN2);

      /*
       * Because this loop may not complete in exactly 1 second intervals
       * we calculate the number of milliseconds that have passed since the last execution
       * and use that to scale the output.
       * We also apply the calibrationFactor to scale the output based on the number of pulses per second per units of measure
       * (litres/minute in this case) coming from the sensor.
       */
      flowRate1 = ((1000.0 / (millis() - flowMeterOldTime)) * flowMeterCount1) / calibrationFactor;
      flowRate2 = ((1000.0 / (millis() - flowMeterOldTime)) * flowMeterCount2) / calibrationFactor;

      /*
       * Note the time this processing pass was executed.
       * Note that because we've disabled interrupts the millis() function won't actually be incrementing right at this point,
       * but it will still return the value it was set to just before interrupts went away.
       */
      flowMeterOldTime = millis();

      /*
       * Divide the flow rate in litres/minute by 60 to determine how many litres have  passed through the sensor in this 1 second interval,
       * then multiply by 1000 to convert to millilitres.
       */
      flowMilliLitres1 = (flowRate1 / 60) * 1000;
      flowMilliLitres2 = (flowRate2 / 60) * 1000;

      //* Add the millilitres passed in this second to the cumulative total
      totalMilliLitres1 += flowMilliLitres1;
      totalMilliLitres2 += flowMilliLitres2;

      // Print the flow rate for this second in litres / minute
      Serial.print("Flow rate1: ");
      Serial.print(flowMilliLitres1, DEC); // Print the integer part of the variable
      Serial.print("mL/Second");
      Serial.print("\t");

      // Print the cumulative total of litres flowed since starting
      Serial.print("Output Liquid Quantity1: ");
      Serial.print(totalMilliLitres1, DEC);
      Serial.println("mL");
      Serial.print("\t");

      Serial.print("||");

      Serial.print("Flow rate2: ");
      Serial.print(flowMilliLitres2, DEC); // Print the integer part of the variable
      Serial.print("mL/Second");
      Serial.print("\t");

      Serial.print("Output Liquid Quantity2: ");
      Serial.print(totalMilliLitres2, DEC);
      Serial.println("mL");
      Serial.print("\t");

      // Reset the pulse counter so we can start incrementing again
      flowMeterCount1 = 0;
      flowMeterCount2 = 0;

      // Enable the interrupt again now that we've finished sending output
      attachInterrupt(FLOW_PIN1, IRAMFlow1, FALLING);
      attachInterrupt(FLOW_PIN2, IRAMFlow2, FALLING);
    }
  }

  digitalWrite(SSR_PIN, HIGH);
  totalMilliLitres1 = 0;
  totalMilliLitres2 = 0;

  detachInterrupt(FLOW_PIN1);
  detachInterrupt(FLOW_PIN2);

  readTDS(&tdsValueResult, &temperatureValueResult);
}

//* valveCallback() function definition
void valveCallback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on Topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  Serial.println();

  //*If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  //* Changes the output state according to the message
  if (String(topic) == "heizou/valve/50")
  {
    Serial.print("Opening Selenoid Valve 50ml");
    openSelenoidValve(50);
  }
  if (String(topic) == "heizou/valve/100")
  {
    Serial.print("Opening Selenoid Valve 100ml");
    openSelenoidValve(100);
  }
  if (String(topic) == "heizou/valve/200")
  {
    Serial.print("Opening Selenoid Valve 200ml");
    openSelenoidValve(200);
  }
}