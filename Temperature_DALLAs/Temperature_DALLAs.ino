/* 
 *  Dallas Temperature DS18B20 sensor example with ESP8266:
 *  v.1.1 
 *  
 *  // error pembacaan data awal, suhu = 1.35
 *  
 *  1. Publish message every SEND_INTERVAL to "/sensor/temp" for temperature.
 *  2. Connect DS18B20 data out to GPIO0.
 *  3. This example use OneWire library and DallasTemperature library, hence you must install them first.
 *  4. Publish first message in 1 minute, can change in line 28, the code in line 98
 *  
 */
#include "M1128.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define DEBUG true
#define DEBUG_BAUD 9600

#define DEVELOPER_ROOT "1"
#define DEVELOPER_USER "dmI0OkvoFRLRzHu3J3tEWQbIXQwDeF9q"
#define DEVELOPER_PASS "dyUiAb1cjkS8FRrokTXxtY1s4DUmOJsa"

#define WIFI_DEFAULT_SSID "SmartDallasTemperature"
#define WIFI_DEFAULT_PASS "abcd1234"

#define DEVICE_PIN_DALLAS 25
#define DEVICE_PIN_DALLAS1 33
#define DEVICE_PIN_RESET  3
//#define SEND_INTERVAL     250     // send data to mqtt broker interval
#define SEND_INTERVAL     120000
#define SEND_FIRST_DATA   60000     //data baru dikirim setelah 60 detik device on    

HardwareSerial *SerialDEBUG = &Serial;
M1128 iot;

OneWire oneWire(DEVICE_PIN_DALLAS);
DallasTemperature sensors(&oneWire);

OneWire oneWire1(DEVICE_PIN_DALLAS1);
DallasTemperature sensors1(&oneWire);

const float filterWeight = 0.05; //range 0 - 1
unsigned int sensorCurMillis = 0;
unsigned int sensorPrevMillis = 0;
unsigned long int dataStart = 0;
char resultT[7]; // Buffer big enough for 6-character float
float lastSensor = 0;
int8_t errSensor = 0;
float currentSensor;

char resultT1[7]; // Buffer big enough for 6-character float
float lastSensor1 = 0;
int8_t errSensor1 = 0;
float currentSensor1;

void setup() {
  if (DEBUG) {
//    SerialDEBUG->begin(DEBUG_BAUD, SERIAL_8N1, SERIAL_TX_ONLY); // for ESP8266
    SerialDEBUG->begin(DEBUG_BAUD, SERIAL_8N1); // for ESP32
    while (!SerialDEBUG);
    SerialDEBUG->println(F("Initializing.."));
  }
  pinMode(DEVICE_PIN_DALLAS,INPUT_PULLUP);
  pinMode(DEVICE_PIN_RESET, FUNCTION_3);
  iot.pinReset = DEVICE_PIN_RESET;
  iot.prod = true;
  iot.cleanSession = true;
  iot.setWill = true;
  iot.apConfigTimeout = 300000;
  iot.wifiConnectTimeout = 120000;
  iot.devConfig(DEVELOPER_ROOT,DEVELOPER_USER,DEVELOPER_PASS);
  iot.wifiConfig(WIFI_DEFAULT_SSID,WIFI_DEFAULT_PASS);
  
  iot.onReceive = callbackOnReceive;
  iot.onConnect = callbackOnConnect;
  iot.onReconnect = callbackOnReconnect;
  iot.onAPConfigTimeout = callbackOnAPConfigTimeout;
  iot.onWiFiConnectTimeout = callbackOnWiFiConnectTimeout;  
  
  sensors.begin();
  sensors1.begin();
  iot.init(DEBUG?SerialDEBUG:NULL);
  measureSensor();
  delay(100);
}

void loop() {
  yield();
  iot.loop();
  measureSensor();
  measureSensor1();
  sendData();
}

void measureSensor() {
  sensors.requestTemperatures();
  currentSensor = sensors.getTempCByIndex(0); //collect sample

  if (currentSensor==-127) errSensor = -1;
  else if (currentSensor==127) errSensor = 1;
  else errSensor = 0;
  if (errSensor==0) { //only if normal, skip if error
    // do filtering
    lastSensor = (filterWeight * currentSensor) + ((1-filterWeight) * lastSensor); 
  }
}
void measureSensor1() {
  sensors1.requestTemperatures();
  currentSensor1 = sensors.getTempCByIndex(0); //collect sample

  if (currentSensor1==-127) errSensor = -1;
  else if (currentSensor1==127) errSensor = 1;
  else errSensor1 = 0;
  if (errSensor1==0) { //only if normal, skip if error
    // do filtering
    lastSensor1 = (filterWeight * currentSensor1) + ((1-filterWeight) * lastSensor1); 
  }
}



void sendData() {
  sensorCurMillis = millis();
  int32_t tframe = sensorCurMillis - sensorPrevMillis;
  if(sensorCurMillis > SEND_FIRST_DATA){
  if (tframe > SEND_INTERVAL || tframe == 0 || sensorPrevMillis==0) {
    sensorPrevMillis = sensorCurMillis;

    dtostrf(lastSensor, 4, 2, resultT);
    dtostrf(lastSensor1, 4, 2, resultT1);
  //  SerialDEBUG->print(F("Temperature: "));
 //   SerialDEBUG->print(resultT);
 //   SerialDEBUG->println(F("°C"));

    //kirim data
    if (iot.mqtt->connected()) {    
      iot.mqtt->publish(iot.constructTopic("sensor/temp"), resultT, true);
      iot.mqtt->publish(iot.constructTopic("sensor1/temp"), resultT, true);
      
      if (errSensor==-1) iot.mqtt->publish(iot.constructTopic("sensor/error"), "Low", true);
      else if (errSensor==1) iot.mqtt->publish(iot.constructTopic("sensor/error"), "High", true);
      else iot.mqtt->publish(iot.constructTopic("sensor/error"), "Normal", true);

      if (errSensor1==-1) iot.mqtt->publish(iot.constructTopic("sensor1/error"), "Low", true);
      else if (errSensor1==1) iot.mqtt->publish(iot.constructTopic("sensor1/error"), "High", true);
      else iot.mqtt->publish(iot.constructTopic("sensor1/error"), "Normal", true);
    }
   }
  }
}

void callbackOnReceive(char* topic, byte* payload, unsigned int length) {
  String strPayload;
  strPayload.reserve(length);
  for (uint32_t i = 0; i < length; i++) strPayload += (char)payload[i];

  if (DEBUG) {
    SerialDEBUG->print(F("Receiving topic: "));
    SerialDEBUG->println(topic);
    SerialDEBUG->print(F("With value: "));
    SerialDEBUG->println(strPayload);
  }
  if (strcmp(topic,iot.constructTopic("reset"))==0 && strPayload=="true") iot.reset();
  else if (strcmp(topic,iot.constructTopic("restart"))==0 && strPayload=="true") iot.restart();
}

void callbackOnConnect() {
  initPublish();    
  initSubscribe();
}

void callbackOnReconnect() {
  initSubscribe();
}

void callbackOnAPConfigTimeout() {
  iot.restart();
}

void callbackOnWiFiConnectTimeout() {
  iot.restart();
  //ESP.deepSleep(300000000); // sleep for 5 minutes
}

void publishState(const char* state) {
  if (iot.mqtt->connected()) iot.mqtt->publish(iot.constructTopic("$state"), state, true);  
}

void initPublish() {
  if (iot.mqtt->connected()) {    
    iot.mqtt->publish(iot.constructTopic("$state"), "init", false);
    iot.mqtt->publish(iot.constructTopic("$sammy"), "1.0.0", false);
    iot.mqtt->publish(iot.constructTopic("$name"), "Smart Dallas Temperature", false);
    iot.mqtt->publish(iot.constructTopic("$model"), "SAM-DLST", false);
    iot.mqtt->publish(iot.constructTopic("$mac"), WiFi.macAddress().c_str(), false);
    iot.mqtt->publish(iot.constructTopic("$localip"), WiFi.localIP().toString().c_str(), false);
    iot.mqtt->publish(iot.constructTopic("$fw/name"), "DHT22", false);
    iot.mqtt->publish(iot.constructTopic("$fw/version"), "1.00", false);    
    iot.mqtt->publish(iot.constructTopic("$reset"), "true", false);
    iot.mqtt->publish(iot.constructTopic("$restart"), "true", false);
    iot.mqtt->publish(iot.constructTopic("$nodes"), "sensor,sensor1", false);
  
  //define node "sensor"
    iot.mqtt->publish(iot.constructTopic("sensor/$name"), "Sensor", false);
    iot.mqtt->publish(iot.constructTopic("sensor/$type"), "Sensor-01", false);
    iot.mqtt->publish(iot.constructTopic("sensor/$properties"), "temp,error", false);

    iot.mqtt->publish(iot.constructTopic("sensor/temp/$name"), "Temperature", false);
    iot.mqtt->publish(iot.constructTopic("sensor/temp/$settable"), "false", false);
    iot.mqtt->publish(iot.constructTopic("sensor/temp/$retained"), "true", false);
    iot.mqtt->publish(iot.constructTopic("sensor/temp/$datatype"), "float", false);  
    iot.mqtt->publish(iot.constructTopic("sensor/temp/$unit"), "°C", false);  
    iot.mqtt->publish(iot.constructTopic("sensor/temp/$format"), "-55:125", false);

    iot.mqtt->publish(iot.constructTopic("sensor/error/$name"), "Error Reading", false);
    iot.mqtt->publish(iot.constructTopic("sensor/error/$settable"), "false", false);
    iot.mqtt->publish(iot.constructTopic("sensor/error/$retained"), "true", false);
    iot.mqtt->publish(iot.constructTopic("sensor/error/$datatype"), "enum", false);  
    iot.mqtt->publish(iot.constructTopic("sensor/error/$format"), "Low,Normal,High", false);

    //
    iot.mqtt->publish(iot.constructTopic("sensor1/$name"), "Sensor", false);
    iot.mqtt->publish(iot.constructTopic("sensor1/$type"), "Sensor-01", false);
    iot.mqtt->publish(iot.constructTopic("sensor1/$properties"), "temp,error", false);

    iot.mqtt->publish(iot.constructTopic("sensor1/temp/$name"), "Temperature", false);
    iot.mqtt->publish(iot.constructTopic("sensor1/temp/$settable"), "false", false);
    iot.mqtt->publish(iot.constructTopic("sensor1/temp/$retained"), "true", false);
    iot.mqtt->publish(iot.constructTopic("sensor1/temp/$datatype"), "float", false);  
    iot.mqtt->publish(iot.constructTopic("sensor1/temp/$unit"), "°C", false);  
    iot.mqtt->publish(iot.constructTopic("sensor1/temp/$format"), "-55:125", false);

    iot.mqtt->publish(iot.constructTopic("sensor1/error/$name"), "Error Reading", false);
    iot.mqtt->publish(iot.constructTopic("sensor1/error/$settable"), "false", false);
    iot.mqtt->publish(iot.constructTopic("sensor1/error/$retained"), "true", false);
    iot.mqtt->publish(iot.constructTopic("sensor1/error/$datatype"), "enum", false);  
    iot.mqtt->publish(iot.constructTopic("sensor1/error/$format"), "Low,Normal,High", false);
  }
}

void initSubscribe() {
  if (iot.mqtt->connected()) { 
    iot.mqtt->subscribe(iot.constructTopic("reset"),1);  
    iot.mqtt->subscribe(iot.constructTopic("restart"),1);  
  }
  publishState("ready");
}
