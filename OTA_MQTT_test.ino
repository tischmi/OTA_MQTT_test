/* Qelle https://github.com/tischmi/ota-mqtt/blob/master/ota-mqtt/ota-mqtt.ino 

HUMIDOR Projekt
---------------
+ MQTT
+ OTA
+ Timer
+ BUILTIN_LED blink
+ DHT22
+ deep sleep nach http://lazyzero.de/elektronik/esp8266/dht_deepsleep/start


-RTC richiges Timing(mqtt-Versand) per NTP
*/

#define PROGRAMM "OTA_MQTT_test"

#include <ESP8266WiFi.h>  //For ESP8266
#include <PubSubClient.h> //For MQTT
#include <ESP8266mDNS.h>  //For OTA
#include <WiFiUdp.h>      //For OTA
#include <ArduinoOTA.h>   //For OTA
#include <SimpleTimer.h>  //For simple Timer
#include "DHT.h"          //For DHT22 hum and temp
#include <Ultrasonic.h>   //For HC SR04

//WIFI configuration
#define wifi_ssid "xxx"
#define wifi_password "xxx"

 //MQTT configuration
#define mqtt_server "192.168.15.114"
#define mqtt_user "esp8266"
#define mqtt_password "esp8266password"
String mqtt_client_id="WemosD102-";   //This text is concatenated with ChipId to get unique client_id
//MQTT Topic configuration
//String mqtt_base_topic="/sensor/"+mqtt_client_id+"/data";
//String mqtt_base_topic="home/humidor";
String mqtt_base_topic="home/ESP07S01";

#define humidity_topic "/humidity"
#define temperature_topic "/temperature"
#define version_topic "/version"
#define status_topic "/status"
#define distance_topic "/distance"
#define waterlevel_topic "/waterlevel"

// DHT config
#define DHTPIN            D1       // what digital pin we're connected to
#define DHTTYPE           DHT22    // DHT 11/22
#define DHTINTERVAL_T     5000     // ms Temperature
#define DHTINTERVAL_H     10000    // ms Humidity

#define DEEPSLEEP         60       // Verweildauer im Deep Sleep in Minuten
#define DEEPSLEEPINTERVAL 30000    // schalte in DEep Sleep alle 30 Sekunden
#define FORCE_DEEPSLEEP

#define LEDBLINKINTERVAL  1000      // LED Binkinterval in ms

#define SR04INTERVAL      1000      // ms
const int PumpPin =  D6;      //

const int SENSORHIGHT = 40;
const int NANOHIGHT   = 30;
const int WatterHightMax = 28;
const int WatterHightMin = 20;

int PumpPinState = LOW;    // Pumpe aus
int Watterlevel = NANOHIGHT;

int BUILTIN_LED_state =1;

//MQTT client
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

//Necesary to make Arduino Software autodetect OTA device
WiFiServer TelnetServer(8266);

//DHT Device
DHT dht(DHTPIN, DHTTYPE);

// SR04 Device
Ultrasonic ultrasonic(D7, D8);

// Timer instanzieren
SimpleTimer timer_BUILTIN_LED;
//SimpleTimer timer_BUILTIN_LED_LOW;   // schaltet LED aus
SimpleTimer timer_DHT_temperature;
SimpleTimer timer_DHT_humidity;
SimpleTimer timer_SR04_distance;

SimpleTimer timer_deepsleep;


void DeepSleep(){ //deep sleep
  #ifdef FORCE_DEEPSLEEP
    Serial.println("Force deepsleep!");
    BUILTIN_LED_LOW();
    mqtt_client.publish((mqtt_base_topic+status_topic).c_str(), "Force deep sleep!");
    ESP.deepSleep(DEEPSLEEP*1000*1000*60);          // eine minute
    delay(100);
  #endif
}  

void BUILTIN_LED_LOW(){
  Serial.println("LED LOW");
  digitalWrite(LED_BUILTIN, LOW);
}

void BUILTIN_LED_flash(){
    Serial.println("LED HIGH");
    digitalWrite(LED_BUILTIN, HIGH);
    
}

void BUILTIN_LED_blink(){
  BUILTIN_LED_state = BUILTIN_LED_state * -1;
  if (BUILTIN_LED_state ==1) {
    digitalWrite(BUILTIN_LED, HIGH);
  } else {
     digitalWrite(BUILTIN_LED, LOW);
  }
}

void SR04_distance_mqtt_push() {
  char msg[50];
  int Distance = ultrasonic.distanceRead();
  Watterlevel = SENSORHIGHT - Distance;
  Serial.print("Wasserstand in CM: ");  Serial.println(Watterlevel);
//  String sWatterlevel = String(Watterlevel);
//  snprintf (msg, 50, "%s",sWatterlevel);
//  mqtt_client.publish((mqtt_base_topic+waterlevel_topic).c_str(), msg);
  //mqtt_client.publish((mqtt_base_topic+distance_topic).c_str(), Distance);
}

void DHT_humidity_mqtt_push() {

  float h = dht.readHumidity();
  Serial.println(h);
  char msg[50];
    if (isnan(h)) {
       Serial.println("Failed to read from DHT sensor!");
       mqtt_client.publish((mqtt_base_topic+status_topic).c_str(), "Failed to read from DHT sensor!");
       return;
    }
    char str_h[6];
    dtostrf(h, 4, 2, str_h);
    snprintf (msg, 75, "%s",str_h);
    mqtt_client.publish((mqtt_base_topic+humidity_topic).c_str(), msg);
    //Serial.println("call BUILTIN_LED_flash from h");
    //BUILTIN_LED_flash();
/**/
}

void DHT_temperature_mqtt_push() {
  float t = dht.readTemperature();
  Serial.println(t);
  char msg[50];
    if (isnan(t)) {
       Serial.println("Failed to read from DHT sensor!");
       return;
    }
    char str_t[6];
    dtostrf(t, 4, 2, str_t);
    snprintf (msg, 75, "%s",str_t);
    mqtt_client.publish((mqtt_base_topic+temperature_topic).c_str(), msg);
    //Serial.println("call BUILTIN_LED_flash from t");
    //BUILTIN_LED_flash();
}


void setup_wifi() {
  delay(10);
  Serial.print("Connecting to ");
  Serial.print(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("OK");
  Serial.print("   IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() { 
  Serial.begin(115200);
  Serial.println(__DATE__);
  Serial.println(__TIME__);
  
  Serial.println("\r\nBooting...");
  
  setup_wifi();
{ //Configuring OTA device...
  Serial.print("Configuring OTA device...");
  TelnetServer.begin();   //Necesary to make Arduino Software autodetect OTA device  
  ArduinoOTA.onStart([]() {Serial.println("OTA starting...");});
  ArduinoOTA.onEnd([]() {Serial.println("OTA update finished!");Serial.println("Rebooting...");});
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {Serial.printf("OTA in progress: %u%%\r\n", (progress / (total / 100)));});  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OK");

  Serial.println("Configuring MQTT server...");
  mqtt_client_id=mqtt_client_id+ESP.getChipId();
  //mqtt_base_topic="/sensor/"+mqtt_client_id+"/data";
  //mqtt_base_topic="home/humidor";
  mqtt_base_topic="home/sensor/";
  
  // IP Adresse zu String konvertieren
  String MyIp = String(WiFi.localIP()[0]) + "." + String(WiFi.localIP()[1]) + "." + String(WiFi.localIP()[2]) + "." + String(WiFi.localIP()[3]);
  
  mqtt_base_topic+=MyIp;

  
  mqtt_client.setServer(mqtt_server, 1883);
  Serial.printf("   Server IP: %s\r\n",mqtt_server);  
  Serial.printf("   Username:  %s\r\n",mqtt_user);
  Serial.println("   Cliend Id: "+mqtt_client_id);  
  Serial.println("   MQTT configured!");

  Serial.println("Setup completed! Running app...");
  
}

  // lass die LED blinken
  Serial.println("Setup BUILTIN_LED Timer...");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PumpPin, OUTPUT);
  
  timer_BUILTIN_LED.setInterval(LEDBLINKINTERVAL, BUILTIN_LED_blink);

  {  //timer initialisieren
  Serial.println("Setup DHT...");
  dht.begin();
  Serial.println("Setup DHT_humidity Timer...");
  timer_DHT_humidity.setInterval(DHTINTERVAL_H, DHT_humidity_mqtt_push);
  Serial.println("Setup DHT_temperature Timer...");
  timer_DHT_temperature.setInterval(DHTINTERVAL_T, DHT_temperature_mqtt_push);
  Serial.println("Setup SR04_distance Timer...");
  timer_SR04_distance.setInterval(SR04INTERVAL, SR04_distance_mqtt_push);
  Serial.println("Setup DeepSleep Timer...");
  timer_deepsleep.setInterval(DEEPSLEEPINTERVAL, DeepSleep);
  }  
}

void mqtt_reconnect() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (client.connect("ESP8266Client")) {    
    if (mqtt_client.connect(mqtt_client_id.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return(true);
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}

long now =0; //in ms
long lastMsg = 0;
float temp = 0.0;
float hum = 0.0;
float diff = 1.0;
int min_timeout=2000; //in ms

void loop() {
  
  ArduinoOTA.handle();
  
  if (!mqtt_client.connected()) {
    mqtt_reconnect();
  }
  mqtt_client.loop();

  timer_BUILTIN_LED.run();      // timer zum LED blinken
  timer_DHT_humidity.run();     // timer DHT humidity
  timer_DHT_temperature.run();  // timer DHT temperature
  timer_SR04_distance.run();     // timer SR04 distance

  timer_deepsleep.run();
  
  //timer_BUILTIN_LED_LOW.setTimeout(100, BUILTIN_LED_LOW);


}
