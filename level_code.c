#include "Arduino.h"

// WATCHDOG
#include <Esp.h>

#define TIMER_INTERVAL_MS 500
// WIFI
#include <ESP8266WiFi.h>
// OTA
#include <ESPAsyncWebServer.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

// MQTT
#include <PubSubClient.h>

#define MAYOR 1
#define MINOR 0
#define PATCH 0
#define WIFI_SSID "JAVI"
#define WIFI_PASS "xavier1234"
// MQTT
#define MSG_BUFFER_SIZE (50)
unsigned long lastMsg = 0;
char msg[MSG_BUFFER_SIZE];
volatile uint32_t lastMillis = 0;

String version = String(MAYOR) + "." + String(MINOR) + "." + String(PATCH);
bool state = 1;
// OTA
AsyncWebServer server(8080);

// MQTT
std::string device = "LEVEL_1";
std::string TOPIC_IP = "Tanque1/canal/ip/" + device;
const char *mqtt_server = "192.168.1.251";
const char *TOPIC = "Tanque1/canal/level/sensor1";
WiFiClient espClient;
PubSubClient client(espClient);

void callback_mqtt(char *topic, byte *payload, unsigned int length);
void reconnect_mqtt();


//APP
//app
#define echoPin D2  // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin D3  //attach pin D3 Arduino to pin Trig of HC-SR04
// defines variables
long duration;  // variable for the duration of sound wave travel
float distance;   // variable for the distance measurement


void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(trigPin, OUTPUT);      // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);       // Sets the echoPin as an INPU
  Serial.begin(9600);
  Serial.printf("\n");
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS); // change it to your ussid and password
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
  }
  Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
        uint32_t seconds = (uint32_t)(millis() / 1000);
                    char reply[100];
                    Serial.println(seconds);
                    sprintf(reply, "%d %s  ultrasonic sensor 1 %.2f mm", seconds,version.c_str(),distance);

    request->send(200, "text/plain", reply); });
  AsyncElegantOTA.begin(&server); // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  // SETUP APP
  sensors.begin(); // Se inicia el sensor
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback_mqtt);

  Serial.println("Delta ms = " + String(millis() - lastMillis) + " " + version);
}

void loop()
{
  if (!client.connected())
  {
    reconnect_mqtt();
  }
  else
  {
    client.loop();

  // Clears the trigPin condition
	  	digitalWrite(trigPin, LOW);
	  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
	  digitalWrite(trigPin, HIGH);
	  delayMicroseconds(10);
	  digitalWrite(trigPin, LOW);
	  // Reads the echoPin, returns the sound wave travel time in microseconds
	  duration = pulseIn(echoPin, HIGH);
	  // Calculating the distance
	  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
	  delay(50);
	  Serial.print("Distance: ");
	  Serial.print(distance);
	  Serial.println(" cm");
	  snprintf(msg, MSG_BUFFER_SIZE, "%.2f", distance);
	  client.publish(TOPIC, msg);
    
    if (millis() - lastMillis > 2000)
    {
      lastMillis = millis();

      digitalWrite(LED_BUILTIN, state);
      state = !state;

      Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
      snprintf(msg, MSG_BUFFER_SIZE, "%s:%s", device.c_str(), WiFi.localIP().toString().c_str());
      client.publish(TOPIC_IP.c_str(), msg);
    }
  }
}

void callback_mqtt(char *topic, byte *payload, unsigned int length)
{
  if (strcmp(TOPIC, topic) == 0)
  {
    
  }
}

void reconnect_mqtt()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(TOPIC, "Teperature sensor");
      // ... and resubscribe
      client.subscribe(TOPIC);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}