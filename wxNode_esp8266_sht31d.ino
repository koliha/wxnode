#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHT31.h>
#include <PubSubClient.h>

#define wifi_ssid "wifi_ssid"
#define wifi_password "wifi_password"
#define mqtt_username "mqttsensor"
#define mqtt_password "mqttsensor"

#define mqtt_server "server.address.com"  // MQTT Cloud address
#define humidity_topic "humidity"
#define temperature_topic "temperature_F"

// esp8266 sensor/node id (name)
String sensorid = "attic";
String mqttprefix = "esp_";
String mqttcname = mqttprefix + sensorid;

// init sensor(s)
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// start wifi
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  // init serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("-----------");
  Serial.println("wxNode v0.4");
  Serial.println("-----------");
  // init onboard LED
  pinMode(LED_BUILTIN, OUTPUT);
  // turn on status LED
  digitalWrite(LED_BUILTIN, LOW);

  // enable SHT31 sensor
  Serial.println("Enabling SHT31 Sensor...");
  sht31.begin(0x44);

  // start networking
  Serial.print("Starting networking");
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(2500);
      Serial.print(".");
  }
  delay(1000);
  Serial.println("");
  Serial.print("IP: ");
  Serial.print(WiFi.localIP());
  Serial.println("");
  delay(2000);

  // Blink LED and turn off (startup finished)
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("[READY]");
  Serial.println("");
  Serial.println("");
  client.setServer(mqtt_server, 1883);
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(String(mqttcname).c_str(),mqtt_username,mqtt_password)) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            digitalWrite(LED_BUILTIN, LOW);
            delay(5000);
            digitalWrite(LED_BUILTIN, HIGH);
        }
    }
}

bool checkBound(float newValue, float prevValue, float maxDiff) {
    return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}

long lastMsg = 0;
float temp1 = 0.0;
float hum = 0.0;
float diff = 0.1;
float runcount = 0;
float runcountlimit = 20;

void loop() {
      if (!client.connected()) {
        reconnect();
      }
      client.loop();
      // read temp from SHT31 sensor
      float tempsht31 = sht31.readTemperature();
      // 5000ms delay between reads
      delay(5000);
      // read humidity
      float newHum = sht31.readHumidity();
      // conversion math
      float newTemp = (tempsht31 * 1.8) + 32;

      // in addition to updating based on "diff" value, force an update after runcountlimit is met
      if (runcount == runcountlimit) {
        temp1 = 0.0;
        hum = 0.0;
        Serial.println("Runcount met, forcing update...");
      }
      else {
           runcount = runcount + 1;
      }

      if (checkBound(newTemp, temp1, diff)) {
        String ctopic = "homeassistant/sensor/" + mqttcname + "_T/config";
        String cpayload = "{ \"dev_cla\": \"temperature\", \"name\": \"" + sensorid + "T\", \"uniq_id\": \"" + mqttcname + "_T\", \"stat_t\": \"homeassistant/sensor/" + mqttcname + "_T/state\", \"unit_of_meas\": \"°F\", \"value_template\": \"{{ value_json.temperature }}\" }";
        String pubtopic = "homeassistant/sensor/" + mqttcname + "_T/state";
        String pubpayloadstart = "{ \"temperature\": ";
        String pubpayloadend = " }";
        String pubpayload = pubpayloadstart + newTemp + pubpayloadend;
        temp1 = newTemp;
        Serial.print("New temperature:");
        Serial.println(String(temp1).c_str());
        Serial.println("Sending:");
        Serial.println(ctopic);
        Serial.println(cpayload);
        Serial.println(pubtopic);
        Serial.println(pubpayload);
        Serial.println("");
        client.publish(String(ctopic).c_str(), String(cpayload).c_str(), true);
        client.publish(String(pubtopic).c_str(), String(pubpayload).c_str(), true);
      }
      if (checkBound(newHum, hum, diff)) {
        String ctopic = "homeassistant/sensor/" + mqttcname + "_H/config";
        String cpayload = "{ \"dev_cla\": \"humidity\", \"name\": \"" + sensorid + "H\", \"uniq_id\": \"" + mqttcname + "_H\", \"stat_t\": \"homeassistant/sensor/" + mqttcname + "_H/state\", \"unit_of_meas\": \"%\", \"value_template\": \"{{ value_json.humidity }}\" }";
        String pubtopic = "homeassistant/sensor/" + mqttcname + "_H/state";
        String pubpayloadstart = "{ \"humidity\": ";
        String pubpayloadend = " }";
        String pubpayload = pubpayloadstart + newHum + pubpayloadend;
        hum = newHum;
        Serial.print("New humidity:");
        Serial.println(String(hum).c_str());
        Serial.println("Sending:");
        Serial.println(ctopic);
        Serial.println(cpayload);
        Serial.println(pubtopic);
        Serial.println(pubpayload);
        Serial.println("");
        client.publish(String(ctopic).c_str(), String(cpayload).c_str(), true);
        client.publish(String(pubtopic).c_str(), String(pubpayload).c_str(), true);
      }

  // additional delay between reads
  // (5000ms of delay is already built-in)
  delay(25000);
}
