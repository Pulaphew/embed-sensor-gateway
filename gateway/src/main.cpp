#include <WiFi.h>
#include <PubSubClient.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>

// ------------------- WiFi & NETPIE Config ---------------------
// const char* ssid = "Frigg";
// const char* password = "0817410115";

const char* ssid = "Frigg";
const char* password = "0817410115";

const char* mqtt_server = "mqtt.netpie.io";
const int   mqtt_port   = 1883;

const char* mqtt_ClientID = "b07d3468-8a9d-4ab1-86fa-839e159e5d50";
const char* mqtt_username = "sPVA5qzxqqMRReUebuo6r6cbQ8cvcZC3";
const char* mqtt_password = "i9CeV1Evu9rkuvj6uecArHpHSPZAzrhi";
// --------------------------------------------------------------

WiFiClient espClient;
PubSubClient client(espClient);

// ------------------- DS1302 CONFIG ---------------------------
#define DS1302_CLK_PIN 14
#define DS1302_DAT_PIN 27
#define DS1302_RST_PIN 26

// Create a ThreeWire instance using the defined pins
ThreeWire myWire(DS1302_DAT_PIN, DS1302_CLK_PIN, DS1302_RST_PIN); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

// ---------------- Button & LED (Toggle Logic) ----------------
const int buttonPin = 12;
const int LEDPin    = 21; 

int lastButtonState = HIGH;       // raw last reading
int currentButtonState = HIGH;    // raw current reading
int lastStableState = HIGH;       // state after debounce

bool curtainEnabled = true; // true=allow command , false=block command

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50 ms debounce
unsigned long lastPrint = 0;

// Function Prototypes
void processSensorData(String originalJson);
String getIsoTimestamp(const RtcDateTime& dt);

// ===============================================================
// WiFi Connection
// ===============================================================
void connectWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
}

// ===============================================================
// MQTT Connection with Auto-subscribe
// ===============================================================
void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to NETPIE MQTT...");

    if (client.connect(mqtt_ClientID, mqtt_username, mqtt_password)) {
      Serial.println("Connected!");
      // Subscribe to all user topics
      client.subscribe("@msg/sensor-data");
    }
    else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// ===============================================================
// CALLBACK: When ESP32 receives any MQTT message
// ===============================================================
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived: ");
  Serial.println(topic);

  String msg = "";
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.print("Payload: ");
  Serial.println(msg);

  // -------------------
  // MODE HANDLING
  // -------------------
  if (String(topic) == "@msg/sensor-data") {
    processSensorData(msg);
  }
}

void readButton() {
  int rawReading = digitalRead(buttonPin);

  if (rawReading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (rawReading != lastStableState) {
      lastStableState = rawReading;

      // Toggle state
      if (lastStableState == LOW) {
        curtainEnabled = !curtainEnabled;

        Serial.print("Curtain Control: ");
        Serial.println(curtainEnabled ? "ENABLED" : "DISABLED");
      }

      digitalWrite(LEDPin, curtainEnabled ? HIGH : LOW);   // ← update LED
    }
  }

  lastButtonState = rawReading;
}

// Helper to format time like "2023-12-06T14:30:00"
String getIsoTimestamp(const RtcDateTime& dt) {
    char datestring[25];
    snprintf(datestring, sizeof(datestring), 
            "%04u-%02u-%02uT%02u:%02u:%02u",
            dt.Year(), dt.Month(), dt.Day(),
            dt.Hour(), dt.Minute(), dt.Second());
    return String(datestring);
}

void processSensorData(String originalJson) {
  // Use Makuna library method to get time
  RtcDateTime now = Rtc.GetDateTime();
  String timestamp = getIsoTimestamp(now);

  // originalJson expected: {"data":{ ... }}
  int dataKeyPos = originalJson.indexOf("\"data\"");
  if (dataKeyPos == -1) {
    Serial.println("processSensorData: no \"data\" key found");
    return;
  }

  int innerStart = originalJson.indexOf('{', dataKeyPos);
  if (innerStart == -1) {
    Serial.println("processSensorData: no opening brace for data");
    return;
  }

  int len = originalJson.length();
  int depth = 0;
  int innerEnd = -1;
  for (int i = innerStart; i < len; ++i) {
    char c = originalJson.charAt(i);
    if (c == '{') depth++;
    else if (c == '}') depth--;

    if (depth == 0) {
      innerEnd = i;
      break;
    }
  }

  if (innerEnd == -1) {
    Serial.println("processSensorData: no matching closing brace");
    return;
  }

  // extract inside of the data object
  String inner = originalJson.substring(innerStart + 1, innerEnd);

  // Build payload
  char payload[512];
  snprintf(payload, sizeof(payload),
           "{\"data\":{%s,\"timestamp\":\"%s\",\"curtainEnabled\":%d}}",
           inner.c_str(),
           timestamp.c_str(),
           curtainEnabled ? 1 : 0);

  Serial.println("==== Gateway Prepared Payload ====");
  Serial.println(payload);

  // -----------------------------------
  // 1) Publish to SHADOW (optional)
  // -----------------------------------
  Serial.println("Gateway → @shadow/data/update");
   
  if (client.publish("@shadow/data/update", payload)) {
    Serial.println("Shadow publish success!");
  } else {
    Serial.println("Shadow publish failed!");
  }

  // -----------------------------------
  // 2) Publish to BACKEND (new)
  // -----------------------------------
  Serial.println("Gateway → @msg/gateway-data");

  if (client.publish("@msg/gateway-data", payload)) {
    Serial.println("Backend publish success!");
  } else {
    Serial.println("Backend publish failed!");
  }
}

void setup() {
  Serial.begin(115200);

  connectWiFi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  connectMQTT();

  // ---------------- RTC INIT ----------------
  Rtc.Begin();

  // 1. Check Write Protection
  if (Rtc.GetIsWriteProtected()) {
      Serial.println("RTC was write protected, enabling writing now");
      Rtc.SetIsWriteProtected(false);
  }

  // 2. Check if running
  if (!Rtc.GetIsRunning()) {
      Serial.println("RTC was not actively running, starting now");
      Rtc.SetIsRunning(true);
  }

  // 3. Set time to compile time ONLY if time is invalid
  RtcDateTime now = Rtc.GetDateTime();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

  if (!now.IsValid() || now < compiled) {
      Serial.println("RTC lost confidence or is older than compile time! Updating...");
      Rtc.SetDateTime(compiled);
  }

  // ---------------- BUTTON INIT ----------------
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(LEDPin, OUTPUT);

  digitalWrite(LEDPin, curtainEnabled ? HIGH : LOW);

  currentButtonState = digitalRead(buttonPin);
  lastStableState = currentButtonState;
}

void printTime() {
  RtcDateTime now = Rtc.GetDateTime();
  Serial.print("Date Time: ");
  Serial.println(getIsoTimestamp(now));
}

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();
  
  // ---------------- BUTTON with Debounce ----------------
  readButton();
  
  // ---------------- RTC TIME READ ----------------
  if (millis() - lastPrint >= 1000) {
    printTime();
    lastPrint = millis();
  }
}