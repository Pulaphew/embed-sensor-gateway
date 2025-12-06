#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>

#define LDR_PIN 34
#define DHTPIN 4
#define DHTTYPE DHT22

#define LED_1 17
#define LED_2 16
#define LED_3 2

#define LED_open 5
#define LED_close 18

// DS1302 Pins (SPI interface)
#define DS1302_CLK_PIN 14
#define DS1302_DAT_PIN 27
#define DS1302_RST_PIN 26

// ------------------- WiFi & NETPIE Config ---------------------
const char* ssid = "Frigg";
const char* password = "0817410115";

const char* mqtt_server = "mqtt.netpie.io";
const int   mqtt_port   = 1883;

const char* mqtt_ClientID = "3c785d48-5bef-46d0-a7af-326bfd8ca900";
const char* mqtt_username = "KPiUhzR4SiBb3BFwnCtXRcq8GESfJWGB";
const char* mqtt_password = "Uh9bctRxCHM6DyfY7YdCjbW1vgSGsFZE";

int curtain_state = 0;

// ----------------------------
// MOTOR SETTINGS 
// ----------------------------
#define PWM_FREQ 5000 
#define PWM_RESOLUTION 8 // 8-bit resolution (0-255)
#define PWM_CHANNEL_1 0 
#define PWM_CHANNEL_2 1
#define PWM_CHANNEL_3 2
#define PWM_CHANNEL_4 3
#define MOTOR1_PIN_A 32
#define MOTOR1_PIN_B 33
#define MOTOR2_PIN_A 18
#define MOTOR2_PIN_B 19
#define MOTOR_SPEED 255 // Speed 0-255
#define MOTOR_TIME 1500 // Time to open/close curtain in milliseconds

// ----------------------------
// FIXED MOTOR FUNCTIONS
// ----------------------------
void stopMotor() {
    ledcWrite(PWM_CHANNEL_1, 0);
    ledcWrite(PWM_CHANNEL_2, 0);
    ledcWrite(PWM_CHANNEL_3, 0);
    ledcWrite(PWM_CHANNEL_4, 0);
    Serial.println("Motors: STOP");
}

// Just START moving, do not stop
void startOpening() {
    Serial.println("Action: OPENING...");
    // Motor 1: Forward
    ledcWrite(PWM_CHANNEL_1, MOTOR_SPEED);
    ledcWrite(PWM_CHANNEL_2, 0);
    // Motor 2: REVERSE 
    ledcWrite(PWM_CHANNEL_3, 0);
    ledcWrite(PWM_CHANNEL_4, MOTOR_SPEED);
}

// Just START moving, do not stop
void startClosing() {
    Serial.println("Action: CLOSING...");
    // Motor 1: Backward
    ledcWrite(PWM_CHANNEL_1, 0);
    ledcWrite(PWM_CHANNEL_2, MOTOR_SPEED);
    // Motor 2: FORWARD 
    ledcWrite(PWM_CHANNEL_3, MOTOR_SPEED);
    ledcWrite(PWM_CHANNEL_4, 0);
}

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

int currentMode = -1;   // 0=manual, 1=templight, 2=voice

// ===============================================================
// CALLBACK: When ESP32 receives any MQTT message
// ===============================================================
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  
  Serial.print("Topic: "); Serial.println(topic);
  Serial.print("Payload: "); Serial.println(msg);

  // -------------------
  // 1. MODE SELECTION
  // -------------------
  if (String(topic) == "@msg/mode") {
    int mode = msg.toInt();
    currentMode = mode;
    Serial.print("Mode changed to: "); Serial.println(mode);

    // Reset LEDs
    digitalWrite(LED_1, LOW); digitalWrite(LED_2, LOW); digitalWrite(LED_3, LOW);

    if (mode == 0) {      // MANUAL
      Serial.println("MANUAL MODE");
      digitalWrite(LED_1, HIGH);
    } 
    else if (mode == 1) { // TEMP-LIGHT
      Serial.println("TEMP-LIGHT MODE");
      digitalWrite(LED_2, HIGH);
    } 
    else if (mode == 2) { // VOICE
      Serial.println("VOICE MODE");
      digitalWrite(LED_3, HIGH);
    }
  }

  // -------------------
  // 2. VOICE COMMAND
  // -------------------
  else if (String(topic) == "@msg/voice-msg") {
    if (currentMode != 2) {
      Serial.println("Error: Not in Voice Mode");
      return;
    }

    if (msg == "open") {
      startOpening();
      delay(MOTOR_TIME); // Run for full duration
      stopMotor();
      curtain_state = 100; // Update state to fully open
    } 
    else if (msg == "close") {
      startClosing();
      delay(MOTOR_TIME); // Run for full duration
      stopMotor();
      curtain_state = 0; // Update state to fully closed
    }
  }

  // -------------------
  // 3. MANUAL COMMAND (0, 25, 50, 75, 100)
  // -------------------
  else if (String(topic) == "@msg/manual-command") {
    if (currentMode != 0) {
      Serial.println("Error: Not in Manual Mode");
      return;
    }

    int targetPos = msg.toInt(); // Target position (e.g., 50)
    
    // Safety check for valid range
    if (targetPos < 0) targetPos = 0;
    if (targetPos > 100) targetPos = 100;

    Serial.print("Current Pos: "); Serial.println(curtain_state);
    Serial.print("Target Pos: "); Serial.println(targetPos);

    if (targetPos == curtain_state) {
      Serial.println("Already at target position.");
      return;
    }

    // Calculate how much we need to move
    int difference = targetPos - curtain_state; // e.g. 50 - 0 = 50
    
    // Calculate time required (Proportional to the difference)
    // If MOTOR_TIME is 5000ms, and we move 50%, time is 2500ms
    int moveTime = abs(difference) * (MOTOR_TIME / 100.0);

    if (difference > 0) {
      // Target is higher -> OPEN
      startOpening();
      delay(moveTime);
      stopMotor();
    } 
    else {
      // Target is lower -> CLOSE
      startClosing();
      delay(moveTime);
      stopMotor();
    }

    // Update global state
    curtain_state = targetPos;
    Serial.print("New Position: "); Serial.println(curtain_state);
  }

  // -------------------
  // 4. TEMP/LIGHT AUTOMATION
  // -------------------
  else if (String(topic) == "@msg/templight-command") {
    if (currentMode != 1) return;

    StaticJsonDocument<200> doc;
    if (deserializeJson(doc, msg) == DeserializationError::Ok) {
      int tempLimit = doc["temp"];
      int lightLimit = doc["light"];
      
      float t = dht.readTemperature();
      int ldr = analogRead(LDR_PIN);
      float l = (ldr / 4095.0) * 100.0;

      if (t >= tempLimit && l >= lightLimit) {
        // Condition met: Open
        if (curtain_state != 100) { // Only move if not already open
            startOpening();
            delay(MOTOR_TIME);
            stopMotor();
            curtain_state = 100;
        }
      } else {
        // Condition not met: Close
        if (curtain_state != 0) { // Only move if not already closed
            startClosing();
            delay(MOTOR_TIME);
            stopMotor();
            curtain_state = 0;
        }
      }
    }
  }
}
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
      client.subscribe("@msg/mode");
      client.subscribe("@msg/voice-msg");
      client.subscribe("@msg/manual-command");
      client.subscribe("@msg/templight-command");

      // Serial.println("Subscribe all topics");
    }
    else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// ===============================================================
// Sensor Reading + Publishing to NETPIE
// ===============================================================
void readSensorsAndPublish() {
  int ldrValue = analogRead(LDR_PIN);
  float light = (ldrValue) / 4095.0 * 100.0;

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("DHT22 read failed");
    return;
  }

  char payload[256];
  snprintf(payload, sizeof(payload),
    "{\"data\":{\"light\":%.2f,\"temperature\":%.2f,\"humidity\":%.2f,\"mode\":%d}}",
    light, temperature, humidity, currentMode
  );

  if (client.publish("@msg/sensor-data", payload)) {
    Serial.println("Publish success:");
    Serial.println(payload);
  } else {
    Serial.println("Publish failed");
  }
}

// ===============================================================
void setup() {
  Serial.begin(115200);
  dht.begin();
  
 // Initialize PWM channels for motors
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_3, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_4, PWM_FREQ, PWM_RESOLUTION);

  // Attach PWM channels to motor pins
  ledcAttachPin(MOTOR1_PIN_A, PWM_CHANNEL_1);
  ledcAttachPin(MOTOR1_PIN_B, PWM_CHANNEL_2);
  ledcAttachPin(MOTOR2_PIN_A, PWM_CHANNEL_3);
  ledcAttachPin(MOTOR2_PIN_B, PWM_CHANNEL_4);

  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_open, OUTPUT);
  pinMode(LED_close, OUTPUT);

  connectWiFi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  connectMQTT();
}

// ===============================================================
void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();
  Serial.println();
  readSensorsAndPublish();
  delay(1000);
}
