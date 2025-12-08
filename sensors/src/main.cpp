#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

#define LDR_PIN 34
#define DHTPIN 4
#define DHTTYPE DHT11

#define EEPROM_SIZE 512
#define EEPROM_TEMP_ADDR 0
#define EEPROM_LIGHT_ADDR 4

// Global variables for limits
int tempLimit = 25; // Default value
int lightLimit = 75;  // Default value

// #define LED_open 5
// #define LED_close 18

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
// SINGLE MOTOR SETTINGS 
// ----------------------------
#define PWM_FREQ 5000 
#define PWM_RESOLUTION 8 // 8-bit resolution (0-255)
#define PWM_CHANNEL_1 0 
#define PWM_CHANNEL_2 1

// Only Motor 1 Pins defined now
#define MOTOR1_PIN_A 32
#define MOTOR1_PIN_B 33

#define MOTOR_SPEED 175 // Speed 0-255
#define MOTOR_TIME 1600 // Time to open/close curtain in milliseconds

// ----------------------------
// FIXED MOTOR FUNCTIONS (SINGLE MOTOR)
// ----------------------------
void stopMotor() {
    ledcWrite(PWM_CHANNEL_1, 0);
    ledcWrite(PWM_CHANNEL_2, 0);
    Serial.println("Motor: STOP");
}

// Just START moving, do not stop
void startOpening() {
    Serial.println("Action: OPENING...");
    // Motor 1: Forward
    ledcWrite(PWM_CHANNEL_1, MOTOR_SPEED);
    ledcWrite(PWM_CHANNEL_2, 0);
}

// Just START moving, do not stop
void startClosing() {
    Serial.println("Action: CLOSING...");
    // Motor 1: Backward
    ledcWrite(PWM_CHANNEL_1, 0);
    ledcWrite(PWM_CHANNEL_2, MOTOR_SPEED);
}

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

int currentMode = 0;   // 0=manual, 1=templight, 2=voice

// Function to save limits to EEPROM
void saveLimits() {
  EEPROM.put(EEPROM_TEMP_ADDR, tempLimit);
  EEPROM.put(EEPROM_LIGHT_ADDR, lightLimit);
  EEPROM.commit();
}

unsigned long lastSensorCheck = 0;
const unsigned long SENSOR_CHECK_INTERVAL = 2000; // Check every 2 seconds in templight mode


// ===============================================================
// TEMP/LIGHT AUTOMATION (moved from callback)
// ===============================================================
void checkTempLightMode() {
  if (currentMode != 1) return; // Only run in templight mode

  float t = dht.readTemperature();
  int ldr = analogRead(LDR_PIN);
  float l = (ldr / 4095.0) * 100.0;

  // Skip if sensor read failed
  // if (isnan(t)) return;
  // t=30.0;
  Serial.print("Temp: "); Serial.print(t);
  Serial.print("°C | Light: "); Serial.print(l);
  Serial.print("% | Limits - Temp: "); Serial.print(tempLimit);
  Serial.print("°C, Light: "); Serial.println(lightLimit + "%");

  // If temp OR light exceeds limit -> CLOSE curtain
  if (t >= tempLimit || l >= lightLimit) {
    if (curtain_state != 0) {
      Serial.println("Condition met: CLOSING curtain");
      startClosing();
      delay(MOTOR_TIME - 110);
      stopMotor();
      curtain_state = 0;
    }
  } 
  // If both conditions are below limit -> OPEN curtain
  else {
    if (curtain_state != 100) {
      Serial.println("Condition not met: OPENING curtain");
      startOpening();
      delay(MOTOR_TIME);
      stopMotor();
      curtain_state = 100;
    }
  }
}

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
      delay(MOTOR_TIME);
      // Run for full duration
      stopMotor();
      curtain_state = 100;
      // Update state to fully open
    } 
    else if (msg == "close") {
      startClosing();
      delay(MOTOR_TIME-110); // Run for full duration
      stopMotor();
      curtain_state = 0;
      // Update state to fully closed
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
    int difference = targetPos - curtain_state;
    // e.g. 50 - 0 = 50
    
    // Calculate time required (Proportional to the difference)
    // If MOTOR_TIME is 1100ms, and we move 50%, time is 550ms
    
    if (difference > 0) {
      // Target is higher -> OPEN
      int moveTime = abs(abs(difference) * (MOTOR_TIME) / 100.0);
      startOpening();
      delay(moveTime); 
      stopMotor();
    } 
    else {
      // Target is lower -> CLOSE
      int moveTime = abs(abs(difference) * (MOTOR_TIME-100) / 100.0);
      startClosing();
      delay(moveTime); 
      stopMotor();
    }

    // Update global state
    curtain_state = targetPos;
    Serial.print("New Position: "); Serial.println(curtain_state);
  }

  // -------------------
  // 4. TEMP/LIGHT LIMIT UPDATE (only updates limits, not automation)
  // -------------------
  else if (String(topic) == "@msg/templight-command") {
    StaticJsonDocument<200> doc; 
    if (deserializeJson(doc, msg) == DeserializationError::Ok) {
      tempLimit = doc["temp"];
      lightLimit = doc["light"];
      
      saveLimits();
      
      Serial.print("New tempLimit: "); Serial.println(tempLimit);
      Serial.print("New lightLimit: "); Serial.println(lightLimit);
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
    Serial.println("Failed to read from DHT sensor!");
    return;
    // humidity = 50;
    // temperature = 30; 
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

// Function to load limits from EEPROM
void loadLimits() {
  tempLimit = EEPROM.read(EEPROM_TEMP_ADDR);
  lightLimit = EEPROM.read(EEPROM_LIGHT_ADDR);
  
  // Validate values (if EEPROM is empty, use defaults)
  if (tempLimit == 255) tempLimit = 25;
  if (lightLimit == 255) lightLimit = 75;
  
  Serial.print("Loaded tempLimit: "); Serial.println(tempLimit);
  Serial.print("Loaded lightLimit: "); Serial.println(lightLimit);
}

// ===============================================================
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);  // Initialize EEPROM
  loadLimits();               // Load saved limits
  
  dht.begin();
  delay(2000); 
  
  // Initialize PWM channels for single motor
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);

  // Attach PWM channels to motor pins
  ledcAttachPin(MOTOR1_PIN_A, PWM_CHANNEL_1);
  ledcAttachPin(MOTOR1_PIN_B, PWM_CHANNEL_2); 

  // pinMode(LED_open, OUTPUT);
  // pinMode(LED_close, OUTPUT);

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
  // Check temp/light conditions continuously
  unsigned long currentTime = millis();
  if (currentTime - lastSensorCheck >= SENSOR_CHECK_INTERVAL) {
    checkTempLightMode();
    lastSensorCheck = currentTime;
  }
  Serial.println();
  readSensorsAndPublish();
  delay(1000);
}