#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <EEPROM.h>
#include <fauxmoESP.h>
#include <PubSubClient.h>  // MQTT Library
// EEPROM MEMORY LAYOUT (Avoid conflicts)
#define EEPROM_SIZE 512
#define MQTT_SERVER_ADDR 50   // Start address for MQTT server
#define MQTT_PORT_ADDR 100    // Start address for MQTT port
#define MQTT_USER_ADDR 150    // Start address for MQTT username
#define MQTT_PASS_ADDR 200    // Start address for MQTT password
#define POSITION_ADDR 300     // Open/Close Position storage

WiFiClient espClient;
PubSubClient mqttClient(espClient);

String mqttServer;
int mqttPort;
String mqttUser;
String mqttPassword;
int mqttFailCount = 0;
bool mqttDisabled = false;  // Prevent infinite retries

fauxmoESP fauxmo;

// Define stepper motor pins
const int dirPin = D6;        // Direction pin
const int stepPin = D5;       // Step pin
const int enablePin = D7;     // Enable pin for the stepper driver

// Define AS5600 I2C address
#define AS5600_ADDR 0x36

// Motor and sensor state
volatile long revolutionCount = 0; // Tracks the number of full revolutions
int lastReading = 0;               // Previous AS5600 reading
int currentReading = 0;            // Current AS5600 reading
long absolutePosition = 0;         // Absolute position across revolutions
long openPosition = -1;             // Open position (-1 means not set)
long closePosition = -1;            // Close position (-1 means not set)
int targetPosition = 0;            // Target position from slider


// Motor movement state
volatile bool motorActive = false; // Is the motor currently active?
bool direction = true;             // Current motor direction
unsigned long lastStepTime = 0;    // Timestamp of last motor step
const unsigned long stepDelay = 2000; // Delay between steps (microseconds)

// Web server
ESP8266WebServer server(8080);


void loadMQTTSettings() {
    mqttServer = readStringFromEEPROM(MQTT_SERVER_ADDR, 40);
    mqttPort = readLongFromEEPROM(MQTT_PORT_ADDR);
    mqttUser = readStringFromEEPROM(MQTT_USER_ADDR, 30);
    mqttPassword = readStringFromEEPROM(MQTT_PASS_ADDR, 30);

    Serial.println("🔹 Loaded MQTT Settings:");
    Serial.println("Server: " + mqttServer);
    Serial.println("Port: " + String(mqttPort));
    Serial.println("User: " + mqttUser);
    Serial.println("Password: " + mqttPassword);
}


void connectMQTT() {
    if (mqttDisabled) return;  // Stop attempts if disabled

    Serial.print("🔄 Connecting to MQTT... ");
    mqttClient.setServer(mqttServer.c_str(), mqttPort);

    if (mqttClient.connect("SmartBlinds", mqttUser.c_str(), mqttPassword.c_str())) {
        Serial.println("✅ Connected to MQTT!");
        mqttFailCount = 0;  // Reset failure count
    } else {
        Serial.println("❌ MQTT Connection Failed.");
        mqttFailCount++;

        if (mqttFailCount >= 5) {
            Serial.println("🚫 Disabling MQTT after 5 failed attempts.");
            mqttDisabled = true;  // Stop future attempts
        }
    }
}


// Handle incoming MQTT messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("MQTT Message: ");
    Serial.println(topic);

    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println("Payload: " + message);

    if (String(topic) == "blinds/set") {
        if (message == "open") {
            handleOpenBlinds();
        } else if (message == "close") {
            handleCloseBlinds();
        } else {
            int percent = message.toInt();
            if (percent >= 0 && percent <= 100) {
                targetPosition = map(percent, 0, 100, openPosition, closePosition);
                moveToPosition(targetPosition);
                Serial.printf("Blinds moving to: %d%%...\n", percent);
                mqttClient.publish("blinds/state", String(percent).c_str(), true);
            }
        }
    }
}



// Helper functions for EEPROM
void saveLongToEEPROM(int address, long value) {
    for (int i = 0; i < 4; i++) {
        EEPROM.write(address + i, (value >> (8 * i)) & 0xFF);
    }
    EEPROM.commit();
}

long readLongFromEEPROM(int address) {
    long value = 0;
    for (int i = 0; i < 4; i++) {
        value |= ((long)EEPROM.read(address + i)) << (8 * i);
    }

    if (value < -1000000 || value > 1000000) return 0;  // Prevent corrupted values
    return value;
}


void saveStringToEEPROM(int startAddr, String data, int maxLength) {
    Serial.print("Saving string to EEPROM at address: ");
    Serial.print(startAddr);
    Serial.print(" -> ");
    Serial.println(data);

    for (int i = 0; i < maxLength; i++) {
        if (i < data.length()) {
            EEPROM.write(startAddr + i, data[i]);
        } else {
            EEPROM.write(startAddr + i, 0);  // Null-terminate
        }
    }
    EEPROM.commit();
}


String readStringFromEEPROM(int startAddr, int maxLength) {
    String data = "";
    for (int i = 0; i < maxLength; i++) {
        char c = EEPROM.read(startAddr + i);
        if (c == 0 || c == 255) break;  // Stop at null terminator or empty value
        data += c;
    }
    Serial.print("Loaded string from EEPROM: ");
    Serial.println(data);
    return data;
}




// Function to move motor to a target position
void moveToPosition(int position) {
  targetPosition = position;

  // Determine direction
  if (position < absolutePosition) { // Move to more negative values
    direction = true; // Clockwise
    digitalWrite(dirPin, HIGH); // Set DIR_PIN for clockwise
  } else if (position > absolutePosition) { // Move to more positive values
    direction = false; // Counterclockwise
    digitalWrite(dirPin, LOW); // Set DIR_PIN for counterclockwise
  } else {
    Serial.println("Already at target position.");
    motorActive = false; // Ensure motor is stopped
    digitalWrite(enablePin, HIGH); // Disable the motor driver
    return; // No movement needed
  }

  digitalWrite(enablePin, LOW); // Enable the motor driver
  motorActive = true;
  Serial.print("Moving to position: ");
  Serial.println(position);

   // Publish new position to MQTT
    int percentage = map(position, openPosition, closePosition, 0, 100);
    mqttClient.publish("blinds/state", String(percentage).c_str(), true);
}

// Fauxmo setup
void setupFauxmo() {
  fauxmo.addDevice("Blinds"); // Add Alexa device
  fauxmo.onSetState([](unsigned char device_id, const char *device_name, bool state, unsigned char value) {
    Serial.printf("Device [%s] state: %s, value: %d\n", device_name, state ? "ON" : "OFF", value);

    if (value == 255) { // Alexa default ON command
      // Fully open blinds
      Serial.println("Opening blinds to 100%...");
      moveToPosition(openPosition);
    } else if (state) { // Alexa set to specific percentage
      int percent = map(value, 0, 255, 0, 100); // Map value (0-255) to percentage (0-100)
      targetPosition = map(percent, 0, 100, openPosition, closePosition);
      Serial.printf("Setting blinds to %d%%...\n", percent);
      moveToPosition(targetPosition);
    } else { // Alexa default OFF command
      // Fully close blinds
      Serial.println("Closing blinds to 0%...");
      moveToPosition(closePosition);
    }
  });
}


void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  

  // Initialize EEPROM
  EEPROM.begin(512);
  loadMQTTSettings();  // ✅ Load MQTT Settings First
openPosition = readLongFromEEPROM(0);
if (openPosition < -1000000 || openPosition > 1000000) openPosition = 0; // Reset invalid values

closePosition = readLongFromEEPROM(4);
if (closePosition < -1000000 || closePosition > 1000000) closePosition = 0;

revolutionCount = readLongFromEEPROM(8);
if (revolutionCount < -1000000 || revolutionCount > 1000000) revolutionCount = 0;

absolutePosition = readLongFromEEPROM(12);
if (absolutePosition < -1000000 || absolutePosition > 1000000) absolutePosition = 0;


  // Initialize motor control pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH); // Disable motor driver initially

  // Initialize I2C for AS5600
  Wire.begin(D2, D1);
  Serial.println("I2C Initialized");

  // Initialize Wi-Fi using WiFiManager
  WiFiManager wifiManager;
  wifiManager.autoConnect("Blinds-Setup");

  Serial.println("Connected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize Fauxmo for Alexa
  fauxmo.setPort(80); // Required for Gen3 Alexa devices
  fauxmo.enable(true);
  setupFauxmo();

  // Configure web server routes
  server.on("/", handleRoot);
  server.on("/open", handleOpenBlinds);
  server.on("/close", handleCloseBlinds);
  server.on("/calibrateOpen", handleCalibrateOpen);
  server.on("/calibrateClose", handleCalibrateClose);
  server.on("/forceOpen", handleForceOpenBlinds);
  server.on("/forceClose", handleForceCloseBlinds);
  server.on("/stop", handleStopMotor);
  server.on("/setPosition", handleSetPosition);
  server.on("/erase", handleEraseEEPROM); 
  server.on("/", handleRoot);
  server.on("/mqtt", handleMQTTSettings); // New button for MQTT
  server.on("/saveMQTT", handleSaveMQTT);


  server.begin();
  Serial.println("Web server started");

  mqttClient.setServer(mqttServer.c_str(), mqttPort);
  mqttClient.setCallback(mqttCallback);

    if (!mqttServer.isEmpty() && mqttPort > 0) {
      mqttClient.setServer(mqttServer.c_str(), mqttPort);
      mqttClient.setCallback(mqttCallback);
      connectMQTT();
  } else {
      Serial.println("❌ MQTT Disabled: No valid settings.");
  }
}

void loop() {
  fauxmo.handle(); // Handle Alexa requests
  mqttClient.loop();  // Handle MQTT messages
  server.handleClient(); // Handle web server requests
  updatePosition(); // Update absolute position

static unsigned long lastDebugTime = 0;
if (millis() - lastDebugTime > 5000) { // Print every 5 seconds
    lastDebugTime = millis();
    Serial.print("AS5600: ");
    Serial.print(currentReading);
    Serial.print(", Revolutions: ");
    Serial.print(revolutionCount);
    Serial.print(", Position: ");
    Serial.println(absolutePosition);
}

  if (motorActive) {
    unsigned long currentTime = micros();
    if (currentTime - lastStepTime >= stepDelay) {
      lastStepTime = currentTime;
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(5); // Minimal pulse width
      digitalWrite(stepPin, LOW);

      if (targetPosition != INT_MAX && abs(absolutePosition - targetPosition) <= 5) {
        motorActive = false;
        digitalWrite(enablePin, HIGH); // Disable motor driver
        Serial.println("Target position reached. Motor stopped.");
      }
    }
  }
}

// Web handlers (add your existing handleOpenBlinds, handleCloseBlinds, and calibration logic here)
// Function to read AS5600 encoder angle and update absolute position should remain unchanged


// Function to read the AS5600 encoder angle
int readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C); // Address of angle high byte
  Wire.endTransmission();

  Wire.requestFrom(AS5600_ADDR, 2);
  int high_byte = Wire.read();
  int low_byte = Wire.read();

  int angle = ((high_byte << 8) | low_byte) & 0x0FFF; // Combine 12-bit angle
  return angle;
}

// Function to update position across revolutions
void updatePosition() {
  currentReading = readAS5600();

  // Detect crossing from 4095 to 0 (clockwise)
  if (currentReading < 100 && lastReading > 4000) {
    revolutionCount++;
    saveLongToEEPROM(8, revolutionCount);
  }
  // Detect crossing from 0 to 4095 (counterclockwise)
  else if (currentReading > 4000 && lastReading < 100) {
    revolutionCount--;
    saveLongToEEPROM(8, revolutionCount);
  }

  // Calculate the absolute position
  absolutePosition = (revolutionCount * 4096) + currentReading;
  lastReading = currentReading; // Update last reading

  saveLongToEEPROM(12, absolutePosition);
}


// Function to save position and revolution data to EEPROM
void savePositionToEEPROM() {
  saveLongToEEPROM(8, revolutionCount);
  saveLongToEEPROM(12, absolutePosition);
  EEPROM.commit();
  Serial.println("Position and revolution count saved to EEPROM.");
}



// Web route handlers
void handleRoot() {
  
    if (ESP.getFreeHeap() < 5000) { // Ensure enough free RAM
        Serial.println("❌ Not enough memory to serve webpage!");
        server.send(500, "text/plain", "Not enough memory.");
        return;
    }

  String positionPercentage = "50"; // Default value

  if (openPosition != -1 && closePosition != -1) {
    int range = closePosition - openPosition;
    int relativePosition = absolutePosition - openPosition;
    int percentage = map(relativePosition, 0, range, 0, 100);
    positionPercentage = String(constrain(percentage, 0, 100)); // Ensure 0-100 range
  }

  String html = "<html><head>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; text-align: center; background: #f5f7fa; }";
  html += ".container { width: 80%; margin: auto; padding: 20px; }";
  html += "h1 { color: #2a5f9e; }";
  html += ".slider-container { margin: 20px auto; }";
  html += ".slider { width: 80%; }";
  html += ".button-container { display: flex; flex-wrap: wrap; justify-content: center; gap: 10px; margin-top: 20px; }";
  html += ".button { padding: 12px 18px; border: none; border-radius: 10px; cursor: pointer; font-size: 16px; color: white; width: 30%; }";
  html += ".open { background: #4CAF50; }";
  html += ".close { background: #f44336; }";
  html += ".stop { background: #ffa500; }";
  html += ".force { background: #3f51b5; }";
  html += ".calibrate { background: #673ab7; }";
  html += ".erase { background: #9e9e9e; }";
  html += ".info { margin-top: 20px; text-align: left; padding: 15px; background: #ffffff; border-radius: 10px; box-shadow: 0px 4px 6px rgba(0,0,0,0.1); }";
  html += "</style>";
  html += "</head><body>";

  html += "<div class='container'>";
  html += "<h1>Smart Blinds Control</h1>";

  // Live Sensor Data
  html += "<div class='info'>";
  html += "<h2>Sensor & Position Data</h2>";
  html += "<p><strong>AS5600 Reading:</strong> " + String(currentReading) + "</p>";
  html += "<p><strong>Revolutions:</strong> " + String(revolutionCount) + "</p>";
  html += "<p><strong>Absolute Position:</strong> " + String(absolutePosition) + "</p>";
  html += "<p><strong>Open Position:</strong> " + (openPosition == -1 ? "Not Set" : String(openPosition)) + "</p>";
  html += "<p><strong>Close Position:</strong> " + (closePosition == -1 ? "Not Set" : String(closePosition)) + "</p>";
  html += "<p><strong>Current Position:</strong> " + positionPercentage + "%</p>";
  html += "</div>";

  // Slider control
  html += "<div class='slider-container'>";
  html += "<input type='range' min='0' max='100' value='" + positionPercentage + "' class='slider' id='positionSlider' oninput='updatePositionValue(this.value)'>";
  html += "<p>Position: <span id='positionValue'>" + positionPercentage + "</span>%</p>";
  html += "<button class='button stop' onclick=\"moveToPosition()\">Move</button>";
  html += "</div>";

  // Main Controls
  html += "<div class='button-container'>";
  html += "<button class='button open' onclick=\"location.href='/open'\">Open</button>";
  html += "<button class='button stop' onclick=\"location.href='/stop'\">Stop</button>";
  html += "<button class='button close' onclick=\"location.href='/close'\">Close</button>";
  html += "</div>";

  // Force Move Controls
  html += "<div class='button-container'>";
  html += "<button class='button force' onclick=\"location.href='/forceOpen'\">Force Open</button>";
  html += "<button class='button force' onclick=\"location.href='/forceClose'\">Force Close</button>";
  html += "</div>";

  // Calibration Controls
  html += "<div class='button-container'>";
  html += "<button class='button calibrate' onclick=\"location.href='/calibrateOpen'\">Set Open Position</button>";
  html += "<button class='button calibrate' onclick=\"location.href='/calibrateClose'\">Set Close Position</button>";
  html += "</div>";

  // Erase EEPROM
  html += "<div class='button-container'>";
  html += "<button class='button erase' onclick=\"location.href='/erase'\">Erase EEPROM</button>";
  html += "<button class='button erase' onclick=\"location.href='/mqtt'\">MQTT Settings</button>";

  html += "</div>";

  // JavaScript for slider control
  html += "<script>";
  html += "function updatePositionValue(val) { document.getElementById('positionValue').innerText = val; }";
  html += "function moveToPosition() {";
  html += "  let pos = document.getElementById('positionSlider').value;";
  html += "  window.location.href = '/setPosition?position=' + pos;";
  html += "}";
  html += "</script>";

  html += "</div></body></html>";
  server.send(200, "text/html", html);
}
void handleMQTTSettings() {
    String html = "<html><head>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; text-align: center; background: #f5f7fa; }";
    html += ".container { width: 80%; margin: auto; padding: 20px; }";
    html += "h1 { color: #2a5f9e; }";
    html += ".form-container { background: #ffffff; padding: 20px; border-radius: 10px; box-shadow: 0px 4px 6px rgba(0,0,0,0.1); margin-top: 20px; text-align: left; }";
    html += "input { width: 100%; padding: 8px; margin-top: 5px; margin-bottom: 15px; border: 1px solid #ccc; border-radius: 5px; }";
    html += ".button-container { display: flex; flex-wrap: wrap; justify-content: center; gap: 10px; margin-top: 20px; }";
    html += ".button { padding: 12px 18px; border: none; border-radius: 10px; cursor: pointer; font-size: 16px; color: white; width: 40%; }";
    html += ".save { background: #4CAF50; }";
    html += ".back { background: #2196F3; }";
    html += "</style>";
    html += "</head><body>";

    html += "<div class='container'>";
    html += "<h1>MQTT Settings</h1>";

    html += "<div class='form-container'>";
    html += "<form action='/saveMQTT' method='get'>";
    html += "<label><strong>MQTT Server:</strong></label>";
    html += "<input type='text' name='server' value='" + mqttServer + "'><br>";
    html += "<label><strong>MQTT Port:</strong></label>";
    html += "<input type='number' name='port' value='" + String(mqttPort) + "'><br>";
    html += "<label><strong>MQTT Username:</strong></label>";
    html += "<input type='text' name='user' value='" + mqttUser + "'><br>";
    html += "<label><strong>MQTT Password:</strong></label>";
    html += "<input type='password' name='password' value='" + mqttPassword + "'><br>";

    html += "<div class='button-container'>";
    html += "<input type='submit' class='button save' value='Save & Restart'>"; // Only this submits the form
    html += "</div>";

    html += "</form>";

    // Back to Home button (OUTSIDE form to prevent form submission)
    html += "<div class='button-container'>";
    html += "<button type='button' class='button back' onclick=\"window.location.href='/'\">Back to Home</button>";
    html += "</div>";

    html += "</div></div>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}



void handleSaveMQTT() {
    if (server.hasArg("server") && server.hasArg("port") && server.hasArg("user") && server.hasArg("password")) {
        mqttServer = server.arg("server");
        mqttPort = server.arg("port").toInt();
        mqttUser = server.arg("user");
        mqttPassword = server.arg("password");

        Serial.println("🔹 Saving MQTT Settings...");
        Serial.println("Server: " + mqttServer);
        Serial.println("Port: " + String(mqttPort));
        Serial.println("User: " + mqttUser);
        Serial.println("Password: " + mqttPassword);

        // Save to EEPROM
        saveStringToEEPROM(MQTT_SERVER_ADDR, mqttServer, 40);
        saveLongToEEPROM(MQTT_PORT_ADDR, mqttPort);
        saveStringToEEPROM(MQTT_USER_ADDR, mqttUser, 30);
        saveStringToEEPROM(MQTT_PASS_ADDR, mqttPassword, 30);
        EEPROM.commit();  // Commit changes

        server.send(200, "text/html", "<html><body><h1>Settings Saved!</h1><p>Restarting in 3 seconds...</p></body></html>");
        delay(3000);
        server.sendHeader("Location", "/");
        delay(300);
        ESP.restart();  // Restart ESP8266 to apply changes
    } else {
        server.send(400, "text/html", "<html><body><h1>Error: Missing Fields</h1></body></html>");
    }
}




void handleOpenBlinds() {
  if (openPosition != -1) {
    direction = true; // Clockwise
    digitalWrite(dirPin, HIGH); // Set DIR_PIN for clockwise
    moveToPosition(openPosition);
    mqttClient.publish("blinds/state", "100");
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleCloseBlinds() {
  if (closePosition != -1) {
    direction = false; // Counterclockwise
    digitalWrite(dirPin, LOW); // Set DIR_PIN for counterclockwise
    moveToPosition(closePosition);
    mqttClient.publish("blinds/state", "0");
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleForceOpenBlinds() {
  direction = true; // Clockwise
  digitalWrite(dirPin, HIGH); // Set DIR_PIN for clockwise
  motorActive = true;
  targetPosition = INT_MAX; // Disable position-based stopping
  Serial.println("Force opening blinds...");
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleForceCloseBlinds() {
  direction = false; // Counterclockwise
  digitalWrite(dirPin, LOW); // Set DIR_PIN for counterclockwise
  motorActive = true;
  targetPosition = INT_MAX; // Disable position-based stopping
  Serial.println("Force closing blinds...");
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStopMotor() {
  motorActive = false;
  digitalWrite(enablePin, HIGH); // Disable the motor driver
  Serial.println("Motor stopped.");
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleCalibrateOpen() {
    openPosition = absolutePosition;
    saveLongToEEPROM(0, openPosition); // Save as 32-bit long
    EEPROM.commit();
    Serial.print("Open position calibrated to: ");
    Serial.println(openPosition);
    server.sendHeader("Location", "/");
    server.send(303);
}

void handleCalibrateClose() {
    closePosition = absolutePosition;
    saveLongToEEPROM(4, closePosition); // Save as 32-bit long
    EEPROM.commit();
    Serial.print("Close position calibrated to: ");
    Serial.println(closePosition);
    server.sendHeader("Location", "/");
    server.send(303);
}

void handleSetPosition() {
  if (server.hasArg("position")) {
    int percent = server.arg("position").toInt();

    // Correct the mapping: higher percentage corresponds to closePosition
    targetPosition = map(percent, 0, 100, openPosition, closePosition);

    moveToPosition(targetPosition);
    Serial.print("Slider moved to position: ");
    Serial.println(targetPosition);
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleEraseEEPROM() {
    Serial.println("Flushing EEPROM...");

    // Set all EEPROM bytes to 0xFF (factory reset)
    for (int i = 0; i < 512; i++) {
        EEPROM.write(i, 0xFF);
    }
    EEPROM.commit();
    delay(100); // Allow EEPROM processing

    Serial.println("EEPROM flushed. Rebooting...");

    server.sendHeader("Location", "/");
    server.send(303);

    delay(2000); // Wait 2 seconds before reboot

    ESP.restart(); // Force full system reboot
}




