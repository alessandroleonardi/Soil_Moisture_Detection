#include <SPI.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include "esp_sleep.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h>

// Initialize OLED SH1107 Display
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// Initialize BME280 Sensor
Adafruit_BME280 bme;

// Initialize Servo Motor
Servo servo;

// MQTT Broker IP
IPAddress server(192, 168, 245, 154);

// WiFiManager and MQTT client objects
WiFiManager wm;
WiFiClient espClient;
PubSubClient client(espClient);

// MQTT Topics
const char* topic_publish = "garden/actuators";
const char* topic_subscribe = "garden/sensors/#";  // Subscribe to all sensors

// Store last known sensor values
float lastTemperature = 20;  // Default temperature (°C)
float lastHumidity = 100;    // Default humidity (%)
float lastMoisture = 100;

// Pin Definitions
#define SOIL_SENSOR_PIN 34  // Soil moisture sensor pin (ADC)
#define SERVO_PIN 12        // Servo motor pin
#define SEALEVELPRESSURE_HPA (1013.25)  // Sea level pressure for BME280
#define SLEEP_TIME 30 * 60 * 1000000  // 30 minutes in microseconds (deep sleep duration)
#define WAKEUP_PIN GPIO_NUM_0  // GPIO 0 for manual wake-up (e.g., pressing Boot button)
#define ACTIVE_TIME 3 * 60 * 1000  // 3 minutes in milliseconds for active time
#define MQTT_WAIT_TIME 5000  // 5 seconds to wait for retained MQTT messages
#define CHECK_INTERVAL 500   // Interval for checking new messages during active time

// Thresholds
#define MOISTURE_THRESHOLD_LOW 30
#define MOISTURE_THRESHOLD_HIGH 60
#define TEMPERATURE_THRESHOLD_LOW 1

// Function to publish MQTT messages (with retention)
void publishMessage(const char* topic, const char* message) {
    client.publish(topic, message, true); // "true" ensures the message is retained
    Serial.print("Published (Retained): ");
    Serial.print(topic);
    Serial.print(" -> ");
    Serial.println(message);
}

// MQTT callback function to handle incoming messages
void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    Serial.print("Received on ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);

    // Convert the payload message to a float and handle the respective sensor topic
    float value = message.toFloat();
    String topicStr = String(topic);

    if (topicStr == "garden/sensors/temperature") {
        lastTemperature = value;  // Update the last known temperature
    } 
    else if (topicStr == "garden/sensors/humidity") {
        lastHumidity = value;  // Update the last known humidity
    }
    else if (topicStr == "garden/sensors/soilmoisture") {
        lastMoisture = value;  // Update the last known soil moisture
    }
}
void publishSensorData(float data, const char* type) {
  char topic[50];  
  snprintf(topic, sizeof(topic), "garden/sensors/%s", type);
  
  char payload[20];  
  snprintf(payload, sizeof(payload), "%.2f", data);  // Limita a 2 decimali

  Serial.println(topic);
  Serial.println(payload);

  publishMessage(topic, payload);
}

// Function to reconnect to MQTT broker and subscribe to topics
void reconnect() {
    int attempts = 0;
    while (!client.connected() && attempts < 5) { // Retry 5 times
        Serial.print("Attempting MQTT connection... ");
        if (client.connect("EspSensor")) {
            Serial.println("Connected to MQTT!");
            client.subscribe(topic_subscribe);  // Subscribe to all sensor topics
            Serial.println("Subscribed to: " + String(topic_subscribe));
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" | Retrying in 3 seconds...");
            delay(3000);  // Retry delay
            attempts++;
        }
    }
    if (!client.connected()) {
        Serial.println("MQTT connection failed after multiple attempts. Restarting ESP...");
        ESP.restart();  // Restart ESP if MQTT connection fails
    }
}

// Function to wait for retained MQTT messages after reconnecting
void waitForMQTTMessages() {
    Serial.println("Waiting for retained messages...");
    unsigned long startTime = millis();
    while (millis() - startTime < MQTT_WAIT_TIME) {
        client.loop();  // Process incoming messages
    }
}

// Function to update OLED display with sensor data
void updateDisplay(float temperature, float humidity, float soilMoisture) {
    display.clearDisplay();
    display.setTextSize(1);  // Use small text for display
    display.setTextColor(SH110X_WHITE);

    display.setCursor(0, 0);
    display.println("DATA \n");

    display.setCursor(0, 15);
    display.print("Temp \n"); 
    display.print(temperature, 1);
    display.println(" C");

    display.setCursor(0, 50);
    display.print("Humidity \n"); 
    display.print(humidity, 1);
    display.println(" %");

    display.setCursor(0, 85);
    display.print("Soil \n"); 
    display.print(soilMoisture, 1);
    display.println(" %");

    display.display();  // Update OLED display
}

// Function to control servo and check soil moisture
void controlServo() {
    publishMessage(topic_publish, "IRRIGATING");
    Serial.println("Activating servo: soil is too dry!");

    for (int i = 0; i < 5; i++) {  // Rotate servo for 5 seconds
        servo.write(90);  // Rotate servo to 90°
        delay(500);
        servo.write(0);   // Return to 0°
        delay(500);

        // Check soil moisture during servo movement
        int soilRaw = analogRead(SOIL_SENSOR_PIN);
        float soilMoisture = map(soilRaw, 0, 4095, 0, 100);
        soilMoisture = constrain(soilMoisture, 0, 100);

        if (soilMoisture > MOISTURE_THRESHOLD_HIGH) {
            Serial.println("Soil moisture > 60%. Stopping servo and going to ARREST.");
            publishMessage(topic_publish, "ARREST");
            servo.write(0);  // Stop the servo
            return;  // Exit the function to prevent further servo movement
        }
    }
    servo.write(0);  // Stop the servo
}

// Function to stay awake for 3 minutes and check for new MQTT messages
void stayAwakeForUpdates() {
    Serial.println("Staying awake for 3 minutes to check for updates...");
    unsigned long startTime = millis();
    while (millis() - startTime < ACTIVE_TIME) {
        client.loop();  // Continuously check for new MQTT messages

        // If temperature drops below 1°C or humidity rises above 60%, stop and sleep
        if (lastTemperature < TEMPERATURE_THRESHOLD_LOW || lastMoisture > MOISTURE_THRESHOLD_HIGH) {
            Serial.println("Critical condition met. Stopping immediately.");
            publishMessage(topic_publish, "ARREST");
            break;  // Stop checking and go to sleep
        }

        delay(CHECK_INTERVAL);  // Small delay to allow processing
    }
}

void setup() {
    Serial.begin(115200);
    // Configure soil sensor pin
    pinMode(SOIL_SENSOR_PIN, INPUT);  
    
    // Attach servo to the specified pin
    servo.attach(SERVO_PIN);  // Attach servo to the specified pin
    // Initialize servo position at 0°
    servo.write(0);  

    // Initialize I2C (GPIO 26 SDA, GPIO 25 SCL) for OLED and BME280 sensors
    Wire.begin(26, 25);
  
    // Initialize OLED SH1107 Display
    if (!display.begin(0x3C, true)) {  
      // I2C address 0x3C for SH1107
        Serial.println("Error: SH1107 OLED not found!");
        while (1);
    }

    // Initialize BME280 sensor
    if (!bme.begin(0x77)) {  
      // I2C address 0x77 for BME280
        Serial.println("Error: BME280 sensor not found!");
        while (1);
    }

    // Clear OLED display
    display.clearDisplay(); 
    display.display();

    // Set up WiFiManager to automatically connect
    bool res = wm.autoConnect("AutoConnectAP", "password");
    if (!res) {
        Serial.println("Failed to connect to WiFi. Restarting ESP...");
        delay(5000);
        ESP.restart();  // Restart if Wi-Fi connection fails
    } else {
        Serial.println("Connected to WiFi successfully!");
    }

    // Set up MQTT client
    client.setServer(server, 1883);
    client.setCallback(callback);
}

void loop() {
    int soilRaw = analogRead(SOIL_SENSOR_PIN);  // Read soil moisture sensor (analog)
    float soilMoisture = map(soilRaw, 0, 4095, 0, 100);  // Map to percentage (0-100%)
    soilMoisture = constrain(soilMoisture, 0, 100);  // Constrain within valid range

    // Read BME280 sensor (temperature, humidity, pressure)
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();

    // Print sensor readings to serial monitor
    Serial.println("==== Sensor Readings ====");
    Serial.print("Soil Moisture (%): "); Serial.print(soilMoisture); Serial.println("%");
    Serial.print("Temperature (°C): "); Serial.print(temperature); Serial.println("°C");
    Serial.print("Humidity (%): "); Serial.print(humidity); Serial.println("%");
    Serial.println("=========================\n");

    // If not connected to MQTT, reconnect
    if (!client.connected()) {
        reconnect();
    }

    // Update OLED display with sensor readings
    updateDisplay(temperature, humidity, soilMoisture);

    // Publish sensor data if it has changed significantly
    if (abs(lastTemperature - temperature) > 1 || abs(lastHumidity - humidity) > 1 || abs(lastMoisture - soilMoisture) > 1) {
        publishSensorData(temperature, "temperature");
        publishSensorData(humidity, "humidity");
        publishSensorData(soilMoisture, "soil_moisture");
        lastTemperature = temperature;
        lastHumidity = humidity;
        lastMoisture = soilMoisture;
    }

    // Process retained messages and control actuators
    if (lastMoisture < MOISTURE_THRESHOLD_LOW) {
        controlServo();
    }

    // Stay awake for updates
    stayAwakeForUpdates();

    // Configure deep sleep mode
    Serial.println("Going to deep sleep...");
    esp_sleep_enable_timer_wakeup(SLEEP_TIME);  // Set deep sleep duration (30 minutes)
    esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, 0);  // Wake up when GPIO 0 (Boot button) is pressed

    delay(1000);  // Ensure logs are printed before sleep
    esp_deep_sleep_start();  // Enter deep sleep mode
}
