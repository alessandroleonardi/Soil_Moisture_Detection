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
#include <WiFiManager.h>
#include <PubSubClient.h>

// Initialize OLED SH1107 Display
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// Initialize BME280 Sensor (temperature, humidity, pressure)
Adafruit_BME280 bme;

// Initialize Servo Motor
Servo servo;

// MQTT Broker IP
IPAddress server(192, 168, 1, 2);

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

// Pin Definitions
#define SOIL_SENSOR_PIN 34  // Soil moisture sensor pin (ADC)
#define SERVO_PIN 12        // Servo motor pin
#define SEALEVELPRESSURE_HPA (1013.25)  // Sea level pressure for BME280
#define SLEEP_TIME 30 * 60 * 1000000  // 30 minutes in microseconds (deep sleep duration)
#define WAKEUP_PIN GPIO_NUM_0  // GPIO 0 for manual wake-up (e.g., pressing Boot button)
#define ACTIVE_TIME 3 * 60 * 1000  // 3 minutes in milliseconds for active time
#define MQTT_WAIT_TIME 5000  // 5 seconds to wait for retained MQTT messages
#define CHECK_INTERVAL 500   // Interval for checking new messages during active time

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

// Function to process retained MQTT messages and control actuators
void processRetainedMessages() {
    Serial.println("Processing retained messages...");
    waitForMQTTMessages();  // Ensure we wait for and process any retained messages

    // If temperature or humidity conditions are critical, deactivate
    if (lastTemperature < 1 || lastHumidity > 60) {
        publishMessage(topic_publish, "DEACTIVATE");
    } 
    else if (lastHumidity < 50) {  // If humidity is below 50%, activate servo
        publishMessage(topic_publish, "ACTIVATE");
        Serial.println("Activating servo: soil is too dry!");
    
        for (int i = 0; i < 5; i++) {  // Rotate servo for 5 seconds
            servo.write(90);  // Rotate servo to 90°
            delay(500);
            servo.write(0);   // Return to 0°
            delay(500);
        }
        servo.write(0);  // Stop the servo
    }
}

// Function to stay awake for 3 minutes and check for new MQTT messages
void stayAwakeForUpdates() {
    Serial.println("Staying awake for 3 minutes to check for updates...");
    unsigned long startTime = millis();
    while (millis() - startTime < ACTIVE_TIME) {
        client.loop();  // Continuously check for new MQTT messages

        // If temperature drops below 1°C or humidity rises above 60%, stop and sleep
        if (lastTemperature < 1 || lastHumidity > 60) {
            Serial.println("Critical condition met (Temp < 1°C or Humidity > 60%). Stopping immediately.");
            publishMessage(topic_publish, "STOP");
            break;  // Stop checking and go to sleep
        }

        delay(CHECK_INTERVAL);  // Small delay to allow processing
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(SOIL_SENSOR_PIN, INPUT);  // Configure soil sensor pin
  
    servo.attach(SERVO_PIN);  // Attach servo to the specified pin
    servo.write(0);  // Initialize servo position at 0°

    // Initialize I2C (GPIO 26 SDA, GPIO 25 SCL) for OLED and BME280 sensors
    Wire.begin(26, 25);
  
    // Initialize OLED SH1107 Display
    if (!display.begin(0x3C, true)) {  // I2C address 0x3C for SH1107
        Serial.println("Error: SH1107 OLED not found!");
        while (1);
    }

    // Initialize BME280 sensor
    if (!bme.begin(0x77)) {  // I2C address 0x77 for BME280
        Serial.println("Error: BME280 sensor not found!");
        while (1);
    }

    display.clearDisplay();  // Clear OLED display
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

    // If not connected to MQTT, reconnect
    if (!client.connected()) {
        reconnect();
    }

    processRetainedMessages();  // Process any retained messages from the broker

    // If "ACTIVATE" was received, stay awake for 3 minutes to check for updates
    stayAwakeForUpdates();

    // Configure deep sleep mode
    Serial.println("Going to deep sleep...");
    esp_sleep_enable_timer_wakeup(SLEEP_TIME);  // Set deep sleep duration (30 minutes)
    esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, 0);  // Wake up when GPIO 0 (Boot button) is pressed

    delay(1000);  // Ensure logs are printed before sleep
    esp_deep_sleep_start();  // Enter deep sleep mode
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

    // If soil is too dry (moisture < 30%), activate servo to water the plants
    if (soilMoisture < 30) { 
        Serial.println("Activating servo: soil is too dry!");
    
        for (int i = 0; i < 5; i++) {  // Rotate servo for 5 seconds
            servo.write(90);  // Rotate servo to 90°
            delay(500);
            servo.write(0);   // Return to 0°
            delay(500);
        }
        servo.write(0);  // Stop the servo
    }

    // Update OLED display with sensor readings
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

    delay(1000);  // Wait 1 second before next reading
}
