# Soil_Moisture_Detection
A project to enhance the capability of soil and moisture detection using a microcontroller.
Smart Garden Monitoring System

Overview

This project is a Smart Garden Monitoring System based on an ESP32 microcontroller. It monitors soil moisture, temperature, and humidity and controls a servo motor to automate irrigation when needed. The system connects to a Wi-Fi network using WiFiManager and communicates with an MQTT broker to send sensor data and receive commands.

Features

Wi-Fi Auto-Configuration: Uses WiFiManager for easy setup.

Sensor Integration:

BME280 for temperature, humidity, and pressure readings.

Soil moisture sensor for monitoring soil conditions.

OLED Display (SH1107): Displays real-time sensor data.

MQTT Communication:

Publishes sensor readings to an MQTT broker.

Subscribes to commands for automation.

Automated Irrigation:

Controls a servo motor to open/close a water valve based on soil moisture.

Deep Sleep Mode: Saves power by sleeping for 30 minutes between readings.

Hardware Requirements

ESP32 development board

BME280 Sensor (Temperature, Humidity, Pressure)

Soil Moisture Sensor

SH1107 OLED Display (64x128)

Servo Motor (for controlling irrigation)

Jumper wires

Pin Configuration

Component

ESP32 Pin

Soil Moisture Sensor

GPIO 34

Servo Motor

GPIO 12

OLED SDA

GPIO 26

OLED SCL

GPIO 25

Installation

Install Required Libraries in the Arduino IDE:

WiFiManager

PubSubClient

Adafruit_GFX

Adafruit_SH110X

Adafruit_Sensor

Adafruit_BME280

ESP32Servo

Upload the Code to the ESP32 board.

Connect to Wi-Fi using the WiFiManager setup page.

Set up an MQTT Broker (e.g., Mosquitto or cloud services like HiveMQ).

Subscribe to MQTT Topics to receive sensor updates.

MQTT Topics

Publish (ESP32 → Broker):

garden/sensors/temperature (Temperature data)

garden/sensors/humidity (Humidity data)

garden/sensors/soilmoisture (Soil moisture data)

Subscribe (Broker → ESP32):

garden/actuators (Receives irrigation commands)

How It Works

Boot & Setup

The ESP32 initializes the sensors, OLED display, and Wi-Fi connection.

If the Wi-Fi is not configured, it opens an access point for setup.

Sensor Readings & Publishing

Reads soil moisture, temperature, and humidity.

Publishes data to MQTT topics.

Automated Irrigation

If soil moisture < 50%, the servo motor activates to open the water valve.

Deep Sleep Mode

The ESP32 enters deep sleep for 30 minutes to save power.

Troubleshooting

Wi-Fi Connection Issues

Restart the ESP32 and reconnect using the WiFiManager portal.

MQTT Not Connecting

Verify the MQTT broker IP and ensure the broker is running.

No Sensor Data

Check wiring and sensor connections.