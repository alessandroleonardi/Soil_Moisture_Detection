# Automated Garden Irrigation & Environmental Monitoring System

This project uses an ESP32 to remotely monitor environmental parameters and control irrigation for a garden. It reads data from a BME280 sensor (temperature, humidity, and pressure) and an analog soil moisture sensor, displays the values on an OLED display, and communicates with a remote MQTT broker. When the soil moisture drops below a specified threshold, the system activates a servo motor to irrigate the garden. It also utilizes deep sleep mode to conserve power.

## Features

- **Remote Monitoring:**  
  - Reads temperature, humidity, and soil moisture.
  - Displays data on an OLED (SH1107) display.
  - Publishes sensor readings to MQTT topics for remote monitoring.
  
- **Automated Irrigation:**  
  - Activates a servo motor to irrigate when soil moisture falls below 30%.
  - Monitors soil moisture during irrigation and stops the servo if moisture exceeds 60%.

- **MQTT Communication:**  
  - Publishes sensor data to topics such as `garden/sensors/temperature`, `humidity`, and `soil_moisture`.
  - Subscribes to topics for remote control commands.
  - Uses retained messages to keep the last known sensor data.

- **Power Management:**  
  - Uses deep sleep mode for 30 minutes to conserve energy.
  - Wakes up via timer or external wake-up (GPIO 0).

## Hardware Components

- **ESP32:** Main microcontroller with WiFi and deep sleep capabilities.
- **BME280 Sensor:** Measures temperature, humidity, and pressure.
- **Soil Moisture Sensor:** Provides an analog signal representing soil moisture.
- **OLED Display (SH1107):** Displays sensor readings locally.
- **Servo Motor:** Controls the irrigation mechanism.

## Software Dependencies

The following libraries are required:

- [WiFiManager](https://github.com/tzapu/WiFiManager)
- [PubSubClient](https://github.com/knolleary/pubsubclient)
- [Adafruit_GFX](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit_SH110X](https://github.com/adafruit/Adafruit_SH110X)
- [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)
- [Adafruit_BME280](https://github.com/adafruit/Adafruit_BME280_Library)
- [ESP32Servo](https://github.com/jkb-git/ESP32Servo)

Make sure these libraries are installed in your Arduino IDE.

## Installation

1. **Clone or Download the Repository:**

   ```bash
   git clone https://github.com/yourusername/automated-garden-irrigation.git
   ```

2. **Open the Project in Arduino IDE:**
   - Open the `.ino` file provided.

3. **Configure WiFi:**
   - The system uses WiFiManager to create an access point (`AutoConnectAP`) for initial configuration. Connect to this network and enter your WiFi credentials.

4. **Set Up MQTT Broker:**
   - Update the MQTT broker IP address in the code if necessary:
     ```cpp
     IPAddress server(192, 168, 245, 154);
     ```
   - Ensure your broker is running and accessible.

5. **Upload the Code:**
   - Connect your ESP32 to your computer and upload the sketch.

## Usage

- **Monitoring:**  
  The ESP32 will continuously monitor sensor values and display them on the OLED. Sensor data is also published to the corresponding MQTT topics.

- **Irrigation Control:**  
  When the soil moisture falls below the defined threshold (30%), the system will activate the servo motor to start irrigation. If soil moisture rises above 60% during irrigation, the servo will stop automatically.

- **Power Management:**  
  After processing sensor readings and checking for updates (including MQTT messages), the ESP32 enters deep sleep for 30 minutes. It can also be woken up externally by pressing a connected button on GPIO 0.

## Code Structure

- **Main Files:**
  - `setup()`: Initializes sensors, display, WiFi, MQTT, and hardware components.
  - `loop()`: Handles sensor readings, display updates, MQTT communication, irrigation control, and deep sleep activation.
  
- **Key Functions:**
  - `publishMessage()`: Publishes messages to MQTT topics with retention.
  - `callback()`: Processes incoming MQTT messages and updates sensor values.
  - `publishSensorData()`: Formats and publishes sensor data.
  - `updateDisplay()`: Updates the OLED with the latest sensor readings.
  - `controlServo()`: Activates and controls the servo motor for irrigation.
  - `stayAwakeForUpdates()`: Keeps the system awake for a brief period to process MQTT messages before entering deep sleep.

## Troubleshooting

- **WiFi Connection Issues:**  
  Ensure you are connecting to the WiFiManager AP for configuration. If connection fails, the ESP32 will restart.

- **MQTT Issues:**  
  Verify that your MQTT broker is running and that the IP address is correct. Check the serial monitor for connection logs.

- **Sensor/Display Initialization:**  
  If the OLED or BME280 fails to initialize, check the wiring and I2C addresses. The OLED should be at address `0x3C` and the BME280 at `0x77`.

## Future Enhancements

- Integration with additional sensors (e.g., light, soil nutrients).
- Cloud integration for remote monitoring and data analytics.
- Enhanced user interface for more detailed data presentation.
- Further optimization of power consumption.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Thanks to the developers of the libraries used in this project.
- Special thanks to the ESP32 community for their continued support.

https://www.canva.com/design/DAGfXd2I2Wg/Qn1kp67lJiK5BCjSkkUfvg/edit?utm_content=DAGfXd2I2Wg&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton
