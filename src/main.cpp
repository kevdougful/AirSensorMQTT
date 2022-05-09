#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <WiFi.h>
#include "Zanshin_BME680.h"

const uint32_t SERIAL_SPEED{115200};

// WiFi Config
#define WIFI_SSID         "COFFEY"
#define WIFI_PASS         "NewOEtHERcHDAntR"
WiFiClient wifiClient;

// MQTT Config
#define MQTT_SERVER       "192.168.1.37"
#define MQTT_SERVERPORT   1883 // use 8883 for SSL
#define MQTT_USERNAME     ""
#define MQTT_PASS         ""
Adafruit_MQTT_Client mqtt(&wifiClient, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_PASS);
// Topics
Adafruit_MQTT_Publish bme688_temperature(&mqtt, "shed/tent/bme688/temperature");
Adafruit_MQTT_Publish bme688_humidity(&mqtt, "shed/tent/bme688/humidity");
Adafruit_MQTT_Publish bme688_pressure(&mqtt, "shed/tent/bme688/pressure");
Adafruit_MQTT_Publish bme688_voc(&mqtt, "shed/tent/bme688/voc");

BME680_Class BME680;

void connectToWiFi(const char *ssid, const char *pass) {
  Serial.print(F("- Connecting to WiFi"));
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // Connected
  Serial.println();
  Serial.println("- WiFi connected!");
  Serial.print("- IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(SERIAL_SPEED);  // Start serial port at Baud rate

  connectToWiFi(WIFI_SSID, WIFI_PASS);

  // BME688 Setup
  Serial.println(F("- Initializing BME688 sensor"));
  while (!BME680.begin(I2C_STANDARD_MODE)) {
    // Start BME688 using I2C, use first device found
    Serial.println(F("-  Unable to find BME688. Trying again in 5 seconds."));
    delay(5000);
  }
  BME680.setOversampling(TemperatureSensor, Oversample16);
  BME680.setOversampling(HumiditySensor, Oversample16);
  BME680.setOversampling(PressureSensor, Oversample16);
  BME680.setIIRFilter(IIR4);
  BME680.setGas(320, 150);
  Serial.println(F("- BME688 Initialized."));
}

void connectToMqttBroker() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }

  Serial.println("MQTT Connected!");
}

void loop() {
  static int32_t  temp, humidity, pressure, voc;  // BME688 readings
  static char     buf[16];                        // sprintf text buffer

  BME680.getSensorData(temp, humidity, pressure, voc);

  connectToMqttBroker();

  bme688_temperature.publish((float)temp / (float)100);
  bme688_humidity.publish((float)humidity / (float)1000);
  bme688_pressure.publish((float)pressure / (float)100);
  bme688_voc.publish((float)voc / (float)100);
  // Take it easy for a bit
  delay(2000);
}
