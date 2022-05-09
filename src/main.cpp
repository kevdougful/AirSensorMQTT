#include "Zanshin_BME680.h"

const uint32_t SERIAL_SPEED{115200};  ///< Set the baud rate for Serial I/O

BME680_Class BME680;

void setup() {
  Serial.begin(SERIAL_SPEED);  // Start serial port at Baud rate
  Serial.print(F("- Initializing BME680 sensor\n"));
  while (!BME680.begin(I2C_STANDARD_MODE)) {
    // Start BME680 using I2C, use first device found
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  // of loop until device is located
  Serial.print(F("- Setting 16x oversampling for all sensors\n"));
  BME680.setOversampling(TemperatureSensor, Oversample16);
  BME680.setOversampling(HumiditySensor, Oversample16);
  BME680.setOversampling(PressureSensor, Oversample16);
  Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680.setIIRFilter(IIR4);
  Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));
  BME680.setGas(320, 150);
}

void loop() {
  static int32_t  temp, humidity, pressure, gas;  // BME readings
  static char     buf[16];                        // sprintf text buffer

  BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings

  // Take it easy for a bit
  delay(2000);
}