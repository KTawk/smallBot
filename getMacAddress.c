//After uploading the following script to the ESP32, hold the boot button then release it. 
// After that you see the MAc Address in the Serial Monitor.

#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32");

  Serial.println("ESP32 Bluetooth address:");

  uint8_t mac[6];
  SerialBT.getBtAddress(mac);   // <-- Correct usage: pass buffer

  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}

void loop() {}