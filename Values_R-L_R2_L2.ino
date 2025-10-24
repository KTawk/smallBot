#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      Serial.printf("PS4 Controller connected (index=%d)\n", i);
      return;
    }
  }
  Serial.println("No empty slot for new controller.");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      Serial.printf("Controller disconnected (index=%d)\n", i);
      return;
    }
  }
}

void processGamepad(ControllerPtr ctl) {
  int lx = ctl->axisX();   // Left stick X (-511 to 512)
  int ly = ctl->axisY();   // Left stick Y (-511 to 512)
  int rx = ctl->axisRX();  // Right stick X (-511 to 512)
  int ry = ctl->axisRY();  // Right stick Y (-511 to 512)
  int l2 = ctl->brake();   // L2 trigger (0 to 1023)
  int r2 = ctl->throttle(); // R2 trigger (0 to 1023)

  Serial.printf("LX:%4d  LY:%4d  RX:%4d  RY:%4d  L2:%4d  R2:%4d\n", lx, ly, rx, ry, l2, r2);
}

void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData()) {
      processGamepad(ctl);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("ESP32 BT Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();  // optional: clear old pairings
}

void loop() {
  if (BP32.update())
    processControllers();

  delay(100);
}
