#include "esp_camera.h"
#include "ESP32QRCodeReader.h"

ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER);

void setup() {
  Serial.begin(115200);
  reader.setup();
  reader.begin();
}

void loop() {
  if (reader.available()) {
    QRCode qrCode = reader.read();
    Serial.print("QR Code: ");
    Serial.println(qrCode.payload);
  }
}