#include <Arduino_LSM6DS3.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    while (1);
  }
  pinMode(10, OUTPUT); // 안전용 SS 핀
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    Serial.print("A: ");
    Serial.print(ax, 3); Serial.print(" ");
    Serial.print(ay, 3); Serial.print(" ");
    Serial.print(az, 3); Serial.print("  G: ");
    Serial.print(gx, 3); Serial.print(" ");
    Serial.print(gy, 3); Serial.print(" ");
    Serial.println(gz, 3);
  }
  delay(150);
}