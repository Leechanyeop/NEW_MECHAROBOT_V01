void IRREAD_UDP(IPAddress remoteIp, unsigned int remotePort) {
  int sensorValues[6];
  ir.readValues(sensorValues);

  // 6개 값을 바이트 배열로 전송 (각각 0~255 범위로 압축하거나 2바이트로 보낼 수 있음)
  uint8_t out[12]; // 6개 * 2바이트 (uint16_t)
  for (int i = 0; i < 6; i++) {
    out[i*2]   = sensorValues[i] & 0xFF;
    out[i*2+1] = (sensorValues[i] >> 8) & 0xFF;
  }

  udp.beginPacket(remoteIp, remotePort);
  udp.write(out, sizeof(out));
  udp.endPacket();
}