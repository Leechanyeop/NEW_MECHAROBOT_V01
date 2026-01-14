#ifndef SENDOPC_H
#define SENDOPC_H

#include <WiFiNINA.h>
#include <ArduinoModbus.h>

// WiFi 설정 (CTL_UNOWiFi.ino와 공유하기 위해 extern 처리)
extern char ssid[];
extern char pass[];

// ModbusTCP 서버 정보 (마스터 -> 외부 서버/OPC 연결용)
extern IPAddress serverIP;
extern uint16_t serverPort;

extern WiFiClient opcWifiClient;
extern ModbusTCPClient modbusTCPClient;

// 레지스터 맵
const uint16_t REG_POS_X_LOW   = 40001;
const uint16_t REG_POS_X_HIGH  = 40002;
const uint16_t REG_ENC_L_LOW   = 40003;
const uint16_t REG_ENC_L_HIGH  = 40004;
const uint16_t REG_ENC_R_LOW   = 40005;
const uint16_t REG_ENC_R_HIGH  = 40006;
const uint16_t REG_SPEED       = 40007;
const uint16_t REG_STATUS      = 40008;

// 자동 보정된 오프셋
extern int32_t registerOffset;

// 함수 프로토타입
uint32_t reg(uint32_t orig);
uint16_t readRegisterSafe(uint32_t address);
bool writeRegisterVerify(uint32_t address, uint16_t value);
bool writeFloatToRegisters(uint32_t startOrig, float value);
void detectRegisterOffset();

#endif