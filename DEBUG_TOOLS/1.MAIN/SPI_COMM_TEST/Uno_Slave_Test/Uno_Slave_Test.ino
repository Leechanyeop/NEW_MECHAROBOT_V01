/**
 * @file Uno_SPI_Slave_Test.ino
 * @brief Arduino Uno (Slave) SPI Test Code
 * 
 * Receives data from SPI Master and prints it to Serial.
 * Responds with the received data + 1 (e.g., if Master sends 'A', Slave returns 'B').
 * 
 * Pins for Uno:
 * Pin 13: SCK
 * Pin 12: MISO
 * Pin 11: MOSI
 * Pin 10: SS
 */

#include <SPI.h>

volatile char receivedData = 0;
volatile bool dataReceived = false;

void setup() {
  Serial.begin(115200);
  
  // Set MISO as output (all others are input by default in Slave mode)
  pinMode(MISO, OUTPUT);

  // Turn on SPI in Slave mode
  SPCR |= _BV(SPE);

  // Turn on interrupts
  SPCR |= _BV(SPIE);

  Serial.println("Uno SPI Slave Test Started");
}

// SPI Interrupt Service Routine
ISR (SPI_STC_vect) {
  receivedData = SPDR;       // Read byte from SPDR
  SPDR = receivedData + 1;   // Prepare response for next transfer
  dataReceived = true;
}

void loop() {
  if (dataReceived) {
    Serial.print("Received from Master: ");
    Serial.println(receivedData);
    dataReceived = false;
  }
}
