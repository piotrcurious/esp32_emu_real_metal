// ESP32-based 8086 hardware emulator using FabGL x86 core and MCP23017-based bus emulation // Requires: FabGL library, ESP32, MCP23017 (2 chips for AD0–AD15), and optionally more for control/status

#include <Arduino.h> #include <Wire.h> #include <fabgl.h> #include <Adafruit_MCP23X17.h>

using namespace fabgl;

// ------------ MCP23017 Configuration ------------ // Adafruit_MCP23X17 mcpAD;    // For AD0–AD15 Adafruit_MCP23X17 mcpCTRL;  // For control lines: ALE, RDn, WRn, DT_Rn, A16–A19/S3–S6

// GPIO pins used for READY, HOLD, etc. const uint8_t PIN_READY = 34; const uint8_t PIN_HOLD = 35; const uint8_t PIN_INTR = 36; const uint8_t PIN_NMI  = 39;

Intel8086 cpu;

// ------------ Bus Emulator ------------ // class BusEmulator { public: void begin() { Wire.begin(); mcpAD.begin_I2C(0x20);    // AD0–AD15 mcpCTRL.begin_I2C(0x21);  // ALE, RDn, WRn, etc.

// Set all AD lines as outputs
for (int i = 0; i < 16; ++i)
  mcpAD.pinMode(i, OUTPUT);

// Control line setup
mcpCTRL.pinMode(0, OUTPUT); // ALE
mcpCTRL.pinMode(1, OUTPUT); // RDn
mcpCTRL.pinMode(2, OUTPUT); // WRn
mcpCTRL.pinMode(3, OUTPUT); // DT_Rn
mcpCTRL.pinMode(4, OUTPUT); // A16/S3
mcpCTRL.pinMode(5, OUTPUT); // A17/S4
mcpCTRL.pinMode(6, OUTPUT); // A18/S5
mcpCTRL.pinMode(7, OUTPUT); // A19/S6

pinMode(PIN_READY, OUTPUT);
pinMode(PIN_HOLD, INPUT);
pinMode(PIN_INTR, INPUT);
pinMode(PIN_NMI, INPUT);

}

void latchAddress(uint32_t addr) { for (int i = 0; i < 16; ++i) mcpAD.digitalWrite(i, (addr >> i) & 1); for (int i = 0; i < 4; ++i) mcpCTRL.digitalWrite(i + 4, (addr >> (16 + i)) & 1);

mcpCTRL.digitalWrite(0, HIGH);  // ALE
delayMicroseconds(1);
mcpCTRL.digitalWrite(0, LOW);

}

void setDataBus(uint8_t val) { for (int i = 0; i < 8; ++i) { mcpAD.pinMode(i, OUTPUT); mcpAD.digitalWrite(i, (val >> i) & 1); } }

uint8_t readDataBus() { uint8_t val = 0; for (int i = 0; i < 8; ++i) { mcpAD.pinMode(i, INPUT); val |= (mcpAD.digitalRead(i) << i); } return val; }

void writeMemoryByte(uint32_t addr, uint8_t val) { latchAddress(addr); setDataBus(val); mcpCTRL.digitalWrite(1, HIGH); // RDn mcpCTRL.digitalWrite(2, LOW);  // WRn delayMicroseconds(1); mcpCTRL.digitalWrite(2, HIGH); }

uint8_t readMemoryByte(uint32_t addr) { latchAddress(addr); mcpCTRL.digitalWrite(2, HIGH);  // WRn mcpCTRL.digitalWrite(1, LOW);   // RDn delayMicroseconds(1); uint8_t val = readDataBus(); mcpCTRL.digitalWrite(1, HIGH); return val; }

uint8_t readPort(uint16_t port) { return 0xFF; // Placeholder }

void writePort(uint16_t port, uint8_t val) { // Placeholder for external port simulation } };

BusEmulator bus;

// ------------ FabGL Memory + I/O Interfaces ------------ // void memRead(uint32_t addr, uint8_t *data, int len) { for (int i = 0; i < len; ++i) data[i] = bus.readMemoryByte(addr + i); }

void memWrite(uint32_t addr, uint8_t *data, int len) { for (int i = 0; i < len; ++i) bus.writeMemoryByte(addr + i, data[i]); }

uint8_t ioRead(uint16_t port) { return bus.readPort(port); }

void ioWrite(uint16_t port, uint8_t val) { bus.writePort(port, val); }

// ------------ Setup and Loop ------------ // void setup() { Serial.begin(115200); bus.begin();

cpu.setMemoryReader(memRead); cpu.setMemoryWriter(memWrite); cpu.setIOPortReader(ioRead); cpu.setIOPortWriter(ioWrite); cpu.reset(); }

void loop() { cpu.runInstruction(); }

