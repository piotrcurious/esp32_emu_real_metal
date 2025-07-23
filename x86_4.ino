// ESP32-based 8086 hardware emulator using FabGL x86 core and MCP23017-based bus emulation // Requires: FabGL library, ESP32, MCP23017 (2 chips for AD0–AD15), and optionally more for control/status

#include <Arduino.h> #include <Wire.h> #include <fabgl.h> #include <Adafruit_MCP23X17.h>

using namespace fabgl;

// ------------ MCP23017 Configuration ------------ // Adafruit_MCP23X17 mcpAD;    // For AD0–AD15 Adafruit_MCP23X17 mcpCTRL;  // For control lines: ALE, RDn, WRn, DT_Rn, A16–A19/S3–S6

// GPIO pins used for READY, HOLD, INTR, NMI const uint8_t PIN_READY = 34; const uint8_t PIN_HOLD = 35; const uint8_t PIN_INTR = 36; const uint8_t PIN_NMI  = 39;

Intel8086 cpu;

// ------------ Simple RAM Model ------------ // #define RAM_SIZE 0x10000  // 64KB RAM for demo uint8_t RAM[RAM_SIZE];

// ------------ Bus Emulator ------------ // class BusEmulator { public: void begin() { Wire.begin(); mcpAD.begin_I2C(0x20);    // AD0–AD15 mcpCTRL.begin_I2C(0x21);  // ALE, RDn, WRn, etc.

for (int i = 0; i < 16; ++i)
  mcpAD.pinMode(i, OUTPUT);

for (int i = 0; i < 8; ++i)
  mcpCTRL.pinMode(i, OUTPUT);

pinMode(PIN_READY, OUTPUT);
pinMode(PIN_HOLD, INPUT);
pinMode(PIN_INTR, INPUT);
pinMode(PIN_NMI, INPUT);

digitalWrite(PIN_READY, HIGH);

}

void latchAddress(uint32_t addr) { for (int i = 0; i < 16; ++i) mcpAD.digitalWrite(i, (addr >> i) & 1); for (int i = 0; i < 4; ++i) mcpCTRL.digitalWrite(i + 4, (addr >> (16 + i)) & 1); mcpCTRL.digitalWrite(0, HIGH);  // ALE delayMicroseconds(1); mcpCTRL.digitalWrite(0, LOW); }

void setDataBus(uint8_t val) { for (int i = 0; i < 8; ++i) { mcpAD.pinMode(i, OUTPUT); mcpAD.digitalWrite(i, (val >> i) & 1); } }

uint8_t readDataBus() { uint8_t val = 0; for (int i = 0; i < 8; ++i) { mcpAD.pinMode(i, INPUT); val |= (mcpAD.digitalRead(i) << i); } return val; }

void writeMemoryByte(uint32_t addr, uint8_t val) { if (addr < RAM_SIZE) { RAM[addr] = val; } else { latchAddress(addr); setDataBus(val); mcpCTRL.digitalWrite(1, HIGH); // RDn mcpCTRL.digitalWrite(2, LOW);  // WRn delayMicroseconds(1); mcpCTRL.digitalWrite(2, HIGH); } }

uint8_t readMemoryByte(uint32_t addr) { if (addr < RAM_SIZE) { return RAM[addr]; } else { latchAddress(addr); mcpCTRL.digitalWrite(2, HIGH);  // WRn mcpCTRL.digitalWrite(1, LOW);   // RDn delayMicroseconds(1); uint8_t val = readDataBus(); mcpCTRL.digitalWrite(1, HIGH); return val; } }

uint8_t readPort(uint16_t port) { switch (port) { case 0x60: return 0x00;            // Keyboard data case 0x61: return 0x80;            // System control port case 0x40: return 0;               // PIT Channel 0 (stub) default: return 0xFF; } }

void writePort(uint16_t port, uint8_t val) { switch (port) { case 0x61: Serial.printf("Port 0x61 write: 0x%02X\n", val); break; case 0x42: Serial.printf("Timer Channel 2 write: 0x%02X\n", val); break; default: Serial.printf("I/O write to port 0x%04X: 0x%02X\n", port, val); break; } }

bool getHOLD() { return digitalRead(PIN_HOLD); }

bool getINTR() { return digitalRead(PIN_INTR); }

bool getNMI() { return digitalRead(PIN_NMI); }

void setREADY(bool ready) { digitalWrite(PIN_READY, ready ? HIGH : LOW); } };

BusEmulator bus;

// ------------ FabGL Memory + I/O Interfaces ------------ // void memRead(uint32_t addr, uint8_t *data, int len) { for (int i = 0; i < len; ++i) data[i] = bus.readMemoryByte(addr + i); }

void memWrite(uint32_t addr, uint8_t *data, int len) { for (int i = 0; i < len; ++i) bus.writeMemoryByte(addr + i, data[i]); }

uint8_t ioRead(uint16_t port) { return bus.readPort(port); }

void ioWrite(uint16_t port, uint8_t val) { bus.writePort(port, val); }

// ------------ Setup and Loop ------------ // void setup() { Serial.begin(115200); bus.begin();

memset(RAM, 0x90, RAM_SIZE); // Fill with NOPs RAM[0x0000] = 0xB8;  // MOV AX, 0x1234 RAM[0x0001] = 0x34; RAM[0x0002] = 0x12; RAM[0x0003] = 0xF4;  // HLT

cpu.setMemoryReader(memRead); cpu.setMemoryWriter(memWrite); cpu.setIOPortReader(ioRead); cpu.setIOPortWriter(ioWrite); cpu.reset(); }

void loop() { bus.setREADY(true); cpu.runInstruction();

if (bus.getINTR()) { // Implement interrupt logic or acknowledge }

if (bus.getNMI()) { // Handle NMI }

if (bus.getHOLD()) { // Grant DMA or external control } }

