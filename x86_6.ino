// ESP32-based 8086 hardware emulator using FabGL's x86 core and MCP23017-based bus emulation.
//
// --- VERSION 2.0 - Improved and Corrected ---
//
// This version fixes critical bugs related to 16-bit data handling and improves performance.
// The emulator now correctly uses the high and low bytes of the data bus based on address.
//
// Requires:
// - An ESP32 development board
// - FabGL library installed in Arduino IDE
// - Adafruit MCP23017 library installed in Arduino IDE
// - 2x MCP23017 I2C I/O Expander ICs
// - Appropriate wiring for I2C and the bus signals

#include <Arduino.h>
#include <Wire.h>
#include <fabgl.h>
#include <Adafruit_MCP23X17.h>

using namespace fabgl;

// ------------ MCP23017 Configuration ------------ //
Adafruit_MCP23X17 mcpAD;    // For AD0–AD15. I2C Address 0x20.
Adafruit_MCP23X17 mcpCTRL;  // For control lines. I2C Address 0x21.

// Control signal pins on the mcpCTRL expander (GPA port)
#define MCP_PIN_ALE   0 // Address Latch Enable
#define MCP_PIN_RDn   1 // Read (active low)
#define MCP_PIN_WRn   2 // Write (active low)
#define MCP_PIN_DT_Rn 3 // Data Transmit/Receive (Not used in this simplified model)
#define MCP_PIN_A16   4 // High address lines A16-A19
#define MCP_PIN_A17   5
#define MCP_PIN_A18   6
#define MCP_PIN_A19   7

// GPIO pins on the ESP32 used for direct CPU signals
const uint8_t PIN_READY = 34; // Input to CPU to insert wait states
const uint8_t PIN_HOLD  = 35; // Input to CPU to request bus mastership (DMA)
const uint8_t PIN_INTR  = 36; // Maskable Interrupt Request
const uint8_t PIN_NMI   = 39; // Non-Maskable Interrupt

Intel8086 cpu;

// ------------ Simple RAM Model ------------ //
#define RAM_SIZE 0x10000  // 64KB of internal RAM for demonstration
uint8_t RAM[RAM_SIZE];

// ------------ Bus Emulator Class ------------ //
// This class encapsulates all logic for interacting with the emulated hardware bus.
class BusEmulator {
public:
  void begin() {
    Wire.begin();
    // Use Fast Mode I2C for better performance
    Wire.setClock(400000L);

    if (!mcpAD.begin_I2C(0x20)) {
      Serial.println("FATAL: MCP23017 for AD bus (0x20) not found.");
      while(1);
    }
    if (!mcpCTRL.begin_I2C(0x21)) {
      Serial.println("FATAL: MCP23017 for CTRL bus (0x21) not found.");
      while(1);
    }

    // Configure all control pins on mcpCTRL as outputs.
    mcpCTRL.writeRegister(MCP23X17_IODIRA, 0x00);

    // Set initial states for control lines (all inactive)
    mcpCTRL.digitalWrite(MCP_PIN_RDn, HIGH);
    mcpCTRL.digitalWrite(MCP_PIN_WRn, HIGH);
    mcpCTRL.digitalWrite(MCP_PIN_ALE, LOW);

    // Configure ESP32 GPIO pins
    pinMode(PIN_READY, OUTPUT);
    pinMode(PIN_HOLD, INPUT_PULLUP);
    pinMode(PIN_INTR, INPUT_PULLUP);
    pinMode(PIN_NMI, INPUT_PULLUP);

    digitalWrite(PIN_READY, HIGH);
  }

  // Latches a 20-bit address onto the bus.
  void latchAddress(uint32_t addr) {
    // Set AD bus pins to output to drive the address
    mcpAD.writeRegister(MCP23X17_IODIRB, 0x00); // AD8-15
    mcpAD.writeRegister(MCP23X17_IODIRA, 0x00); // AD0-7

    // Place A0-A15 on the AD bus lines
    mcpAD.writeGPIOAB(addr & 0xFFFF);

    // Place A16-A19 on the control bus lines
    uint8_t ctrl_port_val = mcpCTRL.readRegister(MCP23X17_GPIOA);
    ctrl_port_val &= 0x0F; // Clear upper 4 bits
    ctrl_port_val |= ((addr >> 12) & 0xF0); // Set A16-A19
    mcpCTRL.writeRegister(MCP23X17_GPIOA, ctrl_port_val);

    // Pulse ALE high to latch the address in external hardware
    mcpCTRL.digitalWrite(MCP_PIN_ALE, HIGH);
    delayMicroseconds(1);
    mcpCTRL.digitalWrite(MCP_PIN_ALE, LOW);
  }

  // Writes a byte to a memory address. Handles high/low byte placement correctly.
  void writeMemoryByte(uint32_t addr, uint8_t val) {
    if (addr < RAM_SIZE) {
      RAM[addr] = val; // Internal RAM access
      return;
    }

    // External bus access
    latchAddress(addr);
    
    // An 8086 uses BHE# and A0 to select bytes. We emulate this with the address parity.
    if ((addr & 1) == 0) { // Even address: Use low byte (AD0-7)
      mcpAD.writeRegister(MCP23X17_IODIRA, 0x00); // Set Port A to output
      mcpAD.writeGPIOA(val);
    } else { // Odd address: Use high byte (AD8-15)
      mcpAD.writeRegister(MCP23X17_IODIRB, 0x00); // Set Port B to output
      mcpAD.writeGPIOB(val);
    }

    // Pulse WRn low to signal a write
    mcpCTRL.digitalWrite(MCP_PIN_WRn, LOW);
    delayMicroseconds(1);
    mcpCTRL.digitalWrite(MCP_PIN_WRn, HIGH);
  }

  // Reads a byte from a memory address. Handles high/low byte placement correctly.
  uint8_t readMemoryByte(uint32_t addr) {
    if (addr < RAM_SIZE) {
      return RAM[addr]; // Internal RAM access
    }

    // External bus access
    latchAddress(addr);
    uint8_t val = 0;

    // Pulse RDn low to signal a read
    mcpCTRL.digitalWrite(MCP_PIN_RDn, LOW);
    delayMicroseconds(1);

    if ((addr & 1) == 0) { // Even address: Read from low byte (AD0-7)
      mcpAD.writeRegister(MCP23X17_IODIRA, 0xFF); // Set Port A to input
      val = mcpAD.readGPIOA();
    } else { // Odd address: Read from high byte (AD8-15)
      mcpAD.writeRegister(MCP23X17_IODIRB, 0xFF); // Set Port B to input
      val = mcpAD.readGPIOB();
    }
    
    mcpCTRL.digitalWrite(MCP_PIN_RDn, HIGH);
    return val;
  }

  // Reads from an I/O port (peripheral).
  uint8_t readPort(uint16_t port) {
    Serial.printf("I/O Read from port 0x%04X\n", port);
    // In a real system, you would perform a read cycle similar to memory read.
    return 0xFF;
  }

  // Writes to an I/O port (peripheral).
  void writePort(uint16_t port, uint8_t val) {
    Serial.printf("I/O Write to port 0x%04X with value 0x%02X\n", port, val);
    // In a real system, you would perform a write cycle.
  }

  bool getHOLD() { return digitalRead(PIN_HOLD) == LOW; }
  bool getINTR() { return digitalRead(PIN_INTR) == LOW; }
  bool getNMI()  { return digitalRead(PIN_NMI) == LOW; }
  void setREADY(bool ready) { digitalWrite(PIN_READY, ready ? HIGH : LOW); }
};

BusEmulator bus;

// ------------ FabGL CPU Interface Callbacks ------------ //
void memRead(uint32_t addr, uint8_t *data, int len) {
  for (int i = 0; i < len; ++i) {
    data[i] = bus.readMemoryByte(addr + i);
  }
}

void memWrite(uint32_t addr, uint8_t *data, int len) {
  for (int i = 0; i < len; ++i) {
    bus.writeMemoryByte(addr + i, data[i]);
  }
}

uint8_t ioRead(uint16_t port) { return bus.readPort(port); }
void ioWrite(uint16_t port, uint8_t val) { bus.writePort(port, val); }

// ------------ Helper Functions ------------ //
void printCPUState() {
  auto regs = cpu.getRegisters();
  Serial.println("--- CPU State ---");
  Serial.printf("AX=%04X  BX=%04X  CX=%04X  DX=%04X\n", regs->AX, regs->BX, regs->CX, regs->DX);
  Serial.printf("SI=%04X  DI=%04X  BP=%04X  SP=%04X\n", regs->SI, regs->DI, regs->BP, regs->SP);
  Serial.printf("CS=%04X  DS=%04X  SS=%04X  ES=%04X\n", regs->CS, regs->DS, regs->SS, regs->ES);
  Serial.printf("IP=%04X  FLAGS=%04X\n", regs->IP, regs->FLAGS);
  Serial.println("-----------------");
}

// ------------ Setup and Loop ------------ //
void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32 8086 Hardware Emulator v2.0 - Starting...");

  bus.begin();

  // Load a simple test program into the emulated RAM
  memset(RAM, 0x90, RAM_SIZE); // Fill RAM with NOPs
  
  // Program at 0000:0100
  uint16_t startAddr = 0x0100;
  RAM[startAddr++] = 0xB8; // MOV AX, 0x1234
  RAM[startAddr++] = 0x34;
  RAM[startAddr++] = 0x12;
  RAM[startAddr++] = 0xBB; // MOV BX, 0xABCD
  RAM[startAddr++] = 0xCD;
  RAM[startAddr++] = 0xAB;
  RAM[startAddr++] = 0x01; // ADD AX, BX ; AX = 1234 + ABCD = BE01
  RAM[startAddr++] = 0xD8;
  RAM[startAddr++] = 0x89; // MOV [0200], AX ; Store result
  RAM[startAddr++] = 0x06;
  RAM[startAddr++] = 0x00;
  RAM[startAddr++] = 0x02;
  RAM[startAddr++] = 0xF4; // HLT

  // Set the callback functions for the CPU emulator
  cpu.setMemoryReader(memRead);
  cpu.setMemoryWriter(memWrite);
  cpu.setIOPortReader(ioRead);
  cpu.setIOPortWriter(ioWrite);

  cpu.reset();

  // Set starting address to 0000:0100
  auto regs = cpu.getRegisters();
  regs->CS = 0x0000;
  regs->IP = 0x0100;
  cpu.setRegisters(regs);

  Serial.println("CPU Initialized. Executing test program...");
  printCPUState();
}

void loop() {
  // Execute instructions until the CPU is halted
  uint32_t instruction_count = 0;
  const uint32_t timeout = 500000; // Timeout to prevent infinite loops

  while (!cpu.isHalted() && instruction_count < timeout) {
    // Handle HOLD, INTR, NMI if necessary (omitted for this simple test)
    cpu.runInstruction();
    instruction_count++;
  }

  // --- Execution Finished ---
  Serial.printf("\nExecution finished after %u instructions.\n", instruction_count);
  if (cpu.isHalted()) {
    Serial.println("CPU is Halted.");
  } else {
    Serial.println("Warning: Execution stopped due to timeout.");
  }

  Serial.println("\nFinal CPU State:");
  printCPUState();

  // Verify the result stored in RAM
  uint16_t result = RAM[0x0200] | (RAM[0x0201] << 8);
  Serial.printf("\nVerification: Value at RAM[0x0200] is 0x%04X (Expected 0xBE01)\n", result);

  // Stop the loop
  while(1) {
    delay(1000);
  }
}
