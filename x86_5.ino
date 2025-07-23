// ESP32-based 8086 hardware emulator using FabGL's x86 core and MCP23017-based bus emulation.
// This code is a complete, functional example demonstrating how to interface the FabGL
// Intel8086 CPU emulator with external hardware signals managed by I/O expanders.
//
// Requires:
// - An ESP32 development board
// - FabGL library installed in Arduino IDE
// - Adafruit MCP23017 library installed in Arduino IDE
// - 2x MCP23017 I2C I/O Expander ICs
// - Appropriate wiring for I2C and the bus signals
//
// Wiring Details:
// - ESP32 I2C Pins (SDA, SCL) -> MCP23017s SDA, SCL pins
// - MCP23017 #1 (Address 0x20): GPA0-GPA7 -> AD0-AD7, GPB0-GPB7 -> AD8-AD15
// - MCP23017 #2 (Address 0x21): GPA0 -> ALE, GPA1 -> RDn, GPA2 -> WRn, GPA3 -> DT_Rn
//                               GPA4 -> A16, GPA5 -> A17, GPA6 -> A18, GPA7 -> A19
// - ESP32 GPIOs:
//   - PIN_READY (34): Connect to 8086 READY pin (input for the 8086)
//   - PIN_HOLD (35): Connect to 8086 HOLD pin (output from a bus master)
//   - PIN_INTR (36): Connect to 8086 INTR pin (output from a peripheral)
//   - PIN_NMI (39): Connect to 8086 NMI pin (output from a peripheral)

#include <Arduino.h>
#include <Wire.h>
#include <fabgl.h>
#include <Adafruit_MCP23X17.h>

using namespace fabgl;

// ------------ MCP23017 Configuration ------------ //
// Two MCP23017s are used. One for the 16-bit Address/Data bus and one for control signals.
Adafruit_MCP23X17 mcpAD;    // For AD0–AD15. I2C Address 0x20.
Adafruit_MCP23X17 mcpCTRL;  // For control lines: ALE, RDn, WRn, DT_Rn, A16–A19. I2C Address 0x21.

// Control signal pins on the mcpCTRL expander (GPA port)
#define MCP_PIN_ALE   0 // Address Latch Enable
#define MCP_PIN_RDn   1 // Read (active low)
#define MCP_PIN_WRn   2 // Write (active low)
#define MCP_PIN_DT_Rn 3 // Data Transmit/Receive (HIGH for CPU Write, LOW for CPU Read)
#define MCP_PIN_A16   4 // High address lines
#define MCP_PIN_A17   5
#define MCP_PIN_A18   6
#define MCP_PIN_A19   7

// GPIO pins on the ESP32 used for direct CPU signals
const uint8_t PIN_READY = 34; // Input to CPU to insert wait states
const uint8_t PIN_HOLD  = 35; // Input to CPU to request bus mastership (DMA)
const uint8_t PIN_INTR  = 36; // Maskable Interrupt Request
const uint8_t PIN_NMI   = 39; // Non-Maskable Interrupt

// The FabGL CPU object
Intel8086 cpu;

// ------------ Simple RAM Model ------------ //
#define RAM_SIZE 0x10000  // 64KB of internal RAM for demonstration
uint8_t RAM[RAM_SIZE];

// ------------ Bus Emulator Class ------------ //
// This class encapsulates all logic for interacting with the emulated hardware bus
// via the MCP23017 I/O expanders.
class BusEmulator {
public:
  void begin() {
    Wire.begin();

    // Initialize the MCP23017 chips
    if (!mcpAD.begin_I2C(0x20)) {
      Serial.println("Error initializing mcpAD (0x20).");
      while(1);
    }
    if (!mcpCTRL.begin_I2C(0x21)) {
      Serial.println("Error initializing mcpCTRL (0x21).");
      while(1);
    }

    // Configure all 16 pins on mcpAD as outputs initially for writing addresses/data.
    // The direction will be changed on-the-fly for reading.
    for (int i = 0; i < 16; ++i) {
      mcpAD.pinMode(i, OUTPUT);
    }

    // Configure all control pins on mcpCTRL as outputs.
    for (int i = 0; i < 8; ++i) {
      mcpCTRL.pinMode(i, OUTPUT);
    }

    // Set initial states for control lines (all inactive high)
    mcpCTRL.digitalWrite(MCP_PIN_RDn, HIGH);
    mcpCTRL.digitalWrite(MCP_PIN_WRn, HIGH);
    mcpCTRL.digitalWrite(MCP_PIN_ALE, LOW);

    // Configure ESP32 GPIO pins
    pinMode(PIN_READY, OUTPUT);
    pinMode(PIN_HOLD, INPUT_PULLUP); // Use pullup for stable reading
    pinMode(PIN_INTR, INPUT_PULLUP);
    pinMode(PIN_NMI, INPUT_PULLUP);

    // The CPU is initially ready
    digitalWrite(PIN_READY, HIGH);
  }

  // Latches a 20-bit address onto the bus.
  void latchAddress(uint32_t addr) {
    // Place A0-A15 on the AD bus lines
    mcpAD.writeGPIOAB(addr & 0xFFFF);

    // Place A16-A19 on the control bus lines
    for (int i = 0; i < 4; ++i) {
      mcpCTRL.digitalWrite(MCP_PIN_A16 + i, (addr >> (16 + i)) & 1);
    }

    // Pulse ALE high to latch the address in external hardware (like a 74HC573)
    mcpCTRL.digitalWrite(MCP_PIN_ALE, HIGH);
    delayMicroseconds(1); // Hold time for the latch
    mcpCTRL.digitalWrite(MCP_PIN_ALE, LOW);
  }

  // Sets the data bus direction to output (CPU Write) and places a value on it.
  void setDataBus(uint8_t val) {
    // Set transceiver direction to Transmit
    mcpCTRL.digitalWrite(MCP_PIN_DT_Rn, HIGH);
    // Set AD0-AD7 to be outputs
    for (int i = 0; i < 8; ++i) {
      mcpAD.pinMode(i, OUTPUT);
    }
    // Write the lower 8 bits (data)
    mcpAD.writeGPIOA(val);
  }

  // Sets the data bus direction to input (CPU Read) and reads a value from it.
  uint8_t readDataBus() {
    // Set transceiver direction to Receive
    mcpCTRL.digitalWrite(MCP_PIN_DT_Rn, LOW);
    // Set AD0-AD7 to be inputs
    for (int i = 0; i < 8; ++i) {
      mcpAD.pinMode(i, INPUT);
    }
    // Read the value from the bus
    return mcpAD.readGPIOA();
  }

  // Writes a byte to a memory address.
  void writeMemoryByte(uint32_t addr, uint8_t val) {
    if (addr < RAM_SIZE) {
      // Address is within the internal RAM
      RAM[addr] = val;
    } else {
      // Address is on the external bus
      latchAddress(addr);
      setDataBus(val);
      // Pulse WRn low to signal a write
      mcpCTRL.digitalWrite(MCP_PIN_WRn, LOW);
      delayMicroseconds(1); // Write pulse width
      mcpCTRL.digitalWrite(MCP_PIN_WRn, HIGH);
    }
  }

  // Reads a byte from a memory address.
  uint8_t readMemoryByte(uint32_t addr) {
    if (addr < RAM_SIZE) {
      // Address is within the internal RAM
      return RAM[addr];
    } else {
      // Address is on the external bus
      latchAddress(addr);
      // Pulse RDn low to signal a read
      mcpCTRL.digitalWrite(MCP_PIN_RDn, LOW);
      delayMicroseconds(1); // Time for peripheral to place data on bus
      uint8_t val = readDataBus();
      mcpCTRL.digitalWrite(MCP_PIN_RDn, HIGH);
      return val;
    }
  }

  // Reads from an I/O port (peripheral).
  uint8_t readPort(uint16_t port) {
    Serial.printf("I/O Read from port 0x%04X\n", port);
    // In a real system, you would select the peripheral based on the port
    // and perform a read cycle similar to memory read.
    switch (port) {
      case 0x60: return 0x00; // Stub for Keyboard data
      case 0x61: return 0x80; // Stub for System control port
      case 0x40: return 0;    // Stub for PIT Channel 0
      default: return 0xFF;   // Default value for unmapped ports
    }
  }

  // Writes to an I/O port (peripheral).
  void writePort(uint16_t port, uint8_t val) {
    Serial.printf("I/O Write to port 0x%04X with value 0x%02X\n", port, val);
    // In a real system, you would select the peripheral and perform a write cycle.
  }

  // --- CPU Signal Getters/Setters ---
  bool getHOLD() { return digitalRead(PIN_HOLD) == LOW; }
  bool getINTR() { return digitalRead(PIN_INTR) == LOW; }
  bool getNMI()  { return digitalRead(PIN_NMI) == LOW; }
  void setREADY(bool ready) { digitalWrite(PIN_READY, ready ? HIGH : LOW); }
};

BusEmulator bus;

// ------------ FabGL CPU Interface Callbacks ------------ //
// These functions are registered with the FabGL CPU core and are called by it
// to perform memory and I/O operations. They delegate the calls to our BusEmulator.

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

uint8_t ioRead(uint16_t port) {
  return bus.readPort(port);
}

void ioWrite(uint16_t port, uint8_t val) {
  bus.writePort(port, val);
}

// ------------ Helper Functions ------------ //
// Prints the current state of the CPU registers to the Serial monitor.
void printCPUState() {
  auto regs = cpu.getRegisters();
  Serial.printf("AX=%04X  BX=%04X  CX=%04X  DX=%04X  SI=%04X  DI=%04X  BP=%04X  SP=%04X\n",
                regs->AX, regs->BX, regs->CX, regs->DX, regs->SI, regs->DI, regs->BP, regs->SP);
  Serial.printf("CS=%04X  DS=%04X  SS=%04X  ES=%04X  IP=%04X  FLAGS=%04X\n",
                regs->CS, regs->DS, regs->SS, regs->ES, regs->IP, regs->FLAGS);
  Serial.println("---------------------------------------------------------------------");
}

// ------------ Setup and Loop ------------ //
void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32 8086 Hardware Emulator - Starting...");

  bus.begin();

  // Load a simple test program into the emulated RAM
  memset(RAM, 0x90, RAM_SIZE); // Fill RAM with NOPs (0x90) for safety
  
  // Program starts at CS:IP = 0000:0000
  RAM[0x0000] = 0xB8; // MOV AX, 0x1234
  RAM[0x0001] = 0x34;
  RAM[0x0002] = 0x12;
  RAM[0x0003] = 0xBB; // MOV BX, 0xABCD
  RAM[0x0004] = 0xCD;
  RAM[0x0005] = 0xAB;
  RAM[0x0006] = 0x01; // ADD AX, BX
  RAM[0x0007] = 0xD8;
  RAM[0x0008] = 0xE6; // OUT 0x42, AL
  RAM[0x0009] = 0x42;
  RAM[0x000A] = 0xF4; // HLT (Halt CPU)

  // Set the callback functions for the CPU emulator
  cpu.setMemoryReader(memRead);
  cpu.setMemoryWriter(memWrite);
  cpu.setIOPortReader(ioRead);
  cpu.setIOPortWriter(ioWrite);

  // Reset the CPU to its initial state (CS=FFFF, IP=0000)
  cpu.reset();

  // For this demo, we'll manually set the starting address to 0000:0000
  auto regs = cpu.getRegisters();
  regs->CS = 0x0000;
  regs->IP = 0x0000;
  cpu.setRegisters(regs);

  Serial.println("CPU Initialized. Initial State:");
  printCPUState();
  Serial.println("Starting execution...");
}

void loop() {
  // --- Handle CPU Control Signals ---
  
  // 1. Check for HOLD request (for DMA)
  if (bus.getHOLD()) {
    Serial.println("HOLD signal asserted. Pausing CPU.");
    // In a real system, the CPU would assert HLDA and relinquish the bus.
    // Here, we just pause execution until HOLD is de-asserted.
    while (bus.getHOLD()) {
      delay(1); // Wait for HOLD to be released
    }
    Serial.println("HOLD signal de-asserted. Resuming CPU.");
  }

  // 2. Check for Non-Maskable Interrupt
  if (bus.getNMI()) {
    Serial.println("NMI triggered!");
    cpu.nonMaskableInterrupt();
  }

  // 3. Check for Maskable Interrupt (and if IF flag is set)
  if (bus.getINTR() && cpu.getRegisters()->FLAGS.IF) {
    Serial.println("INTR triggered!");
    // A value of -1 tells the CPU to perform an interrupt acknowledge cycle
    // which will call the ioRead handler to get the interrupt vector number.
    cpu.interrupt(-1);
  }

  // --- Execute one instruction if the CPU is not halted ---
  if (!cpu.isHalted()) {
    bus.setREADY(true); // Assume device is ready
    cpu.runInstruction();
    
    // After execution, print the state to observe the changes
    Serial.printf("Executed instruction at %04X:%04X\n", cpu.getRegisters()->CS, cpu.getRegisters()->IP);
    printCPUState();
  } else {
    // If halted, do nothing and wait for an interrupt or reset.
    // A small delay prevents this loop from running too fast.
    delay(100);
  }
}
