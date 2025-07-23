#include <Arduino.h>
#include <Wire.h>
#include <fabgl.h>

// This project emulates an Intel 8086 CPU and provides a physical bus interface
// using PCF8574 I/O expanders. This "completed" version is designed to
// interface with an external 32KB (AS6C62256 or similar) static RAM chip.
//
// Hardware Connections:
// ESP32:
// - I2C SDA/SCL connected to all PCF8574s.
//
// PCF8574s (Address jumpers set accordingly):
// - PCF8574 #1 (0x20): 8086 Address Bus A0-A7
// - PCF8574 #2 (0x21): 8086 Address Bus A8-A15
// - PCF8574 #3 (0x22): 8086 Address Bus A16-A19 (pins 0-3) & Control Signals (pins 4-7)
// - PCF8574 #4 (0x23): 8086 Data Bus D0-D7
// - PCF8574 #5 (0x24): 8086 Data Bus D8-D15
//
// External SRAM (e.g., AS6C62256 - 32K x 8):
// - SRAM A0-A14 -> Bus A0-A14
// - SRAM I/O0-I/O7 -> Bus D0-D7 (Note: We are only using the lower 8 bits of the data bus for this simple example)
// - SRAM /CE (Chip Enable) -> GND (Always enabled)
// - SRAM /OE (Output Enable) -> Bus /RD signal
// - SRAM /WE (Write Enable) -> Bus /WR signal

// PCF8574 I2C addresses
#define PCF_ADDR_A0_A7      0x20
#define PCF_ADDR_A8_A15     0x21
#define PCF_ADDR_A16_A19_CTRL 0x22
#define PCF_ADDR_D0_D7      0x23
#define PCF_ADDR_D8_D15     0x24

// PCF8574 Control Signal Pins (on expander at 0x22)
// Note: These signals are active-low on a real 8086 bus.
// We will treat HIGH as "asserted" in this code for simplicity.
#define PIN_A16_A19_MASK 0x0F // Pins 0-3 for A16-A19
#define PIN_RD           4
#define PIN_WR           5
#define PIN_MIO          6
#define PIN_ALE          7

// Memory Map
#define EXT_RAM_START 0x00000
#define EXT_RAM_END   0x07FFF // 32KB RAM
#define BIOS_ROM_START 0xF0000
#define BIOS_ROM_END   0xFFFFF

// FabGL objects
fabgl::i8086 cpu;
uint8_t bios_rom[BIOS_ROM_END - BIOS_ROM_START + 1];

// Custom memory and I/O handlers
uint8_t my_mem_read(uint32_t addr);
void my_mem_write(uint32_t addr, uint8_t value);
uint8_t my_io_read(uint16_t port);
void my_io_write(uint16_t port, uint8_t value);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Serial.println("Initializing 8086 Hardware Emulator...");

  // Initialize all PCF8574s to a known state (all pins high/input)
  pcf_write(PCF_ADDR_A0_A7, 0xFF);
  pcf_write(PCF_ADDR_A8_A15, 0xFF);
  pcf_write(PCF_ADDR_A16_A19_CTRL, 0xFF);
  pcf_write(PCF_ADDR_D0_D7, 0xFF);
  pcf_write(PCF_ADDR_D8_D15, 0xFF);

  // Set custom memory and I/O handlers
  cpu.setMemReadHandler(my_mem_read);
  cpu.setMemWriteHandler(my_mem_write);
  cpu.setIOReadHandler(my_io_read);
  cpu.setIOWriteHandler(my_io_write);

  // --- Load a "BIOS" program into the internal ROM ---
  // This program will copy a message from ROM to external RAM,
  // then loop forever.
  uint16_t bios_entry_point = 0xFFF0;
  uint16_t rom_code_segment = 0xF000;
  
  // The 8086 starts at FFFF:0000, which is physical 0xFFFF0.
  // We place a JMP instruction there to our main BIOS code.
  bios_rom[0xFFF0] = 0xEA; // JMP FAR
  bios_rom[0xFFF1] = 0x00; // Offset low
  bios_rom[0xFFF2] = 0xF0; // Offset high
  bios_rom[0xFFF3] = 0x00; // Segment low
  bios_rom[0xFFF4] = 0xF0; // Segment high

  // Main BIOS code at F000:F000 (physical 0xFF000)
  uint16_t p = 0xF000;
  bios_rom[p++] = 0xBE; bios_rom[p++] = 0x30; bios_rom[p++] = 0xF0; // MOV SI, F030h (source in ROM)
  bios_rom[p++] = 0xBF; bios_rom[p++] = 0x00; bios_rom[p++] = 0x01; // MOV DI, 0100h (destination in RAM)
  bios_rom[p++] = 0xB9; bios_rom[p++] = 0x10; bios_rom[p++] = 0x00; // MOV CX, 16 (count)
  bios_rom[p++] = 0xFC; // CLD
  bios_rom[p++] = 0xF3; bios_rom[p++] = 0xA4; // REP MOVSB
  bios_rom[p++] = 0xEB; bios_rom[p++] = 0xFE; // JMP $ (Halt)

  // Data to be copied, at F000:F030
  const char* message = "Hello from ROM!";
  strncpy((char*)&bios_rom[0xF030], message, 16);

  cpu.initialize(); // Initialize CPU after setting handlers
  
  Serial.println("BIOS Loaded. Starting CPU execution.");
}

void loop() {
  // Execute instructions in blocks for better performance
  cpu.execute(100); 
}

// --- Bus Control Functions ---

void pcf_write(uint8_t addr, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t pcf_read(uint8_t addr) {
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void set_address(uint32_t addr) {
  pcf_write(PCF_ADDR_A0_A7, addr & 0xFF);
  pcf_write(PCF_ADDR_A8_A15, (addr >> 8) & 0xFF);
  uint8_t ctrl_val = pcf_read(PCF_ADDR_A16_A19_CTRL) & ~PIN_A16_A19_MASK; // preserve control signals
  pcf_write(PCF_ADDR_A16_A19_CTRL, ctrl_val | ((addr >> 16) & PIN_A16_A19_MASK));
}

void set_data(uint16_t data) {
  pcf_write(PCF_ADDR_D0_D7, data & 0xFF);
  pcf_write(PCF_ADDR_D8_D15, (data >> 8) & 0xFF);
}

uint16_t read_data() {
  // Before reading, set data bus pins to input mode by writing all 1s.
  pcf_write(PCF_ADDR_D0_D7, 0xFF);
  pcf_write(PCF_ADDR_D8_D15, 0xFF);
  
  uint8_t low_byte = pcf_read(PCF_ADDR_D0_D7);
  uint8_t high_byte = pcf_read(PCF_ADDR_D8_D15);
  return (high_byte << 8) | low_byte;
}

void set_control(uint8_t pin, bool asserted) {
  uint8_t ctrl_val = pcf_read(PCF_ADDR_A16_A19_CTRL);
  // Active low signals: asserted = true means pin goes LOW.
  if (asserted) {
    ctrl_val &= ~(1 << pin);
  } else {
    ctrl_val |= (1 << pin);
  }
  pcf_write(PCF_ADDR_A16_A19_CTRL, ctrl_val);
}

// --- CPU Hook Handlers ---

uint8_t my_mem_read(uint32_t addr) {
  if (addr >= BIOS_ROM_START && addr <= BIOS_ROM_END) {
    // Read from internal BIOS ROM
    return bios_rom[addr - BIOS_ROM_START];
  }

  if (addr >= EXT_RAM_START && addr <= EXT_RAM_END) {
    // Perform a physical read cycle on the external RAM
    Serial.printf("HW MEM READ @ 0x%05X -> ", addr);

    set_control(PIN_WR, false); // De-assert WR
    set_address(addr);
    set_control(PIN_MIO, true); // Memory operation

    set_control(PIN_ALE, true);  // Latch address
    delayMicroseconds(1);
    set_control(PIN_ALE, false);
    
    set_control(PIN_RD, true);   // Assert RD (Output Enable on SRAM)
    delayMicroseconds(1);
    
    uint8_t data = read_data() & 0xFF; // Read the lower byte of the data bus
    
    set_control(PIN_RD, false);  // De-assert RD
    Serial.printf("0x%02X\n", data);
    return data;
  }
  
  // Address is out of range
  return 0xFF;
}

void my_mem_write(uint32_t addr, uint8_t value) {
  if (addr >= BIOS_ROM_START && addr <= BIOS_ROM_END) {
    // Cannot write to ROM
    return;
  }

  if (addr >= EXT_RAM_START && addr <= EXT_RAM_END) {
    // Perform a physical write cycle to the external RAM
    Serial.printf("HW MEM WRITE @ 0x%05X <- 0x%02X\n", addr, value);
    
    set_control(PIN_RD, false); // De-assert RD
    set_address(addr);
    set_data(value);
    set_control(PIN_MIO, true); // Memory operation

    set_control(PIN_ALE, true);  // Latch address
    delayMicroseconds(1);
    set_control(PIN_ALE, false);

    set_control(PIN_WR, true);   // Assert WR (Write Enable on SRAM)
    delayMicroseconds(1);
    set_control(PIN_WR, false);  // De-assert WR
    return;
  }
}

uint8_t my_io_read(uint16_t port) {
  Serial.printf("I/O Read at 0x%04X (Not implemented)\n", port);
  // To implement, perform a bus cycle with M/IO set to false
  return 0xFF;
}

void my_io_write(uint16_t port, uint8_t value) {
  Serial.printf("I/O Write at 0x%04X, Value: 0x%02X (Not implemented)\n", port, value);
  // To implement, perform a bus cycle with M/IO set to false
}
