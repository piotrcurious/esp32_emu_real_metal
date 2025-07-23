#include <Arduino.h>
#include <Wire.h>
#include <fabgl.h>

// PCF8574 I2C addresses
#define PCF_ADDR_A0_A7   0x20
#define PCF_ADDR_A8_A15  0x21
#define PCF_ADDR_A16_A19_CTRL 0x22
#define PCF_ADDR_D0_D7   0x23
#define PCF_ADDR_D8_D15  0x24

// PCF8574 Control Signal Pins (on expander at 0x22)
#define PIN_RD   4
#define PIN_WR   5
#define PIN_MIO  6
#define PIN_ALE  7

// FabGL objects
fabgl::VGAController displayController;
fabgl::PS2Controller keyboardController;
fabgl::i8086 cpu;

// Custom memory and I/O handlers
uint8_t my_mem_read(uint32_t addr);
void my_mem_write(uint32_t addr, uint8_t value);
uint8_t my_io_read(uint16_t port);
void my_io_write(uint16_t port, uint8_t value);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize FabGL
  displayController.begin();
  keyboardController.begin();
  cpu.initialize();

  // Set custom memory and I/O handlers
  cpu.setMemReadHandler(my_mem_read);
  cpu.setMemWriteHandler(my_mem_write);
  cpu.setIOReadHandler(my_io_read);
  cpu.setIOWriteHandler(my_io_write);

  // Load a simple program into the emulated CPU's memory
  // This program will write a value to a memory location
  // and then halt.
  // MOV AL, 0x42
  // MOV [0x1000], AL
  // HLT
  cpu.writeMem(0xF000, 0, 0xB0); // MOV AL
  cpu.writeMem(0xF000, 1, 0x42); // 0x42
  cpu.writeMem(0xF000, 2, 0xA2); // MOV [addr], AL
  cpu.writeMem(0xF000, 3, 0x00); // addr low
  cpu.writeMem(0xF000, 4, 0x10); // addr high
  cpu.writeMem(0xF000, 5, 0xF4); // HLT
  
  cpu.reset();
  // Set the instruction pointer to the start of our program
  cpu.getregs()->IP = 0;
  cpu.getregs()->CS = 0xF000;
  
  Serial.println("8086 Emulator Initialized");
}

void loop() {
  // Execute one instruction
  cpu.execute(1);
  delay(100); // Slow down execution for observation
}

// Write a byte to a PCF8574
void pcf_write(uint8_t addr, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(data);
  Wire.endTransmission();
}

// Read a byte from a PCF8574
uint8_t pcf_read(uint8_t addr) {
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// Set the address bus
void set_address(uint32_t addr) {
  pcf_write(PCF_ADDR_A0_A7, addr & 0xFF);
  pcf_write(PCF_ADDR_A8_A15, (addr >> 8) & 0xFF);
  uint8_t ctrl_val = pcf_read(PCF_ADDR_A16_A19_CTRL) & 0xF0; // preserve control signals
  pcf_write(PCF_ADDR_A16_A19_CTRL, ctrl_val | ((addr >> 16) & 0x0F));
}

// Set the data bus
void set_data(uint16_t data) {
  pcf_write(PCF_ADDR_D0_D7, data & 0xFF);
  pcf_write(PCF_ADDR_D8_D15, (data >> 8) & 0xFF);
}

// Read the data bus
uint16_t read_data() {
  uint8_t low_byte = pcf_read(PCF_ADDR_D0_D7);
  uint8_t high_byte = pcf_read(PCF_ADDR_D8_D15);
  return (high_byte << 8) | low_byte;
}

// Set control signals
void set_control(uint8_t pin, bool state) {
  uint8_t ctrl_val = pcf_read(PCF_ADDR_A16_A19_CTRL);
  if (state) {
    ctrl_val |= (1 << pin);
  } else {
    ctrl_val &= ~(1 << pin);
  }
  pcf_write(PCF_ADDR_A16_A19_CTRL, ctrl_val);
}

// Memory read handler
uint8_t my_mem_read(uint32_t addr) {
  Serial.printf("Memory Read at 0x%05X\n", addr);
  set_address(addr);
  set_control(PIN_MIO, true); // Memory operation
  set_control(PIN_RD, true);
  set_control(PIN_ALE, true);
  delay(1);
  set_control(PIN_ALE, false);
  delay(1);
  uint8_t data = read_data() & 0xFF; // For simplicity, we read a byte
  set_control(PIN_RD, false);
  return data;
}

// Memory write handler
void my_mem_write(uint32_t addr, uint8_t value) {
  Serial.printf("Memory Write at 0x%05X, Value: 0x%02X\n", addr, value);
  set_address(addr);
  set_data(value);
  set_control(PIN_MIO, true); // Memory operation
  set_control(PIN_WR, true);
  set_control(PIN_ALE, true);
  delay(1);
  set_control(PIN_ALE, false);
  delay(1);
  set_control(PIN_WR, false);
}

// I/O read handler
uint8_t my_io_read(uint16_t port) {
  Serial.printf("I/O Read at 0x%04X\n", port);
  set_address(port);
  set_control(PIN_MIO, false); // I/O operation
  set_control(PIN_RD, true);
  set_control(PIN_ALE, true);
  delay(1);
  set_control(PIN_ALE, false);
  delay(1);
  uint8_t data = read_data() & 0xFF;
  set_control(PIN_RD, false);
  return data;
}

// I/O write handler
void my_io_write(uint16_t port, uint8_t value) {
  Serial.printf("I/O Write at 0x%04X, Value: 0x%02X\n", port, value);
  set_address(port);
  set_data(value);
  set_control(PIN_MIO, false); // I/O operation
  set_control(PIN_WR, true);
  set_control(PIN_ALE, true);
  delay(1);
  set_control(PIN_ALE, false);
  delay(1);
  set_control(PIN_WR, false);
}
