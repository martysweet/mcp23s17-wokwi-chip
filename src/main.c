#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

/*
  MCP23S17 MVP Driver for Wokwi
  Marty Sweet - 2024

  This is a minimal implementation of the MCP23S17 SPI GPIO Expander
  It supports the following features:
   - SPI Communication
   - GPIO Input/Output
   - Pull-up resistors
   - Input Polarity

  Source and releases: https://github.com/martysweet/mcp23s17-wokwi-chip
*/

typedef struct {
  pin_t    cs_pin;
  pin_t    dc_pin;
  pin_t    rst_pin;
  spi_dev_t spi;
  uint8_t  spi_buffer[1];

  /* State Machine for MODE, REGISTER */
  uint8_t cmd; 
  uint8_t cmd_reg; // 0x00 - 0x15


  /* Registers */
  uint8_t registers[22];

  /* GPIO Pins */
  pin_t    gpio_pins_a[8];
  pin_t    gpio_pins_b[8];

} chip_state_t;

/** 
 * Register Offsets 
 * https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP23017-Data-Sheet-DS20001952.pdf
 * For now, this only supports BANK=0 configuration (set via IOCON.BANK bit)
 * (Page included in the register comment)
 */
#define REG_IODIRA  (0x00) // Direction register (Input/Output) P18
#define REG_IODIRB  (0x01) // Direction register (Input/Output) P18
#define REG_IPOLA   (0x02) // Input Polarity A P18
#define REG_IPOLB   (0x03) // Input Polarity B P18
#define REG_GPINTENA (0x04) // Not implemented
#define REG_GPINTENB (0x05) // Not implemented
#define REG_DEFVALA (0x06) // Not implemented
#define REG_DEFVALB (0x07) // Not implemented
#define REG_INTCONA (0x08) // Not implemented
#define REG_INTCONB (0x09) // Not implemented
#define REG_IOCONA  (0x0a) // SeqOp Supported Only P20
#define REG_IOCONB  (0x0b) // Not implemented
#define REG_GPPUA   (0x0c) // Pull-up resistor configuration P22
#define REG_GPPUB   (0x0d) // Pull-up resistor configuration P22
#define REG_INTFA   (0x0e) // Not implemented
#define REG_INTFB   (0x0f) // Not implemented
#define REG_INTCAPA (0x10) // Not implemented
#define REG_INTCAPB (0x11) // Not implemented
#define REG_GPIOA   (0x12) // Read/Write P23
#define REG_GPIOB   (0x13) // Read/Write P23
#define REG_OLATA   (0x14) // Stores the desired output state of the pins (write-through from REG_GPIOA)
#define REG_OLATB   (0x15) // Stores the desired output state of the pins (write-through from REG_GPIOB)
#define REG_EMPTY   (0xFF) // Used for SPI state machine

// Command Codes used in SPI communication
#define CMD_NOP     (0x00)
#define CMD_WRITE   (0x40)
#define CMD_READ    (0x41)


static void chip_pin_change(void *user_data, pin_t pin, uint32_t value);
static void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count);
void process_command(chip_state_t *chip, uint8_t *buffer, uint32_t buffer_size);
uint8_t readRegister(chip_state_t *chip, uint8_t address);
void setup_gpio_bank(chip_state_t *chip, uint8_t bank);
void writeRegister(chip_state_t *chip, uint8_t address, uint8_t data);

void chip_reset(chip_state_t *chip) {
  // Reset the registers
  for (int i = 0; i < sizeof(chip->registers); i++) {
    chip->registers[i] = 0;
  }

  // Set Default Registers
  chip->registers[REG_IODIRA] = 0xFF; // All inputs
  chip->registers[REG_IODIRB] = 0xFF; // All inputs

  // Reset the GPIO pins
  setup_gpio_bank(chip, 0);
  setup_gpio_bank(chip, 1);

  // Reset the command state machine
  chip->cmd = CMD_NOP;
  chip->cmd_reg = REG_EMPTY;
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));

  const pin_watch_config_t watch_config = {
    .edge = BOTH,
    .pin_change = chip_pin_change,
    .user_data = chip,
  };
  
  chip->cs_pin = pin_init("CS", INPUT_PULLUP);
  pin_watch(chip->cs_pin, &watch_config);

  chip->rst_pin = pin_init("RST", INPUT_PULLUP);
  pin_watch(chip->rst_pin, &watch_config);

  const spi_config_t spi_config = {
    .sck = pin_init("SCK", INPUT),
    .mosi = pin_init("MOSI", INPUT),
    .miso = pin_init("MISO", OUTPUT),
    .done = chip_spi_done,
    .user_data = chip,
  };
  chip->spi = spi_init(&spi_config);

  // Init GPIO
  char pinName[3];
  for(int i = 0; i < 8; i++){
    sprintf(pinName, "A%d", i);
    chip->gpio_pins_a[i] = pin_init(pinName, INPUT);
  }

  for(int i = 0; i < 8; i++){
    sprintf(pinName, "B%d", i);
    chip->gpio_pins_b[i] = pin_init(pinName, INPUT);
  }

  // Clear state
  chip_reset(chip);
  printf("MCP23S17 Driver Chip Initialized!\n");
}

void chip_pin_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = (chip_state_t*)user_data;

  // Handle CS pin logic
  if (pin == chip->cs_pin) {
    if (value == LOW) {
      spi_start(chip->spi, chip->spi_buffer, sizeof(chip->spi_buffer));
    } else {
      spi_stop(chip->spi);
    }
  }

  // Handle RST pin
  if (pin == chip->rst_pin && value == LOW) {
    spi_stop(chip->spi); // Process remaining data in SPI buffer
    printf("Resetting MCP23S17\n");
    chip_reset(chip);
  }
}

void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  chip_state_t *chip = (chip_state_t*)user_data;
  if (!count) {
    // This means that we got here from spi_stop, and no data was received
    return;
  }

  // The device supports writing or reading registers
  process_command(chip, buffer, count);

  // If CS is still low, it means there's more data to receive
  if (pin_read(chip->cs_pin) == LOW) {
    spi_start(chip->spi, chip->spi_buffer, sizeof(chip->spi_buffer));
  }
}

// Read [opcode] [address] [0xFF - write back clock] 
// Write [opcode] [address] [data]
void process_command(chip_state_t *chip, uint8_t *buffer, uint32_t buffer_size) {
  // If the buffer is empty, we have nothing to do
  if (buffer_size == 0) {
    printf("Invalid buffer size, got data of size 0\n");
    return;
  }

  // Check the current state machine
  if(chip->cmd == CMD_NOP){
    // We are expecting a first byte with the opcode
    int valid_opcode = (buffer[0] ^ CMD_READ) == 0 || (buffer[0] ^ CMD_WRITE) == 0;
    if(valid_opcode){
      chip->cmd = buffer[0];
    } else if(buffer[0] != 0xFF) {  // 0xFF is sent to read out a byte from the chip
      printf("Invalid opcode: %x\n", buffer[0]);
    }

    // Return early, we need to wait for the second byte
    return;
  }

  if(chip->cmd_reg == REG_EMPTY){
    // We are expecting a second byte with the address
    chip->cmd_reg = buffer[0];
    
    // If this is a read command, we can process it now, and set buffer[0] to the response
    if((chip->cmd ^ CMD_READ) == 0){
      uint8_t data = readRegister(chip, chip->cmd_reg);
      printf("Read from address: %x, data: %x\n", chip->cmd_reg, data);
      buffer[0] = data;
      chip->cmd = CMD_NOP;
      chip->cmd_reg = REG_EMPTY;
      return;
    }

    // If this is a write command, we need to wait for the third byte
    return;
  }

  // If we get here, we are expecting a third byte with the data
  if(chip->cmd_reg != REG_EMPTY){
    writeRegister(chip, chip->cmd_reg, buffer[0]);
    printf("Write to address: %x, data: %x\n", chip->cmd_reg, buffer[0]);
    chip->cmd = CMD_NOP;
    chip->cmd_reg = REG_EMPTY;
    return;
  }
}

uint8_t readRegister(chip_state_t *chip, uint8_t address){
  
  // Default read operation
  uint8_t data = chip->registers[address];
  
  // Handle a read of a GPIO register
  // Consider pin version of input pin types only
  if(address == REG_GPIOA || address == REG_GPIOB){
    uint8_t iodir = (address == REG_GPIOA) ? REG_IODIRA : REG_IODIRB;
    uint8_t olat = (address == REG_GPIOA) ? REG_OLATA : REG_OLATB;
    uint8_t ipol = (address == REG_GPIOA) ? REG_IPOLA : REG_IPOLB;
    pin_t *gpio_pins = (address == REG_GPIOA) ? chip->gpio_pins_a : chip->gpio_pins_b;

    // Read the state of the pins
    // Doing a read here in real life may be more complicated due to latency considerations
    // But this should work in simulation. Another approach would be using pin_watch and making it more async.
    data = 0x00;
    for(int i = 0; i < 8; i++){
      uint32_t pin_state = pin_read(gpio_pins[i]);
      uint8_t is_input_pin = (chip->registers[iodir] >> i) & 0x01;
      uint8_t is_inverted = (chip->registers[ipol] >> i) & 0x01;

      if(is_input_pin && is_inverted){
        pin_state = !pin_state;
      }

      data |= (pin_state << i);    
    }
  }

  return data;
}

void writeRegister(chip_state_t *chip, uint8_t address, uint8_t data){

  // Writing to GPIOA or GPIOB
  // Allows setting the output state of the pins
  if(address == REG_GPIOA || address == REG_GPIOB){
    uint8_t iodir = (address == REG_GPIOA) ? REG_IODIRA : REG_IODIRB;
    uint8_t olat = (address == REG_GPIOA) ? REG_OLATA : REG_OLATB;
    pin_t *gpio_pins = (address == REG_GPIOA) ? chip->gpio_pins_a : chip->gpio_pins_b;

    // Write the state of the pins
    for(int i = 0; i < 8; i++){
      uint8_t is_input_pin = (chip->registers[iodir] >> i) & 0x01;
      uint8_t output_state = (data >> i) & 0x01;

      if(is_input_pin){
        // Do nothing
      } else {
        pin_write(gpio_pins[i], output_state);
      }
    }

    // Also commit to the OLAT register (for read-back)
    chip->registers[address] = data;
    chip->registers[olat] = data;
    return;
  }

  // Set the IO Direction of a Pin
  // Set Pull-up resistor configuration
  // Triggers a GPIO setup handler
  if(address == REG_IODIRA || address == REG_IODIRB || address == REG_GPPUA || address == REG_GPPUB){
    chip->registers[address] = data;
    setup_gpio_bank(chip, 0);
    setup_gpio_bank(chip, 1);
    return;
  }


  // Default write operation
  chip->registers[address] = data;
}

void setup_gpio_bank(chip_state_t *chip, uint8_t bank){
  // Setup GPIO Pins based on IODIR and GPPU
  uint8_t iodir = (bank == 0) ? REG_IODIRA : REG_IODIRB;
  uint8_t gppu = (bank == 0) ? REG_GPPUA : REG_GPPUB;
  pin_t *gpio_pins = (bank == 0) ? chip->gpio_pins_a : chip->gpio_pins_b;

  for(int i = 0; i < 8; i++){
    uint8_t is_input_pin = (chip->registers[iodir] >> i) & 0x01;
    uint8_t pullup_enabled = (chip->registers[gppu] >> i) & 0x01;

    uint8_t desired_mode = (is_input_pin) ? INPUT : OUTPUT;
    if(is_input_pin && pullup_enabled){
      desired_mode = INPUT_PULLUP;
    }

    printf("Setting pin %d to mode %d\n", i, desired_mode);

    // Set the pin mode
    pin_mode(gpio_pins[i], desired_mode);
  }
}