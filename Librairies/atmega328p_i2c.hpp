/**
 * @file config_atmega328p.hpp
 * @author Guillaume SANCHEZ / Clément MOQUEL / Paul FILLIETTE
 * @brief 
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef CONFIG_ATMEGA328P_HPP
#define CONFIG_ATMEGA328P_HPP

#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL
#define F_I2C 400000UL

void i2c_init();
void i2c_start();
void i2c_stop();
void i2c_write(uint8_t data);
uint8_t i2c_read_ack();
uint8_t i2c_read_nack();
uint8_t i2c_status();
int error(void);


void i2c_init() {
  // Set TWBR register to adjust I2C clock frequency
  TWBR = ((F_CPU/F_I2C)-16)/2;
  TWSR = 0x00;
  TWCR = (1 << TWEN);
}

void i2c_start() {
  // Send start condition
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

  // Wait for start condition to be transmitted
  while (!(TWCR & (1 << TWINT)));
}

void i2c_stop() {
  // Send stop condition
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void i2c_write(uint8_t data) {
  // Load data into TWDR register
  TWDR = data;

  // Transmit data
  TWCR = (1 << TWINT) | (1 << TWEN);

  // Wait for data to be transmitted
  while (!(TWCR & (1 << TWINT)));
}

uint8_t i2c_read_ack() 
{
  // Read data with ACK
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  // Wait for data to be received
  while (!(TWCR & (1 << TWINT)));
  // Return received data
  return TWDR;
}

uint8_t i2c_read_nack() 
{
  // Read data with NACK
  TWCR = (1 << TWINT) | (1 << TWEN);
  // Wait for data to be received
  while (!(TWCR & (1 << TWINT)));
  // Return received data
  return TWDR;
}

uint8_t i2c_status(void)
{
	/* retourne la valeur codée par les bits de status */
	/* il ne faut pas l'ajuster à droite attention !!  */
	return TWSR & 0xF8;
}

int error(void)
{
  return 0;
}
#endif // CONFIG_ATMEGA328P_HPP