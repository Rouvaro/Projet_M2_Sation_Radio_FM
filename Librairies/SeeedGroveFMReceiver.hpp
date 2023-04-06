/**
 * @file grove_fm_v1_01.hpp
 * @author PAUL FILLIETTE / CLEMENT MOQUEL / GUILLAUME SANCHEZ
 * @brief 
 * @version 0.1
 * @date 2023-03-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef SEEEDGROVEFMRECEIVER_HPP
#define SEEEDGROVEFMRECEIVER_HPP

/* INCLUDES */
#include <avr/io.h>
#include <util/twi.h>
#include "atmega328p_i2c.hpp"

/* DEFINES */
#define READ	        1
#define WRITE	        0
#define BIT(x) (1 << (x))
#define RDA5807M_ADDRESS 0x10 // I2C address of RDA5807M FM module
/* DEFAULT REGISTERS ADDR FOR RDA5807M */
#define RDA5807M_REG_CHIPID 0X00
#define RDA5807M_REG_CONFIG 0x02
#define RDA5807M_REG_CHAN 0x03
#define RDA5807M_REG_EMPHASIS 0x04
#define RDA5807M_REG_GAIN_CTRL 0x05
#define RDA5807M_REG_SEEK_RESULT 0x0A
#define RDA5807M_REG_SIGNAL 0x0B
/* USEFUL RADIO STATIONS */
#define RADIO_STATION_SKYROCK 10280
#define RADIO_STATION_VIRGINRADIO 9430
#define RADIO_STATION_ARL 9620
#define RADIO_STATION_NOSTALGIE 9730
#define RADIO_STATION_CHERIEFM 9530
#define RADIO_STATION_NRJ 10240

/* USEFULL VARIABLES */
uint8_t RDA5807P_REGW[10];
uint16_t gChipID = 0;
int16_t frequency = 10110;
uint16_t vol = 1;
uint8_t bassBit = 1;// bass boost
uint8_t monoBit = 0;// force MONO not stereo
int8_t stationStep = 100;// kHz steps bewteen the stations (North America = 10)
uint8_t minSignalStrength = 36;// anything below this probably set a MONO flag for better reception
long interval = 2000;// interval for the signal level function (2 seconds)
uint8_t signalStrength;

uint8_t RDA5807N_initialization_reg[]={
  0xC4, 0x01,//change 01 to 05 enables the RDS/RBDS
  0x00, 0x00,
  0x04, 0x00,
  0xC3, 0xad,  //05h
  0x60, 0x00,
  0x42, 0x12,
  0x00, 0x00,
  0x00, 0x00,
  0x00, 0x00,  //0x0ah
  0x00, 0x00,
  0x00, 0x00,
  0x00, 0x00,
  0x00, 0x00,
  0x00, 0x00,
  0x00, 0x00,  //0x10h
  0x00, 0x19,
  0x2a, 0x11,
  0xB0, 0x42,
  0x2A, 0x11,  //
  0xb8, 0x31,  //0x15h
  0xc0, 0x00,
  0x2a, 0x91,
  0x94, 0x00,
  0x00, 0xa8,
  0xc4, 0x00,  //0x1ah
  0xF7, 0xcF,
  0x12, 0x14,  //0x1ch
  0x80, 0x6F,
  0x46, 0x08,
  0x00, 0x86,  //10000110
  0x06, 0x61,  //0x20H
  0x00, 0x00,
  0x10, 0x9E,
  0x23, 0xC8,
  0x04, 0x06,
  0x0E, 0x1C,  //0x25H     //0x04 0x08
};

/* ---------- SENSOR CLASS ---------- */
class SeeedGroveFMReceiver
{
public:
  // Constructor
  SeeedGroveFMReceiver();
  // Publics methodes for sensor management
  int initSensor();
  void setVolumeLevel(uint8_t level);
  void setFrequency(uint16_t frequency);
  void setFMSeekUp();
  void setFMSeekDown();
  uint16_t getFrequency();
  uint8_t getVolumeLevel();
  void setVolumeUp();
  void setVolumeDown();

private:
  uint8_t sensor_addr;
  uint8_t sensor_level;
  uint8_t sensor_frequency;
  bool sensor_mute_mode;
  bool sensor_stereo_mode;
  uint16_t setFreqtoChan(uint16_t frequency);
  uint8_t getSigLevel();
  unsigned char writeSensor(unsigned char *data, int nbBytes);
  unsigned char readSensor(unsigned char *data, int nbBytes);
  bool validStop(int16_t frequency);
};

#endif // SEEEDGROVEFMRECEIVER_HPP