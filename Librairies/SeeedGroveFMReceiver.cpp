/**
 * @file SeeedGroveFMReceiver.cpp
 * @author PAUL FILLIETTE / CLEMENT MOQUEL / GUILLAUME SANCHEZ
 * @brief 
 * @version 0.1
 * @date 2023-03-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* INCLUDES */
#include "SeeedGroveFMReceiver.hpp"
#include "atmega328p_i2c.hpp"

/* ---------- METHODES ---------- */
SeeedGroveFMReceiver::SeeedGroveFMReceiver()
{
    this->sensor_addr = RDA5807M_ADDRESS;
    this->sensor_level = 0x0F;
    this->sensor_mute_mode = false;
}

int SeeedGroveFMReceiver::initSensor()
{
    uint8_t error_ind = 0;
    uint8_t RDA5807P_REGR[10] = {0x0};
    uint8_t i = 0;

    RDA5807P_REGW[0] = 0x00;
    RDA5807P_REGW[0] |= monoBit<<5;
    RDA5807P_REGW[0] |= bassBit<<4;
    RDA5807P_REGW[1] = 0x02;

    error_ind = this->writeSensor((uint8_t *)&RDA5807P_REGW[0], 2); // SOFT RESET
    _delay_ms(50);

    error_ind = this->readSensor((uint8_t *)&RDA5807P_REGR[0], 10);
    _delay_ms(50);

    gChipID = RDA5807P_REGR[8];
    gChipID = ((gChipID << 8) | RDA5807P_REGR[9]);

    for (i=0;i<8;i++) 
        RDA5807P_REGW[i] = RDA5807N_initialization_reg[i];

    error_ind = this->writeSensor((uint8_t *)&RDA5807N_initialization_reg[0], 2); //POWER UP
    _delay_ms(600);

    error_ind = this->writeSensor((uint8_t *)&RDA5807N_initialization_reg[0], sizeof(RDA5807N_initialization_reg));
    _delay_ms(50);

    if(error_ind)
        return 0;
    else
        return 1;
}

unsigned char SeeedGroveFMReceiver::writeSensor(unsigned char *data, int nbBytes)
{
    // Send START condition and verifies status of I²C bus
    i2c_start();
    if ( i2c_status() != TW_REP_START && i2c_status() != TW_START ) { error(); }
    
    // Send address of RDA5807P with write bit and verifies status of I²C bus
    i2c_write(this->sensor_addr << 1 | 0);
    if ( i2c_status() != TW_MT_SLA_ACK ) {error();}    
    
    // Send data depend on number of bytes and verifies status of I²C bus
    for(int i = 0 ; i < nbBytes ; i++)
    {
        i2c_write(*data++);
        if ( i2c_status() != TW_MT_DATA_ACK ) {error();}
    }

    // Send stop condition
    i2c_stop();
    return 0;
}

unsigned char SeeedGroveFMReceiver::readSensor(unsigned char *data, int nbBytes)
{
    // Send START condition and verifies status of I²C bus
    i2c_start();
    if (i2c_status() != TW_START) error();

    // Send address of RDA5807M with write bit and verifies status of I²C bus
    i2c_write(this->sensor_addr << 1 & 0);
    if (i2c_status() != TW_MT_SLA_ACK) error();

    // Send register address
    i2c_write(*data++);
    if (i2c_status() != TW_MT_DATA_ACK) error();

    // Send repeated START condition and verifies status of I²C bus
    i2c_start();
    if (i2c_status() != TW_REP_START) error();

    // Send address of RDA5807M with read bit and verifies status of I²C bus
    i2c_write(this->sensor_addr << 1 | 1);
    if (i2c_status() != TW_MR_SLA_ACK) error();

    // Read data depend on number of bytes and verifies status of I²C bus
    for(int i = 0 ; i < nbBytes - 1 ; i++)
    {
        *data++ = i2c_read_ack();
        if (i2c_status() != TW_MR_DATA_ACK) {error();}
    }
    *data++ = i2c_read_nack();
    if (i2c_status() != TW_MR_DATA_NACK) {error();}

    // Send stop condition
    i2c_stop();
    return 0;
}

void SeeedGroveFMReceiver::setVolumeLevel(uint8_t level)
{
    if(level < 0)
    {
        level = 0;
    }
    else if(level > 15)
    {
        level = 15;
    }
    this->sensor_level = level;    
    uint8_t RDA5807P_reg_data[8];
    uint8_t i = 0;

    for ( i = 0 ; i < 8 ; i++)
    {
        RDA5807P_reg_data[i] = RDA5807P_REGW[i];
    }

    RDA5807P_reg_data[7]=(( RDA5807P_REGW[7] & 0xf0 ) | (level & 0x0f));
    RDA5807P_reg_data[3] &= (~(0x10)); //disable tune
    this->writeSensor(&(RDA5807P_reg_data[0]), 8);
}

void SeeedGroveFMReceiver::setVolumeUp()
{

    if(this->sensor_level >= 15)
    {
        this->sensor_level = 15;
    }
    else 
    {
        this->sensor_level += 1;
        uint8_t RDA5807P_reg_data[8];
        uint8_t i = 0;

        for ( i = 0 ; i < 8 ; i++)
        {
            RDA5807P_reg_data[i] = RDA5807P_REGW[i];
        }

        RDA5807P_reg_data[7]=(( RDA5807P_REGW[7] & 0xf0 ) | (this->sensor_level & 0x0f));
        RDA5807P_reg_data[3] &= (~(0x10)); //disable tune
        this->writeSensor(&(RDA5807P_reg_data[0]), 8);
    }
}

void SeeedGroveFMReceiver::setVolumeDown()
{
    if (this->sensor_level <= 0)
    {
        this->sensor_level = 0;
    }
    else
    {
        this->sensor_level -= 1;
        uint8_t RDA5807P_reg_data[8];
        uint8_t i = 0;

        for ( i = 0 ; i < 8 ; i++)
        {
            RDA5807P_reg_data[i] = RDA5807P_REGW[i];
        }

        RDA5807P_reg_data[7]=(( RDA5807P_REGW[7] & 0xf0 ) | (this->sensor_level & 0x0f));
        RDA5807P_reg_data[3] &= (~(0x10)); //disable tune
        this->writeSensor(&(RDA5807P_reg_data[0]), 8);
    }
}

void SeeedGroveFMReceiver::setFrequency(uint16_t frequency)
{
    this->sensor_frequency = frequency;
    uint16_t curChan;
    curChan = this->setFreqtoChan(frequency);

    if((frequency >= 6500)&&(frequency < 7600))
    {
        RDA5807P_REGW[3] = 0x0c;
    }
    else if((frequency >= 7600)&&(frequency < 10800))
    {
        RDA5807P_REGW[3] = 0x08; // sets the BAND bits (00xx = 87-108, 01xx=76-91, 10xx=76-108, 11xx=65-76
        // for north america this must be set to 10xx for some unknown reason
    }
    //SetNoMute
    RDA5807P_REGW[0] |= 1<<6;
    RDA5807P_REGW[0] |= 1<<5;
    RDA5807P_REGW[0] |= 0<<4;
    RDA5807P_REGW[2] = curChan >> 2;
    RDA5807P_REGW[3] = (((curChan&0x0003) << 6) | 0x10) | (RDA5807P_REGW[3] & 0x0f); //set tune bit

    this->writeSensor( &(RDA5807P_REGW[0]), 4);
}

void SeeedGroveFMReceiver::setFMSeekUp()
{
  int signalStrength;

  do {
    do 
      {
        frequency += stationStep;
        if (frequency > 10800) frequency = 8800;
        if (frequency < 8800) frequency = 10800;
      }
      while(!(this->validStop(frequency)));
      _delay_ms(50);
      signalStrength = this->getSigLevel(); // Max is 63 according to Data sheet, but I've seen more
  } 
  while (signalStrength < minSignalStrength);
}

void SeeedGroveFMReceiver::setFMSeekDown()
{
  int signalStrength;

  do {
    do 
      {
        frequency -= stationStep;
        if (frequency > 10800) frequency = 8800;
        if (frequency < 8800) frequency = 10800;
      }
      while(!(this->validStop(frequency)));
      _delay_ms(50);
      signalStrength = this->getSigLevel(); // Max is 63 according to Data sheet, but I've seen more
  } 
  while (signalStrength < minSignalStrength);
}

uint16_t SeeedGroveFMReceiver::setFreqtoChan(uint16_t frequency)
{
  uint8_t channelSpacing = 10;
  uint16_t channel = 0;

  if((frequency >= 6500)&&(frequency < 7600))
    channel = (frequency - 6500)/channelSpacing;
  else if((frequency >= 7600)&&(frequency < 10800))
    channel = (frequency - 7600)/channelSpacing;
  return (channel);
}

uint8_t SeeedGroveFMReceiver::getSigLevel()
{
  uint8_t RDA5807P_reg_data[4]={0};
  this->readSensor(&(RDA5807P_reg_data[0]), 4);
  _delay_ms(50);
  return  (RDA5807P_reg_data[2]>>1);  
}

uint16_t SeeedGroveFMReceiver::getFrequency()
{
    return this->sensor_frequency;
}

uint8_t SeeedGroveFMReceiver::getVolumeLevel()
{
    return this->sensor_level;
}

bool SeeedGroveFMReceiver::validStop(int16_t frequency)
{
  uint8_t RDA5807P_reg_data[4]={0};
  uint8_t falseStation = 0;
  uint8_t i = 0;
  uint16_t curChan;

  if((frequency >= 6500)&&(frequency < 7600))
  {
    RDA5807P_REGW[3] = 0x0c;
  }
  else if((frequency >= 7600)&&(frequency < 10800))
  {
    RDA5807P_REGW[3] = 0x08;// Sets the BAND bits (00xx = 87-108, 01xx=76-91, 10xx=76-108, 11xx=65-76
    // For north america this must be set to 10xx for some unknown reason
  }
  curChan = this->setFreqtoChan(frequency);
  //SetNoMute bit 9 is seek direction (0=seek down, 1=seek up).
  //02H 14
  RDA5807P_REGW[0] |=	1<<6;// Reg zero is bits 15 to bit 8 (this shifts to bit 14)
  RDA5807P_REGW[0] |= monoBit<<5;
  RDA5807P_REGW[0] |= bassBit<<4;
  // HandleBits();
  RDA5807P_reg_data[0] = RDA5807P_REGW[0];
  RDA5807P_reg_data[1] = RDA5807P_REGW[1];
  RDA5807P_reg_data[2] = curChan>>2;//03H 15:8 CHAN
  RDA5807P_reg_data[3] = (((curChan&0x0003) << 6) | 0x10) | (RDA5807P_REGW[3] &0x0f);
  this->writeSensor(&(RDA5807P_reg_data[0]), 4);

  _delay_ms(50); // Dealy 25 ms

  /*
  if (0x5808 == gChipID)
    this->readSensor(&(RDA5807P_reg_data[0]), 4);
  else
  {
    do
    {
      i++;
      if(i > 5) 
        return 0;

      _delay_ms(30);
      //read REG0A&0B
      this->readSensor(&(RDA5807P_reg_data[0]), 4);
    }
    while((RDA5807P_reg_data[0]&0x40)==0);
  }
  */

  // Check FM_TRUE
  if( (RDA5807P_reg_data[2] & 0x01) == 0) 
    falseStation=1; //0B 8  FM TRUE

  if(frequency == 9600) 
    falseStation=1;// North America - if scanning DOWN, the radio will lock on 9600 for some reason!
  _delay_ms(50);
  if (falseStation == 1)
    return 0;
  else
    return 1;
} 