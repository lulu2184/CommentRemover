
#include "Marlin.h"

#ifdef SDSUPPORT
#include "Sd2Card.h"

#ifndef SOFTWARE_SPI



#if (SPR0 != 0 || SPR1 != 1)
#error unexpected SPCR bits
#endif

static void spiInit(uint8_t spiRate) {
  
  SPCR = (1 << SPE) | (1 << MSTR) | (spiRate >> 1);
  SPSR = spiRate & 1 || spiRate == 6 ? 0 : 1 << SPI2X;
}


static uint8_t spiRec() {
  SPDR = 0XFF;
  while (!(SPSR & (1 << SPIF))) {  }
  return SPDR;
}


static inline __attribute__((always_inline))
void spiRead(uint8_t* buf, uint16_t nbyte) {
  if (nbyte-- == 0) return;
  SPDR = 0XFF;
  for (uint16_t i = 0; i < nbyte; i++) {
    while (!(SPSR & (1 << SPIF))) {  }
    buf[i] = SPDR;
    SPDR = 0XFF;
  }
  while (!(SPSR & (1 << SPIF))) {  }
  buf[nbyte] = SPDR;
}


static void spiSend(uint8_t b) {
  SPDR = b;
  while (!(SPSR & (1 << SPIF))) {  }
}


static inline __attribute__((always_inline))
  void spiSendBlock(uint8_t token, const uint8_t* buf) {
  SPDR = token;
  for (uint16_t i = 0; i < 512; i += 2) {
    while (!(SPSR & (1 << SPIF))) {  }
    SPDR = buf[i];
    while (!(SPSR & (1 << SPIF))) {  }
    SPDR = buf[i + 1];
  }
  while (!(SPSR & (1 << SPIF))) {  }
}

#else  


#define nop asm volatile ("nop\n\t")


static uint8_t spiRec() {
  uint8_t data = 0;
  
  cli();
  
  fastDigitalWrite(SPI_MOSI_PIN, HIGH);

  for (uint8_t i = 0; i < 8; i++) {
    fastDigitalWrite(SPI_SCK_PIN, HIGH);

    
    nop;
    nop;

    data <<= 1;

    if (fastDigitalRead(SPI_MISO_PIN)) data |= 1;

    fastDigitalWrite(SPI_SCK_PIN, LOW);
  }
  
  sei();
  return data;
}


static void spiRead(uint8_t* buf, uint16_t nbyte) {
  for (uint16_t i = 0; i < nbyte; i++) {
    buf[i] = spiRec();
  }
}


static void spiSend(uint8_t data) {
  
  cli();
  for (uint8_t i = 0; i < 8; i++) {
    fastDigitalWrite(SPI_SCK_PIN, LOW);

    fastDigitalWrite(SPI_MOSI_PIN, data & 0X80);

    data <<= 1;

    fastDigitalWrite(SPI_SCK_PIN, HIGH);
  }
  
  nop;
  nop;
  nop;
  nop;

  fastDigitalWrite(SPI_SCK_PIN, LOW);
  
  sei();
}


  void spiSendBlock(uint8_t token, const uint8_t* buf) {
  spiSend(token);
  for (uint16_t i = 0; i < 512; i++) {
    spiSend(buf[i]);
  }
}
#endif  


uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg) {
  
  chipSelectLow();

  
  waitNotBusy(300);

  
  spiSend(cmd | 0x40);

  
  for (int8_t s = 24; s >= 0; s -= 8) spiSend(arg >> s);

  
  uint8_t crc = 0XFF;
  if (cmd == CMD0) crc = 0X95;  
  if (cmd == CMD8) crc = 0X87;  
  spiSend(crc);

  
  if (cmd == CMD12) spiRec();

  
  for (uint8_t i = 0; ((status_ = spiRec()) & 0X80) && i != 0XFF; i++) {  }
  return status_;
}


uint32_t Sd2Card::cardSize() {
  csd_t csd;
  if (!readCSD(&csd)) return 0;
  if (csd.v1.csd_ver == 0) {
    uint8_t read_bl_len = csd.v1.read_bl_len;
    uint16_t c_size = (csd.v1.c_size_high << 10)
                      | (csd.v1.c_size_mid << 2) | csd.v1.c_size_low;
    uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1)
                          | csd.v1.c_size_mult_low;
    return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
  } else if (csd.v2.csd_ver == 1) {
    uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16)
                      | (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
    return (c_size + 1) << 10;
  } else {
    error(SD_CARD_ERROR_BAD_CSD);
    return 0;
  }
}

void Sd2Card::chipSelectHigh() {
  digitalWrite(chipSelectPin_, HIGH);
}

void Sd2Card::chipSelectLow() {
#ifndef SOFTWARE_SPI
  spiInit(spiRate_);
#endif  
  digitalWrite(chipSelectPin_, LOW);
}


bool Sd2Card::erase(uint32_t firstBlock, uint32_t lastBlock) {
  csd_t csd;
  if (!readCSD(&csd)) goto fail;
  
  if (!csd.v1.erase_blk_en) {
    
    uint8_t m = (csd.v1.sector_size_high << 1) | csd.v1.sector_size_low;
    if ((firstBlock & m) != 0 || ((lastBlock + 1) & m) != 0) {
      
      error(SD_CARD_ERROR_ERASE_SINGLE_BLOCK);
      goto fail;
    }
  }
  if (type_ != SD_CARD_TYPE_SDHC) {
    firstBlock <<= 9;
    lastBlock <<= 9;
  }
  if (cardCommand(CMD32, firstBlock)
    || cardCommand(CMD33, lastBlock)
    || cardCommand(CMD38, 0)) {
      error(SD_CARD_ERROR_ERASE);
      goto fail;
  }
  if (!waitNotBusy(SD_ERASE_TIMEOUT)) {
    error(SD_CARD_ERROR_ERASE_TIMEOUT);
    goto fail;
  }
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}


bool Sd2Card::eraseSingleBlockEnable() {
  csd_t csd;
  return readCSD(&csd) ? csd.v1.erase_blk_en : false;
}


bool Sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin) {
  errorCode_ = type_ = 0;
  chipSelectPin_ = chipSelectPin;
  
  uint16_t t0 = (uint16_t)millis();
  uint32_t arg;

  
  pinMode(chipSelectPin_, OUTPUT);
  chipSelectHigh();
  pinMode(SPI_MISO_PIN, INPUT);
  pinMode(SPI_MOSI_PIN, OUTPUT);
  pinMode(SPI_SCK_PIN, OUTPUT);

#ifndef SOFTWARE_SPI
  
  pinMode(SS_PIN, OUTPUT);
  
#if SET_SPI_SS_HIGH
  digitalWrite(SS_PIN, HIGH);
#endif  
  
  spiRate_ = SPI_SD_INIT_RATE;
  spiInit(spiRate_);
#endif  

  
  for (uint8_t i = 0; i < 10; i++) spiSend(0XFF);

  
  while ((status_ = cardCommand(CMD0, 0)) != R1_IDLE_STATE) {
    if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT) {
      error(SD_CARD_ERROR_CMD0);
      goto fail;
    }
  }
  
  if ((cardCommand(CMD8, 0x1AA) & R1_ILLEGAL_COMMAND)) {
    type(SD_CARD_TYPE_SD1);
  } else {
    
    for (uint8_t i = 0; i < 4; i++) status_ = spiRec();
    if (status_ != 0XAA) {
      error(SD_CARD_ERROR_CMD8);
      goto fail;
    }
    type(SD_CARD_TYPE_SD2);
  }
  
  arg = type() == SD_CARD_TYPE_SD2 ? 0X40000000 : 0;

  while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE) {
    
    if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT) {
      error(SD_CARD_ERROR_ACMD41);
      goto fail;
    }
  }
  
  if (type() == SD_CARD_TYPE_SD2) {
    if (cardCommand(CMD58, 0)) {
      error(SD_CARD_ERROR_CMD58);
      goto fail;
    }
    if ((spiRec() & 0XC0) == 0XC0) type(SD_CARD_TYPE_SDHC);
    
    for (uint8_t i = 0; i < 3; i++) spiRec();
  }
  chipSelectHigh();

#ifndef SOFTWARE_SPI
  return setSckRate(sckRateID);
#else  
  return true;
#endif  

 fail:
  chipSelectHigh();
  return false;
}


bool Sd2Card::readBlock(uint32_t blockNumber, uint8_t* dst) {
  
  if (type()!= SD_CARD_TYPE_SDHC) blockNumber <<= 9;
  if (cardCommand(CMD17, blockNumber)) {
    error(SD_CARD_ERROR_CMD17);
    goto fail;
  }
  return readData(dst, 512);

 fail:
  chipSelectHigh();
  return false;
}


bool Sd2Card::readData(uint8_t *dst) {
  chipSelectLow();
  return readData(dst, 512);
}

bool Sd2Card::readData(uint8_t* dst, uint16_t count) {
  
  uint16_t t0 = millis();
  while ((status_ = spiRec()) == 0XFF) {
    if (((uint16_t)millis() - t0) > SD_READ_TIMEOUT) {
      error(SD_CARD_ERROR_READ_TIMEOUT);
      goto fail;
    }
  }
  if (status_ != DATA_START_BLOCK) {
    error(SD_CARD_ERROR_READ);
    goto fail;
  }
  
  spiRead(dst, count);

  
  spiRec();
  spiRec();
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}


bool Sd2Card::readRegister(uint8_t cmd, void* buf) {
  uint8_t* dst = reinterpret_cast<uint8_t*>(buf);
  if (cardCommand(cmd, 0)) {
    error(SD_CARD_ERROR_READ_REG);
    goto fail;
  }
  return readData(dst, 16);

 fail:
  chipSelectHigh();
  return false;
}


bool Sd2Card::readStart(uint32_t blockNumber) {
  if (type()!= SD_CARD_TYPE_SDHC) blockNumber <<= 9;
  if (cardCommand(CMD18, blockNumber)) {
    error(SD_CARD_ERROR_CMD18);
    goto fail;
  }
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}


bool Sd2Card::readStop() {
  chipSelectLow();
  if (cardCommand(CMD12, 0)) {
    error(SD_CARD_ERROR_CMD12);
    goto fail;
  }
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}


bool Sd2Card::setSckRate(uint8_t sckRateID) {
  if (sckRateID > 6) {
    error(SD_CARD_ERROR_SCK_RATE);
    return false;
  }
  spiRate_ = sckRateID;
  return true;
}


bool Sd2Card::waitNotBusy(uint16_t timeoutMillis) {
  uint16_t t0 = millis();
  while (spiRec() != 0XFF) {
    if (((uint16_t)millis() - t0) >= timeoutMillis) goto fail;
  }
  return true;

 fail:
  return false;
}


bool Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t* src) {
  
  if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
  if (cardCommand(CMD24, blockNumber)) {
    error(SD_CARD_ERROR_CMD24);
    goto fail;
  }
  if (!writeData(DATA_START_BLOCK, src)) goto fail;

  
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
    error(SD_CARD_ERROR_WRITE_TIMEOUT);
    goto fail;
  }
  
  if (cardCommand(CMD13, 0) || spiRec()) {
    error(SD_CARD_ERROR_WRITE_PROGRAMMING);
    goto fail;
  }
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}


bool Sd2Card::writeData(const uint8_t* src) {
  chipSelectLow();
  
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) goto fail;
  if (!writeData(WRITE_MULTIPLE_TOKEN, src)) goto fail;
  chipSelectHigh();
  return true;

 fail:
  error(SD_CARD_ERROR_WRITE_MULTIPLE);
  chipSelectHigh();
  return false;
}


bool Sd2Card::writeData(uint8_t token, const uint8_t* src) {
  spiSendBlock(token, src);

  spiSend(0xff);  
  spiSend(0xff);  

  status_ = spiRec();
  if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
    error(SD_CARD_ERROR_WRITE);
    goto fail;
  }
  return true;

 fail:
  chipSelectHigh();
  return false;
}


bool Sd2Card::writeStart(uint32_t blockNumber, uint32_t eraseCount) {
  
  if (cardAcmd(ACMD23, eraseCount)) {
    error(SD_CARD_ERROR_ACMD23);
    goto fail;
  }
  
  if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
  if (cardCommand(CMD25, blockNumber)) {
    error(SD_CARD_ERROR_CMD25);
    goto fail;
  }
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}


bool Sd2Card::writeStop() {
  chipSelectLow();
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) goto fail;
  spiSend(STOP_TRAN_TOKEN);
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) goto fail;
  chipSelectHigh();
  return true;

 fail:
  error(SD_CARD_ERROR_STOP_TRAN);
  chipSelectHigh();
  return false;
}

#endif
