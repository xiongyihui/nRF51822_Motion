// W25Q16BV.cpp

#include"W25Q16BV.h"

// CONSTRUCTOR 
W25Q16BV::W25Q16BV(PinName mosi, PinName miso, PinName sclk, PinName cs) : _spi(mosi, miso, sclk), _cs(cs) {
    _spi.format(SPI_NBIT, SPI_MODE);
    _spi.frequency(SPI_FREQ);
    chipDisable();

    exitDeepPowerDown();
    // The SPI Flash cannot safely be accessed the first 1-10 ms after power ON
//    wait_us(WAIT_US_TPUW);
}


// READING
int W25Q16BV::readByte(int addr) {
    chipEnable();
    _spi.write(R_INST);
    _spi.write((addr >> 16) & 0xff);
    _spi.write((addr >>  8) & 0xff);
    _spi.write((addr      ) & 0xff);
    int response = _spi.write(DUMMY_ADDR);
    chipDisable();
    return response;
}
int W25Q16BV::readByte(int a2, int a1, int a0) {
   chipEnable();
   _spi.write(R_INST);
   _spi.write(a2);
   _spi.write(a1);
   _spi.write(a0);
   int response = _spi.write(DUMMY_ADDR);
    chipDisable();
    return response;
}
void W25Q16BV::readStream(int addr, char* buf, int count) {
    if (count < 1)
        return;
    chipEnable();
    _spi.write(R_INST);
    _spi.write((addr >> 16) & 0xff);
    _spi.write((addr >>  8) & 0xff);
    _spi.write((addr      ) & 0xff);
    for (int i = 0; i < count; i++)
        buf[i] =  _spi.write(DUMMY_ADDR);
    chipDisable();
}
void W25Q16BV::readJEDEC(uint8_t* manId, uint8_t* memType, uint8_t* cap)
{
    chipEnable();
    _spi.write(JDEC_INST);
    *manId = _spi.write(DUMMY_ADDR);
    *memType = _spi.write(DUMMY_ADDR);
    *cap = _spi.write(DUMMY_ADDR);
    chipDisable();
}
uint8_t W25Q16BV::readStatus1()
{
  uint8_t status;
  chipEnable();
  _spi.write(STATUS1_INST);
  status = _spi.write(DUMMY_ADDR);
  chipDisable();
  return status;
}
uint8_t W25Q16BV::readStatus2()
{
  uint8_t status;
  chipEnable();
  _spi.write(STATUS2_INST);
  status = _spi.write(DUMMY_ADDR);
  chipDisable();
  return status;
}

// WRITING
void W25Q16BV::writeByte(int addr, int data) {
    writeEnable();
    chipEnable();
    _spi.write(W_INST);
    _spi.write((addr >> 16) & 0xff);
    _spi.write((addr >>  8) & 0xff);
    _spi.write((addr      ) & 0xff);
    _spi.write(data);
    chipDisable();
    writeDisable();
//    wait_us(WAIT_US_TBP);
    waitWhileBusy();
}
void W25Q16BV::writeByte(int a2, int a1, int a0, int data) {
    writeEnable();
    chipEnable();
    _spi.write(W_INST);
    _spi.write(a2);
    _spi.write(a1);
    _spi.write(a0);
    _spi.write(data);
    chipDisable();
    writeDisable();
//    wait_us(WAIT_US_TBP);
    waitWhileBusy();
}
#if 0
void W25Q16BV::writeStream(int addr, char* buf, int count) {
    if (count < 1)
        return;
    writeEnable();
    chipEnable();
    _spi.write(W_INST);
    _spi.write((addr & ADDR_BMASK2) >> ADDR_BSHIFT2);
    _spi.write((addr & ADDR_BMASK1) >> ADDR_BSHIFT1);
    _spi.write((addr & ADDR_BMASK0) >> ADDR_BSHIFT0);
    for (int i = 0; i < count; i++)
        _spi.write(buf[i]);
    chipDisable();
    writeDisable();
    wait(WAIT_TIME_MS);
}
#else
void W25Q16BV::writeStream(int addr, char* buf, int count)
{
  int left = count;
  int offset = 0;
  int len = 0;
  
  if (count < 1) {
    return;
  }
    
  // find length of first page write
  if ((addr / PAGE_SIZE) != ((addr + count) / PAGE_SIZE)) {
    //spans across at least one boundary
    len = PAGE_SIZE - (addr % PAGE_SIZE);
  } else {
    // ends inside same page => use normal length
    len = count % PAGE_SIZE;
  }
  
  //break up large write operation into several page write operations
  while (left > 0) {
    writeEnable();
    chipEnable();
    _spi.write(W_INST);
    _spi.write(((addr + offset) >> 16) & 0xff);
    _spi.write(((addr + offset) >>  8) & 0xff);
    _spi.write(((addr + offset)      ) & 0xff);
    for (int i = 0; i < len; i++) {
      _spi.write(buf[offset + i]);
    }
    chipDisable();
    //writeDisable();
    
    offset += len;
    left -= len;
    len = (left < PAGE_SIZE) ? left : PAGE_SIZE;
    
    //wait_us(WAIT_US_TPP);
    waitWhileBusy();
  }
}
#endif

//ERASING
void W25Q16BV::chipErase() {
    writeEnable();
    chipEnable();
    _spi.write(C_ERASE_INST);
    chipDisable();
//    writeDisable();
//    wait_us(WAIT_US_TCE);
    waitWhileBusy();
}
bool W25Q16BV::blockErase(int startBlock, int num) {
  if ((num < 1) || (startBlock < 0) || ((startBlock+num) > NUM_64KB_BLOCKS)) {
    return false;
  }
  for (int i = 0; i < num; i++) {
    writeEnable();
    chipEnable();
    _spi.write(B_ERASE_INST);
    _spi.write(startBlock + i);
    _spi.write(0);
    _spi.write(0);
    chipDisable();
//    writeDisable();
//    wait_us(WAIT_US_TBE);
    waitWhileBusy();
  }
  return true;
}
bool W25Q16BV::sectorErase(int startSector, int num) {
  if ((num < 1) || (startSector < 0) || ((startSector+num) > NUM_SECTORS)) {
    return false;
  }
  int addr = startSector * SECTOR_SIZE;
  for (int i = 0; i < num; i++) {
    writeEnable();
    chipEnable();
    _spi.write(S_ERASE_INST);
    _spi.write((addr >> 16) & 0xff);
    _spi.write((addr >>  8) & 0xff);
    _spi.write((addr      ) & 0xff);
    chipDisable();
//    writeDisable();
//    wait_us(WAIT_US_TSE);
    waitWhileBusy();
    
    addr += SECTOR_SIZE;
  }
  return true;
}

// deep power down (default state)
void W25Q16BV::enterDeepPowerDown() {
    chipEnable();
    _spi.write(POWERDOWN_INST);
    chipDisable();
}

// Wakeup from deep power down (default state)
void W25Q16BV::exitDeepPowerDown() {
    chipEnable();
    _spi.write(POWERUP_INST);
    chipDisable();
  wait_us(WAIT_US_TRES1);
}

void W25Q16BV::waitWhileBusy() {
  uint8_t status = 0;
  int i = 0;

  do {
    for (i = 0; i < 0x2000; i++);

    status = readStatus1();
  }
  while ((status & STATUS_1_BUSY) != 0);
}

//ENABLE/DISABLE (private functions)
void W25Q16BV::writeEnable() {
    chipEnable();
    _spi.write(WE_INST);
    chipDisable();
}
void W25Q16BV::writeDisable() {
    chipEnable();
    _spi.write(WD_INST);
    chipDisable();
}
void W25Q16BV::chipEnable() {
    _cs = 0;
}
void W25Q16BV::chipDisable() {
    _cs = 1;
}

