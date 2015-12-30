// W25Q16BV.h

#ifndef W25Q16BV_H
#define W25Q16BV_H

#include "mbed.h"
//#include "BitBangedSPI.h"

#define SPI_FREQ        1000000
#define SPI_MODE        0
#define SPI_NBIT        8

#define POWERUP_INST    0xAB
#define POWERDOWN_INST  0xB9
#define STATUS1_INST    0x05
#define STATUS2_INST    0x35
#define JDEC_INST       0x9F
#define UNIQUE_INST     0x4B
#define WE_INST         0x06
#define WD_INST         0x04
#define R_INST          0x03
#define W_INST          0x02
#define S_ERASE_INST    0x20  /* 4KB sector erase */
#define B_ERASE_INST    0xD8  /* 64KB block erase */
#define C_ERASE_INST    0x60

#define DUMMY_ADDR      0x00

#define WAIT_US_TRES1          5   /* Power Up:                 3us */
//#define WAIT_US_TPUW       10000   /* Power Up Write Time:   1-10ms */
//#define WAIT_US_TBP           50   /* Byte Program Time:    20-50us */
//#define WAIT_US_TPP         3000   /* Page Program Time:    0.7-3ms */
//#define WAIT_US_TSE       400000   /* Sector Erase Time:   30-400ms */
//#define WAIT_US_TBE      1000000   /* 64KB Block Erase Time: 1000ms */
//#define WAIT_US_TCE     10000000   /* Chip Erase Time:       3-10s  */

//#define ADDR_BMASK2     0x00ff0000
//#define ADDR_BMASK1     0x0000ff00
//#define ADDR_BMASK0     0x000000ff

//#define ADDR_BSHIFT2    16
//#define ADDR_BSHIFT1    8
//#define ADDR_BSHIFT0    0

#define PAGE_SIZE          256
#define SECTOR_SIZE       4096
#define NUM_SECTORS        512
#define NUM_64KB_BLOCKS     32

#define STATUS_1_BUSY     0x01

class W25Q16BV /*: public BitBangedSPI*/ {
public:
    W25Q16BV(PinName mosi, PinName miso, PinName sclk, PinName cs);
    
    int readByte(int addr);                                 // takes a 24-bit (3 bytes) address and returns the data (1 byte) at that location
    int readByte(int a2, int a1, int a0);                   // takes the address in 3 separate bytes A[23,16], A[15,8], A[7,0]
    void readStream(int addr, char* buf, int count);        // takes a 24-bit address, reads count bytes, and stores results in buf

    void readJEDEC(uint8_t* manId, uint8_t* memType, uint8_t* cap);
    uint8_t readStatus1();
    uint8_t readStatus2();
    
    void writeByte(int addr, int data);                     // takes a 24-bit (3 bytes) address and a byte of data to write at that location
    void writeByte(int a2, int a1, int a0, int data);       // takes the address in 3 separate bytes A[23,16], A[15,8], A[7,0]
    void writeStream(int addr, char* buf, int count);       // write count bytes of data from buf to memory, starting at addr  
    
    void chipErase();                                       // erase all data on chip
    bool blockErase(int startBlock, int num=1);             // erase all data in the specified number of 64KB blocks, return false if block number is invalid
    bool sectorErase(int startSector, int num=1);           // erase all data in the specified number of  4KB sectors, return false if sector number is invalid
    
    void enterDeepPowerDown();
    
    void exitDeepPowerDown();
    
    
private:

    void waitWhileBusy();

    void writeEnable();                                     // write enable
    void writeDisable();                                    // write disable
    void chipEnable();                                      // chip enable
    void chipDisable();                                     // chip disable
    
//    BitBangedSPI _spi;
    SPI _spi;
    DigitalOut _cs;
};

#endif

