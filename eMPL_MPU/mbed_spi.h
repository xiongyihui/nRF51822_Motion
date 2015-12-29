
/******************************************************************************
 * $Id: mbed_spi.h $
 *****************************************************************************/
/**
 *  @defgroup MBED_System_Layer MBED System Layer
 *  @brief  MBED System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file       mbed_spi.h
 *      @brief      Serial communication functions needed by eMPL to
 *                  communicate to the MPU devices.
 *      @details    This driver assumes that eMPL is with a sub-master clock set
 *                  to 20MHz. The following MBEDs are supported:
 */
#ifndef _MBED_SPI_H_
#define _MBED_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "spi_api.h"

/**
 *  @brief  Set up the SPI port and configure the MBED as the master.
 */
void mbed_spi_init(PinName mosi, PinName miso, PinName sclk, PinName cs);

/**
 *  @brief  Enable SPI port.
 */
void mbed_spi_enable(void);

/**
 *  @brief  Disable SPI communication.
 *  This function will disable the SPI hardware and should be called prior to
 *  entering low-power mode.
 */
void mbed_spi_disable(void);

/**
 *  @brief      Write to a device register.
 *
 *  @param[in]  reg_addr    Slave register to be written to.
 *  @param[in]  length      Number of bytes to write.
 *  @param[out] data        Data to be written to register.
 *
 *  @return     0 if successful.
 */
int mbed_spi_write(unsigned char reg_addr,
                   unsigned char length,
                   unsigned char const *data);
/**
 *  @brief      Read from a device.
 *
 *  @param[in]  reg_addr    Slave register to be read from.
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Data from register.
 *
 *  @return     0 if successful.
 */
int mbed_spi_read(unsigned char reg_addr,
                  unsigned char length,
                  unsigned char *data);

#ifdef __cplusplus
}
#endif

#endif  /* _MBED_I2C_H_ */

/**
 * @}
 */
