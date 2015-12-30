
#include <stdio.h>
#include "spi_api.h"
#include "gpio_api.h"

#ifdef TARGET_MCU_NRF51822
#include "nrf51.h"
static PinName mbed_spi_mosi;
static PinName mbed_spi_miso;
static PinName mbed_spi_sclk;
static PinName mbed_spi_cs;
static uint32_t mbed_spi_mosi_cnf;
static uint32_t mbed_spi_miso_cnf;
static uint32_t mbed_spi_sclk_cnf;
static uint32_t mbed_spi_cs_cnf;
#endif

static spi_t mbed_spi_object;
static gpio_t mbed_cs_object;



void mbed_spi_init(PinName mosi, PinName miso, PinName sclk, PinName cs)
{
    spi_init(&mbed_spi_object, mosi, miso, sclk, NC);
    spi_format(&mbed_spi_object, 8, 0, 0);
    spi_frequency(&mbed_spi_object, 1000000);

    gpio_init_out(&mbed_cs_object, cs);
    gpio_write(&mbed_cs_object, 1);

#ifdef TARGET_MCU_NRF51822
    mbed_spi_mosi = mosi;
    mbed_spi_miso = miso;
    mbed_spi_sclk = sclk;
    mbed_spi_cs   = cs;
    
    mbed_spi_mosi_cnf = NRF_GPIO->PIN_CNF[mosi];
    mbed_spi_miso_cnf = NRF_GPIO->PIN_CNF[miso];
    mbed_spi_sclk_cnf = NRF_GPIO->PIN_CNF[sclk];
    mbed_spi_cs_cnf = NRF_GPIO->PIN_CNF[cs];
#endif
}

int mbed_spi_write(unsigned char reg_addr,
                   unsigned char length,
                   unsigned char const *data)
{
    int i;

    gpio_write(&mbed_cs_object, 0);
    spi_master_write(&mbed_spi_object, reg_addr);
    for (i = 0; i < length; i++) {
        spi_master_write(&mbed_spi_object, data[i]);
    }
    gpio_write(&mbed_cs_object, 1);
    return 0;
}

int mbed_spi_read(unsigned char reg_addr,
                  unsigned char length,
                  unsigned char *data)
{
    int i;

    gpio_write(&mbed_cs_object, 0);
    spi_master_write(&mbed_spi_object, reg_addr | 0x80);
    for (i = 0; i < length; i++) {
        data[i] = spi_master_write(&mbed_spi_object, 0xff);
    }

    gpio_write(&mbed_cs_object, 1);
    return 0;
}

void mbed_spi_enable(void)
{
#if defined(TARGET_MCU_NRF51822) && !defined(DEBUG)
    // NRF_GPIO->PIN_CNF[mbed_spi_mosi] = mbed_spi_mosi_cnf;
    // NRF_GPIO->PIN_CNF[mbed_spi_miso] = mbed_spi_miso_cnf;
    // NRF_GPIO->PIN_CNF[mbed_spi_sclk] = mbed_spi_sclk_cnf;
    // NRF_GPIO->PIN_CNF[mbed_spi_cs] = mbed_spi_cs_cnf;
    
    mbed_spi_object.spi->ENABLE = 1;
#endif
}

void mbed_spi_disable(void)
{
#if defined(TARGET_MCU_NRF51822) && !defined(DEBUG)
    mbed_spi_mosi_cnf = NRF_GPIO->PIN_CNF[mbed_spi_mosi];
    mbed_spi_miso_cnf = NRF_GPIO->PIN_CNF[mbed_spi_miso];
    mbed_spi_sclk_cnf = NRF_GPIO->PIN_CNF[mbed_spi_sclk];
    mbed_spi_cs_cnf = NRF_GPIO->PIN_CNF[mbed_spi_cs];
    
    mbed_spi_object.spi->ENABLE = 0;
    
    // NRF_GPIO->PIN_CNF[mbed_spi_mosi] = 2;
    // NRF_GPIO->PIN_CNF[mbed_spi_miso] = 2;
    // NRF_GPIO->PIN_CNF[mbed_spi_sclk] = 2;
    // NRF_GPIO->PIN_CNF[mbed_spi_cs] = 2;
#endif
}
