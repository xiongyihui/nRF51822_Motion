

#ifndef _MPU_MBED_CONFIG_H_
#define _MPU_MBED_CONFIG_H_

#include "wait_api.h"
#include "us_ticker_api.h"

//#define MPU_USE_I2C
#define MPU_USE_SPI

#define MPU9250
//#define MPU6050

#ifdef MPU_USE_I2C
#include "mbed_i2c.h"
#define mpu_hal_write(a, b, c, d)   mbed_i2c_write(a, b, c, d)
#define mpu_hal_read(a, b, c, d)    mbed_i2c_read(a, b, c, d)

#else // MPU_USE_SPI
#include "mbed_spi.h"
#define mpu_hal_write(a, b, c, d)   mbed_spi_write(b, c, d)
#define mpu_hal_read(a, b, c, d)    mbed_spi_read(b, c, d)

#endif

#define __no_operation              __nop
#define delay_ms                    wait_ms
#define log_i(args...)              printf(args)
#define log_e(args...)              printf(args)
#define labs                        abs
#define fabs(x)                     (((x)>0)?(x):-(x))
#define min(x, y)                   (((x) < (y)) ? (x) : (y))

static inline unsigned long get_ms(unsigned long *t)
{
    unsigned long ms = us_ticker_read() / 1000;
    *t = ms;
    return ms;
}
static inline int reg_int_cb(struct int_param_s *int_param)
{
    return 0;
}

#endif // _MPU_MBED_CONFIG_H_