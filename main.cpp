#include "mbed.h"
#include "mbed_spi.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
 
#define LOG(...)    { printf(__VA_ARGS__); }
 
#define MPU9250_MISO  p16
#define MPU9250_MOSI  p12
#define MPU9250_SCLK  p13
#define MPU9250_CS    p15
#define MPU9250_INT   p10
 
/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)
 
Ticker ticker;
InterruptIn motion_probe(MPU9250_INT);
 
volatile uint8_t compass_event = 0;
volatile uint8_t motion_event = 0;
static signed char board_orientation[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};
 
unsigned short inv_orientation_matrix_to_scalar( const signed char *mtx);
 
 
void compass_tick_handle(void)
{
    compass_event = 1;
}
 
 
void motion_interrupt_handle(void)
{
    motion_event = 1;
}
 
void tap_cb(unsigned char direction, unsigned char count)
{
    LOG("Tap motion detected\n");
}
 
void android_orient_cb(unsigned char orientation)
{
    LOG("Oriention changed\n");
}
 
void disable_uart(void)
{
    // diable uart
    ((NRF_UART_Type *)UART_0)->ENABLE = 0;
    
    // change uart pins' settings
    NRF_GPIO->PIN_CNF[USBTX] = 0x02;
    NRF_GPIO->PIN_CNF[USBRX] = 0x02;
}
 
int main(void)
{
    LOG("---- eMPL MPU library @ Seeed ----\n");
 
    mbed_spi_init(MPU9250_MOSI, MPU9250_MISO, MPU9250_SCLK, MPU9250_CS);
 
 
    if (mpu_init(0)) {
        LOG("failed to initialize mpu9250\r\n");
    }
 
    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS); //
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
 
    /* Read back configuration in case it was set improperly. */
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
 
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(board_orientation));
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
 
    uint16_t dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                            DMP_FEATURE_GYRO_CAL;
 
    dmp_enable_feature(dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
 
    dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
    dmp_set_tap_thresh(TAP_XYZ, 50);
 
    motion_probe.fall(motion_interrupt_handle);
 
    ticker.attach(compass_tick_handle, 0.1);
    
    mbed_spi_disable();
 
    int try_to_sleep = 1;
    while (true) {
        if (motion_event) {
            try_to_sleep = 0;
 
            unsigned long sensor_timestamp;
            short gyro[3], accel[3], sensors;
            long quat[4];
            unsigned char more = 1;
 
            mbed_spi_enable();
            while (more) {
                /* This function gets new data from the FIFO when the DMP is in
                 * use. The FIFO can contain any combination of gyro, accel,
                 * quaternion, and gesture data. The sensors parameter tells the
                 * caller which data fields were actually populated with new data.
                 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
                 * the FIFO isn't being filled with accel data.
                 * The driver parses the gesture data to determine if a gesture
                 * event has occurred; on an event, the application will be notified
                 * via a callback (assuming that a callback function was properly
                 * registered). The more parameter is non-zero if there are
                 * leftover packets in the FIFO.
                 */
                dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
                              &more);
                /* Gyro and accel data are written to the FIFO by the DMP in chip
                 * frame and hardware units. This behavior is convenient because it
                 * keeps the gyro and accel outputs of dmp_read_fifo and
                 * mpu_read_fifo consistent.
                 */
                if (sensors & INV_XYZ_GYRO) {
                    LOG("gyro: %d, %d, %d\n", gyro[0], gyro[1], gyro[2]);
                }
                if (sensors & INV_XYZ_ACCEL) {
                    LOG("acc: %d, %d, %d\n", accel[0], accel[1], accel[2]);
                }
 
                /* Unlike gyro and accel, quaternions are written to the FIFO in
                 * the body frame, q30. The orientation is set by the scalar passed
                 * to dmp_set_orientation during initialization.
                 */
                if (sensors & INV_WXYZ_QUAT) {
                    LOG("QUAT: %ld, %ld, %ld, %ld\n", quat[0], quat[1], quat[2], quat[3]);
                }
 
            }
 
            mbed_spi_disable();
            motion_event = 0;
        }
        
        if (compass_event) {
            try_to_sleep = 0;
            
            unsigned long compass_timestamp = 0;
            short compass[3];
 
            mbed_spi_enable();
            int retval = mpu_get_compass_reg(compass, &compass_timestamp);
            mbed_spi_disable();
            if (retval) {
                LOG("read compass error: %d\n", retval);
            } else {
                LOG("compass: %d, %d, %d\n", compass[0], compass[1], compass[2]);
            }
            
            compass_event = 0;
        }
        
        if (try_to_sleep) {
            sleep();
        }
        
        try_to_sleep = 1;
    }
}
 
/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;
 
    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}
 
unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
 
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
 
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
 
 
    return scalar;
}
