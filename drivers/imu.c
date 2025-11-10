#include "imu.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdio.h>

// ========== I2C CONFIGURATION ==========
// Grove Port 6: GP26 (SDA), GP27 (SCL)
#define I2C_PORT i2c1
#define I2C_SDA 26
#define I2C_SCL 27
#define I2C_FREQ 400000

// ========== LSM303DLHC ADDRESSES ==========
#define ACCEL_ADDR 0x19
#define MAG_ADDR 0x1E

// ========== ACCELEROMETER REGISTERS ==========
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23
#define OUT_X_L_A 0x28

// ========== MAGNETOMETER REGISTERS ==========
#define CRA_REG_M 0x00
#define MR_REG_M 0x02
#define OUT_X_H_M 0x03

// ========== CALIBRATION VALUES ==========
// *** IMPORTANT: These are YOUR calibration values from the rover! ***
#define MAG_X_OFFSET 0.045455f
#define MAG_Y_OFFSET 0.070000f
#define MAG_Z_OFFSET 0.036224f

// ========== INTERNAL FUNCTIONS ==========

static void i2c_init_custom(void) {
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

static void lsm303_init(void) {
    uint8_t buf[2];
    
    // Configure accelerometer: 100Hz, normal mode, all axes enabled
    buf[0] = CTRL_REG1_A;
    buf[1] = 0x57;
    i2c_write_blocking(I2C_PORT, ACCEL_ADDR, buf, 2, false);
    
    // Configure accelerometer: ±2g scale, high resolution
    buf[0] = CTRL_REG4_A;
    buf[1] = 0x08;
    i2c_write_blocking(I2C_PORT, ACCEL_ADDR, buf, 2, false);
    
    // Configure magnetometer: 75Hz output rate
    buf[0] = CRA_REG_M;
    buf[1] = 0x18;
    i2c_write_blocking(I2C_PORT, MAG_ADDR, buf, 2, false);
    
    // Configure magnetometer: Continuous conversion mode
    buf[0] = MR_REG_M;
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, MAG_ADDR, buf, 2, false);
    
    sleep_ms(100);
}

// ========== PUBLIC FUNCTIONS ==========

void imu_init(void) {
    i2c_init_custom();
    lsm303_init();
    printf("IMU initialized on I2C1 (GP26/GP27) with calibration offsets:\n");
    printf("  X=%.6f, Y=%.6f, Z=%.6f\n", MAG_X_OFFSET, MAG_Y_OFFSET, MAG_Z_OFFSET);
}

void imu_read_accel(imu_vector_t *accel) {
    uint8_t buffer[6];
    uint8_t reg = OUT_X_L_A | 0x80; // Auto-increment
    
    i2c_write_blocking(I2C_PORT, ACCEL_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ACCEL_ADDR, buffer, 6, false);
    
    // Combine bytes (little endian)
    int16_t x_raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t y_raw = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t z_raw = (int16_t)(buffer[5] << 8 | buffer[4]);
    
    // Convert to g (±2g scale, 12-bit resolution)
    accel->x = x_raw / 16384.0f;
    accel->y = y_raw / 16384.0f;
    accel->z = z_raw / 16384.0f;
}

void imu_read_mag(imu_vector_t *mag) {
    uint8_t buffer[6];
    uint8_t reg = OUT_X_H_M;
    
    i2c_write_blocking(I2C_PORT, MAG_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MAG_ADDR, buffer, 6, false);
    
    // Combine bytes (big endian) - X, Z, Y order in hardware
    int16_t x_raw = (int16_t)(buffer[0] << 8 | buffer[1]);
    int16_t z_raw = (int16_t)(buffer[2] << 8 | buffer[3]);
    int16_t y_raw = (int16_t)(buffer[4] << 8 | buffer[5]);
    
    // Convert to gauss and apply calibration
    mag->x = (x_raw / 1100.0f) - MAG_X_OFFSET;
    mag->y = (y_raw / 1100.0f) - MAG_Y_OFFSET;
    mag->z = (z_raw / 980.0f) - MAG_Z_OFFSET;
}

float imu_calculate_heading(imu_vector_t *mag) {
    float heading = atan2(mag->y, mag->x) * 180.0f / M_PI;
    if (heading < 0) {
        heading += 360.0f;
    }
    return heading;
}

float imu_calculate_tilt_x(imu_vector_t *accel) {
    return atan2(accel->y, accel->z) * 180.0f / M_PI;
}

float imu_calculate_tilt_y(imu_vector_t *accel) {
    return atan2(-accel->x, sqrt(accel->y * accel->y + accel->z * accel->z)) * 180.0f / M_PI;
}