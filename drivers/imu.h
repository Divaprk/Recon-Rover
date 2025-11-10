#ifndef IMU_H
#define IMU_H

#include <stdint.h>

// 3D vector structure for sensor data
typedef struct {
    float x;
    float y;
    float z;
} imu_vector_t;

/**
 * @brief Initialize the IMU sensor
 * Call this once in main() during initialization
 */
void imu_init(void);

/**
 * @brief Read accelerometer data
 * @param accel Pointer to vector structure to store acceleration in g
 */
void imu_read_accel(imu_vector_t *accel);

/**
 * @brief Read magnetometer data (already calibrated)
 * @param mag Pointer to vector structure to store magnetic field in gauss
 */
void imu_read_mag(imu_vector_t *mag);

/**
 * @brief Calculate compass heading from magnetometer data
 * @param mag Pointer to magnetometer vector
 * @return Heading in degrees (0-360, where 0 = North)
 */
float imu_calculate_heading(imu_vector_t *mag);

/**
 * @brief Calculate tilt angle around X axis
 * @param accel Pointer to accelerometer vector
 * @return Tilt angle in degrees
 */
float imu_calculate_tilt_x(imu_vector_t *accel);

/**
 * @brief Calculate tilt angle around Y axis
 * @param accel Pointer to accelerometer vector
 * @return Tilt angle in degrees
 */
float imu_calculate_tilt_y(imu_vector_t *accel);

#endif // IMU_H