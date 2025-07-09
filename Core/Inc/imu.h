#ifndef _IMU_H_
#define _IMU_H_

#define IMU_ADDR 0x68
#define IMU_ADDR_8 (IMU_ADDR << 1)
#define IMU_REG 0x3B
#include <stdint.h>

void imu_init(void);
short int imu_read_word(uint8_t reg_addr);
void imu_read_all(short int data[7]);
void imu_calibrate(int euler_offset[2], int gyro_offset[3]);

#endif /* IMU_H_ */
