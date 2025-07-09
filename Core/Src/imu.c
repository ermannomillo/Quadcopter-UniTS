#include "imu.h"
#include "main.h"
#include "math.h"
#include "filter.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

void imu_init()
{
	unsigned char data[2];
	data[0]=0x6b;
	data[1]=0;

	HAL_I2C_Master_Transmit(&hi2c1, IMU_ADDR_8, data, 2, 1000);
	HAL_Delay(300);
}

short int imu_read_word(uint8_t reg_addr) {
	unsigned char data[2];
	HAL_I2C_Master_Transmit(&hi2c1, IMU_ADDR_8, &reg_addr, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, IMU_ADDR_8, data, 2, 1000);
	return (short int)(data[0] << 8) | data[1];
}

void imu_read_all(short int data[7]) {
	int reg = IMU_REG;
	for (int i = 0; i < 7; i++) {
		data[i] = imu_read_word(reg);
		reg += 2;
	}
}

void imu_calibrate(int euler_offset[2], int gyro_offset[3]) {
	long int euler_sum[2] = {0};
	long int gyro_sum[3] = {0};
	short int data[7];
	int euler[3];

	for (int i = 0; i < 100; i++) {
		imu_read_all(data);
		filter_compute_gravity_angles(data[0], data[1], data[2], euler);

		if (euler[0] > 0) euler[0] -= 360000;

		euler_sum[0] += euler[0];
		euler_sum[1] += euler[1];

		gyro_sum[0] += data[4];
		gyro_sum[1] += data[5];
		gyro_sum[2] += data[6];

		HAL_Delay(10);
	}

	euler_offset[0] = euler_sum[0] / 100;
	euler_offset[1] = euler_sum[1] / 100;

	for (int i = 0; i < 3; i++) {
		gyro_offset[i] = gyro_sum[i] / 100;
	}

}


