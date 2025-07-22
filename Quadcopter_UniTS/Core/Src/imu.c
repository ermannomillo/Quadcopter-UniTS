#include "imu.h"
#include "main.h"
#include "math.h"
#include "filter.h"

// External handles for I2C and UART communication
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

// Initialize the IMU
void imu_init()
{
	unsigned char data[2];
	data[0] = 0x6b;  // Power management register address
	data[1] = 0;     // Set to 0 to wake up the IMU

	HAL_I2C_Master_Transmit(&hi2c1, IMU_ADDR_8, data, 2, 1000); // Transmit initialization command over I2C
	HAL_Delay(300); // Wait for IMU to stabilize
}

// Read a 16-bit word from a given register address of the IMU
short int imu_read_word(uint8_t reg_addr) {
	unsigned char data[2];

	// Send the register address to read from
	HAL_I2C_Master_Transmit(&hi2c1, IMU_ADDR_8, &reg_addr, 1, 1000);

	// Read 2 bytes of data from the IMU
	HAL_I2C_Master_Receive(&hi2c1, IMU_ADDR_8, data, 2, 1000);

	// Combine the two bytes into a signed 16-bit value
	return (short int)(data[0] << 8) | data[1];
}

// Read all IMU data
void imu_read_all(short int data[7]) {
	int reg = IMU_REG; // Starting register address for sequential reads
	for (int i = 0; i < 7; i++) {
		data[i] = imu_read_word(reg); // Read each 16-bit word
		reg += 2; // Move to next register (2 bytes apart)
	}
}

// Calibrate the IMU
void imu_calibrate(int euler_offset[2], int gyro_offset[3]) {
	long int euler_sum[2] = {0}; // Accumulate pitch and roll
	long int gyro_sum[3] = {0};  // Accumulate gyro data
	short int data[7];           // Raw sensor readings
	int euler[3];                // Computed euler angles (pitch, roll, yaw)

	for (int i = 0; i < 100; i++) {
		imu_read_all(data); // Read all IMU data
		filter_compute_gravity_angles(data[0], data[1], data[2], euler); // Calculate pitch and roll from accelerometer

		// Adjust pitch if it's positive
		if (euler[0] > 0) euler[0] -= 360000;

		// Accumulate pitch and roll
		euler_sum[0] += euler[0];
		euler_sum[1] += euler[1];

		// Accumulate gyro data
		gyro_sum[0] += data[4];
		gyro_sum[1] += data[5];
		gyro_sum[2] += data[6];

		HAL_Delay(10); // Small delay between samples
	}

	// Compute average pitch and roll offsets
	euler_offset[0] = euler_sum[0] / 100;
	euler_offset[1] = euler_sum[1] / 100;

	// Compute average gyro offsets
	for (int i = 0; i < 3; i++) {
		gyro_offset[i] = gyro_sum[i] / 100;
	}
}



