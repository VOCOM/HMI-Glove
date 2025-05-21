#ifndef MPU6050_HPP
#define MPU6050_HPP

// Hardware Headers
#include "hardware/i2c.h"

// OS Headers
#include "FreeRTOS.h"
#include "task.h"

// Program Headers
#include "math.hpp"

/**
 * ID: Range | Resolution
 * Force
 * 0: +- 2 g | 16384 LSB/g
 * 1: +- 4 g |  8192 LSB/g
 * 2: +- 8 g |  4096 LSB/g
 * 3: +-16 g |  2048 LSB/g
 *
 * Angular
 * 0: +- 250 deg/s | 131.0 LSB/deg/s
 * 1: +- 500 deg/s |  65.5 LSB/deg/s
 * 2: +-1000 deg/s |  32.8 LSB/deg/s
 * 3: +-2000 deg/s |  16.4 LSB/deg/s
 */

class MPU6050 {
public:
	MPU6050(i2c_inst_t* bus);

	void Reset();
	void Calibrate();
	void Update();

	void SetAccelerationResolution(uint level);
	void SetGryoscopeResolution(uint level);

	Vector3 Acceleration;
	Vector3 Gyroscope;
	float Temperature;

private:
	void UpdateAcceleration();
	void UpdateGryoscope();
	void UpdateTemperature();

	void WriteRegister(uint8_t address, uint8_t data);
	void ReadRegister(uint8_t address);

private:
	i2c_inst_t* bus;
	uint8_t buffer[6];

	// Filters
	bool useFilters  = true;
	float alphaAccel = 0.3;
	float alphaGyro  = 0.2;

	// Offsets
	Vector3 OFFSET_ACCELEROMETER = {0.064, 0.007, -0.053};
	Vector3 OFFSET_GYROSCOPE     = {-2.038, 1.399, -1.285};

	// Resolution
	float resForce;
	float resAngular;

private: // Addresses
	static const int I2C_ADDRESS           = 0x68;
	static const int REGISTER_PWR_MGMT     = 0x6B;
	static const int REGISTER_ACCEL        = 0x3B;
	static const int REGISTER_ACCEL_CONFIG = 0x1C;
	static const int REGISTER_GYRO         = 0x43;
	static const int REGISTER_GYRO_CONFIG  = 0x1B;
	static const int REGISTER_TEMP         = 0x41;
};

#endif /* MPU6050 */
