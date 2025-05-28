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
 * @brief MPU6050 wrapper
 * Frame axis: ENU [East North Up]
 * Variance: +-0.2864789 deg/s | 0.005 rad/s
 *
 * Full Range | Resolution
 * Accelerometer
 * 0: +- 2 g | 16384 LSB/g
 * 1: +- 4 g |  8192 LSB/g
 * 2: +- 8 g |  4096 LSB/g
 * 3: +-16 g |  2048 LSB/g
 *
 * Gyroscope
 * 0: +- 250 deg/s | 131.0 LSB/deg/s
 * 1: +- 500 deg/s |  65.5 LSB/deg/s
 * 2: +-1000 deg/s |  32.8 LSB/deg/s
 * 3: +-2000 deg/s |  16.4 LSB/deg/s
 *
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

protected:
	void WriteRegister(uint8_t address, uint8_t data);
	void ReadRegister(uint8_t address, uint8_t bytes);

	i2c_inst_t* bus;

	// Addresses
	static const int I2C_ADDRESS           = 0x68;
	static const int REGISTER_CONFIG_GYRO  = 0x1B;
	static const int REGISTER_CONFIG_ACCEL = 0x1C;
	static const int REGISTER_CONFIG_PIN   = 0X37;
	static const int REGISTER_MEASUREMENTS = 0x3B;
	static const int REGISTER_POWER_MGMT   = 0x6B;

private:
	uint8_t buffer[14];
	bool calibrate{false};

	// Low Pass Filter Gain
	float alphaAccel{0.1}; // 0 > a > 1 | LF <-> HF
	float alphaGyro{0.3};  // 0 > a > 1 | LF <-> HF

	// Offsets
	Vector3 offsetGyro{-2.000, 1.414, -1.330};
	Vector3 offsetAccel{0.058, 0.006, -0.049};

	// Resolution
	float resolutionAccel;
	float resolutionGyro;
};

#endif /* MPU6050 */
