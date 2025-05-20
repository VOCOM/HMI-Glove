#ifndef MPU6050_HPP
#define MPU6050_HPP

// Hardware Headers
#include "hardware/i2c.h"

// OS Headers
#include "FreeRTOS.h"
#include "task.h"

// Program Headers
#include "dataTypes.hpp"

class MPU6050 {
public:
	MPU6050(i2c_inst_t* bus);

	void Reset();
	void SetAccelerationResolution(uint level);
	void SetGryoscopeResolution(uint level);
	void GetAcceleration();
	void GetGryoscope();
	void GetTemperature();
	void GetAll();

	Vector3 Acceleration;
	Vector3 Gyroscope;
	float Temperature;

private:
	void WriteRegister(uint8_t address, uint8_t data);
	void ReadRegister(uint8_t address);

private:
	i2c_inst_t* bus;
	uint8_t buffer[6];

private: // Resolution
	float RESOLUTION_FORCE;
	float RESOLUTION_ANGULAR;

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
