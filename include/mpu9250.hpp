#ifndef MPU9250_HPP
#define MPU9250_HPP

// Hardware Headers
#include "hardware/dma.h"
#include "hardware/spi.h"

// OS Headers
#include "FreeRTOS.h"
#include "task.h"

// Program Headers
#include "math.hpp"

/**
 * @brief MPU9250 Wrapper
 * Frame axis: ENU [East  North Up  ] (Accelerometer & Gyroscope)
 * Frame axis: NED [North East  Down] (Magnetometer)
 * Variance:
 *
 * Full Range
 * Gyroscope
 * 0: +- 250 deg/s
 * 1: +- 500 deg/s
 * 2: +-1000 deg/s
 * 3: +-2000 deg/s
 *
 * Accelerometer
 * 0: +- 2 g
 * 1: +- 4 g
 * 2: +- 8 g
 * 3: +-16 g
 *
 * Magnetometer
 * +- 4912 uT
 */
class MPU9250 {
public:
	MPU9250(spi_inst_t* bus);

public:
	Vector3 Gyroscope;
	Vector3 Accelerometer;
	Vector3 Magnetometer;

private:
	spi_inst_t* bus;
};

#endif /* MPU9250 */
