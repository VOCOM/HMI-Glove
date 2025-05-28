#ifndef PCA9548A_HPP
#define PCA9548A_HPP

// Hardware Headers
#include "hardware/i2c.h"

// OS Headers
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief PCA9548A Wrapper
 *
 * #TODO: Abstract I2C Slave interface
 */
class PCA9548A {
public:
	PCA9548A(i2c_inst_t* bus, uint8_t address);

	void Select(uint8_t channel);

private:
	i2c_inst_t* bus;
	uint8_t i2cAddress;
	uint8_t currentChannel{};
};

#endif /* PCA9548A */
