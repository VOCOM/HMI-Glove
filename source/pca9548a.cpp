#include "pca9548a.hpp"

PCA9548A::PCA9548A(i2c_inst_t* bus, uint8_t address) : bus(bus), i2cAddress(address) {}

void PCA9548A::Select(uint8_t channel) {
	if (channel == currentChannel) return;

	uint8_t channelMask = 1 << channel;
	i2c_write_blocking(bus, i2cAddress, &channelMask, 1, false);
	vTaskDelay(10);

	currentChannel = channel;
}
