#ifndef BLESERVICE_HPP
#define BLESERVICE_HPP

#include "btstack.h"

// Compiled from compile_gatt.py
#include "le_peripheral.h"

#include "math.hpp"

class BLEService {
public:
	void Init(const char* name);
	void Start();

	void Publish(const Quaternion& o, uint32_t timestamp);

private:
	// Packet Handlers
	static void HCIPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
	static void ATTPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);

	static uint16_t ATTReadCallback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size);
	static int ATTWriteCallback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t* buffer, uint16_t buffer_size);

	// Packing Methods
	static uint16_t PackQuaternionW(float val);
	static uint16_t PackQuaternionXYZ(float val);
	void AddToPacket(uint8_t data);
	void AddToPacket(uint16_t data);
	void AddToPacket(uint32_t data);

	static hci_con_handle_t connectionHandle;

	char name[20];
	uint8_t advData[25]{2, 0x01, 0x06, 1, 0x09};

	uint8_t packetSize{};
	uint8_t packet[100]{};

	const uint8_t serviceUUID[16]        = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
	const uint8_t characteristicUUID[16] = {0xFE, 0xED, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
};

#endif /* BLESERVICE */
