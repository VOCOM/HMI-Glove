#include "bleService.hpp"

#include <cstdio>
#include <cstring>

hci_con_handle_t BLEService::connectionHandle;

void BLEService::Init(const char* name) {
	// ATT
	att_server_init(profile_data, ATTReadCallback, ATTWriteCallback);

	// Register HCI Callback
	static btstack_packet_callback_registration_t btstack_event_callback;
	btstack_event_callback.callback = &HCIPacketHandler;
	hci_add_event_handler(&btstack_event_callback);

	// Register ATT Callback
	// static btstack_packet_callback_registration_t att_event_callback;
	// att_server_register_packet_handler();

	uint8_t len = std::strlen(name);
	if (len > 20) len = 20;
	std::strncpy(this->name, name, len);

	advData[3] += len;
	for (int i = 0; i < len; i++) advData[5 + i] = name[i];

	// GAP Advertisement
	gap_advertisements_set_params(0x0030, 0x0030, 0, 0, 0, 0, 0);
	gap_advertisements_set_data(sizeof(advData), advData);
	gap_advertisements_enable(1);
}

void BLEService::Start() {
	// Power on BLE Controller
	hci_power_control(HCI_POWER_ON);

	btstack_run_loop_execute();
}

void BLEService::Publish(const Quaternion& o, uint32_t timestamp) {
	if (connectionHandle == HCI_CON_HANDLE_INVALID) return;

	packetSize = 0;
	AddToPacket(timestamp);
	AddToPacket(PackQuaternionW(o.w));
	AddToPacket(PackQuaternionXYZ(o.x));
	AddToPacket(PackQuaternionXYZ(o.y));
	AddToPacket(PackQuaternionXYZ(o.z));

	if (att_server_can_send_packet_now(connectionHandle)) {
		att_server_notify(connectionHandle, ATT_CHARACTERISTIC_00000001_0000_1000_8000_00805f9b34fb_01_VALUE_HANDLE, packet, packetSize);
	} else {
		att_server_request_can_send_now_event(connectionHandle);
	}
}

void BLEService::HCIPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
	if (packet_type != HCI_EVENT_PACKET) return;

	switch (hci_event_packet_get_type(packet)) {
	case BTSTACK_EVENT_STATE: // Status OK
		break;
	case HCI_EVENT_LE_META: // Waiting for Connection
		if (hci_event_le_meta_get_subevent_code(packet) != HCI_SUBEVENT_LE_CONNECTION_COMPLETE) break;
		connectionHandle = hci_subevent_le_connection_complete_get_connection_handle(packet);
		printf("Connected\n");
		break;
	case HCI_EVENT_DISCONNECTION_COMPLETE: // Disconnected
		printf("Disconnected\n");
		break;
	default:
		break;
	}
}
void BLEService::ATTPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
	if (packet_type != HCI_EVENT_PACKET) return;

	switch (hci_event_packet_get_type(packet)) {
	case ATT_EVENT_CAN_SEND_NOW:
		break;
	default:
		break;
	}
}

uint16_t BLEService::ATTReadCallback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
	if (att_handle == ATT_CHARACTERISTIC_00000001_0000_1000_8000_00805f9b34fb_01_VALUE_HANDLE) {
		return 0;
	}

	return 0;
}
int BLEService::ATTWriteCallback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
	return 0;
}

uint16_t BLEService::PackQuaternionW(float val) {
	return val * 65535.0f;
}
uint16_t BLEService::PackQuaternionXYZ(float val) {
	return (val + 1.0f) * 32767.5f;
}

void BLEService::AddToPacket(uint8_t data) {
	packet[packetSize++] = data;
}
void BLEService::AddToPacket(uint16_t data) {
	packet[packetSize++] = data;
	packet[packetSize++] = data >> 8;
}
void BLEService::AddToPacket(uint32_t data) {
	for (int i = 0; i < 4; i++) {
		packet[packetSize++] = data;
		data >>= 8;
	}
}
