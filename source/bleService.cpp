#include "bleService.hpp"

#include <cstdio>
#include <cstring>

bool BLEService::notificationEnabled          = false;
hci_con_handle_t BLEService::connectionHandle = HCI_CON_HANDLE_INVALID;
uint8_t BLEService::ble_packet_size           = {};
uint8_t BLEService::ble_packet[100]           = {};

void BLEService::Init(const char* name) {
	// ATT
	att_server_init(profile_data, ATTReadCallback, ATTWriteCallback);

	// Register HCI Callback
	static btstack_packet_callback_registration_t btstack_event_callback;
	btstack_event_callback.callback = &PacketHandler;
	hci_add_event_handler(&btstack_event_callback);

	// Register ATT Callback
	att_server_register_packet_handler(PacketHandler);

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

void BLEService::Publish(const Pose pose[6], uint32_t timestamp) {
	if (connectionHandle == HCI_CON_HANDLE_INVALID) return;
	if (!notificationEnabled) return;

	ble_packet_size = 0;
	AddToPacket(timestamp);
	for (int i = 0; i < 6; i++) {
		const Vector3& p    = pose[i].Position;
		const Quaternion& o = pose[i].Orientation;
		AddToPacket(PackQuaternionW(o.w));
		AddToPacket(PackQuaternionXYZ(o.x));
		AddToPacket(PackQuaternionXYZ(o.y));
		AddToPacket(PackQuaternionXYZ(o.z));
		AddToPacket(PackVector3(p.x));
		AddToPacket(PackVector3(p.y));
		AddToPacket(PackVector3(p.z));
	}

	if (att_server_can_send_packet_now(connectionHandle)) {
		att_server_notify(connectionHandle, ATT_CHARACTERISTIC_00000001_0000_1000_8000_00805f9b34fb_01_VALUE_HANDLE, ble_packet, ble_packet_size);
	} else {
		att_server_request_can_send_now_event(connectionHandle);
	}
}

void BLEService::PacketHandler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
	if (packet_type != HCI_EVENT_PACKET) return;

	uint8_t event_type = hci_event_packet_get_type(packet);
	switch (event_type) {
	/* LE Events */
	case GAP_LE_SCAN_START:
		break;
	case GAP_LE_SCAN_STOP:
		break;

	/* HCI Events */
	case HCI_EVENT_META_GAP:
		break;
	case HCI_EVENT_COMMAND_COMPLETE:
		break;
	case HCI_EVENT_LE_META:
		if (hci_event_le_meta_get_subevent_code(packet) != HCI_SUBEVENT_LE_CONNECTION_COMPLETE) break;
		connectionHandle = hci_subevent_le_connection_complete_get_connection_handle(packet);
		printf("BLE: Connected\n");
		break;
	case HCI_EVENT_DISCONNECTION_COMPLETE:
		notificationEnabled = false;
		connectionHandle    = HCI_CON_HANDLE_INVALID;
		printf("BLE: Disconnected\n");
		break;
	case HCI_EVENT_TRANSPORT_PACKET_SENT:
		break;
	case HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS:
		break;
	case HCI_EVENT_VENDOR_SPECIFIC:
		break;

	/* ATT Events */
	case ATT_EVENT_CONNECTED:
		break;
	case ATT_EVENT_DISCONNECTED:
		break;
	case ATT_EVENT_MTU_EXCHANGE_COMPLETE:
		break;
	case ATT_EVENT_HANDLE_VALUE_INDICATION_COMPLETE:
		break;
	case ATT_EVENT_CAN_SEND_NOW:
		att_server_notify(connectionHandle, ATT_CHARACTERISTIC_00000001_0000_1000_8000_00805f9b34fb_01_VALUE_HANDLE, ble_packet, ble_packet_size);
		break;

	/* Unhandled */
	default:
		printf("BLE: Unhandled Packet 0x%X\n", event_type);
		break;
	}
}

uint16_t BLEService::ATTReadCallback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
	printf("BLE: Unhandled ATT Read Callback\n");
	return 0;
}
int BLEService::ATTWriteCallback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
	switch (transaction_mode) {
	case ATT_TRANSACTION_MODE_NONE:
		if (buffer_size > 2) break;
		notificationEnabled = buffer[0] > 0;
		break;
	case ATT_TRANSACTION_MODE_ACTIVE:
		break;
	case ATT_TRANSACTION_MODE_EXECUTE:
		break;
	case ATT_TRANSACTION_MODE_CANCEL:
		break;
	default:
		printf("BLE: Unhandled Transaction Packet\n");
		break;
	}

	return ATT_ERROR_SUCCESS;
}

// Helper Methods
void BLEService::AddToPacket(uint8_t data) {
	ble_packet[ble_packet_size++] = data;
}
void BLEService::AddToPacket(uint16_t data) {
	ble_packet[ble_packet_size++] = data;
	ble_packet[ble_packet_size++] = data >> 8;
}
void BLEService::AddToPacket(uint32_t data) {
	for (int i = 0; i < 4; i++) {
		ble_packet[ble_packet_size++] = data;
		data >>= 8;
	}
}

// Packing Methods
uint16_t BLEService::PackQuaternionW(float val) {
	return val * 65535.0f;
}
uint16_t BLEService::PackQuaternionXYZ(float val) {
	return (val + 1.0f) * 32767.5f;
}
uint16_t BLEService::PackVector3(float val) {
	return (val + 2.0f) * 16383.75f;
}
