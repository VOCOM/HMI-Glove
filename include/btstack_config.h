#ifndef BTSTACK_CONFIG_HPP
#define BTSTACK_CONFIG_HPP

// --- System ---
#define HAVE_LWIP

// --- Embedded ---
#define HAVE_EMBEDDED_TIME_MS

//  --- FreeRTOS ---
#define HAVE_FREERTOS_INCLUDE_PREFIX

// --- BTstack ---
#define ENABLE_BLE
#define ENABLE_SM
#define ENABLE_LE_PERIPHERAL

// --- Memory Configuration ---
#define HCI_ACL_PAYLOAD_SIZE 256
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4
#define MAX_NR_GATT_CLIENTS 1
#define MAX_NR_HCI_CONNECTIONS 1
#define MAX_NR_SM_LOOKUP_ENTRIES 1

#define MAX_NR_LE_L2CAP_CREDITS 4
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define MAX_ATT_DB_SIZE 512 // Adjust based on your attribute table
#define MAX_NR_GATT_SUBSCRIPTIONS 4
#define MAX_NR_GATT_SERVICES 4
#define MAX_NR_GATT_CHARACTERISTICS 8
#define NVM_NUM_DEVICE_DB_ENTRIES 16

// --- Other ---
#define HAVE_TICK
#define HAVE_INIT_SCRIPT

// --- Debugging ---
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

#endif /* BTSTACK_CONFIG */
