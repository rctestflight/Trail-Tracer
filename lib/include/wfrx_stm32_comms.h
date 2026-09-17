#ifndef __WFRX_STM32_COMMS_H
#define __WFRX_STM32_COMMS_H

#include <stdint.h>

// Union to include all generic response datatypes
typedef union{
	int32_t int32;
	float flt32;
} __WfRxGeneric32;

// Define the UART packet regularly sent by the STM32
typedef struct __attribute__((packed)){
	uint32_t status_bits;  // 32 individual status bits, see google doc for defs
	float s1_x;  // sensor pair 1 X distance from wire
	float s1_y;  // sensor pair 1 Y distance from wire
	float s1_radius;  // sensor pair 1 absolute distance from wire
	float s1_angle;  // sensor pair 1 angle to wire
	float s1_btx_radius[2];
	float s2_x;  // sensor pair 2 X distance from wire
	float s2_y;  // sensor pair 2 Y distance from wire
	float s2_radius;  // sensor pair 2 absolute distance from wire
	float s2_angle;  // sensor pair 2 angle to wire
	float s2_btx_radius[2];
	float vbatt;  // Vehicle battery voltage measured through charge port board
	float vcharge;  // external charger voltage measured through charge port board
	float icharge;  // charging current measured through charge port board
	uint32_t charge_code;  // Value used for charge port debugging
	float v5v;  // BEC voltage measured on Rx board
	float vesp_3v3;  // ESP32 3.3V rail voltage measured on Rx board
	float temperature;  // temperature measured inside the STM32
	float coil_data[2][4];  // real and imaginary DFT results for each coil (re(s1c1), im(s1c2), re(s1c2), etc...)
	uint32_t rid;  // STM32 response ID, see google doc for defs
	__WfRxGeneric32 response;  // STM32 response value, can be float or int32 depending on rid
	uint16_t crc;
} __StmTxPacketStruct;

// Union of the ESP packet struct and the UART Rx buffer
typedef union {
	__StmTxPacketStruct items;
	uint8_t uart_data[sizeof(__StmTxPacketStruct)];
}StmTxPacket;

#define STM_TX_PACKET_SIZE sizeof(StmTxPacket)

#define ESP_TX_PACKET_SIZE 8
typedef struct{
	uint32_t cid;
	__WfRxGeneric32 cmd;
} __EspTxPacketStruct;

typedef union{
	__EspTxPacketStruct items;
	uint8_t uart_data[ESP_TX_PACKET_SIZE];
} EspTxPacket;


#endif  // __WFRX_STM32_COMMS_H
