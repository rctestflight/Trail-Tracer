/*
 * rctf_wfrx_status_bits.h
 *
 *  Created on: May 26, 2024
 *      Author: ttyle
 */

#ifndef INC_RCTF_WFRX_STATUS_BITS_H_
#define INC_RCTF_WFRX_STATUS_BITS_H_

#define WFRX_STATUS_CMD_ACK 			(1 << 0)
#define WFRX_STATUS_CMD_PROGRESS 		(1 << 1)
#define WFRX_STATUS_S1_PGA_B0 			(1 << 2)
#define WFRX_STATUS_S1_PGA_B1 			(1 << 3)
#define WFRX_STATUS_S1_SIGNAL_WEAK 		(1 << 4)
#define WFRX_STATUS_S1_SIGNAL_LOST 		(1 << 5)
#define WFRX_STATUS_S1_SIGNAL_OVERLOAD 	(1 << 6)
#define WFRX_STATUS_S2_PGA_B0 			(1 << 7)
#define WFRX_STATUS_S2_PGA_B1 			(1 << 8)
#define WFRX_STATUS_S2_SIGNAL_WEAK 		(1 << 9)
#define WFRX_STATUS_S2_SIGNAL_LOST 		(1 << 10)
#define WFRX_STATUS_S2_SIGNAL_OVERLOAD 	(1 << 11)
#define WFRX_STATUS_VDD_FAULT 			(1 << 12)
#define WFRX_STATUS_CP_ENABLE			(1 << 13)
#define WFRX_STATUS_CP_COOLDOWN			(1 << 14)
#define WFRX_STATUS_ESP_VDD_FAULT		(1 << 15)
#define WFRX_STATUS_5V_FAULT			(1 << 16)
#define WFRX_STATUS_S1_SIGNAL_DISABLED	(1 << 17)
#define WFRX_STATUS_S2_SIGNAL_DISABLED	(1 << 18)

#define WFRX_STATUS_GET_BIT(X, Y)  ((X) & (Y) ? 1 : 0)
#define WFRX_STATUS_SET_BIT(X, Y)  ((X) | (Y))
#define WFRX_STATUS_RESET_BIT(X, Y)  ((X) & (~(Y)))

#endif /* INC_RCTF_WFRX_STATUS_BITS_H_ */