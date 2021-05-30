/*
 * deca_timestamps.h
 *
 *  Created on: 27 Απρ 2021
 *      Author: kostasdeligiorgis
 */

#ifndef INC_DECA_TIMESTAMPS_H_
#define INC_DECA_TIMESTAMPS_H_

#include <DWM_functions.h>
#include <stdlib.h>

#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "main.h"


extern uint64_t get_tx_timestamp_u64(void);
extern uint64_t get_rx_timestamp_u64(void);
extern void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
extern void final_msg_set_ts(uint8 *ts_field, uint64_t ts);


#endif /* INC_DECA_TIMESTAMPS_H_ */
