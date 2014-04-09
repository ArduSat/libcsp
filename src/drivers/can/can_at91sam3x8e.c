/*
	Licensed to the Apache Software Foundation (ASF) under one
	or more contributor license agreements.	 See the NOTICE file
	distributed with this work for additional information
	regarding copyright ownership.	The ASF licenses this file
	to you under the Apache License, Version 2.0 (the
	"License"); you may not use this file except in compliance
	with the License.	 You may obtain a copy of the License at

	http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing,
	software distributed under the License is distributed on an
	"AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
	KIND, either express or implied.	See the License for the
	specific language governing permissions and limitations
	under the License.
*/

/**
 * can_at91sam3x8e.c
 * Glue between SAM3X8E CAN drivers and CSP CAN interface
 * Implements function prototypes found in csp_if_can.h
 *
 * The Atmel SAM3X8E MCU contains two CAN devices. However, CSP only currently
 * supports max one (1) device.	 These drivers will allow the user to specify
 * either CAN0 (default) and CAN1, but not both. However, the infrastructure has
 * been setup to utilize both interfaces simultaneously in the future.
 *
 * A given CAN device has 8 mailboxes, which are further split into 4 TX and 4
 * RX mailboxes. CSP utilizes CAN push model to transfer data

 *
 * @author Bryan Wong <bryan@nanosatisfi.com>
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <pio.h>

#include <csp/csp.h>
#include <csp/csp_endian.h>
#include <csp/interfaces/csp_if_can.h>

#include "can.h" // csp version
#include "sam3x8e/system/drivers/can/can.h"	 // sam3x8e-skeleton version
#include "csp/drivers/can_at91sam3x8e.h"

// MOB segmentation ; mbox indices < 4 are for tx mboxes;
#define CAN_TX_MBOXES 4
#define CAN_RX_MBOX_BEGIN_IDX 4
#define CAN_MBOXES	8
#define CAN_RX_TCR_MASK (CAN_TCR_MB4 | CAN_TCR_MB5 | CAN_TCR_MB6 | CAN_TCR_MB7)
#define CAN_RX_IER_MASK (0x00000000 | CAN_IER_MB4 | CAN_IER_MB5 | CAN_IER_MB6 | CAN_IER_MB7)

#define is_tx_mailbox(m)	(m < CAN_RX_MBOX_BEGIN_IDX)
#define is_rx_mailbox(m)	(!is_tx_mailbox(m))

// # of attempts to write and read data from registers.
#define WRITE_TRIES 3
#define READ_TRIES 3
#define RESET_TRIES 10
#define RESET_DISABLE_WAIT_MS 500

// Data specific to the two CAN devices
uint8_t CAN_IDS[] = {ID_CAN0,ID_CAN1};
Can *p_cans[] = {CAN0,CAN1};
IRQn_Type IRQ_numbers[] = {CAN0_IRQn,CAN1_IRQn};
char* can_ifcs[] = {"CAN0","CAN1"};

// callback functions
can_tx_callback_t txcbs[] = {NULL,NULL};
can_rx_callback_t rxcbs[] = {NULL,NULL};
uint8_t can_reset_count[] = {0,0};
uint8_t can_in_middle_of_reset = false;
uint16_t mbox_overwritten_cnt = 0;
uint16_t read_give_up_cnt = 0;
uint16_t queue_full_cnt = 0;
uint16_t isr_cnt = 0;

// Indices for the above arrays
#define CAN0_INDEX 0
#define CAN1_INDEX 1
#define INVALID_INDEX -1

// current index of initialized interface. This restricts only one
// interface to be used at a time. The index is initialized during can_init
int8_t curr_idx = INVALID_INDEX;

// Mailbox status
typedef enum {
	MBOX_TX_FREE = 0,
	MBOX_TX_USED = 1,
} mbox_tx_status_t;

// For storage of mailbox information
#define NUM_CAN_DEVICES 2
mbox_tx_status_t mbox_tx_statuses[NUM_CAN_DEVICES][CAN_TX_MBOXES];
can_mb_conf_t mbox_configs[NUM_CAN_DEVICES][CAN_MBOXES];
uint32_t stored_ids[NUM_CAN_DEVICES];
uint32_t stored_masks[NUM_CAN_DEVICES];
struct csp_can_config stored_configs[NUM_CAN_DEVICES];
uint32_t stored_baudrate_kbps[NUM_CAN_DEVICES];
// ISR prototype
static void can_isr (uint8_t dev);
int can_reset_entry_point (int8_t index);


/**
 * can_init
 * implements csp_if_can.h prototype; initializes specified CAN device
 * @param id - CAN id, calculated by CSP node
 * @param mask - rx id mask for rx mailboxes
 * @param atxcb - tx callback (triggers csp_if_can to send followup frame
 *	in broken up messages)
 * @param arxcb - rx callback
 * @param conf - requires all three fields to be included.
 * @return 0 on success, -1 on error.
 */
int can_init (uint32_t id, uint32_t mask, can_tx_callback_t atxcb,
							can_rx_callback_t arxcb, struct csp_can_config *conf) {

	uint32_t baudrate_kbps;

	if (curr_idx != INVALID_INDEX) {
		csp_log_warn("Only one device can be activated at a time (device %s is already activated).\r\n",
								 can_ifcs[curr_idx]);
		return -1;
	}

	if (strcmp(conf->ifc,can_ifcs[CAN1_INDEX]) == 0) {
		curr_idx = CAN1_INDEX;
		pio_configure(PIOB,PIO_PERIPH_A, PIO_PB15A_CANRX1|PIO_PB14A_CANTX1,
									PIO_DEFAULT);
	} else {
		// by default, use CAN0
		curr_idx = CAN0_INDEX;
		pio_configure(PIOA,PIO_PERIPH_A, PIO_PA1A_CANRX0|PIO_PA0A_CANTX0,
									PIO_DEFAULT);
	}

	csp_assert(conf && conf->bitrate && conf->clock_speed && conf->ifc);

	// Convert baudrate from bps to kbps
	baudrate_kbps = conf->bitrate / 1000;

	// Set callbacks
	txcbs[curr_idx] = atxcb;
	rxcbs[curr_idx] = arxcb;

	// enable the module clock
	if (pmc_enable_periph_clk(CAN_IDS[curr_idx]) != 0) {
		csp_log_warn("Failure enabling PMC peripheral clock for dev %d!\r\n",
								 curr_idx);
	}

	// Store data, so we can reset without needing to pass in data again.
	stored_ids[curr_idx] = id;
	stored_masks[curr_idx] = mask;
	stored_configs[curr_idx] = *conf;
	stored_baudrate_kbps[curr_idx] = baudrate_kbps;
	return can_reset_entry_point(curr_idx);
}


/**
 * can_reset
 * reset the can
 * @index the index of the device
 * @return 0 on success, -1 on error, -2 if not enabled.
 */

int can_reset(int8_t index) {
	uint8_t reset_attempts = 0;
	int status = 0;
	uint8_t consecutive_good = 0;

	// if an ISR occurs it will try to read from the device. This makes
	// sure the read doesn't occur during reset.
	can_in_middle_of_reset = true;

	if (index != curr_idx) return -1;

	// we may need to do this a couple times because errant leftover
	// ISRs and the like may try to access CAN during this time, and
	// causing additional rx/tx errors.

	status = -1;
	while (reset_attempts++ < RESET_TRIES) {

		// disabling resets the system registers
		if (consecutive_good == 0) can_disable(p_cans[curr_idx]);

		vTaskDelay(RESET_DISABLE_WAIT_MS);

		if (consecutive_good == 0) {
			if (can_reset_entry_point(index) != 0) {
				status = -1;
				break;
			}
		}

		if (can_get_rx_error_cnt(p_cans[curr_idx]) == 0 &&
			can_get_tx_error_cnt(p_cans[curr_idx]) == 0) {
			// want to have two consecutive clean systems before we claim success.
			consecutive_good++;
		} else {
			// reset to 0
			consecutive_good = 0;
		}

		// heuristically, 3 seemed to be pretty good.
		if (consecutive_good >= 3) {
			status = 0; // good!
			break;
		}
	}

	if (status == 0) {
		can_reset_count[curr_idx]++;
	}

	can_in_middle_of_reset = false; // allow ISRs to read and do things

	return status;
}


/**
 * can_reset_entry_point
 * this used to be part of can_init, but we allow an entry for resetting the
 * device without necessarily changing the top level configuration.
 * @index the index of the device
 * @return 0 on success, -1 on error, -2 if not enabled.
 */
int can_reset_entry_point (int8_t index) {
	uint8_t mbox;

	// chck against current index.
	if (index != curr_idx) {
		return -2;
	}

	// initialize the sam3x drivers
	if (can_init_hw(p_cans[curr_idx],stored_configs[curr_idx].clock_speed,
					stored_baudrate_kbps[curr_idx]) == 0) {
		csp_log_warn("Failed to initialize %s! clock_speed = %d, baudrate_kbps = %d\r\n",
					 can_ifcs[curr_idx],stored_configs[curr_idx].clock_speed,
					 stored_baudrate_kbps[curr_idx]);
		return -1;
	}

	// Reset mailboxes
	can_reset_all_mailbox(p_cans[curr_idx]);

	for (mbox = 0; mbox < CAN_MBOXES; mbox++) {
		if (is_rx_mailbox(mbox)) {

			mbox_configs[curr_idx][mbox].ul_mb_idx = mbox; // mailbox index (4-7)
			mbox_configs[curr_idx][mbox].uc_obj_type = CAN_MB_RX_MODE;
			mbox_configs[curr_idx][mbox].ul_id_msk = stored_masks[curr_idx]; // rx based on device mask
			mbox_configs[curr_idx][mbox].uc_id_ver = 1; // 1 = extended
			mbox_configs[curr_idx][mbox].ul_id = stored_ids[curr_idx]; // our ID
			can_mailbox_init(p_cans[curr_idx],&(mbox_configs[curr_idx][mbox]));

		} else {

			mbox_configs[curr_idx][mbox].ul_mb_idx = mbox; // mailbox index (0-3)
			mbox_configs[curr_idx][mbox].uc_obj_type = CAN_MB_TX_MODE;
			mbox_configs[curr_idx][mbox].uc_tx_prio = 15; // priority, relevant only on tx mode
			mbox_configs[curr_idx][mbox].uc_id_ver = 1; // 1 = extended
			can_mailbox_init(p_cans[curr_idx],&(mbox_configs[curr_idx][mbox]));

		}
	}

	// allow interrupts and initial receipt for RX mailboxes
	can_enable_interrupt(p_cans[curr_idx],CAN_RX_IER_MASK);
	NVIC_EnableIRQ(IRQ_numbers[curr_idx]); // Enable CAN device interrupts
	can_global_send_transfer_cmd(p_cans[curr_idx],CAN_RX_TCR_MASK);

	return 0;
}


/**
 * can_get_various_cnts
 * returns different statistic counts of the current device
 * @param pointers to the counts
 */
void can_get_various_cnts (uint8_t *reset_cnt_store,
                           uint16_t *queue_full_cnt_store,
                           uint16_t *read_give_up_cnt_store,
                           uint16_t *mbox_overwritten_cnt_store,
                           uint16_t *isr_cnt_store) {
	*reset_cnt_store = can_reset_count[curr_idx];
    *queue_full_cnt_store = queue_full_cnt;
    *read_give_up_cnt_store = read_give_up_cnt;
    *mbox_overwritten_cnt_store = mbox_overwritten_cnt;
    *isr_cnt_store = isr_cnt;
}


/**
 * can_get_active_idx
 * returns curr_idx
 * @return curr_idx
 */
uint8_t can_get_active_idx (void) {
	return curr_idx;
}

/**
 * can_get_num_free_tx_mboxes
 * returns number of free tx mailboxes. If 0, usually that means there
 * is a problem with the bus. (CAN bus constipation)
 * @param idx = index of device you're requesting
 * @return number of free tx mboxes
 */
uint8_t can_get_num_free_tx_mboxes (uint8_t idx) {
	uint8_t i;
	uint8_t sum = 0;

	for(i = 0; i < CAN_TX_MBOXES; i++) {
		if (mbox_tx_statuses[idx][i] == MBOX_TX_FREE) {
			sum++;
		}
	}
	return sum;
}


/**
 * can_send
 * implements csp_if_can.h prototype; sends out data message
 * @param id - CSP generated can ID
 * @param data - data payload
 * @param dlc - payload size in bytes
 * @param task_woken - used to determine whether to enter critical section.
 * @return 0 on success, -1 on error.
 */
int can_send (can_id_t id, uint8_t data[], uint8_t dlc,
							CSP_BASE_TYPE * task_woken) {

	int32_t i, m = -1, wc = 0;
	uint32_t temp[2];
	uint32_t can_status;
	uint32_t can_mr;

	if (curr_idx == INVALID_INDEX) {
		csp_log_warn("can_send: CAN device not initialized.\r\n");
		return -1;
	}

	can_mr = p_cans[curr_idx]->CAN_MR;
	can_status = can_get_status(p_cans[curr_idx]);

	// Check to see if can is disabled. If so, return error.
	if (!(can_mr & CAN_MR_CANEN)) {
		return -1;
	}

	// Check to see if in the middle of reset.
	if (can_in_middle_of_reset) {
		return -1;
	}

	// Check to see if in bus off mode. If so, return error.
	if (can_status & CAN_SR_BOFF) {
		return -1;
	}

	// Disable interrupts while looping mailboxes
	if (task_woken == NULL) {
		portENTER_CRITICAL();
	}

	// Find free mailbox
	for(i = 0; i < CAN_TX_MBOXES; i++) {
		if (mbox_tx_statuses[curr_idx][i] == MBOX_TX_FREE) {
			mbox_tx_statuses[curr_idx][i] = MBOX_TX_USED;
			m = i;
			break;
		}
	}

	// Enable interrupts
	if (task_woken == NULL) {
		portEXIT_CRITICAL();
	}

	// Return if no available mailbox was found
	if (m < 0) {
		return -1;
	}

	mbox_configs[curr_idx][m].uc_length = dlc;
	mbox_configs[curr_idx][m].ul_id = id;

	// copy data
	switch (dlc) {
		case 8:
			*(((uint8_t *) &(temp[0])) + 3) = data[7];
		case 7:
			*(((uint8_t *) &(temp[0])) + 2) = data[6];
		case 6:
			*(((uint8_t *) &(temp[0])) + 1) = data[5];
		case 5:
			*(((uint8_t *) &(temp[0])) + 0) = data[4];
		case 4:
			*(((uint8_t *) &(temp[1])) + 3) = data[3];
		case 3:
			*(((uint8_t *) &(temp[1])) + 2) = data[2];
		case 2:
			*(((uint8_t *) &(temp[1])) + 1) = data[1];
		case 1:
			*(((uint8_t *) &(temp[1])) + 0) = data[0];
		default:
			break;
	}

	mbox_configs[curr_idx][m].ul_datah = temp[0];
	mbox_configs[curr_idx][m].ul_datal = temp[1];

	// update the mailbox
	while (can_mailbox_write(p_cans[curr_idx],&mbox_configs[curr_idx][m]) !=
				 CAN_MAILBOX_TRANSFER_OK) {
		if (++wc == WRITE_TRIES) {
			csp_log_warn("can_mailbox_write call returned unsuccessfully after %d attempts.\r\n",
									 WRITE_TRIES);
			return -1;
		}
	}

	can_global_send_transfer_cmd(p_cans[curr_idx],0x1u<<m); // initiate transfer
	can_enable_interrupt(p_cans[curr_idx],1<<m); // enable interrupts

	return 0;
}


/**
 * CAN ISRs,
 * implements sam3x8e.h prototype for handlers tied to CAN IRQs
 */
void CAN0_Handler (void) {can_isr(CAN0_INDEX);}
void CAN1_Handler (void) {can_isr(CAN1_INDEX);}

/**
 * can_isr
 * a helper for CAN handlers
 * @param dev is the index to the appropriate CAN device (for sanity check)
 */
void can_isr (uint8_t dev) {

	uint32_t dev_status = 0x0, mb_status = 0x0, read_status = 0x0;
	uint8_t m;
	int32_t rc = 0;
	portBASE_TYPE task_woken = pdFALSE;
	can_frame_t frame;
	uint8_t events_occurred;

	// Make sure that the handler is for the currently activated device.
	if (curr_idx != dev) {
		printf("Unexpected device index %d\r\n",dev);
		return;
	}

	do {
		// repeat while there is still incoming mailbox data. Stop when there
		// is no new data.
		events_occurred = false;

		// first, check mailbox status from the device
		dev_status = can_get_status(p_cans[curr_idx]);

		// Run through the mailboxes
		for (m = 0; m < CAN_MBOXES; m++) {

			// next, double check status from the
			// mailbox itself to see if it's ready.
			mb_status = can_mailbox_get_status(p_cans[curr_idx], m);

			if ((dev_status & (1 << m)) &&
				((mb_status & CAN_MSR_MRDY) == CAN_MSR_MRDY)) {
				// when an rx mailbox is ready, it has data.
				// when a tx mailbox is ready, it is empty.

				if (is_rx_mailbox(m)) {

					events_occurred = true;

					// update our mailbox config.
					// must update ul status before read
					mbox_configs[curr_idx][m].ul_status = mb_status;

					// attempt to read the mailbox
					rc = 0;

					while ((read_status = can_mailbox_read(p_cans[curr_idx],
                                                           &mbox_configs[curr_idx][m]))
						   == CAN_MAILBOX_NOT_READY)
						{
							if (++rc == READ_TRIES) {
                                read_give_up_cnt++;
								return;
							}
						}

                    if (read_status == CAN_MAILBOX_RX_OVER) {
                        mbox_overwritten_cnt++;
                    }

					frame.dlc = mbox_configs[curr_idx][m].uc_length;

					uint32_t temp[] = {mbox_configs[curr_idx][m].ul_datah,
									   mbox_configs[curr_idx][m].ul_datal};

					switch (frame.dlc) {
						case 8:
							frame.data[7] = *(((uint8_t *) &(temp[0])) + 3);
						case 7:
							frame.data[6] = *(((uint8_t *) &(temp[0])) + 2);
						case 6:
							frame.data[5] = *(((uint8_t *) &(temp[0])) + 1);
						case 5:
							frame.data[4] = *(((uint8_t *) &(temp[0])) + 0);
						case 4:
							frame.data[3] = *(((uint8_t *) &(temp[1])) + 3);
						case 3:
							frame.data[2] = *(((uint8_t *) &(temp[1])) + 2);
						case 2:
							frame.data[1] = *(((uint8_t *) &(temp[1])) + 1);
						case 1:
							frame.data[0] = *(((uint8_t *) &(temp[1])) + 0);
						default:
							break;

					}

					//note: can_mailbox_read does not load the proper (full)
					//ID that we want, so we have to grab it manually.
					frame.id = p_cans[curr_idx]->CAN_MB[m].CAN_MID;

					// Call RX Callback
					if (rxcbs[curr_idx] != NULL &&
                        ((rxcbs[curr_idx])(&frame, &task_woken) == CSP_ERR_NOMEM)) {
                        queue_full_cnt++;
                    }

				} else if (is_tx_mailbox(m)
						   && mbox_tx_statuses[curr_idx][m]
						   != MBOX_TX_FREE) {

					events_occurred = true;

					// We have transferred the data

					// Disable interrupt for mailbox/
					can_disable_interrupt(p_cans[curr_idx],1<<m);

					// Get identifier
					can_id_t id = mbox_configs[curr_idx][m].ul_id;

					// Call TX Callback with no error
					if (txcbs[curr_idx] != NULL) {
						(txcbs[curr_idx])(id, CAN_NO_ERROR, &task_woken);
					}

					// Release mailbox
					mbox_tx_statuses[curr_idx][m] = MBOX_TX_FREE;
				}
			}
		} // for
	} while (events_occurred); // while
    isr_cnt++;
}
