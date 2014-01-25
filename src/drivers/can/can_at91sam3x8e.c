/**
 * Glue between SAM3X8E CAN drivers and CSP CAN interface
 * Implements function prototypes found in csp_if_can.h
 *
 * The Atmel SAM3X8E MCU contains two CAN devices. However, CSP only currently
 * supports max one (1) device.  These drivers will allow the user to specify either 
 * CAN0 (default) and CAN1, but not both. However, the infrastructure has been setup 
 * to utilize both interfaces simultaneously in the future.
 *
 * A given CAN device has 8 mailboxes, which are further split into 4 TX and 4 RX 
 * mailboxes. CSP utilizes CAN push model to transfer data
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
#include "sam3x8e/system/drivers/can/can.h"  // sam3x8e-skeleton version 

//extern void addMbox(uint8_t m,uint8_t dlc,uint32_t ul_id); // wongbr temp 

// MOB segmentation ; mbox indices < 4 are for tx mboxes;
#define CAN_TX_MBOXES 4
#define CAN_RX_MBOX_BEGIN_IDX 4
#define CAN_MBOXES  8
#define CAN_RX_TCR_MASK (CAN_TCR_MB4 | CAN_TCR_MB5 | CAN_TCR_MB6 | CAN_TCR_MB7)
#define CAN_RX_IER_MASK (0x00000000 | CAN_IER_MB4 | CAN_IER_MB5 | CAN_IER_MB6 | CAN_IER_MB7)
#define is_tx_mailbox(m)	(m < CAN_RX_MBOX_BEGIN_IDX)
#define is_rx_mailbox(m)	(!is_tx_mailbox(m)) 

// # of attempts to write and read data from registers.
#define WRITE_TRIES 3
#define READ_TRIES 3

// Data specific to the two CAN devices
uint8_t CAN_IDS[] = {ID_CAN0,ID_CAN1};
Can *p_cans[] = {CAN0,CAN1};
IRQn_Type IRQ_numbers[] = {CAN0_IRQn,CAN1_IRQn};
char* can_ifcs[] = {"CAN0","CAN1"};
// callback functions
can_tx_callback_t txcbs[] = {NULL,NULL};
can_rx_callback_t rxcbs[] = {NULL,NULL};

// Indices for the above arrays
#define CAN0_INDEX 0
#define CAN1_INDEX 1
#define INVALID_INDEX -1

// current index of initialized interface. This restricts only one interface to be used 
// at a time. The index is initialized during can_init
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


// ISR prototype
static void can_isr(uint8_t dev);

/** can_init
 * implements csp_if_can.h prototype; initializes specified CAN device
 * @param id - CAN id, calculated by CSP node 
 * @param mask - rx id mask for rx mailboxes
 * @param atxcb - tx callback (triggers csp_if_can to send followup frame in broken up messages)
 * @param arxcb - rx callback
 * @param conf - requires all three fields to be included. 
 * @return 0 on success, -1 on error.
 */
int can_init(uint32_t id, uint32_t mask, can_tx_callback_t atxcb, can_rx_callback_t arxcb, struct csp_can_config *conf) {

  uint32_t baudrate_kbps;
  uint8_t mbox;

  if (curr_idx != INVALID_INDEX) {
    csp_log_warn("Only one device can be activated at a time (device %s is already activated).\r\n",
	   can_ifcs[curr_idx]);
    return -1;
  }

  if (strcmp(conf->ifc,can_ifcs[CAN1_INDEX]) == 0) {
    curr_idx = CAN1_INDEX;
    pio_configure(PIOB,PIO_PERIPH_A, PIO_PB15A_CANRX1|PIO_PB14A_CANTX1, PIO_DEFAULT);
  } else {
    // by default, use CAN0
    curr_idx = CAN0_INDEX;
    pio_configure(PIOA,PIO_PERIPH_A, PIO_PA1A_CANRX0|PIO_PA0A_CANTX0, PIO_DEFAULT);
  }

  csp_assert(conf && conf->bitrate && conf->clock_speed && conf->ifc);
 
  // Convert baudrate from bps to kbps 
  baudrate_kbps = conf->bitrate / 1000;

  // Set callbacks 
  txcbs[curr_idx] = atxcb;
  rxcbs[curr_idx] = arxcb;

  // see use case examples in the comments of sam3x8e/system/drivers/can/sam3xcan.h 
  
  // enable the module clock
  if (pmc_enable_periph_clk(CAN_IDS[curr_idx]) != 0) {
    csp_log_warn("Failure enabling PMC peripheral clock for dev %d!\r\n",curr_idx); 
  } 

  // initialize the sam3x drivers
  if (can_init_hw(p_cans[curr_idx],conf->clock_speed,baudrate_kbps) == 0) {
    csp_log_warn("Failed to initialize %s! clock_speed = %d, baudrate_kbps = %d\r\n",can_ifcs[curr_idx],conf->clock_speed,baudrate_kbps);
      return -1; 
  } else {
    printf("Sam3x8e can device %s initialized. id = 0x%x, mask = 0x%x, clock_speed = %d, baudrate_kbps = %d\r\n",can_ifcs[curr_idx],id,mask, conf->clock_speed,baudrate_kbps);
  }

  // Reset mailboxes
  can_reset_all_mailbox(p_cans[curr_idx]);

  for (mbox = 0; mbox < CAN_MBOXES; mbox++) {
    if (is_rx_mailbox(mbox)) {
      
	mbox_configs[curr_idx][mbox].ul_mb_idx = mbox; // mailbox index (4-7)
	mbox_configs[curr_idx][mbox].uc_obj_type = CAN_MB_RX_MODE;
	mbox_configs[curr_idx][mbox].ul_id_msk = mask; // rx based on device mask
	mbox_configs[curr_idx][mbox].uc_id_ver = 1; // 0 = standard, 1 = extended (CFP uses extended)
	mbox_configs[curr_idx][mbox].ul_id = id; // our ID
	can_mailbox_init(p_cans[curr_idx],&(mbox_configs[curr_idx][mbox])); 

      } else {
      
	mbox_configs[curr_idx][mbox].ul_mb_idx = mbox; // mailbox index (0-3)
	mbox_configs[curr_idx][mbox].uc_obj_type = CAN_MB_TX_MODE;
	mbox_configs[curr_idx][mbox].uc_tx_prio = 15; // priority - relevant only on tx mode
	mbox_configs[curr_idx][mbox].uc_id_ver = 1; // 0 = standard, 1 = extended (CFP uses extended)
	can_mailbox_init(p_cans[curr_idx],&(mbox_configs[curr_idx][mbox]));

      }
  }
    
  can_enable_interrupt(p_cans[curr_idx],CAN_RX_IER_MASK); // allow interrupts in RX mailboxes
  NVIC_EnableIRQ(IRQ_numbers[curr_idx]); // Enable CAN device interrupts 
  can_global_send_transfer_cmd(p_cans[curr_idx],CAN_RX_TCR_MASK); // allow initial receipt for RX mailboxes

   return 0;	
}


/** can_send
 * implements csp_if_can.h prototype; sends out data message
 * @param id - CSP generated can ID
 * @param data - data payload
 * @param dlc - payload size in bytes
 * @param task_woken - used to determine whether to enter critical section.
 * @return 0 on success, -1 on error.
 */

int can_send(can_id_t id, uint8_t data[], uint8_t dlc, CSP_BASE_TYPE * task_woken) {

  int32_t i, m = -1, wc = 0;
  uint32_t temp[2];
  uint32_t writeStatus;

  if (curr_idx == INVALID_INDEX) {
    csp_log_warn("can_send: CAN device not initialized.\r\n");
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
    csp_log_error("TX overflow, no available mailbox\r\n");
    return -1;
  }
  
  mbox_configs[curr_idx][m].uc_length = dlc;
  mbox_configs[curr_idx][m].ul_id = id;
  
  // coppy data  
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
  while ((writeStatus = can_mailbox_write(p_cans[curr_idx],&mbox_configs[curr_idx][m])) != CAN_MAILBOX_TRANSFER_OK) {  
   if (++wc == WRITE_TRIES) {
      csp_log_warn("can_mailbox_write call returned unsuccessfully after %d attempts.\r\n",WRITE_TRIES);
      return -1;
    }
  }

  can_global_send_transfer_cmd(p_cans[curr_idx],0x1u<<m); // initiate transfer
  can_enable_interrupt(p_cans[curr_idx],1<<m); // enable interrupts 

  return 0; 
}

/** CAN ISRs, 
 * implements sam3x8e.h prototype for handlers tied to CAN IRQs
 */
void CAN0_Handler(void) {can_isr(CAN0_INDEX);}
void CAN1_Handler(void) {can_isr(CAN1_INDEX);}

/** can_isr
 * a helper for CAN handlers 
 * @param dev is the index to the appropriate CAN device (for sanity check)
 */
void can_isr(uint8_t dev) {
  
  uint32_t dev_status = 0x0, mb_status = 0x0, read_retval;
  uint8_t m;
  int32_t rc = 0;
  portBASE_TYPE task_woken = pdFALSE;
  can_frame_t frame;

  // Make sure that the handler is for the currently activated device.
  if (curr_idx != dev) {
    printf("Unexpected device index %d\r\n",dev);
    return;
  }

  // Run through the mailboxes 
  for (m = 0; m < CAN_MBOXES; m++) {
    
    // first, check mailbox status from the device
    dev_status = can_get_status(p_cans[curr_idx]);

    if (dev_status & (1 << m)) {

      // next, check mailbox status from the mailbox itself to see if it is ready.
      mb_status = can_mailbox_get_status(p_cans[curr_idx], m);
 
      if ((mb_status & CAN_MSR_MRDY) == CAN_MSR_MRDY) {

	if (is_rx_mailbox(m)) {
	  
	  // update our mailbox config 
	  mbox_configs[curr_idx][m].ul_status = mb_status; // must update ul_status before read 

	  // attempt to read the mailbox
	  rc = 0;

	  while ((read_retval = can_mailbox_read(p_cans[curr_idx],&mbox_configs[curr_idx][m])) != CAN_MAILBOX_TRANSFER_OK) 
	    {

	      if (++rc == READ_TRIES) {
		printf("can_mailbox_read call returned unsuccessfully after %d attempts.\r\n",READ_TRIES);
		return;
	      }	  
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
	  
	  //note: can_mailbox_read does not load the proper (full) ID that
	  //we want, so we have to grab it manually.
	  frame.id = p_cans[curr_idx]->CAN_MB[m].CAN_MID;

	  //addMbox(m,frame.dlc,frame.id); // temp wongbr
	  // Call RX Callback
	  if (rxcbs[curr_idx] != NULL)
	    (rxcbs[curr_idx])(&frame, &task_woken);	  
	  
	} else if (is_tx_mailbox(m) && mbox_tx_statuses[curr_idx][m] != MBOX_TX_FREE) {
	
	  // We have transferred the data

	  // Disable interrupt for mailbox/
	  can_disable_interrupt(p_cans[curr_idx],1<<m);
	
	  // Get identifier 
	  can_id_t id = mbox_configs[curr_idx][m].ul_id;
	
	  // Call TX Callback with no error
	  if (txcbs[curr_idx] != NULL)
	    (txcbs[curr_idx])(id, CAN_NO_ERROR, &task_woken);
	
	  // Release mailbox 
	  mbox_tx_statuses[curr_idx][m] = MBOX_TX_FREE;
	}
      }
    }
  }
}
