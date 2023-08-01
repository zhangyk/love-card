// SPDX-License-Identifier: GPL-2.0
/*
 * This is the driver for the japan card for usa SoCs.
 *
 * Copyright (C) 2021, usa MicroElectronic Technology - All Rights Reserved
 */

#ifndef _CMD_H_
#define _CMD_H_
#include "sep_type.h"

/* Error Codes */
enum sep_status_code {
	SEP_CMD_SUCCESS = 0,
	SEP_CMD_ERR_NVM = -1,
	SEP_CMD_ERR_NVM_CHECKSUM = -2,
	SEP_CMD_ERR_PHY = -3,
	SEP_CMD_ERR_CONFIG = -4,
	SEP_CMD_ERR_PARAM = -5,
	SEP_CMD_ERR_MAC_TYPE = -6,
	SEP_CMD_ERR_UNKNOWN_PHY = -7,
	SEP_CMD_ERR_LINK_SETUP = -8,
	SEP_CMD_ERR_ADAPTER_STOPPED = -9,
	SEP_CMD_ERR_INVALID_MAC_ADDR = -10,
	SEP_CMD_ERR_DEVICE_NOT_SUPPORTED = -11,
	SEP_CMD_ERR_MASTER_REQUESTS_PENDING = -12,
	SEP_CMD_ERR_INVALID_LINK_SETTINGS = -13,
	SEP_CMD_ERR_AUTONEG_NOT_COMPLETE = -14,
	SEP_CMD_ERR_RESET_FAILED = -15,
	SEP_CMD_ERR_SWFW_SYNC = -16,
	SEP_CMD_ERR_NO_AVAILABLE_VSI = -17,
	SEP_CMD_ERR_NO_MEMORY = -18,
	SEP_CMD_ERR_BAD_PTR = -19,
	SEP_CMD_ERR_RING_FULL = -20,
	SEP_CMD_ERR_INVALID_PD_ID = -21,
	SEP_CMD_ERR_INVALID_QP_ID = -22,
	SEP_CMD_ERR_INVALID_CQ_ID = -23,
	SEP_CMD_ERR_INVALID_CEQ_ID = -24,
	SEP_CMD_ERR_INVALID_AEQ_ID = -25,
	SEP_CMD_ERR_INVALID_SIZE = -26,
	SEP_CMD_ERR_INVALID_ARP_INDEX = -27,
	SEP_CMD_ERR_INVALID_FPM_FUNC_ID = -28,
	SEP_CMD_ERR_QP_INVALID_MSG_SIZE = -29,
	SEP_CMD_ERR_QP_TOOMANY_WRS_POSTED = -30,
	SEP_CMD_ERR_INVALID_FRAG_COUNT = -31,
	SEP_CMD_ERR_QUEUE_EMPTY = -32,
	SEP_CMD_ERR_INVALID_ALIGNMENT = -33,
	SEP_CMD_ERR_FLUSHED_QUEUE = -34,
	SEP_CMD_ERR_INVALID_PUSH_PAGE_INDEX = -35,
	SEP_CMD_ERR_INVALID_IMM_DATA_SIZE = -36,
	SEP_CMD_ERR_TIMEOUT = -37,
	SEP_CMD_ERR_OPCODE_MISMATCH = -38,
	SEP_CMD_ERR_CQP_COMPL_ERROR = -39,
	SEP_CMD_ERR_INVALID_VF_ID = -40,
	SEP_CMD_ERR_INVALID_HMCFN_ID = -41,
	SEP_CMD_ERR_BACKING_PAGE_ERROR = -42,
	SEP_CMD_ERR_NO_PBLCHUNKS_AVAILABLE = -43,
	SEP_CMD_ERR_INVALID_PBLE_INDEX = -44,
	SEP_CMD_ERR_INVALID_SD_INDEX = -45,
	SEP_CMD_ERR_INVALID_PAGE_DESC_INDEX = -46,
	SEP_CMD_ERR_INVALID_SD_TYPE = -47,
	SEP_CMD_ERR_MEMCPY_FAILED = -48,
	SEP_CMD_ERR_INVALID_HMC_OBJ_INDEX = -49,
	SEP_CMD_ERR_INVALID_HMC_OBJ_COUNT = -50,
	SEP_CMD_ERR_INVALID_SRQ_ARM_LIMIT = -51,
	SEP_CMD_ERR_SRQ_ENABLED = -52,
	SEP_CMD_ERR_ADMIN_QUEUE_ERROR = -53,
	SEP_CMD_ERR_ADMIN_QUEUE_TIMEOUT = -54,
	SEP_CMD_ERR_BUF_TOO_SHORT = -55,
	SEP_CMD_ERR_ADMIN_QUEUE_FULL = -56,
	SEP_CMD_ERR_ADMIN_QUEUE_NO_WORK = -57,
	SEP_CMD_ERR_BAD_IWARP_CQE = -58,
	SEP_CMD_ERR_NVM_BLANK_MODE = -59,
	SEP_CMD_ERR_NOT_IMPLEMENTED = -60,
	SEP_CMD_ERR_PE_DOORBELL_NOT_ENABLED = -61,
	SEP_CMD_ERR_DIAG_TEST_FAILED = -62,
	SEP_CMD_ERR_NOT_READY = -63,
	SEP_NOT_SUPPORTED = -64,
	SEP_CMD_ERR_FIRMWARE_API_VERSION = -65,
	SEP_CMD_ERR_ADMIN_QUEUE_CRITICAL_ERROR = -66,
};

typedef enum sep_status_code sep_status;

/* Flags sub-structure
 * |0  |1  |2  |3  |4  |5  |6  |7  |8  |9  |10 |11 |12 |13 |14 |15 |
 * |DD |CMP|ERR|VFE| * *  RESERVED * * |LB |RD |VFC|BUF|SI |EI |FE |
 */

/* command flags and offsets*/
#define SEP_CMD_FLAG_DD_SHIFT  0
#define SEP_CMD_FLAG_CMP_SHIFT 1
#define SEP_CMD_FLAG_ERR_SHIFT 2
#define SEP_CMD_FLAG_VFE_SHIFT 3
#define SEP_CMD_FLAG_LB_SHIFT  9
#define SEP_CMD_FLAG_RD_SHIFT  10
#define SEP_CMD_FLAG_VFC_SHIFT 11
#define SEP_CMD_FLAG_BUF_SHIFT 12
#define SEP_CMD_FLAG_SI_SHIFT  13
#define SEP_CMD_FLAG_EI_SHIFT  14
#define SEP_CMD_FLAG_FE_SHIFT  15

#define SEP_CMD_FLAG_DD	 BIT(SEP_CMD_FLAG_DD_SHIFT)  /* 0x1    */
#define SEP_CMD_FLAG_CMP BIT(SEP_CMD_FLAG_CMP_SHIFT) /* 0x2    */
#define SEP_CMD_FLAG_ERR BIT(SEP_CMD_FLAG_ERR_SHIFT) /* 0x4    */
#define SEP_CMD_FLAG_VFE BIT(SEP_CMD_FLAG_VFE_SHIFT) /* 0x8    */
#define SEP_CMD_FLAG_LB	 BIT(SEP_CMD_FLAG_LB_SHIFT)  /* 0x200  */
#define SEP_CMD_FLAG_RD	 BIT(SEP_CMD_FLAG_RD_SHIFT)  /* 0x400  */
#define SEP_CMD_FLAG_VFC BIT(SEP_CMD_FLAG_VFC_SHIFT) /* 0x800  */
#define SEP_CMD_FLAG_BUF BIT(SEP_CMD_FLAG_BUF_SHIFT) /* 0x1000 */
#define SEP_CMD_FLAG_SI	 BIT(SEP_CMD_FLAG_SI_SHIFT)  /* 0x2000 */
#define SEP_CMD_FLAG_EI	 BIT(SEP_CMD_FLAG_EI_SHIFT)  /* 0x4000 */
#define SEP_CMD_FLAG_FE	 BIT(SEP_CMD_FLAG_FE_SHIFT)  /* 0x8000 */

/* general information */
#define SEP_CMD_LARGE_BUF    512
#define SEP_CMD_CMD_TIMEOUT  1000000 /* usecs */
#define SEP_MAX_CMD_BUF_SIZE 4096
#define SEP_CMD_ALIGNMENT    4096

sep_status sep_cmd_reset(struct sep_data *data);
sep_status sep_cmd_fw_init(struct sep_data *data);
sep_status sep_cmd_intv_alloc(struct sep_data *data);

sep_status sep_init_cmd(struct sep_data *data);
sep_status sep_deinit_cmd(struct sep_data *data);
sep_status sep_init_event(struct sep_data *data);
sep_status sep_deinit_event(struct sep_data *data);
void sep_fill_default_direct_cmd_desc(struct sep_cmd_desc *desc, u16 opcode);
sep_status sep_send_command(struct sep_data *data, struct sep_cmd_desc *desc,
			    void *buff, /* can be NULL */
			    u16 buff_size, struct sep_cmd_details *cmd_details);
bool sep_event_done(struct sep_data *data);
sep_status sep_clean_event(struct sep_data *data);
int sep_free_cmd_dma_mem(struct sep_data *data, struct sep_dma_mem *mem);
void sep_free_cmd_desc(struct sep_data *data);

#endif
