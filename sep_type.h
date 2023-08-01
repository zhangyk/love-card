// SPDX-License-Identifier: GPL-2.0
/*
 * This is the driver for the japan card for usa SoCs.
 *
 * Copyright (C) 2021, usa MicroElectronic Technology - All Rights Reserved
 */

#ifndef _SEP_TYPE_H_
#define _SEP_TYPE_H_

/* Transmit Descriptor - Advanced */
struct gmac_tx_desc {
	u32 txdes0;
	u32 txdes1;
	u32 txdes2;
	u32 txdes3;
	u32 txdes4;
	u32 txdes5;
	u32 txdes6;
	u32 txdes7;
};

/* Receive Descriptor - Advanced */
struct gmac_rx_desc {
	u32 rxdes0;
	u32 rxdes1;
	u32 rxdes2;
	u32 rxdes3;
	u32 rxdes4;
	u32 rxdes5;
	u32 rxdes6;
	u32 rxdes7;
};

struct gmac_buffer_info {
	dma_addr_t dma;
	struct page *page;
	__u32 page_offset;
	__u16 pagecnt_bias;
};

struct gmac_ring {

	void *desc;
	dma_addr_t dma;

	u16 ntu;
	u16 ntc;

	u32 size;  /* length in bytes */
	u16 count; /* amount of descriptors */

	struct gmac_buffer_info *buf_info;
};

/* Admin Queue command opcodes */
enum sep_cmd_opc {
	sep_cmd_opc_reset = 0x0301,
	sep_cmd_opc_init = 0x0302,
	sep_cmd_opc_intv_alloc = 0x0303,
};

/* error codes */
enum sep_cmd_err {
	SEP_CMD_RC_OK = 0,	  /* success */
	SEP_CMD_RC_EPERM = 1,	  /* Operation not permitted */
	SEP_CMD_RC_ENOENT = 2,	  /* No such element */
	SEP_CMD_RC_ESRCH = 3,	  /* Bad opcode */
	SEP_CMD_RC_EINTR = 4,	  /* operation interrupted */
	SEP_CMD_RC_EIO = 5,	  /* I/O error */
	SEP_CMD_RC_ENXIO = 6,	  /* No such resource */
	SEP_CMD_RC_E2BIG = 7,	  /* Arg too long */
	SEP_CMD_RC_EAGAIN = 8,	  /* Try again */
	SEP_CMD_RC_ENOMEM = 9,	  /* Out of memory */
	SEP_CMD_RC_EACCES = 10,	  /* Permission denied */
	SEP_CMD_RC_EFAULT = 11,	  /* Bad address */
	SEP_CMD_RC_EBUSY = 12,	  /* Device or resource busy */
	SEP_CMD_RC_EEXIST = 13,	  /* object already exists */
	SEP_CMD_RC_EINVAL = 14,	  /* Invalid argument */
	SEP_CMD_RC_ENOTTY = 15,	  /* Not a typewriter */
	SEP_CMD_RC_ENOSPC = 16,	  /* No space left or alloc failure */
	SEP_CMD_RC_ENOSYS = 17,	  /* Function not implemented */
	SEP_CMD_RC_ERANGE = 18,	  /* Parameter out of range */
	SEP_CMD_RC_EFLUSHED = 19, /* Cmd flushed due to prev cmd error */
	SEP_CMD_RC_BAD_ADDR = 20, /* Descriptor contains a bad pointer */
	SEP_CMD_RC_EMODE = 21,	  /* Op not allowed in current dev mode */
	SEP_CMD_RC_EFBIG = 22,	  /* File too large */
};

/* ASQ transaction details */
struct sep_cmd_details {
	void *callback; /* cast from type I40E_ADMINQ_CALLBACK */
	u64 cookie;
	u16 flags_ena;
	u16 flags_dis;
};

struct sep_cmd_intv_map_message {
	u32 txq;
	u32 rxq;
	u32 other;
};


/* This header file defines the i40e Admin Queue commands and is shared between
 * i40e Firmware and Software.
 *
 * This file needs to comply with the Linux Kernel coding style.
 */
struct sep_cmd_desc {
	__le16 flags;
	__le16 opcode;
	__le16 datalen;
	__le16 retval;
	union {
		struct {
			__le32 param0;
			__le32 param1;
			__le32 param2;
			__le32 param3;
		} internal;
		struct {
			__le32 param0;
			__le32 param1;
			__le32 addr_high;
			__le32 addr_low;
		} external;
		u8 raw[16];
	} params;
};

/* ARQ event information */
struct sep_event_info {
	struct sep_cmd_desc desc;
	u16 msg_len;
	u16 buf_len;
	u8 *msg_buf;
};

/* memory allocation tracking */
struct sep_dma_mem {
	void *va;
	dma_addr_t pa;
	u32 size;
};

/* Admin Queue information */
struct sep_cmd_info {
	struct sep_dma_mem desc; /* descriptor ring memory */
	struct sep_dma_mem indr_buf;

	u16 init;

	u32 cmd_timeout;	/* send queue cmd write back timeout*/
	u16 cmd_buf_size;	/* send queue buffer size */

	struct mutex cmd_mutex; /* Send queue lock */

	/* last status values on send and receive queues */
	enum sep_cmd_err cmd_last_status;
};

struct sep_data {

	/* device */
	struct pci_dev *pdev;
	struct cdev cdev;
	struct class *cdev_class;
	dev_t c_devid;

	/* memory map */
	void __iomem *io_base;
	uint64_t bar0_phy_addr;
	uint64_t bar0_len;

	/* phy */
	struct delayed_work state_queue;
	struct mutex lock;
	s32 phy_speed;
	s32 phy_duplex;
	unsigned phy_link : 1;
	u8 mdc_cycthr;
	u32 phy_addr;

	/* gmac */
	u8 mac_addr[6];
	struct dentry *dbgfs;
	struct gmac_ring tx_ring;
	struct gmac_ring rx_ring;

	/* open ref */
	spinlock_t spinlock;
	u32 ref;

	struct sep_cmd_info ci;
	struct sep_cmd_info ei;

	/* interrupt */
	struct msix_entry *msix_entries;
};

#endif