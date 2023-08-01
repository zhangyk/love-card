// SPDX-License-Identifier: GPL-2.0
/*
 * This is the driver for the japan card for usa SoCs.
 *
 * Copyright (C) 2021, usa MicroElectronic Technology - All Rights Reserved
 */

#include <linux/types.h>
#include <linux/io.h>
#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>

#include "sep_main.h"

const char *sep_cmd_err_str(struct sep_data *data, enum sep_cmd_err aq_err)
{
	switch (aq_err) {
		case SEP_CMD_RC_OK:
			return "OK";
		case SEP_CMD_RC_EPERM:
			return "SEP_CMD_RC_EPERM";
		case SEP_CMD_RC_ENOENT:
			return "SEP_CMD_RC_ENOENT";
		case SEP_CMD_RC_ESRCH:
			return "SEP_CMD_RC_ESRCH";
		case SEP_CMD_RC_EINTR:
			return "SEP_CMD_RC_EINTR";
		case SEP_CMD_RC_EIO:
			return "SEP_CMD_RC_EIO";
		case SEP_CMD_RC_ENXIO:
			return "SEP_CMD_RC_ENXIO";
		case SEP_CMD_RC_E2BIG:
			return "SEP_CMD_RC_E2BIG";
		case SEP_CMD_RC_EAGAIN:
			return "SEP_CMD_RC_EAGAIN";
		case SEP_CMD_RC_ENOMEM:
			return "SEP_CMD_RC_ENOMEM";
		case SEP_CMD_RC_EACCES:
			return "SEP_CMD_RC_EACCES";
		case SEP_CMD_RC_EFAULT:
			return "SEP_CMD_RC_EFAULT";
		case SEP_CMD_RC_EBUSY:
			return "SEP_CMD_RC_EBUSY";
		case SEP_CMD_RC_EEXIST:
			return "SEP_CMD_RC_EEXIST";
		case SEP_CMD_RC_EINVAL:
			return "SEP_CMD_RC_EINVAL";
		case SEP_CMD_RC_ENOTTY:
			return "SEP_CMD_RC_ENOTTY";
		case SEP_CMD_RC_ENOSPC:
			return "SEP_CMD_RC_ENOSPC";
		case SEP_CMD_RC_ENOSYS:
			return "SEP_CMD_RC_ENOSYS";
		case SEP_CMD_RC_ERANGE:
			return "SEP_CMD_RC_ERANGE";
		case SEP_CMD_RC_EFLUSHED:
			return "SEP_CMD_RC_EFLUSHED";
		case SEP_CMD_RC_BAD_ADDR:
			return "SEP_CMD_RC_BAD_ADDR";
		case SEP_CMD_RC_EMODE:
			return "SEP_CMD_RC_EMODE";
		case SEP_CMD_RC_EFBIG:
			return "SEP_CMD_RC_EFBIG";
	}

	return "SEP CMD : Un-Known RC";
}

const char *sep_cmd_stat_str(struct sep_data *data, sep_status stat_err)
{
	switch (stat_err) {
		case 0:
			return "OK";
		case SEP_CMD_ERR_NVM:
			return "SEP_CMD_ERR_NVM";
		case SEP_CMD_ERR_NVM_CHECKSUM:
			return "SEP_CMD_ERR_NVM_CHECKSUM";
		case SEP_CMD_ERR_PHY:
			return "SEP_CMD_ERR_PHY";
		case SEP_CMD_ERR_CONFIG:
			return "SEP_CMD_ERR_CONFIG";
		case SEP_CMD_ERR_PARAM:
			return "SEP_CMD_ERR_PARAM";
		case SEP_CMD_ERR_MAC_TYPE:
			return "SEP_CMD_ERR_MAC_TYPE";
		case SEP_CMD_ERR_UNKNOWN_PHY:
			return "SEP_CMD_ERR_UNKNOWN_PHY";
		case SEP_CMD_ERR_LINK_SETUP:
			return "SEP_CMD_ERR_LINK_SETUP";
		case SEP_CMD_ERR_ADAPTER_STOPPED:
			return "SEP_CMD_ERR_ADAPTER_STOPPED";
		case SEP_CMD_ERR_INVALID_MAC_ADDR:
			return "SEP_CMD_ERR_INVALID_MAC_ADDR";
		case SEP_CMD_ERR_DEVICE_NOT_SUPPORTED:
			return "SEP_CMD_ERR_DEVICE_NOT_SUPPORTED";
		case SEP_CMD_ERR_MASTER_REQUESTS_PENDING:
			return "SEP_CMD_ERR_MASTER_REQUESTS_PENDING";
		case SEP_CMD_ERR_INVALID_LINK_SETTINGS:
			return "SEP_CMD_ERR_INVALID_LINK_SETTINGS";
		case SEP_CMD_ERR_AUTONEG_NOT_COMPLETE:
			return "SEP_CMD_ERR_AUTONEG_NOT_COMPLETE";
		case SEP_CMD_ERR_RESET_FAILED:
			return "SEP_CMD_ERR_RESET_FAILED";
		case SEP_CMD_ERR_SWFW_SYNC:
			return "SEP_CMD_ERR_SWFW_SYNC";
		case SEP_CMD_ERR_NO_AVAILABLE_VSI:
			return "SEP_CMD_ERR_NO_AVAILABLE_VSI";
		case SEP_CMD_ERR_NO_MEMORY:
			return "SEP_CMD_ERR_NO_MEMORY";
		case SEP_CMD_ERR_BAD_PTR:
			return "SEP_CMD_ERR_BAD_PTR";
		case SEP_CMD_ERR_RING_FULL:
			return "SEP_CMD_ERR_RING_FULL";
		case SEP_CMD_ERR_INVALID_PD_ID:
			return "SEP_CMD_ERR_INVALID_PD_ID";
		case SEP_CMD_ERR_INVALID_QP_ID:
			return "SEP_CMD_ERR_INVALID_QP_ID";
		case SEP_CMD_ERR_INVALID_CQ_ID:
			return "SEP_CMD_ERR_INVALID_CQ_ID";
		case SEP_CMD_ERR_INVALID_CEQ_ID:
			return "SEP_CMD_ERR_INVALID_CEQ_ID";
		case SEP_CMD_ERR_INVALID_AEQ_ID:
			return "SEP_CMD_ERR_INVALID_AEQ_ID";
		case SEP_CMD_ERR_INVALID_SIZE:
			return "SEP_CMD_ERR_INVALID_SIZE";
		case SEP_CMD_ERR_INVALID_ARP_INDEX:
			return "SEP_CMD_ERR_INVALID_ARP_INDEX";
		case SEP_CMD_ERR_INVALID_FPM_FUNC_ID:
			return "SEP_CMD_ERR_INVALID_FPM_FUNC_ID";
		case SEP_CMD_ERR_QP_INVALID_MSG_SIZE:
			return "SEP_CMD_ERR_QP_INVALID_MSG_SIZE";
		case SEP_CMD_ERR_QP_TOOMANY_WRS_POSTED:
			return "SEP_CMD_ERR_QP_TOOMANY_WRS_POSTED";
		case SEP_CMD_ERR_INVALID_FRAG_COUNT:
			return "SEP_CMD_ERR_INVALID_FRAG_COUNT";
		case SEP_CMD_ERR_QUEUE_EMPTY:
			return "SEP_CMD_ERR_QUEUE_EMPTY";
		case SEP_CMD_ERR_INVALID_ALIGNMENT:
			return "SEP_CMD_ERR_INVALID_ALIGNMENT";
		case SEP_CMD_ERR_FLUSHED_QUEUE:
			return "SEP_CMD_ERR_FLUSHED_QUEUE";
		case SEP_CMD_ERR_INVALID_PUSH_PAGE_INDEX:
			return "SEP_CMD_ERR_INVALID_PUSH_PAGE_INDEX";
		case SEP_CMD_ERR_INVALID_IMM_DATA_SIZE:
			return "SEP_CMD_ERR_INVALID_IMM_DATA_SIZE";
		case SEP_CMD_ERR_TIMEOUT:
			return "SEP_CMD_ERR_TIMEOUT";
		case SEP_CMD_ERR_OPCODE_MISMATCH:
			return "SEP_CMD_ERR_OPCODE_MISMATCH";
		case SEP_CMD_ERR_CQP_COMPL_ERROR:
			return "SEP_CMD_ERR_CQP_COMPL_ERROR";
		case SEP_CMD_ERR_INVALID_VF_ID:
			return "SEP_CMD_ERR_INVALID_VF_ID";
		case SEP_CMD_ERR_INVALID_HMCFN_ID:
			return "SEP_CMD_ERR_INVALID_HMCFN_ID";
		case SEP_CMD_ERR_BACKING_PAGE_ERROR:
			return "SEP_CMD_ERR_BACKING_PAGE_ERROR";
		case SEP_CMD_ERR_NO_PBLCHUNKS_AVAILABLE:
			return "SEP_CMD_ERR_NO_PBLCHUNKS_AVAILABLE";
		case SEP_CMD_ERR_INVALID_PBLE_INDEX:
			return "SEP_CMD_ERR_INVALID_PBLE_INDEX";
		case SEP_CMD_ERR_INVALID_SD_INDEX:
			return "SEP_CMD_ERR_INVALID_SD_INDEX";
		case SEP_CMD_ERR_INVALID_PAGE_DESC_INDEX:
			return "SEP_CMD_ERR_INVALID_PAGE_DESC_INDEX";
		case SEP_CMD_ERR_INVALID_SD_TYPE:
			return "SEP_CMD_ERR_INVALID_SD_TYPE";
		case SEP_CMD_ERR_MEMCPY_FAILED:
			return "SEP_CMD_ERR_MEMCPY_FAILED";
		case SEP_CMD_ERR_INVALID_HMC_OBJ_INDEX:
			return "SEP_CMD_ERR_INVALID_HMC_OBJ_INDEX";
		case SEP_CMD_ERR_INVALID_HMC_OBJ_COUNT:
			return "SEP_CMD_ERR_INVALID_HMC_OBJ_COUNT";
		case SEP_CMD_ERR_INVALID_SRQ_ARM_LIMIT:
			return "SEP_CMD_ERR_INVALID_SRQ_ARM_LIMIT";
		case SEP_CMD_ERR_SRQ_ENABLED:
			return "SEP_CMD_ERR_SRQ_ENABLED";
		case SEP_CMD_ERR_ADMIN_QUEUE_ERROR:
			return "SEP_CMD_ERR_ADMIN_QUEUE_ERROR";
		case SEP_CMD_ERR_ADMIN_QUEUE_TIMEOUT:
			return "SEP_CMD_ERR_ADMIN_QUEUE_TIMEOUT";
		case SEP_CMD_ERR_BUF_TOO_SHORT:
			return "SEP_CMD_ERR_BUF_TOO_SHORT";
		case SEP_CMD_ERR_ADMIN_QUEUE_FULL:
			return "SEP_CMD_ERR_ADMIN_QUEUE_FULL";
		case SEP_CMD_ERR_ADMIN_QUEUE_NO_WORK:
			return "SEP_CMD_ERR_ADMIN_QUEUE_NO_WORK";
		case SEP_CMD_ERR_BAD_IWARP_CQE:
			return "SEP_CMD_ERR_BAD_IWARP_CQE";
		case SEP_CMD_ERR_NVM_BLANK_MODE:
			return "SEP_CMD_ERR_NVM_BLANK_MODE";
		case SEP_CMD_ERR_NOT_IMPLEMENTED:
			return "SEP_CMD_ERR_NOT_IMPLEMENTED";
		case SEP_CMD_ERR_PE_DOORBELL_NOT_ENABLED:
			return "SEP_CMD_ERR_PE_DOORBELL_NOT_ENABLED";
		case SEP_CMD_ERR_DIAG_TEST_FAILED:
			return "SEP_CMD_ERR_DIAG_TEST_FAILED";
		case SEP_CMD_ERR_NOT_READY:
			return "SEP_CMD_ERR_NOT_READY";
		case SEP_NOT_SUPPORTED:
			return "SEP_CMD_NOT_SUPPORTED";
		case SEP_CMD_ERR_FIRMWARE_API_VERSION:
			return "SEP_CMD_ERR_FIRMWARE_API_VERSION";
		case SEP_CMD_ERR_ADMIN_QUEUE_CRITICAL_ERROR:
			return "SEP_CMD_ERR_ADMIN_QUEUE_CRITICAL_ERROR";
	}

	return "SEP CMD : Un-Known Err";
}

sep_status sep_cmd_reset(struct sep_data *data)
{
	struct sep_cmd_desc desc = {0};
	sep_status status;

	sep_fill_default_direct_cmd_desc(&desc, sep_cmd_opc_reset);

	status = sep_send_command(data, &desc, NULL, 0, NULL);

	return status;
}

sep_status sep_cmd_fw_init(struct sep_data *data)
{
	struct sep_cmd_desc desc = {0};
	sep_status status;

	sep_fill_default_direct_cmd_desc(&desc, sep_cmd_opc_init);

	status = sep_send_command(data, &desc, NULL, 0, NULL);

	return status;
}

sep_status sep_cmd_intv_alloc(struct sep_data *data)
{
	struct sep_cmd_desc desc = {0};
	struct sep_cmd_intv_map_message intv;
	sep_status status;

	intv.txq = 0;
	intv.rxq = 0;
	intv.other = 1;

	sep_fill_default_direct_cmd_desc(&desc, sep_cmd_opc_intv_alloc);
	desc.flags |= cpu_to_le16((u16)(SEP_CMD_FLAG_BUF | SEP_CMD_FLAG_RD));

	status = sep_send_command(data, &desc, &intv, sizeof(intv), NULL);

	return status;
}

sep_status sep_clean_event(struct sep_data *data)
{
	sep_status ret_code = 0;
	struct sep_cmd_desc *desc;
	u16 flags;

	/* take the lock before we start messing with the ring */
	mutex_lock(&data->ei.cmd_mutex);

	if (data->ei.init == 0) {
		pr_err("AQRX: Admin queue not initialized.\n");
		ret_code = SEP_CMD_ERR_QUEUE_EMPTY;
		goto clean_event_err;
	}

	/* now clean the next descriptor */
	desc = (struct sep_cmd_desc *)PTR_ALIGN(data->ei.desc.va, 32);

	flags = le16_to_cpu(desc->flags);

	if (!ior32(FREG_EVENT_READY) || !(flags & SEP_CMD_FLAG_DD)) {
		ret_code = SEP_CMD_ERR_QUEUE_EMPTY;
		goto clean_event_err;
	}

	if (flags & SEP_CMD_FLAG_ERR) {
		ret_code = SEP_CMD_ERR_ADMIN_QUEUE_ERROR;
	}

	/* Restore the original datalen and buffer address in the desc,
	 * FW updates datalen to indicate the event message
	 * size
	 */
	memset((void *)desc, 0, sizeof(struct sep_cmd_desc));

	desc->flags = cpu_to_le16(SEP_CMD_FLAG_BUF);
	desc->datalen = cpu_to_le16((u16)data->ei.cmd_buf_size);
	desc->params.external.addr_high =
		cpu_to_le32(upper_32_bits(data->ei.indr_buf.pa));
	desc->params.external.addr_low =
		cpu_to_le32(lower_32_bits(data->ei.indr_buf.pa));

	iow32(FREG_EVENT_READY, 1);

clean_event_err:
	mutex_unlock(&data->ei.cmd_mutex);

	return ret_code;
}

bool sep_event_done(struct sep_data *data)
{
	struct sep_cmd_desc *desc;
	u16 flags;

	/* AQ designers suggest use of head for better
	 * timing reliability than DD bit
	 */
	desc = (struct sep_cmd_desc *)PTR_ALIGN(data->ei.desc.va, 32);
	flags = le16_to_cpu(desc->flags);
	if (flags & SEP_CMD_FLAG_DD) {
		return true;
	}

	return false;
}

static bool sep_cmd_done(struct sep_data *data)
{
	struct sep_cmd_desc *desc;
	u16 flags;

	/* AQ designers suggest use of head for better
	 * timing reliability than DD bit
	 */
	desc = (struct sep_cmd_desc *)PTR_ALIGN(data->ci.desc.va, 32);
	flags = le16_to_cpu(desc->flags);

	if (flags & SEP_CMD_FLAG_DD) {
		return true;
	}

	return false;
}

void sep_cmd_trigger_fw(struct sep_data *data)
{
	iow32(FREG_CMD_EXE, 1);

	pr_info(">>>CMD trigger\n");
}

sep_status sep_send_command(struct sep_data *data, struct sep_cmd_desc *desc,
			    void *buff, /* can be NULL */
			    u16 buff_size, struct sep_cmd_details *cmd_details)
{
	sep_status status = 0;
	struct sep_cmd_desc *desc_on_ring;
	bool cmd_completed = false;
	u16 retval = 0;
	u32 total_delay = 0;
	void *data_buf;

	mutex_lock(&data->ci.cmd_mutex);

	if (data->ci.init == 0) {
		pr_err("AQTX: Admin queue not initialized.\n");
		status = SEP_CMD_ERR_QUEUE_EMPTY;
		goto send_command_error;
	}

	data->ci.cmd_last_status = SEP_CMD_RC_OK;

	/* clear requested flags and then set additional flags if defined */
	if (cmd_details) {
		desc->flags &= ~cpu_to_le16(cmd_details->flags_dis);
		desc->flags |= cpu_to_le16(cmd_details->flags_ena);
	}

	if (buff_size > data->ci.cmd_buf_size) {
		pr_debug("AQTX: Invalid buffer size: %d.\n", buff_size);
		status = SEP_CMD_ERR_INVALID_SIZE;
		goto send_command_error;
	}

	/* if the desc is available copy the temp desc to the right place */
	desc_on_ring = PTR_ALIGN(data->ci.desc.va, 32);
	data_buf = PTR_ALIGN(data->ci.indr_buf.va, 32);
	*desc_on_ring = *desc;

	dev_info(&data->pdev->dev, "desc va %p 0x%16.16llx opcode 0x%x\n",
		 desc_on_ring, *((uint64_t *)(desc)), desc->opcode);
	dev_info(&data->pdev->dev, "data va %p\n", data_buf);
	/* if buff is not NULL assume indirect command */
	if (buff != NULL) {
		/* copy the user buff into the respective DMA buff */
		memcpy(data_buf, buff, buff_size);
		desc_on_ring->datalen = cpu_to_le16(buff_size);

		/* Update the address values in the desc with the pa value
		 * for respective buffer
		 */
		desc_on_ring->params.external.addr_high =
			cpu_to_le32(upper_32_bits(data->ci.indr_buf.pa));
		desc_on_ring->params.external.addr_low =
			cpu_to_le32(lower_32_bits(data->ci.indr_buf.pa));
	}

	/* bump the tail */
	pr_debug("AQTX: desc and buffer: 0x%llx %d\n", (u64)buff, buff_size);

	sep_cmd_trigger_fw(data);

	/* if cmd_details are not defined or async flag is not set,
	 * we need to wait for desc write back
	 */

	do {
		/* AQ designers suggest use of head for better
		 * timing reliability than DD bit
		 */
		if (sep_cmd_done(data))
			break;
		udelay(50);
		total_delay += 50;
	} while (total_delay < data->ci.cmd_timeout);

	/* if ready, copy the desc back to temp */
	if (sep_cmd_done(data)) {
		*desc = *desc_on_ring;
		if (buff != NULL)
			memcpy(buff, data_buf, buff_size);
		retval = le16_to_cpu(desc->retval);
		if (retval != 0) {
			pr_debug("AQTX: Command completed with error 0x%X.\n",
				 retval);

			/* strip off FW internal code */
			retval &= 0xff;
		}
		cmd_completed = true;
		if ((enum sep_cmd_err)retval == SEP_CMD_RC_OK)
			status = 0;
		else if ((enum sep_cmd_err)retval == SEP_CMD_RC_EBUSY)
			status = SEP_CMD_ERR_NOT_READY;
		else
			status = SEP_CMD_ERR_ADMIN_QUEUE_ERROR;
		data->ci.cmd_last_status = retval;
	}

	pr_debug("AQTX: desc and buffer writeback: %llx %d \n", (u64)buff,
		 buff_size);

	/* update the error if time out occurred */
	if ((!cmd_completed)) {
		pr_info("AQTX: Writeback timeout.\n");
		status = SEP_CMD_ERR_ADMIN_QUEUE_TIMEOUT;
	}

send_command_error:
	mutex_unlock(&data->ci.cmd_mutex);
	return status;
}

void sep_fill_default_direct_cmd_desc(struct sep_cmd_desc *desc, u16 opcode)
{
	/* zero out the desc */
	memset((void *)desc, 0, sizeof(struct sep_cmd_desc));
	desc->opcode = cpu_to_le16(opcode);
	desc->flags = cpu_to_le16(SEP_CMD_FLAG_SI);
}

int sep_allocate_cmd_dma_mem(struct sep_data *data, struct sep_dma_mem *mem,
			     u64 size, u32 alignment)
{

	mem->size = ALIGN(size, alignment);
	mem->va = dma_alloc_coherent(&data->pdev->dev, mem->size, &mem->pa,
				     GFP_KERNEL);
	if (!mem->va)
		return -ENOMEM;

	dev_info(&data->pdev->dev, "cmd vaddr = 0x%llx paddr = 0x%llx\n",
		 (u64)mem->va, (u64)mem->pa);
	return 0;
}

int sep_free_cmd_dma_mem(struct sep_data *data, struct sep_dma_mem *mem)
{

	pr_info("free cmd vaddr = 0x%llx paddr = 0x%llx\n", (u64)mem->va,
		(u64)mem->pa);

	dma_free_coherent(&data->pdev->dev, mem->size, mem->va, mem->pa);
	mem->va = NULL;
	mem->pa = 0;
	mem->size = 0;

	return 0;
}

static sep_status sep_alloc_cmd_desc(struct sep_data *data)
{
	sep_status ret_code;

	ret_code = sep_allocate_cmd_dma_mem(data, &data->ci.desc,
					    sizeof(struct sep_cmd_desc),
					    SEP_CMD_ALIGNMENT);

	return ret_code;
}

void sep_free_cmd_desc(struct sep_data *data)
{
	sep_free_cmd_dma_mem(data, &data->ci.desc);
}

static sep_status sep_alloc_cmd_bufs(struct sep_data *data)
{
	sep_status ret_code;

	/* allocate the mapped buffers */
	ret_code = sep_allocate_cmd_dma_mem(data, &data->ci.indr_buf,
					    data->ci.cmd_buf_size,
					    SEP_CMD_ALIGNMENT);

	return ret_code;
}

static void sep_free_cmd_bufs(struct sep_data *data)
{
	/* only unmap if the address is non-NULL */
	sep_free_cmd_dma_mem(data, &data->ci.indr_buf);
}

static sep_status sep_alloc_event_bufs(struct sep_data *data)
{
	sep_status ret_code;
	struct sep_cmd_desc *desc;

	ret_code = sep_allocate_cmd_dma_mem(data, &data->ei.indr_buf,
					    data->ei.cmd_buf_size,
					    SEP_CMD_ALIGNMENT);
	if (ret_code)
		goto unwind_alloc_event_bufs;

	desc = (struct sep_cmd_desc *)PTR_ALIGN(data->ei.desc.va, 32);
	desc->flags = cpu_to_le16(SEP_CMD_FLAG_BUF);
	desc->opcode = 0;
	/* This is in accordance with Admin queue design, there is no
	 * register for buffer size configuration
	 */
	desc->datalen = cpu_to_le16((u16)data->ei.cmd_buf_size);
	desc->retval = 0;
	desc->params.external.addr_high =
		cpu_to_le32(upper_32_bits(data->ei.indr_buf.pa));
	desc->params.external.addr_low =
		cpu_to_le32(lower_32_bits(data->ei.indr_buf.pa));
	desc->params.external.param0 = 0;
	desc->params.external.param1 = 0;

unwind_alloc_event_bufs:
	return ret_code;
}

static void sep_free_event_bufs(struct sep_data *data)
{
	sep_free_cmd_dma_mem(data, &data->ei.indr_buf);
}

static sep_status sep_alloc_event_desc(struct sep_data *data)
{
	sep_status ret_code;

	ret_code = sep_allocate_cmd_dma_mem(data, &data->ei.desc,
					    sizeof(struct sep_cmd_desc),
					    SEP_CMD_ALIGNMENT);

	return ret_code;
}

static void sep_free_event_desc(struct sep_data *data)
{
	sep_free_cmd_dma_mem(data, &data->ei.desc);
}

sep_status sep_init_cmd(struct sep_data *data)
{
	sep_status ret_code = 0;

	if (data->ci.init > 0) {
		/* queue already initialized */
		ret_code = SEP_CMD_ERR_NOT_READY;
		goto init_exit;
	}

	ret_code = sep_alloc_cmd_desc(data);
	if (ret_code)
		goto init_exit;

	ret_code = sep_alloc_cmd_bufs(data);
	if (ret_code)
		goto init_free_desc;

	iow32(FREG_CMD_ADDR_L, lower_32_bits(data->ci.desc.pa));
	iow32(FREG_CMD_ADDR_H, upper_32_bits(data->ci.desc.pa));

	/* success! */
	data->ci.init = 1;
	goto init_exit;

init_free_desc:
	sep_free_cmd_desc(data);
	data->ci.init = 0;
init_exit:
	return ret_code;
}

sep_status sep_deinit_cmd(struct sep_data *data)
{
	sep_status ret_code = 0;

	if (data->ci.init < 1) {
		/* queue already initialized */
		ret_code = SEP_CMD_ERR_NOT_READY;
		goto deinit_exit;
	}

	sep_free_cmd_desc(data);
	sep_free_cmd_bufs(data);

	/* success! */
	data->ci.init = 0;

deinit_exit:
	return ret_code;
}

sep_status sep_init_event(struct sep_data *data)
{
	sep_status ret_code = 0;

	if (data->ei.init > 0) {
		/* queue already initialized */
		ret_code = SEP_CMD_ERR_NOT_READY;
		goto init_event_exit;
	}

	/* allocate the ring memory */
	ret_code = sep_alloc_event_desc(data);
	if (ret_code)
		goto init_event_exit;

	/* allocate buffers in the rings */
	ret_code = sep_alloc_event_bufs(data);
	if (ret_code)
		goto init_event_free_desc;

	iow32(FREG_EVENT_ADDR_L, lower_32_bits(data->ei.desc.pa));
	iow32(FREG_EVENT_ADDR_H, upper_32_bits(data->ei.desc.pa));
	iow32(FREG_EVENT_READY, 1);

	/* success! */
	data->ei.init = 1;
	goto init_event_exit;

init_event_free_desc:
	sep_free_event_desc(data);
	data->ei.init = 0;
init_event_exit:
	return ret_code;
}


sep_status sep_deinit_event(struct sep_data *data)
{
	sep_status ret_code = 0;

	if (data->ei.init < 1) {
		/* queue already initialized */
		ret_code = SEP_CMD_ERR_NOT_READY;
		goto deinit_exit;
	}

	sep_free_event_desc(data);
	sep_free_event_bufs(data);

	/* success! */
	data->ei.init = 0;

deinit_exit:
	return ret_code;
}
