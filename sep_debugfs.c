// SPDX-License-Identifier: GPL-2.0
/*
 * This is the driver for the japan card for usa SoCs.
 *
 * Copyright (C) 2021, usa MicroElectronic Technology - All Rights Reserved
 */

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/mii.h>

#include "sep_main.h"

static struct dentry *sep_debugfs_root;

static char sep_dbg_reg_ops_buf[256] = "";
static char sep_dbg_sepdev_ops_buf[256] = "";

struct usa_gmac_reg_info {
	u32 ofs;
	char *name;
};

static const struct usa_gmac_reg_info usa_gmac_reg_info_tbl[] = {
	{GMAC_REG_MAC_CTRL, "MAC_CTRL"},
	{GMAC_REG_FDPX_FC, "FDPX_FC"},
	{GMAC_REG_DMA_BURST_LEN, "DMA_BURST_LEN"},
	{GMAC_REG_DMA_FIFO_STS, "DMA_FIFO_STS"},
	{GMAC_REG_WOL_CTRL, "WOL_CTRL"},
	{GMAC_REG_WOL_STS, "WOL_STS"},
	{GMAC_REG_PTP_RX_UCST_DA, "PTP_RX_UCST_DA"},
	{GMAC_REG_PTP_TX_UCST_DA, "PTP_TX_UCST_DA"},
	{GMAC_REG_PTP_TX_PSEC, "PTP_TX_PSEC"},
	{GMAC_REG_PTP_TX_PNSEC, "PTP_TX_PNSEC"},
	{GMAC_REG_PTP_RX_PSEC, "PTP_RX_PSEC"},
	{GMAC_REG_PTP_RX_PNSEC, "PTP_RX_PNSEC"},
	{GMAC_REG_PTP_TX_P_PSEC, "PTP_TX_P_PSEC"},
	{GMAC_REG_PTP_TX_P_PNSEC, "PTP_TX_P_PNSEC"},
	{GMAC_REG_PTP_RX_P_PSEC, "PTP_RX_P_PSEC"},
	{GMAC_REG_PTP_RX_P_PNSEC, "PTP_RX_P_PNSEC"},
	{GMAC_REG_PTP_TMRNN, "PTP_TMRNN"},
	{GMAC_REG_PTP_TMRN, "PTP_TMRN"},
	{GMAC_REG_PTP_TMRS, "PTP_TMRS"},
	{GMAC_REG_PTP_PER_NS, "PTP_PER_NS"},
	{GMAC_REG_PTP_PER_NNS, "PTP_PER_NNS"},
	{GMAC_REG_PTP_OFFSET, "PTP_OFFSET"},
	{GMAC_REG_PTP_ADJ, "PTP_ADJ"},
	{GMAC_REG_AXI_CTRL, "AXI_CTRL"},
	{GMAC_REG_PHY_IF, "PHY_IF"},
	{GMAC_REG_SW_RST_CYCLE_CNT, "SW_RST_CYCLE_CNT"},
	{GMAC_REG_EEE_CTRL, "EEE_CTRL"},
	{GMAC_REG_REV, "REV"},
	{GMAC_REG_HW_FEATURE, "HW_FEATURE"},
	{GMAC_REG_BMCST_THR_CTRL, "BMCST_THR_CTRL"},
	{GMAC_REG_PHY_CTRL, "PHY_CTRL"},
	{GMAC_REG_PHY_DATA, "PHY_DATA"},
	{GMAC_REG_SYS_ISR, "SYS_ISR"},
	{GMAC_REG_SYS_IE, "SYS_IE"},
	{GMAC_REG_MAC_ADDR_L, "MAC_ADDR_L"},
	{GMAC_REG_MAC_ADDR_H, "MAC_ADDR_H"},
	{GMAC_REG_MCST_ADDR_HASH_TBL0, "MCST_ADDR_HASH_TBL0"},
	{GMAC_REG_MCST_ADDR_HASH_TBL1, "MCST_ADDR_HASH_TBL1"},
	{GMAC_REG_L2_FLT0_ADDR_L, "L2_FLT0_ADDR_L"},
	{GMAC_REG_L2_FLT0_ADDR_H, "L2_FLT0_ADDR_H"},
	{GMAC_REG_L2_FLT1_ADDR_L, "L2_FLT1_ADDR_L"},
	{GMAC_REG_L2_FLT1_ADDR_H, "L2_FLT1_ADDR_H"},
	{GMAC_REG_L2_FLT2_ADDR_L, "L2_FLT2_ADDR_L"},
	{GMAC_REG_L2_FLT2_ADDR_H, "L2_FLT2_ADDR_H"},
	{GMAC_REG_L2_FLT3_ADDR_L, "L2_FLT3_ADDR_L"},
	{GMAC_REG_L2_FLT3_ADDR_H, "L2_FLT3_ADDR_H"},
	{GMAC_REG_L2_FLT4_ADDR_L, "L2_FLT4_ADDR_L"},
	{GMAC_REG_L2_FLT4_ADDR_H, "L2_FLT4_ADDR_H"},
	{GMAC_REG_L2_FLT5_ADDR_L, "L2_FLT5_ADDR_L"},
	{GMAC_REG_L2_FLT5_ADDR_H, "L2_FLT5_ADDR_H"},
	{GMAC_REG_L2_FLT6_ADDR_L, "L2_FLT6_ADDR_L"},
	{GMAC_REG_L2_FLT6_ADDR_H, "L2_FLT6_ADDR_H"},
	{GMAC_REG_L2_FLT7_ADDR_L, "L2_FLT7_ADDR_L"},
	{GMAC_REG_L2_FLT7_ADDR_H, "L2_FLT7_ADDR_H"},
	{GMAC_REG_RSSC, "RSSC"},
	{GMAC_REG_RSS_RK0, "RSS_RK0"},
	{GMAC_REG_RSS_RK1, "RSS_RK1"},
	{GMAC_REG_RSS_RK2, "RSS_RK2"},
	{GMAC_REG_RSS_RK3, "RSS_RK3"},
	{GMAC_REG_RSS_RK4, "RSS_RK4"},
	{GMAC_REG_RSS_RK5, "RSS_RK5"},
	{GMAC_REG_RSS_RK6, "RSS_RK6"},
	{GMAC_REG_RSS_RK7, "RSS_RK7"},
	{GMAC_REG_RSS_RK8, "RSS_RK8"},
	{GMAC_REG_RSS_RK9, "RSS_RK9"},
	{GMAC_REG_TQ_WB_CTRL, "TQ_WB_CTRL"},
	{GMAC_REG_RQ_WB_CTRL, "RQ_WB_CTRL"},

	{GMAC_REG_QPAIR_CTRL, "QPAIR_CTRL"},

	{GMAC_REG_QUEUE_ISR, "QUEUE_ISR"},
	{GMAC_REG_QUEUE_IER, "QUEUE_IER"},

	{GMAC_REG_TX_ITC, "TX_ITC"},
	{GMAC_REG_RX_ITC, "RX_ITC"},

	{GMAC_REG_RX_DESC_CTRL, "RX_DESC_CTRL"},
	{GMAC_REG_RX_RING_BASE_L, "RX_RING_BASE_L"},
	{GMAC_REG_RX_RING_BASE_H, "RX_RING_BASE_H"},
	{GMAC_REG_RX_RING_HEAD, "RX_RING_HEAD"},
	{GMAC_REG_RX_RING_TAIL, "RX_RING_TAIL"},


	{GMAC_REG_TX_DESC_CTRL, "TX_DESC_CTRL"},
	{GMAC_REG_TX_RING_BASE_L, "TX_RING_BASE_L"},
	{GMAC_REG_TX_RING_BASE_H, "TX_RING_BASE_H"},
	{GMAC_REG_TX_RING_HEAD, "TX_RING_HEAD"},
	{GMAC_REG_TX_RING_TAIL, "TX_RING_TAIL"},


	{GMAC_REG_TPH_CTRL, "TPH_CTRL"},
	{GMAC_REG_FUNC_ID, "FUNC_ID"},
	{GMAC_REG_TQ_BUF_CTRL, "TQ_BUF_CTRL"},
	{GMAC_REG_RQ_BUF_CTRL, "RQ_BUF_CTRL"},
	{GMAC_REG_RQ_BUF_SZ, "RQ_BUF_SZ"},
#if defined(USING_TX_FILTER) && (USING_TX_FILTER == 1)
	{FREG_TSTL, "FREG_TSTL"},
	{FREG_TSTH, "FREG_TSTH"},
	{FREG_TSDL, "FREG_TSDL"},
	{FREG_TSDH, "FREG_TSDH"},
	{FREG_DBG, "FREG_DBG"},
#endif
	/* List Terminator */
	{0, NULL},
};


static void seq_gmac_dump(struct sep_data *data)
{
	u32 i;
	struct usa_gmac_reg_info *reginfo;
	struct gmac_tx_desc *tx_desc;
	struct gmac_rx_desc *rx_desc;
	struct gmac_buffer_info *buf_info;

	if (data->pdev)
		dev_info(&data->pdev->dev, "japanCard Device Info");

	/* Print Registers */
	dev_info(&data->pdev->dev, "Register Dump\n");
	pr_info("%-20s Name Value\n", "Register");
	pr_info("-----------------------------------");
	for (reginfo = (struct usa_gmac_reg_info *)usa_gmac_reg_info_tbl;
	     reginfo->name; reginfo++) {
		if (reginfo->name)
			pr_info("%-20s[%4.4x] %8.8x", reginfo->name,
				reginfo->ofs, ior32(reginfo->ofs));
	}

	/* Print Tx Ring Summary */
	dev_info(&data->pdev->dev, "Tx Ring Summary\n");
	pr_info("[NTU] [NTC] [ntc->dma] [ head ]\n");
	pr_info("%5.5d %5.5d 0x%16.16llx %5.5d\n", data->tx_ring.ntu,
		data->tx_ring.ntc,
		le64_to_cpu(data->tx_ring.buf_info[data->tx_ring.ntc].dma),
		ior32(GMAC_REG_TX_RING_HEAD));

	dev_info(&data->pdev->dev, "Tx Ring Dump\n");
	for (i = 0; i < data->tx_ring.count; i++) {
		const char *next_desc;
		tx_desc = &((struct gmac_tx_desc *)data->tx_ring.desc)[i];
		buf_info = &data->tx_ring.buf_info[i];

		if (i == data->tx_ring.ntu && i == data->tx_ring.ntc)
			next_desc = " NTC/U";
		else if (i == data->tx_ring.ntu)
			next_desc = " NTU";
		else if (i == data->tx_ring.ntc)
			next_desc = " NTC";
		else
			next_desc = "";
		pr_info("[%4.4d-0x%16.16llx] %8.8x %8.8x %8.8x %8.8x "
			"%8.8x %8.8x %8.8x %8.8x\t"
			"%16.16llx\t%s\n",
			i,
			(data->tx_ring.dma + i * sizeof(struct gmac_tx_desc)),
			le32_to_cpu(tx_desc->txdes0),
			le32_to_cpu(tx_desc->txdes1),
			le32_to_cpu(tx_desc->txdes2),
			le32_to_cpu(tx_desc->txdes3),
			le32_to_cpu(tx_desc->txdes4),
			le32_to_cpu(tx_desc->txdes5),
			le32_to_cpu(tx_desc->txdes6),
			le32_to_cpu(tx_desc->txdes7),
			le64_to_cpu(buf_info->dma), next_desc);
	}
	/* Print Rx Ring Summary */
	dev_info(&data->pdev->dev, "Rx Ring Summary\n");
	pr_info("[NTU] [NTC] [ntc->dma] [ head ]\n");
	pr_info("%5.5d %5.5d 0x%16.16llx %5.5d\n", data->rx_ring.ntu,
		data->rx_ring.ntc,
		le64_to_cpu(data->rx_ring.buf_info[data->rx_ring.ntc].dma),
		ior32(GMAC_REG_RX_RING_HEAD));

	dev_info(&data->pdev->dev, "Rx Ring Dump\n");
	for (i = 0; i < data->rx_ring.count; i++) {
		const char *next_desc;
		rx_desc = &((struct gmac_rx_desc *)data->rx_ring.desc)[i];
		buf_info = &data->rx_ring.buf_info[i];

		if (i == data->rx_ring.ntu && i == data->rx_ring.ntc)
			next_desc = " NTC/U";
		else if (i == data->rx_ring.ntu)
			next_desc = " NTU";
		else if (i == data->rx_ring.ntc)
			next_desc = " NTC";
		else
			next_desc = "";
		pr_info("[%4.4d-0x%16.16llx] %8.8x %8.8x %8.8x %8.8x "
			"%8.8x %8.8x %8.8x %8.8x\t"
			"%16.16llx\t%s\n",
			i,
			(data->rx_ring.dma + i * sizeof(struct gmac_rx_desc)),
			le32_to_cpu(rx_desc->rxdes0),
			le32_to_cpu(rx_desc->rxdes1),
			le32_to_cpu(rx_desc->rxdes2),
			le32_to_cpu(rx_desc->rxdes3),
			le32_to_cpu(rx_desc->rxdes4),
			le32_to_cpu(rx_desc->rxdes5),
			le32_to_cpu(rx_desc->rxdes6),
			le32_to_cpu(rx_desc->rxdes7),
			le64_to_cpu(buf_info->dma), next_desc);
	}
}

static ssize_t seq_dbg_common_ops_read(struct file *filp, char __user *buffer,
				       size_t count, loff_t *ppos,
				       char *dbg_buf)
{
	struct sep_data *data = filp->private_data;
	char *buf;
	int len;

	/* don't allow partial reads */
	if (*ppos != 0)
		return 0;

	buf = kasprintf(GFP_KERNEL, "%s: %s\n", pci_name(data->pdev), dbg_buf);
	if (!buf)
		return -ENOMEM;

	if (count < strlen(buf)) {
		kfree(buf);
		return -ENOSPC;
	}

	len = simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));

	kfree(buf);

	return len;
}

/**
 * sep_dbg_reg_ops_read - read for reg_ops datum
 * @filp: the opened file
 * @buffer: where to write the data for the user to read
 * @count: the size of the user's buffer
 * @ppos: file position offset
 **/
static ssize_t seq_dbg_reg_ops_read(struct file *filp, char __user *buffer,
				    size_t count, loff_t *ppos)
{
	return seq_dbg_common_ops_read(filp, buffer, count, ppos,
				       sep_dbg_reg_ops_buf);
}

/**
 * sep_dbg_reg_ops_write - write into reg_ops datum
 * @filp: the opened file
 * @buffer: where to find the user's data
 * @count: the length of the user's data
 * @ppos: file position offset
 **/
static ssize_t seq_dbg_reg_ops_write(struct file *filp,
				     const char __user *buffer, size_t count,
				     loff_t *ppos)
{
	struct sep_data *data = filp->private_data;
	int len;

	/* don't allow partial writes */
	if (*ppos != 0)
		return 0;
	if (count >= sizeof(sep_dbg_reg_ops_buf))
		return -ENOSPC;

	len = simple_write_to_buffer(sep_dbg_reg_ops_buf,
				     sizeof(sep_dbg_reg_ops_buf) - 1, ppos,
				     buffer, count);
	if (len < 0)
		return len;

	sep_dbg_reg_ops_buf[len] = '\0';

	if (strncmp(sep_dbg_reg_ops_buf, "write", 5) == 0) {
		u32 reg, value;
		int cnt;
		cnt = sscanf(&sep_dbg_reg_ops_buf[5], "%x %x", &reg, &value);
		if ((cnt == 2) && (reg >= GMAC_REG_MAC_CTRL) &&
		    (reg <= FREG_DBG)) {
			iowrite32(value, data->io_base + reg);
			ioread32(data->io_base + reg);
			pr_info("write: 0x%08x = 0x%08x\n", reg, value);
		} else {
			pr_info("write <reg> <value>\n");
		}
	} else if (strncmp(sep_dbg_reg_ops_buf, "read", 4) == 0) {
		u32 reg, value;
		int cnt;
		cnt = sscanf(&sep_dbg_reg_ops_buf[4], "%x", &reg);
		if ((cnt == 1) && (reg >= GMAC_REG_MAC_CTRL) &&
		    (reg <= FREG_DBG)) {
			value = ioread32(data->io_base + reg);
			pr_info("read 0x%08x = 0x%08x\n", reg, value);
		} else {
			pr_info("read error reg range 0x%x ~ 0x%x\n",
				GMAC_REG_MAC_CTRL, FREG_DBG);
		}
	} else if (strncmp(sep_dbg_reg_ops_buf, "dump", 4) == 0) {
		seq_gmac_dump(data);
	} else if (strncmp(sep_dbg_reg_ops_buf, "statistic", 4) == 0) {
#if defined(USING_TX_FILTER) && (USING_TX_FILTER == 1)
		pr_info("**** japan card statistic ****\n");
		pr_info("tx ok %lld\n",
			(u64)ioread32(data->io_base + FREG_TSTL) +
				((u64)ioread32(data->io_base + FREG_TSTH)
				 << 32));
		pr_info("tx drop %lld\n",
			(u64)ioread32(data->io_base + FREG_TSDL) +
				((u64)ioread32(data->io_base + FREG_TSDH)
				 << 32));
		pr_info("***********************************\n");
#else
		pr_info("not support!!!\n")
#endif
	} else {
		pr_info("Unknown command %s\n", sep_dbg_reg_ops_buf);
		pr_info("Available commands:\n");
		pr_info("\tread <reg>          - read register\n");
		pr_info("\twrite <reg> <value> - write register\n");
		pr_info("\tdump                - dump register and ring "
			"info\n");
		pr_info("\tstatistic           - get tx statistic\n");
	}
	return count;
}

static const struct file_operations seq_dbg_reg_ops_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = seq_dbg_reg_ops_read,
	.write = seq_dbg_reg_ops_write,
};

/**
 * sep_dbg_sepdev_ops_read - read for sepdev_ops datum
 * @filp: the opened file
 * @buffer: where to write the data for the user to read
 * @count: the size of the user's buffer
 * @ppos: file position offset
 **/
static ssize_t seq_dbg_sepdev_ops_read(struct file *filp, char __user *buffer,
				       size_t count, loff_t *ppos)
{
	return seq_dbg_common_ops_read(filp, buffer, count, ppos,
				       sep_dbg_sepdev_ops_buf);
}

/**
 * sep_dbg_sepdev_ops_write - write into sepdev_ops datum
 * @filp: the opened file
 * @buffer: where to find the user's data
 * @count: the length of the user's data
 * @ppos: file position offset
 **/
static ssize_t seq_dbg_sepdev_ops_write(struct file *filp,
					const char __user *buffer, size_t count,
					loff_t *ppos)
{
	struct sep_data *data = filp->private_data;
	int len;
	u32 i;

	/* don't allow partial writes */
	if (*ppos != 0)
		return 0;
	if (count >= sizeof(sep_dbg_sepdev_ops_buf))
		return -ENOSPC;

	len = simple_write_to_buffer(sep_dbg_sepdev_ops_buf,
				     sizeof(sep_dbg_sepdev_ops_buf) - 1, ppos,
				     buffer, count);
	if (len < 0)
		return len;

	sep_dbg_sepdev_ops_buf[len] = '\0';

	if (strncmp(sep_dbg_sepdev_ops_buf, "link", 4) == 0) {
		gphy_get_link_status(data);
		if (data->phy_link) {
			dev_info(&data->pdev->dev, "Link up\n");
		} else {
			dev_info(&data->pdev->dev, "Link down\n");
			for (i = 0; i < MII_ESTATUS; i++)
				pr_info("%2.2d-0x%8.8x\n", i,
					gmac_mdio_read(data, i));
		}
	} else if (strncmp(sep_dbg_sepdev_ops_buf, "read", 4) == 0) {
		u32 reg, value;
		int cnt;
		cnt = sscanf(&sep_dbg_sepdev_ops_buf[4], "%x", &reg);
		if ((cnt == 1) && (reg >= MII_BMCR) && (reg <= MII_NCONFIG)) {
			value = gmac_mdio_read(data, reg);
			pr_info("read 0x%08x = 0x%08x\n", reg, value);
		} else {
			pr_info("read error reg range 0x%x ~ 0x%x\n", MII_BMCR,
				MII_NCONFIG);
		}
	} else {
		pr_info("Unknown command %s\n", sep_dbg_sepdev_ops_buf);
		pr_info("Available commands:\n");
		pr_info("\tlink       - get link status\n");
		pr_info("\tread <reg> - read register\n");
	}

	return count;
}

static const struct file_operations seq_dbg_dev_ops_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = seq_dbg_sepdev_ops_read,
	.write = seq_dbg_sepdev_ops_write,
};

int seq_dbg_adapter_init(struct sep_data *data)
{
	struct dentry *d;
	int ret = 0;

	d = debugfs_create_dir(pci_name(data->pdev), sep_debugfs_root);
	if (!d) {
		dev_err(&data->pdev->dev, "create debugfs file fail!\n");
		return -ENOMEM;
	}
	data->dbgfs = d;

	d = debugfs_create_file("reg_ops", 0600, data->dbgfs, data,
				&seq_dbg_reg_ops_fops);
	if (!d) {
		ret = -ENOMEM;
		goto err;
	}

	d = debugfs_create_file("dev_ops", 0600, data->dbgfs, data,
				&seq_dbg_dev_ops_fops);
	if (!d) {
		ret = -ENOMEM;
		goto err;
	}

	return 0;
err:
	debugfs_remove_recursive(data->dbgfs);
	return ret;
}


void seq_dbg_adapter_exit(struct sep_data *data)
{
	debugfs_remove_recursive(data->dbgfs);
	data->dbgfs = NULL;
}

int seq_dbgfs_init(const char *name)
{
	sep_debugfs_root = debugfs_create_dir(name, NULL);

	if (!sep_debugfs_root)
		return -ENOMEM;

	return 0;
}

void seq_dbgfs_exit(void)
{
	debugfs_remove_recursive(sep_debugfs_root);
}
