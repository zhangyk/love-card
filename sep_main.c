// SPDX-License-Identifier: GPL-2.0
/*
 * This is the driver for the japan card for usa SoCs.
 *
 * Copyright (C) 2021, usa MicroElectronic Technology - All Rights Reserved
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/file.h>
#include <linux/mii.h>
#include <linux/mdio.h>
#include <linux/phy.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/etherdevice.h>
#include <linux/workqueue.h>
#include <linux/mm.h>
#include "config.h"
#include "gmac_regmap.h"
#include "sep_main.h"

static int major; /* default to dynamic major */
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

#define GMAC_TX_DESC(B, i) (&(((struct gmac_tx_desc *)(B))[i]))
#define GMAC_RX_DESC(B, i) (&(((struct gmac_rx_desc *)(B))[i]))

#define SEP_DMA_ATTR (DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING)

/* sep_pci_tbl - PCI Device ID Table
 *
 * Wildcard entries (PCI_ANY_ID) should come last
 * Last entry must be all 0s
 *
 *  Vendor ID
 *  Device ID
 *  SubVendor ID
 *  SubDevice ID
 *  Class
 *  Class Mask
 * private data (not used)
 */
static const struct pci_device_id sep_pci_tbl[] = {
#if USING_PF0
	{
		vendor : 0x1556,
		device : 0x1111,
		subvendor : PCI_ANY_ID,
		subdevice : PCI_ANY_ID,
		class : 0,
		class_mask : 0,
	},
#else
	{
		vendor : 0x1556,
		device : 0x2222,
		subvendor : PCI_ANY_ID,
		subdevice : PCI_ANY_ID,
		class : 0,
		class_mask : 0,
	},
#endif
	/* required last entry */
	{
		0,
	},
};
MODULE_DEVICE_TABLE(pci, sep_pci_tbl);

/**
 * @brief  gmac_get_tx_free_num
 *
 * @param  data
 * @retval tx free descriptor number
 */
static inline u16 gmac_get_tx_free_num(struct sep_data *data)
{
	struct gmac_ring *tx = &data->tx_ring;

	rmb();
	// return (tx->count - ((tx->count - ior32(GMAC_REG_TX_RING_HEAD) + tx->ntu) & (tx->count - 1)) - 1);
	return ((tx->count + ior32(GMAC_REG_TX_RING_HEAD) - tx->ntu) &
		(tx->count - 1)) -
	       1;
}

/**
 * @brief  gmac_received_pack_num
 *
 * @param  data
 * @retval rx received pkt number
 */
static inline u16 gmac_received_pack_num(struct sep_data *data)
{
	struct gmac_ring *rx = &data->rx_ring;

	return (((rx->count + ior32(GMAC_REG_RX_RING_HEAD) - rx->ntc) &
		 (rx->count - 1)) -
		1);
}

/**
 * @brief  gmac_mdio_read
 *
 * @param  data   : private data
 * @param  regnum : PHY register index
 *
 * @retval The PHY register value
 */
u16 gmac_mdio_read(struct sep_data *data, s32 regnum)
{
	u32 hwaddr, cmd;
	s32 phy;

	hwaddr = GMAC_PHYCR_PHYAD(data->phy_addr) | GMAC_PHYCR_REGAD(regnum);

	cmd = hwaddr | GMAC_PHYCR_PHYRD | GMAC_PHYCR_OP(GMAC_MDIO_OP_RD) |
	      GMAC_PHYCR_SOF(GMAC_MDIO_SOF) |
	      GMAC_PHYCR_MDC_CYCTHR(data->mdc_cycthr);

	iow32(GMAC_REG_PHY_CTRL, cmd);

	if (readx_poll_timeout(ior32, GMAC_REG_PHY_CTRL, cmd,
			       !(cmd & GMAC_PHYCR_PHYRD), 100,
			       10 * SEP_MDIO_COMMAND_TIMEOUT))
		dev_err(&data->pdev->dev, "read phy reg fail reg %d\n", regnum);

	phy = ior32(GMAC_REG_PHY_DATA);

	return GMAC_PHYDATA_RDATA(phy);
}

/**
 * @brief  gmac_mdio_write
 *
 * @param data   : private data
 * @param regnum : PHY register index
 * @param val    : The register value that want to set
 *
 * @retval 0 on success, negative on failure
 */
s32 gmac_mdio_write(struct sep_data *data, s32 regnum, u16 val)
{

	u32 hwaddr, cmd;
	s32 err;

	iow32(GMAC_REG_PHY_DATA, (u32)(GMAC_PHYDATA_WDATA(val)));

	hwaddr = GMAC_PHYCR_PHYAD(data->phy_addr) | GMAC_PHYCR_REGAD(regnum);

	cmd = hwaddr | GMAC_PHYCR_PHYWR | GMAC_PHYCR_OP(GMAC_MDIO_OP_WR) |
	      GMAC_PHYCR_SOF(GMAC_MDIO_SOF) |
	      GMAC_PHYCR_MDC_CYCTHR(data->mdc_cycthr);

	iow32(GMAC_REG_PHY_CTRL, cmd);

	err = readx_poll_timeout(ior32, GMAC_REG_PHY_CTRL, cmd,
				 !(cmd & GMAC_PHYCR_PHYWR), 100,
				 10 * SEP_MDIO_COMMAND_TIMEOUT);

	if (err)
		dev_err(&data->pdev->dev, "write phy reg fail reg %d\n",
			regnum);
	return err;
}

/**
 * @brief  gmac_sw_reset
 *
 * @param  data : private data
 *
 * @retval 0 on success, negative on failure
 */
int gmac_sw_reset(struct sep_data *data)
{
	u32 gmaccr, i;

	gmaccr = ior32(GMAC_REG_MAC_CTRL) | GMAC_CTRL_SW_RST;
	iow32(GMAC_REG_MAC_CTRL, gmaccr);

	for (i = 0; i < 5; i++) {
		gmaccr = ior32(GMAC_REG_MAC_CTRL);
		if (!(gmaccr & GMAC_CTRL_SW_RST))
			return 0;

		udelay(1000);
	}

	dev_err(&data->pdev->dev, "Software reset failed\n");

	return -EIO;
}

/**
 * @brief  gmac l2 filter config
 *
 * @param  data : private data
 *
 * @retval None
 */
void gmac_mac_filter(struct sep_data *data)
{
	u32 mac = 0;
	mac = ((u32)data->mac_addr[5] | ((u32)data->mac_addr[4] << 8) |
	       ((u32)data->mac_addr[3] << 16) | ((u32)data->mac_addr[2] << 24));
	iow32(GMAC_L2_FLT_ADDR_L(0), mac);

	mac = 0;
	mac = ((u32)data->mac_addr[1] | ((u32)data->mac_addr[0] << 8) |
	       GMAC_L2_FLT_EN | GMAC_L2_FLT_DA);
	iow32(GMAC_L2_FLT_ADDR_H(0), mac);
}

/**
 * @brief  set gmac mac address
 *
 * @param data : private data
 *
 * @retval 0 on success, negative on failure
 */
int gmac_address_config(struct sep_data *data)
{
	u8 addr[6];

	random_ether_addr(data->mac_addr);
	if (!is_valid_ether_addr(data->mac_addr)) {
		dev_err(&data->pdev->dev, "invalid MAC address\n");
		return -EIO;
	}

	dev_info(&data->pdev->dev, "using random MAC address %pM\n",
		 data->mac_addr);

	addr[0] = data->mac_addr[5];
	addr[1] = data->mac_addr[4];
	addr[2] = data->mac_addr[3];
	addr[3] = data->mac_addr[2];
	addr[4] = data->mac_addr[1];
	addr[5] = data->mac_addr[0];
	iow32(GMAC_REG_MAC_ADDR_H, *((u32 *)(addr + 4)) & 0xffff);
	iow32(GMAC_REG_MAC_ADDR_L, *((u32 *)addr));

	return 0;
}

/**
 * @brief  gmac_resource_init
 *
 * @param data : private data
 *
 * @retval 0 on success, negative on failure
 */
int gmac_resource_init(struct sep_data *data)
{
	u32 reg;
	u32 i;
	s32 ret;
	struct gmac_rx_desc *rx_desc;
	struct gmac_tx_desc *tx_desc;
	u32 size;
	u64 dma;


	/* tx padding enable,hardware crc enable,receive funt frame enable */
	reg = ior32(GMAC_REG_MAC_CTRL);
	reg |= (GMAC_CTRL_TX_PAD | GMAC_CTRL_APPEND_CRC | GMAC_CTRL_RX_RUNT);

	/* receive broadcast frame */
	reg |= GMAC_CTRL_RX_ALL_BCST_EN | GMAC_CTRL_RX_ALL_MCST_EN;

	iow32(GMAC_REG_MAC_CTRL, reg);

	/* set mac address */
	gmac_address_config(data);
	// gmac_mac_filter(data);

	/**************************** tx queue init ****************************/
	data->tx_ring.count = SEP_DEFAULT_TXD;
	data->tx_ring.ntu = 0;
	data->tx_ring.ntc = 0;

	size = data->tx_ring.count * sizeof(struct gmac_buffer_info);
	data->tx_ring.buf_info = (struct gmac_buffer_info *)vmalloc(size);
	if (!data->tx_ring.buf_info)
		goto tx_buf_info;
	memset(data->tx_ring.buf_info, 0x00, size);

	/* alloc tx desc */
	data->tx_ring.size = data->tx_ring.count * sizeof(struct gmac_tx_desc);
	// data->tx_ring.size = ALIGN(data->tx_ring.size, 4096);
	data->tx_ring.desc =
		dma_alloc_coherent(&data->pdev->dev, data->tx_ring.size,
				   &data->tx_ring.dma, GFP_ATOMIC);
	if (!data->tx_ring.desc) {
		dev_err(&data->pdev->dev, "malloc tx descriptor fail\n");
		ret = -ENOMEM;
		goto tx_desc;
	}
	memset(data->tx_ring.desc, 0x00, data->tx_ring.size);

	dma = data->tx_ring.dma + SEP_DMA_OFFSET;
	dev_info(&data->pdev->dev, "tx desc va %p pa 0x%llx\n",
		 data->tx_ring.desc, cpu_to_le64(dma));

	reg = (u32)(GMAC_TX_DESC_SZ((sizeof(struct gmac_tx_desc) >> 2)) |
		    GMAC_TX_DESC_CNT(data->tx_ring.count));
	iow32(GMAC_REG_TX_DESC_CTRL, reg);
	iow32(GMAC_REG_TX_RING_BASE_H,
	      ((cpu_to_le64(dma) & DMA_BIT_MASK(48)) >> 32));
	iow32(GMAC_REG_TX_RING_BASE_L, (cpu_to_le64(dma) & DMA_BIT_MASK(32)));

	for (i = 0; i < data->tx_ring.count; i++) {
		tx_desc = GMAC_TX_DESC(data->tx_ring.desc, i);
		/*alloc tx buffers*/
		data->tx_ring.buf_info[i].page = dev_alloc_pages(0);
		if (unlikely(!data->tx_ring.buf_info[i].page)) {
			dev_err(&data->pdev->dev, "alloc tx buffer fail\n");
			ret = -ENOMEM;
			goto tx_buf;
		}

		data->tx_ring.buf_info[i].dma = dma_map_page_attrs(
			&data->pdev->dev, data->tx_ring.buf_info[i].page, 0,
			PAGE_SIZE, DMA_BIDIRECTIONAL, SEP_DMA_ATTR);
		if (dma_mapping_error(&data->pdev->dev,
				      data->tx_ring.buf_info[i].dma)) {
			__free_pages(data->tx_ring.buf_info[i].page, 0);
			data->tx_ring.buf_info[i].page = NULL;
			dev_err(&data->pdev->dev, "map tx buffer fail\n");
			ret = -EINVAL;
			goto tx_buf;
		}

		// page_ref_add(data->tx_ring.buf_info[i].page, USHRT_MAX - 1);
		dma = data->tx_ring.buf_info[i].dma + SEP_DMA_OFFSET;
		tx_desc->txdes3 = GMAC_RXDES2_RXBUF_BADRH(cpu_to_le64(dma));
		tx_desc->txdes4 = GMAC_RXDES3_RXBUF_BADRL(cpu_to_le64(dma));

		dev_info(&data->pdev->dev, "page addr %p %p dma 0x%llx\n",
			 page_address(data->tx_ring.buf_info[i].page),
			 data->tx_ring.buf_info[i].page, dma);
	}
	print_hex_dump(KERN_INFO, "tx desc", DUMP_PREFIX_OFFSET, 16, 4,
		       data->tx_ring.desc, data->tx_ring.size, false);

	/**************************** rx queue init ****************************/
	data->rx_ring.count = SEP_DEFAULT_RXD;
	data->rx_ring.ntu = 0;
	data->rx_ring.ntc = (data->rx_ring.count - 1);

	size = data->rx_ring.count * sizeof(struct gmac_buffer_info);
	data->rx_ring.buf_info = (struct gmac_buffer_info *)vmalloc(size);
	if (!data->rx_ring.buf_info)
		goto rx_buf_info;
	memset(data->rx_ring.buf_info, 0x00, size);

	/* alloc rx desc */
	data->rx_ring.size = data->rx_ring.count * sizeof(struct gmac_rx_desc);
	// data->rx_ring.size = ALIGN(data->rx_ring.size, 4096);
	data->rx_ring.desc =
		dma_alloc_coherent(&data->pdev->dev, data->rx_ring.size,
				   &data->rx_ring.dma, GFP_ATOMIC);
	if (!data->rx_ring.desc) {
		dev_err(&data->pdev->dev, "malloc tx descriptor fail\n");
		ret = -ENOMEM;
		goto rx_desc;
	}
	memset(data->rx_ring.desc, 0x00, data->rx_ring.size);

	dma = data->rx_ring.dma + SEP_DMA_OFFSET;
	dev_info(&data->pdev->dev, "rx desc va %p pa 0x%llx\n",
		 data->rx_ring.desc, cpu_to_le64(dma));

	reg = (u32)(GMAC_RX_DESC_SZ((sizeof(struct gmac_rx_desc) >> 2)) |
		    GMAC_RX_DESC_CNT(data->rx_ring.count));
	iow32(GMAC_REG_RX_DESC_CTRL, reg);
	iow32(GMAC_REG_RX_RING_BASE_H,
	      ((cpu_to_le64(dma) & DMA_BIT_MASK(48)) >> 32));
	iow32(GMAC_REG_RX_RING_BASE_L, (cpu_to_le64(dma) & DMA_BIT_MASK(32)));

	iow32(GMAC_REG_RQ_BUF_SZ, (BUFFER_SIZE & 0x3fff));

	for (i = 0; i < data->rx_ring.count; i++) {
		rx_desc = GMAC_RX_DESC(data->rx_ring.desc, i);
		/*alloc rx buffers*/
		data->rx_ring.buf_info[i].page = dev_alloc_pages(0);
		if (unlikely(!data->rx_ring.buf_info[i].page)) {
			dev_err(&data->pdev->dev, "alloc rx buffer fail\n");
			ret = -ENOMEM;
			goto rx_buf;
		}

		data->rx_ring.buf_info[i].dma = dma_map_page_attrs(
			&data->pdev->dev, data->rx_ring.buf_info[i].page, 0,
			PAGE_SIZE, DMA_BIDIRECTIONAL, SEP_DMA_ATTR);
		if (dma_mapping_error(&data->pdev->dev,
				      data->rx_ring.buf_info[i].dma)) {
			__free_pages(data->rx_ring.buf_info[i].page, 0);
			data->rx_ring.buf_info[i].page = NULL;
			dev_err(&data->pdev->dev, "map rx buffer fail\n");
			ret = -EINVAL;
			goto rx_buf;
		}

		// page_ref_add(data->rx_ring.buf_info[i].page, USHRT_MAX - 1);
		dma = data->rx_ring.buf_info[i].dma + SEP_DMA_OFFSET;
		rx_desc->rxdes2 = GMAC_RXDES2_RXBUF_BADRH(cpu_to_le64(dma));
		rx_desc->rxdes3 = GMAC_RXDES3_RXBUF_BADRL(cpu_to_le64(dma));

		dev_info(&data->pdev->dev, "page addr %p %p dma 0x%llx\n",
			 page_address(data->rx_ring.buf_info[i].page),
			 data->rx_ring.buf_info[i].page, dma);
	}
	iow32(GMAC_REG_RX_RING_TAIL, (data->rx_ring.count - 1));
	print_hex_dump(KERN_INFO, "rx desc", DUMP_PREFIX_OFFSET, 16, 4,
		       data->rx_ring.desc, data->rx_ring.size, false);

	return 0;

rx_buf:
	for (i = 0; i < data->rx_ring.count; i++) {
		if (data->rx_ring.buf_info[i].page) {

			dma_unmap_page_attrs(
				&data->pdev->dev, data->rx_ring.buf_info[i].dma,
				PAGE_SIZE, DMA_FROM_DEVICE, SEP_DMA_ATTR);
			__free_pages(data->rx_ring.buf_info[i].page, 0);
		}
	}
	dma_free_coherent(&data->pdev->dev, data->rx_ring.size,
			  data->rx_ring.desc, data->rx_ring.dma);
rx_desc:
	vfree(data->rx_ring.buf_info);
rx_buf_info:
tx_buf:
	for (i = 0; i < data->tx_ring.count; i++) {

		if (data->tx_ring.buf_info[i].page) {
			dma_unmap_page_attrs(
				&data->pdev->dev, data->tx_ring.buf_info[i].dma,
				PAGE_SIZE, DMA_FROM_DEVICE, SEP_DMA_ATTR);
			__free_pages(data->tx_ring.buf_info[i].page, 0);
		}
	}

	dma_free_coherent(&data->pdev->dev, data->tx_ring.size,
			  data->tx_ring.desc, data->tx_ring.dma);
tx_desc:
	vfree(data->tx_ring.buf_info);
tx_buf_info:

	return ret;
}

/**
 * @brief  gmac_tx_rx_enable
 *
 * @param  data : private data
 *
 * @retval None
 */
void gmac_tx_rx_enable(struct sep_data *data)
{
	u32 reg;

	/* enable gmac tx&rx */
	reg = ior32(GMAC_REG_MAC_CTRL);
	reg |= (GMAC_CTRL_RXMAC_EN | GMAC_CTRL_TXMAC_EN | GMAC_CTRL_RXDMA_EN |
		GMAC_CTRL_TXDMA_EN);
	iow32(GMAC_REG_MAC_CTRL, reg);

	/* enable queue tx&rx */
	reg = ior32(GMAC_REG_QPAIR_CTRL);
	reg |= (GMAC_QPAIR_CTRL_RX_EN | GMAC_QPAIR_CTRL_TX_EN);
	iow32(GMAC_REG_QPAIR_CTRL, reg);
}

/**
 * @brief  gmac_rx_rx_disable
 *
 * @param data : private data
 *
 * @retval None
 */
void gmac_rx_rx_disable(struct sep_data *data)
{
	u32 reg;

	/* enable gmac tx&rx */
	reg = ior32(GMAC_REG_MAC_CTRL);
	reg &= ~(GMAC_CTRL_RXMAC_EN | GMAC_CTRL_TXMAC_EN | GMAC_CTRL_RXDMA_EN |
		 GMAC_CTRL_TXDMA_EN);
	iow32(GMAC_REG_MAC_CTRL, reg);

	/* enable queue tx&rx */
	reg = ior32(GMAC_REG_QPAIR_CTRL);
	reg &= ~(GMAC_QPAIR_CTRL_RX_EN | GMAC_QPAIR_CTRL_TX_EN);
	iow32(GMAC_REG_QPAIR_CTRL, reg);
}

/**
 * @brief  send one frame
 *
 * @param  data : private data
 * @param  buf  : send buffer
 * @param  len  : send data len
 *
 * @retval actual send length in bytes
 */
int gmac_xmit_frmae(struct sep_data *data, const char __user *buf, size_t len)
{
	struct gmac_ring *tx = &data->tx_ring;
	struct gmac_tx_desc *desc;
	u8 *send;

	if ((0 == gmac_get_tx_free_num(data)) || (len > BUFFER_SIZE))
		return -1;

	desc = GMAC_TX_DESC(tx->desc, tx->ntu);

	send = page_address(data->tx_ring.buf_info[tx->ntu].page);

	if (copy_from_user(send, buf, len))
		return -2;


	dma_sync_single_for_device(&data->pdev->dev,
				   data->tx_ring.buf_info[tx->ntu].dma,
				   PAGE_SIZE, DMA_TO_DEVICE);

	desc->txdes0 =
		GMAC_TXDES0_FTS | GMAC_TXDES0_LTS | GMAC_TXDES0_BUF_SIZE(len);
	desc->txdes5 = GMAC_TXDES5_L2_LEN(14);

	tx->ntu = (tx->ntu + 1) & (tx->count - 1);

	iow32(GMAC_REG_TX_RING_TAIL, tx->ntu);

	dev_dbg(&data->pdev->dev, "xmit frame ntu %d tail%d\n", tx->ntu,
		ior32(GMAC_REG_TX_RING_TAIL));

	return len;
}

/**
 * @brief  gphy_get_device_capability
 *
 * @param  data : private data
 *
 * @retval The gphy capability
 */
static uint32_t gphy_get_device_capability(struct sep_data *data)
{
	uint16_t bmsr = 0;
	uint32_t cap = 0;

	bmsr = gmac_mdio_read(data, MII_BMSR);

	if (bmsr & BMSR_ANEGCAPABLE)
		cap |= PHY_SUPPORTED_Autoneg;

	if (bmsr & BMSR_10FULL)
		cap |= PHY_SUPPORTED_10baseT_Full;

	if (bmsr & BMSR_10HALF)
		cap |= PHY_SUPPORTED_10baseT_Half;

	if (bmsr & BMSR_100FULL)
		cap |= PHY_SUPPORTED_100baseT_Full;

	if (bmsr & BMSR_100HALF)
		cap |= PHY_SUPPORTED_100baseT_Half;

	if (bmsr & BMSR_ESTATEN) {
		bmsr = gmac_mdio_read(data, MII_ESTATUS);

		if (bmsr & ESTATUS_1000_TFULL)
			cap |= PHY_SUPPORTED_1000baseT_Full;

		if (bmsr & ESTATUS_1000_THALF)
			cap |= PHY_SUPPORTED_1000baseT_Half;
	}

	return cap;
}

/**
 * @brief  gphy_autonegotiation_config
 *
 * @param  data : private data
 *
 * @retval None
 */
void gphy_autonegotiation_config(struct sep_data *data)
{
	uint16_t adv;
	uint16_t bmsr;
	uint16_t oldadv = 0;
	uint16_t cap;

	adv = gmac_mdio_read(data, MII_ADVERTISE);
	oldadv = adv;
	cap = gphy_get_device_capability(data);

	adv &= ~(ADVERTISE_ALL | ADVERTISE_100BASE4 | ADVERTISE_PAUSE_CAP |
		 ADVERTISE_PAUSE_ASYM);

	if (cap & PHY_ADVERTISED_10baseT_Half)
		adv |= ADVERTISE_10HALF;

	if (cap & PHY_ADVERTISED_10baseT_Full)
		adv |= ADVERTISE_10FULL;

	if (cap & PHY_ADVERTISED_100baseT_Half)
		adv |= ADVERTISE_100HALF;

	if (cap & PHY_ADVERTISED_100baseT_Full)
		adv |= ADVERTISE_100FULL;

	if (cap & PHY_ADVERTISED_Pause)
		adv |= ADVERTISE_PAUSE_CAP;

	if (cap & PHY_ADVERTISED_Asym_Pause)
		adv |= ADVERTISE_PAUSE_ASYM;

	if (cap & PHY_ADVERTISED_1000baseX_Half)
		adv |= ADVERTISE_1000XHALF;

	if (cap & PHY_ADVERTISED_1000baseX_Full)
		adv |= ADVERTISE_1000XFULL;

	if (adv != oldadv)
		gmac_mdio_write(data, MII_ADVERTISE, adv);

	bmsr = gmac_mdio_read(data, MII_BMSR);

	if (bmsr & BMSR_ESTATEN) {
		adv = gmac_mdio_read(data, MII_CTRL1000);
		oldadv = adv;
		adv &= ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF);

		if (cap & (PHY_SUPPORTED_1000baseT_Half |
			   PHY_SUPPORTED_1000baseT_Full)) {
			if (cap & PHY_SUPPORTED_1000baseT_Half)
				adv |= ADVERTISE_1000HALF;

			if (cap & PHY_SUPPORTED_1000baseT_Full)
				adv |= ADVERTISE_1000FULL;
		}

		if (adv != oldadv)
			gmac_mdio_write(data, MII_CTRL1000, adv);
	}
}

/**
 * @brief  gphy_restart_autonegotiation
 *
 * @param  data : private data
 *
 * @retval None
 */
static void gphy_restart_autonegotiation(struct sep_data *data)
{
	uint16_t bmcr;

	bmcr = gmac_mdio_read(data, MII_BMCR);
	bmcr |= (BMCR_ANENABLE | BMCR_ANRESTART);
	/* Don't isolate the PHY if we're negotiating */
	bmcr &= ~(BMCR_ISOLATE);
	gmac_mdio_write(data, MII_BMCR, bmcr);
}

/**
 * @brief  gphy_forced_speed_and_duplex
 *
 * @param data   : private data
 * @param speed  : phy speed
 * @param duplex : phy work mode
 *
 * @retval None
 */
static void gphy_forced_speed_and_duplex(struct sep_data *data, s32 speed,
					 s32 duplex)
{
	u16 ctl = BMCR_ANRESTART;

	if (speed == SPEED_1000)
		ctl |= BMCR_SPEED1000;
	else if (speed == SPEED_100)
		ctl |= BMCR_SPEED100;

	if (duplex == DUPLEX_FULL)
		ctl |= BMCR_FULLDPLX;

	gmac_mdio_write(data, MII_BMCR, ctl);

	return;
}

/**
 * @brief  gphy_get_device_identifier
 *
 * @param  data : private data
 *
 * @retval gphy identifier
 */
static u32 gphy_get_device_identifier(struct sep_data *data)
{
	u32 phyid = 0xffffffff;
	u16 physid = 0;

	physid = gmac_mdio_read(data, MII_PHYSID1);
	phyid = (physid & 0xffff) << 16;
	physid = gmac_mdio_read(data, MII_PHYSID2);
	phyid |= (physid & 0xffff);

	return phyid;
}

/**
 * @brief  gphy_init
 *
 * @param  data : private data
 *
 * @retval None
 */
void gphy_init(struct sep_data *data)
{
	u16 val = 0;
	u32 cap = 0;

	dev_info(&data->pdev->dev, "gphy id 0x%x\n",
		 gphy_get_device_identifier(data));

	/* 1.Clk delay */
	val = 0xC34D;
	gmac_mdio_write(data, 0x1B, val);

	gmac_mdio_write(data, 0x1E, 0x0);

	gmac_mdio_write(data, 0x1F, 1);
	gmac_mdio_write(data, 0x1E, 0xd787);

	gmac_mdio_write(data, 0x1F, 2);
	gmac_mdio_write(data, 0x17, 0xf000);
	gmac_mdio_write(data, 0x1E, 0x00a7);

	gmac_mdio_write(data, 0x1F, 0);

	/* 2.Reset, be careful */
	gmac_mdio_write(data, MII_BMCR, BMCR_RESET);

	while (gmac_mdio_read(data, MII_BMCR) & BMCR_RESET)
		;

	/* 3.Read feature */
	cap = gphy_get_device_capability(data);

	/* 4.Config communication param */
	if (cap & PHY_SUPPORTED_Autoneg) {
		gphy_autonegotiation_config(data);
		gphy_restart_autonegotiation(data);
	} else {
		gphy_forced_speed_and_duplex(data, SPEED_1000, DUPLEX_FULL);
	}
}

/**
 * @brief  gphy_get_link_status
 *
 * @param  data : private data
 *
 * @retval 0 on success, negative on failure
 */
int gphy_get_link_status(struct sep_data *data)
{
	uint16_t bmsr;
	uint16_t bmcr;
	uint16_t gblpa;
	uint16_t lpa;
	uint16_t estatus;
	uint16_t cap;

	data->phy_speed = SPEED_10;
	data->phy_duplex = DUPLEX_HALF;
	data->phy_link = 0;

	/* get capability */
	cap = gphy_get_device_capability(data);

	/* 2.Get the link status */
	bmsr = gmac_mdio_read(data, MII_BMSR);

	if (!(bmsr & BMSR_LSTATUS)) {
		return 0;
	}

	data->phy_link = 1;
	/* 3.Get speed and duplex information */
	if (gmac_mdio_read(data, MII_BMCR) & BMCR_ANENABLE) {
		if (cap & (PHY_SUPPORTED_1000baseT_Full |
			   PHY_SUPPORTED_1000baseT_Half)) {
			udelay(1000);

			gblpa = gmac_mdio_read(data, MII_STAT1000);
			gblpa &= gmac_mdio_read(data, MII_CTRL1000) << 2;
		}

		if (gblpa &
		    (PHY_MII_1000BTSR_1000FD | PHY_MII_1000BTSR_1000HD)) {
			data->phy_speed = SPEED_1000;

			if (gblpa & PHY_MII_1000BTSR_1000FD) {
				data->phy_duplex = DUPLEX_FULL;
			}

			return 0;
		}

		/* Set the baseline so we only have to set them
		 * if they're different
		 */
		lpa = gmac_mdio_read(data, MII_ADVERTISE);
		lpa &= gmac_mdio_read(data, MII_LPA);

		if (lpa & (LPA_100FULL | LPA_100HALF)) {
			data->phy_speed = SPEED_100;

			if (lpa & LPA_100FULL) {
				data->phy_duplex = DUPLEX_FULL;
			}
		} else if (lpa & LPA_10FULL) {
			data->phy_duplex = DUPLEX_FULL;
		}

		if ((bmsr & BMSR_ESTATEN) && !(bmsr & BMSR_ERCAP))
			estatus = gmac_mdio_read(data, MII_ESTATUS);

		if (estatus & (ESTATUS_1000_TFULL | ESTATUS_1000_THALF)) {
			data->phy_speed = SPEED_1000;

			if (estatus & ESTATUS_1000_TFULL) {
				data->phy_duplex = DUPLEX_FULL;
			}
		}
	} else {
		bmcr = gmac_mdio_read(data, MII_BMCR);

		if (bmcr & BMCR_FULLDPLX)
			data->phy_duplex = DUPLEX_FULL;

		if (bmcr & BMCR_SPEED1000) {
			data->phy_speed = SPEED_1000;
		} else if (bmcr & BMCR_SPEED100) {
			data->phy_speed = SPEED_100;
		}
	}

	return 0;
}

/**
 * @brief  gphy_speed_to_str
 *
 * @param  speed : gphy speed
 *
 * @retval The string of current speed
 */
const char *gphy_speed_to_str(int speed)
{
	switch (speed) {
		case SPEED_10:
			return "10Mbps";
		case SPEED_100:
			return "100Mbps";
		case SPEED_1000:
			return "1Gbps";
		case SPEED_2500:
			return "2.5Gbps";
		case SPEED_5000:
			return "5Gbps";
		case SPEED_10000:
			return "10Gbps";
		case SPEED_14000:
			return "14Gbps";
		case SPEED_20000:
			return "20Gbps";
		case SPEED_25000:
			return "25Gbps";
		case SPEED_40000:
			return "40Gbps";
		case SPEED_50000:
			return "50Gbps";
		case SPEED_56000:
			return "56Gbps";
		case SPEED_100000:
			return "100Gbps";
		case SPEED_UNKNOWN:
			return "Unknown";
		default:
			return "Unsupported";
	}
}

/**
 * @brief  gphy_duplex_to_str
 *
 * @param  duplex : gphy duplex
 *
 * @retval The string of current duplex
 */
const char *gphy_duplex_to_str(unsigned int duplex)
{
	if (duplex == DUPLEX_HALF)
		return "Half";
	if (duplex == DUPLEX_FULL)
		return "Full";
	if (duplex == DUPLEX_UNKNOWN)
		return "Unknown";
	return "Unsupported";
}

/**
 * @brief  gphy link state monitor
 *
 * @param  work : work queue struct
 *
 * @retval None
 */
void phy_state_machine(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sep_data *data =
		container_of(dwork, struct sep_data, state_queue);
	int old_link;
	u32 reg;

	mutex_lock(&data->lock);

	old_link = data->phy_link;

	gphy_get_link_status(data);
	if (old_link != data->phy_link) {
		if (data->phy_link) {
			dev_info(&data->pdev->dev, "Link is Up - %s/%s\n",
				 gphy_speed_to_str(data->phy_speed),
				 gphy_duplex_to_str(data->phy_duplex));
			/* set gmac speed and work mode */
			reg = ior32(GMAC_REG_MAC_CTRL);

			if (data->phy_speed == SPEED_1000)
				reg |= GMAC_CTRL_SPEED(GMAC_SPEED_1000);
			else if (data->phy_speed == SPEED_100)
				reg |= GMAC_CTRL_SPEED(GMAC_SPEED_100);
			else
				reg |= GMAC_CTRL_SPEED(GMAC_SPEED_10);

			if (data->phy_duplex == DUPLEX_FULL)
				reg |= GMAC_CTRL_DUPLEX_FULL;
			else
				reg &= ~GMAC_CTRL_DUPLEX_FULL;

			iow32(GMAC_REG_MAC_CTRL, reg);

			/* enable gmac tx rx function*/
			gmac_tx_rx_enable(data);
		} else {
			gmac_rx_rx_disable(data);
			dev_info(&data->pdev->dev, "Link is Down\n");
		}
	}
	mutex_unlock(&data->lock);

	queue_delayed_work(system_power_efficient_wq, &data->state_queue, HZ);
}

int sep_open(struct inode *inode, struct file *file)
{
	struct sep_data *data;
	data = container_of(inode->i_cdev, struct sep_data, cdev);
	if (IS_ERR(data)) {
		pr_err("fail to get japan private data!!!\n");
		return -1;
	}

	/* check phy link status */
	if (!data->phy_link) {
		dev_err(&data->pdev->dev, "phy not link!!!\n");
		return -2;
	}

	spin_lock(&data->spinlock);
	data->ref++;
	spin_unlock(&data->spinlock);

	file->private_data = data;

	return 0;
}

ssize_t sep_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
	struct sep_data *data = (struct sep_data *)(file->private_data);
	struct gmac_rx_desc *desc;
	struct gmac_ring *rx;
	u32 pkt_len;
	u32 ret;

	if (len > BUFFER_SIZE)
		return -4;

	rx = &data->rx_ring;

	if (gmac_received_pack_num(data)) {
		rx->ntc = (rx->ntc + 1) & (rx->count - 1);

		desc = GMAC_RX_DESC(rx->desc, rx->ntc);
		dev_dbg(&data->pdev->dev, "get frame desc0 0x%x\n",
			desc->rxdes0);
		dev_dbg(&data->pdev->dev, "get frame desc1 0x%x\n",
			desc->rxdes1);

		if (!(desc->rxdes0 & GMAC_RXDES0_FRS))
			return -2;
		if (!(desc->rxdes0 & GMAC_RXDES0_LRS))
			return -3;

		pkt_len = GMAC_RXDES0_VDBC((desc->rxdes0 - 4)); /* 4 for crc */

		dma_sync_single_for_cpu(&data->pdev->dev,
					data->rx_ring.buf_info[rx->ntc].dma,
					PAGE_SIZE, DMA_FROM_DEVICE);

		ret = copy_to_user(
			buf, page_address(data->rx_ring.buf_info[rx->ntc].page),
			pkt_len);
		if (ret) {
			dev_err(&data->pdev->dev, "fail read len %d\n", ret);
			return -1;
		}
		wmb();
		iow32(GMAC_REG_RX_RING_TAIL, rx->ntc);
		// pr_info("rx->ntc %d read  %d\n", rx->ntc, ((u8 *)(page_address(data->rx_ring.buf_info[rx->ntc].page)))[0]);
		return pkt_len;
	}
	return 0;
}

ssize_t sep_write(struct file *file, const char __user *buf, size_t len,
		  loff_t *ppos)
{
	struct sep_data *data = (struct sep_data *)(file->private_data);

	return gmac_xmit_frmae(data, buf, len);
}

static const struct file_operations sep_fileops = {
	.owner = THIS_MODULE,
	.open = sep_open,
	.write = sep_write,
	.read = sep_read,
	.llseek = no_llseek,
};

static char *cdev_node(struct device *dev, mode_t *mode)
{
	if (mode)
		*mode = 0666;

	return NULL;
}

static irqreturn_t sep_msix_clean_rings(int irq, void *data)
{

	return IRQ_HANDLED;
}

static irqreturn_t sep_msix_other(int irq, void *data)
{
	return IRQ_HANDLED;
}

static int sep_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret;
	struct sep_data *data;

	char name[128];
	int count = 0;
	int i, vectors, vector_threshold;

	/* Catch broken hardware that put the wrong VF device ID in
	 * the PCIe SR-IOV capability.
	 */
	if (pdev->is_virtfn) {
		WARN(1, KERN_ERR "%s (%hx:%hx) should not be a VF!\n",
		     pci_name(pdev), id->vendor, id->device);
		return -EINVAL;
	}

	dev_info(&pdev->dev, "PCI device[0x%04x:0x%04x] has been probed!!!\n",
		 id->vendor, id->device);


	ret = pci_enable_device_mem(pdev);
	if (ret) {
		dev_err(&pdev->dev, "enable PCIe device[0x%04x:0x%04x] fail\n",
			id->vendor, id->device);
		return ret;
	}

	if (!dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(43))) {
		dev_info(&pdev->dev, "using dma bit mask 43\n");
	} else {
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(&pdev->dev,
				"no usable DMA configuration, aborting\n");
			goto err_dma_mask;
		}
	}

	ret = pci_request_mem_regions(pdev, DEV_NAME);
	if (ret) {
		dev_err(&pdev->dev,
			"pci request selected mem regions failed 0x%x\n", ret);
		goto err_pci_reg;
	}

	// pci_enable_pcie_error_reporting(pdev);

	pci_set_master(pdev);
	pci_save_state(pdev);

	/* alloc private data */
	data = kvzalloc(sizeof(struct sep_data),
			GFP_KERNEL | __GFP_RETRY_MAYFAIL);
	if (!data) {
		ret = -ENOMEM;
		goto err_mem;
	}

	data->pdev = pdev;
	data->bar0_phy_addr = pci_resource_start(pdev, 0);
	data->bar0_len = pci_resource_len(pdev, 0);

	data->io_base = devm_ioremap(&pdev->dev, pci_resource_start(pdev, 0),
				     pci_resource_len(pdev, 0));
	if (!data->io_base) {
		ret = -EIO;
		dev_err(&pdev->dev, "ioremap failed 0x%x\n", ret);
		goto err_ioremap;
	}
	dev_info(&pdev->dev, "gmac io base %p phy 0x%llx len %lld\n",
		 data->io_base, data->bar0_phy_addr, data->bar0_len);

	spin_lock_init(&(data->spinlock));

	if (major) {
		data->c_devid = MKDEV(major, 0);
		ret = register_chrdev_region(data->c_devid, 1, DEV_NAME);
	} else {
		ret = alloc_chrdev_region(&data->c_devid, 0, 1, DEV_NAME);
		major = MAJOR(data->c_devid);
	}

	if (ret < 0) {
		dev_err(&pdev->dev, "register-chrdev failed: %d\n", ret);
		goto err_register;
	}
	if (!major) {
		major = ret;
		dev_dbg(&pdev->dev, "got dynamic major %d\n", major);
	}

	/* ignore minor errs, and succeed */
	cdev_init(&data->cdev, &sep_fileops);
	cdev_add(&data->cdev, data->c_devid, 1);

	dev_info(&pdev->dev, "add cdev success dev id %d\n", data->c_devid);

	snprintf(name, sizeof(name), "%s-class-%d", DEV_NAME, count);

	data->cdev_class = class_create(THIS_MODULE, name);
	if (!data->cdev_class) {
		ret = -1;
		goto err_class;
	}

	memset(name, 0x00, sizeof(name));
	snprintf(name, sizeof(name), "%s-dev-%d", DEV_NAME, count);
	data->cdev_class->devnode = (void *)cdev_node;
	device_create(data->cdev_class, NULL, data->c_devid, NULL, name);

	mdelay(1000);

	data->ci.init = 0;
	data->ci.cmd_buf_size = SEP_MAX_CMD_BUF_SIZE;
	data->ci.cmd_timeout = SEP_CMD_CMD_TIMEOUT;
	sep_init_cmd(data);

	/* Event */
	data->ei.init = 0;
	data->ei.cmd_buf_size = SEP_MAX_CMD_BUF_SIZE;
	data->ei.cmd_timeout = SEP_CMD_CMD_TIMEOUT;
	sep_init_event(data);

	dev_info(&pdev->dev, "gmac hw version 0x%x\n", ior32(GMAC_REG_REV));

	ret = gmac_sw_reset(data);
	if (ret)
		goto err_gmac_reset;

	data->mdc_cycthr = 80;
	data->phy_addr = 1;

	/* gmac init */
	if (gmac_resource_init(data))
		goto err_resource;
	wmb();

	vectors = min_t(int, MSIX_VECTORS, num_online_cpus());

	vector_threshold = MIN_MSIX_COUNT;

	data->msix_entries =
		kcalloc(vectors, sizeof(struct msix_entry), GFP_KERNEL);
	if (!data->msix_entries) {
		ret = -ENOMEM;
		goto err_resource;
	}

	for (i = 0; i < vectors; i++)
		data->msix_entries[i].entry = i;

	vectors = pci_enable_msix_range(pdev, data->msix_entries,
					vector_threshold, vectors);
	if (vectors < 0) {
		/* A negative count of allocated vectors indicates an error in
			* acquiring within the specified range of MSI-X vectors
			*/
		pr_err("Failed to allocate MSI-X interrupts. Err: %d\n",
		       vectors);
		ret = vectors;
		goto err_msix;
	}

	memset(name, 0x00, sizeof(name));
	snprintf(name, sizeof(name), "%s-TxRx-msix", pci_name(pdev));
	vectors -= 1;
	for (i = 0; i < vectors; i++) {
		ret = request_irq(data->msix_entries[i].vector,
				  &sep_msix_clean_rings, 0, name, data);
		if (ret) {
			pr_err("request_irq failed for Tx Rx MSIX interrupt "
			       "Error: %d\n",
			       ret);
			goto err_msix;
		}
	}

	ret = request_irq(data->msix_entries[i].vector, &sep_msix_other, 0,
			  pci_name(pdev), data);
	if (ret) {
		pr_err("request_irq for msix_other failed: %d\n", ret);
		goto err_msix;
	}

	sep_cmd_reset(data);

	sep_cmd_fw_init(data);

	sep_cmd_intv_alloc(data);

	gphy_init(data);

	/* init work queue */
	INIT_DELAYED_WORK(&data->state_queue, phy_state_machine);

	pci_set_drvdata(pdev, data);

	queue_delayed_work(system_power_efficient_wq, &data->state_queue,
			   (HZ * 10));

	ret = seq_dbg_adapter_init(data);
	if (ret)
		goto err_debugfs;

	return 0;
err_debugfs:
err_msix:
	while (i) {
		i--;
		irq_set_affinity_hint(data->msix_entries[i].vector, NULL);
		free_irq(data->msix_entries[i].vector, NULL);
	}
	pci_disable_msix(pdev);
	kfree(data->msix_entries);
	data->msix_entries = NULL;
err_resource:
err_gmac_reset:
err_class:
	cdev_del(&data->cdev);
err_register:
	iounmap(data->io_base);
err_ioremap:
	kfree(data);
err_mem:
	pci_release_mem_regions(pdev);
err_pci_reg:
err_dma_mask:
	pci_disable_device(pdev);
	return ret;
}

/**
 * @brief
 *
 * @param pdev
 */
static void sep_remove(struct pci_dev *pdev)
{
	struct sep_data *data = pci_get_drvdata(pdev);
	u32 i;

	dev_info(&pdev->dev, "remove usa japan card driver!!!\n");

	seq_dbg_adapter_exit(data);

	cdev_del(&data->cdev);
	unregister_chrdev_region(data->c_devid, 1);
	device_destroy(data->cdev_class, data->c_devid);
	class_destroy(data->cdev_class);

	iounmap(data->io_base);

	pci_clear_master(pdev);
	pci_release_mem_regions(pdev);
	// pci_disable_pcie_error_reporting(pdev);
	pci_disable_device(pdev);

	for (i = 0; i < data->rx_ring.count; i++) {
		if (data->rx_ring.buf_info[i].page) {

			dma_unmap_page_attrs(
				&data->pdev->dev, data->rx_ring.buf_info[i].dma,
				PAGE_SIZE, DMA_FROM_DEVICE, SEP_DMA_ATTR);
			__free_pages(data->rx_ring.buf_info[i].page, 0);
		}
	}

	for (i = 0; i < data->tx_ring.count; i++) {

		if (data->tx_ring.buf_info[i].page) {
			dma_unmap_page_attrs(
				&data->pdev->dev, data->tx_ring.buf_info[i].dma,
				PAGE_SIZE, DMA_FROM_DEVICE, SEP_DMA_ATTR);
			__free_pages(data->tx_ring.buf_info[i].page, 0);
		}
	}

	dma_free_coherent(&data->pdev->dev, data->rx_ring.size,
			  data->rx_ring.desc, data->rx_ring.dma);
	vfree(data->rx_ring.buf_info);

	dma_free_coherent(&data->pdev->dev, data->tx_ring.size,
			  data->tx_ring.desc, data->tx_ring.dma);
	vfree(data->tx_ring.buf_info);

	if (data->msix_entries) {
		pci_disable_msix(pdev);
		kfree(data->msix_entries);
		data->msix_entries = NULL;
	}
	kfree(data);
}



static struct pci_driver sep_driver = {
	.name = DEV_NAME,
	.id_table = sep_pci_tbl,
	.probe = sep_probe,
	.remove = sep_remove,
};

/**
 * @brief sep_init_module
 *
 * @retval None
 */
static int __init sep_init_module(void)
{
	int ret;

	pr_info("load japan card driver\n");

	ret = seq_dbgfs_init((char *)DEV_NAME);

	if (ret) {
		pr_err("register debugfs fail!\n");
		return ret;
	}

	ret = pci_register_driver(&sep_driver);
	if (ret) {
		pr_err("register pci driver fail!\n");
		return ret;
	}


	return 0;
}

/**
 * @brief  sep_exit_module
 *
 * @retval None
 */
static void __exit sep_exit_module(void)
{
	pci_unregister_driver(&sep_driver);
	seq_dbgfs_exit();
}

module_init(sep_init_module);
module_exit(sep_exit_module);

MODULE_AUTHOR("usa Corporation");
MODULE_DESCRIPTION("usa 1 Gigabit PCI Express japan Card Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);