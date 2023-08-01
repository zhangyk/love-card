// SPDX-License-Identifier: GPL-2.0
/*
 * This is the driver for the japan card for usa SoCs.
 *
 * Copyright (C) 2021, usa MicroElectronic Technology - All Rights Reserved
 */

#ifndef _GMAC_REGMAP_H_
#define _GMAC_REGMAP_H_

#define USING_TX_FILTER 1

/* fake Registers Offset */
#define FREG_CMD_ADDR_L	  0xD00 /* cmd desc base least significant address fake register */
#define FREG_CMD_ADDR_H	  0xD04 /* cmd desc base most significant address fake register */
#define FREG_CMD_EXE	  0xD08 /* cmd trigger fake register */
#define FREG_EVENT_ADDR_L 0xD0C /*  */
#define FREG_EVENT_ADDR_H 0xD10 /*  */
#define FREG_EVENT_READY  0xD14 /*  */
#define FREG_QUEUE_ISR	  0xD18 /* interrupt status fake register */
#define FREG_QUEUE_IER	  0xD1C /* interrupt enable fake register */
#define FREG_TDBAL	  0xD20 /* tx desc ring base least significant address fake register */
#define FREG_TDBAH	  0xD24 /* tx desc ring base most significant address fake register */
#define FREG_TDH	  0xD28 /* tx desc ring header fake register */
#define FREG_TDT	  0xD2C /* tx desc ring tail fake register */
#define FREG_TDCTL	  0XD30 /* tx desc control fake register */
#define FREG_TSTL	  0xD34 /* offset 0xD34 tx send statistic fake register */
#define FREG_TSTH	  0xD38 /* offset 0xD38 tx send statistic fake register */
#define FREG_TSDL	  0xD3C /* offset 0xD3C tx statistic fake register */
#define FREG_TSDH	  0xD40 /* offset 0xD40 tx statistic fake register */
#define FREG_DBG	  0XD44 /* offset 0xD44 debug fake register */

/* GMAC Registers Offset */
#define GMAC_REG_MAC_CTRL	     0x000
#define GMAC_REG_FDPX_FC	     0X004
#define GMAC_REG_DMA_BURST_LEN	     0X00C
#define GMAC_REG_DMA_FIFO_STS	     0X010
#define GMAC_REG_WOL_CTRL	     0X018
#define GMAC_REG_WOL_STS	     0X01C
#define GMAC_REG_PTP_RX_UCST_DA	     0X034
#define GMAC_REG_PTP_TX_UCST_DA	     0x038
#define GMAC_REG_PTP_TX_PSEC	     0X03C
#define GMAC_REG_PTP_TX_PNSEC	     0X040
#define GMAC_REG_PTP_RX_PSEC	     0X044
#define GMAC_REG_PTP_RX_PNSEC	     0X048
#define GMAC_REG_PTP_TX_P_PSEC	     0X04C
#define GMAC_REG_PTP_TX_P_PNSEC	     0X050
#define GMAC_REG_PTP_RX_P_PSEC	     0X054
#define GMAC_REG_PTP_RX_P_PNSEC	     0X058
#define GMAC_REG_PTP_TMRNN	     0X05C
#define GMAC_REG_PTP_TMRN	     0X060
#define GMAC_REG_PTP_TMRS	     0X064
#define GMAC_REG_PTP_PER_NS	     0X068
#define GMAC_REG_PTP_PER_NNS	     0X06C
#define GMAC_REG_PTP_OFFSET	     0X070
#define GMAC_REG_PTP_ADJ	     0X074
#define GMAC_REG_AXI_CTRL	     0X078
#define GMAC_REG_PHY_IF		     0X07C
#define GMAC_REG_SW_RST_CYCLE_CNT    0X080
#define GMAC_REG_EEE_CTRL	     0X084
#define GMAC_REG_REV		     0X088
#define GMAC_REG_HW_FEATURE	     0X08C
#define GMAC_REG_BMCST_THR_CTRL	     0X098
#define GMAC_REG_PHY_CTRL	     0X09C
#define GMAC_REG_PHY_DATA	     0X0A0
#define GMAC_REG_SYS_ISR	     0X19C
#define GMAC_REG_SYS_IE		     0X1A0
#define GMAC_REG_MAC_ADDR_L	     0X1A8
#define GMAC_REG_MAC_ADDR_H	     0X1AC
#define GMAC_REG_MCST_ADDR_HASH_TBL0 0X1B0
#define GMAC_REG_MCST_ADDR_HASH_TBL1 0X1B4
#define GMAC_REG_L2_FLT0_ADDR_L	     0X1B8
#define GMAC_REG_L2_FLT0_ADDR_H	     0X1BC
#define GMAC_REG_L2_FLT1_ADDR_L	     0X1C0
#define GMAC_REG_L2_FLT1_ADDR_H	     0X1C4
#define GMAC_REG_L2_FLT2_ADDR_L	     0X1C8
#define GMAC_REG_L2_FLT2_ADDR_H	     0X1CC
#define GMAC_REG_L2_FLT3_ADDR_L	     0X1D0
#define GMAC_REG_L2_FLT3_ADDR_H	     0X1D4
#define GMAC_REG_L2_FLT4_ADDR_L	     0X1D8
#define GMAC_REG_L2_FLT4_ADDR_H	     0X1DC
#define GMAC_REG_L2_FLT5_ADDR_L	     0X1E0
#define GMAC_REG_L2_FLT5_ADDR_H	     0X1E4
#define GMAC_REG_L2_FLT6_ADDR_L	     0X1E8
#define GMAC_REG_L2_FLT6_ADDR_H	     0X1EC
#define GMAC_REG_L2_FLT7_ADDR_L	     0X1F0
#define GMAC_REG_L2_FLT7_ADDR_H	     0X1F4
#define GMAC_REG_RSSC		     0X2A0
#define GMAC_REG_RSS_RK(_i)	     (0X2A4 + ((_i)*4)) /* 0X2A4+4*N(N=0-7) */
#define GMAC_REG_RSS_RK0	     0X2A4
#define GMAC_REG_RSS_RK1	     0X2A8
#define GMAC_REG_RSS_RK2	     0X2AC
#define GMAC_REG_RSS_RK3	     0X2B0
#define GMAC_REG_RSS_RK4	     0X2B4
#define GMAC_REG_RSS_RK5	     0X2B8
#define GMAC_REG_RSS_RK6	     0X2BC
#define GMAC_REG_RSS_RK7	     0X2C0
#define GMAC_REG_RSS_RK8	     0X2C4
#define GMAC_REG_RSS_RK9	     0X2C8
#define GMAC_REG_TQ_WB_CTRL	     0X314
#define GMAC_REG_RQ_WB_CTRL	     0X318

#define GMAC_REG_QPAIR_CTRL 0X400
#if defined(USING_TX_FILTER) && USING_TX_FILTER == 1
#define GMAC_REG_QUEUE_ISR FREG_QUEUE_ISR
#define GMAC_REG_QUEUE_IER FREG_QUEUE_IER
#else
#define GMAC_REG_QUEUE_ISR 0X404
#define GMAC_REG_QUEUE_IER 0X408
#endif
#define GMAC_REG_TX_ITC 0X40C
#define GMAC_REG_RX_ITC 0X410

#define GMAC_REG_RX_DESC_CTRL	0X414
#define GMAC_REG_RX_RING_BASE_L 0X418
#define GMAC_REG_RX_RING_BASE_H 0X41C
#define GMAC_REG_RX_RING_HEAD	0X420
#define GMAC_REG_RX_RING_TAIL	0X424

#if defined(USING_TX_FILTER) && USING_TX_FILTER == 1
#define GMAC_REG_TX_DESC_CTRL	FREG_TDCTL
#define GMAC_REG_TX_RING_BASE_L FREG_TDBAL
#define GMAC_REG_TX_RING_BASE_H FREG_TDBAH
#define GMAC_REG_TX_RING_HEAD	FREG_TDH
#define GMAC_REG_TX_RING_TAIL	FREG_TDT
#else
#define GMAC_REG_TX_DESC_CTRL	0X440
#define GMAC_REG_TX_RING_BASE_L 0X444
#define GMAC_REG_TX_RING_BASE_H 0X448
#define GMAC_REG_TX_RING_HEAD	0X44C
#define GMAC_REG_TX_RING_TAIL	0X450
#endif

#define GMAC_REG_TPH_CTRL    0X464
#define GMAC_REG_FUNC_ID     0X468
#define GMAC_REG_TQ_BUF_CTRL 0X46C
#define GMAC_REG_RQ_BUF_CTRL 0X470
#define GMAC_REG_RQ_BUF_SZ   0X474

/* ======================================================================================*/

#define GMAC_SPEED_10	0
#define GMAC_SPEED_100	1
#define GMAC_SPEED_1000 2

#define GMAC_CTRL_SW_RST	   BIT(31)
#define GMAC_CTRL_CTAG_RM	   BIT(28)
#define GMAC_CTRL_STAG_RM	   BIT(27)
#define GMAC_CTRL_DUPLEX_FULL	   BIT(26)
#define GMAC_CTRL_SPEED(x)	   ((x & 0x3) << 24)
#define GMAC_CTRL_IFG		   BIT(23)
#define GMAC_CTRL_IFG_CNT(x)	   ((x & 0x7) << 20)
#define GMAC_CTRL_LOOPBACK	   BIT(19)
#define GMAC_CTRL_PTP_EN	   BIT(18)
#define GMAC_CTRL_TX_PAD	   BIT(17)
#define GMAC_CTRL_APPEND_CRC	   BIT(15)
#define GMAC_CTRL_JUMBO_LONG	   BIT(13)
#define GMAC_CTRL_RX_RUNT	   BIT(12)
#define GMAC_CTRL_RX_ALL_BCST_EN   BIT(11)
#define GMAC_CTRL_RX_ALL_MCST_EN   BIT(10)
#define GMAC_CTRL_RX_HASH_MCAST_EN BIT(9)
#define GMAC_CTRL_L4_FLT_EN	   BIT(7)
#define GMAC_CTRL_L3_FLT_EN	   BIT(6)
#define GMAC_CTRL_VLAN_FLT_EN	   BIT(5)
#define GMAC_CTRL_L2_FLT_EN	   BIT(4)
#define GMAC_CTRL_RXMAC_EN	   BIT(3)
#define GMAC_CTRL_TXMAC_EN	   BIT(2)
#define GMAC_CTRL_RXDMA_EN	   BIT(1)
#define GMAC_CTRL_TXDMA_EN	   BIT(0)

#define GMAC_FDPX_FC_PAUSE_TIME(x) ((x & 0xffff) << 16)
#define GMAC_FDPX_FC_LOW_THR(x)	   ((x & 0x7f) << 9)
#define GMAC_FDPX_FC_HIGH_THR(x)   ((x & 0x7f) << 2)
#define GMAC_FDPX_FC_EN		   BIT(0)

#define GMAC_HDPX_FC_LOW_THR(x) ((x & 0x7f) << 8)
#define GMAC_HDPX_FC_JAM_LEN(x) ((x & 0xf) << 4)
#define GMAC_HDPX_FC_EN		BIT(0)

#define GMAC_TXDMA_BURST_LEN_64	 ((0 & 0x3) << 2)
#define GMAC_TXDMA_BURST_LEN_128 ((1 & 0x3) << 2)
#define GMAC_TXDMA_BURST_LEN_256 ((2 & 0x3) << 2)
#define GMAC_TXDMA_BURST_LEN_512 ((3 & 0x3) << 2)

#define GMAC_RXDMA_BURST_LEN_64	 ((0 & 0x3) << 0)
#define GMAC_RXDMA_BURST_LEN_128 ((1 & 0x3) << 0)
#define GMAC_RXDMA_BURST_LEN_256 ((2 & 0x3) << 0)
#define GMAC_RXDMA_BURST_LEN_512 ((3 & 0x3) << 0)

#define GMAC_RX_BUF_SIZE(x) ((x & 0x3fff) << 0)

#define GMAC_WOLCR_OUTPUT_ACT_HIGH  ((0 & 0x3) << 24)
#define GMAC_WOLCR_OUTPUT_ACT_LOW   ((1 & 0x3) << 24)
#define GMAC_WOLCR_OUTPUT_PULSE_POS ((2 & 0x3) << 24)
#define GMAC_WOLCR_OUTPUT_PULSE_NEG ((3 & 0x3) << 24)
#define GMAC_WOLCR_SW_PDNPHY	    BIT(18)
#define GMAC_WOLCR_WAKEUP_SEL(x)    ((x & 0x3) << 16)
#define GMAC_WOLCR_WAKEUP4	    BIT(6)
#define GMAC_WOLCR_WAKEUP3	    BIT(5)
#define GMAC_WOLCR_WAKEUP2	    BIT(4)
#define GMAC_WOLCR_WAKEUP1	    BIT(3)
#define GMAC_WOLCR_MAGICPKT	    BIT(2)
#define GMAC_WOLCR_LINKCHG1	    BIT(1)
#define GMAC_WOLCR_LINKCHG0	    BIT(0)

#define GMAC_WOLSR_WAKEUP4  BIT(6)
#define GMAC_WOLSR_WAKEUP3  BIT(5)
#define GMAC_WOLSR_WAKEUP2  BIT(4)
#define GMAC_WOLSR_WAKEUP1  BIT(3)
#define GMAC_WOLSR_MAGICPKT BIT(2)
#define GMAC_WOLSR_LINKCHG1 BIT(1)
#define GMAC_WOLSR_LINKCHG0 BIT(0)

#define GMAC_PTP_TX_NSEC(x) ((x & 0x3fffffff) << 0)

#define GMAC_PTP_RX_NSEC(x) ((x & 0x3fffffff) << 0)

#define GMAC_PTP_TX_P_NSEC(x) ((x & 0x3fffffff) << 0)

#define GMAC_PTP_RX_P_NSEC(x) ((x & 0x3fffffff) << 0)

#define GMAC_PTP_TMR1(x) ((x & 0x3fffffff) << 0)

#define GMAC_PTP_ADJ_EN BIT(31)

#define GMAC_PTP_ADJ_VAL(x) ((x & 0x3fffffff) << 0)

#define GMAC_AXI_CTRL_RID(x) ((x & 0x7f) << 25)
#define GMAC_AXI_CTRL_WID(x) ((x & 0x7f) << 9)

#define GMAC_BMCST_THR_TIME_STEP BIT(24)
#define GMAC_BMCST_THR_NUM(x)	 ((x & 0xff) << 16)
#define GMAC_BMCST_THR_VAL(x)	 ((x & 0x1f) << 0)

#define GMAC_MDIO_SOF	    1
#define GMAC_MDIO_EXT_SOF   0
#define GMAC_MDIO_OP_RD	    2
#define GMAC_MDIO_OP_WR	    1
#define GMAC_PHYCR_PHYWR    BIT(27)
#define GMAC_PHYCR_PHYRD    BIT(26)
#define GMAC_PHYCR_REGAD(x) ((x & 0x1f) << 21)
#define GMAC_PHYCR_PHYAD(x) ((x & 0x1f) << 16)
#define GMAC_PHYCR_OP(x)    ((x & 0x3) << 14)
#define GMAC_PHYCR_SOF(x)   ((x & 0x3) << 12)

#define GMAC_PHYCR_MDC_CYCTHR(x) ((x & 0xff) << 0)
#define GMAC_PHYDATA_RDATA(x)	 ((x & 0xffff0000) >> 16)
#define GMAC_PHYDATA_WDATA(x)	 ((x & 0xffff) >> 0)

#define GMAC_INT_PDELAY_RESP_OUT BIT(24)
#define GMAC_INT_PDELAY_RESP_IN	 BIT(23)
#define GMAC_INT_PDELAY_REQ_OUT	 BIT(22)
#define GMAC_INT_PDELAY_REQ_IN	 BIT(21)
#define GMAC_INT_DELAY_REQ_OUT	 BIT(20)
#define GMAC_INT_DELAY_REQ_IN	 BIT(19)
#define GMAC_INT_SYNC_OUT	 BIT(18)
#define GMAC_INT_SYNC_IN	 BIT(17)
#define GMAC_INT_TSU_SEC_INC	 BIT(16)
#define GMAC_INT_RX_LPI_IN	 BIT(12)
#define GMAC_INT_RX_LPI_EXIT	 BIT(11)
#define GMAC_INT_PHYSTS_CHG	 BIT(9)
#define GMAC_INT_AHB_ERR	 BIT(8)
#define GMAC_INT_TPKT_LOST	 BIT(7)
#define GMAC_INT_RXBUF_UNAVA	 BIT(2)

#define GMAC_CTAG_TYPE(x) ((x & 0xffff) << 16)
#define GMAC_STAG_TYPE(x) ((x & 0xffff) << 0)

#define GMAC_L2_FLT_EN	       BIT(22)
#define GMAC_L2_REDR_EN	       BIT(21)
#define GMAC_L2_FLT_VMD(x)     ((x & 0x7) << 18)
#define GMAC_L2_FLT_INV	       BIT(17)
#define GMAC_L2_FLT_DA	       BIT(16)
#define GMAC_L2_FLT_ADDR_LS(x) ((x & 0xffff) << 0)
#define GMAC_L2_FLT_ADDR_L(_i) (GMAC_REG_L2_FLT0_ADDR_L + ((_i)*4))
#define GMAC_L2_FLT_ADDR_H(_i) (GMAC_REG_L2_FLT0_ADDR_H + ((_i)*4))

#define GMAC_VLAN_FLT_ID1_EN	 BIT(27)
#define GMAC_VLAN_FLT_ID0_EN	 BIT(26)
#define GMAC_VLAN_FLT_ID1_INV	 BIT(25)
#define GMAC_VLAN_FLT_ID0_INV	 BIT(24)
#define GMAC_VLAN_FLT_ID1(x)	 ((x & 0xfff) << 12)
#define GMAC_VLAN_FLT_ID0(x)	 ((x & 0xfff) << 0)
#define GMAC_VLAN_FLT_ID1_VAL(r) ((r & 0xfff000) >> 12)
#define GMAC_VLAN_FLT_ID0_VAL(r) (r & 0xfff)

#define GMAC_L3_FLT_IP7_EN	 BIT(15)
#define GMAC_L3_FLT_IP6_EN	 BIT(14)
#define GMAC_L3_FLT_IP5_EN	 BIT(13)
#define GMAC_L3_FLT_IP4_EN	 BIT(12)
#define GMAC_L3_FLT_IP3_EN	 BIT(11)
#define GMAC_L3_FLT_IP2_EN	 BIT(10)
#define GMAC_L3_FLT_IP1_EN	 BIT(9)
#define GMAC_L3_FLT_IP0_EN	 BIT(8)
#define GMAC_L3_FLT_IP7_SEL_DEST BIT(7)
#define GMAC_L3_FLT_IP6_SEL_DEST BIT(6)
#define GMAC_L3_FLT_IP5_SEL_DEST BIT(5)
#define GMAC_L3_FLT_IP4_SEL_DEST BIT(4)
#define GMAC_L3_FLT_IP3_SEL_DEST BIT(3)
#define GMAC_L3_FLT_IP2_SEL_DEST BIT(2)
#define GMAC_L3_FLT_IP1_SEL_DEST BIT(1)
#define GMAC_L3_FLT_IP0_SEL_DEST BIT(0)
#define GMAC_L3_FLT_IPn_EN(n)	 BIT((8 + n))


#define GMAC_IP6_FLT_MASK_D(x) ((x & 0x1f) << 15)
#define GMAC_IP6_FLT_MASK_C(x) ((x & 0x1f) << 10)
#define GMAC_IP6_FLT_MASK_B(x) ((x & 0x1f) << 5)
#define GMAC_IP6_FLT_MASK_A(x) ((x & 0x1f) << 0)

#define GMAC_L4_FLT_DST_PORT(x) ((x & 0xffff) << 16)
#define GMAC_L4_FLT_SRC_PORT(x) ((x & 0xffff) << 0)

#define GMAC_RSS_MAC_HASH_EN	  BIT(10)
#define GMAC_RSS_TCPIP4_HASH_EN	  BIT(9)
#define GMAC_RSS_IP4_HASH_EN	  BIT(8)
#define GMAC_RSS_TCPIP6EX_HASH_EN BIT(7)
#define GMAC_RSS_IP6EX_HASH_EN	  BIT(6)
#define GMAC_RSS_IP6_HASH_EN	  BIT(5)
#define GMAC_RSS_TCPIP6_HASH_EN	  BIT(4)
#define GMAC_RSS_UDPIP4_HASH_EN	  BIT(3)
#define GMAC_RSS_UDPIP6_HASH_EN	  BIT(2)
#define GMAC_RSS_UDPIP6EX_HASH_EN BIT(1)
#define GMAC_RSS_EN		  BIT(0)

#define GMAC_RSS_TBL_ENT7(x) ((x & 0xf) << 28)
#define GMAC_RSS_TBL_ENT6(x) ((x & 0xf) << 24)
#define GMAC_RSS_TBL_ENT5(x) ((x & 0xf) << 20)
#define GMAC_RSS_TBL_ENT4(x) ((x & 0xf) << 16)
#define GMAC_RSS_TBL_ENT3(x) ((x & 0xf) << 12)
#define GMAC_RSS_TBL_ENT2(x) ((x & 0xf) << 8)
#define GMAC_RSS_TBL_ENT1(x) ((x & 0xf) << 4)
#define GMAC_RSS_TBL_ENT0(x) ((x & 0xf) << 0)

#define GMAC_TSO_MID_FLAG(x) ((x & 0xfff) << 16)
#define GMAC_TSO_FST_FLAG(x) ((x & 0xfff) << 0)

#define GMAC_TSO_LST_FLAG(x) ((x & 0xfff) << 0)

#define GMAC_TQ_BUF_CTRL_TIMEOUT(x) ((x & 0xffff) << 16)
#define GMAC_TQ_BUF_CTRL_DES_THR(x) ((x & 0xf) << 12)
#define GMAC_TQ_BUF_CTRL_BUF_THR(x) ((x & 0xf) << 8)
#define GMAC_TQ_BUF_CTRL_BUF_SZ(x)  ((x & 0xff) << 0)

#define GMAC_RQ_BUF_CTRL_TIMEOUT(x) ((x & 0xffff) << 16)
#define GMAC_RQ_BUF_CTRL_DES_THR(x) ((x & 0xf) << 12)
#define GMAC_RQ_BUF_CTRL_BUF_THR(x) ((x & 0xf) << 8)
#define GMAC_RQ_BUF_CTRL_BUF_SZ(x)  ((x & 0xff) << 0)

#define GMAC_TQ_WB_THR(x)     ((x & 0xf) << 16)
#define GMAC_TQ_WB_TIMEOUT(x) ((x & 0xffff) << 0)

#define GMAC_RQ_WB_THR(x)     ((x & 0xf) << 16)
#define GMAC_RQ_WB_TIMEOUT(x) ((x & 0xffff) << 0)

#define GMAC_QPAIR_CTRL_SYN_EN	BIT(5)
#define GMAC_QPAIR_CTRL_RX_EN	BIT(4)
#define GMAC_QPAIR_CTRL_TX_EN	BIT(3)
#define GMAC_QPAIR_CTRL_PRIO(x) ((x & 0x7) << 0)

#define GMAC_QISR_TX_PKT_LOST BIT(3)
#define GMAC_QISR_TX_PKT_CMPL BIT(2)
#define GMAC_QISR_RX_PKT_LOST BIT(1)
#define GMAC_QISR_RX_PKT_CMPL BIT(0)

#define GMAC_INT_UNIT_1	 (1)
#define GMAC_INT_UNIT_4	 (4)
#define GMAC_INT_UNIT_16 (16)
#define GMAC_INT_UNIT_64 (64)

#define GMAC_TXINT_CYC_BIG_UNIT BIT(16)
#define GMAC_TXINT_CYC_CNT(x)	((x & 0xff) << 8)
#define GMAC_TXINT_THR(x)	((x & 0x7) << 4)
#define GMAC_TXINT_UNIT(x)	((x & 0x3) << 0)

#define GMAC_RXINT_RST(x)	((x & 0xff) << 20)
#define GMAC_RXINT_CYC_BIG_UNIT BIT(16)
#define GMAC_RXINT_CYC_CNT(x)	((x & 0xff) << 8)
#define GMAC_RXINT_THR(x)	((x & 0x7) << 4)
#define GMAC_RXINT_UNIT(x)	((x & 0x3) << 0)

#define GMAC_RX_DESC_SZ(x)  ((x & 0xf) << 12)
#define GMAC_RX_DESC_CNT(x) (((x)&0xfff) << 0)

#define GMAC_TX_DESC_SZ(x)  ((x & 0xf) << 12)
#define GMAC_TX_DESC_CNT(x) (((x)&0xfff) << 0)

#define GMAC_TPH_TX_CUPID(x)	  ((x & 0xff) << 18)
#define GMAC_TPH_RX_CUPID(x)	  ((x & 0xff) << 10)
#define GMAC_TPH_PH_DATA(x)	  ((x & 0x3) << 8)
#define GMAC_TPH_PH_DESC(x)	  ((x & 0x3) << 6)
#define GMAC_TPH_RX_DATA_EN	  BIT(5)
#define GMAC_TPH_RX_DESC_WB_EN	  BIT(4)
#define GMAC_TPH_RX_DESC_FETCH_EN BIT(3)
#define GMAC_TPH_TX_DATA_EN	  BIT(2)
#define GMAC_TPH_TX_DESC_WB_EN	  BIT(1)
#define GMAC_TPH_TX_DESC_FETCH_EN BIT(0)

#define GMAC_BMC_EN		 BIT(23)
#define GMAC_BMC_MCST_FLT2_EN	 BIT(22)
#define GMAC_BMC_MCST_FLT1_EN	 BIT(21)
#define GMAC_BMC_MCST_FLT0_EN	 BIT(20)
#define GMAC_BMC_MCST_FLT_EN	 BIT(19)
#define GMAC_BMC_BCST_FLT3_EN	 BIT(18)
#define GMAC_BMC_BCST_FLT2_EN	 BIT(17)
#define GMAC_BMC_BCST_FLT1_EN	 BIT(16)
#define GMAC_BMC_BCST_FLT0_EN	 BIT(15)
#define GMAC_BMC_BCST_FLT_EN	 BIT(14)
#define GMAC_BMC_VLAN_FLT7_EN	 BIT(13)
#define GMAC_BMC_VLAN_FLT6_EN	 BIT(12)
#define GMAC_BMC_VLAN_FLT5_EN	 BIT(11)
#define GMAC_BMC_VLAN_FLT4_EN	 BIT(10)
#define GMAC_BMC_VLAN_FLT3_EN	 BIT(9)
#define GMAC_BMC_VLAN_FLT2_EN	 BIT(8)
#define GMAC_BMC_VLAN_FLT1_EN	 BIT(7)
#define GMAC_BMC_VLAN_FLT0EN	 BIT(6)
#define GMAC_BMC_VLAN_FLT_MOD(x) ((x & 0x3) << 4)
#define GMAC_BMC_MAC_FLT3_EN	 BIT(3)
#define GMAC_BMC_MAC_FLT2_EN	 BIT(2)
#define GMAC_BMC_MAC_FLT1_EN	 BIT(1)
#define GMAC_BMC_MAC_FLT0_EN	 BIT(0)

#define GMAC_BMC_FLT1_VTAG(x) ((x & 0xfff) << 12)
#define GMAC_BMC_FLT0_VTAG(x) ((x & 0xfff) << 0)

#define WAKEUP_FRAME4	(1 << 6)
#define WAKEUP_FRAME3	(1 << 5)
#define WAKEUP_FRAME2	(1 << 4)
#define WAKEUP_FRAME1	(1 << 3)
#define WAKEUP_MAGICPKT (1 << 2)
#define WAKEUP_LINKUP	(1 << 1)
#define WAKEUP_LINKDOWN (1 << 0)

/* TXDES0 */
#define GMAC_TXDES0_TXDMA_OWN	BIT(31)
#define GMAC_TXDES0_FTS		(1 << 18)
#define GMAC_TXDES0_LTS		(1 << 17)
#define GMAC_TXDES0_BUF_SIZE(x) ((x & 0x1ffff) << 0)

#define GMAC_TXDES0_CRC_ERR (1 << 19)
#define GMAC_TXDES0_EDOTR   (1 << 15)

#define FRAME_RUNT	  (1 << 2)
#define FRAME_WHOLE	  (GMAC_TXDES0_FTS | GMAC_TXDES0_LTS)
#define FRAME_FIRST_PART  GMAC_TXDES0_FTS
#define FRAME_MIDDLE_PART 0
#define FRAME_LAST_PART	  GMAC_TXDES0_LTS

/* TXDES1 */
#define GMAC_TXDES1_PKT_BMC   BIT(8)
#define GMAC_TXDES1_INC_IP_ID BIT(7)
#define GMAC_TXDES1_TSO_EN    BIT(6)
#define GMAC_TXDES1_PKT_LLC   BIT(5)
#define GMAC_TXDES1_IPCS_EN   BIT(4)
#define GMAC_TXDES1_UDPCS_EN  BIT(3)
#define GMAC_TXDES1_TCPCS_EN  BIT(2)
#define GMAC_TXDES1_STAG_INS  BIT(1)
#define GMAC_TXDES1_CTAG_INS  BIT(0)

/* TXDES2*/
#define GMAC_TXDES2_STAG(x) ((x & 0xffff) << 16)
#define GMAC_TXDES2_CTAG(x) ((x & 0xffff) << 0)

#define GMAC_TXDES1_DEFAULT (GMAC_TXDES1_TXIC) //| GMAC_TXDES1_TX2FIC)

/* TXDES3 */
#define GMAC_TXDES3_TXBUF_BADRH(x) ((u32)(((u64)x) >> 32) & 0xffff)

/* TXDES4 */
#define GMAC_TXDES4_TXBUF_BADRL(x) (((u64)x) & 0xffffffff)

/* TXDES5 */
#define GMAC_TXDES5_L4_LEN(x) ((x & 0xff) << 16)
#define GMAC_TXDES5_L3_LEN(x) ((x & 0x1ff) << 7)
#define GMAC_TXDES5_L2_LEN(x) ((x & 0x7f) << 0)

/* TXDES6 */
#define GMAC_TXDES6_MSS(x) ((x & 0xffff) << 0)

/* TXDES8 */
#define GMAC_TXDES8_SA_IDX(x)	 ((x & 0xffff) << 1)
#define GMAC_TXDES8_TRAIL_LEN(x) ((x & 0xffff) << 11)
#define GMAC_TXDES8_PROTO_ESP	 BIT(21)
#define GMAC_TXDES8_PROTO_ENC	 BIT(20)
#define GMAC_TXDES8_IPSEC_PKT	 BIT(0)

/* RXDES0 */
#define GMAC_RXDES0_RXDMA_OWN	 BIT(31)
#define GMAC_RXDES0_FRS		 BIT(25)
#define GMAC_RXDES0_LRS		 BIT(24)
#define GMAC_RXDES0_PAUSE_FRAME	 BIT(23)
#define GMAC_RXDES0_PAUSE_OPCODE BIT(22)
#define GMAC_RXDES0_FIFO_FULL	 BIT(21)
#define GMAC_RXDES0_RX_ODD_NB	 BIT(20)
#define GMAC_RXDES0_RUNT	 BIT(19)
#define GMAC_RXDES0_FTL		 BIT(18)
#define GMAC_RXDES0_CRC_ERR	 BIT(17)
#define GMAC_RXDES0_RX_ERR	 BIT(16)
#define GMAC_RXDES0_BROADCAST	 BIT(15)
#define GMAC_RXDES0_MULTICAST	 BIT(14)
#define GMAC_RXDES0_VDBC(x)	 ((x & 0x3fff) << 0)

/* RXDES1 */
#define GMAC_RXDES1_IPSEC_PROTO_AH  BIT(15)
#define GMAC_RXDES1_BMC_PKT	    BIT(14)
#define GMAC_RXDES1_TS_TYPE(x)	    (((x) >> 12) & 0x3)
#define GMAC_RXDES1_TS_NO_TMSTMP    0x1
#define GMAC_RXDES1_TS_EVENT_TMSTMP 0x2
#define GMAC_RXDES1_TS_PEER_TMSTMP  0x3
#define GMAC_RXDES1_IPCS_FAIL	    BIT(11)
#define GMAC_RXDES1_UDPCS_FAIL	    BIT(10)
#define GMAC_RXDES1_TCPCS_FAIL	    BIT(9)
#define GMAC_RXDES1_DF		    BIT(8)
#define GMAC_RXDES1_LLC_PKT	    BIT(7)
#define GMAC_RXDES1_SNAP_PKT	    BIT(6)
#define GMAC_RXDES1_UDP_TYPE	    BIT(5)
#define GMAC_RXDES1_TCP_TYPE	    BIT(4)
#define GMAC_RXDES1_IP6_TYPE	    BIT(3)
#define GMAC_RXDES1_IP4_TYPE	    BIT(2)
#define GMAC_RXDES1_IPSEC_PKT	    BIT(1)
#define GMAC_RXDES1_L2_TYPE	    BIT(0)

/* RXDES2 */
#define GMAC_RXDES2_RXBUF_BADRH(x) ((u32)((u64)x >> 32) & 0xffff)

/* RXDES3 */
#define GMAC_RXDES3_RXBUF_BADRL(x) (((u64)x) & 0xffffffff)

/* RXDES4 */
#define GMAC_RXDES4_STAG_AVA BIT(16)
#define GMAC_RXDES4_STAG(x)  ((x & 0xffff) << 0)

/* RXDES5 */
#define GMAC_RXDES5_CTAG_AVA BIT(16)
#define GMAC_RXDES5_CTAG(x)  ((x & 0xffff) << 0)

/* RXDES6 */
#define GMAC_RXDES6_RSS_TYPE(x) ((x & 0xf) << 0)

/* RXDES8 */
#define GMAC_RXDES8_IPSEC_SECP BIT(0)


#endif /* _GMAC_REGMAP_H_ */