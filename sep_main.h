// SPDX-License-Identifier: GPL-2.0
/*
 * This is the driver for the japan card for usa SoCs.
 *
 * Copyright (C) 2021, usa MicroElectronic Technology - All Rights Reserved
 */

#ifndef _SEP_MAIN_H_
#define _SEP_MAIN_H_

#include "cmd.h"
#include "sep_type.h"
#include "gmac_regmap.h"

#define DRV_VERSION "0.0.1"
#define DEV_NAME    "usa_sep_card"

#define PHY_SUPPORTED_10baseT_Half	(1 << 0)
#define PHY_SUPPORTED_10baseT_Full	(1 << 1)
#define PHY_SUPPORTED_100baseT_Half	(1 << 2)
#define PHY_SUPPORTED_100baseT_Full	(1 << 3)
#define PHY_SUPPORTED_1000baseT_Half	(1 << 4)
#define PHY_SUPPORTED_1000baseT_Full	(1 << 5)
#define PHY_SUPPORTED_Autoneg		(1 << 6)
#define PHY_SUPPORTED_TP		(1 << 7)
#define PHY_SUPPORTED_AUI		(1 << 8)
#define PHY_SUPPORTED_MII		(1 << 9)
#define PHY_SUPPORTED_FIBRE		(1 << 10)
#define PHY_SUPPORTED_BNC		(1 << 11)
#define PHY_SUPPORTED_10000baseT_Full	(1 << 12)
#define PHY_SUPPORTED_Pause		(1 << 13)
#define PHY_SUPPORTED_Asym_Pause	(1 << 14)
#define PHY_SUPPORTED_2500baseX_Full	(1 << 15)
#define PHY_SUPPORTED_Backplane		(1 << 16)
#define PHY_SUPPORTED_1000baseKX_Full	(1 << 17)
#define PHY_SUPPORTED_10000baseKX4_Full (1 << 18)
#define PHY_SUPPORTED_10000baseKR_Full	(1 << 19)
#define PHY_SUPPORTED_10000baseR_FEC	(1 << 20)
#define PHY_SUPPORTED_1000baseX_Half	(1 << 21)
#define PHY_SUPPORTED_1000baseX_Full	(1 << 22)

/* Indicates what features are advertised by the interface. */
#define PHY_ADVERTISED_10baseT_Half	 (1 << 0)
#define PHY_ADVERTISED_10baseT_Full	 (1 << 1)
#define PHY_ADVERTISED_100baseT_Half	 (1 << 2)
#define PHY_ADVERTISED_100baseT_Full	 (1 << 3)
#define PHY_ADVERTISED_1000baseT_Half	 (1 << 4)
#define PHY_ADVERTISED_1000baseT_Full	 (1 << 5)
#define PHY_ADVERTISED_Autoneg		 (1 << 6)
#define PHY_ADVERTISED_TP		 (1 << 7)
#define PHY_ADVERTISED_AUI		 (1 << 8)
#define PHY_ADVERTISED_MII		 (1 << 9)
#define PHY_ADVERTISED_FIBRE		 (1 << 10)
#define PHY_ADVERTISED_BNC		 (1 << 11)
#define PHY_ADVERTISED_10000baseT_Full	 (1 << 12)
#define PHY_ADVERTISED_Pause		 (1 << 13)
#define PHY_ADVERTISED_Asym_Pause	 (1 << 14)
#define PHY_ADVERTISED_2500baseX_Full	 (1 << 15)
#define PHY_ADVERTISED_Backplane	 (1 << 16)
#define PHY_ADVERTISED_1000baseKX_Full	 (1 << 17)
#define PHY_ADVERTISED_10000baseKX4_Full (1 << 18)
#define PHY_ADVERTISED_10000baseKR_Full	 (1 << 19)
#define PHY_ADVERTISED_10000baseR_FEC	 (1 << 20)
#define PHY_ADVERTISED_1000baseX_Half	 (1 << 21)
#define PHY_ADVERTISED_1000baseX_Full	 (1 << 22)

/* PHY_MII_STAT1000 masks */
#define PHY_MII_1000BTSR_MSCF	0x8000
#define PHY_MII_1000BTSR_MSCR	0x4000
#define PHY_MII_1000BTSR_LRS	0x2000
#define PHY_MII_1000BTSR_RRS	0x1000
#define PHY_MII_1000BTSR_1000FD 0x0800
#define PHY_MII_1000BTSR_1000HD 0x0400


#if defined(SEP_USING_HW_AT1T) && SEP_USING_HW_AT1T == 1
#define SEP_DMA_OFFSET 0
#else
#define SEP_DMA_OFFSET 0x10000000000
#endif


#define MSIX_VECTORS   2
#define MIN_MSIX_COUNT 2


static inline u32 __ior32(struct sep_data *data, unsigned long reg)
{
	return ioread32(data->io_base + reg);
}
#define ior32(reg) __ior32(data, reg)

static inline void __iow32(struct sep_data *data, unsigned long reg, u32 val)
{
	iowrite32(val, data->io_base + reg);
}
#define iow32(reg, val) __iow32(data, reg, val)

int seq_dbg_adapter_init(struct sep_data *data);
void seq_dbg_adapter_exit(struct sep_data *data);
int seq_dbgfs_init(const char *name);
void seq_dbgfs_exit(void);

u16 gmac_mdio_read(struct sep_data *data, s32 regnum);
s32 gmac_mdio_write(struct sep_data *data, s32 regnum, u16 val);
int gphy_get_link_status(struct sep_data *data);


#endif /* _SEP_MAIN_H_ */