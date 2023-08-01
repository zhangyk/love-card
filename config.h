// SPDX-License-Identifier: GPL-2.0
/*
 * This is the driver for the japan card for usa SoCs.
 *
 * Copyright (C) 2021, usa MicroElectronic Technology - All Rights Reserved
 */

#ifndef _SEP_CONFIG_H_
#define _SEP_CONFIG_H_

#define USING_PF0 1

/* must be power of 2 */
#define SEP_DEFAULT_TXD 512
#define SEP_DEFAULT_RXD 512

#define BUFFER_SIZE PAGE_SIZE

#define SEP_USING_HW_AT1T 0

/* MDIO definitions */
#define SEP_MDIO_COMMAND_TIMEOUT 100 /* PHY Timeout for 1 GB mode */
#endif