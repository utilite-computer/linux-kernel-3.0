/*
 * This file is part of igb CompuLab Implementation
 *
 */

#ifndef _LINUX_IGB_H
#define _LINUX_IGB_H

#include <linux/if_ether.h>

struct igb_platform_data {
	u8 mac_address[ETH_ALEN];
};

int igb_set_platform_data(const struct igb_platform_data *data);
struct igb_platform_data *igb_get_platform_data(void);

#endif
