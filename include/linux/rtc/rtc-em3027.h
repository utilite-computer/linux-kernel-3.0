/*
 * Platform data for the EM3027 I2C RTC driver
 *
 * Copyright (C) 2013 by CompuLab ltd
 * Author: Andrey Gelman <andrey.gelman@compulab.co.il>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_EM3027_H
#define _LINUX_EM3027_H

/**
 * Trickle charger current limiting resistors selection
 * The bitmasks may be or-ed to set up more than one resistor
 * in parallel.
 */
#define EM3027_TRICKLE_CHARGER_80K	0x80
#define EM3027_TRICKLE_CHARGER_20K	0x40
#define EM3027_TRICKLE_CHARGER_5K	0x20
#define EM3027_TRICKLE_CHARGER_1_5K	0x10

struct em3027_platform_data {
	int charger_resistor_sel;
};

#endif	/* _LINUX_EM3027_H */
