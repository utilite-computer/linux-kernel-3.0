/*
 * scf0403.h -- Platform glue for DataImage SFC0403852GGU04 LCD
 *
 * Copyright (c) 2009 Alberto Panizzo <maramaopercheseimorto@gmail.com>
 * Copyright (c) 2012 CompuLab, Ltd
 *           Dmitry Lifshitz <lifshitz@compulab.co.il>
 *           Ilya Ledvich <ilya@compulab.co.il>
 *
 * * Based on Marek Vasut work in lms283gf05.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.
*/

#ifndef __LINUX_SPI_SCF0403_H
#define __LINUX_SPI_SCF0403_H

struct scf0403_pdata {
	unsigned int	reset_gpio;
};

#endif /* __LINUX_SPI_SCF0403_H */

