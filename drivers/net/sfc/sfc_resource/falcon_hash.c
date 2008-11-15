/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file contains EtherFabric NIC hash algorithms implementation.
 *
 * Copyright 2005-2007: Solarflare Communications Inc,
 *                      9501 Jeronimo Road, Suite 250,
 *                      Irvine, CA 92618, USA
 *
 * Developed and maintained by Solarflare Communications:
 *                      <linux-xen-drivers@solarflare.com>
 *                      <onload-dev@solarflare.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation, incorporated herein by reference.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 ****************************************************************************
 */

#include <ci/efhw/debug.h>
#include <ci/driver/efab/hardware.h>


/* this mask is per filter bank hence /2 */
#define FILTER_MASK(n)  ((n) / 2u - 1u)

/*
 *  Main Functions related to the Hash Table Generation
 *  Author: Srinivasaih, Nataraj
 * Created: Thu May 13:32:41 PDT 2004
 * $Id: falcon_hash.c,v 1.20 2008/01/29 08:28:56 ok_sasha Exp $
 */
/***************************************************************************
Class	Maximum number of	Valid address ranges
	hosts per network
A	16777214		1.0.0.1 through 9.255.255.254
				11.0.0.1 through 126.255.255.254
B	65534			128.0.0.1 through 172.15.255.254
				172.32.0.1 through 191.255.255.254
C	254			192.0.0.1 through 192.167.255.254
				192.169.0.1 through 223.255.255.254
P	16777214		10.0.0.1 through 10.255.255.254 (10/8)
	1048574			172.16.0.1 through 172.31.255.254 (172.16/12)
	65534			192.168.0.1 through 192.168.255.254 (192.168/16)

R	-			0.0.0.0 through 0.255.255.255
				(used if host will be assigned a
				valid address dynamically)
				127.0.0.0 through 127.255.255.255
				(loopback addresses)

P : Private internets only
R : Reserved
****************************************************************************/

/* All LE parameters */
unsigned int
falcon_hash_get_key(unsigned int src_ip, unsigned int src_port,
		    unsigned int dest_ip, unsigned int dest_port,
		    int tcp, int full)
{

	unsigned int result = 0;
	int net_type;

	EFHW_ASSERT(tcp == 0 || tcp == 1);
	EFHW_ASSERT(full == 0 || full == 1);

	net_type = tcp << 4 | full;

	/* Note that src_ip and src_port ignored if a wildcard filter */
	switch (net_type) {
	case 0x01:		/* UDP Full */
		result = ((dest_ip & 0xfffffffe) | (!(dest_ip & 1))) ^
		    (((dest_port << 16) & 0xFFFF0000) |
		     ((src_ip >> 16) & 0x0000FFFF)) ^
		    (((src_ip << 16) & 0xFFFF0000) |
		     ((src_port & 0x0000FFFF)));
		EFHW_TRACE("falcon_hash_get_key: UDP Full %x", result);
		break;
	case 0x00:		/* UDP Wild Card */
		result = ((dest_ip & 0xfffffffe) | (!(dest_ip & 1))) ^
		    (((dest_port << 16) & 0x00000000) |
		     ((src_ip >> 16) & 0x00000000)) ^
		    (((src_ip << 16) & 0x00000000) |
		     ((dest_port & 0x0000FFFF)));
		EFHW_TRACE("falcon_hash_get_key: UDP Wildcard %x", result);
		break;
	case 0x10:		/* TCP Wild Card */
		result = (dest_ip) ^
		    (((dest_port << 16) & 0xFFFF0000) |
		     ((src_ip >> 16) & 0x00000000)) ^
		    (((src_ip << 16) & 0x00000000) |
		     ((src_port & 0x00000000)));
		EFHW_TRACE("falcon_hash_get_key: TCP Wildcard %x", result);
		break;
	case 0x11:		/* TCP Full */
		result = (dest_ip) ^
		    (((dest_port << 16) & 0xFFFF0000) |
		     ((src_ip >> 16) & 0x0000FFFF)) ^
		    (((src_ip << 16) & 0xFFFF0000) |
		     ((src_port & 0x0000FFFF)));
		EFHW_TRACE("falcon_hash_get_key: TCP Full %x", result);
		break;
	default:
		EFHW_ASSERT(0);

	}
	return (result);
}

/* This function generates the First Hash key */
unsigned int falcon_hash_function1(unsigned int key, unsigned int nfilters)
{

	unsigned short int lfsr_reg;
	unsigned int tmp_key;
	int index;

	unsigned short int lfsr_input;
	unsigned short int single_bit_key;
	unsigned short int bit16_lfsr;
	unsigned short int bit3_lfsr;

	lfsr_reg = 0xFFFF;
	tmp_key = key;

	/* For Polynomial equation X^16+X^3+1 */
	for (index = 0; index < 32; index++) {
		/* Get the bit from key and shift the key */
		single_bit_key = (tmp_key & 0x80000000) >> 31;
		tmp_key = tmp_key << 1;

		/* get the Tap bits to XOR operation */
		bit16_lfsr = (lfsr_reg & 0x8000) >> 15;
		bit3_lfsr = (lfsr_reg & 0x0004) >> 2;

		/* Get the Input value to the LFSR */
		lfsr_input = ((bit16_lfsr ^ bit3_lfsr) ^ single_bit_key);

		/* Shift and store out of the two TAPs */
		lfsr_reg = lfsr_reg << 1;
		lfsr_reg = lfsr_reg | (lfsr_input & 0x0001);

	}

	lfsr_reg = lfsr_reg & FILTER_MASK(nfilters);

	return lfsr_reg;
}

/* This function generates the Second Hash */
unsigned int
falcon_hash_function2(unsigned int key, unsigned int nfilters)
{
	return (unsigned int)(((unsigned long long)key * 2 - 1) &
			      FILTER_MASK(nfilters));
}

/* This function iterates through the hash table */
unsigned int
falcon_hash_iterator(unsigned int hash1, unsigned int hash2,
		     unsigned int n_search, unsigned int nfilters)
{
	return ((hash1 + (n_search * hash2)) & FILTER_MASK(nfilters));
}
