/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2005:      Fen Systems Ltd.
 * Copyright 2006:      Solarflare Communications Inc,
 *                      9501 Jeronimo Road, Suite 250,
 *                      Irvine, CA 92618, USA
 *
 * Initially developed by Michael Brown <mbrown@fensystems.co.uk>
 * Maintained by Solarflare Communications <linux-net-drivers@solarflare.com>
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

#ifndef EFX_ETHTOOL_H
#define EFX_ETHTOOL_H

#include "net_driver.h"

/*
 * Ethtool support
 */

extern int efx_ethtool_get_settings(struct net_device *net_dev,
				    struct ethtool_cmd *ecmd);
extern int efx_ethtool_set_settings(struct net_device *net_dev,
				    struct ethtool_cmd *ecmd);

extern struct ethtool_ops efx_ethtool_ops;

#endif /* EFX_ETHTOOL_H */
