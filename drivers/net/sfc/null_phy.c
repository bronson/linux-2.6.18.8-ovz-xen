/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2006-2008: Solarflare Communications Inc,
 *                      9501 Jeronimo Road, Suite 250,
 *                      Irvine, CA 92618, USA
 *
 * Developed by Solarflare Communications <linux-net-drivers@solarflare.com>
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

#include "efx.h"
#include "falcon.h"
#include "gmii.h"
#include "phy.h"

static int falcon_null_phy_check_hw(struct efx_nic *efx)
{
	int link_ok = falcon_xaui_link_ok(efx);

	/* Generate PHY event that a PHY would have generated */
	if (link_ok != efx->link_up) {
		efx->link_up = link_ok;
		efx->mac_op->fake_phy_event(efx);
	}

	return 0;
}

static void falcon_null_phy_reconfigure(struct efx_nic *efx)
{
	/* CX4 is always 10000FD only */
	efx->link_options = GM_LPA_10000FULL;

	falcon_null_phy_check_hw(efx);
}

struct efx_phy_operations falcon_null_phy_ops = {
	.reconfigure     = falcon_null_phy_reconfigure,
	.check_hw        = falcon_null_phy_check_hw,
	.fini            = efx_port_dummy_op_void,
	.clear_interrupt = efx_port_dummy_op_void,
	.init            = efx_port_dummy_op_int,
	.reset_xaui      = efx_port_dummy_op_void,
	.mmds            = 0,
	.loopbacks       = 0,
	.startup_loopback = 0,
};
