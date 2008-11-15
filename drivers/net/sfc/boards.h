/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2007:      Solarflare Communications Inc,
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

#ifndef EFX_BOARDS_H
#define EFX_BOARDS_H

/* Board IDs (must fit in 8 bits). Note that 0 must never be assigned because
 * on early boards it means there is no revision info. Board types pre 400x
 * are not covered here, but this is not a problem because:
 * - the early Falcon boards (FPGA, 401, 403) don't have any extra H/W we
 * need care about and aren't being updated.
 */
enum efx_board_type {
	EFX_BOARD_INVALID = 0, /* Early boards do not have board rev. info. */
	EFX_BOARD_SFE4001 = 1,
	EFX_BOARD_SFE4002 = 2,
	EFX_BOARD_SFE4003 = 3,
	EFX_BOARD_SFE4005 = 4,
	/* Insert new types before here */
	EFX_BOARD_MAX
};

extern int efx_set_board_info(struct efx_nic *efx, u16 revision_info);

/* SFE4001 (10GBASE-T) */
extern int sfe4001_poweron(struct efx_nic *efx);
extern void sfe4001_poweroff(struct efx_nic *efx);

#endif
