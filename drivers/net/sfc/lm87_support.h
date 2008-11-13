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
 ****************************************************************************/

#ifndef EFX_LM87_SUPPORT_H
#define EFX_LM87_SUPPORT_H

/* The interrupt bit masks. These are the same in the interrupt status and
 * interrupt mask registers. */
/* Register 1 bits */
#define EFX_LM87_2_5V_INT	(1)
#define EFX_LM87_VCCP1_INT	(2)
#define EFX_LM87_VCC_INT	(4)
#define EFX_LM87_5_V_INT	(8)
#define EFX_LM87_ITMP_INT	(0x10)
#define EFX_LM87_ETMP_INT	(0x20)
#define EFX_LM87_FAN1_INT	(0x40)
#define EFX_LM87_FAN2_INT	(0x80)
/* Register 2 bits */
#define EFX_LM87_12V_INT	(0x100)
#define EFX_LM87_VCCP2_INT	(0x200)
/* Bits 2 and 3 are reserved. */
#define EFX_LM87_CI_INT		(0x1000)
#define EFX_LM87_THERM_INT	(0x2000)
#define EFX_LM87_D1_INT		(0x4000)
#define EFX_LM87_D2_INT		(0x8000)

#define EFX_LM87_NO_INTS	((u16)-1)

extern
int efx_probe_lm87(struct efx_nic *efx, int addr, const u8 *limits,
		   int nlimits, const u16 irqmask);

extern void efx_remove_lm87(struct efx_nic *efx);

extern int efx_check_lm87(struct efx_nic *efx, unsigned mask);

#endif /* EFX_LM87_SUPPORT_H */
