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

#ifndef EFX_PHY_H
#define EFX_PHY_H

/****************************************************************************
 * 10Xpress (SFX7101) PHY
 */
extern struct efx_phy_operations falcon_tenxpress_phy_ops;

enum tenxpress_state {
	TENXPRESS_STATUS_OFF = 0,
	TENXPRESS_STATUS_OTEMP = 1,
	TENXPRESS_STATUS_NORMAL = 2,
};

extern void tenxpress_set_state(struct efx_nic *efx,
				enum tenxpress_state state);
extern void tenxpress_phy_blink(struct efx_nic *efx, int blink);
extern void tenxpress_crc_err(struct efx_nic *efx);

/****************************************************************************
 * Marvell 88E1111 "Alaska" PHY control
 */
extern struct efx_phy_operations alaska_phy_operations;

/****************************************************************************
* Exported functions from the driver for Transwitch CX4 retimer
*/
extern struct efx_phy_operations falcon_txc_phy_ops;

#define TXC_GPIO_DIR_INPUT  (0)
#define TXC_GPIO_DIR_OUTPUT (1)

extern void txc_set_gpio_dir(struct efx_nic *p, int pin, int dir);
extern void txc_set_gpio_val(struct efx_nic *p, int pin, int val);

/****************************************************************************
 * Exported functions from the driver for PMC PM8358 PHY
 */
extern struct efx_phy_operations falcon_pm8358_phy_ops;

/****************************************************************************
 * Exported functions from the driver for XFP optical PHYs
 */
extern struct efx_phy_operations falcon_xfp_phy_ops;

/* The QUAKE XFP PHY provides various H/W control states for LEDs */
#define QUAKE_LED_LINK_INVAL	(0)
#define QUAKE_LED_LINK_STAT	(1)
#define QUAKE_LED_LINK_ACT	(2)
#define QUAKE_LED_LINK_ACTSTAT	(3)
#define QUAKE_LED_OFF		(4)
#define QUAKE_LED_ON		(5)
#define QUAKE_LED_LINK_INPUT	(6)	/* Pin is an input. */
/* What link the LED tracks */
#define QUAKE_LED_TXLINK	(0)
#define QUAKE_LED_RXLINK	(8)

extern void xfp_set_led(struct efx_nic *p, int led, int state);

/****************************************************************************
 * NULL PHY ops
 */
extern struct efx_phy_operations falcon_null_phy_ops;

#endif
