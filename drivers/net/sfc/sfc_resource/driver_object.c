/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file contains support for the global driver variables.
 *
 * Copyright 2005-2007: Solarflare Communications Inc,
 *                      9501 Jeronimo Road, Suite 250,
 *                      Irvine, CA 92618, USA
 *
 * Developed and maintained by Solarflare Communications:
 *                      <linux-xen-drivers@solarflare.com>
 *                      <onload-dev@solarflare.com>
 *
 * Certain parts of the driver were implemented by
 *          Alexandra Kossovsky <Alexandra.Kossovsky@oktetlabs.ru>
 *          OKTET Labs Ltd, Russia,
 *          http://oktetlabs.ru, <info@oktetlabs.ru>
 *          by request of Solarflare Communications
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

#include <ci/efrm/nic_table.h>
#include <ci/efrm/resource.h>
#include <ci/efrm/debug.h>

/* We use #define rather than static inline here so that the Windows
 * "prefast" compiler can see its own locking primitive when these
 * two function are used (and then perform extra checking where they
 * are used)
 *
 * Both macros operate on an irq_flags_t
*/

#define efrm_driver_lock(irqlock_state) \
	spin_lock_irqsave(&efrm_nic_table.lock, irqlock_state)

#define efrm_driver_unlock(irqlock_state)		\
	spin_unlock_irqrestore(&efrm_nic_table.lock,	\
			       irqlock_state);

/* These routines are all methods on the architecturally singleton
   global variables: efrm_nic_table, efrm_rm_table.

   I hope we never find a driver model that does not allow global
   structure variables :) (but that would break almost every driver I've
   ever seen).
*/

/*! Exported driver state */
struct efrm_nic_table efrm_nic_table;
EXPORT_SYMBOL(efrm_nic_table);

/* Internal table with resource managers.
 * We'd like to not export it, but we are still using efrm_rm_table
 * in the char driver. So, it is declared in the private header with
 * a purpose. */
struct efrm_resource_manager *efrm_rm_table[EFRM_RESOURCE_NUM];
EXPORT_SYMBOL(efrm_rm_table);

int efrm_driver_ctor(void)
{
	memset(&efrm_nic_table, 0, sizeof(efrm_nic_table));
	memset(&efrm_rm_table, 0, sizeof(efrm_rm_table));

	spin_lock_init(&efrm_nic_table.lock);

	EFRM_TRACE("%s: driver created", __FUNCTION__);
	return 0;
}

int efrm_driver_dtor(void)
{
	EFRM_ASSERT(!efrm_nic_table_held());

	spin_lock_destroy(&efrm_nic_table.lock);
	EFRM_TRACE("%s: driver deleted", __FUNCTION__);
	return 0;
}

int efrm_driver_register_nic(struct efhw_nic *nic, int nic_index)
{
	int rc = 0;
	irq_flags_t lock_flags;

	EFRM_ASSERT(nic_index >= 0);

	efrm_driver_lock(lock_flags);

	if (efrm_nic_table_held()) {
		EFRM_WARN("%s: driver object is in use", __FUNCTION__);
		rc = -EBUSY;
		goto done;
	}

	if (efrm_nic_table.nic_count == EFHW_MAX_NR_DEVS) {
		EFRM_WARN("%s: filled up NIC table size %d", __FUNCTION__,
			  EFHW_MAX_NR_DEVS);
		rc = -E2BIG;
		goto done;
	}

	EFRM_ASSERT(efrm_nic_table.nic[nic_index] == NULL);
	efrm_nic_table.nic[nic_index] = nic;
	nic->index = nic_index;

	if (efrm_nic_table.a_nic == NULL)
		efrm_nic_table.a_nic = nic;

	efrm_nic_table.nic_count++;
	efrm_driver_unlock(lock_flags);
	return rc;

done:
	efrm_driver_unlock(lock_flags);
	return rc;
}

int efrm_driver_unregister_nic(struct efhw_nic *nic)
{
	int rc = 0;
	int nic_index = nic->index;
	irq_flags_t lock_flags;

	EFRM_ASSERT(nic_index >= 0);

	efrm_driver_lock(lock_flags);

	if (efrm_nic_table_held()) {
		EFRM_WARN("%s: driver object is in use", __FUNCTION__);
		rc = -EBUSY;
		goto done;
	}

	EFRM_ASSERT(efrm_nic_table.nic[nic_index] == nic);

	nic->index = -1;
	efrm_nic_table.nic[nic_index] = NULL;

	--efrm_nic_table.nic_count;

	if (efrm_nic_table.a_nic == nic) {
		if (efrm_nic_table.nic_count == 0) {
			efrm_nic_table.a_nic = NULL;
		} else {
			for (nic_index = 0; nic_index < EFHW_MAX_NR_DEVS;
			     nic_index++) {
				if (efrm_nic_table.nic[nic_index] != NULL)
					efrm_nic_table.a_nic =
					    efrm_nic_table.nic[nic_index];
			}
			EFRM_ASSERT(efrm_nic_table.a_nic);
		}
	}

done:
	efrm_driver_unlock(lock_flags);
	return rc;
}
