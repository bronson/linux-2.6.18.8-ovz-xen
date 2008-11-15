/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file provides public interface of efrm library -- resource handling.
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

#ifndef __CI_EFRM_RESOURCE_H__
#define __CI_EFRM_RESOURCE_H__

/*--------------------------------------------------------------------
 *
 * headers for type dependencies
 *
 *--------------------------------------------------------------------*/

#include <ci/efhw/efhw_types.h>
#include <ci/efrm/resource_id.h>
#include <ci/efrm/sysdep.h>
#include <ci/efhw/common_sysdep.h>

#ifndef __ci_driver__
#error "Driver-only file"
#endif

/*--------------------------------------------------------------------
 *
 * struct efrm_resource - represents an allocated resource
 *                   (eg. pinned pages of memory, or resource on a NIC)
 *
 *--------------------------------------------------------------------*/

/*! Representation of an allocated resource */
struct efrm_resource {
	atomic_t rs_ref_count; /*!< users count; see
				* __efrm_resource_ref_count_zero() */
	efrm_resource_handle_t rs_handle;
};

/*--------------------------------------------------------------------
 *
 * managed resource abstraction
 *
 *--------------------------------------------------------------------*/

/*! Factory for resources of a specific type */
struct efrm_resource_manager {
	const char *rm_name;	/*!< human readable only */
	spinlock_t rm_lock;
#ifndef NDEBUG
	unsigned rm_type;
#endif
	int rm_resources;
	int rm_resources_hiwat;
	/*! table of allocated resources */
	struct efrm_resource **rm_table;
	unsigned rm_table_size;
	/**
	 * Destructor for the resource manager. Other resource managers
	 * might be already dead, although the system guarantees that
	 * managers are destructed in the order by which they were created
	 */
	void (*rm_dtor)(struct efrm_resource_manager *);
};

#ifdef NDEBUG
# define EFRM_RESOURCE_ASSERT_VALID(rs, rc_mbz)
# define EFRM_RESOURCE_MANAGER_ASSERT_VALID(rm)
#else
/*! Check validity of resource and report on failure */
extern void efrm_resource_assert_valid(struct efrm_resource *,
				       int rc_may_be_zero,
				       const char *file, int line);
# define EFRM_RESOURCE_ASSERT_VALID(rs, rc_mbz) \
	efrm_resource_assert_valid((rs), (rc_mbz), __FILE__, __LINE__)

/*! Check validity of resource manager and report on failure */
extern void efrm_resource_manager_assert_valid(struct efrm_resource_manager *,
					       const char *file, int line);
# define EFRM_RESOURCE_MANAGER_ASSERT_VALID(rm) \
	efrm_resource_manager_assert_valid((rm), __FILE__, __LINE__)
#endif

/*! Check the reference count on the resource provided and delete its
 *  handle it in its owning resource manager if the
 *  reference count has fallen to zero.
 *
 *  Returns TRUE if the caller should really free the resource.
 */
extern bool __efrm_resource_ref_count_zero(unsigned type, unsigned instance);

#endif /* __CI_EFRM_RESOURCE_H__ */
