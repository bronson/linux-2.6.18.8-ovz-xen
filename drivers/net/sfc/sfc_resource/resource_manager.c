/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file contains generic code for resources and resource managers.
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

#include <ci/efrm/debug.h>
#include <ci/efrm/nic_table.h>
#include <ci/efhw/iopage.h>
#include <ci/efrm/driver_private.h>

/**********************************************************************
 * Internal stuff.
 */

#define EFRM_RM_TABLE_SIZE_INIT 256

static int grow_table(struct efrm_resource_manager *rm, unsigned min_size)
{
	irq_flags_t lock_flags;
	struct efrm_resource **table, **old_table;
	unsigned new_size;

	EFRM_RESOURCE_MANAGER_ASSERT_VALID(rm);

	spin_lock_irqsave(&rm->rm_lock, lock_flags);

	/* Check whether the size of the table increased whilst the lock was
	 * dropped. */
	if (min_size <= rm->rm_table_size) {
		spin_unlock_irqrestore(&rm->rm_lock, lock_flags);
		return 0;
	}

	new_size = rm->rm_table_size << 1;
	if (new_size < min_size)
		new_size = min_size;

	spin_unlock_irqrestore(&rm->rm_lock, lock_flags);
	if (in_atomic()) {
		EFRM_WARN("%s: in_atomic in grow_table()", __FUNCTION__);
		EFRM_WARN("%s: allocating %u bytes", __FUNCTION__,
			  (unsigned)(new_size *
				     sizeof(struct efrm_resource *)));
		return -ENOMEM;
	}

	table =
	    (struct efrm_resource **)vmalloc(new_size *
					     sizeof(struct efrm_resource *));
	spin_lock_irqsave(&rm->rm_lock, lock_flags);

	if (table == 0) {
		EFRM_ERR("%s: out of memory in grow_table()", __FUNCTION__);
		EFRM_ERR("%s: allocating %u bytes", __FUNCTION__,
			 (unsigned)(new_size *
				    sizeof(struct efrm_resource *)));
		spin_unlock_irqrestore(&rm->rm_lock, lock_flags);
		return -ENOMEM;
	}

	/* Could have got bigger while we dropped the lock... */
	if (new_size <= rm->rm_table_size) {
		spin_unlock_irqrestore(&rm->rm_lock, lock_flags);
		vfree(table);
		return 0;
	}

	memcpy(table, rm->rm_table, rm->rm_table_size * sizeof(*table));
	memset(table + rm->rm_table_size, 0,
	       sizeof(*table) * (new_size - rm->rm_table_size));
	/* remember old table so we can free the
	   memory after we drop the lock (bug 1040) */
	old_table = rm->rm_table;
	rm->rm_table = table;
	rm->rm_table_size = new_size;
	spin_unlock_irqrestore(&rm->rm_lock, lock_flags);
	vfree(old_table);

	return 0;
}

/**********************************************************************
 * struct efrm_resource_manager
 */

void efrm_resource_manager_dtor(struct efrm_resource_manager *rm)
{
	EFRM_RESOURCE_MANAGER_ASSERT_VALID(rm);

	/* call destructor */
	EFRM_DO_DEBUG(if (rm->rm_resources)
		      EFRM_ERR("%s: %s leaked %d resources",
			       __FUNCTION__, rm->rm_name, rm->rm_resources));
	EFRM_ASSERT(rm->rm_resources == 0);

	rm->rm_dtor(rm);

	/* clear out things built by efrm_resource_manager_ctor */
	spin_lock_destroy(&rm->rm_lock);
	vfree(rm->rm_table);

	/* and the free the memory */
	EFRM_DO_DEBUG(memset(rm, 0, sizeof(*rm)));
	kfree(rm);
}

/* Construct a resource manager.  Resource managers are singletons. */
int
efrm_resource_manager_ctor(struct efrm_resource_manager *rm,
			   void (*dtor)(struct efrm_resource_manager *),
			   const char *name, unsigned type,
			   int initial_table_size)
{
	EFRM_ASSERT(rm);
	EFRM_ASSERT(dtor);

	rm->rm_name = name;
	EFRM_DO_DEBUG(rm->rm_type = type);
	rm->rm_dtor = dtor;
	spin_lock_init(&rm->rm_lock);
	rm->rm_resources = 0;
	rm->rm_resources_hiwat = 0;

	/* if not set then pick a number */
	rm->rm_table_size = (initial_table_size) ?
		initial_table_size : EFRM_RM_TABLE_SIZE_INIT;

	rm->rm_table = vmalloc(rm->rm_table_size *
			       sizeof(struct efrm_resource *));

	if (rm->rm_table == 0) {
		spin_lock_destroy(&rm->rm_lock);
		return -ENOMEM;
	}
	memset(rm->rm_table, 0, sizeof(*rm->rm_table) * rm->rm_table_size);

	EFRM_RESOURCE_MANAGER_ASSERT_VALID(rm);
	return 0;
}

int efrm_resource_manager_insert(struct efrm_resource *rs)
{
	irq_flags_t lock_flags;
	struct efrm_resource_manager *rm;
	int instance = EFRM_RESOURCE_INSTANCE(rs->rs_handle);

	EFRM_ASSERT(EFRM_RESOURCE_TYPE(rs->rs_handle) < EFRM_RESOURCE_NUM);
	rm = efrm_rm_table[EFRM_RESOURCE_TYPE(rs->rs_handle)];
	EFRM_ASSERT(EFRM_RESOURCE_TYPE(rs->rs_handle) == rm->rm_type);
	EFRM_RESOURCE_ASSERT_VALID(rs, 0);

	/* Put an entry in the resource table. */
	spin_lock_irqsave(&rm->rm_lock, lock_flags);
	if ((unsigned)instance >= rm->rm_table_size) {
		spin_unlock_irqrestore(&rm->rm_lock, lock_flags);
		if (grow_table(rm, instance + 1) < 0)
			return -ENOMEM;
		spin_lock_irqsave(&rm->rm_lock, lock_flags);
	}
	EFRM_ASSERT(rm->rm_table_size > (unsigned)instance);
	EFRM_ASSERT(rm->rm_table[instance] == NULL);
	rm->rm_table[instance] = rs;
	rm->rm_resources++;
	if (rm->rm_resources > rm->rm_resources_hiwat)
		rm->rm_resources_hiwat = rm->rm_resources;

	/* Put the resource in the linked list. */
	/* ?? broken list_add(&rm->rm_resources, &rs->rs_link); */
	/* DJR wrote that it causes problem on driver unload, and DR tried
	 * it and saw (probably) this cause an assertion failure due to a
	 * bad link structure in
	 * /runbench/results/2005/09/22/0_DupTester_15-16-46 */

	spin_unlock_irqrestore(&rm->rm_lock, lock_flags);

	return 0;
}

bool __efrm_resource_ref_count_zero(unsigned type, unsigned instance)
{
	/* This is rather nasty because when a resource's ref count goes to
	 * zero there is still a pointer to it in the [rm_table].  Thus
	 * arriving here does not guarantee that we have exclusive access
	 * to the resource and can free it.  In fact the resource may
	 * already have been freed by another thread (after we dropped our
	 * ref, but before arriving here).
	 *
	 * At this point the only pointers to this resource should be [rs]
	 * and the one in [rm_table].  EXCEPT: Someone could have got in
	 * and looked-up the resource in the table before we got the lock.
	 * In this case the ref will have been hiked again.
	 *
	 * Therefore, if ref count is non-zero here, we shouldn't do
	 * anything, as someone else holds a ref to the resource, and will
	 * eventually release it.
	 *
	 * Otherwise, we zero-out the table entry.  Therefore we have the
	 * only pointer to the resource, and can kill it safely.
	 */
	struct efrm_resource_manager *rm = efrm_rm_table[type];
	irq_flags_t lock_flags;
	struct efrm_resource *rs;
	bool do_free = false;

	EFRM_TRACE("efrm_resource_ref_count_zero: type=%d instance=%d",
		   rm->rm_type, instance);

	EFRM_RESOURCE_MANAGER_ASSERT_VALID(rm);
	EFRM_ASSERT(rm->rm_table_size > instance);

	spin_lock_irqsave(&rm->rm_lock, lock_flags);

	rs = rm->rm_table[instance];
	if (rs != NULL) {
		do_free = atomic_read(&rs->rs_ref_count) == 0;
		if (do_free) {
			EFRM_ASSERT(rm->rm_resources > 0);
			--rm->rm_resources;
			rm->rm_table[instance] = 0;
		}
	}

	spin_unlock_irqrestore(&rm->rm_lock, lock_flags);

	return do_free;
}
EXPORT_SYMBOL(__efrm_resource_ref_count_zero);

/*
 * vi: sw=8:ai:aw
 */
