/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file contains filters support.
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
#include <ci/driver/efab/hardware.h>
#include <ci/efhw/falcon.h>
#include <ci/efrm/vi_resource_manager.h>
#include <ci/efrm/private.h>
#include <ci/efrm/filter.h>
#include <ci/efrm/buffer_table.h>

struct filter_resource_manager {
	struct efrm_resource_manager rm;
	struct kfifo *free_ids;
};

static struct filter_resource_manager *efrm_filter_manager;

void efrm_filter_resource_free(struct filter_resource *frs)
{
	struct efhw_nic *nic;
	int nic_i;
	int id;

	EFRM_RESOURCE_ASSERT_VALID(&frs->rs, 1);

	EFRM_TRACE("%s: " EFRM_RESOURCE_FMT, __FUNCTION__,
		   EFRM_RESOURCE_PRI_ARG(frs->rs.rs_handle));

	/* if we have a PT endpoint */
	if (NULL != frs->pt) {
		/* Detach the filter */
		EFRM_FOR_EACH_NIC_IN_SET(&frs->nic_set, nic_i, nic)
		    efhw_nic_ipfilter_detach(nic, frs->filter_idx);

		/* Release our ref to the PT resource. */
		EFRM_TRACE("%s: releasing PT resource reference",
			   __FUNCTION__);
		efrm_vi_resource_release(frs->pt);
	}

	/* Disable the filter. */
	EFRM_FOR_EACH_NIC_IN_SET(&frs->nic_set, nic_i, nic)
	    efhw_nic_ipfilter_clear(nic, frs->filter_idx);

	/* Free this filter. */
	id = EFRM_RESOURCE_INSTANCE(frs->rs.rs_handle);
	EFRM_VERIFY_EQ(kfifo_put(efrm_filter_manager->free_ids,
				 (unsigned char *)&id, sizeof(id)),
		       sizeof(id));

	EFRM_DO_DEBUG(memset(frs, 0, sizeof(*frs)));
	kfree(frs);
}
EXPORT_SYMBOL(efrm_filter_resource_free);

static void filter_rm_dtor(struct efrm_resource_manager *rm)
{
	EFRM_TRACE("filter_rm_dtor");

	EFRM_RESOURCE_MANAGER_ASSERT_VALID(&efrm_filter_manager->rm);
	EFRM_ASSERT(&efrm_filter_manager->rm == rm);

	kfifo_vfree(efrm_filter_manager->free_ids);
	EFRM_TRACE("filter_rm_dtor: done");
}

/**********************************************************************/
/**********************************************************************/
/**********************************************************************/

int efrm_create_filter_resource_manager(struct efrm_resource_manager **rm_out)
{
	int rc;

	EFRM_ASSERT(rm_out);

	efrm_filter_manager =
	    kmalloc(sizeof(struct filter_resource_manager), GFP_KERNEL);
	if (efrm_filter_manager == 0)
		return -ENOMEM;
	memset(efrm_filter_manager, 0, sizeof(*efrm_filter_manager));

	rc = efrm_resource_manager_ctor(&efrm_filter_manager->rm,
					filter_rm_dtor, "FILTER",
					EFRM_RESOURCE_FILTER, 0);
	if (rc < 0)
		goto fail1;

	/* Create a pool of free instances */
	rc = efrm_kfifo_id_ctor(&efrm_filter_manager->free_ids,
				0, EFHW_IP_FILTER_NUM,
				&efrm_filter_manager->rm.rm_lock);
	if (rc != 0)
		goto fail2;

	*rm_out = &efrm_filter_manager->rm;
	EFRM_TRACE("%s: filter resources created - %d IDs",
		   __FUNCTION__, kfifo_len(efrm_filter_manager->free_ids));
	return 0;

fail2:
	efrm_resource_manager_dtor(&efrm_filter_manager->rm);
fail1:
	memset(efrm_filter_manager, 0, sizeof(*efrm_filter_manager));
	kfree(efrm_filter_manager);
	return rc;

}

/*--------------------------------------------------------------------
 *!
 * Called to set/change the PT endpoint of a filter
 *
 * Example of use is TCP helper when it finds a wildcard IP filter
 * needs to change which application it delivers traffic to
 *
 * \param frs           filter resource
 * \param pt_handle     handle of new PT endpoint
 *
 * \return              standard error codes
 *
 *--------------------------------------------------------------------*/
int
efrm_filter_resource_set_ptresource(struct filter_resource *frs,
				    struct vi_resource *ptrs)
{
	int rc, pti, nic_i;
	struct efhw_nic *nic;

	EFRM_ASSERT(frs);

	/* if filter is attached to a valid PT endpoint */
	if (NULL != frs->pt) {

		EFRM_TRACE("%s: detaching PT resource " EFRM_RESOURCE_FMT
			   " from filter ",
			   __FUNCTION__,
			   EFRM_RESOURCE_PRI_ARG(frs->rs.rs_handle));
		/* Detach the filter */
		EFRM_FOR_EACH_NIC_IN_SET(&frs->nic_set, nic_i, nic)
		    efhw_nic_ipfilter_detach(nic, frs->filter_idx);

		/* release reference */
		efrm_vi_resource_release(frs->pt);
		frs->pt = NULL;
	}

	if (ptrs != NULL) {
		/* get PT endpoint index */
		EFRM_RESOURCE_ASSERT_VALID(&ptrs->rs, 0);
		EFRM_ASSERT(EFRM_RESOURCE_TYPE(ptrs->rs.rs_handle) ==
			    EFRM_RESOURCE_VI);
		pti = EFRM_RESOURCE_INSTANCE(ptrs->rs.rs_handle);
		if (pti == 0) {
			EFRM_ERR("%s: cannot filter for channel 0",
				 __FUNCTION__);
			rc = -EINVAL;
			goto fail2;
		}
		frs->pt = ptrs;
		EFRM_TRACE("%s: attaching PT resource " EFRM_RESOURCE_FMT
			   " to filter",
			   __FUNCTION__,
			   EFRM_RESOURCE_PRI_ARG(frs->pt->rs.rs_handle));
		EFRM_FOR_EACH_NIC_IN_SET(&frs->nic_set, nic_i, nic)
		    efhw_nic_ipfilter_attach(nic, frs->filter_idx, pti);
		efrm_vi_resource_ref(frs->pt);
	}
	return 0;

fail2:
	efrm_vi_resource_release(frs->pt);
	return rc;
}
EXPORT_SYMBOL(efrm_filter_resource_set_ptresource);

int efrm_filter_resource_clear(struct filter_resource *frs)
{
	struct efhw_nic *nic;
	int nic_i;

	EFRM_ASSERT(frs);
	EFRM_FOR_EACH_NIC_IN_SET(&frs->nic_set, nic_i, nic)
	    efhw_nic_ipfilter_clear(nic, frs->filter_idx);

	return 0;
}
EXPORT_SYMBOL(efrm_filter_resource_clear);

int
__efrm_filter_resource_set(struct filter_resource *frs, int type,
			   unsigned saddr, uint16_t sport,
			   unsigned daddr, uint16_t dport)
{
	struct efhw_nic *nic;
	int nic_i, rc = 0;
	unsigned instance = EFRM_RESOURCE_INSTANCE(frs->pt->rs.rs_handle);

	EFRM_ASSERT(frs);
	EFRM_ASSERT(frs->pt);

	if (efrm_nic_table.a_nic->devtype.variant >= 'B') {
		/* Scatter setting must match the setting for
		 * the corresponding RX queue */
		if (!(frs->pt->flags & EFHW_VI_JUMBO_EN))
			type |= EFHW_IP_FILTER_TYPE_NOSCAT_B0_MASK;
	}

	EFRM_FOR_EACH_NIC_IN_SET(&frs->nic_set, nic_i, nic)
	    if (rc >= 0)
		rc = efhw_nic_ipfilter_set(nic, type, &frs->filter_idx,
					   instance,
					   saddr, sport, daddr, dport);

	return rc;
}
EXPORT_SYMBOL(__efrm_filter_resource_set);;

int
efrm_filter_resource_alloc(struct vi_resource *vi_parent,
			   struct filter_resource **frs_out)
{
	struct efhw_nic *nic;
	int nic_i, rc, instance;
	struct filter_resource *frs;

	EFRM_ASSERT(frs_out);
	EFRM_ASSERT(efrm_filter_manager);
	EFRM_RESOURCE_MANAGER_ASSERT_VALID(&efrm_filter_manager->rm);
	EFRM_ASSERT(vi_parent == NULL ||
		    EFRM_RESOURCE_TYPE(vi_parent->rs.rs_handle) ==
		    EFRM_RESOURCE_VI);

	/* Allocate resource data structure. */
	frs = kmalloc(sizeof(struct filter_resource), GFP_KERNEL);
	if (!frs)
		return -ENOMEM;
	efrm_nic_set_clear(&frs->nic_set);

	/* Allocate an instance. */
	rc = kfifo_get(efrm_filter_manager->free_ids,
		       (unsigned char *)&instance, sizeof(instance));
	if (rc != sizeof(instance)) {
		EFRM_TRACE("%s: out of instances", __FUNCTION__);
		EFRM_ASSERT(rc == 0);
		rc = -EBUSY;
		goto fail1;
	}

	/* Initialise the resource DS. */
	efrm_resource_init(&frs->rs, EFRM_RESOURCE_FILTER, instance);
	frs->pt = vi_parent;
	if (frs->pt)
		efrm_vi_resource_ref(frs->pt);
	frs->filter_idx = -1;
	EFRM_FOR_EACH_NIC(nic_i, nic)
	    efrm_nic_set_write(&frs->nic_set, nic_i, true);

	EFRM_TRACE("%s: " EFRM_RESOURCE_FMT " Q %d idx %x",
		   __FUNCTION__,
		   EFRM_RESOURCE_PRI_ARG(frs->rs.rs_handle),
		   vi_parent == NULL ? -1 :
		   EFRM_RESOURCE_INSTANCE(vi_parent->rs.rs_handle),
		   frs->filter_idx);

	/* Put it into the resource manager's table. */
	rc = efrm_resource_manager_insert(&frs->rs);
	if (rc != 0) {
		if (atomic_dec_and_test(&frs->rs.rs_ref_count))
			efrm_filter_resource_free(frs);
		return rc;
	}

	*frs_out = frs;
	return 0;

fail1:
	memset(frs, 0, sizeof(*frs));
	kfree(frs);
	return rc;
}
EXPORT_SYMBOL(efrm_filter_resource_alloc);
