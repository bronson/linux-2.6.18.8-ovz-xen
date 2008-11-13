/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file contains non-contiguous I/O buffers support.
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
#include <ci/efhw/iopage.h>
#include <ci/driver/efab/hardware.h>
#include <ci/efrm/private.h>
#include <ci/efrm/iobufset.h>
#include <ci/efrm/vi_resource_manager.h>
#include <ci/efrm/buffer_table.h>

#define EFRM_IOBUFSET_MAX_NUM_INSTANCES 0x00010000

struct iobufset_resource_manager {
	struct efrm_resource_manager rm;
	struct kfifo *free_ids;
};

struct iobufset_resource_manager *efrm_iobufset_manager;

#define iobsrs(rs1)  iobufset_resource(rs1)

/* Returns size of iobufset resource data structure. */
static inline size_t iobsrs_size(int no_pages)
{
	return offsetof(struct iobufset_resource, bufs) +
	    no_pages * sizeof(efhw_iopage_t);
}

void efrm_iobufset_resource_free(struct iobufset_resource *rs)
{
	unsigned int no_pages;
	unsigned int i;
	int id;

	EFRM_RESOURCE_ASSERT_VALID(&rs->rs, 1);
	no_pages = rs->n_bufs;

	if (rs->buf_tbl_alloc.base != (unsigned)-1)
		efrm_buffer_table_free(&rs->buf_tbl_alloc);

	/* see comment on call to efhw_iopage_alloc in the alloc routine above
	   for discussion on use of efrm_nic_table.a_nic here */
	EFRM_ASSERT(efrm_nic_table.a_nic);
	if (rs->order == 0) {
		for (i = 0; i < rs->n_bufs; ++i)
			efhw_iopage_free(efrm_nic_table.a_nic, &rs->bufs[i]);
	} else {
		/* it is important that this is executed in increasing page
		 * order because some implementations of
		 * efhw_iopages_init_from_iopage() assume this */
		for (i = 0; i < rs->n_bufs;
		     i += rs->pages_per_contiguous_chunk) {
			efhw_iopages_t iopages;
			efhw_iopages_init_from_iopage(&iopages, &rs->bufs[i],
						    rs->order);
			efhw_iopages_free(efrm_nic_table.a_nic, &iopages);
		}
	}

	/* free the instance number */
	id = EFRM_RESOURCE_INSTANCE(rs->rs.rs_handle);
	EFRM_VERIFY_EQ(kfifo_put(efrm_iobufset_manager->free_ids,
				 (unsigned char *)&id, sizeof(id)), sizeof(id));

	efrm_vi_resource_release(rs->evq);

	EFRM_DO_DEBUG(memset(rs, 0, sizeof(*rs)));
	if (iobsrs_size(no_pages) < PAGE_SIZE) {
		kfree(rs);
	} else {
		vfree(rs);
	}
}
EXPORT_SYMBOL(efrm_iobufset_resource_free);

int
efrm_iobufset_resource_alloc(int32_t n_pages,
			     int32_t pages_per_contiguous_chunk,
			     struct vi_resource *vi_evq,
			     bool phys_addr_mode,
			     uint32_t faultonaccess,
			     struct iobufset_resource **iobrs_out)
{
	struct iobufset_resource *iobrs;
	int rc, instance, object_size;
	unsigned int i;

	EFRM_ASSERT(iobrs_out);
	EFRM_ASSERT(efrm_iobufset_manager);
	EFRM_RESOURCE_MANAGER_ASSERT_VALID(&efrm_iobufset_manager->rm);
	EFRM_RESOURCE_ASSERT_VALID(&vi_evq->rs, 0);
	EFRM_ASSERT(EFRM_RESOURCE_TYPE(vi_evq->rs.rs_handle) ==
		    EFRM_RESOURCE_VI);
	EFRM_ASSERT(efrm_nic_table.a_nic);

	/* allocate the resource data structure. */
	object_size = iobsrs_size(n_pages);
	if (object_size < PAGE_SIZE) {
		/* this should be OK from a tasklet */
		/* Necessary to do atomic alloc() as this
		   can be called from a weird-ass iSCSI context that is
		   !in_interrupt but is in_atomic - See BUG3163 */
		iobrs = kmalloc(object_size, GFP_ATOMIC);
	} else {		/* can't do this within a tasklet */
#ifndef NDEBUG
		if (in_interrupt() || in_atomic()) {
			EFRM_ERR("%s(): alloc->u.iobufset.in_n_pages=%d",
				 __FUNCTION__, n_pages);
			EFRM_ASSERT(!in_interrupt());
			EFRM_ASSERT(!in_atomic());
		}
#endif
		iobrs = (struct iobufset_resource *) vmalloc(object_size);
	}
	if (iobrs == 0) {
		rc = -ENOMEM;
		goto fail1;
	}

	/* Allocate an instance number. */
	rc = kfifo_get(efrm_iobufset_manager->free_ids,
		       (unsigned char *)&instance, sizeof(instance));
	if (rc != sizeof(instance)) {
		EFRM_TRACE("%s: out of instances", __FUNCTION__);
		EFRM_ASSERT(rc == 0);
		rc = -EBUSY;
		goto fail3;
	}

	efrm_resource_init(&iobrs->rs, EFRM_RESOURCE_IOBUFSET, instance);

	iobrs->evq = vi_evq;
	efrm_vi_resource_ref(iobrs->evq);

	iobrs->n_bufs = n_pages;
	iobrs->pages_per_contiguous_chunk = pages_per_contiguous_chunk;
	iobrs->order = fls(iobrs->pages_per_contiguous_chunk - 1);
	iobrs->faultonaccess = faultonaccess;

	EFRM_TRACE("%s: " EFRM_RESOURCE_FMT " %u pages", __FUNCTION__,
		   EFRM_RESOURCE_PRI_ARG(iobrs->rs.rs_handle), iobrs->n_bufs);

	/* Allocate the iobuffers. */
	if (iobrs->order == 0) {
		/* make sure iobufs are in a known state in case we don't
		 * finish our allocation */
		for (i = 0; i < iobrs->n_bufs; ++i)
			memset(&iobrs->bufs[i], 0, sizeof(iobrs->bufs[i]));

		for (i = 0; i < iobrs->n_bufs; ++i) {
			/* due to bug2426 we have to specifiy a NIC when
			 * allocating a DMAable page, which is a bit messy.
			 * For now we assume that if the page is suitable
			 * (e.g. DMAable) by one nic (efrm_nic_table.a_nic),
			 * it is suitable for all NICs.
			 * XXX I bet that breaks in Solaris.
			 */
			rc = efhw_iopage_alloc(efrm_nic_table.a_nic,
					     &iobrs->bufs[i]);
			if (rc < 0) {
				EFRM_ERR("%s: failed (rc %d) to allocate "
					 "page (i=%u)", __FUNCTION__, rc, i);
				goto fail4;
			}
		}
	} else {
		efhw_iopages_t iopages;
		unsigned j;

		/* make sure iobufs are in a known state in case we don't
		 * finish our allocation */
		for (i = 0; i < iobrs->n_bufs; ++i)
			memset(&iobrs->bufs[i], 0, sizeof(iobrs->bufs[i]));

		for (i = 0; i < iobrs->n_bufs;
		     i += iobrs->pages_per_contiguous_chunk) {
			rc = efhw_iopages_alloc(efrm_nic_table.a_nic,
						&iopages, iobrs->order);
			if (rc < 0) {
				EFRM_ERR("%s: failed (rc %d) to allocate "
					 "pages (i=%u order %d)",
					 __FUNCTION__, rc, i, iobrs->order);
				goto fail4;
			}
			for (j = 0; j < iobrs->pages_per_contiguous_chunk;
			     j++) {
				/* some implementation of
				 * efhw_iopage_init_from_iopages() rely on
				 * this function being called for
				 * _all_ pages in the chunk */
				efhw_iopage_init_from_iopages(
							&iobrs->bufs[i + j],
							&iopages, j);
			}
		}
	}

	iobrs->buf_tbl_alloc.base = (unsigned)-1;

	if (!phys_addr_mode) {
		unsigned instance = EFAB_VI_RESOURCE_INSTANCE(iobrs->evq);
		/* Allocate space in the NIC's buffer table. */
		rc = efrm_buffer_table_alloc(fls(iobrs->n_bufs - 1),
					     &iobrs->buf_tbl_alloc);
		if (rc < 0) {
			EFRM_ERR("%s: failed (%d) to alloc %d buffer table "
				 "entries", __FUNCTION__, rc, iobrs->n_bufs);
			goto fail5;
		}
		EFRM_ASSERT(((unsigned)1 << iobrs->buf_tbl_alloc.order) >=
			    (unsigned)iobrs->n_bufs);

		/* Initialise the buffer table entries. */
		for (i = 0; i < iobrs->n_bufs; ++i) {
			/*\ ?? \TODO burst them! */
			efrm_buffer_table_set(&iobrs->buf_tbl_alloc, i,
					      efhw_iopage_dma_addr(&iobrs->
								 bufs[i]),
					      instance);
		}
		efrm_buffer_table_commit();
	}

	EFRM_TRACE("%s: " EFRM_RESOURCE_FMT " %d pages @ "
		   EFHW_BUFFER_ADDR_FMT, __FUNCTION__,
		   EFRM_RESOURCE_PRI_ARG(iobrs->rs.rs_handle),
		   iobrs->n_bufs, EFHW_BUFFER_ADDR(iobrs->buf_tbl_alloc.base,
						   0));

	/* Put it into the resource manager's table. */
	rc = efrm_resource_manager_insert(&iobrs->rs);
	if (rc != 0) {
		if (atomic_dec_and_test(&iobrs->rs.rs_ref_count))
			efrm_iobufset_resource_free(iobrs);
		return rc;
	}

	*iobrs_out = iobrs;
	return 0;

fail5:
	i = iobrs->n_bufs;
fail4:
	/* see comment on call to efhw_iopage_alloc above for a discussion
	 * on use of efrm_nic_table.a_nic here */
	if (iobrs->order == 0) {
		while (i--) {
			efhw_iopage_t *page = &iobrs->bufs[i];
			efhw_iopage_free(efrm_nic_table.a_nic, page);
		}
	} else {
		unsigned int j;
		for (j = 0; j < i; j += iobrs->pages_per_contiguous_chunk) {
			efhw_iopages_t iopages;

			EFRM_ASSERT(j % iobrs->pages_per_contiguous_chunk
				    == 0);
			/* it is important that this is executed in increasing
			 * page order because some implementations of
			 * efhw_iopages_init_from_iopage() assume this */
			efhw_iopages_init_from_iopage(&iopages,
						      &iobrs->bufs[j],
						      iobrs->order);
			efhw_iopages_free(efrm_nic_table.a_nic, &iopages);
		}
	}
	efrm_vi_resource_release(iobrs->evq);
fail3:
	if (object_size < PAGE_SIZE) {
		kfree(iobrs);
	} else {
		vfree(iobrs);
	}
fail1:
	return rc;
}
EXPORT_SYMBOL(efrm_iobufset_resource_alloc);

static void iobufset_rm_dtor(struct efrm_resource_manager *rm)
{
	EFRM_ASSERT(&efrm_iobufset_manager->rm == rm);
	kfifo_vfree(efrm_iobufset_manager->free_ids);
}

int
efrm_create_iobufset_resource_manager(struct efrm_resource_manager **rm_out)
{
	int rc, max;

	EFRM_ASSERT(rm_out);

	efrm_iobufset_manager =
	    kmalloc(sizeof(*efrm_iobufset_manager), GFP_KERNEL);
	if (efrm_iobufset_manager == 0)
		return -ENOMEM;
	memset(efrm_iobufset_manager, 0, sizeof(*efrm_iobufset_manager));

	/*
	 * Bug 1145, 1370: We need to set initial size of both the resource
	 * table and instance id table so they never need to grow as we
	 * want to be allocate new iobufset at tasklet time. Lets make
	 * a pessimistic guess at maximum number of iobufsets possible.
	 * Could be less because
	 *   - jumbo frames have same no of packets per iobufset BUT more
	 *     pages per buffer
	 *   - buffer table entries used independently of iobufsets by
	 *     sendfile
	 *
	 * Based on TCP/IP stack setting of PKTS_PER_SET_S=5 ...
	 *  - can't use this define here as it breaks the layering.
	 */
#define MIN_PAGES_PER_IOBUFSET  (1 << 4)

	max = efrm_buffer_table_size() / MIN_PAGES_PER_IOBUFSET;
	max = min_t(int, max, EFRM_IOBUFSET_MAX_NUM_INSTANCES);

	rc = efrm_kfifo_id_ctor(&efrm_iobufset_manager->free_ids,
				0, max, &efrm_iobufset_manager->rm.rm_lock);
	if (rc != 0)
		goto fail1;

	rc = efrm_resource_manager_ctor(&efrm_iobufset_manager->rm,
					iobufset_rm_dtor, "IOBUFSET",
					EFRM_RESOURCE_IOBUFSET, max);
	if (rc < 0)
		goto fail2;

	*rm_out = &efrm_iobufset_manager->rm;
	return 0;

fail2:
	kfifo_vfree(efrm_iobufset_manager->free_ids);
fail1:
	EFRM_DO_DEBUG(memset(efrm_iobufset_manager, 0,
			     sizeof(*efrm_iobufset_manager)));
	kfree(efrm_iobufset_manager);
	return rc;
}
