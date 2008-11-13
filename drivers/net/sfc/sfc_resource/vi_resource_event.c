/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file contains event handling for VI resource.
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
#include <ci/efhw/eventq.h>
#include <ci/efrm/private.h>
#include <ci/efrm/vi_resource_private.h>

void
efrm_eventq_request_wakeup(struct vi_resource *virs, unsigned current_ptr,
			   unsigned nic_index)
{
	struct efhw_nic *nic;
	int next_i;
	EFRM_ASSERT(efrm_nic_set_read(&virs->nic_set, nic_index));
	nic = efrm_nic_table.nic[nic_index];
	EFRM_ASSERT(nic);
	next_i = ((current_ptr / sizeof(efhw_event_t)) &
		  (virs->evq_capacity - 1));

	efhw_nic_wakeup_request(nic, efrm_eventq_dma_addr(virs, nic_index),
				next_i,
				EFRM_RESOURCE_INSTANCE(virs->rs.rs_handle));
}
EXPORT_SYMBOL(efrm_eventq_request_wakeup);

void efrm_eventq_reset(struct vi_resource *virs, int nic_index)
{
	struct efhw_nic *nic = efrm_nic_table.nic[nic_index];
	int instance = EFRM_RESOURCE_INSTANCE(virs->rs.rs_handle);

	EFRM_ASSERT(virs->evq_capacity != 0);
	EFRM_ASSERT(efrm_nic_set_read(&virs->nic_set, nic_index));

	/* FIXME: Protect against concurrent resets. */

	efhw_nic_event_queue_disable(nic, instance, 0);

	memset(efrm_eventq_base(virs, nic_index), EFHW_CLEAR_EVENT_VALUE,
	       efrm_eventq_bytes(virs, nic_index));
	efhw_nic_event_queue_enable(nic, instance, virs->evq_capacity,
				    efrm_eventq_dma_addr(virs, nic_index),
				    virs->nic_info[nic_index].evq_pages.
				    buf_tbl_alloc.base);
	EFRM_TRACE("%s: " EFRM_RESOURCE_FMT, __FUNCTION__,
		   EFRM_RESOURCE_PRI_ARG(virs->rs.rs_handle));
}
EXPORT_SYMBOL(efrm_eventq_reset);

int
efrm_eventq_register_callback(struct vi_resource *virs,
			      void (*handler) (void *, int,
					       struct efhw_nic *nic),
			      void *arg)
{
	int instance;
	int bit;

	EFRM_RESOURCE_ASSERT_VALID(&virs->rs, 0);
	EFRM_ASSERT(virs->evq_capacity != 0);

	instance = EFRM_RESOURCE_INSTANCE(virs->rs.rs_handle);

	/* The handler can be set only once. */
	bit = test_and_set_bit(VI_RESOURCE_EVQ_STATE_CALLBACK_REGISTERED,
			       &efrm_vi_manager->evq_infos[instance].evq_state);
	if (bit)
		return -EBUSY;

	/* Store the details. The order is important here. */
	virs->evq_callback_arg = arg;
	virs->evq_callback_fn = handler;

	return 0;
}
EXPORT_SYMBOL(efrm_eventq_register_callback);

void efrm_eventq_kill_callback(struct vi_resource *virs)
{
	int nic_i, instance;
	struct efhw_nic *nic;
	struct vi_resource_evq_info *evq_info;
	int32_t evq_state;
	int bit;

	EFRM_RESOURCE_ASSERT_VALID(&virs->rs, 0);
	EFRM_ASSERT(virs->evq_capacity != 0);

	/* Clean out the callback so a new one can be installed. */
	virs->evq_callback_fn = NULL;

	instance = EFRM_RESOURCE_INSTANCE(virs->rs.rs_handle);
	evq_info = &efrm_vi_manager->evq_infos[instance];

	/* Disable the event queue. */
	EFRM_FOR_EACH_NIC_IN_SET(&virs->nic_set, nic_i, nic)
	    efhw_nic_event_queue_disable(nic, instance, /*timer_only */ 1);

	/* Disable the callback. */
	bit = test_and_clear_bit(VI_RESOURCE_EVQ_STATE_CALLBACK_REGISTERED,
				 &evq_info->evq_state);
	EFRM_ASSERT(bit);	/* do not call me twice! */

	/* Spin until the callback is complete. */
	do {
		rmb();

		udelay(1);
		evq_state = evq_info->evq_state;
	} while ((evq_state & VI_RESOURCE_EVQ_STATE(BUSY)));
}
EXPORT_SYMBOL(efrm_eventq_kill_callback);

static void
efrm_eventq_do_callback(struct efhw_nic *nic, unsigned instance,
			bool is_timeout)
{
	void (*handler) (void *, int is_timeout, struct efhw_nic *nic);
	void *arg;
	struct vi_resource_evq_info *evq_info;
	int32_t evq_state;
	int32_t new_evq_state;
	struct vi_resource *virs;
	int bit;

	EFRM_TRACE("%s: q=%d %s", __FUNCTION__, instance,
		   is_timeout ? "timeout" : "wakeup");
	EFRM_ASSERT(efrm_vi_manager);

	evq_info = &efrm_vi_manager->evq_infos[instance];

	/* Set the BUSY bit and clear WAKEUP_PENDING.  Do this
	 * before waking up the sleeper to avoid races. */
	while (1) {
		evq_state = evq_info->evq_state;
		new_evq_state = evq_state;

		if ((evq_state & VI_RESOURCE_EVQ_STATE(BUSY)) != 0) {
			EFRM_ERR("%s:%d: evq_state[%d] corrupted!",
				 __FUNCTION__, __LINE__, instance);
			return;
		}

		if (!is_timeout)
			new_evq_state &= ~VI_RESOURCE_EVQ_STATE(WAKEUP_PENDING);

		if (evq_state & VI_RESOURCE_EVQ_STATE(CALLBACK_REGISTERED)) {
			new_evq_state |= VI_RESOURCE_EVQ_STATE(BUSY);
			if (cmpxchg(&evq_info->evq_state, evq_state,
				    new_evq_state) == evq_state) {
				virs = evq_info->evq_virs;
				break;
			}

		} else {
			/* Just update the state if necessary. */
			if (new_evq_state == evq_state ||
			    cmpxchg(&evq_info->evq_state, evq_state,
				    new_evq_state) == evq_state)
				return;
		}

		udelay(1);
	}

	/* Call the callback if any. */
	if (evq_state & VI_RESOURCE_EVQ_STATE(CALLBACK_REGISTERED)) {
		/* Retrieve the callback fn. */
		handler = virs->evq_callback_fn;
		arg = virs->evq_callback_arg;
		if (handler != NULL)	/* avoid races */
			handler(arg, is_timeout, nic);
	}

	/* Clear the BUSY bit. */
	bit =
	    test_and_clear_bit(VI_RESOURCE_EVQ_STATE_BUSY,
			       &evq_info->evq_state);
	if (!bit) {
		EFRM_ERR("%s:%d: evq_state corrupted!",
			 __FUNCTION__, __LINE__);
	}
}

void efrm_handle_wakeup_event(struct efhw_nic *nic, efhw_event_t *ev)
{
	efrm_eventq_do_callback(nic,
				(unsigned int)FALCON_EVENT_WAKE_EVQ_ID(ev),
				false);
}

void efrm_handle_timeout_event(struct efhw_nic *nic, efhw_event_t *ev)
{
	efrm_eventq_do_callback(nic,
				(unsigned int)FALCON_EVENT_WAKE_EVQ_ID(ev),
				true);
}
