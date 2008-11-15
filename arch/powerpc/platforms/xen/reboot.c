/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * Copyright (C) IBM Corp. 2006
 *
 * Authors: Jimi Xenidis <jimix@watson.ibm.com>
 */

#include <linux/module.h>
#include <xen/interface/xen.h>
#include <xen/interface/io/console.h>
#include <xen/xencons.h>
#include <asm/hypervisor.h>
#include <asm/machdep.h>

static void domain_machine_restart(char * __unused)
{
	/* We really want to get pending console data out before we die. */
	xencons_force_flush();
	HYPERVISOR_shutdown(SHUTDOWN_reboot);
}

static void domain_machine_power_off(void)
{
	/* We really want to get pending console data out before we die. */
	xencons_force_flush();
	HYPERVISOR_shutdown(SHUTDOWN_poweroff);
}

void xen_reboot_init(struct machdep_calls *md)
{
	if (md != NULL) {
		ppc_md.restart	 = md->restart;
		ppc_md.power_off = md->power_off;
		ppc_md.halt	 = md->halt;
	} else {
		ppc_md.restart	 = domain_machine_restart;
		ppc_md.power_off = domain_machine_power_off;
		ppc_md.halt	 = domain_machine_power_off;
	}
}
