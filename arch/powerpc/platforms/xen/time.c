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
#include <linux/time.h>
#include <linux/rtc.h>
#include <asm/hypervisor.h>
#include <asm/machdep.h>
#include <asm/time.h>
#include <asm/udbg.h>

#ifdef DEBUG
#define DBG(fmt...) printk(fmt)
#else
#define DBG(fmt...)
#endif

void time_resume(void)
{
	snapshot_timebase();
}

static inline ulong time_from_shared(void)
{
	ulong t;

	DBG("tb_freq: %ld\n", ppc_tb_freq);

	t = mftb() - HYPERVISOR_shared_info->arch.boot_timebase;
	t /= ppc_tb_freq;
	t += HYPERVISOR_shared_info->wc_sec;

	return t;
}

static void (*host_md_get_rtc_time)(struct rtc_time *tm);
static void xen_get_rtc_time(struct rtc_time *tm)
{
	if (is_initial_xendomain()) {
		host_md_get_rtc_time(tm);
		return;
	} else {
		ulong t;

		t = time_from_shared();
		to_tm(t, tm);
	}
}

static int (*host_md_set_rtc_time)(struct rtc_time *tm);
static int xen_set_rtc_time(struct rtc_time *tm)
{
	ulong sec;

	if (is_initial_xendomain()) {
		host_md_set_rtc_time(tm);
		return 0;
	}

	sec = mktime(tm->tm_year, tm->tm_mon, tm->tm_mday,
		     tm->tm_hour, tm->tm_min, tm->tm_sec);

	HYPERVISOR_shared_info->wc_sec = sec;
	HYPERVISOR_shared_info->arch.boot_timebase = mftb();

	return 0;
}

static unsigned long (*host_md_get_boot_time)(void);
static unsigned long __init xen_get_boot_time(void)
{
	ulong t;

	if (is_initial_xendomain()) {
		t = host_md_get_boot_time();

		HYPERVISOR_shared_info->wc_sec = t;
		HYPERVISOR_shared_info->arch.boot_timebase = mftb();
		DBG("%s: time: %ld\n", __func__, t);
	} else {
		t = time_from_shared();
		DBG("%s: %ld\n", __func__, t);
	}
	return t;
}

void __init xen_setup_time(struct machdep_calls *host_md)
{
	ppc_md.get_boot_time = xen_get_boot_time;
	host_md_get_boot_time = host_md->get_boot_time;

	ppc_md.set_rtc_time = xen_set_rtc_time;
	host_md_set_rtc_time = host_md->set_rtc_time;

	ppc_md.get_rtc_time = xen_get_rtc_time;
	host_md_get_rtc_time = host_md->get_rtc_time;
}
