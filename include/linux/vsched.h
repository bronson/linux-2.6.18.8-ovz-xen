/*
 *  include/linux/vsched.h
 *
 *  Copyright (C) 2005  SWsoft
 *  All rights reserved.
 *  
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

#ifndef __VSCHED_H__
#define __VSCHED_H__

#include <linux/config.h>
#include <linux/cache.h>
#include <linux/fairsched.h>
#include <linux/sched.h>

extern int vsched_create(int id, struct fairsched_node *node);
extern int vsched_destroy(struct vcpu_scheduler *vsched);
extern int vsched_taskcount(struct vcpu_scheduler *vsched);

extern int vsched_mvpr(struct task_struct *p, struct vcpu_scheduler *vsched);
extern int vsched_set_vcpus(struct vcpu_scheduler *vsched, unsigned int vcpus);

unsigned long ve_scale_khz(unsigned long khz);

#endif
