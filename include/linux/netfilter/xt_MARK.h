#ifndef _XT_MARK_H_target
#define _XT_MARK_H_target

#include <net/compat.h>

/* Version 0 */
struct xt_mark_target_info {
	unsigned long mark;
};

/* Version 1 */
enum {
	XT_MARK_SET=0,
	XT_MARK_AND,
	XT_MARK_OR,
};

struct xt_mark_target_info_v1 {
	unsigned long mark;
	u_int8_t mode;
};

#ifdef CONFIG_COMPAT
struct compat_xt_mark_target_info_v1 {
	compat_ulong_t mark;
	u_int8_t mode;
};
#endif /*CONFIG_COMPAT*/
#endif /*_XT_MARK_H_target */
