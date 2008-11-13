#ifndef _XT_MARK_H
#define _XT_MARK_H

#include <net/compat.h>

struct xt_mark_info {
    unsigned long mark, mask;
    u_int8_t invert;
};

#ifdef CONFIG_COMPAT
struct compat_xt_mark_info {
    compat_ulong_t mark, mask;
    u_int8_t invert;
};
#endif /*CONFIG_COMPAT*/
#endif /*_XT_MARK_H*/
