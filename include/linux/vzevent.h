#ifndef __LINUX_VZ_EVENT_H__
#define __LINUX_VZ_EVENT_H__

#if defined(CONFIG_VZ_EVENT) || defined(CONFIG_VZ_EVENT_MODULE)
extern int vzevent_send(int msg, const char *attrs_fmt, ...);
#else
static inline int vzevent_send(int msg, const char *attrs_fmt, ...)
{
	return 0;
}
#endif

#endif /* __LINUX_VZ_EVENT_H__ */
