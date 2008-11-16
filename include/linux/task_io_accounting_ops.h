/*
 * Task I/O accounting operations
 */
#ifndef __TASK_IO_ACCOUNTING_OPS_INCLUDED
#define __TASK_IO_ACCOUNTING_OPS_INCLUDED

#include <ub/io_acct.h>

#ifdef CONFIG_TASK_IO_ACCOUNTING
static inline void task_io_account_read(size_t bytes)
{
	ub_io_account_read(bytes);
	current->ioac.read_bytes += bytes;
}

static inline void task_io_account_write(struct page *page, size_t bytes,
		int sync)
{
	if (sync)
		ub_io_account_write(bytes);
	else
		ub_io_account_dirty(page, bytes);

	current->ioac.write_bytes += bytes;
}

static inline void task_io_account_cancelled_write(size_t bytes)
{
	ub_io_account_write_cancelled(bytes);
	current->ioac.cancelled_write_bytes += bytes;
}

static inline void task_io_accounting_init(struct task_struct *tsk)
{
	memset(&tsk->ioac, 0, sizeof(tsk->ioac));
}

#else

static inline void task_io_account_read(size_t bytes)
{
}

static inline void task_io_account_write(struct page *page, size_t bytes,
		int sync)
{
}

static inline void task_io_account_cancelled_write(size_t bytes)
{
}

static inline void task_io_accounting_init(struct task_struct *tsk)
{
}

#endif		/* CONFIG_TASK_IO_ACCOUNTING */
#endif		/* __TASK_IO_ACCOUNTING_OPS_INCLUDED */
