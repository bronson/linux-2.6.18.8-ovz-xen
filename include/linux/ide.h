#ifndef _IDE_H
#define _IDE_H
/*
 *  linux/include/linux/ide.h
 *
 *  Copyright (C) 1994-2002  Linus Torvalds & authors
 */

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/ata.h>
#include <linux/blkdev.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/bio.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/completion.h>
#include <linux/pm.h>
#ifdef CONFIG_BLK_DEV_IDEACPI
#include <acpi/acpi.h>
#endif
#include <asm/byteorder.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/mutex.h>

#if defined(CONFIG_CRIS) || defined(CONFIG_FRV)
# define SUPPORT_VLB_SYNC 0
#else
# define SUPPORT_VLB_SYNC 1
#endif

/*
 * Used to indicate "no IRQ", should be a value that cannot be an IRQ
 * number.
 */
 
#define IDE_NO_IRQ		(-1)

typedef unsigned char	byte;	/* used everywhere */

/*
 * Probably not wise to fiddle with these
 */
#define ERROR_MAX	8	/* Max read/write errors per sector */
#define ERROR_RESET	3	/* Reset controller every 4th retry */
#define ERROR_RECAL	1	/* Recalibrate every 2nd retry */

#define HWIF(drive)		((ide_hwif_t *)((drive)->hwif))
#define HWGROUP(drive)		((ide_hwgroup_t *)(HWIF(drive)->hwgroup))

/*
 * Definitions for accessing IDE controller registers
 */
#define IDE_NR_PORTS		(10)

struct ide_io_ports {
	unsigned long	data_addr;

	union {
		unsigned long error_addr;	/*   read:  error */
		unsigned long feature_addr;	/*  write: feature */
	};

	unsigned long	nsect_addr;
	unsigned long	lbal_addr;
	unsigned long	lbam_addr;
	unsigned long	lbah_addr;

	unsigned long	device_addr;

	union {
		unsigned long status_addr;	/*  read: status  */
		unsigned long command_addr;	/* write: command */
	};

	unsigned long	ctl_addr;

	unsigned long	irq_addr;
};

#define OK_STAT(stat,good,bad)	(((stat)&((good)|(bad)))==(good))

#define BAD_R_STAT	(ATA_BUSY | ATA_ERR)
#define BAD_W_STAT	(BAD_R_STAT | ATA_DF)
#define BAD_STAT	(BAD_R_STAT | ATA_DRQ)
#define DRIVE_READY	(ATA_DRDY | ATA_DSC)

#define BAD_CRC		(ATA_ABORTED | ATA_ICRC)

#define SATA_NR_PORTS		(3)	/* 16 possible ?? */

#define SATA_STATUS_OFFSET	(0)
#define SATA_ERROR_OFFSET	(1)
#define SATA_CONTROL_OFFSET	(2)

/*
 * Our Physical Region Descriptor (PRD) table should be large enough
 * to handle the biggest I/O request we are likely to see.  Since requests
 * can have no more than 256 sectors, and since the typical blocksize is
 * two or more sectors, we could get by with a limit of 128 entries here for
 * the usual worst case.  Most requests seem to include some contiguous blocks,
 * further reducing the number of table entries required.
 *
 * The driver reverts to PIO mode for individual requests that exceed
 * this limit (possible with 512 byte blocksizes, eg. MSDOS f/s), so handling
 * 100% of all crazy scenarios here is not necessary.
 *
 * As it turns out though, we must allocate a full 4KB page for this,
 * so the two PRD tables (ide0 & ide1) will each get half of that,
 * allowing each to have about 256 entries (8 bytes each) from this.
 */
#define PRD_BYTES       8
#define PRD_ENTRIES	256

/*
 * Some more useful definitions
 */
#define PARTN_BITS	6	/* number of minor dev bits for partitions */
#define MAX_DRIVES	2	/* per interface; 2 assumed by lots of code */
#define SECTOR_SIZE	512

#define IDE_LARGE_SEEK(b1,b2,t)	(((b1) > (b2) + (t)) || ((b2) > (b1) + (t)))

/*
 * Timeouts for various operations:
 */
enum {
	/* spec allows up to 20ms */
	WAIT_DRQ	= HZ / 10,	/* 100ms */
	/* some laptops are very slow */
	WAIT_READY	= 5 * HZ,	/* 5s */
	/* should be less than 3ms (?), if all ATAPI CD is closed at boot */
	WAIT_PIDENTIFY	= 10 * HZ,	/* 10s */
	/* worst case when spinning up */
	WAIT_WORSTCASE	= 30 * HZ,	/* 30s */
	/* maximum wait for an IRQ to happen */
	WAIT_CMD	= 10 * HZ,	/* 10s */
	/* Some drives require a longer IRQ timeout. */
	WAIT_FLOPPY_CMD	= 50 * HZ,	/* 50s */
	/*
	 * Some drives (for example, Seagate STT3401A Travan) require a very
	 * long timeout, because they don't return an interrupt or clear their
	 * BSY bit until after the command completes (even retension commands).
	 */
	WAIT_TAPE_CMD	= 900 * HZ,	/* 900s */
	/* minimum sleep time */
	WAIT_MIN_SLEEP	= HZ / 50,	/* 20ms */
};

/*
 * Op codes for special requests to be handled by ide_special_rq().
 * Values should be in the range of 0x20 to 0x3f.
 */
#define REQ_DRIVE_RESET		0x20
#define REQ_DEVSET_EXEC		0x21
#define REQ_PARK_HEADS		0x22
#define REQ_UNPARK_HEADS	0x23

/*
 * Check for an interrupt and acknowledge the interrupt status
 */
struct hwif_s;
typedef int (ide_ack_intr_t)(struct hwif_s *);

/*
 * hwif_chipset_t is used to keep track of the specific hardware
 * chipset used by each IDE interface, if known.
 */
enum {		ide_unknown,	ide_generic,	ide_pci,
		ide_cmd640,	ide_dtc2278,	ide_ali14xx,
		ide_qd65xx,	ide_umc8672,	ide_ht6560b,
		ide_rz1000,	ide_trm290,
		ide_cmd646,	ide_cy82c693,	ide_4drives,
		ide_pmac,	ide_acorn,
		ide_au1xxx,	ide_palm3710
};

typedef u8 hwif_chipset_t;

/*
 * Structure to hold all information about the location of this port
 */
typedef struct hw_regs_s {
	union {
		struct ide_io_ports	io_ports;
		unsigned long		io_ports_array[IDE_NR_PORTS];
	};

	int		irq;			/* our irq number */
	ide_ack_intr_t	*ack_intr;		/* acknowledge interrupt */
	hwif_chipset_t  chipset;
	struct device	*dev, *parent;
	unsigned long	config;
} hw_regs_t;

void ide_init_port_data(struct hwif_s *, unsigned int);
void ide_init_port_hw(struct hwif_s *, hw_regs_t *);

static inline void ide_std_init_ports(hw_regs_t *hw,
				      unsigned long io_addr,
				      unsigned long ctl_addr)
{
	unsigned int i;

	for (i = 0; i <= 7; i++)
		hw->io_ports_array[i] = io_addr++;

	hw->io_ports.ctl_addr = ctl_addr;
}

/* for IDE PCI controllers in legacy mode, temporary */
static inline int __ide_default_irq(unsigned long base)
{
	switch (base) {
#ifdef CONFIG_IA64
	case 0x1f0: return isa_irq_to_vector(14);
	case 0x170: return isa_irq_to_vector(15);
#else
	case 0x1f0: return 14;
	case 0x170: return 15;
#endif
	}
	return 0;
}

#if defined(CONFIG_ARM) || defined(CONFIG_FRV) || defined(CONFIG_M68K) || \
    defined(CONFIG_MIPS) || defined(CONFIG_MN10300) || defined(CONFIG_PARISC) \
    || defined(CONFIG_PPC) || defined(CONFIG_SPARC) || defined(CONFIG_SPARC64)
#include <asm/ide.h>
#else
#include <asm-generic/ide_iops.h>
#endif

#define MAX_HWIFS	10

/* Currently only m68k, apus and m8xx need it */
#ifndef IDE_ARCH_ACK_INTR
# define ide_ack_intr(hwif) (1)
#endif

/* Currently only Atari needs it */
#ifndef IDE_ARCH_LOCK
# define ide_release_lock()			do {} while (0)
# define ide_get_lock(hdlr, data)		do {} while (0)
#endif /* IDE_ARCH_LOCK */

/*
 * Now for the data we need to maintain per-drive:  ide_drive_t
 */

#define ide_scsi	0x21
#define ide_disk	0x20
#define ide_optical	0x7
#define ide_cdrom	0x5
#define ide_tape	0x1
#define ide_floppy	0x0

/*
 * Special Driver Flags
 *
 * set_geometry	: respecify drive geometry
 * recalibrate	: seek to cyl 0
 * set_multmode	: set multmode count
 * reserved	: unused
 */
typedef union {
	unsigned all			: 8;
	struct {
		unsigned set_geometry	: 1;
		unsigned recalibrate	: 1;
		unsigned set_multmode	: 1;
		unsigned reserved	: 5;
	} b;
} special_t;

/*
 * Status returned from various ide_ functions
 */
typedef enum {
	ide_stopped,	/* no drive operation was started */
	ide_started,	/* a drive operation was started, handler was set */
} ide_startstop_t;

enum {
	IDE_TFLAG_LBA48			= (1 << 0),
	IDE_TFLAG_FLAGGED		= (1 << 2),
	IDE_TFLAG_OUT_DATA		= (1 << 3),
	IDE_TFLAG_OUT_HOB_FEATURE	= (1 << 4),
	IDE_TFLAG_OUT_HOB_NSECT		= (1 << 5),
	IDE_TFLAG_OUT_HOB_LBAL		= (1 << 6),
	IDE_TFLAG_OUT_HOB_LBAM		= (1 << 7),
	IDE_TFLAG_OUT_HOB_LBAH		= (1 << 8),
	IDE_TFLAG_OUT_HOB		= IDE_TFLAG_OUT_HOB_FEATURE |
					  IDE_TFLAG_OUT_HOB_NSECT |
					  IDE_TFLAG_OUT_HOB_LBAL |
					  IDE_TFLAG_OUT_HOB_LBAM |
					  IDE_TFLAG_OUT_HOB_LBAH,
	IDE_TFLAG_OUT_FEATURE		= (1 << 9),
	IDE_TFLAG_OUT_NSECT		= (1 << 10),
	IDE_TFLAG_OUT_LBAL		= (1 << 11),
	IDE_TFLAG_OUT_LBAM		= (1 << 12),
	IDE_TFLAG_OUT_LBAH		= (1 << 13),
	IDE_TFLAG_OUT_TF		= IDE_TFLAG_OUT_FEATURE |
					  IDE_TFLAG_OUT_NSECT |
					  IDE_TFLAG_OUT_LBAL |
					  IDE_TFLAG_OUT_LBAM |
					  IDE_TFLAG_OUT_LBAH,
	IDE_TFLAG_OUT_DEVICE		= (1 << 14),
	IDE_TFLAG_WRITE			= (1 << 15),
	IDE_TFLAG_FLAGGED_SET_IN_FLAGS	= (1 << 16),
	IDE_TFLAG_IN_DATA		= (1 << 17),
	IDE_TFLAG_CUSTOM_HANDLER	= (1 << 18),
	IDE_TFLAG_DMA_PIO_FALLBACK	= (1 << 19),
	IDE_TFLAG_IN_HOB_FEATURE	= (1 << 20),
	IDE_TFLAG_IN_HOB_NSECT		= (1 << 21),
	IDE_TFLAG_IN_HOB_LBAL		= (1 << 22),
	IDE_TFLAG_IN_HOB_LBAM		= (1 << 23),
	IDE_TFLAG_IN_HOB_LBAH		= (1 << 24),
	IDE_TFLAG_IN_HOB_LBA		= IDE_TFLAG_IN_HOB_LBAL |
					  IDE_TFLAG_IN_HOB_LBAM |
					  IDE_TFLAG_IN_HOB_LBAH,
	IDE_TFLAG_IN_HOB		= IDE_TFLAG_IN_HOB_FEATURE |
					  IDE_TFLAG_IN_HOB_NSECT |
					  IDE_TFLAG_IN_HOB_LBA,
	IDE_TFLAG_IN_FEATURE		= (1 << 1),
	IDE_TFLAG_IN_NSECT		= (1 << 25),
	IDE_TFLAG_IN_LBAL		= (1 << 26),
	IDE_TFLAG_IN_LBAM		= (1 << 27),
	IDE_TFLAG_IN_LBAH		= (1 << 28),
	IDE_TFLAG_IN_LBA		= IDE_TFLAG_IN_LBAL |
					  IDE_TFLAG_IN_LBAM |
					  IDE_TFLAG_IN_LBAH,
	IDE_TFLAG_IN_TF			= IDE_TFLAG_IN_NSECT |
					  IDE_TFLAG_IN_LBA,
	IDE_TFLAG_IN_DEVICE		= (1 << 29),
	IDE_TFLAG_HOB			= IDE_TFLAG_OUT_HOB |
					  IDE_TFLAG_IN_HOB,
	IDE_TFLAG_TF			= IDE_TFLAG_OUT_TF |
					  IDE_TFLAG_IN_TF,
	IDE_TFLAG_DEVICE		= IDE_TFLAG_OUT_DEVICE |
					  IDE_TFLAG_IN_DEVICE,
	/* force 16-bit I/O operations */
	IDE_TFLAG_IO_16BIT		= (1 << 30),
	/* ide_task_t was allocated using kmalloc() */
	IDE_TFLAG_DYN			= (1 << 31),
};

struct ide_taskfile {
	u8	hob_data;	/*  0: high data byte (for TASKFILE IOCTL) */

	u8	hob_feature;	/*  1-5: additional data to support LBA48 */
	u8	hob_nsect;
	u8	hob_lbal;
	u8	hob_lbam;
	u8	hob_lbah;

	u8	data;		/*  6: low data byte (for TASKFILE IOCTL) */

	union {			/*  7: */
		u8 error;	/*   read:  error */
		u8 feature;	/*  write: feature */
	};

	u8	nsect;		/*  8: number of sectors */
	u8	lbal;		/*  9: LBA low */
	u8	lbam;		/* 10: LBA mid */
	u8	lbah;		/* 11: LBA high */

	u8	device;		/* 12: device select */

	union {			/* 13: */
		u8 status;	/*  read: status  */
		u8 command;	/* write: command */
	};
};

typedef struct ide_task_s {
	union {
		struct ide_taskfile	tf;
		u8			tf_array[14];
	};
	u32			tf_flags;
	int			data_phase;
	struct request		*rq;		/* copy of request */
	void			*special;	/* valid_t generally */
} ide_task_t;

/* ATAPI packet command flags */
enum {
	/* set when an error is considered normal - no retry (ide-tape) */
	PC_FLAG_ABORT			= (1 << 0),
	PC_FLAG_SUPPRESS_ERROR		= (1 << 1),
	PC_FLAG_WAIT_FOR_DSC		= (1 << 2),
	PC_FLAG_DMA_OK			= (1 << 3),
	PC_FLAG_DMA_IN_PROGRESS		= (1 << 4),
	PC_FLAG_DMA_ERROR		= (1 << 5),
	PC_FLAG_WRITING			= (1 << 6),
	/* command timed out */
	PC_FLAG_TIMEDOUT		= (1 << 7),
};

/*
 * With each packet command, we allocate a buffer of IDE_PC_BUFFER_SIZE bytes.
 * This is used for several packet commands (not for READ/WRITE commands).
 */
#define IDE_PC_BUFFER_SIZE	256

struct ide_atapi_pc {
	/* actual packet bytes */
	u8 c[12];
	/* incremented on each retry */
	int retries;
	int error;

	/* bytes to transfer */
	int req_xfer;
	/* bytes actually transferred */
	int xferred;

	/* data buffer */
	u8 *buf;
	/* current buffer position */
	u8 *cur_pos;
	int buf_size;
	/* missing/available data on the current buffer */
	int b_count;

	/* the corresponding request */
	struct request *rq;

	unsigned long flags;

	/*
	 * those are more or less driver-specific and some of them are subject
	 * to change/removal later.
	 */
	u8 pc_buf[IDE_PC_BUFFER_SIZE];

	/* idetape only */
	struct idetape_bh *bh;
	char *b_data;

	/* idescsi only for now */
	struct scatterlist *sg;
	unsigned int sg_cnt;

	struct scsi_cmnd *scsi_cmd;
	void (*done) (struct scsi_cmnd *);

	unsigned long timeout;
};

struct ide_devset;
struct ide_driver_s;

#ifdef CONFIG_BLK_DEV_IDEACPI
struct ide_acpi_drive_link;
struct ide_acpi_hwif_link;
#endif

struct ide_drive_s;

struct ide_disk_ops {
	int		(*check)(struct ide_drive_s *, const char *);
	int		(*get_capacity)(struct ide_drive_s *);
	void		(*setup)(struct ide_drive_s *);
	void		(*flush)(struct ide_drive_s *);
	int		(*init_media)(struct ide_drive_s *, struct gendisk *);
	int		(*set_doorlock)(struct ide_drive_s *, struct gendisk *,
					int);
	ide_startstop_t	(*do_request)(struct ide_drive_s *, struct request *,
				      sector_t);
	int		(*end_request)(struct ide_drive_s *, int, int);
	int		(*ioctl)(struct ide_drive_s *, struct block_device *,
				 fmode_t, unsigned int, unsigned long);
};

/* ATAPI device flags */
enum {
	IDE_AFLAG_DRQ_INTERRUPT		= (1 << 0),

	/* ide-cd */
	/* Drive cannot eject the disc. */
	IDE_AFLAG_NO_EJECT		= (1 << 3),
	/* Drive is a pre ATAPI 1.2 drive. */
	IDE_AFLAG_PRE_ATAPI12		= (1 << 4),
	/* TOC addresses are in BCD. */
	IDE_AFLAG_TOCADDR_AS_BCD	= (1 << 5),
	/* TOC track numbers are in BCD. */
	IDE_AFLAG_TOCTRACKS_AS_BCD	= (1 << 6),
	/*
	 * Drive does not provide data in multiples of SECTOR_SIZE
	 * when more than one interrupt is needed.
	 */
	IDE_AFLAG_LIMIT_NFRAMES		= (1 << 7),
	/* Seeking in progress. */
	IDE_AFLAG_SEEKING		= (1 << 8),
	/* Saved TOC information is current. */
	IDE_AFLAG_TOC_VALID		= (1 << 9),
	/* We think that the drive door is locked. */
	IDE_AFLAG_DOOR_LOCKED		= (1 << 10),
	/* SET_CD_SPEED command is unsupported. */
	IDE_AFLAG_NO_SPEED_SELECT	= (1 << 11),
	IDE_AFLAG_VERTOS_300_SSD	= (1 << 12),
	IDE_AFLAG_VERTOS_600_ESD	= (1 << 13),
	IDE_AFLAG_SANYO_3CD		= (1 << 14),
	IDE_AFLAG_FULL_CAPS_PAGE	= (1 << 15),
	IDE_AFLAG_PLAY_AUDIO_OK		= (1 << 16),
	IDE_AFLAG_LE_SPEED_FIELDS	= (1 << 17),

	/* ide-floppy */
	/* Avoid commands not supported in Clik drive */
	IDE_AFLAG_CLIK_DRIVE		= (1 << 19),
	/* Requires BH algorithm for packets */
	IDE_AFLAG_ZIP_DRIVE		= (1 << 20),
	/* Supports format progress report */
	IDE_AFLAG_SRFP			= (1 << 22),

	/* ide-tape */
	IDE_AFLAG_IGNORE_DSC		= (1 << 23),
	/* 0 When the tape position is unknown */
	IDE_AFLAG_ADDRESS_VALID		= (1 <<	24),
	/* Device already opened */
	IDE_AFLAG_BUSY			= (1 << 25),
	/* Attempt to auto-detect the current user block size */
	IDE_AFLAG_DETECT_BS		= (1 << 26),
	/* Currently on a filemark */
	IDE_AFLAG_FILEMARK		= (1 << 27),
	/* 0 = no tape is loaded, so we don't rewind after ejecting */
	IDE_AFLAG_MEDIUM_PRESENT	= (1 << 28),

	IDE_AFLAG_NO_AUTOCLOSE		= (1 << 29),
};

/* device flags */
enum {
	/* restore settings after device reset */
	IDE_DFLAG_KEEP_SETTINGS		= (1 << 0),
	/* device is using DMA for read/write */
	IDE_DFLAG_USING_DMA		= (1 << 1),
	/* okay to unmask other IRQs */
	IDE_DFLAG_UNMASK		= (1 << 2),
	/* don't attempt flushes */
	IDE_DFLAG_NOFLUSH		= (1 << 3),
	/* DSC overlap */
	IDE_DFLAG_DSC_OVERLAP		= (1 << 4),
	/* give potential excess bandwidth */
	IDE_DFLAG_NICE1			= (1 << 5),
	/* device is physically present */
	IDE_DFLAG_PRESENT		= (1 << 6),
	/* device ejected hint */
	IDE_DFLAG_DEAD			= (1 << 7),
	/* id read from device (synthetic if not set) */
	IDE_DFLAG_ID_READ		= (1 << 8),
	IDE_DFLAG_NOPROBE		= (1 << 9),
	/* need to do check_media_change() */
	IDE_DFLAG_REMOVABLE		= (1 << 10),
	/* needed for removable devices */
	IDE_DFLAG_ATTACH		= (1 << 11),
	IDE_DFLAG_FORCED_GEOM		= (1 << 12),
	/* disallow setting unmask bit */
	IDE_DFLAG_NO_UNMASK		= (1 << 13),
	/* disallow enabling 32-bit I/O */
	IDE_DFLAG_NO_IO_32BIT		= (1 << 14),
	/* for removable only: door lock/unlock works */
	IDE_DFLAG_DOORLOCKING		= (1 << 15),
	/* disallow DMA */
	IDE_DFLAG_NODMA			= (1 << 16),
	/* powermanagment told us not to do anything, so sleep nicely */
	IDE_DFLAG_BLOCKED		= (1 << 17),
	/* ide-scsi emulation */
	IDE_DFLAG_SCSI			= (1 << 18),
	/* sleeping & sleep field valid */
	IDE_DFLAG_SLEEPING		= (1 << 19),
	IDE_DFLAG_POST_RESET		= (1 << 20),
	IDE_DFLAG_UDMA33_WARNED		= (1 << 21),
	IDE_DFLAG_LBA48			= (1 << 22),
	/* status of write cache */
	IDE_DFLAG_WCACHE		= (1 << 23),
	/* used for ignoring ATA_DF */
	IDE_DFLAG_NOWERR		= (1 << 24),
	/* retrying in PIO */
	IDE_DFLAG_DMA_PIO_RETRY		= (1 << 25),
	IDE_DFLAG_LBA			= (1 << 26),
	/* don't unload heads */
	IDE_DFLAG_NO_UNLOAD		= (1 << 27),
	/* heads unloaded, please don't reset port */
	IDE_DFLAG_PARKED		= (1 << 28),
	IDE_DFLAG_MEDIA_CHANGED		= (1 << 29),
	/* write protect */
	IDE_DFLAG_WP			= (1 << 30),
	IDE_DFLAG_FORMAT_IN_PROGRESS	= (1 << 31),
};

struct ide_drive_s {
	char		name[4];	/* drive name, such as "hda" */
        char            driver_req[10];	/* requests specific driver */

	struct request_queue	*queue;	/* request queue */

	struct request		*rq;	/* current request */
	struct ide_drive_s 	*next;	/* circular list of hwgroup drives */
	void		*driver_data;	/* extra driver data */
	u16			*id;	/* identification info */
#ifdef CONFIG_IDE_PROC_FS
	struct proc_dir_entry *proc;	/* /proc/ide/ directory entry */
	const struct ide_proc_devset *settings; /* /proc/ide/ drive settings */
#endif
	struct hwif_s		*hwif;	/* actually (ide_hwif_t *) */

	const struct ide_disk_ops *disk_ops;

	unsigned long dev_flags;

	unsigned long sleep;		/* sleep until this time */
	unsigned long service_start;	/* time we started last request */
	unsigned long service_time;	/* service time of last request */
	unsigned long timeout;		/* max time to wait for irq */

	special_t	special;	/* special action flags */

	u8	select;			/* basic drive/head select reg value */
	u8	retry_pio;		/* retrying dma capable host in pio */
	u8	waiting_for_dma;	/* dma currently in progress */
	u8	dma;			/* atapi dma flag */

        u8	quirk_list;	/* considered quirky, set for a specific host */
        u8	init_speed;	/* transfer rate set at boot */
        u8	current_speed;	/* current transfer rate set */
	u8	desired_speed;	/* desired transfer rate set */
        u8	dn;		/* now wide spread use */
	u8	acoustic;	/* acoustic management */
	u8	media;		/* disk, cdrom, tape, floppy, ... */
	u8	ready_stat;	/* min status value for drive ready */
	u8	mult_count;	/* current multiple sector setting */
	u8	mult_req;	/* requested multiple sector setting */
	u8	io_32bit;	/* 0=16-bit, 1=32-bit, 2/3=32bit+sync */
	u8	bad_wstat;	/* used for ignoring ATA_DF */
	u8	head;		/* "real" number of heads */
	u8	sect;		/* "real" sectors per track */
	u8	bios_head;	/* BIOS/fdisk/LILO number of heads */
	u8	bios_sect;	/* BIOS/fdisk/LILO sectors per track */

	/* delay this long before sending packet command */
	u8 pc_delay;

	unsigned int	bios_cyl;	/* BIOS/fdisk/LILO number of cyls */
	unsigned int	cyl;		/* "real" number of cyls */
	unsigned int	drive_data;	/* used by set_pio_mode/selectproc */
	unsigned int	failures;	/* current failure count */
	unsigned int	max_failures;	/* maximum allowed failure count */
	u64		probed_capacity;/* initial reported media capacity (ide-cd only currently) */

	u64		capacity64;	/* total number of sectors */

	int		lun;		/* logical unit */
	int		crc_count;	/* crc counter to reduce drive speed */

	unsigned long	debug_mask;	/* debugging levels switch */

#ifdef CONFIG_BLK_DEV_IDEACPI
	struct ide_acpi_drive_link *acpidata;
#endif
	struct list_head list;
	struct device	gendev;
	struct completion gendev_rel_comp;	/* to deal with device release() */

	/* current packet command */
	struct ide_atapi_pc *pc;

	/* callback for packet commands */
	void (*pc_callback)(struct ide_drive_s *, int);

	void (*pc_update_buffers)(struct ide_drive_s *, struct ide_atapi_pc *);
	int  (*pc_io_buffers)(struct ide_drive_s *, struct ide_atapi_pc *,
			      unsigned int, int);

	unsigned long atapi_flags;

	struct ide_atapi_pc request_sense_pc;
	struct request request_sense_rq;
};

typedef struct ide_drive_s ide_drive_t;

#define to_ide_device(dev)		container_of(dev, ide_drive_t, gendev)

#define to_ide_drv(obj, cont_type)	\
	container_of(obj, struct cont_type, kref)

#define ide_drv_g(disk, cont_type)	\
	container_of((disk)->private_data, struct cont_type, driver)

struct ide_port_info;

struct ide_tp_ops {
	void	(*exec_command)(struct hwif_s *, u8);
	u8	(*read_status)(struct hwif_s *);
	u8	(*read_altstatus)(struct hwif_s *);
	u8	(*read_sff_dma_status)(struct hwif_s *);

	void	(*set_irq)(struct hwif_s *, int);

	void	(*tf_load)(ide_drive_t *, struct ide_task_s *);
	void	(*tf_read)(ide_drive_t *, struct ide_task_s *);

	void	(*input_data)(ide_drive_t *, struct request *, void *,
			      unsigned int);
	void	(*output_data)(ide_drive_t *, struct request *, void *,
			       unsigned int);
};

extern const struct ide_tp_ops default_tp_ops;

/**
 * struct ide_port_ops - IDE port operations
 *
 * @init_dev:		host specific initialization of a device
 * @set_pio_mode:	routine to program host for PIO mode
 * @set_dma_mode:	routine to program host for DMA mode
 * @selectproc:		tweaks hardware to select drive
 * @reset_poll:		chipset polling based on hba specifics
 * @pre_reset:		chipset specific changes to default for device-hba resets
 * @resetproc:		routine to reset controller after a disk reset
 * @maskproc:		special host masking for drive selection
 * @quirkproc:		check host's drive quirk list
 * @clear_irq:		clear IRQ
 *
 * @mdma_filter:	filter MDMA modes
 * @udma_filter:	filter UDMA modes
 *
 * @cable_detect:	detect cable type
 */
struct ide_port_ops {
	void	(*init_dev)(ide_drive_t *);
	void	(*set_pio_mode)(ide_drive_t *, const u8);
	void	(*set_dma_mode)(ide_drive_t *, const u8);
	void	(*selectproc)(ide_drive_t *);
	int	(*reset_poll)(ide_drive_t *);
	void	(*pre_reset)(ide_drive_t *);
	void	(*resetproc)(ide_drive_t *);
	void	(*maskproc)(ide_drive_t *, int);
	void	(*quirkproc)(ide_drive_t *);
	void	(*clear_irq)(ide_drive_t *);

	u8	(*mdma_filter)(ide_drive_t *);
	u8	(*udma_filter)(ide_drive_t *);

	u8	(*cable_detect)(struct hwif_s *);
};

struct ide_dma_ops {
	void	(*dma_host_set)(struct ide_drive_s *, int);
	int	(*dma_setup)(struct ide_drive_s *);
	void	(*dma_exec_cmd)(struct ide_drive_s *, u8);
	void	(*dma_start)(struct ide_drive_s *);
	int	(*dma_end)(struct ide_drive_s *);
	int	(*dma_test_irq)(struct ide_drive_s *);
	void	(*dma_lost_irq)(struct ide_drive_s *);
	void	(*dma_timeout)(struct ide_drive_s *);
};

struct ide_host;

typedef struct hwif_s {
	struct hwif_s *next;		/* for linked-list in ide_hwgroup_t */
	struct hwif_s *mate;		/* other hwif from same PCI chip */
	struct hwgroup_s *hwgroup;	/* actually (ide_hwgroup_t *) */
	struct proc_dir_entry *proc;	/* /proc/ide/ directory entry */

	struct ide_host *host;

	char name[6];			/* name of interface, eg. "ide0" */

	struct ide_io_ports	io_ports;

	unsigned long	sata_scr[SATA_NR_PORTS];

	ide_drive_t	drives[MAX_DRIVES];	/* drive info */

	u8 major;	/* our major number */
	u8 index;	/* 0 for ide0; 1 for ide1; ... */
	u8 channel;	/* for dual-port chips: 0=primary, 1=secondary */

	u32 host_flags;

	u8 pio_mask;

	u8 ultra_mask;
	u8 mwdma_mask;
	u8 swdma_mask;

	u8 cbl;		/* cable type */

	hwif_chipset_t chipset;	/* sub-module for tuning.. */

	struct device *dev;

	ide_ack_intr_t *ack_intr;

	void (*rw_disk)(ide_drive_t *, struct request *);

	const struct ide_tp_ops		*tp_ops;
	const struct ide_port_ops	*port_ops;
	const struct ide_dma_ops	*dma_ops;

	/* dma physical region descriptor table (cpu view) */
	unsigned int	*dmatable_cpu;
	/* dma physical region descriptor table (dma view) */
	dma_addr_t	dmatable_dma;

	/* maximum number of PRD table entries */
	int prd_max_nents;
	/* PRD entry size in bytes */
	int prd_ent_size;

	/* Scatter-gather list used to build the above */
	struct scatterlist *sg_table;
	int sg_max_nents;		/* Maximum number of entries in it */
	int sg_nents;			/* Current number of entries in it */
	int sg_dma_direction;		/* dma transfer direction */

	/* data phase of the active command (currently only valid for PIO/DMA) */
	int		data_phase;

	struct ide_task_s task;		/* current command */

	unsigned int nsect;
	unsigned int nleft;
	struct scatterlist *cursg;
	unsigned int cursg_ofs;

	int		rqsize;		/* max sectors per request */
	int		irq;		/* our irq number */

	unsigned long	dma_base;	/* base addr for dma ports */

	unsigned long	config_data;	/* for use by chipset-specific code */
	unsigned long	select_data;	/* for use by chipset-specific code */

	unsigned long	extra_base;	/* extra addr for dma ports */
	unsigned	extra_ports;	/* number of extra dma ports */

	unsigned	present    : 1;	/* this interface exists */
	unsigned	serialized : 1;	/* serialized all channel operation */
	unsigned	sharing_irq: 1;	/* 1 = sharing irq with another hwif */
	unsigned	sg_mapped  : 1;	/* sg_table and sg_nents are ready */

	struct device		gendev;
	struct device		*portdev;

	struct completion gendev_rel_comp; /* To deal with device release() */

	void		*hwif_data;	/* extra hwif data */

#ifdef CONFIG_BLK_DEV_IDEACPI
	struct ide_acpi_hwif_link *acpidata;
#endif
} ____cacheline_internodealigned_in_smp ide_hwif_t;

#define MAX_HOST_PORTS 4

struct ide_host {
	ide_hwif_t	*ports[MAX_HOST_PORTS];
	unsigned int	n_ports;
	struct device	*dev[2];
	unsigned int	(*init_chipset)(struct pci_dev *);
	unsigned long	host_flags;
	void		*host_priv;
};

/*
 *  internal ide interrupt handler type
 */
typedef ide_startstop_t (ide_handler_t)(ide_drive_t *);
typedef int (ide_expiry_t)(ide_drive_t *);

/* used by ide-cd, ide-floppy, etc. */
typedef void (xfer_func_t)(ide_drive_t *, struct request *rq, void *, unsigned);

typedef struct hwgroup_s {
		/* irq handler, if active */
	ide_startstop_t	(*handler)(ide_drive_t *);

		/* BOOL: protects all fields below */
	volatile int busy;
		/* BOOL: wake us up on timer expiry */
	unsigned int sleeping	: 1;
		/* BOOL: polling active & poll_timeout field valid */
	unsigned int polling	: 1;

		/* current drive */
	ide_drive_t *drive;
		/* ptr to current hwif in linked-list */
	ide_hwif_t *hwif;

		/* current request */
	struct request *rq;

		/* failsafe timer */
	struct timer_list timer;
		/* timeout value during long polls */
	unsigned long poll_timeout;
		/* queried upon timeouts */
	int (*expiry)(ide_drive_t *);

	int req_gen;
	int req_gen_timer;
} ide_hwgroup_t;

typedef struct ide_driver_s ide_driver_t;

extern struct mutex ide_setting_mtx;

/*
 * configurable drive settings
 */

#define DS_SYNC	(1 << 0)

struct ide_devset {
	int		(*get)(ide_drive_t *);
	int		(*set)(ide_drive_t *, int);
	unsigned int	flags;
};

#define __DEVSET(_flags, _get, _set) { \
	.flags	= _flags, \
	.get	= _get,	\
	.set	= _set,	\
}

#define ide_devset_get(name, field) \
static int get_##name(ide_drive_t *drive) \
{ \
	return drive->field; \
}

#define ide_devset_set(name, field) \
static int set_##name(ide_drive_t *drive, int arg) \
{ \
	drive->field = arg; \
	return 0; \
}

#define ide_devset_get_flag(name, flag) \
static int get_##name(ide_drive_t *drive) \
{ \
	return !!(drive->dev_flags & flag); \
}

#define ide_devset_set_flag(name, flag) \
static int set_##name(ide_drive_t *drive, int arg) \
{ \
	if (arg) \
		drive->dev_flags |= flag; \
	else \
		drive->dev_flags &= ~flag; \
	return 0; \
}

#define __IDE_DEVSET(_name, _flags, _get, _set) \
const struct ide_devset ide_devset_##_name = \
	__DEVSET(_flags, _get, _set)

#define IDE_DEVSET(_name, _flags, _get, _set) \
static __IDE_DEVSET(_name, _flags, _get, _set)

#define ide_devset_rw(_name, _func) \
IDE_DEVSET(_name, 0, get_##_func, set_##_func)

#define ide_devset_w(_name, _func) \
IDE_DEVSET(_name, 0, NULL, set_##_func)

#define ide_ext_devset_rw(_name, _func) \
__IDE_DEVSET(_name, 0, get_##_func, set_##_func)

#define ide_ext_devset_rw_sync(_name, _func) \
__IDE_DEVSET(_name, DS_SYNC, get_##_func, set_##_func)

#define ide_decl_devset(_name) \
extern const struct ide_devset ide_devset_##_name

ide_decl_devset(io_32bit);
ide_decl_devset(keepsettings);
ide_decl_devset(pio_mode);
ide_decl_devset(unmaskirq);
ide_decl_devset(using_dma);

#ifdef CONFIG_IDE_PROC_FS
/*
 * /proc/ide interface
 */

#define ide_devset_rw_field(_name, _field) \
ide_devset_get(_name, _field); \
ide_devset_set(_name, _field); \
IDE_DEVSET(_name, DS_SYNC, get_##_name, set_##_name)

#define ide_devset_rw_flag(_name, _field) \
ide_devset_get_flag(_name, _field); \
ide_devset_set_flag(_name, _field); \
IDE_DEVSET(_name, DS_SYNC, get_##_name, set_##_name)

struct ide_proc_devset {
	const char		*name;
	const struct ide_devset	*setting;
	int			min, max;
	int			(*mulf)(ide_drive_t *);
	int			(*divf)(ide_drive_t *);
};

#define __IDE_PROC_DEVSET(_name, _min, _max, _mulf, _divf) { \
	.name = __stringify(_name), \
	.setting = &ide_devset_##_name, \
	.min = _min, \
	.max = _max, \
	.mulf = _mulf, \
	.divf = _divf, \
}

#define IDE_PROC_DEVSET(_name, _min, _max) \
__IDE_PROC_DEVSET(_name, _min, _max, NULL, NULL)

typedef struct {
	const char	*name;
	mode_t		mode;
	read_proc_t	*read_proc;
	write_proc_t	*write_proc;
} ide_proc_entry_t;

void proc_ide_create(void);
void proc_ide_destroy(void);
void ide_proc_register_port(ide_hwif_t *);
void ide_proc_port_register_devices(ide_hwif_t *);
void ide_proc_unregister_device(ide_drive_t *);
void ide_proc_unregister_port(ide_hwif_t *);
void ide_proc_register_driver(ide_drive_t *, ide_driver_t *);
void ide_proc_unregister_driver(ide_drive_t *, ide_driver_t *);

read_proc_t proc_ide_read_capacity;
read_proc_t proc_ide_read_geometry;

/*
 * Standard exit stuff:
 */
#define PROC_IDE_READ_RETURN(page,start,off,count,eof,len) \
{					\
	len -= off;			\
	if (len < count) {		\
		*eof = 1;		\
		if (len <= 0)		\
			return 0;	\
	} else				\
		len = count;		\
	*start = page + off;		\
	return len;			\
}
#else
static inline void proc_ide_create(void) { ; }
static inline void proc_ide_destroy(void) { ; }
static inline void ide_proc_register_port(ide_hwif_t *hwif) { ; }
static inline void ide_proc_port_register_devices(ide_hwif_t *hwif) { ; }
static inline void ide_proc_unregister_device(ide_drive_t *drive) { ; }
static inline void ide_proc_unregister_port(ide_hwif_t *hwif) { ; }
static inline void ide_proc_register_driver(ide_drive_t *drive, ide_driver_t *driver) { ; }
static inline void ide_proc_unregister_driver(ide_drive_t *drive, ide_driver_t *driver) { ; }
#define PROC_IDE_READ_RETURN(page,start,off,count,eof,len) return 0;
#endif

enum {
	/* enter/exit functions */
	IDE_DBG_FUNC =			(1 << 0),
	/* sense key/asc handling */
	IDE_DBG_SENSE =			(1 << 1),
	/* packet commands handling */
	IDE_DBG_PC =			(1 << 2),
	/* request handling */
	IDE_DBG_RQ =			(1 << 3),
	/* driver probing/setup */
	IDE_DBG_PROBE =			(1 << 4),
};

/* DRV_NAME has to be defined in the driver before using the macro below */
#define __ide_debug_log(lvl, fmt, args...)			\
{								\
	if (unlikely(drive->debug_mask & lvl))			\
		printk(KERN_INFO DRV_NAME ": " fmt, ## args);	\
}

/*
 * Power Management state machine (rq->pm->pm_step).
 *
 * For each step, the core calls ide_start_power_step() first.
 * This can return:
 *	- ide_stopped :	In this case, the core calls us back again unless
 *			step have been set to ide_power_state_completed.
 *	- ide_started :	In this case, the channel is left busy until an
 *			async event (interrupt) occurs.
 * Typically, ide_start_power_step() will issue a taskfile request with
 * do_rw_taskfile().
 *
 * Upon reception of the interrupt, the core will call ide_complete_power_step()
 * with the error code if any. This routine should update the step value
 * and return. It should not start a new request. The core will call
 * ide_start_power_step() for the new step value, unless step have been
 * set to IDE_PM_COMPLETED.
 */
enum {
	IDE_PM_START_SUSPEND,
	IDE_PM_FLUSH_CACHE	= IDE_PM_START_SUSPEND,
	IDE_PM_STANDBY,

	IDE_PM_START_RESUME,
	IDE_PM_RESTORE_PIO	= IDE_PM_START_RESUME,
	IDE_PM_IDLE,
	IDE_PM_RESTORE_DMA,

	IDE_PM_COMPLETED,
};

/*
 * Subdrivers support.
 *
 * The gendriver.owner field should be set to the module owner of this driver.
 * The gendriver.name field should be set to the name of this driver
 */
struct ide_driver_s {
	const char			*version;
	ide_startstop_t	(*do_request)(ide_drive_t *, struct request *, sector_t);
	int		(*end_request)(ide_drive_t *, int, int);
	ide_startstop_t	(*error)(ide_drive_t *, struct request *rq, u8, u8);
	struct device_driver	gen_driver;
	int		(*probe)(ide_drive_t *);
	void		(*remove)(ide_drive_t *);
	void		(*resume)(ide_drive_t *);
	void		(*shutdown)(ide_drive_t *);
#ifdef CONFIG_IDE_PROC_FS
	ide_proc_entry_t *		(*proc_entries)(ide_drive_t *);
	const struct ide_proc_devset *	(*proc_devsets)(ide_drive_t *);
#endif
};

#define to_ide_driver(drv) container_of(drv, ide_driver_t, gen_driver)

int ide_device_get(ide_drive_t *);
void ide_device_put(ide_drive_t *);

struct ide_ioctl_devset {
	unsigned int	get_ioctl;
	unsigned int	set_ioctl;
	const struct ide_devset *setting;
};

int ide_setting_ioctl(ide_drive_t *, struct block_device *, unsigned int,
		      unsigned long, const struct ide_ioctl_devset *);

int generic_ide_ioctl(ide_drive_t *, struct block_device *, unsigned, unsigned long);

extern int ide_vlb_clk;
extern int ide_pci_clk;

extern int ide_end_request (ide_drive_t *drive, int uptodate, int nrsecs);
int ide_end_dequeued_request(ide_drive_t *drive, struct request *rq,
			     int uptodate, int nr_sectors);

extern void ide_set_handler (ide_drive_t *drive, ide_handler_t *handler, unsigned int timeout, ide_expiry_t *expiry);

void ide_execute_command(ide_drive_t *, u8, ide_handler_t *, unsigned int,
			 ide_expiry_t *);

void ide_execute_pkt_cmd(ide_drive_t *);

void ide_pad_transfer(ide_drive_t *, int, int);

ide_startstop_t __ide_error(ide_drive_t *, struct request *, u8, u8);

ide_startstop_t ide_error (ide_drive_t *drive, const char *msg, byte stat);

void ide_fix_driveid(u16 *);

extern void ide_fixstring(u8 *, const int, const int);

int ide_busy_sleep(ide_hwif_t *, unsigned long, int);

int ide_wait_stat(ide_startstop_t *, ide_drive_t *, u8, u8, unsigned long);

extern ide_startstop_t ide_do_reset (ide_drive_t *);

extern int ide_devset_execute(ide_drive_t *drive,
			      const struct ide_devset *setting, int arg);

extern void ide_do_drive_cmd(ide_drive_t *, struct request *);

extern void ide_end_drive_cmd(ide_drive_t *, u8, u8);

void ide_tf_dump(const char *, struct ide_taskfile *);

void ide_exec_command(ide_hwif_t *, u8);
u8 ide_read_status(ide_hwif_t *);
u8 ide_read_altstatus(ide_hwif_t *);
u8 ide_read_sff_dma_status(ide_hwif_t *);

void ide_set_irq(ide_hwif_t *, int);

void ide_tf_load(ide_drive_t *, ide_task_t *);
void ide_tf_read(ide_drive_t *, ide_task_t *);

void ide_input_data(ide_drive_t *, struct request *, void *, unsigned int);
void ide_output_data(ide_drive_t *, struct request *, void *, unsigned int);

int ide_io_buffers(ide_drive_t *, struct ide_atapi_pc *, unsigned int, int);

extern void SELECT_DRIVE(ide_drive_t *);
void SELECT_MASK(ide_drive_t *, int);

u8 ide_read_error(ide_drive_t *);
void ide_read_bcount_and_ireason(ide_drive_t *, u16 *, u8 *);

extern int drive_is_ready(ide_drive_t *);

void ide_pktcmd_tf_load(ide_drive_t *, u32, u16, u8);

int ide_check_atapi_device(ide_drive_t *, const char *);

void ide_init_pc(struct ide_atapi_pc *);

/* Disk head parking */
extern wait_queue_head_t ide_park_wq;
ssize_t ide_park_show(struct device *dev, struct device_attribute *attr,
		      char *buf);
ssize_t ide_park_store(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t len);

/*
 * Special requests for ide-tape block device strategy routine.
 *
 * In order to service a character device command, we add special requests to
 * the tail of our block device request queue and wait for their completion.
 */
enum {
	REQ_IDETAPE_PC1		= (1 << 0), /* packet command (first stage) */
	REQ_IDETAPE_PC2		= (1 << 1), /* packet command (second stage) */
	REQ_IDETAPE_READ	= (1 << 2),
	REQ_IDETAPE_WRITE	= (1 << 3),
};

int ide_queue_pc_tail(ide_drive_t *, struct gendisk *, struct ide_atapi_pc *);

int ide_do_test_unit_ready(ide_drive_t *, struct gendisk *);
int ide_do_start_stop(ide_drive_t *, struct gendisk *, int);
int ide_set_media_lock(ide_drive_t *, struct gendisk *, int);
void ide_create_request_sense_cmd(ide_drive_t *, struct ide_atapi_pc *);
void ide_retry_pc(ide_drive_t *, struct gendisk *);

static inline unsigned long ide_scsi_get_timeout(struct ide_atapi_pc *pc)
{
	return max_t(unsigned long, WAIT_CMD, pc->timeout - jiffies);
}

int ide_scsi_expiry(ide_drive_t *);

ide_startstop_t ide_issue_pc(ide_drive_t *, unsigned int, ide_expiry_t *);

ide_startstop_t do_rw_taskfile(ide_drive_t *, ide_task_t *);

void task_end_request(ide_drive_t *, struct request *, u8);

int ide_raw_taskfile(ide_drive_t *, ide_task_t *, u8 *, u16);
int ide_no_data_taskfile(ide_drive_t *, ide_task_t *);

int ide_taskfile_ioctl(ide_drive_t *, unsigned int, unsigned long);

extern int ide_driveid_update(ide_drive_t *);
extern int ide_config_drive_speed(ide_drive_t *, u8);
extern u8 eighty_ninty_three (ide_drive_t *);
extern int taskfile_lib_get_identify(ide_drive_t *drive, u8 *);

extern int ide_wait_not_busy(ide_hwif_t *hwif, unsigned long timeout);

extern void ide_stall_queue(ide_drive_t *drive, unsigned long timeout);

extern void ide_timer_expiry(unsigned long);
extern irqreturn_t ide_intr(int irq, void *dev_id);
extern void do_ide_request(struct request_queue *);

void ide_init_disk(struct gendisk *, ide_drive_t *);

#ifdef CONFIG_IDEPCI_PCIBUS_ORDER
extern int __ide_pci_register_driver(struct pci_driver *driver, struct module *owner, const char *mod_name);
#define ide_pci_register_driver(d) __ide_pci_register_driver(d, THIS_MODULE, KBUILD_MODNAME)
#else
#define ide_pci_register_driver(d) pci_register_driver(d)
#endif

void ide_pci_setup_ports(struct pci_dev *, const struct ide_port_info *, int,
			 hw_regs_t *, hw_regs_t **);
void ide_setup_pci_noise(struct pci_dev *, const struct ide_port_info *);

#ifdef CONFIG_BLK_DEV_IDEDMA_PCI
int ide_pci_set_master(struct pci_dev *, const char *);
unsigned long ide_pci_dma_base(ide_hwif_t *, const struct ide_port_info *);
int ide_pci_check_simplex(ide_hwif_t *, const struct ide_port_info *);
int ide_hwif_setup_dma(ide_hwif_t *, const struct ide_port_info *);
#else
static inline int ide_hwif_setup_dma(ide_hwif_t *hwif,
				     const struct ide_port_info *d)
{
	return -EINVAL;
}
#endif

typedef struct ide_pci_enablebit_s {
	u8	reg;	/* byte pci reg holding the enable-bit */
	u8	mask;	/* mask to isolate the enable-bit */
	u8	val;	/* value of masked reg when "enabled" */
} ide_pci_enablebit_t;

enum {
	/* Uses ISA control ports not PCI ones. */
	IDE_HFLAG_ISA_PORTS		= (1 << 0),
	/* single port device */
	IDE_HFLAG_SINGLE		= (1 << 1),
	/* don't use legacy PIO blacklist */
	IDE_HFLAG_PIO_NO_BLACKLIST	= (1 << 2),
	/* set for the second port of QD65xx */
	IDE_HFLAG_QD_2ND_PORT		= (1 << 3),
	/* use PIO8/9 for prefetch off/on */
	IDE_HFLAG_ABUSE_PREFETCH	= (1 << 4),
	/* use PIO6/7 for fast-devsel off/on */
	IDE_HFLAG_ABUSE_FAST_DEVSEL	= (1 << 5),
	/* use 100-102 and 200-202 PIO values to set DMA modes */
	IDE_HFLAG_ABUSE_DMA_MODES	= (1 << 6),
	/*
	 * keep DMA setting when programming PIO mode, may be used only
	 * for hosts which have separate PIO and DMA timings (ie. PMAC)
	 */
	IDE_HFLAG_SET_PIO_MODE_KEEP_DMA	= (1 << 7),
	/* program host for the transfer mode after programming device */
	IDE_HFLAG_POST_SET_MODE		= (1 << 8),
	/* don't program host/device for the transfer mode ("smart" hosts) */
	IDE_HFLAG_NO_SET_MODE		= (1 << 9),
	/* trust BIOS for programming chipset/device for DMA */
	IDE_HFLAG_TRUST_BIOS_FOR_DMA	= (1 << 10),
	/* host is CS5510/CS5520 */
	IDE_HFLAG_CS5520		= (1 << 11),
	/* ATAPI DMA is unsupported */
	IDE_HFLAG_NO_ATAPI_DMA		= (1 << 12),
	/* set if host is a "non-bootable" controller */
	IDE_HFLAG_NON_BOOTABLE		= (1 << 13),
	/* host doesn't support DMA */
	IDE_HFLAG_NO_DMA		= (1 << 14),
	/* check if host is PCI IDE device before allowing DMA */
	IDE_HFLAG_NO_AUTODMA		= (1 << 15),
	/* host uses MMIO */
	IDE_HFLAG_MMIO			= (1 << 16),
	/* no LBA48 */
	IDE_HFLAG_NO_LBA48		= (1 << 17),
	/* no LBA48 DMA */
	IDE_HFLAG_NO_LBA48_DMA		= (1 << 18),
	/* data FIFO is cleared by an error */
	IDE_HFLAG_ERROR_STOPS_FIFO	= (1 << 19),
	/* serialize ports */
	IDE_HFLAG_SERIALIZE		= (1 << 20),
	/* use legacy IRQs */
	IDE_HFLAG_LEGACY_IRQS		= (1 << 21),
	/* force use of legacy IRQs */
	IDE_HFLAG_FORCE_LEGACY_IRQS	= (1 << 22),
	/* limit LBA48 requests to 256 sectors */
	IDE_HFLAG_RQSIZE_256		= (1 << 23),
	/* use 32-bit I/O ops */
	IDE_HFLAG_IO_32BIT		= (1 << 24),
	/* unmask IRQs */
	IDE_HFLAG_UNMASK_IRQS		= (1 << 25),
	/* serialize ports if DMA is possible (for sl82c105) */
	IDE_HFLAG_SERIALIZE_DMA		= (1 << 27),
	/* force host out of "simplex" mode */
	IDE_HFLAG_CLEAR_SIMPLEX		= (1 << 28),
	/* DSC overlap is unsupported */
	IDE_HFLAG_NO_DSC		= (1 << 29),
	/* never use 32-bit I/O ops */
	IDE_HFLAG_NO_IO_32BIT		= (1 << 30),
	/* never unmask IRQs */
	IDE_HFLAG_NO_UNMASK_IRQS	= (1 << 31),
};

#ifdef CONFIG_BLK_DEV_OFFBOARD
# define IDE_HFLAG_OFF_BOARD	0
#else
# define IDE_HFLAG_OFF_BOARD	IDE_HFLAG_NON_BOOTABLE
#endif

struct ide_port_info {
	char			*name;
	unsigned int		(*init_chipset)(struct pci_dev *);
	void			(*init_iops)(ide_hwif_t *);
	void                    (*init_hwif)(ide_hwif_t *);
	int			(*init_dma)(ide_hwif_t *,
					    const struct ide_port_info *);

	const struct ide_tp_ops		*tp_ops;
	const struct ide_port_ops	*port_ops;
	const struct ide_dma_ops	*dma_ops;

	ide_pci_enablebit_t	enablebits[2];
	hwif_chipset_t		chipset;
	u32			host_flags;
	u8			pio_mask;
	u8			swdma_mask;
	u8			mwdma_mask;
	u8			udma_mask;
};

int ide_pci_init_one(struct pci_dev *, const struct ide_port_info *, void *);
int ide_pci_init_two(struct pci_dev *, struct pci_dev *,
		     const struct ide_port_info *, void *);
void ide_pci_remove(struct pci_dev *);

#ifdef CONFIG_PM
int ide_pci_suspend(struct pci_dev *, pm_message_t);
int ide_pci_resume(struct pci_dev *);
#else
#define ide_pci_suspend NULL
#define ide_pci_resume NULL
#endif

void ide_map_sg(ide_drive_t *, struct request *);
void ide_init_sg_cmd(ide_drive_t *, struct request *);

#define BAD_DMA_DRIVE		0
#define GOOD_DMA_DRIVE		1

struct drive_list_entry {
	const char *id_model;
	const char *id_firmware;
};

int ide_in_drive_list(u16 *, const struct drive_list_entry *);

#ifdef CONFIG_BLK_DEV_IDEDMA
int ide_dma_good_drive(ide_drive_t *);
int __ide_dma_bad_drive(ide_drive_t *);
int ide_id_dma_bug(ide_drive_t *);

u8 ide_find_dma_mode(ide_drive_t *, u8);

static inline u8 ide_max_dma_mode(ide_drive_t *drive)
{
	return ide_find_dma_mode(drive, XFER_UDMA_6);
}

void ide_dma_off_quietly(ide_drive_t *);
void ide_dma_off(ide_drive_t *);
void ide_dma_on(ide_drive_t *);
int ide_set_dma(ide_drive_t *);
void ide_check_dma_crc(ide_drive_t *);
ide_startstop_t ide_dma_intr(ide_drive_t *);

int ide_allocate_dma_engine(ide_hwif_t *);
void ide_release_dma_engine(ide_hwif_t *);

int ide_build_sglist(ide_drive_t *, struct request *);
void ide_destroy_dmatable(ide_drive_t *);

#ifdef CONFIG_BLK_DEV_IDEDMA_SFF
int config_drive_for_dma(ide_drive_t *);
extern int ide_build_dmatable(ide_drive_t *, struct request *);
void ide_dma_host_set(ide_drive_t *, int);
extern int ide_dma_setup(ide_drive_t *);
void ide_dma_exec_cmd(ide_drive_t *, u8);
extern void ide_dma_start(ide_drive_t *);
int ide_dma_end(ide_drive_t *);
int ide_dma_test_irq(ide_drive_t *);
extern const struct ide_dma_ops sff_dma_ops;
#else
static inline int config_drive_for_dma(ide_drive_t *drive) { return 0; }
#endif /* CONFIG_BLK_DEV_IDEDMA_SFF */

void ide_dma_lost_irq(ide_drive_t *);
void ide_dma_timeout(ide_drive_t *);

#else
static inline int ide_id_dma_bug(ide_drive_t *drive) { return 0; }
static inline u8 ide_find_dma_mode(ide_drive_t *drive, u8 speed) { return 0; }
static inline u8 ide_max_dma_mode(ide_drive_t *drive) { return 0; }
static inline void ide_dma_off_quietly(ide_drive_t *drive) { ; }
static inline void ide_dma_off(ide_drive_t *drive) { ; }
static inline void ide_dma_on(ide_drive_t *drive) { ; }
static inline void ide_dma_verbose(ide_drive_t *drive) { ; }
static inline int ide_set_dma(ide_drive_t *drive) { return 1; }
static inline void ide_check_dma_crc(ide_drive_t *drive) { ; }
static inline void ide_release_dma_engine(ide_hwif_t *hwif) { ; }
#endif /* CONFIG_BLK_DEV_IDEDMA */

#ifdef CONFIG_BLK_DEV_IDEACPI
extern int ide_acpi_exec_tfs(ide_drive_t *drive);
extern void ide_acpi_get_timing(ide_hwif_t *hwif);
extern void ide_acpi_push_timing(ide_hwif_t *hwif);
extern void ide_acpi_init(ide_hwif_t *hwif);
void ide_acpi_port_init_devices(ide_hwif_t *);
extern void ide_acpi_set_state(ide_hwif_t *hwif, int on);
#else
static inline int ide_acpi_exec_tfs(ide_drive_t *drive) { return 0; }
static inline void ide_acpi_get_timing(ide_hwif_t *hwif) { ; }
static inline void ide_acpi_push_timing(ide_hwif_t *hwif) { ; }
static inline void ide_acpi_init(ide_hwif_t *hwif) { ; }
static inline void ide_acpi_port_init_devices(ide_hwif_t *hwif) { ; }
static inline void ide_acpi_set_state(ide_hwif_t *hwif, int on) {}
#endif

void ide_remove_port_from_hwgroup(ide_hwif_t *);
void ide_unregister(ide_hwif_t *);

void ide_register_region(struct gendisk *);
void ide_unregister_region(struct gendisk *);

void ide_undecoded_slave(ide_drive_t *);

void ide_port_apply_params(ide_hwif_t *);

struct ide_host *ide_host_alloc(const struct ide_port_info *, hw_regs_t **);
void ide_host_free(struct ide_host *);
int ide_host_register(struct ide_host *, const struct ide_port_info *,
		      hw_regs_t **);
int ide_host_add(const struct ide_port_info *, hw_regs_t **,
		 struct ide_host **);
void ide_host_remove(struct ide_host *);
int ide_legacy_device_add(const struct ide_port_info *, unsigned long);
void ide_port_unregister_devices(ide_hwif_t *);
void ide_port_scan(ide_hwif_t *);

static inline void *ide_get_hwifdata (ide_hwif_t * hwif)
{
	return hwif->hwif_data;
}

static inline void ide_set_hwifdata (ide_hwif_t * hwif, void *data)
{
	hwif->hwif_data = data;
}

const char *ide_xfer_verbose(u8 mode);
extern void ide_toggle_bounce(ide_drive_t *drive, int on);
extern int ide_set_xfer_rate(ide_drive_t *drive, u8 rate);

u64 ide_get_lba_addr(struct ide_taskfile *, int);
u8 ide_dump_status(ide_drive_t *, const char *, u8);

struct ide_timing {
	u8  mode;
	u8  setup;	/* t1 */
	u16 act8b;	/* t2 for 8-bit io */
	u16 rec8b;	/* t2i for 8-bit io */
	u16 cyc8b;	/* t0 for 8-bit io */
	u16 active;	/* t2 or tD */
	u16 recover;	/* t2i or tK */
	u16 cycle;	/* t0 */
	u16 udma;	/* t2CYCTYP/2 */
};

enum {
	IDE_TIMING_SETUP	= (1 << 0),
	IDE_TIMING_ACT8B	= (1 << 1),
	IDE_TIMING_REC8B	= (1 << 2),
	IDE_TIMING_CYC8B	= (1 << 3),
	IDE_TIMING_8BIT		= IDE_TIMING_ACT8B | IDE_TIMING_REC8B |
				  IDE_TIMING_CYC8B,
	IDE_TIMING_ACTIVE	= (1 << 4),
	IDE_TIMING_RECOVER	= (1 << 5),
	IDE_TIMING_CYCLE	= (1 << 6),
	IDE_TIMING_UDMA		= (1 << 7),
	IDE_TIMING_ALL		= IDE_TIMING_SETUP | IDE_TIMING_8BIT |
				  IDE_TIMING_ACTIVE | IDE_TIMING_RECOVER |
				  IDE_TIMING_CYCLE | IDE_TIMING_UDMA,
};

struct ide_timing *ide_timing_find_mode(u8);
u16 ide_pio_cycle_time(ide_drive_t *, u8);
void ide_timing_merge(struct ide_timing *, struct ide_timing *,
		      struct ide_timing *, unsigned int);
int ide_timing_compute(ide_drive_t *, u8, struct ide_timing *, int, int);

int ide_scan_pio_blacklist(char *);

u8 ide_get_best_pio_mode(ide_drive_t *, u8, u8);

int ide_set_pio_mode(ide_drive_t *, u8);
int ide_set_dma_mode(ide_drive_t *, u8);

void ide_set_pio(ide_drive_t *, u8);

static inline void ide_set_max_pio(ide_drive_t *drive)
{
	ide_set_pio(drive, 255);
}

extern spinlock_t ide_lock;
extern struct mutex ide_cfg_mtx;
/*
 * Structure locking:
 *
 * ide_cfg_mtx and ide_lock together protect changes to
 * ide_hwif_t->{next,hwgroup}
 * ide_drive_t->next
 *
 * ide_hwgroup_t->busy: ide_lock
 * ide_hwgroup_t->hwif: ide_lock
 * ide_hwif_t->mate: constant, no locking
 * ide_drive_t->hwif: constant, no locking
 */

#define local_irq_set(flags)	do { local_save_flags((flags)); local_irq_enable_in_hardirq(); } while (0)

extern struct bus_type ide_bus_type;
extern struct class *ide_port_class;

static inline void ide_dump_identify(u8 *id)
{
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 2, id, 512, 0);
}

static inline int hwif_to_node(ide_hwif_t *hwif)
{
	return hwif->dev ? dev_to_node(hwif->dev) : -1;
}

static inline ide_drive_t *ide_get_pair_dev(ide_drive_t *drive)
{
	ide_drive_t *peer = &drive->hwif->drives[(drive->dn ^ 1) & 1];

	return (peer->dev_flags & IDE_DFLAG_PRESENT) ? peer : NULL;
}
#endif /* _IDE_H */
