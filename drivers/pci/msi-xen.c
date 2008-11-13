/*
 * File:	msi.c
 * Purpose:	PCI Message Signaled Interrupt (MSI)
 *
 * Copyright (C) 2003-2004 Intel
 * Copyright (C) Tom Long Nguyen (tom.l.nguyen@intel.com)
 */

#include <linux/mm.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/smp_lock.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>

#include <asm/errno.h>
#include <asm/io.h>
#include <asm/smp.h>

#include "pci.h"
#include "msi.h"

static int pci_msi_enable = 1;

static struct msi_ops *msi_ops;

int msi_register(struct msi_ops *ops)
{
	msi_ops = ops;
	return 0;
}

static LIST_HEAD(msi_dev_head);
DEFINE_SPINLOCK(msi_dev_lock);

struct msi_dev_list {
	struct pci_dev *dev;
	struct list_head list;
	spinlock_t pirq_list_lock;
	struct list_head pirq_list_head;
};

struct msi_pirq_entry {
	struct list_head list;
	int pirq;
	int entry_nr;
};

static struct msi_dev_list *get_msi_dev_pirq_list(struct pci_dev *dev)
{
	struct msi_dev_list *msi_dev_list, *ret = NULL;
	unsigned long flags;

	spin_lock_irqsave(&msi_dev_lock, flags);

	list_for_each_entry(msi_dev_list, &msi_dev_head, list)
		if ( msi_dev_list->dev == dev )
			ret = msi_dev_list;

	if ( ret ) {
		spin_unlock_irqrestore(&msi_dev_lock, flags);
		return ret;
	}

	/* Has not allocate msi_dev until now. */
	ret = kmalloc(sizeof(struct msi_dev_list), GFP_ATOMIC);

	/* Failed to allocate msi_dev structure */
	if ( !ret ) {
		spin_unlock_irqrestore(&msi_dev_lock, flags);
		return NULL;
	}

	spin_lock_init(&ret->pirq_list_lock);
	INIT_LIST_HEAD(&ret->pirq_list_head);
	list_add_tail(&ret->list, &msi_dev_head);
	spin_unlock_irqrestore(&msi_dev_lock, flags);
	return ret;
}

static int attach_pirq_entry(int pirq, int entry_nr,
                             struct msi_dev_list *msi_dev_entry)
{
	struct msi_pirq_entry *entry = kmalloc(sizeof(*entry), GFP_ATOMIC);
	unsigned long flags;

	if (!entry)
		return -ENOMEM;
	entry->pirq = pirq;
	entry->entry_nr = entry_nr;
	spin_lock_irqsave(&msi_dev_entry->pirq_list_lock, flags);
	list_add_tail(&entry->list, &msi_dev_entry->pirq_list_head);
	spin_unlock_irqrestore(&msi_dev_entry->pirq_list_lock, flags);
	return 0;
}

static void detach_pirq_entry(int entry_nr,
 							struct msi_dev_list *msi_dev_entry)
{
	unsigned long flags;
	struct msi_pirq_entry *pirq_entry;

	list_for_each_entry(pirq_entry, &msi_dev_entry->pirq_list_head, list) {
		if (pirq_entry->entry_nr == entry_nr) {
			spin_lock_irqsave(&msi_dev_entry->pirq_list_lock, flags);
			list_del(&pirq_entry->list);
			spin_unlock_irqrestore(&msi_dev_entry->pirq_list_lock, flags);
			kfree(pirq_entry);
			return;
		}
	}
}

/*
 * pciback will provide device's owner
 */
int (*get_owner)(struct pci_dev *dev);

int register_msi_get_owner(int (*func)(struct pci_dev *dev))
{
	if (get_owner) {
		printk(KERN_WARNING "register msi_get_owner again\n");
		return -EEXIST;
	}
	get_owner = func;
	return 0;
}

int unregister_msi_get_owner(int (*func)(struct pci_dev *dev))
{
	if (get_owner == func)
		get_owner = NULL;
	return 0;
}

static int msi_get_dev_owner(struct pci_dev *dev)
{
	int owner = DOMID_SELF;

	BUG_ON(!is_initial_xendomain());
	if (get_owner && (owner = get_owner(dev)) >=0 ) {
		printk(KERN_INFO "get owner for dev %x get %x \n",
				    dev->devfn, owner);
		return owner;
	}
	else
		return DOMID_SELF;
}

static int msi_unmap_pirq(struct pci_dev *dev, int pirq)
{
	struct physdev_unmap_pirq unmap;
	int rc;
	domid_t domid = DOMID_SELF;

	domid = msi_get_dev_owner(dev);
	unmap.domid = domid;
	unmap.pirq = pirq;

	if ((rc = HYPERVISOR_physdev_op(PHYSDEVOP_unmap_pirq, &unmap)))
		printk(KERN_WARNING "unmap irq %x failed\n", pirq);

	if (rc < 0)
		return rc;
    return 0;
}

/*
 * Protected by msi_lock
 */
static int msi_map_pirq_to_vector(struct pci_dev *dev, int pirq,
                                  int entry_nr, int msi)
{
	struct physdev_map_pirq map_irq;
	int rc;
	domid_t domid = DOMID_SELF;

	domid = msi_get_dev_owner(dev);

	map_irq.domid = domid;
	map_irq.type = MAP_PIRQ_TYPE_MSI;
	map_irq.index = -1;
	map_irq.pirq = pirq;
    map_irq.msi_info.bus = dev->bus->number;
    map_irq.msi_info.devfn = dev->devfn;
	map_irq.msi_info.entry_nr = entry_nr;
    map_irq.msi_info.msi = msi;

	if ((rc = HYPERVISOR_physdev_op(PHYSDEVOP_map_pirq, &map_irq)))
		printk(KERN_WARNING "map irq failed\n");

	if (rc < 0)
		return rc;

	return map_irq.pirq;
}

static int msi_map_vector(struct pci_dev *dev, int entry_nr, int msi)
{
	return msi_map_pirq_to_vector(dev, -1, entry_nr, msi);
}

static int msi_init(void)
{
	static int status = 0;

	if (pci_msi_quirk) {
		pci_msi_enable = 0;
		printk(KERN_WARNING "PCI: MSI quirk detected. MSI disabled.\n");
		status = -EINVAL;
	}

	return status;
}

void pci_scan_msi_device(struct pci_dev *dev) { }

void disable_msi_mode(struct pci_dev *dev, int pos, int type)
{
	u16 control;

	pci_read_config_word(dev, msi_control_reg(pos), &control);
	if (type == PCI_CAP_ID_MSI) {
		/* Set enabled bits to single MSI & enable MSI_enable bit */
		msi_disable(control);
		pci_write_config_word(dev, msi_control_reg(pos), control);
		dev->msi_enabled = 0;
	} else {
		msix_disable(control);
		pci_write_config_word(dev, msi_control_reg(pos), control);
		dev->msix_enabled = 0;
	}
    	if (pci_find_capability(dev, PCI_CAP_ID_EXP)) {
		/* PCI Express Endpoint device detected */
		pci_intx(dev, 1);  /* enable intx */
	}
}

static void enable_msi_mode(struct pci_dev *dev, int pos, int type)
{
	u16 control;

	pci_read_config_word(dev, msi_control_reg(pos), &control);
	if (type == PCI_CAP_ID_MSI) {
		/* Set enabled bits to single MSI & enable MSI_enable bit */
		msi_enable(control, 1);
		pci_write_config_word(dev, msi_control_reg(pos), control);
		dev->msi_enabled = 1;
	} else {
		msix_enable(control);
		pci_write_config_word(dev, msi_control_reg(pos), control);
		dev->msix_enabled = 1;
	}
    	if (pci_find_capability(dev, PCI_CAP_ID_EXP)) {
		/* PCI Express Endpoint device detected */
		pci_intx(dev, 0);  /* disable intx */
	}
}

#ifdef CONFIG_PM
int pci_save_msi_state(struct pci_dev *dev)
{
	int pos;

	pos = pci_find_capability(dev, PCI_CAP_ID_MSI);
	if (pos <= 0 || dev->no_msi)
		return 0;

	if (!dev->msi_enabled)
		return 0;

	/* Restore dev->irq to its default pin-assertion vector */
	msi_unmap_pirq(dev, dev->irq);
	/* Disable MSI mode */
	disable_msi_mode(dev, pos, PCI_CAP_ID_MSI);
	/* Set the flags for use of restore */
	dev->msi_enabled = 1;
	return 0;
}

void pci_restore_msi_state(struct pci_dev *dev)
{
	int pos, pirq;

	pos = pci_find_capability(dev, PCI_CAP_ID_MSI);
	if (pos <= 0)
		return;

	if (!dev->msi_enabled)
		return;

	pirq = msi_map_pirq_to_vector(dev, dev->irq, 0, 1);
	if (pirq < 0)
		return;
	enable_msi_mode(dev, pos, PCI_CAP_ID_MSI);
}

int pci_save_msix_state(struct pci_dev *dev)
{
	int pos;
	unsigned long flags;
	struct msi_dev_list *msi_dev_entry;
	struct msi_pirq_entry *pirq_entry, *tmp;

	pos = pci_find_capability(dev, PCI_CAP_ID_MSIX);
	if (pos <= 0 || dev->no_msi)
		return 0;

	/* save the capability */
	if (!dev->msix_enabled)
		return 0;

	msi_dev_entry = get_msi_dev_pirq_list(dev);

	spin_lock_irqsave(&msi_dev_entry->pirq_list_lock, flags);
        list_for_each_entry_safe(pirq_entry, tmp,
                                 &msi_dev_entry->pirq_list_head, list)
		msi_unmap_pirq(dev, pirq_entry->pirq);
	spin_unlock_irqrestore(&msi_dev_entry->pirq_list_lock, flags);

	disable_msi_mode(dev, pos, PCI_CAP_ID_MSIX);
	/* Set the flags for use of restore */
	dev->msix_enabled = 1;

	return 0;
}

void pci_restore_msix_state(struct pci_dev *dev)
{
	int pos;
	unsigned long flags;
	struct msi_dev_list *msi_dev_entry;
	struct msi_pirq_entry *pirq_entry, *tmp;

	pos = pci_find_capability(dev, PCI_CAP_ID_MSIX);
	if (pos <= 0)
		return;

	if (!dev->msix_enabled)
		return;

	msi_dev_entry = get_msi_dev_pirq_list(dev);

	spin_lock_irqsave(&msi_dev_entry->pirq_list_lock, flags);
	list_for_each_entry_safe(pirq_entry, tmp,
							 &msi_dev_entry->pirq_list_head, list)
		msi_map_pirq_to_vector(dev, pirq_entry->pirq, pirq_entry->entry_nr, 0);
	spin_unlock_irqrestore(&msi_dev_entry->pirq_list_lock, flags);

	enable_msi_mode(dev, pos, PCI_CAP_ID_MSIX);
}
#endif

/**
 * msi_capability_init - configure device's MSI capability structure
 * @dev: pointer to the pci_dev data structure of MSI device function
 *
 * Setup the MSI capability structure of device function with a single
 * MSI vector, regardless of device function is capable of handling
 * multiple messages. A return of zero indicates the successful setup
 * of an entry zero with the new MSI vector or non-zero for otherwise.
 **/
static int msi_capability_init(struct pci_dev *dev)
{
	int pos, pirq;
	u16 control;

   	pos = pci_find_capability(dev, PCI_CAP_ID_MSI);
	pci_read_config_word(dev, msi_control_reg(pos), &control);

	pirq = msi_map_vector(dev, 0, 1);
	if (pirq < 0)
		return -EBUSY;

	dev->irq = pirq;
	/* Set MSI enabled bits	 */
	enable_msi_mode(dev, pos, PCI_CAP_ID_MSI);
	dev->msi_enabled = 1;

	return 0;
}

/**
 * msix_capability_init - configure device's MSI-X capability
 * @dev: pointer to the pci_dev data structure of MSI-X device function
 * @entries: pointer to an array of struct msix_entry entries
 * @nvec: number of @entries
 *
 * Setup the MSI-X capability structure of device function with a
 * single MSI-X vector. A return of zero indicates the successful setup of
 * requested MSI-X entries with allocated vectors or non-zero for otherwise.
 **/
static int msix_capability_init(struct pci_dev *dev,
				struct msix_entry *entries, int nvec)
{
	int pirq, i, j, mapped, pos;
	struct msi_dev_list *msi_dev_entry = get_msi_dev_pirq_list(dev);
	struct msi_pirq_entry *pirq_entry;

	if (!msi_dev_entry)
		return -ENOMEM;

	/* MSI-X Table Initialization */
	for (i = 0; i < nvec; i++) {
		mapped = 0;
		list_for_each_entry(pirq_entry, &msi_dev_entry->pirq_list_head, list) {
			if (pirq_entry->entry_nr == entries[i].entry) {
				printk(KERN_WARNING "msix entry %d for dev %02x:%02x:%01x are \
				       not freed before acquire again.\n", entries[i].entry,
					   dev->bus->number, PCI_SLOT(dev->devfn),
					   PCI_FUNC(dev->devfn));
				(entries + i)->vector = pirq_entry->pirq;
				mapped = 1;
				break;
			}
		}
		if (mapped)
			continue;
		pirq = msi_map_vector(dev, entries[i].entry, 0);
		if (pirq < 0)
			break;
		attach_pirq_entry(pirq, entries[i].entry, msi_dev_entry);
		(entries + i)->vector = pirq;
	}

	if (i != nvec) {
		for (j = --i; j >= 0; j--) {
			msi_unmap_pirq(dev, entries[j].vector);
			detach_pirq_entry(entries[j].entry, msi_dev_entry);
			entries[j].vector = 0;
		}
		return -EBUSY;
	}

	pos = pci_find_capability(dev, PCI_CAP_ID_MSIX);
	enable_msi_mode(dev, pos, PCI_CAP_ID_MSIX);
	dev->msix_enabled = 1;

	return 0;
}

/**
 * pci_enable_msi - configure device's MSI capability structure
 * @dev: pointer to the pci_dev data structure of MSI device function
 *
 * Setup the MSI capability structure of device function with
 * a single MSI vector upon its software driver call to request for
 * MSI mode enabled on its hardware device function. A return of zero
 * indicates the successful setup of an entry zero with the new MSI
 * vector or non-zero for otherwise.
 **/
extern int pci_frontend_enable_msi(struct pci_dev *dev);
int pci_enable_msi(struct pci_dev* dev)
{
	struct pci_bus *bus;
	int pos, temp, status = -EINVAL;

	if (!pci_msi_enable || !dev)
 		return status;

	if (dev->no_msi)
		return status;

	for (bus = dev->bus; bus; bus = bus->parent)
		if (bus->bus_flags & PCI_BUS_FLAGS_NO_MSI)
			return -EINVAL;

	status = msi_init();
	if (status < 0)
		return status;

#ifdef CONFIG_XEN_PCIDEV_FRONTEND
	if (!is_initial_xendomain())
	{
		int ret;

		temp = dev->irq;
		ret = pci_frontend_enable_msi(dev);
		if (ret)
			return ret;

		dev->irq_old = temp;

		return ret;
	}
#endif

	temp = dev->irq;

	pos = pci_find_capability(dev, PCI_CAP_ID_MSI);
	if (!pos)
		return -EINVAL;

	/* Check whether driver already requested for MSI-X vectors */
	if (dev->msix_enabled) {
		printk(KERN_INFO "PCI: %s: Can't enable MSI.  "
			   "Device already has MSI-X vectors assigned\n",
			   pci_name(dev));
		dev->irq = temp;
		return -EINVAL;
	}

	status = msi_capability_init(dev);
	if ( !status )
		dev->irq_old = temp;
    else
		dev->irq = temp;

	return status;
}

extern void pci_frontend_disable_msi(struct pci_dev* dev);
void pci_disable_msi(struct pci_dev* dev)
{
	int pos;
	int pirq;

	if (!pci_msi_enable)
		return;
	if (!dev)
		return;

#ifdef CONFIG_XEN_PCIDEV_FRONTEND
	if (!is_initial_xendomain()) {
		pci_frontend_disable_msi(dev);
		dev->irq = dev->irq_old;
		return;
	}
#endif

	pos = pci_find_capability(dev, PCI_CAP_ID_MSI);
	if (!pos)
		return;

	pirq = dev->irq;
	/* Restore dev->irq to its default pin-assertion vector */
	dev->irq = dev->irq_old;
	msi_unmap_pirq(dev, pirq);

	/* Disable MSI mode */
	disable_msi_mode(dev, pos, PCI_CAP_ID_MSI);
}

/**
 * pci_enable_msix - configure device's MSI-X capability structure
 * @dev: pointer to the pci_dev data structure of MSI-X device function
 * @entries: pointer to an array of MSI-X entries
 * @nvec: number of MSI-X vectors requested for allocation by device driver
 *
 * Setup the MSI-X capability structure of device function with the number
 * of requested vectors upon its software driver call to request for
 * MSI-X mode enabled on its hardware device function. A return of zero
 * indicates the successful configuration of MSI-X capability structure
 * with new allocated MSI-X vectors. A return of < 0 indicates a failure.
 * Or a return of > 0 indicates that driver request is exceeding the number
 * of vectors available. Driver should use the returned value to re-send
 * its request.
 **/
extern int pci_frontend_enable_msix(struct pci_dev *dev,
		struct msix_entry *entries, int nvec);
int pci_enable_msix(struct pci_dev* dev, struct msix_entry *entries, int nvec)
{
	struct pci_bus *bus;
	int status, pos, nr_entries;
	int i, j, temp;
	u16 control;

	if (!pci_msi_enable || !dev || !entries)
 		return -EINVAL;

	if (dev->no_msi)
		return -EINVAL;

	for (bus = dev->bus; bus; bus = bus->parent)
		if (bus->bus_flags & PCI_BUS_FLAGS_NO_MSI)
			return -EINVAL;

#ifdef CONFIG_XEN_PCIDEV_FRONTEND
	if (!is_initial_xendomain()) {
		int ret;

		ret = pci_frontend_enable_msix(dev, entries, nvec);
		if (ret) {
			printk("get %x from pci_frontend_enable_msix\n", ret);
			return ret;
		}

        return 0;
	}
#endif

	status = msi_init();
	if (status < 0)
		return status;

	pos = pci_find_capability(dev, PCI_CAP_ID_MSIX);
	if (!pos)
 		return -EINVAL;

	pci_read_config_word(dev, msi_control_reg(pos), &control);
	nr_entries = multi_msix_capable(control);
	if (nvec > nr_entries)
		return -EINVAL;

	/* Check for any invalid entries */
	for (i = 0; i < nvec; i++) {
		if (entries[i].entry >= nr_entries)
			return -EINVAL;		/* invalid entry */
		for (j = i + 1; j < nvec; j++) {
			if (entries[i].entry == entries[j].entry)
				return -EINVAL;	/* duplicate entry */
		}
	}

	temp = dev->irq;
	/* Check whether driver already requested for MSI vector */
	if (dev->msi_enabled) {
		printk(KERN_INFO "PCI: %s: Can't enable MSI-X.  "
		       "Device already has an MSI vector assigned\n",
		       pci_name(dev));
		dev->irq = temp;
		return -EINVAL;
	}

	status = msix_capability_init(dev, entries, nvec);

	if ( !status )
		dev->irq_old = temp;
	else
		dev->irq = temp;

	return status;
}

extern void pci_frontend_disable_msix(struct pci_dev* dev);
void pci_disable_msix(struct pci_dev* dev)
{
	int pos;
	u16 control;


	if (!pci_msi_enable)
		return;
	if (!dev)
		return;

#ifdef CONFIG_XEN_PCIDEV_FRONTEND
	if (!is_initial_xendomain()) {
		pci_frontend_disable_msix(dev);
		dev->irq = dev->irq_old;
		return;
	}
#endif

	pos = pci_find_capability(dev, PCI_CAP_ID_MSIX);
	if (!pos)
		return;

	pci_read_config_word(dev, msi_control_reg(pos), &control);
	if (!(control & PCI_MSIX_FLAGS_ENABLE))
		return;

	msi_remove_pci_irq_vectors(dev);

	/* Disable MSI mode */
	disable_msi_mode(dev, pos, PCI_CAP_ID_MSIX);
}

/**
 * msi_remove_pci_irq_vectors - reclaim MSI(X) vectors to unused state
 * @dev: pointer to the pci_dev data structure of MSI(X) device function
 *
 * Being called during hotplug remove, from which the device function
 * is hot-removed. All previous assigned MSI/MSI-X vectors, if
 * allocated for this device function, are reclaimed to unused state,
 * which may be used later on.
 **/
void msi_remove_pci_irq_vectors(struct pci_dev* dev)
{
	unsigned long flags;
	struct msi_dev_list *msi_dev_entry;
	struct msi_pirq_entry *pirq_entry, *tmp;

	if (!pci_msi_enable || !dev)
 		return;

	msi_dev_entry = get_msi_dev_pirq_list(dev);

	spin_lock_irqsave(&msi_dev_entry->pirq_list_lock, flags);
	if (!list_empty(&msi_dev_entry->pirq_list_head))
	{
		printk(KERN_WARNING "msix pirqs for dev %02x:%02x:%01x are not freed \
		       before acquire again.\n", dev->bus->number, PCI_SLOT(dev->devfn),
			   PCI_FUNC(dev->devfn));
		list_for_each_entry_safe(pirq_entry, tmp,
		                         &msi_dev_entry->pirq_list_head, list) {
			msi_unmap_pirq(dev, pirq_entry->pirq);
			list_del(&pirq_entry->list);
			kfree(pirq_entry);
		}
	}
	spin_unlock_irqrestore(&msi_dev_entry->pirq_list_lock, flags);
	dev->irq = dev->irq_old;
}

void pci_no_msi(void)
{
	pci_msi_enable = 0;
}

EXPORT_SYMBOL(pci_enable_msi);
EXPORT_SYMBOL(pci_disable_msi);
EXPORT_SYMBOL(pci_enable_msix);
EXPORT_SYMBOL(pci_disable_msix);
#ifdef CONFIG_XEN
EXPORT_SYMBOL(register_msi_get_owner);
EXPORT_SYMBOL(unregister_msi_get_owner);
#endif

