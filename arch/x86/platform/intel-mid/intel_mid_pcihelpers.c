#include <linux/export.h>
#include <linux/pci.h>

#include <asm/intel_mid_pcihelpers.h>

/* Unified message bus read/write operation */
static DEFINE_SPINLOCK(msgbus_lock);

static struct pci_dev *pci_root;

static int intel_mid_msgbus_init(void)
{
        pci_root = pci_get_bus_and_slot(0, PCI_DEVFN(0, 0));
        if (!pci_root) {
                printk(KERN_ALERT "%s: Error: msgbus PCI handle NULL",
                        __func__);
                return -ENODEV;
        }
        return 0;
}
fs_initcall(intel_mid_msgbus_init);

/* called only from where is later then fs_initcall */
u32 intel_mid_soc_stepping(void)
{
        return pci_root->revision;
}
EXPORT_SYMBOL(intel_mid_soc_stepping);

