#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>
#include <linux/netdevice.h>

#include "igb.h"

#define DRV_VERSION "1.0"

char igb_driver_name[] = "igb_blank";
char igb_driver_version[] = DRV_VERSION;

static DEFINE_PCI_DEVICE_TABLE(igb_pci_tbl_blank) = {
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I211_BLANK) },
	/* required last entry */
	{0, }
};
MODULE_DEVICE_TABLE(pci, igb_pci_tbl_blank);

static int igb_probe_blank(struct pci_dev *, const struct pci_device_id *);
static void __devexit igb_remove_blank(struct pci_dev *pdev);

static int __devinit igb_probe_blank(struct pci_dev *pdev,
			       const struct pci_device_id *ent)
{
	struct net_device *netdev;
	struct igb_adapter *adapter;
	struct e1000_hw *hw;
	int err, pci_using_dac;

	err = pci_enable_device_mem(pdev);
	if (err)
		return err;

	pci_using_dac = 0;
	err = dma_set_mask(pci_dev_to_dev(pdev), DMA_BIT_MASK(64));
	if (!err) {
		err = dma_set_coherent_mask(pci_dev_to_dev(pdev), DMA_BIT_MASK(64));
		if (!err)
			pci_using_dac = 1;
	} else {
		err = dma_set_mask(pci_dev_to_dev(pdev), DMA_BIT_MASK(32));
		if (err) {
			err = dma_set_coherent_mask(pci_dev_to_dev(pdev), DMA_BIT_MASK(32));
			if (err) {
				IGB_ERR("No usable DMA configuration, "
				        "aborting\n");
				goto err_dma;
			}
		}
	}

	pci_set_master(pdev);

	netdev = alloc_etherdev(sizeof(struct igb_adapter));
	SET_NETDEV_DEV(netdev, &pdev->dev);
	pci_set_drvdata(pdev, netdev);
	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	hw = &adapter->hw;
	hw->back = adapter;

	err = pci_request_selected_regions(pdev,
	                                   pci_select_bars(pdev,
                                                           IORESOURCE_MEM),
	                                   igb_driver_name);
	if (err)
		goto err_pci_reg;

	err = -EIO;
	hw->hw_addr = ioremap(pci_resource_start(pdev, 0),
	                      pci_resource_len(pdev, 0));
	if (!hw->hw_addr)
		goto err_ioremap;

	return 0;

err_ioremap:
	pci_release_selected_regions(pdev,
	                             pci_select_bars(pdev, IORESOURCE_MEM));
err_pci_reg:
err_dma:
	pci_disable_device(pdev);
	return err;
}

static void __devexit igb_remove_blank(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	iounmap(hw->hw_addr);
	if (hw->flash_address)
		iounmap(hw->flash_address);
	pci_release_selected_regions(pdev,
	                    pci_select_bars(pdev, IORESOURCE_MEM));

	free_netdev(netdev);

	pci_disable_device(pdev);
}

static struct pci_driver igb_driver_blank = {
	.name     = igb_driver_name,
	.id_table = igb_pci_tbl_blank,
	.probe    = igb_probe_blank,
	.remove   = __devexit_p(igb_remove_blank),
};

static int __init igb_init_module_blank(void)
{
	int ret;
	ret = pci_register_driver(&igb_driver_blank);
	return ret;
}
module_init(igb_init_module_blank);

static void __exit igb_exit_module_blank(void)
{
	pci_unregister_driver(&igb_driver_blank);
}
module_exit(igb_exit_module_blank);


MODULE_AUTHOR("CompuLab");
MODULE_DESCRIPTION("CompuLab Blank IGB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
