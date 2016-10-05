/*
 * platform_sst_pci.c: SST platform data initialization file
 *
 * (C) Copyright 2016 Intel Corporation
 * Author:  Mattijs Korpershoek <mattijsx.korpershoek@intel.com>
 *          Sebastien Guiriec <sebastien.guiriec@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/intel_mid_dma.h>
#include <asm/intel-mid.h>
#include <asm/platform_sst_audio.h>

#define PCI_DEVICE_ID_INTEL_SST_MRFLD 0x119a

#define SST_V2_MAILBOX_RECV           0x400

#define MRFLD_FW_LSP_DDR_BASE         0xc5e00000
#define MRFLD_FW_MOD_END (MRFLD_FW_LSP_DDR_BASE + 0x1fffff)
#define MRFLD_FW_MOD_TABLE_OFFSET     0x80000
#define MRFLD_FW_MOD_TABLE_SIZE       0x100
#define MRFLD_FW_MOD_OFFSET (MRFLD_FW_MOD_TABLE_OFFSET + MRFLD_FW_MOD_TABLE_SIZE)

/* LPE viewpoint addresses */
#define SST_MRFLD_IRAM_PHY_START      0xff2c0000
#define SST_MRFLD_IRAM_PHY_END        0xff2d4000
#define SST_MRFLD_DRAM_PHY_START      0xff300000
#define SST_MRFLD_DRAM_PHY_END        0xff320000
#define SST_MRFLD_IMR_VIRT_START      0xc5e00000
#define SST_MRFLD_IMR_VIRT_END        0xc5ffffff
#define SST_MRFLD_SHIM_PHY_ADDR       0xff340000
#define SST_MRFLD_MBOX_PHY_ADDR       0xff344000
#define SST_MRFLD_DMA0_PHY_ADDR       0xff298000
#define SST_MRFLD_DMA1_PHY_ADDR       0xff29c000
#define SST_MRFLD_SSP0_PHY_ADDR       0xff2a0000
#define SST_MRFLD_SSP2_PHY_ADDR       0xff2a2000


struct sst_platform_info sst_data;

static const struct sst_info mrfld_sst_info = {
	.use_elf             = false,
	.max_streams         = MAX_NUM_STREAMS_MRFLD,
	.iram_use            = true,
	.iram_start          = SST_MRFLD_IRAM_PHY_START,
	.iram_end            = SST_MRFLD_IRAM_PHY_END,
	.dram_use            = true,
	.dram_start          = SST_MRFLD_DRAM_PHY_START,
	.dram_end            = SST_MRFLD_DRAM_PHY_END,
	.imr_use             = true,
	.imr_start           = SST_MRFLD_IMR_VIRT_START,
	.imr_end             = SST_MRFLD_IMR_VIRT_END,
	.mailbox_start       = SST_MRFLD_MBOX_PHY_ADDR,
	.dma_max_len         = SST_MAX_DMA_LEN_MRFLD,
	.num_probes          = 0,
	.lpe_viewpt_rqd      = true,
};

static const struct sst_ipc_info mrfld_ipc_info = {
	.ipc_offset          = 0,
	.mbox_recv_off       = SST_V2_MAILBOX_RECV,
};

static const struct sst_lib_dnld_info  mrfld_lib_dnld_info = {
	.mod_base            = MRFLD_FW_LSP_DDR_BASE,
	.mod_end             = MRFLD_FW_MOD_END,
	.mod_table_offset    = MRFLD_FW_MOD_TABLE_OFFSET,
	.mod_table_size      = MRFLD_FW_MOD_TABLE_SIZE,
	.mod_ddr_dnld        = true,
};

static const struct sst_res_info mrfld_res_info = {
	.shim_offset         = 0x140000,
	.shim_size           = 0x000100,
	.shim_phy_addr       = 0xff340000,
	.ssp0_offset         = 0xa0000,
	.ssp0_size           = 0x1000,
	.dma0_offset         = 0x98000,
	.dma0_size           = 0x4000,
	.dma1_offset         = 0x9c000,
	.dma1_size           = 0x4000,
	.iram_offset         = 0x0c0000,
	.iram_size           = 0x14000,
	.dram_offset         = 0x100000,
	.dram_size           = 0x28000,
	.mbox_offset         = 0x144000,
	.mbox_size           = 0x1000,
};

static void set_mrfld_sst_config(struct sst_platform_info *sst_info)
{
	sst_info->probe_data = &mrfld_sst_info;
	sst_info->ipc_info   = &mrfld_ipc_info;
	sst_info->lib_info   = &mrfld_lib_dnld_info;
	sst_info->res_info   = &mrfld_res_info;
	sst_info->platform   = "sst-mfld-platform";

	return;
}

static struct sst_platform_info *get_sst_platform_data(struct pci_dev *pdev)
{
	struct sst_platform_info *sst_pinfo = NULL;

	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_SST_MRFLD:
		set_mrfld_sst_config(&sst_data);
		sst_pinfo = &sst_data;
		break;
	default:
		return NULL;
	}

	return sst_pinfo;
}

static void sst_pci_early_quirks(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = get_sst_platform_data(pci_dev);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_SST_MRFLD,
			sst_pci_early_quirks);
