/*
* Support for Intel Camera Imaging ISP subsystem.
 * Copyright (c) 2010 - 2018, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
*/

#include "vied_nci_psys_resource_model.h"

/*
 * Cell types by cell IDs
 */
const vied_nci_cell_type_ID_t vied_nci_cell_type[VIED_NCI_N_CELL_ID] = {
	VIED_NCI_SP_CTRL_TYPE_ID,
	VIED_NCI_SP_SERVER_TYPE_ID,
	VIED_NCI_SP_SERVER_TYPE_ID,
	VIED_NCI_VP_TYPE_ID,
	VIED_NCI_VP_TYPE_ID,
	VIED_NCI_VP_TYPE_ID,
	VIED_NCI_VP_TYPE_ID,
	VIED_NCI_ACC_ISA_TYPE_ID,
	VIED_NCI_ACC_PSA_TYPE_ID,
	VIED_NCI_ACC_PSA_TYPE_ID,
	VIED_NCI_ACC_PSA_TYPE_ID,
	VIED_NCI_ACC_PSA_TYPE_ID,
	VIED_NCI_ACC_PSA_TYPE_ID,
	VIED_NCI_ACC_PSA_TYPE_ID,
	VIED_NCI_ACC_OSA_TYPE_ID,
	VIED_NCI_GDC_TYPE_ID,
	VIED_NCI_GDC_TYPE_ID
};

/*
 * Memory types by memory IDs
 */
const vied_nci_mem_type_ID_t vied_nci_mem_type[VIED_NCI_N_MEM_ID] = {
	VIED_NCI_VMEM_TYPE_ID,
	VIED_NCI_VMEM_TYPE_ID,
	VIED_NCI_VMEM_TYPE_ID,
	VIED_NCI_VMEM_TYPE_ID,
	VIED_NCI_GMEM_TYPE_ID,/* VMEM4 is GMEM according to vied_nci_cell_mem */
	VIED_NCI_BAMEM_TYPE_ID,
	VIED_NCI_BAMEM_TYPE_ID,
	VIED_NCI_BAMEM_TYPE_ID,
	VIED_NCI_BAMEM_TYPE_ID,
	VIED_NCI_DMEM_TYPE_ID,
	VIED_NCI_DMEM_TYPE_ID,
	VIED_NCI_DMEM_TYPE_ID,
	VIED_NCI_DMEM_TYPE_ID,
	VIED_NCI_DMEM_TYPE_ID,
	VIED_NCI_DMEM_TYPE_ID,
	VIED_NCI_DMEM_TYPE_ID,
	VIED_NCI_DMEM_TYPE_ID,
	VIED_NCI_PMEM_TYPE_ID,
	VIED_NCI_PMEM_TYPE_ID,
	VIED_NCI_PMEM_TYPE_ID,
	VIED_NCI_PMEM_TYPE_ID
};

/*
 * Cell mem count by cell type ID
 */
const uint16_t vied_nci_N_cell_mem[VIED_NCI_N_CELL_TYPE_ID] = {
	VIED_NCI_N_SP_CTRL_MEM,
	VIED_NCI_N_SP_SERVER_MEM,
	VIED_NCI_N_VP_MEM,
	VIED_NCI_N_ACC_PSA_MEM,
	VIED_NCI_N_ACC_ISA_MEM,
	VIED_NCI_N_ACC_OSA_MEM
};

/*
 * Cell mem type by cell type ID and memory index
 */
const vied_nci_mem_type_ID_t
vied_nci_cell_mem_type[VIED_NCI_N_CELL_TYPE_ID][VIED_NCI_N_MEM_TYPE_ID] = {
	{
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_DMEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID
	},
	{
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_DMEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID
	},
	{
		VIED_NCI_GMEM_TYPE_ID,
		VIED_NCI_DMEM_TYPE_ID,
		VIED_NCI_VMEM_TYPE_ID,
		VIED_NCI_BAMEM_TYPE_ID,
		VIED_NCI_PMEM_TYPE_ID
	},
	{
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID
	},
	{
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID
	},
	{
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID
	},
	{
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID,
		VIED_NCI_N_MEM_TYPE_ID
	}
};

/*
 * Ext mem ID by memory index
 */
const vied_nci_mem_ID_t
vied_nci_ext_mem[VIED_NCI_N_MEM_TYPE_ID] = {
		VIED_NCI_VMEM4_ID, /* VIED_NCI_GMEM_TYPE_ID */
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
};

/*
 * Cell mem ID by cell ID and memory index
 */
const vied_nci_mem_ID_t
vied_nci_cell_mem[VIED_NCI_N_CELL_ID][VIED_NCI_N_MEM_TYPE_ID] = {
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_DMEM0_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	},
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_DMEM1_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	},
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_DMEM2_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	},
	{
		VIED_NCI_VMEM4_ID,
		VIED_NCI_DMEM4_ID,
		VIED_NCI_VMEM0_ID,
		VIED_NCI_BAMEM0_ID,
		VIED_NCI_PMEM0_ID
	},
	{
		VIED_NCI_VMEM4_ID,
		VIED_NCI_DMEM5_ID,
		VIED_NCI_VMEM1_ID,
		VIED_NCI_BAMEM1_ID,
		VIED_NCI_PMEM1_ID
	},
	{
		VIED_NCI_VMEM4_ID,
		VIED_NCI_DMEM6_ID,
		VIED_NCI_VMEM2_ID,
		VIED_NCI_BAMEM2_ID,
		VIED_NCI_PMEM2_ID
	},
	{
		VIED_NCI_VMEM4_ID,
		VIED_NCI_DMEM7_ID,
		VIED_NCI_VMEM3_ID,
		VIED_NCI_BAMEM3_ID,
		VIED_NCI_PMEM3_ID
	},
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	},
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	},
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	},
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	},
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	},
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	},
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	},
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	},
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	},
	{
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID,
		VIED_NCI_N_MEM_ID
	}
};

/*
 * Memory sizes by mem ID
 */
const uint16_t vied_nci_mem_size[VIED_NCI_N_MEM_ID] = {
	VIED_NCI_VMEM0_MAX_SIZE,
	VIED_NCI_VMEM1_MAX_SIZE,
	VIED_NCI_VMEM2_MAX_SIZE,
	VIED_NCI_VMEM3_MAX_SIZE,
	VIED_NCI_VMEM4_MAX_SIZE,
	VIED_NCI_BAMEM0_MAX_SIZE,
	VIED_NCI_BAMEM1_MAX_SIZE,
	VIED_NCI_BAMEM2_MAX_SIZE,
	VIED_NCI_BAMEM3_MAX_SIZE,
	VIED_NCI_DMEM0_MAX_SIZE,
	VIED_NCI_DMEM1_MAX_SIZE,
	VIED_NCI_DMEM2_MAX_SIZE,
	VIED_NCI_DMEM3_MAX_SIZE,
	VIED_NCI_DMEM4_MAX_SIZE,
	VIED_NCI_DMEM5_MAX_SIZE,
	VIED_NCI_DMEM6_MAX_SIZE,
	VIED_NCI_DMEM7_MAX_SIZE,
	VIED_NCI_PMEM0_MAX_SIZE,
	VIED_NCI_PMEM1_MAX_SIZE,
	VIED_NCI_PMEM2_MAX_SIZE,
	VIED_NCI_PMEM3_MAX_SIZE
};

/*
 * Memory word sizes by mem type ID
 */
const uint16_t vied_nci_mem_word_size[VIED_NCI_N_DATA_MEM_TYPE_ID] = {
	VIED_NCI_GMEM_WORD_SIZE,
	VIED_NCI_DMEM_WORD_SIZE,
	VIED_NCI_VMEM_WORD_SIZE,
	VIED_NCI_BAMEM_WORD_SIZE
};

/*
 * Number of channels by device ID
 */
const uint16_t vied_nci_dev_chn_size[VIED_NCI_N_DEV_CHN_ID] = {
	VIED_NCI_DEV_CHN_DMA_EXT0_MAX_SIZE,
	VIED_NCI_DEV_CHN_GDC_MAX_SIZE,
	VIED_NCI_DEV_CHN_DMA_EXT1_READ_MAX_SIZE,
	VIED_NCI_DEV_CHN_DMA_EXT1_WRITE_MAX_SIZE,
	VIED_NCI_DEV_CHN_DMA_INTERNAL_MAX_SIZE,
	VIED_NCI_DEV_CHN_DMA_IPFD_MAX_SIZE,
	VIED_NCI_DEV_CHN_DMA_ISA_MAX_SIZE,
	VIED_NCI_DEV_CHN_DMA_FW_MAX_SIZE
};
