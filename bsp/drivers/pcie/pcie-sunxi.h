/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner PCIe host controller driver
 *
 * Copyright (C) 2022 allwinner Co., Ltd.
 *
 * Author: songjundong <songjundong@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _PCIE_SUNXI_H
#define _PCIE_SUNXI_H

#include <sunxi-gpio.h>
#include <linux/bits.h>
#include <asm/io.h>
#include <linux/irqreturn.h>
#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/msi.h>
#include <linux/pci.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

#include "pcie-sunxi-dma.h"

#define PCIE_PORT_LINK_CONTROL			0x710
#define   PORT_LINK_MODE_MASK			(0x3f << 16)
#define   PORT_LINK_MODE_1_LANES		(0x1 << 16)
#define   PORT_LINK_MODE_2_LANES		(0x3 << 16)
#define   PORT_LINK_MODE_4_LANES		(0x7 << 16)
#define   PORT_LINK_LPBK_ENABLE		        (0x1 << 2)

#define PCIE_LINK_WIDTH_SPEED_CONTROL		0x80C
#define   PORT_LOGIC_SPEED_CHANGE		(0x1 << 17)
#define   PORT_LOGIC_LINK_WIDTH_MASK		(0x1ff << 8)
#define   PORT_LOGIC_LINK_WIDTH_1_LANES		(0x1 << 8)
#define   PORT_LOGIC_LINK_WIDTH_2_LANES		(0x2 << 8)
#define   PORT_LOGIC_LINK_WIDTH_4_LANES		(0x4 << 8)

#define PCIE_ATU_VIEWPORT			0x900
#define   PCIE_ATU_REGION_INBOUND		(0x1 << 31)
#define   PCIE_ATU_REGION_OUTBOUND		(0x0 << 31)
#define   PCIE_ATU_REGION_INDEX2		(0x2 << 0)
#define   PCIE_ATU_REGION_INDEX1		(0x1 << 0)
#define   PCIE_ATU_REGION_INDEX0		(0x0 << 0)

#define PCIE_ATU_INDEX0				0x0
#define PCIE_ATU_INDEX1				0x1
#define PCIE_ATU_INDEX2				0x2
#define PCIE_ATU_INDEX3				0x3
#define PCIE_ATU_INDEX4				0x4
#define PCIE_ATU_INDEX5				0x5
#define PCIE_ATU_INDEX6				0x6
#define PCIE_ATU_INDEX7				0x7

#define PCIE_EP_REBAR_SIZE_32M			0x200

#define PCIE_ATU_CR1_OUTBOUND(reg)		(0x300000 + ((reg) * 0x200))
#define   PCIE_ATU_TYPE_MEM			(0x0 << 0)
#define   PCIE_ATU_TYPE_IO			(0x2 << 0)
#define   PCIE_ATU_TYPE_CFG0			(0x4 << 0)
#define   PCIE_ATU_TYPE_CFG1			(0x5 << 0)
#define PCIE_ATU_CR2_OUTBOUND(reg)		(0x300004 + ((reg) * 0x200))
#define   PCIE_ATU_DMA_BYPASS			BIT(27)
#define PCIE_ATU_BAR_MODE_ENABLE		BIT(30)
#define PCIE_ATU_ENABLE				BIT(31)

#define PCIE_ATU_LOWER_BASE_OUTBOUND(reg)	(0x300008 + ((reg) * 0x200))
#define PCIE_ATU_UPPER_BASE_OUTBOUND(reg)	(0x30000c + ((reg) * 0x200))
#define PCIE_ATU_LIMIT_OUTBOUND(reg)		(0x300010 + ((reg) * 0x200))
#define PCIE_ATU_LOWER_TARGET_OUTBOUND(reg)	(0x300014 + ((reg) * 0x200))
#define PCIE_ATU_UPPER_TARGET_OUTBOUND(reg)	(0x300018 + ((reg) * 0x200))

#define PCIE_ATU_CR1_INBOUND(reg)		(0x300100 + ((reg) * 0x200))
#define   PCIE_ATU_TYPE_MEM			(0x0 << 0)
#define   PCIE_ATU_TYPE_IO			(0x2 << 0)
#define   PCIE_ATU_TYPE_CFG0			(0x4 << 0)
#define   PCIE_ATU_TYPE_CFG1			(0x5 << 0)
#define   PCIE_ATU_FUNC_NUM(pf)			((pf) << 20)
#define PCIE_ATU_CR2_INBOUND(reg)		(0x300104 + ((reg) * 0x200))
#define   PCIE_ATU_MATCH_MODE			BIT(30)
#define   PCIE_ATU_FUNC_NUM_MATCH_EN		BIT(19)
#define PCIE_ATU_FUNC_NUM(pf)			((pf) << 20)

#define PCIE_ATU_LOWER_BASE_INBOUND(reg)	(0x300108 + ((reg) * 0x200))
#define PCIE_ATU_UPPER_BASE_INBOUND(reg)	(0x30010c + ((reg) * 0x200))
#define PCIE_ATU_LIMIT_INBOUND(reg)		(0x300110 + ((reg) * 0x200))
#define PCIE_ATU_LOWER_TARGET_INBOUND(reg)	(0x300114 + ((reg) * 0x200))
#define PCIE_ATU_UPPER_TARGET_INBOUND(reg)	(0x300118 + ((reg) * 0x200))

#define PCIE_ATU_BUS(x)				(((x) & 0xff) << 24)
#define PCIE_ATU_DEV(x)				(((x) & 0x1f) << 19)
#define PCIE_ATU_FUNC(x)			(((x) & 0x7) << 16)

#define PCIE_MISC_CONTROL_1_CFG			0x8bc
#define PCIE_TYPE1_CLASS_CODE_REV_ID_REG	0x08

#define PCIE_ADDRESS_ALIGNING			(~0x3)
#define PCIE_HIGH_16				16
#define PCIE_BAR_NUM				6
#define PCIE_MEM_FLAGS				0x4
#define PCIE_IO_FLAGS				0x1
#define PCIE_BAR_REG				0x4
#define PCIE_HIGH16_MASK			0xffff0000
#define PCIE_LOW16_MASK				0x0000ffff
#define PCIE_INTERRUPT_LINE_MASK		0xffff00ff
#define PCIE_INTERRUPT_LINE_ENABLE		0x00000100
#define	PCIE_PRIMARY_BUS_MASK			0xff000000
#define PCIE_PRIMARY_BUS_ENABLE			0x00010100
#define PCIE_MEMORY_MASK			0xfff00000

#define PCIE_CPU_BASE				0x20000000

#define PCIE_COMBO_PHY_BGR		0x04
#define   PHY_ACLK_EN			BIT(17)
#define   PHY_HCLK_EN			BIT(16)
#define   PHY_TERSTN			BIT(1)
#define   PHY_PW_UP_RSTN		BIT(0)
#define PCIE_COMBO_PHY_CTL		0x10
#define   PHY_USE_SEL			BIT(31)	/* 0:PCIE; 1:USB3 */
#define	  PHY_CLK_SEL			BIT(30) /* 0:internal clk; 1:exteral clk */
#define   PHY_BIST_EN			BIT(16)
#define   PHY_PIPE_SW			BIT(9)
#define   PHY_PIPE_SEL			BIT(8)  /* 0:PIPE resetn ctrl by PCIE ctrl; 1:PIPE resetn ctrl by  */
#define   PHY_PIPE_CLK_INVERT		BIT(4)
#define   PHY_FPGA_SYS_RSTN		BIT(1)  /* for PFGA  */
#define   PHY_RSTN			BIT(0)

#define NEXT_CAP_PTR_MASK		0xff00
#define CAP_ID_MASK			0x00ff

/*
 * Maximum number of MSI IRQs can be 256 per controller. But keep
 * it 32 as of now. Probably we will never need more than 32. If needed,
 * then increment it in multiple of 32.
 */
#define INT_PCI_MSI_NR			32
#define MAX_MSI_IRQS			256
#define MAX_MSI_IRQS_PER_CTRL		32
#define MAX_MSI_CTRLS			(MAX_MSI_IRQS / MAX_MSI_IRQS_PER_CTRL)
/* #define MAX_MSI_IRQS			32 */
/* #define MAX_MSI_CTRLS		(MAX_MSI_IRQS / 32) */
#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80C
#define PORT_LOGIC_SPEED_CHANGE		(0x1 << 17)
#define LINK_CONTROL2_LINK_STATUS2	0xa0
/* Parameters for the waiting for link up routine */
#define LINK_WAIT_MAX_RETRIE		20
#define LINK_WAIT_USLEEP_MIN		90000
#define LINK_WAIT_USLEEP_MAX		100000
#define SPEED_CHANGE_USLEEP_MIN		100
#define SPEED_CHANGE_USLEEP_MAX		1000

#define PCIE_MSI_ADDR_LO		0x820
#define PCIE_MSI_ADDR_HI		0x824
#define PCIE_MSI_INTR_ENABLE(reg)	(0x828 + ((reg) * 0x0c))
/* #define PCIE_MSI_INTR_MASK(reg)	(0x82C + ((reg) * 0x0c)) */
/* #define PCIE_MSI_INTR_STATUS(reg)	(0x830 + ((reg) * 0x0c)) */
/* #define PCIE_MSI_INTR_ENABLE		0x828 */
#define PCIE_MSI_INTR_MASK		0x82C
#define PCIE_MSI_INTR_STATUS		0x830

#define PCIE_CTRL_MGMT_BASE		0x900000

#define USER_DEFINED_REGISTER_LIST	0x400000
#define PCIE_VER			0x00
#define PCIE_ADDR_PAGE_CFG		0x04
#define PCIE_AWMISC_CTRL		0x200
#define PCIE_ARMISC_CTRL		0x220
#define PCIE_LTSSM_CTRL			0xc00
#define   PCIE_LINK_TRAINING		BIT(0) /* 0:disable; 1:enable  */
#define   DEVICE_TYPE_MASK		GENMASK(7, 4)
#define   DEVICE_TYPE_RC		BIT(6)
#define PCIE_INT_ENABLE_CLR		0xE04  /* BIT(1):RDLH_LINK_MASK; BIT(0):SMLH_LINK_MASK  */
#define PCIE_LINK_STAT			0xE0C  /* BIT(1):RDLH_LINK;      BIT(0):SMLH_LINK  */
#define   RDLH_LINK_UP			(1<<1)
#define   SMLH_LINK_UP			(1<<0)

#define PCIE_PHY_CFG			0x800
#define   SYS_CLK			0
#define   PAD_CLK			1
#define PCIE_LINK_UP_MASK		(0x3<<16)

#define PCIE_RC_RP_ATS_BASE		0x400000

#define SUNXI_PCIE_BAR_CFG_CTRL_DISABLED		0x0
#define SUNXI_PCIE_BAR_CFG_CTRL_IO_32BITS		0x1
#define SUNXI_PCIE_BAR_CFG_CTRL_MEM_32BITS		0x4
#define SUNXI_PCIE_BAR_CFG_CTRL_PREFETCH_MEM_32BITS	0x5
#define SUNXI_PCIE_BAR_CFG_CTRL_MEM_64BITS		0x6
#define SUNXI_PCIE_BAR_CFG_CTRL_PREFETCH_MEM_64BITS	0x7

#define SUNXI_PCIE_EP_MSI_CTRL_REG			0x90
#define   SUNXI_PCIE_EP_MSI_CTRL_MMC_OFFSET		17
#define   SUNXI_PCIE_EP_MSI_CTRL_MMC_MASK		GENMASK(19, 17)
#define   SUNXI_PCIE_EP_MSI_CTRL_MME_OFFSET		20
#define   SUNXI_PCIE_EP_MSI_CTRL_MME_MASK		GENMASK(22, 20)
#define   SUNXI_PCIE_EP_MSI_CTRL_ME			BIT(16)
#define   SUNXI_PCIE_EP_MSI_CTRL_MASK_MSI_CAP		BIT(24)
#define   SUNXI_PCIE_EP_DUMMY_IRQ_ADDR			0x1

#define PCIE_PHY_FUNC_CFG		(PCIE_CTRL_MGMT_BASE + 0x2c0)
#define PCIE_RC_BAR_CONF		(PCIE_CTRL_MGMT_BASE + 0x300)


enum gpio_type {
	PCIE_REST = 0,
	PCIE_POWER,
	PCIE_REG,
	MAX_GPIO_NUM,
};

enum power_type {
	PCIE_VDD = 0,
	PCIE_VCC,
	PCIE_VCC_SLOT,
	MAX_POWER_NUM,
};

struct power {
	struct regulator *pmic;
	int power_vol;
	char power_str[32];
};

enum sunxi_pcie_device_mode {
	SUNXI_PCIE_EP_TYPE,
	SUNXI_PCIE_RC_TYPE,
};

struct sunxi_pcie_of_data {
	enum sunxi_pcie_device_mode	mode;
	u32				func_offset;
	const struct sunxi_pcie_ep_ops	*ops;
	bool			cpu_pcie_addr_quirk;
};

struct sunxi_pcie_ep_func {
	struct list_head	list;
	u8			func_no;
	u8			msi_cap;
	u8			msix_cap;
};

struct sunxi_pcie_ep {
	struct pci_epc		*epc;
	struct list_head	func_list;
	const struct sunxi_pcie_ep_ops *ops;
	phys_addr_t		phys_base;
	size_t			addr_size;
	size_t			page_size;
	u8			bar_to_atu[PCI_STD_NUM_BARS];
	phys_addr_t		*outbound_addr;
	u32			num_ib_windows;
	u32			num_ob_windows;
	unsigned long		*ib_window_map;
	unsigned long		*ob_window_map;
	void __iomem		*msi_mem;
	phys_addr_t		msi_mem_phys;
	struct pci_epf_bar	*epf_bar[PCI_STD_NUM_BARS];
};

struct sunxi_pcie_ep_ops {
	void	(*ep_init)(struct sunxi_pcie_ep *ep);
	int	(*raise_irq)(struct sunxi_pcie_ep *ep, u8 func_no,
			     enum pci_epc_irq_type type, u16 interrupt_num);
	const struct pci_epc_features *(*get_features)(struct sunxi_pcie_ep *ep);
	unsigned int (*func_conf_select)(struct sunxi_pcie_ep *ep, u8 func_no);
};

struct pcie_port {
	struct device		*dev;
	u8			root_bus_nr;
	void __iomem		*dbi_base;
	void __iomem		*phy_base;
	u64			cfg0_base;
	void __iomem		*va_cfg0_base;
	u32			cfg0_size;
	resource_size_t		io_base;
	phys_addr_t		io_bus_addr;
	u32			io_size;
	struct dma_trx_obj	*dma_obj;
	int			irq;
	u32			lanes;
	u32			num_viewport;
	struct pcie_host_ops	*ops;
	int			msi_irq;
	struct irq_domain	*irq_domain;
	struct irq_domain	*msi_domain;
	unsigned long		msi_pages;
	struct page		*msi_page;
	u32			num_vectors;
	u32			irq_status[MAX_MSI_CTRLS];
	struct pci_host_bridge  *bridge;
	raw_spinlock_t		lock;
	unsigned long		msi_map[BITS_TO_LONGS(INT_PCI_MSI_NR)];
	int			msi_ext;
	bool			cpu_pcie_addr_quirk;
};

struct sunxi_pcie {
	struct device		dev;
	void __iomem		*dbi_base;
	void __iomem		*app_base;
	void __iomem		*phy_base;
	void __iomem		*clk_base;
	int			link_irq;
	int			msi_irq;
	int			link_gen;
	struct pcie_port	pp;
	struct sunxi_pcie_ep	ep;
	struct clk		*pcie_pclk;
	struct clk		*pcie_ref;
	struct clk		*pcie_hosc;
	struct clk		*pcie_per;
	struct clk		*pcie_aux;
	struct reset_control    *pcie_rst;
	struct phy		*phy;
	struct regmap		*phy_i2c;
	int			iodvdd;
	const struct sunxi_pcie_of_data *drvdata;
	u32			io_base;
	u32			interrupt_control;
	u32			msi_enable;
	u32			prsnt_gpio;
	struct regulator	*pcie3v3;
	bool clk_freq_24M;
	bool clk_freq_100M;
};

#define to_sunxi_pcie_from_pp(x)	\
		container_of((x), struct sunxi_pcie, pp)

#define to_sunxi_pcie_from_ep(endpoint)   \
		container_of((endpoint), struct sunxi_pcie, ep)

struct pcie_host_ops {
	void (*readl_rc)(struct pcie_port *pp, void __iomem *dbi_base, u32 *val);
	void (*writel_rc)(struct pcie_port *pp,	u32 val, void __iomem *dbi_base);
	int  (*rd_own_conf)(struct pcie_port *pp, int where, int size, u32 *val);
	int  (*wr_own_conf)(struct pcie_port *pp, int where, int size, u32 val);
	int  (*link_up)(struct pcie_port *pp);
	void (*host_init)(struct pcie_port *pp);
	void (*scan_bus)(struct pcie_port *pp);
};

struct pci_sys_data {
#ifdef CONFIG_PCI_DOMAINS
	int		domain;
#endif
	struct list_head node;
	int		busnr;		/* primary bus number			*/
	u64		mem_offset;	/* bus->cpu memory mapping offset	*/
	unsigned long	io_offset;	/* bus->cpu IO mapping offset		*/
	struct pci_bus	*bus;		/* PCI bus				*/
	struct list_head resources;	/* root bus resources (apertures)       */
	struct resource io_res;
	char		io_res_name[12];
					/* IRQ mapping				*/
	int		(*map_irq)(const struct pci_dev *, u8, u8);
					/* Resource alignement requirements	*/
	resource_size_t (*align_resource)(struct pci_dev *dev,
					  const struct resource *res,
					  resource_size_t start,
					  resource_size_t size,
					  resource_size_t align);
	void		(*add_bus)(struct pci_bus *bus);
	void		(*remove_bus)(struct pci_bus *bus);
	void		*private_data;	/* platform controller private data	*/
};

u32 sunxi_pcie_read_dbi(struct sunxi_pcie *pci, u32 reg, size_t size);
void sunxi_pcie_write_dbi(struct sunxi_pcie *pci, u32 reg, size_t size, u32 val);

irqreturn_t sunxi_handle_msi_irq(struct pcie_port *pp);
void sunxi_pcie_msi_init(struct pcie_port *pp);
int sunxi_pcie_link_up(struct pcie_port *pp);
void sunxi_pcie_setup_rc(struct pcie_port *pp);
int sunxi_pcie_host_init(struct pcie_port *pp);
void sunxi_pcie_set_rate(struct pcie_port *pp);
void sunxi_pcie_write_dbi(struct sunxi_pcie *pci, u32 reg, size_t size, u32 val);
u32 sunxi_pcie_read_dbi(struct sunxi_pcie *pci, u32 reg, size_t size);
int sunxi_pcie_init_dma_trx(struct pcie_port *pp);
void sunxi_pcie_ltssm_enable(struct sunxi_pcie *pcie, bool enable);
int sunxi_pcie_establish_link(struct pcie_port *pp);
int sunxi_pcie_wait_for_link(struct pcie_port *pp);
int sunxi_pcie_cfg_write(void __iomem *addr, int size, u32 val);
int sunxi_pcie_cfg_read(void __iomem *addr, int size, u32 *val);
int sunxi_pcie_speed_chang(struct sunxi_pcie *pci, int gen);
int sunxi_pcie_wr_own_conf(struct pcie_port *pp, int where, int size, u32 val);

#if IS_ENABLED(CONFIG_AW_PCIE_RC)
int sunxi_plat_add_pcie_port(struct pcie_port *pp, struct platform_device *pdev);
#else
static inline int sunxi_plat_add_pcie_port(struct pcie_port *pp,
					struct platform_device *pdev)
{
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_AW_PCIE_EP)
int sunxi_plat_ep_init(struct sunxi_pcie_ep *ep);
#else
static inline int sunxi_plat_ep_init(struct sunxi_pcie_ep *ep)
{
	return 0;
}
#endif

void sunxi_pcie_writel(u32 val, struct sunxi_pcie *pcie, u32 offset);
u32 sunxi_pcie_readl(struct sunxi_pcie *pcie, u32 offset);
void sunxi_pcie_writel_dbi(struct sunxi_pcie *pci, u32 reg, u32 val);
u32 sunxi_pcie_readl_dbi(struct sunxi_pcie *pci, u32 reg);
void sunxi_pcie_writew_dbi(struct sunxi_pcie *pci, u32 reg, u16 val);
u16 sunxi_pcie_readw_dbi(struct sunxi_pcie *pci, u32 reg);
void sunxi_pcie_writeb_dbi(struct sunxi_pcie *pci, u32 reg, u8 val);
u8 sunxi_pcie_readb_dbi(struct sunxi_pcie *pci, u32 reg);
void sunxi_pcie_dbi_ro_wr_en(struct sunxi_pcie *pci);
void sunxi_pcie_dbi_ro_wr_dis(struct sunxi_pcie *pci);

#endif /* _PCIE_SUNXI_H */
