/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */

#ifndef _SUNXI_CRASHDUMP_H
#define _SUNXI_CRASHDUMP_H

extern int sunxi_crash_dump2pc_init(void);
extern void sunxi_crash_dump2pc_exit(void);
extern void sunxi_kernel_panic_printf(const char *str, ...);

#ifdef CONFIG_AW_CRASHDUMP_KEY

extern int sunxi_crashdump_key_register(void);
extern void sunxi_crashdump_key_unregister(void);

#else
static inline int sunxi_crashdump_key_register(void)
{
	return 0;
}

static inline void sunxi_crashdump_key_unregister(void)
{

}

#endif

#endif	/*_SUNXI_CRASHDUMP_H*/
