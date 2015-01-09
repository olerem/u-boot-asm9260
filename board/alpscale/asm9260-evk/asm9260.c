/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * David Mueller, ELSOFT AG, <d.mueller@elsoft.ch>
 *
 * (C) Copyright 2005
 * JinHua Luo, GuangDong Linux Center, <luo.jinhua@gd-linux.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>

#if defined(CONFIG_CMD_NAND)
#include <linux/mtd/nand.h>
#endif

DECLARE_GLOBAL_DATA_PTR;


static inline void delay (unsigned long loops)
{
	__asm__ volatile ("1:\n"
			  "subs %0, %1, #1\n"
			  "bne 1b":"=r" (loops):"0" (loops));
}

/*
 * Miscellaneous platform dependent initialisations
 */

int board_init (void)
{
	/* arch number of S3C2440-Board */
	gd->bd->bi_arch_number = MACH_TYPE_ASM9260;

	/* adress of boot parameters */
	gd->bd->bi_boot_params = BOOT_PARAMETER_ADDR;

	//icache_enable();
	//dcache_enable();

	return 0;
}


#if 0
void board_video_init(GraphicDevice *pGD) 
{ 

} 
#endif

int dram_init (void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;

	return 0;
}

/*
 * Check Board Identity
 */
int checkboard(void)
{
	char *s = getenv("serial#");

	puts("Board: ASM9260-EVK");

	if (s != NULL) {
		puts(", serial# ");
		puts(s);
	}
	putc('\n');

	return (0);
}

#ifdef CONFIG_CMD_NET
extern int mac9260_eth_initialize(bd_t *bis);

int board_eth_init(bd_t *bis)
{
	int rc = 0;

#ifdef	CONFIG_MAC9260
	rc = mac9260_eth_initialize(bis);
#endif

	return rc;
}
#endif
