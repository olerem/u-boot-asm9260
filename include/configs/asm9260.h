/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 * Gary Jennejohn <garyj@denx.de>
 * David Mueller <d.mueller@elsoft.ch>
 *
 * Modified for the friendly-arm SBC-2410X by
 * (C) Copyright 2005
 * JinHua Luo, GuangDong Linux Center, <luo.jinhua@gd-linux.com>
 *
 * Configuation settings for the friendly-arm SBC-2410X board.
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

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * High Level Configuration Options
 * (easy to change)
 */
#define CONFIG_ARM926EJS	1	/* This is an arm926ejs CPU core  */
#define MACH_TYPE_ASM9260	15000
#define BOOT_PARAMETER_ADDR	0x20000100

#undef	CONFIG_USE_IRQ
#undef	CONFIG_SKIP_LOWLEVEL_INIT
#define	CONFIG_SKIP_RELOCATE_UBOOT

/* addr of environment */
#define CONFIG_ENV_OFFSET 		(0x20000 + 0x80000) /* sramloader + u-boot */
#define CONFIG_ENV_SIZE			0x20000	/* Total Size of Environment Sector */

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + 1024*1024)
#define CONFIG_SYS_GBL_DATA_SIZE	128	/* size in bytes reserved for initial data */

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(256*1024)	/* regular stack */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(4*1024)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	(4*1024)	/* FIQ stack */
#endif

#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs  */
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG      		1       /* Required for ramdisk support */

/*UART configuration*/
#define	CONFIG_ASM9260_UART
/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_BAUDRATE	115200
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }

#define CONFIG_BOOTDELAY	1
#define CONFIG_BOOTCOMMAND	"bootm"

#define CONFIG_BOOTARGS		"console=ttyS4,115200n8 root=/dev/mtdblock2 init=/linuxrc " \
				"mtdparts=NAND:1M@0(bootloader),5M@1M(kernel),24M@6M(rootfs)ro,50M@30M(yaffs2),-@80M(ubifs)"

/* rootfs is ubifs */
#if 0
#define CONFIG_BOOTARGS		"console=ttyS4,115200n8 ubi.mtd=2 root=ubi0:ubifs rootfstype=ubifs init=/linuxrc " \
					"mtdparts=NAND:1M@0(bootloader),5M@1M(kernel),-@6M(rootfs)"
#endif

/* mtd spi: boot linux from spi flash */
//#define CONFIG_BOOTARGS		"console=ttyS4,115200n8 root=/dev/mtdblock2 init=/linuxrc mtdparts=spi2.0:320K@0(bootloader)ro,2752K@320K(kernel)ro,8M@3M(rootfs),-@11M(ubifs)"

#define CONFIG_CMDLINE_EDITING
#define CONFIG_AUTO_COMPLETE

/*
 * Miscellaneous configurable options
 */
#define	CONFIG_SYS_LONGHELP				/* undef to save memory		*/
#define	CONFIG_SYS_PROMPT		"[u-boot@ASM9260]# "	/* Monitor Command Prompt	*/
#define	CONFIG_SYS_CBSIZE		256		/* Console I/O Buffer Size	*/
/* Print Buffer Size */
#define	CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16) /* Print Buffer Size */
#define	CONFIG_SYS_MAXARGS		16		/* max number of command args	*/
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE	/* Boot Argument Buffer Size	*/

#define CONFIG_SYS_MEMTEST_START	0x20d00000	/* memtest works on	*/
#define CONFIG_SYS_MEMTEST_END		0x22000000	/* 19 MB in DRAM	*/

#define	CONFIG_SYS_LOAD_ADDR		0x20800000	/* default load address	*/

#define	CONFIG_SYS_HZ			1000

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS		1	   /* we have 1 bank of DRAM */
#define PHYS_SDRAM_1			0x20000000 /* SDRAM Bank #1 */
#define PHYS_SDRAM_1_SIZE		0x02000000 /* 64 MB */

/*
 * BOOTP options
 */
#define CONFIG_BOOTP_BOOTFILESIZE	1
#define CONFIG_BOOTP_BOOTPATH		1
#define CONFIG_BOOTP_GATEWAY		1
#define CONFIG_BOOTP_HOSTNAME		1

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#define	CONFIG_SYS_NO_FLASH
#define CONFIG_ENV_IS_IN_NAND	1
#define CONFIG_CMD_NAND

/*-----------------------------------------------------------------------
 * Command
 */
#define CONFIG_CMD_BOOTD	/* bootd			*/
#define CONFIG_CMD_EDITENV	/* editenv			*/
#define CONFIG_CMD_LOADB	/* loadb			*/
#define CONFIG_CMD_MEMORY	/* md mm nm mw cp cmp crc base loop mtest */
#define CONFIG_CMD_MISC		/* Misc functions like sleep etc*/
#define CONFIG_CMD_NET		/* bootp, tftpboot, rarpboot	*/
#define CONFIG_CMD_NFS		/* NFS support			*/
#define CONFIG_CMD_RUN		/* run command in env variable	*/
#define CONFIG_CMD_SAVEENV	/* saveenv			*/
#define CONFIG_CMD_SOURCE	/* "source" command support	*/
#define	CONFIG_CMD_PING		1
#define CONFIG_CMD_DHCP		1

#define CONFIG_CMD_BDI
#define CONFIG_CMD_CONSOLE	/* coninfo			*/

/* 下列命令不能加入，否则会出错 */
//#define CONFIG_CMD_IMI		/* iminfo			*/
//#define CONFIG_CMD_ITEST		/* Integer (and string) test	*/
//#define CONFIG_CMD_ECHO		/* echo arguments		*/

/*-----------------------------------------------------------------------
 * ETH 
 */
#define CONFIG_MAC9260
#define CONFIG_NETMASK		255.255.255.0	/* talk on MY local net */
#define CONFIG_IPADDR		192.168.100.100	/* static IP I currently own */
#define CONFIG_GATEWAYIP	192.168.100.1	/* current Gateway IP of my dev pc */
#define CONFIG_SERVERIP		192.168.100.149/* current IP of my dev pc */
#define CONFIG_ETHADDR 		00:e0:a3:a4:98:67
#define CONFIG_BOOTFILE		"uImage"	/* file to load */
#define CONFIG_NET_MULTI

/*-----------------------------------------------------------------------
 * NAND flash settings
 */
#define CONFIG_NAND_ASM9260
#define CONFIG_SYS_NAND_BASE 0x80600098 
#define CONFIG_SYS_MAX_NAND_DEVICE	1	/* Max number of NAND devices		*/
#define	CONFIG_MTD_NAND_VERIFY_WRITE
#define	CONFIG_MTD_NAND_ASM9260_HWECC
//#define	CONFIG_MTD_NAND_ASM9260_DEBUG
//#define	CONFIG_MTD_NAND_ASAP9260_DMA
/* nand_util.c:45 */
#define	CONFIG_SYS_64BIT_VSPRINTF

/*-----------------------------------------------------------------------
 * USB
 */
#define	CONFIG_USB_ASM9260
#define	CONFIG_MUSB_HCD
#define	CONFIG_MUSB_UDC

#ifdef CONFIG_USB_ASM9260
#define CONFIG_CMD_USB
#ifdef CONFIG_MUSB_HCD
#define CONFIG_USB_STORAGE
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION
#endif
#endif

#if 0
#define	CONFIG_CMD_UBI
#define	CONFIG_CMD_UBIFS
#define	CONFIG_CMD_MTDPARTS
#define	CONFIG_MTD_DEVICE
#define	CONFIG_MTD_PARTITIONS
#define	CONFIG_LZO
#define	CONFIG_RBTREE

#define MTDIDS_DEFAULT "nand0=NAND"
#define MTDPARTS_DEFAULT "mtdparts=NAND:5M@2M(kernel),60M@14M(rootfs),-@74M(NULL)"
#endif

#endif


