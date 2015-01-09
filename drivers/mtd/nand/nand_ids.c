/*
 *  drivers/mtd/nandids.c
 *
 *  Copyright (C) 2002 Thomas Gleixner (tglx@linutronix.de)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <common.h>
#include <linux/mtd/nand.h>
/*
*	Chip ID list
*
*	Name. ID code, pagesize, chipsize in MegaByte, eraseblock size,
*	options
*
*	Pagesize; 0, 256, 512
*	0	get this information from the extended chip ID
+	256	256 Byte page size
*	512	512 Byte page size
*/	
struct nand_flash_dev nand_flash_ids[] = {

	{"TC58DVG02D5TA00", NAND_MFR_TOSHIBA, 0xD1, 0x800, 0x80, 0x20000, 64, NAND_NO_AUTOINCR | NAND_NO_READRDY | NAND_NO_SUBPAGE_WRITE},
	{"TC58NVG1S3ETA00", NAND_MFR_TOSHIBA, 0xDA, 0x800, 0x100, 0x20000, 64, NAND_NO_AUTOINCR | NAND_NO_READRDY | NAND_NO_SUBPAGE_WRITE},
	{"K9F1G08U0C", NAND_MFR_SAMSUNG, 0xf1, 0x800, 0x80, 0x20000, 64, NAND_NO_AUTOINCR | NAND_NO_READRDY | NAND_NO_SUBPAGE_WRITE},
	{NULL,}
};

/*
*	Manufacturer ID list
*/
struct nand_manufacturers nand_manuf_ids[] = {
	{NAND_MFR_TOSHIBA, "Toshiba"},
	{NAND_MFR_SAMSUNG, "Samsung"},
	{NAND_MFR_FUJITSU, "Fujitsu"},
	{NAND_MFR_NATIONAL, "National"},
	{NAND_MFR_RENESAS, "Renesas"},
	{NAND_MFR_STMICRO, "ST Micro"},
	{NAND_MFR_HYNIX, "Hynix"},
	{NAND_MFR_MICRON, "Micron"},
	{NAND_MFR_AMD, "AMD"},
	{0x0, "Unknown"}
};
