/*
 * (C) Copyright 2006 OpenMoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
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

#include <nand.h>
#include <malloc.h>
#include <asm/arch/hardware.h>
#include <asm/arch/pinctrl.h>
#include <asm/io.h>
#include <asm/errno.h>

// ================== Definitions ====================

// timing parameters
#define  TITC  0x0
#define  TWHR  0x6//0x6
#define  TRHW  0x6//0x6
#define  TADL  0x0
#define  TCCS  0x0

#define  TWH   0x8//0x8
#define  TWP   0x8//0x8

#define  TCAD  0x0

// cmd parameters
#define  RESET                        0xFF
#define  SYNC_RESET                   0xFC

#define  READ_ID                      0x90

#define  READ_STATUS                  0x70
#define  READ_STATUS_ENHANCE          0x78

#define  CHANGE_WRITE_COLUMN          0x85
#define  CHANGE_ROW_ADDRESS           0x85

#define  READ_PAGE_1                  0x00
#define  READ_PAGE_2                  0x30

#define  PROGRAM_PAGE_1               0x80
#define  PROGRAM_PAGE_2               0x10

#define  PROGRAM_PAGE1                0x80

#define  WRITE_PAGE                   0x10
#define  WRITE_PAGE_CACHE             0x15
#define  WRITE_MULTIPLANE             0x11

#define  ERASE_BLOCK_1                0x60
#define  ERASE_BLOCK_2                0xD0



// seq parameter
#define  SEQ1     0x21   // 6'b100001
#define  SEQ2     0x22   // 6'b100010
#define  SEQ4     0x24   // 6'b100100
#define  SEQ5     0x25   // 6'b100101
#define  SEQ6     0x26   // 6'b100110
#define  SEQ7     0x27   // 6'b100111
#define  SEQ9     0x29   // 6'b101001
#define  SEQ10    0x2A   // 6'b101010
#define  SEQ11    0x2B   // 6'b101011
#define  SEQ15    0x2F   // 6'b101111
#define  SEQ0     0x00   // 6'b000000
#define  SEQ3     0x03   // 6'b000011
#define  SEQ8     0x08   // 6'b001000
#define  SEQ12    0x0C   // 6'b001100
#define  SEQ13    0x0D   // 6'b001101
#define  SEQ14    0x0E   // 6'b001110
#define  SEQ16    0x30   // 6'b110000
#define  SEQ17    0x15   // 6'b010101
#define  SEQ18    0x32   // 6'h110010

// cmd register
#define  ADDR_SEL_0    0x0
#define  ADDR_SEL_1    0x1

#define  INPUT_SEL_BIU  0x0
#define  INPUT_SEL_DMA  0x1

// control register parameter
#define  DISABLE_STATUS    1
#define  EN_STATUS         0

#define  RNB_SEL           0
#define  NO_RNB_SEL        1

#define  BIG_BLOCK_EN      0
#define  SMALL_BLOCK_EN    1

#define  LOOKUP_EN         1
#define  LOOKUP_DIS        0

#define  WORK_MODE_ASYNC   0
#define  WORK_MODE_SYNC    1

#define  PROT_EN           1
#define  PROT_DIS          0

#define  IO_WIDTH_8        0
#define  IO_WIDTH_16       1

#define  DATA_SIZE_FULL_PAGE  0
#define  DATA_SIZE_CUSTOM     1

#define  PAGE_SIZE_256B        0x0
#define  PAGE_SIZE_512B        0x1
#define  PAGE_SIZE_1024B       0x2
#define  PAGE_SIZE_2048B       0x3
#define  PAGE_SIZE_4096B       0x4
#define  PAGE_SIZE_8192B       0x5
#define  PAGE_SIZE_16384B      0x6
#define  PAGE_SIZE_32768B      0x7
#define  PAGE_SIZE_0B          0x0

#define  BLOCK_SIZE_32P        0x0
#define  BLOCK_SIZE_64P        0x1
#define  BLOCK_SIZE_128P       0x2
#define  BLOCK_SIZE_256P       0x3

#define  ECC_DIS          0
#define  ECC_EN           1

#define  INT_DIS          0
#define  INT_EN           1

#define  SPARE_DIS        0
#define  SPARE_EN         1

#define  ADDR0_AUTO_INCR_DIS  0
#define  ADDR0_AUTO_INCR_EN   1

#define  ADDR1_AUTO_INCR_DIS  0
#define  ADDR1_AUTO_INCR_EN   1

#define  ADDR_CYCLE_0      0x0
#define  ADDR_CYCLE_1      0x1
#define  ADDR_CYCLE_2      0x2
#define  ADDR_CYCLE_3      0x3
#define  ADDR_CYCLE_4      0x4
#define  ADDR_CYCLE_5      0x5

//generic_seq_ctrl
#define  CMD0_EN      0x1
#define  CMD0_DIS     0x0

#define  ADDR0_EN     0x1
#define  ADDR0_DIS    0x0

#define  CMD1_EN      0x1
#define  CMD1_DIS     0x0

#define  ADDR1_EN     0x1
#define  ADDR1_DIS    0x0

#define  CMD2_EN      0x1
#define  CMD2_DIS     0x0

#define  CMD3_EN      0x1
#define  CMD3_DIS     0x0

#define  ADDR2_EN     0x1
#define  ADDR2_DIS    0x0 

#define  DEL_DIS_ALL  0x0
#define  DEL_EN_ALL   0x3
#define  DEL_EN_0     0x1
#define  DEL_EN_1     0x2

#define  DATA_EN      0x1
#define  DATA_DIS     0x0

#define  COL_ADDR_EN  0x1
#define  COL_ADDR_DIS 0x0

// int_mask register
#define  FIFO_ERROR_DIS  0
#define  FIFO_ERROR_EN   1

#define  MEM7_RDY_DIS    0
#define  MEM7_RDY_EN     1

#define  MEM6_RDY_DIS    0
#define  MEM6_RDY_EN     1

#define  MEM5_RDY_DIS    0
#define  MEM5_RDY_EN     1

#define  MEM4_RDY_DIS    0
#define  MEM4_RDY_EN     1

#define  MEM3_RDY_DIS    0
#define  MEM3_RDY_EN     1

#define  MEM2_RDY_DIS    0
#define  MEM2_RDY_EN     1

#define  MEM1_RDY_DIS    0
#define  MEM1_RDY_EN     1

#define  MEM0_RDY_DIS    0
#define  MEM0_RDY_EN     1

#define  ECC_TRSH_ERR_DIS  0
#define  ECC_TRSH_ERR_EN   1

#define  ECC_FATAL_ERR_DIS 0
#define  ECC_FATAL_ERR_EN  1

#define  CMD_END_INT_DIS   0
#define  CMD_END_INT_EN    1

#define  PROT_INT_DIS   0
#define  PROT_INT_EN    1

// dma ctrl register
#define  DMA_START_EN   0x1
#define  DMA_START_DIS  0x0

#define  DMA_DIR_WRITE  0x0
#define  DMA_DIR_READ   0x1

#define  DMA_MODE_SFR   0x0
#define  DMA_MODE_SG    0x1

#define  DMA_BURST_INCR4   0x0
#define  DMA_BURST_STREAM  0x1
#define  DMA_BURST_SINGLE  0x2
#define  DMA_BURST_INCR    0x3
#define  DMA_BURST_INCR8   0x4
#define  DMA_BURST_INCR16  0x5

//ecc ctrl register
#define  ECC_WORD_POS_SPARE  1
#define  ECC_WORD_POS_DATA   0

#define  ECC_THRESHOLD_0     0x0
#define  ECC_THRESHOLD_1     0x1
#define  ECC_THRESHOLD_2     0x2
#define  ECC_THRESHOLD_3     0x3
#define  ECC_THRESHOLD_4     0x4
#define  ECC_THRESHOLD_5     0x5
#define  ECC_THRESHOLD_6     0x6
#define  ECC_THRESHOLD_7     0x7
#define  ECC_THRESHOLD_8     0x8
#define  ECC_THRESHOLD_9     0x9
#define  ECC_THRESHOLD_10    0xA
#define  ECC_THRESHOLD_11    0xB
#define  ECC_THRESHOLD_12    0xC
#define  ECC_THRESHOLD_13    0xD
#define  ECC_THRESHOLD_14    0xE
#define  ECC_THRESHOLD_15    0xF

#define  ECC_CAP_2    0x0
#define  ECC_CAP_4    0x1
#define  ECC_CAP_6    0x2
#define  ECC_CAP_8    0x3
#define  ECC_CAP_10   0x4
#define  ECC_CAP_12   0x5
#define  ECC_CAP_14   0x6
#define  ECC_CAP_16   0x7

// boot parameter
#define  BOOT_REQ      0x1

struct asm9260_nand_regs{
	volatile uint32_t nand_command;          
	volatile uint32_t nand_control;          
	volatile uint32_t nand_status;           
	volatile uint32_t nand_int_mask;         
	volatile uint32_t nand_int_status;       
	volatile uint32_t nand_ecc_ctrl;         
	volatile uint32_t nand_ecc_offset;       
	volatile uint32_t nand_addr0_l;          
	volatile uint32_t nand_addr1_l;
	volatile uint32_t nand_addr0_h;          
	volatile uint32_t nand_addr1_h;          
	volatile uint32_t nand_rsvd0;
	volatile uint32_t nand_spare_size;    
	volatile uint32_t nand_rsvd1;
	volatile uint32_t nand_protect;
	volatile uint32_t nand_rsvd2;
	volatile uint32_t nand_lookup_en;        
	volatile uint32_t nand_lookup0;          
	volatile uint32_t nand_lookup1;          
	volatile uint32_t nand_lookup2;          
	volatile uint32_t nand_lookup3;          
	volatile uint32_t nand_lookup4;          
	volatile uint32_t nand_lookup5;          
	volatile uint32_t nand_lookup6;          
	volatile uint32_t nand_lookup7;          
	volatile uint32_t nand_dma_addr;         
	volatile uint32_t nand_dma_cnt;          
	volatile uint32_t nand_dma_ctrl;
	volatile uint32_t nand_rsvd3;
	volatile uint32_t nand_rsvd4;
	volatile uint32_t nand_rsvd5;
	volatile uint32_t nand_rsvd6;
	volatile uint32_t nand_mem_ctrl;         
	volatile uint32_t nand_data_size;        
	volatile uint32_t nand_read_status;      
	volatile uint32_t nand_time_seq_0;       
	volatile uint32_t nand_timings_asyn;     
	volatile uint32_t nand_timings_syn;      
	volatile uint32_t nand_fifo_data;        
	volatile uint32_t nand_time_mode;        
	volatile uint32_t nand_dma_addr_offset;  
	volatile uint32_t nand_rsvd7;
	volatile uint32_t nand_rsvd8;
	volatile uint32_t nand_rsvd9;
	volatile uint32_t nand_fifo_init;        
	volatile uint32_t nand_generic_seq_ctrl; 
	volatile uint32_t nand_err_cnt00;        
	volatile uint32_t nand_err_cnt01;        
	volatile uint32_t nand_err_cnt10;        
	volatile uint32_t nand_err_cnt11;        
	volatile uint32_t nand_time_seq_1;       
};

/**
 * struct ecc_info - ASAP1826T ECC INFO Structure
 * @ecc_cap:	The ECC module correction ability.
 * @ecc_threshold:		The acceptable errors level
 * @ecc_bytes_per_sector:		ECC bytes per sector
 */
struct ecc_info {
	int ecc_cap;
	int ecc_threshold;
	int ecc_bytes_per_sector;
};

/*
*	ECC info list
*
*	ecc_cap, ecc_threshold, ecc bytes per sector
*/
struct ecc_info ecc_info_table[8] = {
	{ECC_CAP_2, ECC_THRESHOLD_2, 4},
	{ECC_CAP_4, ECC_THRESHOLD_4, 7},
	{ECC_CAP_6, ECC_THRESHOLD_6, 10},
	{ECC_CAP_8, ECC_THRESHOLD_8, 13},
	{ECC_CAP_10, ECC_THRESHOLD_10, 17},
	{ECC_CAP_12, ECC_THRESHOLD_12, 20},
	{ECC_CAP_14, ECC_THRESHOLD_14, 23},
	{ECC_CAP_16, ECC_THRESHOLD_15, 26},
};

#ifdef CONFIG_MTD_NAND_ASM9260_DEBUG
#define DBG(x...) printk("ASM9260_NAND_DBG: " x)
#else
#define DBG(x...)
#endif

struct asm9260_nand_regs *nand_regs = (struct asm9260_nand_regs *)(NAND_BASE_ADDR);
static uint32_t page_shift, block_shift, addr_cycles, row_cycles, col_cycles;
static uint32_t asm9260_nand_spare_data_size;
static uint32_t asm9260_nand_acceptable_err_level = 0;
static uint32_t asm9260_nand_ecc_correction_ability = 0;
static int read_cache_byte_cnt = 0;
static uint8_t read_cache[4] = {0};
static uint32_t *read_val = (uint32_t *)read_cache;
static uint8_t __attribute__((aligned(32))) NandAddr[32] = {0}; 
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
static uint8_t *asm9260_nand_verify_buffer = NULL;
#endif
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
static uint8_t *asm9260_nand_dma_buf;
static uint8_t *asm9260_nand_dma_read_buf;
static uint8_t *asm9260_nand_dma_write_buf;
#endif

/*
* 等待NAND控制器ready
*/
static int asm9260_nand_controller_ready(void)
{
	int ret = 1;
	int waittime = 0;
	
	while ((nand_regs->nand_status) & (1UL << 8))
	{
		waittime++;
		if (waittime > 0x1000000)
		{
			ret = 0;
			break;
		}
	}
	return ret;
}

/*
* 等待NAND设备ready
*/
static int asm9260_nand_dev_ready(struct mtd_info *mtd)
{
	int ret = 1;
	int waittime = 0;
	
	while (!((nand_regs->nand_status) & (1UL << 0)))
	{
		waittime++;
		if (waittime > 0x1000000)
		{
			ret = 0;
			break;
		}
	}
	return ret;
}

#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
/*
* 等待DMA传输完成
*/
static int asm9260_nand_dma_ready(void)
{
	int ret = 1;
	int waittime = 0;

	while(!(nand_regs->nand_dma_ctrl & 0x00000001))
	{
		waittime++;
		if (waittime > 0x1000000)
		{
			ret = 0;
			break;
		}
	}
	return ret;
}

/*
* 检查DMA传输是否发生错误
*/
static int asm9260_nand_dma_error(void)
{
	int ret = 0;

	if ((nand_regs->nand_dma_ctrl & 0x00000002) != 0)
	{
		return -EIO;
	}
	
	return ret;
}
#endif

/*
* NAND芯片复位
*/
int asm9260_nand_reset(uint8_t nChip)
{
	nand_regs->nand_mem_ctrl = (0xff00 | nChip);
	nand_regs->nand_command  = (RESET << 8)
					        | (ADDR_SEL_0 << 7)
					        | (INPUT_SEL_BIU << 6)
					        | (SEQ0);
	

	return !asm9260_nand_dev_ready(nand_info);
}

static void asm9260_nand_pin_mux(void)
{
	/*set pin assign*/
    set_pin_mux(11,0,5);
    set_pin_mux(11,1,5);
    set_pin_mux(11,2,5);
    set_pin_mux(11,3,5);
    set_pin_mux(11,4,5);
    set_pin_mux(11,5,5);
    set_pin_mux(11,6,5);
    set_pin_mux(12,0,5);
    set_pin_mux(12,1,5);
    set_pin_mux(12,2,5);
    set_pin_mux(12,3,5);
    set_pin_mux(12,4,5);
    set_pin_mux(12,5,5);
    set_pin_mux(12,6,5);
    set_pin_mux(12,7,5);
}

/*
* 设置NAND控制器时序
*/
static int asm9260_nand_timing_config(void)
{
	int ret = 0;
	uint32_t twhr;
	uint32_t trhw;
	uint32_t trwh;
	uint32_t trwp;
	uint32_t tadl = 0;
	uint32_t tccs = 0;
	uint32_t tsync = 0;
	uint32_t trr = 0;
	uint32_t twb = 0;

	/*default config before read id*/

	nand_regs->nand_control = (ADDR_CYCLE_1 << 18) 		| (ADDR1_AUTO_INCR_DIS << 17)
						   | (ADDR0_AUTO_INCR_DIS << 16)	| (WORK_MODE_ASYNC << 15)
						   | (PROT_DIS << 14) 			| (LOOKUP_DIS << 13)
						   | (IO_WIDTH_8 << 12) 		 	| (DATA_SIZE_CUSTOM << 11)
						   | (PAGE_SIZE_4096B << 8) 		| (BLOCK_SIZE_32P << 6)
						   | (ECC_DIS << 5) 				| (INT_DIS << 4)
						   | (SPARE_DIS << 3) 			| (ADDR_CYCLE_1);
	

	// init timing registers
//	trwh = 8;
//	trwp = 8;

//	nand_regs->nand_timings_asyn = (trwh << 4) | (trwp);

//	ret = asm9260_nand_reset(0);
//	if (ret != 0)
//	{
//		return ret;
//	}
	
	trwh = 1; //TWH;
	trwp = 1; //TWP;
	nand_regs->nand_timings_asyn = (trwh << 4) | (trwp);

	twhr = 2;
	trhw = 4;
	nand_regs->nand_time_seq_0 = (twhr << 24) |
						         (trhw << 16) |
						         (tadl << 8)  |
						         (tccs);

	nand_regs->nand_time_seq_1 = (tsync << 16) |
						         (trr << 9) |
						         (twb);
	
	return ret;
}

static int asm9260_nand_inithw(uint8_t nChip)
{
	int ret = 0;

	/*open clk*/
	writel(0x00000400, HW_AHBCLKCTRL1 + 4);  // open nand pclk
	writel(0x00000008, HW_NANDCLKDIV);      // set nand clk to 1/2 pclk

	asm9260_nand_pin_mux();				/*设置PIN Mux*/

	nand_regs->nand_mem_ctrl = (0xff00 |  nChip);
	nand_regs->nand_mem_ctrl = (1UL << (nChip+8)) ^ (nand_regs->nand_mem_ctrl);
	

	ret = asm9260_nand_timing_config();		/*设置NAND控制器时序*/
	if (ret != 0)
	{
		return ret;
	}
	
	ret = asm9260_nand_reset(nChip);	/*复位*/

	return ret;
}

static void asm9260_select_chip(struct mtd_info *mtd, int chip)
{
	if (chip == -1)
	{
		nand_regs->nand_mem_ctrl = 0xFF00;		
	}
	else
	{
		nand_regs->nand_mem_ctrl = 0xFF00 | chip;
		nand_regs->nand_mem_ctrl = (1UL << (chip+8)) ^ (nand_regs->nand_mem_ctrl);	//clear WP reg
	}
}

static void asm9260_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
	static int count = 0;
	printf("asm9260_cmd_ctrl count %d\n", ++count);
}

static __inline int constant_fls(int x)
{
	int r = 32;

	if (!x)
		return 0;
	if (!(x & 0xffff0000u)) {
		x <<= 16;
		r -= 16;
	}
	if (!(x & 0xff000000u)) {
		x <<= 8;
		r -= 8;
	}
	if (!(x & 0xf0000000u)) {
		x <<= 4;
		r -= 4;
	}
	if (!(x & 0xc0000000u)) {
		x <<= 2;
		r -= 2;
	}
	if (!(x & 0x80000000u)) {
		x <<= 1;
		r -= 1;
	}
	return r;
}

int __ffs(unsigned long x)
{
	unsigned long __t = (x);

	return (constant_fls((int)(__t & -__t)) - 1);
}

/*
* 配置NAND控制器
*/

static void asm9260_nand_controller_config (struct mtd_info *mtd)
{
	static int count = 1;
	uint32_t chip_size   = mtd->size;
	uint32_t page_size   = mtd->writesize;

	if (count)
	{
		count = 0;
		page_shift = __ffs(page_size);
		block_shift = __ffs(mtd->erasesize) - page_shift;
		
		addr_cycles = 2 + (((chip_size >> page_size) > 65536) ? 3 : 2);
		col_cycles  = 2;
		row_cycles  = addr_cycles - col_cycles;
		DBG("page_shift: 0x%x.\n", page_shift);
		DBG("block_shift: 0x%x.\n", block_shift);
		DBG("col_cycles: 0x%x.\n", col_cycles);
		DBG("addr_cycles: 0x%x.\n", addr_cycles);
	}
	
	nand_regs->nand_control = (EN_STATUS << 23)				| (NO_RNB_SEL << 22)
						    | (BIG_BLOCK_EN << 21)	 		| (addr_cycles << 18)
						    | (ADDR1_AUTO_INCR_DIS << 17) 	| (ADDR0_AUTO_INCR_DIS << 16)
					 	    | (WORK_MODE_ASYNC << 15)	 	| (PROT_DIS << 14)
						    | (LOOKUP_DIS << 13)			 	| (IO_WIDTH_8 << 12)
						    | (DATA_SIZE_FULL_PAGE << 11) 	| (((page_shift - 8) & 0x7) << 8)
						    | (((block_shift - 5) & 0x3) << 6)	 | (ECC_EN<< 5)
						    | (INT_DIS << 4)				 	| (SPARE_EN << 3)
						    | (addr_cycles);

}


/*******************************************************************************
*函数名:	asm9260_nand_make_addr_lp
*功能:		设置将要发送到控制器的地址，控制器将地址处理后发送给NAND。
*输入参数:	nPage--NAND页数
*			oob_flag--设置的如果是OOB的地址，该参数需为1，否则为0
*输出参数:	pAddr--设置后地址的保存处
*返回值:	无
*NOTE:		1、调用方需保证存放地址的内存，
*			该驱动申请了全局数组NandAddr[32]专用于设置地址的存放
*			2、擦除时，控制器会自动取行地址发给NAND，因此不需要进行专门的处理
*******************************************************************************/
static void asm9260_nand_make_addr_lp(struct mtd_info *mtd, uint32_t nPage, uint32_t nColumn, uint8_t *pAddr)
{
	int i = 0;
	uint32_t row_addr = nPage;

	//清空
	memset(pAddr, 0, 32);

	//设置列地址
	for (i=0; i<col_cycles; i++)
	{
		pAddr[i] = (uint8_t)(nColumn & 0xFF);
		nColumn = nColumn >> 8;
	}
	
	//设置行地址,其实就是nPage页号
	for (i = col_cycles; i < addr_cycles; i++)
	{
		pAddr[i] = (uint8_t)(row_addr & 0xFF);		//字节掩码
		row_addr = row_addr >> 8;				//字节位数
	}
}


/**
 * nand_command_lp - [DEFAULT] Send command to NAND large page device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device. This is the version for the new large page
 * devices We dont have the separate regions as we have in the small page
 * devices.  We must emulate NAND_CMD_READOOB to keep the code compatible.
 */
static void asm9260_nand_command_lp(struct mtd_info *mtd, unsigned int command, int column, int page_addr)
{
	uint32_t *addr = (uint32_t *)NandAddr;
	int ret;
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	static int flag = 0;
#endif

	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	ret = !asm9260_nand_dev_ready(0);
	if (ret)
		DBG("wait for device ready timeout.\n");
	
	switch (command)
	{
		case NAND_CMD_PAGEPROG:
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA

			if (flag == 1)
			{
				flag = 0;
				nand_regs->nand_dma_ctrl = (DMA_START_EN<<7)
										 | (DMA_DIR_WRITE<<6)
										 | (DMA_MODE_SFR<<5)
										 | (DMA_BURST_INCR16<<2);
				nand_regs->nand_command  = (PROGRAM_PAGE_2<<16)
										 | (PROGRAM_PAGE_1<<8)
										 | (ADDR_SEL_0<<7)
										 | (INPUT_SEL_DMA<<6)
										 | (SEQ12);

				break;
			}
#endif

		case NAND_CMD_CACHEDPROG:
		case NAND_CMD_ERASE2:
			break;
			
		case NAND_CMD_RESET:
			nand_regs->nand_command = (RESET << 8)
							        | (ADDR_SEL_0 << 7)
							        | (INPUT_SEL_BIU << 6)
							        | (SEQ0);
			break;

		case NAND_CMD_READID:
			nand_regs->nand_control   = (ADDR_CYCLE_1 << 18) 		| (ADDR1_AUTO_INCR_DIS << 17)
									  | (ADDR0_AUTO_INCR_DIS << 16)	| (WORK_MODE_ASYNC << 15)
									  | (PROT_DIS << 14) 			| (LOOKUP_DIS << 13)
									  | (IO_WIDTH_8 << 12) 		 	| (DATA_SIZE_CUSTOM << 11)
									  | (PAGE_SIZE_4096B << 8) 		| (BLOCK_SIZE_32P << 6)
									  | (ECC_DIS << 5) 				| (INT_DIS << 4)
									  | (SPARE_DIS << 3) 			| (ADDR_CYCLE_1); 
			nand_regs->nand_fifo_init = 1;	//reset FIFO
			nand_regs->nand_data_size = 4;	//ID 4 Bytes
			nand_regs->nand_addr0_l   = column;
			nand_regs->nand_command   = (READ_ID << 8)
									  | (ADDR_SEL_0 << 7)
									  | (INPUT_SEL_BIU << 6)
									  | (SEQ1);

			read_cache_byte_cnt = 0;
			
			break;

		case NAND_CMD_READ0:

			/*1、复位FIFO，配置NAND控制器*/
			nand_regs->nand_fifo_init = 1;
			
			asm9260_nand_controller_config(mtd);

			if (column == 0)
			{
				nand_regs->nand_ecc_ctrl = (asm9260_nand_acceptable_err_level << 8) | (asm9260_nand_ecc_correction_ability << 5);
				nand_regs->nand_ecc_offset = mtd->writesize + asm9260_nand_spare_data_size;
				nand_regs->nand_spare_size = asm9260_nand_spare_data_size;
			}
			else if (column == mtd->writesize)
			{
				nand_regs->nand_control = ((nand_regs->nand_control) & (~(ECC_EN << 5))) | (DATA_SIZE_CUSTOM<< 11);
				nand_regs->nand_spare_size = mtd->oobsize;
				nand_regs->nand_data_size = mtd->oobsize;
			}
			else
			{
				printk("couldn't support the column\n");
				break;
			}

			/*2、选择chip，配置NAND地址*/
			asm9260_nand_make_addr_lp(mtd, page_addr, column, NandAddr);

			nand_regs->nand_addr0_l = addr[0];
			nand_regs->nand_addr0_h = addr[1];

#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
			if (column == 0)
			{
				/*DMA配置，DMA有两种模式，其中SFR managed模式在单个DMA包情况下使用*/
				nand_regs->nand_dma_addr = (unsigned int)asm9260_nand_dma_read_buf;
				nand_regs->nand_dma_cnt  = mtd->writesize + (unsigned int)asm9260_nand_spare_data_size;
				nand_regs->nand_dma_ctrl = (DMA_START_EN<<7)
										 | (DMA_DIR_READ<<6)
										 | (DMA_MODE_SFR<<5)
										 | (DMA_BURST_INCR16<<2);
				nand_regs->nand_command  = (READ_PAGE_2<<16)
										 | (READ_PAGE_1<<8)
										 | (ADDR_SEL_0<<7)
										 | (INPUT_SEL_DMA<<6)
										 | (SEQ10);
				break;
			}
#endif

			/*3、发起命令*/
			nand_regs->nand_command = (READ_PAGE_2<<16)
								    | (READ_PAGE_1<<8)
								    | (ADDR_SEL_0<<7)
								    | (INPUT_SEL_BIU<<6)
								    | (SEQ10);
			
			read_cache_byte_cnt = 0;
			break;
		case NAND_CMD_SEQIN:

			/*1、复位FIFO，配置NAND控制器*/
			nand_regs->nand_fifo_init = 1;
			asm9260_nand_controller_config(mtd);

			if (column == 0)
			{
				nand_regs->nand_ecc_ctrl = (asm9260_nand_acceptable_err_level << 8) | (asm9260_nand_ecc_correction_ability << 5);
				nand_regs->nand_ecc_offset = mtd->writesize + asm9260_nand_spare_data_size;
				nand_regs->nand_spare_size = asm9260_nand_spare_data_size;
			}
			else if (column == mtd->writesize)
			{
				nand_regs->nand_control = ((((nand_regs->nand_control)) | (DATA_SIZE_CUSTOM << 11)) & (~(ECC_EN << 5))) & (~(SPARE_EN << 3));
				nand_regs->nand_data_size = mtd->oobsize;
			}
 
			/*2、选择chip，配置NAND地址*/
			asm9260_nand_make_addr_lp(mtd, page_addr, column, NandAddr);
			nand_regs->nand_addr0_l = addr[0];
			nand_regs->nand_addr0_h = addr[1];

#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
			if (column == 0)
			{
				flag = 1;

				/*DMA配置，DMA有两种模式，其中SFR managed模式在单个DMA包情况下使用*/
				nand_regs->nand_dma_addr = (unsigned int)asm9260_nand_dma_write_buf;
				nand_regs->nand_dma_cnt  = mtd->writesize + asm9260_nand_spare_data_size;
				break;
			}
#endif

			/*3、发起命令*/
			nand_regs->nand_command = (PROGRAM_PAGE_2 << 16)
								    | (PROGRAM_PAGE_1 << 8)
								    | (ADDR_SEL_0 << 7)
								    | (INPUT_SEL_BIU << 6)
								    | (SEQ12);

			break;
		case NAND_CMD_STATUS:
			
			asm9260_nand_controller_config(mtd);
			nand_regs->nand_control = ((((nand_regs->nand_control) & (~(SPARE_EN << 3))) & (~(ECC_EN<< 5))) | (DATA_SIZE_CUSTOM << 11));
			nand_regs->nand_data_size = 1;
			nand_regs->nand_command = (READ_STATUS<<8) |
							          (ADDR_SEL_0<<7) |
							          (INPUT_SEL_BIU<<6) |
							          (SEQ1);

			read_cache_byte_cnt = 0;
			break;

		case NAND_CMD_ERASE1:

			asm9260_nand_make_addr_lp(mtd, page_addr, column, NandAddr);
			nand_regs->nand_addr0_l = addr[0];
			nand_regs->nand_addr0_h = addr[1];

			asm9260_nand_controller_config(mtd);
			nand_regs->nand_control = (nand_regs->nand_control) & ((~(ECC_EN << 5)) & (~(SPARE_EN << 3)));

			nand_regs->nand_command = (ERASE_BLOCK_2<<16)
									| (ERASE_BLOCK_1<<8)
									| (ADDR_SEL_0<<7)
									| (INPUT_SEL_BIU<<6)
									| (SEQ14);
			break;

		default:
			printk("don't support this command : 0x%x!\n", command);
	}

	if((command == NAND_CMD_RESET) || (command == NAND_CMD_READID) || (command == NAND_CMD_STATUS) || (command == NAND_CMD_ERASE1))
	{
		ret = !asm9260_nand_dev_ready(0);
		if (ret)
			DBG("wait for device ready timeout, ret = 0x%x.\n", ret);
	}
	else if ((command == NAND_CMD_READ0))
	{
#ifdef	CONFIG_MTD_NAND_ASAP9260_DMA
	
		if (column == 0)
		{
			ret = !asm9260_nand_dev_ready(0);
			if (ret)
				DBG("wait for device ready timeout, ret = 0x%x.\n", ret);

			ret = !asm9260_nand_dma_ready();
			if (ret)
				DBG("wait for dma ready timeout, ret = 0x%x.\n", ret);
		}
		else
		{
			ret = !asm9260_nand_controller_ready();
			if (ret)
				DBG("wait for device ready timeout, ret = 0x%x.\n", ret);
		}
#else
		ret = !asm9260_nand_controller_ready();
		if (ret)
			DBG("wait for device ready timeout, ret = 0x%x.\n", ret);
#endif
	}
	return ;
}

static u_int8_t asm9260_nand_read_byte(struct mtd_info *mtd)
{
	uint8_t this_byte;

	if ((read_cache_byte_cnt <= 0) || (read_cache_byte_cnt > 4))
	{
		*read_val = (nand_regs->nand_fifo_data);
		read_cache_byte_cnt = 4;
	}

	this_byte = read_cache[sizeof(read_cache) - read_cache_byte_cnt];
	read_cache_byte_cnt--;

	return this_byte;
}

static uint16_t asm9260_nand_read_word(struct mtd_info *mtd)
{
	uint16_t this_word = 0;
	uint16_t *val_tmp = (uint16_t *)read_cache;
		
	if ((read_cache_byte_cnt <= 0) || (read_cache_byte_cnt > 4))
	{
		*read_val = (nand_regs->nand_fifo_data);
		read_cache_byte_cnt = 4;
	}

	if (read_cache_byte_cnt == 4)
		this_word = val_tmp[0];
	else if (read_cache_byte_cnt == 2)
		this_word = val_tmp[1];

	read_cache_byte_cnt -= 2;

	return this_word;
}

static void asm9260_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	uint32_t i;
	uint32_t *tmpbuf = (u32 *)buf;

	if (len & 0x3)
	{
		printk("Unsupported length\n");
		return;
	}

	for (i = 0; i < (len>>2); i++)
	{
		tmpbuf[i] = (nand_regs->nand_fifo_data);
	}
}


static void asm9260_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	u32 i;
	u32 *tmpbuf = (u32 *)buf;

	if (len & 0x3)
	{
		printk("Unsupported length\n");
		return;
	}

	for (i = 0; i < (len >> 2); i++)
	{
		nand_regs->nand_fifo_data = tmpbuf[i];
	}

}

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
/**
 * nand_verify_buf - [DEFAULT] Verify chip data against buffer
 * @mtd:	MTD device structure
 * @buf:	buffer containing the data to compare
 * @len:	number of bytes to compare
 *
 * Default verify function for 8bit buswith
 */
static int asm9260_nand_verify_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	int i;

#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	memcpy(asm9260_nand_verify_buffer, asm9260_nand_dma_read_buf, mtd->writesize + mtd->oobsize);
#else
	asm9260_nand_read_buf(mtd, asm9260_nand_verify_buffer, mtd->writesize + asm9260_nand_spare_data_size);
#endif

	for (i = 0; i < len; i++)
	{
		if (buf[i] != asm9260_nand_verify_buffer[i])
		{
			printk("nand verify buffer error!!  val i: 0x%x\n", i);
			return -EFAULT;
		}
	}
	return 0;
}
#endif

static int asm9260_nand_wait(struct mtd_info *mtd, struct nand_chip *chip)
{
	int status = 0;
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	int ret = 0;
#endif

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(100);

	chip->dev_ready(mtd);

#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	ret = !asm9260_nand_dma_ready();
	if (ret != 0)
		printk("ASM9260 dma not ready\n");
	
	ret = asm9260_nand_dma_error();
	if (ret != 0)
		printk("ASM9260 dma error\n");
#endif
	chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
	status = (int)chip->read_byte(mtd);

	return status;
}

static void asm9260_nand_write_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t *buf)
{
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	memcpy(asm9260_nand_dma_write_buf, buf, mtd->writesize);  // copy data to buf
	memcpy(asm9260_nand_dma_write_buf + mtd->writesize, chip->oob_poi, mtd->oobsize);  // copy data to oobbuf                                                
#else
	uint8_t *temp_ptr;
	temp_ptr = (uint8_t *)buf;
	chip->write_buf(mtd, temp_ptr, mtd->writesize);

	temp_ptr = chip->oob_poi;
	chip->write_buf(mtd, temp_ptr, asm9260_nand_spare_data_size);
#endif
	return ;
}

static int asm9260_nand_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip, uint8_t *buf, int page)
{
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	memcpy(buf, asm9260_nand_dma_read_buf, mtd->writesize);  // copy data to buf
	memcpy(chip->oob_poi, asm9260_nand_dma_read_buf + mtd->writesize, mtd->oobsize);  // copy data to oobbuf                                                
#else
	uint8_t *temp_ptr;
	temp_ptr = buf;
	chip->read_buf(mtd, temp_ptr, mtd->writesize);

	temp_ptr = chip->oob_poi;
	memset(temp_ptr, 0xff, mtd->oobsize);
	chip->read_buf(mtd, temp_ptr, asm9260_nand_spare_data_size);
#endif

	return 0;
}

static int asm9260_nand_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
			      uint8_t *buf, int page)
{
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	memcpy(buf, asm9260_nand_dma_read_buf, mtd->writesize);  // copy data to buf
	memcpy(chip->oob_poi, asm9260_nand_dma_read_buf + mtd->writesize, mtd->oobsize);  // copy data to oobbuf   
	chip->waitfunc(mtd, chip);
	
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0x00, page);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	chip->waitfunc(mtd, chip);
#else
	uint8_t *temp_ptr;
	temp_ptr = buf;
	chip->read_buf(mtd, temp_ptr, mtd->writesize);

	temp_ptr = chip->oob_poi;
	chip->read_buf(mtd, temp_ptr, asm9260_nand_spare_data_size);
	chip->waitfunc(mtd, chip);
	
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0x00, page);
	chip->read_buf(mtd, temp_ptr, mtd->oobsize);
	chip->waitfunc(mtd, chip);
#endif

	return 0;
}
#if 0
static void asm9260_nand_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				const uint8_t *buf)
{
	chip->write_buf(mtd, buf, mtd->writesize);
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
}
#endif
static void asm9260_nand_init_chip(struct nand_chip *nand_chip)
{
	nand_chip->select_chip = asm9260_select_chip;
	nand_chip->cmd_ctrl    = asm9260_cmd_ctrl;
	nand_chip->IO_ADDR_R   = (void  __iomem	*)(&nand_regs->nand_fifo_data);
	nand_chip->IO_ADDR_W   = (void  __iomem	*)(&nand_regs->nand_fifo_data);
	nand_chip->cmdfunc     = asm9260_nand_command_lp;
	nand_chip->read_byte   = asm9260_nand_read_byte;
	nand_chip->read_word   = asm9260_nand_read_word;
	nand_chip->read_buf    = asm9260_nand_read_buf;
	nand_chip->write_buf   = asm9260_nand_write_buf;
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	nand_chip->verify_buf  = asm9260_nand_verify_buf;
#endif
	nand_chip->dev_ready   = asm9260_nand_dev_ready;
	nand_chip->chip_delay  = 100;
	nand_chip->waitfunc    = asm9260_nand_wait;	

#ifdef CONFIG_MTD_NAND_ASM9260_HWECC
	nand_chip->ecc.mode = NAND_ECC_HW;
#else 
	nand_chip->ecc.mode = NAND_ECC_NONE;
#endif
	
	nand_chip->ecc.write_page = asm9260_nand_write_page_hwecc;
	nand_chip->ecc.read_page  = asm9260_nand_read_page_hwecc;
	nand_chip->ecc.read_page_raw = asm9260_nand_read_page_raw;
	//nand_chip->ecc.write_page_raw = asm9260_nand_write_page_raw;
	nand_chip->ecc.calculate = NULL;
	nand_chip->ecc.correct   = NULL;
	nand_chip->ecc.hwctl     = NULL;
	
}

int asm9260_ecc_cap_select(int nand_page_size, int nand_oob_size)
{
	int ecc_bytes = 0;
	int i;

	for (i=(ARRAY_SIZE(ecc_info_table) - 1); i>=0; i--)
	{
		if ((nand_oob_size - ecc_info_table[i].ecc_bytes_per_sector * (nand_page_size >> 9)) > (28 + 2))
		{
			asm9260_nand_ecc_correction_ability = ecc_info_table[i].ecc_cap;
			asm9260_nand_acceptable_err_level = ecc_info_table[i].ecc_threshold;
			ecc_bytes = ecc_info_table[i].ecc_bytes_per_sector * (nand_page_size >> 9);
			break;
		}
	}

	return ecc_bytes;
}

int board_nand_init(struct nand_chip *nand)
{
	int ret = 0, i;
	struct nand_chip *asm9260_nand = nand;

	/* initialise the hardware */
	ret = asm9260_nand_inithw(0);
	if (ret != 0)
	{
		printk("init failed,ret = 0x%x\n",ret);
		return -EIO;
	}

	asm9260_nand_init_chip(asm9260_nand);

	/* first scan to find the device and get the page size */
	if (nand_scan_ident(nand_info, 1)) {
		printf("nand scan ident failed.\n");
		return -ENXIO;
	}

	if (asm9260_nand->ecc.mode == NAND_ECC_HW) {
		/* ECC is calculated for the whole page (1 step) */
		asm9260_nand->ecc.size = nand_info->writesize;
		asm9260_nand->ecc.bytes  = asm9260_ecc_cap_select(nand_info->writesize, nand_info->oobsize);
		asm9260_nand->ecc.layout = (struct nand_ecclayout *)malloc(sizeof(struct nand_ecclayout));
		memset(asm9260_nand->ecc.layout, 0, sizeof(struct nand_ecclayout));
		asm9260_nand->ecc.layout->eccbytes = asm9260_nand->ecc.bytes;
		
		for (i=0; i<asm9260_nand->ecc.bytes; i++)
		{
			asm9260_nand->ecc.layout->eccpos[i] = nand_info->oobsize - asm9260_nand->ecc.bytes + i;
		}
		asm9260_nand->ecc.layout->oobfree[0].offset = 2;
		asm9260_nand->ecc.layout->oobfree[0].length = nand_info->oobsize - asm9260_nand->ecc.bytes - 2;
	}
	else
		printf("CONFIG_MTD_NAND_ASM9260_HWECC must be defined in asm9260.h!");
	
	asm9260_nand_spare_data_size = nand_info->oobsize - asm9260_nand->ecc.bytes;
	DBG("asm9260_nand_spare_data_size : 0x%x. \n", asm9260_nand_spare_data_size);

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	asm9260_nand_verify_buffer = malloc(nand_info->writesize + nand_info->oobsize);
	if (!asm9260_nand_verify_buffer) {
		printf("asm9260_nand_verify_buffer: failed to allocate mtd_info storage\n");
		return -ENOMEM;
	}
#endif

#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	asm9260_nand_dma_buf = (unsigned char *)malloc((nand_info->writesize + nand_info->oobsize) * 2);
	if (!asm9260_nand_dma_buf)
	{
		printf("asm9260_nand: failed to allocate dma buffer storage\n");
		free(asm9260_nand_verify_buffer);
		return -ENOMEM;
	}
	
	asm9260_nand_dma_read_buf = asm9260_nand_dma_buf;
	asm9260_nand_dma_write_buf = asm9260_nand_dma_buf + (nand_info->writesize + nand_info->oobsize);
#endif

	/* second phase scan */
	if (nand_scan_tail(nand_info)) {
		printf("nand scan tail failed.\n");
		return -ENXIO;
	}

	DBG("mtd->writesize 0x%x\n", nand_info->writesize);
	DBG("mtd->erasesize 0x%x\n", nand_info->erasesize);
	DBG("mtd->oobsize 0x%x\n", nand_info->oobsize);
	DBG("chip->chipsize 0x%x\n", (uint32_t)asm9260_nand->chipsize);
	DBG("chip->page_shift 0x%x\n", asm9260_nand->page_shift);
	DBG("chip->phys_erase_shift 0x%x\n", asm9260_nand->phys_erase_shift);
	DBG("chip->pagemask 0x%x\n", asm9260_nand->pagemask);
	DBG("chip->badblockpos 0x%x\n", asm9260_nand->badblockpos);
	DBG("chip->bbt_erase_shift 0x%x\n", asm9260_nand->bbt_erase_shift);
	DBG("chip->buffers 0x%x\n", (uint32_t)asm9260_nand->buffers);
	DBG("chip->chip_shift 0x%x\n", asm9260_nand->chip_shift);
	DBG("chip->options 0x%x\n", asm9260_nand->options);

	return 0;
}



