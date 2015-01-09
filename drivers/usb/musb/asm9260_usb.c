#include <common.h>
#include <asm/arch/hardware.h>
#include <asm/io.h>
#include "asm9260_usb.h"


/* MUSB platform configuration */
struct musb_config musb_cfg = {
	(struct musb_regs *)USB1_BaseAddr,
	 0x3FFFFFF,
	 0
};



int musb_platform_init(void)
{
	u32 wait_lock = 0; 
	u32 nCPUclk_div = 2;
	u32 nHclk_div = 2;
	u32 nSysPLL = 480;
	u32 nWaitMax = 0x10000;

	/*must set sys pll syspll>60Mhz if usb work*/
	writel(readl(HW_PDRUNCFG)&(SYSPLL_PD_DIS&USB1_PD_DIS&SYSPLL_PD_DIS),HW_PDRUNCFG);//open syspll power
	writel(nCPUclk_div ,HW_CPUCLKDIV); 			//CPU=PLLCLK/2=240MHz
	writel(nHclk_div,HW_SYSAHBCLKDIV); 			//HCK=CPUCLK/2=120MHz
	writel(nSysPLL,HW_SYSPLLCTRL); 			//sysPLL=480MHz

	while((readl(HW_SYSPLLSTAT)&SYS_PLL_STAT_POR)==0x0)//wait syspll lock
		{
				if (wait_lock++ > nWaitMax)         
					{
								break;
						}
		}

	udelay(1000);//wait
	writel(MAIN_CLK_SEL_SYSPLL,HW_MAINCLKSEL); //select syspll to main clk
	writel(MAIN_CLK_SEL_DISABLE,HW_MAINCLKUEN);
	writel(MAIN_CLK_SEL_ENABLE ,HW_MAINCLKUEN);


	writel(AHB_CLK_ENABLE_USB1,HW_AHBCLKCTRL0+SET);//Set Bit 8 open usb1 clk
	
	/*reset USB1:clear bit8 then set bit8*/
	writel(AHB_PRERESET_USB1,HW_PRESETCTRL0+CLR);
	udelay(100);
	writel(AHB_PRERESET_USB1,HW_PRESETCTRL0+SET);
	udelay(100);
	

	writeb(MUSB_POWER_SOFTCONN|MUSB_POWER_HSENAB,USB1_Power); 	//0x40 FULL    0x60 HIGH
	writeb(MUSB_DEVCTL_SESSION,USB1_DevCtl);        // set session bit
	
	return 0;
}



void musb_platform_deinit(void)
{
	writel(AHB_CLK_ENABLE_USB1,HW_AHBCLKCTRL0+SET);//Set Bit 8 open usb1 clk
	/*reset USB1:clear bit8 then set bit8*/
	writel(AHB_PRERESET_USB1,HW_PRESETCTRL0+CLR);
	udelay(100);
	writel(AHB_PRERESET_USB1,HW_PRESETCTRL0+SET);
	udelay(100);
	
}



