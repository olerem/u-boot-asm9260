#ifndef __ASM9260_USB_H__
#define __ASM9260_USB_H__

#include <asm/types.h>
#include <asm/arch/hardware.h>
#include "musb_core.h"



#define SET	4
#define CLR 8

#define SYSPLL_PD_EN           1<<2
#define SYSPLL_PD_DIS          (0xFFFFFFFB)

#define USB0_PD_EN             1<<8
#define USB0_PD_DIS            (0xFFFFFEFF)

#define USB1_PD_EN             1<<9
#define USB1_PD_DIS            (0xFFFFFDFF)

#define USBPLL_PD_EN           1<<10
#define USBPLL_PD_DIS          (0xFFFFFCFF)


#define AHB_CLK_ENABLE_USB1 	1<<8
#define AHB_PRERESET_USB1 		1<<8

#define SYS_PLL_STAT_NOPOR 		0
#define SYS_PLL_STAT_POR 		1

#define MAIN_CLK_SEL_12M 		0
#define MAIN_CLK_SEL_SYSPLL 	1

#define MAIN_CLK_SEL_DISABLE	0
#define MAIN_CLK_SEL_ENABLE 	1


int musb_platform_init(void);
void musb_platform_deinit(void);


#endif
