/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Alex Zuepke <azu@sysgo.de>
 *
 * (C) Copyright 2002
 * Gary Jennejohn, DENX Software Engineering, <garyj@denx.de>
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
#include <asm/arch/hardware.h>
#include <asm/io.h>

int timer_load_val = 0;
static ulong timer_clk;

/* macro to read the 16 bit timer */
static inline ulong READ_TIMER(void)
{
	return readl(HW_TIMER0_TC0);
}

static ulong timestamp;
static ulong lastdec;

int timer_init(void)
{
	unsigned int pllclk ,hclkdiv, cpuclkdiv, hclk;

	hclkdiv = readl(HW_SYSAHBCLKDIV);
	cpuclkdiv = readl(HW_CPUCLKDIV);
	pllclk = readl(HW_SYSPLLCTRL);
	hclk = ((pllclk / cpuclkdiv) / hclkdiv);

	writel(1<<4, HW_AHBCLKCTRL1 + 4);
	writel(1<<4, HW_PRESETCTRL1 + 8);
	writel(1<<4, HW_PRESETCTRL1 + 4);
	writel(1<<4, HW_AHBCLKCTRL1 + 4);

	writel(0x3,HW_TIMER0_DIR + 8);      // 
	writel(0x1,HW_TIMER0_DIR + 4);      // timer0 count-down

	writel(0x10000000, HW_TIMER0_MR0);         //
	writel(hclk * 1000000 / CONFIG_SYS_HZ -1, HW_TIMER0_PR);
	if (timer_load_val == 0) {
		timer_load_val = 0x10000000;
		timer_clk = CONFIG_SYS_HZ;
	}

	writel(0x3, HW_TIMER0_CTCR + 8);    // timer mode--timer
	writel(0x2, HW_TIMER0_MCR + 4);     // if (tc == mr0) then reset tc
	writel(1<<0, HW_TIMER0_TCR + 4);    // enable timer0

	return (0);
}

/*
 * timer without interrupts
 */

void reset_timer(void)
{
	reset_timer_masked();
}

ulong get_timer(ulong base)
{
	return get_timer_masked() - base;
}

void set_timer(ulong t)
{
	timestamp = t;
}

void __udelay (unsigned long usec)
{
	ulong tmo;
	ulong start = get_ticks();

	tmo = usec / (1000000 / timer_clk);
			
	while ((ulong) (get_ticks() - start) < tmo)
		/*NOP*/;
}

void reset_timer_masked(void)
{
	/* reset time */
	lastdec = READ_TIMER();
	timestamp = 0;
}

ulong get_timer_masked(void)
{
	ulong tmr = get_ticks();

	return tmr;
}

void udelay_masked(unsigned long usec)
{
	ulong tmo;
	ulong endtime;
	signed long diff;

	tmo = usec / (1000000 / timer_clk);
	
	endtime = get_ticks() + tmo;

	do {
		ulong now = get_ticks();
		diff = endtime - now;
	} while (diff >= 0);
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On ARM it just returns the timer value.
 */
unsigned long long get_ticks(void)
{
	ulong now = READ_TIMER();

	if (lastdec >= now) {
		/* normal mode */
		timestamp += lastdec - now;
	} else {
		/* we have an overflow ... */
		timestamp += lastdec + timer_load_val - now;
	}
	lastdec = now;

	return timestamp;
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
	ulong tbclk;
	
	tbclk = CONFIG_SYS_HZ;

	return tbclk;
}

