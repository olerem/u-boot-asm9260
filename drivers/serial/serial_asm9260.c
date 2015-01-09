#include <common.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>

void serial_setbrg (void)
{

}

int serial_init (void)
{
	serial_setbrg ();
	
	return (0);
}

void serial_exit (void)
{

}

void serial_putc (const char c)
{
	u32 waittime;
	waittime = 0; 	
	while ( ((readl(HW_UART4_STAT)) & 0x02000000) != 0 )
	{			
		waittime++;
		if(waittime > 0x1000000)
			break;
	}
	writeb(c,HW_UART4_DATA);
	
	if(c == '\n')
		serial_putc('\r');
}

void serial_puts (const char *s)
{
	while (*s) {
		serial_putc (*s++);
	}
}

int serial_getc (void)
{
	while ( ((readl(HW_UART4_STAT)) & 0x01000000) !=0 );

	return readb(HW_UART4_DATA);
}

int serial_tstc (void)
{
	return ((readl(HW_UART4_STAT) & 0x01000000) != 0x01000000);
}
