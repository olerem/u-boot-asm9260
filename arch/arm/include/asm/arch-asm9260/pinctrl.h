#ifndef __PIN_CTRL_H__
#define __PIN_CTRL_H__

#include <asm/types.h>

#define HW_PIN_CTRL_PIN2IRQ_OFF		(0x80)
#define HW_PIN_CTRL_IRQEN_OFF		(0x90)
#define HW_PIN_CTRL_IRQLEVEL_OFF	(0xA0)
#define HW_PIN_CTRL_IRQPOL_OFF		(0xB0)
#define HW_PIN_CTRL_IRQSTAT_OFF		(0xC0)
/* GPIO TYPE */
#define PIN_FUNCTION_0      0
#define PIN_FUNCTION_1      1
#define PIN_FUNCTION_2      2
#define PIN_FUNCTION_3      3
#define PIN_FUNCTION_4      4
#define PIN_FUNCTION_5      5
#define PIN_FUNCTION_6      6
#define PIN_FUNCTION_7      7

#define PIN_FUNCTION_GPIO   0

void set_pin_mux(u32 port, u32 pin, u32 mux_type);
void set_pin_dir(u32 nPort, u32 nPin, int bOut);
void set_gpio(u32 nPort, u32 nPin);
void clear_gpio(u32 nPort, u32 nPin);
void write_gpio(u32 nPort, u32 nPin, u32 nValue);
void write_port(u32 nPort, u32 nMask, u32 nValue);
u32 read_gpio(u32 nPort, u32 nPin);
u32 read_port(u32 nPort);
void set_pin_pullup(u32 port,u32 pin); 

#define GPIO_IRQ_LEVEL_LOW			0
#define GPIO_IRQ_LEVEL_HIGH			1
#define GPIO_IRQ_EDGE_FALLING		0
#define GPIO_IRQ_EDGE_RISING		1

void gpioIrq_enable_edge(u32 nPort, u32 nPin, u32 nType);
void gpioIrq_enable_level(u32 nPort, u32 nPin, u32 nType);
void gpioIrq_disable(u32 nPort, u32 nPin);
void gpioIrq_enable(u32 nPort, u32 nPin);
void gpioIrq_clear(u32 nPort, u32 nPin);
int is_gpioIrq_pending(u32 nPort, u32 nPin);

#endif

