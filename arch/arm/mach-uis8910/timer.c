#include <common.h>
#include <asm/io.h>
//#include <asm/arch/cpu.h>
//#include <asm/arch/timer.h>

int timer_init(void)
{
	return 0;
}

uint64_t get_timer_us(uint64_t base)
{
	return 0;
}

unsigned long get_timer(unsigned long base)
{
	uint64_t us = get_timer_us(0);

	return us;
}

unsigned long long get_ticks(void)
{
	return get_timer(0);
}

ulong get_tbclk(void)
{
	return CONFIG_SYS_HZ;
}

void __udelay(unsigned long usec)
{
	uint64_t endtime;
	signed long diff;

	endtime = get_timer_us(0) + usec;

	do {
		uint64_t now = get_timer_us(0);
		diff = endtime - now;
	} while (diff >= 0);
} 