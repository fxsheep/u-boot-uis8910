#include <common.h>
#include <dm.h>
#include <errno.h>
#include <serial.h>

int board_init(void)
{
	return 0;
}

u32 get_board_rev(void)
{
	return 0xBEEF;
}

int dram_init(void)
{
	gd->ram_size = 256 * 1024;
	return 0;
}

int print_cpuinfo(void)
{
	return 0;
}