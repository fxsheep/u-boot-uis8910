#pragma once

#define CONFIG_SYS_TEXT_BASE    0x810040
#define CONFIG_SYS_LOAD_ADDR    0x80000000

#define CONFIG_SYS_MALLOC_LEN   (CONFIG_ENV_SIZE + (80 << 20))

#define CONFIG_SYS_INIT_RAM_ADDR	0x800000
#define CONFIG_SYS_INIT_RAM_SIZE	0x40000
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_INIT_RAM_ADDR + \
			CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)

#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_SDRAM_BASE		0x800000