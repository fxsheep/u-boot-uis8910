#include <common.h>
#include <dm.h>
#include <errno.h>
#include <fdtdec.h>
#include <linux/compiler.h>
//#include <asm/io.h>
//#include <asm/arch/uart.h>
#include <serial.h>
#include <clk.h>

DECLARE_GLOBAL_DATA_PTR;

struct uis8910_uart {
    uint32_t uart_tx;          // 0x00000000
    uint32_t uart_rx;          // 0x00000004
    uint32_t uart_baud;        // 0x00000008
    uint32_t uart_conf;        // 0x0000000c
    uint32_t uart_rxtrig;      // 0x00000010
    uint32_t uart_txtrig;      // 0x00000014
    uint32_t uart_delay;       // 0x00000018
    uint32_t uart_status;      // 0x0000001c
    uint32_t uart_rxfifo_stat; // 0x00000020
    uint32_t uart_txfifo_stat; // 0x00000024
    uint32_t uart_rxfifo_hdlc; // 0x00000028
    uint32_t uart_at_status;   // 0x0000002c
    uint32_t uart_swfc_cc;     // 0x00000030
};

/* Information about a serial port */
struct uis8910_serial_plat {
	struct uis8910_uart *reg;  /* address of registers in physical memory */
};

unsigned halCalcDivider20(unsigned input, unsigned output)
{
    if (input == 0 || output == 0 || output > (input / 6))
        return 0;

    unsigned delta = -1U;
    unsigned rset = 1;
    unsigned rdiv = 1;
    for (unsigned nset = 16; nset >= 6; nset--)
    {
        unsigned ndiv = (input + (nset * output / 2)) / (nset * output);
        if (ndiv <= 1 || ndiv >= (1 << 16))
            continue;

        unsigned out = input / (nset * ndiv);
        unsigned diff = (out > output) ? out - output : output - out;
        if (diff < delta)
        {
            delta = diff;
            rset = nset;
            rdiv = ndiv;
        }
    }

    if (delta == -1U)
        return 0;
    return ((rset - 1) << 16) | (rdiv - 1);
}

static void __maybe_unused uis8910_serial_init(struct uis8910_uart *uart)
{
	writel(550920, &uart->uart_conf);
	writel(40, &uart->uart_delay);
	writel(64, &uart->uart_rxtrig);
	writel(0, &uart->uart_txtrig);
	while ( (readl(&uart->uart_conf) & 0x6000) != 0 );
	writel(readl(&uart->uart_status), &uart->uart_status);
}

static void __maybe_unused uis8910_serial_baud(struct uis8910_uart *uart, uint uclk,
					   int baudrate)
{
    unsigned divider = halCalcDivider20(uclk, baudrate);
	writel(divider, &uart->uart_baud);
}

#ifndef CONFIG_SPL_BUILD
int uis8910_serial_setbrg(struct udevice *dev, int baudrate)
{
	struct uis8910_serial_plat *plat = dev_get_plat(dev);
	struct uis8910_uart *const uart = plat->reg;
	u32 uclk;
	uclk = 26000000;

	uis8910_serial_baud(uart, uclk, baudrate);

	return 0;
}

static int uis8910_serial_probe(struct udevice *dev)
{
	struct uis8910_serial_plat *plat = dev_get_plat(dev);
	struct uis8910_uart *const uart = plat->reg;

	uis8910_serial_init(uart);

	return 0;
}

static int uis8910_serial_getc(struct udevice *dev)
{
	struct uis8910_serial_plat *plat = dev_get_plat(dev);
	struct uis8910_uart *const uart = plat->reg;

	if (!(readb(&uart->uart_rxfifo_stat) & 0xff))
		return -EAGAIN;

	return (int)(readb(&uart->uart_rx) & 0xff);
}

static int uis8910_serial_putc(struct udevice *dev, const char ch)
{
	struct uis8910_serial_plat *plat = dev_get_plat(dev);
	struct uis8910_uart *const uart = plat->reg;

	if ((readb(&uart->uart_txfifo_stat) & 0xff) > 127)
		return -EAGAIN;

	writeb(ch, &uart->uart_tx);

	return 0;
}

static int uis8910_serial_pending(struct udevice *dev, bool input)
{
	struct uis8910_serial_plat *plat = dev_get_plat(dev);
	struct uis8910_uart *const uart = plat->reg;

	if (input)
		return (int)(readb(&uart->uart_rxfifo_stat) & 0xff);
	else
		return (int)(readb(&uart->uart_txfifo_stat) & 0xff);
}

static int uis8910_serial_of_to_plat(struct udevice *dev)
{
	struct uis8910_serial_plat *plat = dev_get_plat(dev);
	fdt_addr_t addr;

	addr = dev_read_addr(dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	plat->reg = (struct uis8910_uart *)addr;

	return 0;
}

static const struct dm_serial_ops uis8910_serial_ops = {
	.putc = uis8910_serial_putc,
	.pending = uis8910_serial_pending,
	.getc = uis8910_serial_getc,
	.setbrg = uis8910_serial_setbrg,
};

static const struct udevice_id uis8910_serial_ids[] = {
	{ .compatible = "unisoc,uis8910-uart" },
	{ }
};

U_BOOT_DRIVER(serial_uis8910) = {
	.name	= "serial_uis8910",
	.id	= UCLASS_SERIAL,
	.of_match = uis8910_serial_ids,
	.of_to_plat = uis8910_serial_of_to_plat,
	.plat_auto = sizeof(struct uis8910_serial_plat),
	.probe = uis8910_serial_probe,
	.ops	= &uis8910_serial_ops,
};
#endif

#ifdef CONFIG_DEBUG_UART_UIS8910

#include <debug_uart.h>

static inline void _debug_uart_init(void)
{
	struct uis8910_uart *uart = (struct uis8910_uart *)CONFIG_DEBUG_UART_BASE;

	uis8910_serial_baud(uart, CONFIG_DEBUG_UART_CLOCK, CONFIG_BAUDRATE);
	uis8910_serial_init(uart);
}

static inline void _debug_uart_putc(int ch)
{
	struct uis8910_uart *uart = (struct uis8910_uart *)CONFIG_DEBUG_UART_BASE;

	while ((readb(&uart->uart_txfifo_stat) & 0xff) > 127);
	writeb(ch, &uart->uart_tx);
}

DEBUG_UART_FUNCS

#endif
