#if 1
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

#define RX_FIFO_COUNT_SHIFT	0
#define RX_FIFO_COUNT_MASK	(0xff << RX_FIFO_COUNT_SHIFT)
#define RX_FIFO_FULL		(1 << 8)
#define TX_FIFO_COUNT_SHIFT	16
#define TX_FIFO_COUNT_MASK	(0xff << TX_FIFO_COUNT_SHIFT)
#define TX_FIFO_FULL		(1 << 24)

/* Information about a serial port */
struct uis8910_serial_platdata {
	struct uis8910_uart *reg;  /* address of registers in physical memory */
	u8 port_id;     /* uart port number */
};

/*
 * The coefficient, used to calculate the baudrate on S5P UARTs is
 * calculated as
 * C = UBRDIV * 16 + number_of_set_bits_in_UDIVSLOT
 * however, section 31.6.11 of the datasheet doesn't recomment using 1 for 1,
 * 3 for 2, ... (2^n - 1) for n, instead, they suggest using these constants:
 */
static const int udivslot[] = {
	0,
	0x0080,
	0x0808,
	0x0888,
	0x2222,
	0x4924,
	0x4a52,
	0x54aa,
	0x5555,
	0xd555,
	0xd5d5,
	0xddd5,
	0xdddd,
	0xdfdd,
	0xdfdf,
	0xffdf,
};

static void __maybe_unused uis8910_serial_init(struct uis8910_uart *uart)
{
	/* enable FIFOs, auto clear Rx FIFO */
	writel(0x3, &uart->ufcon);
	writel(0, &uart->umcon);
	/* 8N1 */
	writel(0x3, &uart->ulcon);
	/* No interrupts, no DMA, pure polling */
	writel(0x245, &uart->ucon);
}

static void __maybe_unused uis8910_serial_baud(struct uis8910_uart *uart, uint uclk,
					   int baudrate)
{
	u32 val;

	val = uclk / baudrate;

	writel(val / 16 - 1, &uart->ubrdiv);

	if (uis8910_uart_divslot())
		writew(udivslot[val % 16], &uart->rest.slot);
	else
		writeb(val % 16, &uart->rest.value);
}

#ifndef CONFIG_SPL_BUILD
int uis8910_serial_setbrg(struct udevice *dev, int baudrate)
{
	struct uis8910_serial_platdata *plat = dev->platdata;
	struct uis8910_uart *const uart = plat->reg;
	u32 uclk;
	uclk = 24000000;

	uis8910_serial_baud(uart, uclk, baudrate);

	return 0;
}

static int uis8910_serial_probe(struct udevice *dev)
{
	struct uis8910_serial_platdata *plat = dev->platdata;
	struct uis8910_uart *const uart = plat->reg;

	uis8910_serial_init(uart);

	return 0;
}

static int serial_err_check(const struct uis8910_uart *const uart, int op)
{
	unsigned int mask;

	/*
	 * UERSTAT
	 * Break Detect	[3]
	 * Frame Err	[2] : receive operation
	 * Parity Err	[1] : receive operation
	 * Overrun Err	[0] : receive operation
	 */
	if (op)
		mask = 0x8;
	else
		mask = 0xf;

	return readl(&uart->uerstat) & mask;
}

static int uis8910_serial_getc(struct udevice *dev)
{
	struct uis8910_serial_platdata *plat = dev->platdata;
	struct uis8910_uart *const uart = plat->reg;

	if (!(readl(&uart->ufstat) & RX_FIFO_COUNT_MASK))
		return -EAGAIN;

	serial_err_check(uart, 0);
	return (int)(readb(&uart->urxh) & 0xff);
}

static int uis8910_serial_putc(struct udevice *dev, const char ch)
{
	struct uis8910_serial_platdata *plat = dev->platdata;
	struct uis8910_uart *const uart = plat->reg;

	if (readl(&uart->ufstat) & TX_FIFO_FULL)
		return -EAGAIN;

	writeb(ch, &uart->utxh);
	serial_err_check(uart, 1);

	return 0;
}

static int uis8910_serial_pending(struct udevice *dev, bool input)
{
	struct uis8910_serial_platdata *plat = dev->platdata;
	struct uis8910_uart *const uart = plat->reg;
	uint32_t ufstat = readl(&uart->ufstat);

	if (input)
		return (ufstat & RX_FIFO_COUNT_MASK) >> RX_FIFO_COUNT_SHIFT;
	else
		return (ufstat & TX_FIFO_COUNT_MASK) >> TX_FIFO_COUNT_SHIFT;
}

static int uis8910_serial_ofdata_to_platdata(struct udevice *dev)
{
	struct uis8910_serial_platdata *plat = dev->platdata;
	fdt_addr_t addr;

	addr = devfdt_get_addr(dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	plat->reg = (struct uis8910_uart *)addr;
	plat->port_id = fdtdec_get_int(gd->fdt_blob, dev_of_offset(dev),
					"id", dev->seq);
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
	.ofdata_to_platdata = uis8910_serial_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct uis8910_serial_platdata),
	.probe = uis8910_serial_probe,
	.ops	= &uis8910_serial_ops,
};
#endif

#ifdef CONFIG_DEBUG_UART_S5P

#include <debug_uart.h>

static inline void _debug_uart_init(void)
{
	struct uis8910_uart *uart = (struct uis8910_uart *)CONFIG_DEBUG_UART_BASE;

	uis8910_serial_init(uart);
	uis8910_serial_baud(uart, CONFIG_DEBUG_UART_CLOCK, CONFIG_BAUDRATE);
}

static inline void _debug_uart_putc(int ch)
{
	struct uis8910_uart *uart = (struct uis8910_uart *)CONFIG_DEBUG_UART_BASE;

	while (readl(&uart->ufstat) & TX_FIFO_FULL);

	writeb(ch, &uart->utxh);
}

DEBUG_UART_FUNCS

#endif

#endif