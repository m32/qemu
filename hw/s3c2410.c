/*
 * Samsung S3C2410A RISC Microprocessor support (ARM920T based SoC).
 *
 * Copyright (c) 2007 OpenMoko, Inc.
 * Author: Andrzej Zaborowski <andrew@openedhand.com>
 * With:	Michel Pollet <buserror@gmail.com>
 *
 * This code is licenced under the GNU GPL v2.
 */

#include "s3c.h"
#include "qemu-timer.h"
#include "qemu-char.h"
#include "console.h"
#include "devices.h"
#include "arm-misc.h"
#include "i2c.h"
#include "pxa.h"
#include "sysemu.h"
/*
#include "s3c2410_adc.c"
#include "s3c2410_clkpwr.c"
#include "s3c2410_dma.c"
#include "s3c2410_i2c.c"
#include "s3c2410_i2s.c"
#include "s3c2410_mc.c"
#include "s3c2410_pic.c"
#include "s3c2410_pwm.c"
#include "s3c2410_spi.c"
#include "s3c2410_uart.c"
#include "s3c2410_wdt.c"
*/

/* On-chip UARTs */
static struct {
    target_phys_addr_t base;
    int irq[3];
    int dma[1];
} s3c2410_uart[] = {
    {
        0x50004000,
        { S3C_PICS_RXD1, S3C_PICS_TXD1, S3C_PICS_ERR1 },
        { S3C_RQ_UART1 },
    },
    {
        0x50000000,
        { S3C_PICS_RXD0, S3C_PICS_TXD0, S3C_PICS_ERR0 },
        { S3C_RQ_UART0 },
    },
    {
        0x50008000,
        { S3C_PICS_RXD2, S3C_PICS_TXD2, S3C_PICS_ERR2 },
        { S3C_RQ_UART2 },
    },
    { 0, { 0, 0, 0 }, { 0 } }
};

/* General CPU reset */
static void s3c2410_reset(void *opaque)
{
	fprintf(stderr, "%s:%d\n", __func__, __LINE__);
    struct s3c_state_s *s = (struct s3c_state_s *) opaque;
    int i;
    s3c_mc_reset(s->mc);
    s3c_pic_reset(s->pic);
    s3c_dma_reset(s->dma);
    s3c_gpio_reset(s->io);
    s3c_lcd_reset(s->lcd);
    s3c_timers_reset(s->timers);
    s3c_mmci_reset(s->mmci);
    s3c_adc_reset(s->adc);
    s3c_i2c_reset(s->i2c);
    s3c_i2s_reset(s->i2s);
    s3c_rtc_reset(s->rtc);
    s3c_spi_reset(s->spi);
    s3c_udc_reset(s->udc);
    s3c_wdt_reset(s->wdt);
    s3c_clkpwr_reset(s);
    s->nand->reset(s->nand);
    for (i = 0; s3c2410_uart[i].base; i ++)
        s3c_uart_reset(s->uart[i]);
    cpu_reset(s->env);
}

struct s3c_state_s * g_s3c;

/* Initialise an S3C24XX microprocessor.  */
struct s3c_state_s *s3c24xx_init(
		uint32_t cpu_id,
		uint32_t xtal,
		unsigned int sdram_size,
		uint32_t sram_address,
		SDState *mmc)
{
    struct s3c_state_s *s;
    int iomemtype, i;
    s = (struct s3c_state_s *) qemu_mallocz(sizeof(struct s3c_state_s));

    g_s3c = s;

    s->cpu_id = cpu_id;
    s->clock.xtal = xtal;
    s->clock.pclk = 66500000; // S3C_PCLK_FREQ;	// TEMP

    s->env = cpu_init("arm920t");
    if (!s->env) {
        fprintf(stderr, "Unable to initialize ARM920T\n");
        exit(2);
    }
    register_savevm("s3c24xx", 0, 0,
                    cpu_save, cpu_load, s->env);

	ram_addr_t s3c_sdram_phys = qemu_ram_alloc(sdram_size) | IO_MEM_RAM;

    cpu_register_physical_memory(S3C_RAM_BASE, sdram_size, s3c_sdram_phys);

	{//Devices specific to DeviceEmulator
		/* "cached" mirror of SDRAM banks for WinCE */
		cpu_register_physical_memory(0xa0000000, sdram_size, s3c_sdram_phys);

		/* DMA Transport */
		cpu_register_physical_memory(0x500f0000, 0x4000, qemu_ram_alloc(0x4000) | IO_MEM_RAM);
		/* Folder Sharing */
		cpu_register_physical_memory(0x500f4000, 0x1000, qemu_ram_alloc(0x1000) | IO_MEM_RAM);
		/* Emulator Server */
		cpu_register_physical_memory(0x500f5000, 0x1000, qemu_ram_alloc(0x1000) | IO_MEM_RAM);
	}

    /* If OM pins are 00, SRAM is mapped at 0x0 instead.  */
    cpu_register_physical_memory(sram_address, S3C_SRAM_SIZE, qemu_ram_alloc(S3C_SRAM_SIZE) | IO_MEM_RAM);

    s->mc = s3c_mc_init(0x48000000);
    s->pic = s3c_pic_init(0x4a000000, arm_pic_init_cpu(s->env));
    s->irq = s3c_pic_get(s->pic);
    s->dma = s3c_dma_init(0x4b000000, &s->irq[S3C_PIC_DMA0]);
    s->drq = s3c_dma_get(s->dma);

    s3c_clkpwr_init(s, 0x4c000000);

    s->lcd = s3c_lcd_init(0x4d000000, s->irq[S3C_PIC_LCD]);

    if (s->cpu_id == S3C_CPU_2440)
    	s->nand = s3c2440_nand_init(0x4e000000);
    else
    	s->nand = s3c2410_nand_init(0x4e000000);

    for (i = 0; s3c2410_uart[i].base; i ++) {
        s->uart[i] = s3c_uart_init(&s->clock,
						s3c2410_uart[i].base,
                        &s->irq[s3c2410_uart[i].irq[0]],
                        &s->drq[s3c2410_uart[i].dma[0]]);
        if (serial_hds[i])
            s3c_uart_attach(s->uart[i], serial_hds[i]);
    }

    s->timers = s3c_timers_init(&s->clock, 0x51000000, &s->irq[S3C_PIC_TIMER0], s->drq);
    s->udc = s3c_udc_init(0x52000000, s->irq[S3C_PIC_USBD], s->drq);
    s->wdt = s3c_wdt_init(&s->clock, 0x53000000, s->irq[S3C_PIC_WDT]);
    s->i2c = s3c_i2c_init1(s, 0x54000000);
    s->i2s = s3c_i2s_init(0x55000000, s->drq);
    s->io = s3c_gpio_init(0x56000000, s->irq, s->cpu_id);
    s->rtc = s3c_rtc_init(0x57000000, s->irq[S3C_PIC_RTC]);
    s->adc = s3c_adc_init(0x58000000, s->irq[S3C_PICS_ADC], s->irq[S3C_PICS_TC]);
    s->spi = s3c_spi_init(0x59000000,
                    s->irq[S3C_PIC_SPI0], s->drq[S3C_RQ_SPI0],
                    s->irq[S3C_PIC_SPI1], s->drq[S3C_RQ_SPI1], s->io);
    s->mmci = s3c_mmci_init(0x5a000000, s->cpu_id, mmc,
                    s->irq[S3C_PIC_SDI], s->drq);

    if (usb_enabled) {
        usb_ohci_init_pxa(0x49000000, 3, -1, s->irq[S3C_PIC_USBH]);
    }

    qemu_register_reset(s3c2410_reset, s);

    s->nand->setwp(s->nand, 1);

    /* Power on reset */
    s3c_gpio_setpwrstat(s->io, 1);
    return s;
}
