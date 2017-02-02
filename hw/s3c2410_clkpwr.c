#include "s3c.h"
#include "sysbus.h"

/* Clock & power management */
#define S3C_LOCKTIME	0x00	/* PLL Lock Time Count register */
#define S3C_MPLLCON	0x04	/* MPLL Configuration register */
#define S3C_UPLLCON	0x08	/* UPLL Configuration register */
#define S3C_CLKCON	0x0c	/* Clock Generator Control register */
#define S3C_CLKSLOW	0x10	/* Slow Clock Control register */
#define S3C_CLKDIVN	0x14	/* Clock Divider Control register */

#define S3C2440_CAMDIVN	0x18	/* Camera Clock Divider register */

static void s3c_clkpwr_update(struct s3c_state_s *s)
{
	uint32_t mpll[2] = { s->clkpwr_regs[S3C_MPLLCON >> 2], s->clkpwr_regs[S3C_UPLLCON >> 2] };
	uint32_t clk[2];
	int i;

	for (i = 0; i < 2; i++) {
		uint32_t mdiv = ((mpll[i] >> 12) & 0xff) + 8,
				pdiv = ((mpll[i] >> 4) & 0x3f) + 2,
				sdiv = (mpll[i]) & 0x3;
		clk[i] = (mdiv * s->clock.xtal * 2) / (pdiv * (1 << sdiv));
	}

	s->clock.clk = clk[0];
	uint32_t ratio = s->clkpwr_regs[S3C_CLKDIVN >> 2];

	switch( (ratio & 0x6) >> 1 ) {
		case 0:
			s->clock.hclk = s->clock.clk;
			break;
		case 1:
			s->clock.hclk = s->clock.clk/2;
			break;
		case 2:
			s->clock.hclk = s->clock.clk/4;
			break;
		case 3:
			s->clock.hclk = s->clock.clk/3;
			break;
	}
	switch ( ratio&0x1) {
		case 0:
			s->clock.pclk = s->clock.hclk;
			break;
		case 1:
			s->clock.pclk = s->clock.hclk/2;
			break;
	}
	s->clock.uclk = clk[1] / 2;
	#define MHZ	1000000
	printf("S3C: CLK=%d HCLK=%d PCLK=%d UCLK=%d\n",
				s->clock.clk/MHZ, s->clock.hclk/MHZ, s->clock.pclk/MHZ,
				s->clock.uclk/MHZ);
}

void s3c_clkpwr_reset(struct s3c_state_s *s)
{
    s->clkpwr_regs[S3C_LOCKTIME >> 2] = 0x00ffffff;
    s->clkpwr_regs[S3C_MPLLCON >> 2] = 0x0005c080;
    s->clkpwr_regs[S3C_UPLLCON >> 2] = 0x00028080;
    s->clkpwr_regs[S3C_CLKCON >> 2] = 0x0007fff0;
    s->clkpwr_regs[S3C_CLKSLOW >> 2] = 0x00000004;
    s->clkpwr_regs[S3C_CLKDIVN >> 2] = 0x00000000;
    s->clkpwr_regs[S3C2440_CAMDIVN >> 2] = 0x00000000;
    s3c_clkpwr_update(s);
}

static uint32_t s3c_clkpwr_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c_state_s *s = (struct s3c_state_s *) opaque;

    switch (addr) {
    case S3C_LOCKTIME ... S3C_CLKDIVN:
        return s->clkpwr_regs[addr >> 2];
    case S3C2440_CAMDIVN:
    	if (s->cpu_id == S3C_CPU_2440)
    		return s->clkpwr_regs[addr >> 2];
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

static void s3c_clkpwr_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c_state_s *s = (struct s3c_state_s *) opaque;

    switch (addr) {
    case S3C_LOCKTIME:
    case S3C_MPLLCON:
    case S3C_UPLLCON:
    case S3C_CLKDIVN:
        s->clkpwr_regs[addr >> 2] = value;
        if (addr != S3C_LOCKTIME)
        	s3c_clkpwr_update(s);
        break;
    case S3C_CLKCON:
        if (value & (1 << 3)) {
            cpu_interrupt(s->env, CPU_INTERRUPT_HALT);
            printf("%s: processor powered off\n", __FUNCTION__);
            s3c_gpio_setpwrstat(s->io, 2);
#if 0
            cpu_reset(s->env);
            s->env->regs[15] = 0;	/* XXX */
#endif
        } else
            if (value & (1 << 2))	/* Normal IDLE mode */
                cpu_interrupt(s->env, CPU_INTERRUPT_HALT);
        if ((s->clkpwr_regs[addr >> 2] ^ value) & 1)
            printf("%s: SPECIAL mode %s\n", __FUNCTION__,
                            (value & 1) ? "on" : "off");
        s->clkpwr_regs[addr >> 2] = value;
        break;
    case S3C_CLKSLOW:
        if ((s->clkpwr_regs[addr >> 2] ^ value) & (1 << 4))
            printf("%s: SLOW mode %s\n", __FUNCTION__,
                            (value & (1 << 4)) ? "on" : "off");
        s->clkpwr_regs[addr >> 2] = value;
        break;
    case S3C2440_CAMDIVN:
    	if (s->cpu_id == S3C_CPU_2440) {
    		s->clkpwr_regs[addr >> 2] = value;
    		break;
    	}
    default:
        printf("%s: Bad register 0x%x (cpu %08x)\n", __FUNCTION__, /*(unsigned long)*/addr, s->cpu_id);
    }
}

static CPUReadMemoryFunc *s3c_clkpwr_readfn[] = {
    s3c_clkpwr_read,
    s3c_clkpwr_read,
    s3c_clkpwr_read,
};

static CPUWriteMemoryFunc *s3c_clkpwr_writefn[] = {
    s3c_clkpwr_write,
    s3c_clkpwr_write,
    s3c_clkpwr_write,
};

static void s3c_clkpwr_save(QEMUFile *f, void *opaque)
{
    struct s3c_state_s *s = (struct s3c_state_s *) opaque;
    int i;
    for (i = 0; i < 7; i ++)
        qemu_put_be32s(f, &s->clkpwr_regs[i]);
}

static int s3c_clkpwr_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c_state_s *s = (struct s3c_state_s *) opaque;
    int i;
    for (i = 0; i < 7; i ++)
        qemu_get_be32s(f, &s->clkpwr_regs[i]);
    return 0;
}

void s3c_clkpwr_init(struct s3c_state_s *s, target_phys_addr_t base)
{
    int iomemtype;
    s->clkpwr_base = base;
    s3c_clkpwr_reset(s);

    iomemtype = cpu_register_io_memory(0, s3c_clkpwr_readfn,
                    s3c_clkpwr_writefn, s);
    cpu_register_physical_memory(s->clkpwr_base, 0xffffff, iomemtype);
    register_savevm("s3c24xx_clkpwr", 0, 0,
                    s3c_clkpwr_save, s3c_clkpwr_load, s);
}