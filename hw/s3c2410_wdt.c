#include "s3c.h"
#include "sysbus.h"
#include "sysemu.h"
#include "qemu-timer.h"

/* Watchdog Timer */
struct s3c_wdt_state_s {
	struct s3c_freq_s * freq;
    target_phys_addr_t base;
    qemu_irq irq;
    uint16_t control;
    uint16_t data;
    uint16_t count;
    QEMUTimer *tm;
    int64_t timestamp;
};

static void s3c_wdt_start(struct s3c_wdt_state_s *s)
{
    int enable = s->control & (1 << 5);
    int prescaler = (s->control >> 8) + 1;
    int divider = prescaler << (((s->control >> 3) & 3) + 4);
    if (enable) {
        s->timestamp = qemu_get_clock(vm_clock);
        qemu_mod_timer(s->tm, s->timestamp + muldiv64(divider * s->count,
                                ticks_per_sec, s->freq->pclk));
    } else
        qemu_del_timer(s->tm);
}

static void s3c_wdt_stop(struct s3c_wdt_state_s *s)
{
    int prescaler = (s->control >> 8) + 1;
    int divider = prescaler << (((s->control >> 3) & 3) + 4);
    int diff;

    diff = muldiv64(qemu_get_clock(vm_clock) - s->timestamp, s->freq->pclk,
                    ticks_per_sec) / divider;
    s->count -= MIN(s->count, diff);
    s->timestamp = qemu_get_clock(vm_clock);
}

void s3c_wdt_reset(struct s3c_wdt_state_s *s)
{
    s->control = 0x8021;
    s->data = 0x8000;
    s->count = 0x8000;
    s3c_wdt_start(s);
}

static void s3c_wdt_timeout(void *opaque)
{
    struct s3c_wdt_state_s *s = (struct s3c_wdt_state_s *) opaque;
    if (s->control & (1 << 0)) {
        qemu_system_reset_request();
        return;
    }
    if (s->control & (1 << 2))
        qemu_irq_raise(s->irq);
    s->count = s->data;
    s3c_wdt_start(s);
}

#define S3C_WTCON	0x00	/* Watchdog timer control register */
#define S3C_WTDAT	0x04	/* Watchdog timer data register */
#define S3C_WTCNT	0x08	/* Watchdog timer count register */

static uint32_t s3c_wdt_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c_wdt_state_s *s = (struct s3c_wdt_state_s *) opaque;

    switch (addr) {
    case S3C_WTCON:
        return s->control;
    case S3C_WTDAT:
        return s->data;
    case S3C_WTCNT:
        s3c_wdt_stop(s);
        return s->count;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

static void s3c_wdt_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c_wdt_state_s *s = (struct s3c_wdt_state_s *) opaque;

    switch (addr) {
    case S3C_WTCON:
        s3c_wdt_stop(s);
        s->control = value;
        s3c_wdt_start(s);
        break;
    case S3C_WTDAT:
        s->data = value;
        break;
    case S3C_WTCNT:
        s->count = value;
        s3c_wdt_start(s);
        break;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
    }
}

static CPUReadMemoryFunc *s3c_wdt_readfn[] = {
    s3c_wdt_read,
    s3c_wdt_read,
    s3c_wdt_read,
};

static CPUWriteMemoryFunc *s3c_wdt_writefn[] = {
    s3c_wdt_write,
    s3c_wdt_write,
    s3c_wdt_write,
};

static void s3c_wdt_save(QEMUFile *f, void *opaque)
{
    struct s3c_wdt_state_s *s = (struct s3c_wdt_state_s *) opaque;

    s3c_wdt_stop(s);
    qemu_put_be16s(f, &s->control);
    qemu_put_be16s(f, &s->data);
    qemu_put_be16s(f, &s->count);
    qemu_put_sbe64s(f, &s->timestamp);
}

static int s3c_wdt_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c_wdt_state_s *s = (struct s3c_wdt_state_s *) opaque;

    qemu_get_be16s(f, &s->control);
    qemu_get_be16s(f, &s->data);
    qemu_get_be16s(f, &s->count);
    qemu_get_sbe64s(f, &s->timestamp);
    s3c_wdt_start(s);

    return 0;
}

struct s3c_wdt_state_s *s3c_wdt_init(struct s3c_freq_s * freq, target_phys_addr_t base, qemu_irq irq)
{
    int iomemtype;
    struct s3c_wdt_state_s *s = (struct s3c_wdt_state_s *)
            qemu_mallocz(sizeof(struct s3c_wdt_state_s));

    s->freq = freq;
    s->base = base;
    s->irq = irq;
    s->tm = qemu_new_timer(vm_clock, s3c_wdt_timeout, s);

    s3c_wdt_reset(s);

    iomemtype = cpu_register_io_memory(0, s3c_wdt_readfn,
                    s3c_wdt_writefn, s);
    cpu_register_physical_memory(s->base, 0xffffff, iomemtype);

    register_savevm("s3c24xx_wdt", 0, 0, s3c_wdt_save, s3c_wdt_load, s);

    return s;
}

