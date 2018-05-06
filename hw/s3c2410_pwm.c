#include "s3c.h"
#include "hw.h"
#include "qemu-timer.h"

/* PWM timers controller */
struct s3c_timer_state_s;
struct s3c_timers_state_s {
	struct s3c_freq_s * freq;
    target_phys_addr_t base;
    qemu_irq *dma;
    DisplayState *ds;
    struct s3c_timer_state_s {
        QEMUTimer *t;
        struct s3c_timers_state_s *s;
        int n;
        int running;
        uint32_t divider;
        uint16_t count;
        int64_t reload;
        qemu_irq irq;
        gpio_handler_t cmp_cb;
        void *cmp_opaque;
    } timer[5];

    uint16_t compareb[4];
    uint16_t countb[5];
    uint32_t config[2];
    uint32_t control;
};

static const int s3c_tm_bits[] = { 0, 8, 12, 16, 20 };

static uint16_t s3c_timers_get(struct s3c_timers_state_s *s, int tm)
{
    uint16_t elapsed;
    if (!s->timer[tm].running)
        return s->timer[tm].count;

    elapsed = muldiv64(qemu_get_clock(vm_clock) - s->timer[tm].reload,
                    s->timer[tm].divider, ticks_per_sec);
    if (unlikely(elapsed > s->timer[tm].count))
        return s->timer[tm].count;

    return s->timer[tm].count - elapsed;
}

static void s3c_timers_stop(struct s3c_timers_state_s *s, int tm)
{
    s->timer[tm].count = s3c_timers_get(s, tm);
    s->timer[tm].running = 0;
}

static void s3c_timers_start(struct s3c_timers_state_s *s, int tm)
{
    if (s->timer[tm].running)
        return;

    s->timer[tm].divider = s->freq->pclk >>
            (((s->config[1] >> (tm * 4)) & 3) + 1);
    if (tm < 2)
        s->timer[tm].divider /= ((s->config[0] >> 0) & 0xff) + 1;
    else
        s->timer[tm].divider /= ((s->config[0] >> 8) & 0xff) + 1;
    s->timer[tm].running = 1;
    s->timer[tm].reload = qemu_get_clock(vm_clock);
    qemu_mod_timer(s->timer[tm].t,
                    s->timer[tm].reload + muldiv64(s->timer[tm].count,
                            ticks_per_sec, s->timer[tm].divider));
}

void s3c_timers_reset(struct s3c_timers_state_s *s)
{
    int i;
    s->config[0] = 0x00000000;
    s->config[1] = 0x00000000;
    s->control = 0x00000000;

    for (i = 0; i < 5; i ++) {
        if (s->timer[i].running)
            s3c_timers_stop(s, i);
        s->countb[i] = 0x0000;
        s->timer[i].count = 0;
    }
    for (i = 0; i < 4; i ++)
        s->compareb[i] = 0x0000;
}

static void s3c_timers_tick(void *opaque)
{
    struct s3c_timer_state_s *t = (struct s3c_timer_state_s *) opaque;
    struct s3c_timers_state_s *s = t->s;
    if (!t->running)
        return;

    if (((s->config[1] >> 20) & 0xf) == t->n + 1) {
        qemu_irq_raise(s->dma[S3C_RQ_TIMER0]);	/* TODO */
        qemu_irq_raise(s->dma[S3C_RQ_TIMER1]);
        qemu_irq_raise(s->dma[S3C_RQ_TIMER2]);
    } else
        qemu_irq_raise(t->irq);

    t->running = 0;
    t->count = 0;

    if (s->control & (1 << ((t->n == 4) ? 22 : (s3c_tm_bits[t->n] + 3)))) {
        /* Auto-reload */
        t->count = s->countb[t->n];
        s3c_timers_start(s, t->n);
    } else
        s->control &= ~(1 << s3c_tm_bits[t->n]);
}

#define S3C_TCFG0	0x00	/* Timer Configuration register 0 */
#define S3C_TCFG1	0x04	/* Timer Configuration register 1 */
#define S3C_TCON	0x08	/* Timer Control register */
#define S3C_TCNTB0	0x0c	/* Timer 0 Count Buffer register */
#define S3C_TCMPB0	0x10	/* Timer 0 Compare Buffer register */
#define S3C_TCNTO0	0x14	/* Timer 0 Count Observation register */
#define S3C_TCNTB1	0x18	/* Timer 1 Count Buffer register */
#define S3C_TCMPB1	0x1c	/* Timer 1 Compare Buffer register */
#define S3C_TCNTO1	0x20	/* Timer 1 Count Observation register */
#define S3C_TCNTB2	0x24	/* Timer 2 Count Buffer register */
#define S3C_TCMPB2	0x28	/* Timer 2 Compare Buffer register */
#define S3C_TCNTO2	0x2c	/* Timer 2 Count Observation register */
#define S3C_TCNTB3	0x30	/* Timer 3 Count Buffer register */
#define S3C_TCMPB3	0x34	/* Timer 3 Compare Buffer register */
#define S3C_TCNTO3	0x38	/* Timer 3 Count Observation register */
#define S3C_TCNTB4	0x3c	/* Timer 4 Count Buffer register */
#define S3C_TCNTO4	0x40	/* Timer 4 Count Observation register */

static uint32_t s3c_timers_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c_timers_state_s *s = (struct s3c_timers_state_s *) opaque;
    int tm = 0;

    switch (addr) {
    case S3C_TCFG0:
        return s->config[0];
    case S3C_TCFG1:
        return s->config[1];
    case S3C_TCON:
        return s->control;
    case S3C_TCMPB3:    tm ++;
    case S3C_TCMPB2:    tm ++;
    case S3C_TCMPB1:    tm ++;
    case S3C_TCMPB0:
        return s->compareb[tm];
    case S3C_TCNTB4:    tm ++;
    case S3C_TCNTB3:    tm ++;
    case S3C_TCNTB2:    tm ++;
    case S3C_TCNTB1:    tm ++;
    case S3C_TCNTB0:
        return s->countb[tm];
    case S3C_TCNTO4:    tm ++;
    case S3C_TCNTO3:    tm ++;
    case S3C_TCNTO2:    tm ++;
    case S3C_TCNTO1:    tm ++;
    case S3C_TCNTO0:
        return s3c_timers_get(s, tm);
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

static void s3c_timers_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c_timers_state_s *s = (struct s3c_timers_state_s *) opaque;
    int tm = 0;

    switch (addr) {
    case S3C_TCFG0:
        s->config[0] = value & 0x00ffffff;
        break;
    case S3C_TCFG1:
        s->config[1] = value & 0x00ffffff;
        break;
    case S3C_TCON:
        for (tm = 0; tm < 5; tm ++) {
            if (value & (2 << (s3c_tm_bits[tm]))) {
                if (s->timer[tm].running) {
                    s3c_timers_stop(s, tm);
                    s->timer[tm].count = s->countb[tm];
                    s3c_timers_start(s, tm);
                } else
                    s->timer[tm].count = s->countb[tm];
            }
            if (((value >> s3c_tm_bits[tm]) & 1) ^ s->timer[tm].running) {
                if (s->timer[tm].running)
                    s3c_timers_stop(s, tm);
                else
                    s3c_timers_start(s, tm);
            }
        }

        s->control = value & 0x007fff1f;
        break;
    case S3C_TCMPB3:    tm ++;
    case S3C_TCMPB2:    tm ++;
    case S3C_TCMPB1:    tm ++;
    case S3C_TCMPB0:
        s->compareb[tm] = value & 0xffff;
        if (s->timer[tm].cmp_cb)
            s->timer[tm].cmp_cb(tm, s->compareb[tm], s->timer[tm].cmp_opaque);
        break;
    case S3C_TCNTB4:    tm ++;
    case S3C_TCNTB3:    tm ++;
    case S3C_TCNTB2:    tm ++;
    case S3C_TCNTB1:    tm ++;
    case S3C_TCNTB0:
        s->countb[tm] = value & 0xffff;
        break;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
    }
}

static CPUReadMemoryFunc *s3c_timers_readfn[] = {
    s3c_timers_read,
    s3c_timers_read,
    s3c_timers_read,
};

static CPUWriteMemoryFunc *s3c_timers_writefn[] = {
    s3c_timers_write,
    s3c_timers_write,
    s3c_timers_write,
};

static void s3c_timers_save(QEMUFile *f, void *opaque)
{
    struct s3c_timers_state_s *s = (struct s3c_timers_state_s *) opaque;
    int i;
    for (i = 0; i < 5; i ++) {
        qemu_put_be32(f, s->timer[i].running);
        qemu_put_be32s(f, &s->timer[i].divider);
        qemu_put_be16(f, s3c_timers_get(s, i));
        qemu_put_sbe64s(f, &s->timer[i].reload);
    }

    for (i = 0; i < 4; i ++)
        qemu_put_be16s(f, &s->compareb[i]);
    for (i = 0; i < 5; i ++)
        qemu_put_be16s(f, &s->countb[i]);
    for (i = 0; i < 2; i ++)
        qemu_put_be32s(f, &s->config[i]);
    qemu_put_be32s(f, &s->control);
}

static int s3c_timers_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c_timers_state_s *s = (struct s3c_timers_state_s *) opaque;
    int i, running[5];
    for (i = 0; i < 5; i ++) {
        s->timer[i].running = 0;
        running[i] = qemu_get_be32(f);
        qemu_get_be32s(f, &s->timer[i].divider);
        qemu_get_be16s(f, &s->timer[i].count);
        qemu_get_sbe64s(f, &s->timer[i].reload);
    }

    for (i = 0; i < 4; i ++)
        qemu_get_be16s(f, &s->compareb[i]);
    for (i = 0; i < 5; i ++)
        qemu_get_be16s(f, &s->countb[i]);
    for (i = 0; i < 2; i ++)
        qemu_get_be32s(f, &s->config[i]);
    qemu_get_be32s(f, &s->control);

    for (i = 0; i < 5; i ++)
        if (running[i])
            s3c_timers_start(s, i);

    return 0;
}

struct s3c_timers_state_s *s3c_timers_init(struct s3c_freq_s * freq, target_phys_addr_t base,
                qemu_irq *pic, qemu_irq *dma)
{
    int i, iomemtype;
    struct s3c_timers_state_s *s = (struct s3c_timers_state_s *)
            qemu_mallocz(sizeof(struct s3c_timers_state_s));

    s->freq = freq;
    s->base = base;
    s->dma = dma;

    s3c_timers_reset(s);

    for (i = 0; i < 5; i ++) {
        s->timer[i].t = qemu_new_timer(vm_clock,
                        s3c_timers_tick, &s->timer[i]);
        s->timer[i].s = s;
        s->timer[i].n = i;
        s->timer[i].cmp_cb = 0;
        s->timer[i].irq = pic[i];
    }

    iomemtype = cpu_register_io_memory(0, s3c_timers_readfn,
                    s3c_timers_writefn, s);
    cpu_register_physical_memory(s->base, 0xffffff, iomemtype);

    register_savevm("s3c24xx_timers", 0, 0,
                    s3c_timers_save, s3c_timers_load, s);

    return s;
}

void s3c_timers_cmp_handler_set(void *opaque, int line,
                gpio_handler_t handler, void *cmp_opaque)
{
    struct s3c_timers_state_s *s = (struct s3c_timers_state_s *) opaque;
    if (line > 4 || line < 0) {
        printf("%s: Bad timer number %i.\n", __FUNCTION__, line);
        exit(-1);
    }
    s->timer[line].cmp_cb = handler;
    s->timer[line].cmp_opaque = cmp_opaque;
}
