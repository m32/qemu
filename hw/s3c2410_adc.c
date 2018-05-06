#include "s3c.h"
#include "hw.h"

/* ADC & Touchscreen interface */
struct s3c_adc_state_s {
    target_phys_addr_t base;
    qemu_irq irq;
    qemu_irq tcirq;
    int down;
    int scale[6];

    uint16_t control;
    uint16_t ts;
    uint16_t delay;
    int16_t xdata;
    int16_t ydata;
};

void s3c_adc_reset(struct s3c_adc_state_s *s)
{
    s->down = 0;
    s->control = 0x3fc4;
    s->ts = 0x58;
    s->delay = 0xff;
}

static void s3c_adc_event(void *opaque,
                int x, int y, int z, int buttons_state)
{
    struct s3c_adc_state_s *s = (struct s3c_adc_state_s *) opaque;
    int sx = x * s->scale[0] + y * s->scale[1] + s->scale[2];
    int sy = x * s->scale[3] + y * s->scale[4] + s->scale[5];
    s->down = !!buttons_state;
    s->xdata = 1 | ((!s->down) << 15) | ((sx >> 13) & 0xfff) | (1 << 14) | ((s->ts & 3) << 12);
    s->ydata = 1 | ((!s->down) << 15) | ((sy >> 13) & 0xfff) | (1 << 14) | ((s->ts & 3) << 12);
    qemu_irq_raise(s->tcirq);
//    qemu_irq_raise(s->irq);
}

#define S3C_ADCCON	0x00	/* ADC Control register */
#define S3C_ADCTSC	0x04	/* ADC Touchscreen Control register */
#define S3C_ADCDLY	0x08	/* ADC Start or Interval Delay register */
#define S3C_ADCDAT0	0x0c	/* ADC Conversion Data register 0 */
#define S3C_ADCDAT1	0x10	/* ADC Conversion Data register 1 */

static uint32_t s3c_adc_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c_adc_state_s *s = (struct s3c_adc_state_s *) opaque;

    switch (addr) {
    case S3C_ADCCON:
        return s->control;
    case S3C_ADCTSC:
        return s->ts;
    case S3C_ADCDLY:
        return s->delay;
    case S3C_ADCDAT0:
        return s->xdata;
    case S3C_ADCDAT1:
        return s->ydata;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

static void s3c_adc_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c_adc_state_s *s = (struct s3c_adc_state_s *) opaque;
    switch (addr) {
    case S3C_ADCCON:
        s->control = (s->control & 0x8000) | (value & 0x7ffe);
        if (value & 1) {
            s->control |= 1 << 15;
        }
        break;

    case S3C_ADCTSC:
        s->ts = value & 0xff;
        break;

    case S3C_ADCDLY:
        s->delay = value & 0xffff;
        break;

    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
    }
}

static CPUReadMemoryFunc *s3c_adc_readfn[] = {
    s3c_adc_read,
    s3c_adc_read,
    s3c_adc_read,
};

static CPUWriteMemoryFunc *s3c_adc_writefn[] = {
    s3c_adc_write,
    s3c_adc_write,
    s3c_adc_write,
};

static void s3c_adc_save(QEMUFile *f, void *opaque)
{
    struct s3c_adc_state_s *s = (struct s3c_adc_state_s *) opaque;

    qemu_put_be16s(f, &s->control);
    qemu_put_be16s(f, &s->ts);
    qemu_put_be16s(f, &s->delay);
    qemu_put_sbe16s(f, &s->xdata);
    qemu_put_sbe16s(f, &s->ydata);
}

static int s3c_adc_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c_adc_state_s *s = (struct s3c_adc_state_s *) opaque;

    qemu_get_be16s(f, &s->control);
    qemu_get_be16s(f, &s->ts);
    qemu_get_be16s(f, &s->delay);
    qemu_get_sbe16s(f, &s->xdata);
    qemu_get_sbe16s(f, &s->ydata);

    return 0;
}

struct s3c_adc_state_s *s3c_adc_init(target_phys_addr_t base, qemu_irq irq,
                qemu_irq tcirq)
{
    int iomemtype;
    struct s3c_adc_state_s *s = (struct s3c_adc_state_s *)
            qemu_mallocz(sizeof(struct s3c_adc_state_s));

    s->base = base;
    s->irq = irq;
    s->tcirq = tcirq;

    s3c_adc_reset(s);

    iomemtype = cpu_register_io_memory(0, s3c_adc_readfn,
                    s3c_adc_writefn, s);
    cpu_register_physical_memory(s->base, 0xffffff, iomemtype);

    /* We want absolute coordinates */
    qemu_add_mouse_event_handler(s3c_adc_event, s, 1,
                    "QEMU S3C2410-driven Touchscreen");

    register_savevm("s3c24xx_adc", 0, 0, s3c_adc_save, s3c_adc_load, s);

    return s;
}

void s3c_adc_setscale(struct s3c_adc_state_s *adc, const int m[])
{
    memcpy(adc->scale, m, 6 * sizeof(int));
}

