#include "s3c.h"
#include "sysbus.h"

/* DMA controller */
#define S3C_DMA_CH_N	4

struct s3c_dma_ch_state_s;
struct s3c_dma_state_s {	/* Modelled as an interrupt controller */
    target_phys_addr_t base;
    qemu_irq *drqs;
    struct s3c_dma_ch_state_s {
        qemu_irq intr;
        int curr_tc;
        int req;
        int running;
        uint32_t con;
        uint32_t isrc;
        uint32_t isrcc;
        uint32_t idst;
        uint32_t idstc;
        uint32_t csrc;
        uint32_t cdst;
        uint32_t mask;
    } ch[S3C_DMA_CH_N];
};

static inline void s3c_dma_ch_run(struct s3c_dma_state_s *s,
                struct s3c_dma_ch_state_s *ch)
{
    int width, burst, t;
    uint8_t buffer[4];
    width = 1 << ((ch->con >> 20) & 3);				/* DSZ */
    burst = (ch->con & (1 << 28)) ? 4 : 1;			/* TSZ */

    while (!ch->running && ch->curr_tc > 0 && ch->req &&
                    (ch->mask & (1 << 1))) {		/* ON_OFF */
        if (width > sizeof(buffer)) {
            printf("%s: wrong access width\n", __FUNCTION__);
            return;
        }
        ch->running = 1;
        while (ch->curr_tc --) {
            for (t = 0; t < burst; t ++) {
                cpu_physical_memory_read(ch->csrc, buffer, width);
                cpu_physical_memory_write(ch->cdst, buffer, width);

                if (!(ch->isrcc & 1))				/* INT */
                    ch->csrc += width;
                if (!(ch->idstc & 1))				/* INT */
                    ch->cdst += width;
            }

            if (!(ch->con & (1 << 27)) && !ch->req)		/* SERVMODE */
                break;
        }
        ch->running = 0;

        if (!(ch->con & (1 << 23))) {				/* SWHW_SEL */
            ch->req = 0;
        }

        if (ch->curr_tc <= 0) {
            if (ch->con & (1 << 22))				/* RELOAD */
                ch->mask &= ~(1 << 1);				/* ON_OFF */
            else {
                if (!(ch->con & (1 << 23))) {			/* SWHW_SEL */
                    printf("%s: auto-reload software controlled transfer\n",
                                    __FUNCTION__);
                    break;
                }
                ch->csrc = ch->isrc;				/* S_ADDR */
                ch->cdst = ch->idst;				/* D_ADDR */
                ch->curr_tc = ch->con & 0xfffff;		/* TC */
                ch->con |= 1 << 22;				/* ON_OFF */
            }

            if (ch->con & (1 << 31))				/* DMD_HS */
                ch->req = 0;

            if (ch->con & (1 << 29)) {				/* INT */
                qemu_irq_raise(ch->intr);
                /* Give the system a chance to respond.  */
                break;
            }
        }
    }
}

void s3c_dma_reset(struct s3c_dma_state_s *s)
{
    int i;
    for (i = 0; i < S3C_DMA_CH_N; i ++) {
        s->ch[i].curr_tc = 0;
        s->ch[i].csrc = 0;
        s->ch[i].isrc = 0;
        s->ch[i].isrcc = 0;
        s->ch[i].cdst = 0;
        s->ch[i].idst = 0;
        s->ch[i].idstc = 0;
        s->ch[i].con = 0;
        s->ch[i].csrc = 0;
        s->ch[i].cdst = 0;
        s->ch[i].mask = 0;
    }
}

static void s3c_dma_dreq(void *opaque, int line, int req)
{
    struct s3c_dma_state_s *s = (struct s3c_dma_state_s *) opaque;
    struct s3c_dma_ch_state_s *ch = &s->ch[line >> 4];

    if (ch->con & (1 << 23))					/* SWHW_SEL */
        if (((ch->con >> 24) & 7) == (line & 7)) {		/* HWSRCSEL */
            ch->req = req;
            s3c_dma_ch_run(s, ch);
        }
}

#define S3C_DISRC	0x00	/* DMA Initial Source register */
#define S3C_DISRCC	0x04	/* DMA Initial Source Control register */
#define S3C_DIDST	0x08	/* DMA Initial Destination register */
#define S3C_DIDSTC	0x0c	/* DMA Initial Destination Control register */
#define S3C_DCON	0x10	/* DMA Control register */
#define S3C_DSTAT	0x14	/* DMA Count register */
#define S3C_DCSRC	0x18	/* DMA Current Source register */
#define S3C_DCDST	0x1c	/* DMA Current Destination register */
#define S3C_DMASKTRIG	0x20	/* DMA Mask Trigger register */

static uint32_t s3c_dma_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c_dma_state_s *s = (struct s3c_dma_state_s *) opaque;
    struct s3c_dma_ch_state_s *ch = 0;

    if (addr >= 0 && addr <= (S3C_DMA_CH_N << 6)) {
        ch = &s->ch[addr >> 6];
        addr &= 0x3f;
    }

    switch (addr) {
    case S3C_DISRC:
        return ch->isrc;
    case S3C_DISRCC:
        return ch->isrcc;
    case S3C_DIDST:
        return ch->idst;
    case S3C_DIDSTC:
        return ch->idstc;
    case S3C_DCON:
        return ch->con;
    case S3C_DSTAT:
        return ch->curr_tc;
    case S3C_DCSRC:
        return ch->csrc;
    case S3C_DCDST:
        return ch->cdst;
    case S3C_DMASKTRIG:
        return ch->mask;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

static void s3c_dma_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c_dma_state_s *s = (struct s3c_dma_state_s *) opaque;
    struct s3c_dma_ch_state_s *ch = 0;

    if (addr >= 0 && addr <= (S3C_DMA_CH_N << 6)) {
        ch = &s->ch[addr >> 6];
        addr &= 0x3f;
    }

    switch (addr) {
    case S3C_DCON:
        ch->con = value;
        break;
    case S3C_DISRC:
        ch->isrc = value;
        break;
    case S3C_DISRCC:
        ch->isrcc = value;
        break;
    case S3C_DIDST:
        ch->idst = value;
        break;
    case S3C_DIDSTC:
        ch->idstc = value;
        break;
    case S3C_DMASKTRIG:
        if (~ch->mask & value & (1 << 1)) {			/* ON_OFF */
            ch->curr_tc = ch->con & 0xfffff;			/* TC */
            ch->csrc = ch->isrc;				/* S_ADDR */
            ch->cdst = ch->idst;				/* D_ADDR */
        }

        ch->mask = value;
        if (value & (1 << 2)) {					/* STOP */
            ch->mask &= ~(3 << 1);				/* ON_OFF */
        } else if (!(ch->con & (1 << 23))) {			/* SWHW_SEL */
            ch->req = value & 1;				/* SW_TRIG */
            s3c_dma_ch_run(s, ch);
        }
        break;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
    }
}

static CPUReadMemoryFunc *s3c_dma_readfn[] = {
    s3c_dma_read,
    s3c_dma_read,
    s3c_dma_read,
};

static CPUWriteMemoryFunc *s3c_dma_writefn[] = {
    s3c_dma_write,
    s3c_dma_write,
    s3c_dma_write,
};

static void s3c_dma_save(QEMUFile *f, void *opaque)
{
    struct s3c_dma_state_s *s = (struct s3c_dma_state_s *) opaque;
    int i;
    for (i = 0; i < S3C_DMA_CH_N; i ++) {
        qemu_put_be32(f, s->ch[i].curr_tc);
        qemu_put_be32(f, s->ch[i].req);
        qemu_put_be32s(f, &s->ch[i].con);
        qemu_put_be32s(f, &s->ch[i].isrc);
        qemu_put_be32s(f, &s->ch[i].isrcc);
        qemu_put_be32s(f, &s->ch[i].idst);
        qemu_put_be32s(f, &s->ch[i].idstc);
        qemu_put_be32s(f, &s->ch[i].csrc);
        qemu_put_be32s(f, &s->ch[i].cdst);
        qemu_put_be32s(f, &s->ch[i].mask);
    }
}

static int s3c_dma_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c_dma_state_s *s = (struct s3c_dma_state_s *) opaque;
    int i;
    for (i = 0; i < S3C_DMA_CH_N; i ++) {
        s->ch[i].curr_tc = qemu_get_be32(f);
        s->ch[i].req = qemu_get_be32(f);
        qemu_get_be32s(f, &s->ch[i].con);
        qemu_get_be32s(f, &s->ch[i].isrc);
        qemu_get_be32s(f, &s->ch[i].isrcc);
        qemu_get_be32s(f, &s->ch[i].idst);
        qemu_get_be32s(f, &s->ch[i].idstc);
        qemu_get_be32s(f, &s->ch[i].csrc);
        qemu_get_be32s(f, &s->ch[i].cdst);
        qemu_get_be32s(f, &s->ch[i].mask);
    }
    return 0;
}

struct s3c_dma_state_s *s3c_dma_init(target_phys_addr_t base, qemu_irq *pic)
{
    int iomemtype;
    struct s3c_dma_state_s *s = (struct s3c_dma_state_s *)
            qemu_mallocz(sizeof(struct s3c_dma_state_s));

    s->base = base;
    s->ch[0].intr = pic[0];
    s->ch[1].intr = pic[1];
    s->ch[2].intr = pic[2];
    s->ch[3].intr = pic[3];
    s->drqs = qemu_allocate_irqs(s3c_dma_dreq, s, S3C_RQ_MAX);

    s3c_dma_reset(s);

    iomemtype = cpu_register_io_memory(0, s3c_dma_readfn,
                    s3c_dma_writefn, s);
    cpu_register_physical_memory(s->base, 0xffffff, iomemtype);

    register_savevm("s3c24xx_dma", 0, 0, s3c_dma_save, s3c_dma_load, s);

    return s;
}

qemu_irq *s3c_dma_get(struct s3c_dma_state_s *s)
{
    return s->drqs;
}

