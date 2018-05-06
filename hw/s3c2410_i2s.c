#include "s3c.h"
#include "hw.h"

/* IIS-BUS interface */
static inline void s3c_i2s_update(struct s3c_i2s_state_s *s)
{
    s->tx_en =
            (s->control & (1 << 0)) && !(s->control & (1 << 3)) &&
            (s->mode & (1 << 7)) && (s->fcontrol & (1 << 13));
    s->rx_en =
            (s->control & (1 << 0)) && !(s->control & (1 << 2)) &&
            (s->mode & (1 << 6)) && (s->fcontrol & (1 << 12));
    s->control &= ~0xc0;
    /* The specs are unclear about the FIFO-ready flags logic.
     * Implement semantics that make most sense.  */
    if (s->tx_en && s->tx_len)
        s->control |= (1 << 7);
    if (s->rx_en && s->rx_len)
        s->control |= (1 << 6);

    qemu_set_irq(s->dma[S3C_RQ_I2SSDO], (s->control >> 5) &
                    (s->control >> 7) & (s->fcontrol >> 15) & 1);
    qemu_set_irq(s->dma[S3C_RQ_I2SSDI0], (s->control >> 4) &
                    (s->control >> 6) & (s->fcontrol >> 14) & 1);
    qemu_set_irq(s->dma[S3C_RQ_I2SSDI1], (s->control >> 4) &
                    (s->control >> 6) & (s->fcontrol >> 14) & 1);
}

void s3c_i2s_reset(struct s3c_i2s_state_s *s)
{
    s->control = 0x100;
    s->mode = 0x000;
    s->prescaler = 0x000;
    s->fcontrol = 0x0000;
    s->tx_len = 0;
    s->rx_len = 0;
    s3c_i2s_update(s);
}

#define S3C_IISCON	0x00	/* IIS Control register */
#define S3C_IISMOD	0x04	/* IIS Mode register */
#define S3C_IISPSR	0x08	/* IIS Prescaler register */
#define S3C_IISFCON	0x0c	/* IIS FIFO Interface register */
#define S3C_IISFIFO	0x10	/* IIS FIFO register */

static uint32_t s3c_i2s_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c_i2s_state_s *s = (struct s3c_i2s_state_s *) opaque;
    uint32_t ret;

    switch (addr) {
    case S3C_IISCON:
        return s->control | 0x80;
		//XXX: WM5 port: fixes wavedev.dll hanging
		//0x80 is FIFO TX Ready
    case S3C_IISMOD:
        return s->mode;
    case S3C_IISPSR:
        return s->prescaler;
    case S3C_IISFCON:
        return s->fcontrol |
                (MAX(32 - s->tx_len, 0) << 6) |
                MIN(s->rx_len, 32);
    case S3C_IISFIFO:
        if (s->rx_len > 0) {
            s->rx_len --;
            s3c_i2s_update(s);
            s->cycle ^= 1;
            if (s->cycle) {
                s->buffer = (uint16_t) (ret = s->codec_in(s->opaque));
                return ret >> 16;
            } else
                return s->buffer;
        }
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

static void s3c_i2s_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c_i2s_state_s *s = (struct s3c_i2s_state_s *) opaque;

    switch (addr) {
    case S3C_IISCON:
        s->control = (s->control & 0x100) | (value & 0x03f);
        s3c_i2s_update(s);
        break;
    case S3C_IISMOD:
        s->mode = value & 0x1ff;
        s3c_i2s_update(s);
        break;
    case S3C_IISPSR:
        s->prescaler = value & 0x3ff;
        break;
    case S3C_IISFCON:
        s->fcontrol = value & 0xf000;
        s3c_i2s_update(s);
        break;
    case S3C_IISFIFO:
        if (s->tx_len && s->tx_en) {
            s->tx_len --;
            s3c_i2s_update(s);
            if (s->cycle)
                s->codec_out(s->opaque, value | ((uint32_t) s->buffer << 16));
            else
                s->buffer = (uint16_t) value;
            s->cycle ^= 1;
        }
        break;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
    }
}

static CPUReadMemoryFunc *s3c_i2s_readfn[] = {
    s3c_i2s_read,
    s3c_i2s_read,
    s3c_i2s_read,
};

static CPUWriteMemoryFunc *s3c_i2s_writefn[] = {
    s3c_i2s_write,
    s3c_i2s_write,
    s3c_i2s_write,
};

static void s3c_i2s_save(QEMUFile *f, void *opaque)
{
    struct s3c_i2s_state_s *s = (struct s3c_i2s_state_s *) opaque;
    qemu_put_be16s(f, &s->control);
    qemu_put_be16s(f, &s->mode);
    qemu_put_be16s(f, &s->prescaler);
    qemu_put_be16s(f, &s->fcontrol);

    qemu_put_be32(f, s->tx_en);
    qemu_put_be32(f, s->rx_en);
    qemu_put_be32(f, s->tx_len);
    qemu_put_be32(f, s->rx_len);
    qemu_put_be16(f, s->buffer);
    qemu_put_be32(f, s->cycle);
}

static int s3c_i2s_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c_i2s_state_s *s = (struct s3c_i2s_state_s *) opaque;
    qemu_get_be16s(f, &s->control);
    qemu_get_be16s(f, &s->mode);
    qemu_get_be16s(f, &s->prescaler);
    qemu_get_be16s(f, &s->fcontrol);

    s->tx_en = qemu_get_be32(f);
    s->rx_en = qemu_get_be32(f);
    s->tx_len = qemu_get_be32(f);
    s->rx_len = qemu_get_be32(f);
    s->buffer = qemu_get_be16(f);
    s->cycle = qemu_get_be32(f);

    return 0;
}

static void s3c_i2s_data_req(void *opaque, int tx, int rx)
{
    struct s3c_i2s_state_s *s = (struct s3c_i2s_state_s *) opaque;
    s->tx_len = tx;
    s->rx_len = rx;
    s3c_i2s_update(s);
}

struct s3c_i2s_state_s *s3c_i2s_init(target_phys_addr_t base, qemu_irq *dma)
{
    int iomemtype;
    struct s3c_i2s_state_s *s = (struct s3c_i2s_state_s *)
            qemu_mallocz(sizeof(struct s3c_i2s_state_s));

    s->base = base;
    s->dma = dma;
    s->data_req = s3c_i2s_data_req;

    s3c_i2s_reset(s);

    iomemtype = cpu_register_io_memory(0, s3c_i2s_readfn,
                    s3c_i2s_writefn, s);
    cpu_register_physical_memory(s->base, 0xffffff, iomemtype);

    register_savevm("s3c24xx_iis", 0, 0, s3c_i2s_save, s3c_i2s_load, s);

    return s;
}
