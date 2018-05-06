#include "s3c.h"
#include "hw.h"
#include "qemu-char.h"

/* UART */
struct s3c_uart_state_s {
    struct s3c_freq_s * freq;
    target_phys_addr_t base;
    qemu_irq *irq;
    qemu_irq *dma;
    uint8_t data;
    uint8_t rxfifo[16];
    int rxstart;
    int rxlen;
#define UART_MAX_CHR	4
    int chr_num;
    CharDriverState *chr[UART_MAX_CHR];

    uint8_t lcontrol;
    uint8_t fcontrol;
    uint8_t mcontrol;
    uint16_t control;
    uint16_t brdiv;
    uint8_t errstat;
};

void s3c_uart_reset(struct s3c_uart_state_s *s)
{
    s->lcontrol = 0x00;
    s->fcontrol = 0x00;
    s->mcontrol = 0x00;
    s->control = 0x0000;
    s->errstat = 0;

    s->rxstart = 0;
    s->rxlen = 0;
}

static void s3c_uart_err(struct s3c_uart_state_s *s, int err)
{
    s->errstat |= err;
    if (s->control & (1 << 6))
        qemu_irq_raise(s->irq[2]);
}

inline static void s3c_uart_full(struct s3c_uart_state_s *s, int pulse)
{
    if (s->fcontrol & 1)			/* FIFOEnable */
        if (s->rxlen < (((s->fcontrol >> 4) & 3) + 1) * 4) {
            if (((s->control >> 0) & 3) != 1 ||	/* ReceiveMode */
                            !s->rxlen)
                return;
            if (!(s->control & (1 << 7)))	/* RxTimeOutEnable */
                return;
            /* When the Rx FIFO trigger level is not reached, the interrupt
             * is generated anyway, just after a small timeout instead of
             * immediately.  */
        }

    switch ((s->control >> 0) & 3) {		/* ReceiveMode */
    case 1:
        if ((s->control & (1 << 8)) || pulse)	/* RxInterruptType */
            qemu_irq_raise(s->irq[0]);
        break;
    case 2:
    case 3:
        qemu_irq_raise(s->dma[0]);
        break;
    }
}

inline static void s3c_uart_empty(struct s3c_uart_state_s *s, int pulse)
{
    switch ((s->control >> 2) & 3) {		/* TransmitMode */
    case 1:
        if ((s->control & (1 << 9)) || pulse)	/* TxInterruptType */
            qemu_irq_raise(s->irq[1]);
        break;
    case 2:
    case 3:
        qemu_irq_raise(s->dma[0]);
        break;
    }
}

inline static void s3c_uart_update(struct s3c_uart_state_s *s)
{
    s3c_uart_empty(s, 0);
    s3c_uart_full(s, 0);
}

static void s3c_uart_params_update(struct s3c_uart_state_s *s)
{
    QEMUSerialSetParams ssp;
    int i;
    if (!s->chr)
        return;

    /* XXX Calculate PCLK frequency from clock manager registers */
    ssp.speed = (s->freq->pclk >> 4) / (s->brdiv + 1);

    switch ((s->lcontrol >> 3) & 7) {
    case 4:
    case 6:
        ssp.parity = 'O';
        break;
    case 5:
    case 7:
        ssp.parity = 'E';
        break;
    default:
        ssp.parity = 'N';
    }

    ssp.data_bits = 5 + (s->lcontrol & 3);

    ssp.stop_bits = (s->lcontrol & (1 << 2)) ? 2 : 1;

    for (i = 0; i < s->chr_num; i ++)
        qemu_chr_ioctl(s->chr[i], CHR_IOCTL_SERIAL_SET_PARAMS, &ssp);
}

static int s3c_uart_is_empty(void *opaque)
{
    struct s3c_uart_state_s *s = (struct s3c_uart_state_s *) opaque;
    if (s->fcontrol & 1)			/* FIFOEnable */
        return 16 - s->rxlen;
    else
        return 1 - s->rxlen;
}

static void s3c_uart_rx(void *opaque, const uint8_t *buf, int size)
{
    struct s3c_uart_state_s *s = (struct s3c_uart_state_s *) opaque;
    int left;

    if (s->fcontrol & 1) {			/* FIFOEnable */
        if (s->rxlen + size > 16) {
            size = 16 - s->rxlen;
            s3c_uart_err(s, 1);
        }

        left = 16 - ((s->rxstart + s->rxlen) & 15);
        if (size > left) {
            memcpy(s->rxfifo + ((s->rxstart + s->rxlen) & 15), buf, left);
            memcpy(s->rxfifo, buf + left, size - left);
        } else
            memcpy(s->rxfifo + ((s->rxstart + s->rxlen) & 15), buf, size);
        s->rxlen += size;
    } else {
        if (s->rxlen + size > 1)
            s3c_uart_err(s, 1);
        s->rxlen = 1;
        s->data = buf[0];
    }
    s3c_uart_full(s, 1);
}

/* S3C2410 UART doesn't seem to understand break conditions.  */
static void s3c_uart_event(void *opaque, int event)
{
}

#define S3C_ULCON	0x00	/* UART Line Control register */
#define S3C_UCON	0x04	/* UART Control register */
#define S3C_UFCON	0x08	/* UART FIFO Control register */
#define S3C_UMCON	0x0c	/* UART Modem Control register */
#define S3C_UTRSTAT	0x10	/* UART Tx/Rx Status register */
#define S3C_UERSTAT	0x14	/* UART Error Status register */
#define S3C_UFSTAT	0x18	/* UART FIFO Status register */
#define S3C_UMSTAT	0x1c	/* UART Modem Status register */
#define S3C_UTXH	0x20	/* UART Transmit Buffer register */
#define S3C_URXH	0x24	/* UART Receive Buffer register */
#define S3C_UBRDIV	0x28	/* UART Baud Rate Divisor register */

static uint32_t s3c_uart_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c_uart_state_s *s = (struct s3c_uart_state_s *) opaque;
    uint8_t ret;

    switch (addr) {
    case S3C_ULCON:
        return s->lcontrol;
    case S3C_UCON:
        return s->control;
    case S3C_UFCON:
        return s->fcontrol;
    case S3C_UMCON:
        return s->mcontrol;
    case S3C_UTRSTAT:
        return 6 | !!s->rxlen;
    case S3C_UERSTAT:
        /* XXX: UERSTAT[3] is Reserved but Linux thinks it is BREAK */
        ret = s->errstat;
        s->errstat = 0;
        s3c_uart_update(s);
        return ret;
    case S3C_UFSTAT:
        s3c_uart_update(s);
        return s->rxlen ? s->rxlen | (1 << 8) : 0;
    case S3C_UMSTAT:
        s3c_uart_update(s);
        return 0x11;
    case S3C_UTXH:	/* why this is called by u-boot is not clear */
    	return 0;
    case S3C_URXH:
        s3c_uart_update(s);
        if (s->rxlen) {
            s->rxlen --;
            if (s->fcontrol & 1) {		/* FIFOEnable */
                ret = s->rxfifo[s->rxstart ++];
                s->rxstart &= 15;
            } else
                ret = s->data;
            return ret;
        }
        return 0;
    case S3C_UBRDIV:
        return s->brdiv;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

static void s3c_uart_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c_uart_state_s *s = (struct s3c_uart_state_s *) opaque;
    uint8_t ch;
    int i;

    switch (addr) {
    case S3C_ULCON:
        if ((s->lcontrol ^ value) & (1 << 6))
            printf("%s: UART Infra-red mode %s\n", __FUNCTION__,
                            (value & (1 << 6)) ? "on" : "off");
        s->lcontrol = value;
        s3c_uart_params_update(s);
        s3c_uart_update(s);
        break;
    case S3C_UCON:
        /* XXX: UCON[4] is Reserved but Linux thinks it is BREAK */
        if ((s->control ^ value) & (1 << 5))
            printf("%s: UART loopback test mode %s\n", __FUNCTION__,
                            (value & (1 << 5)) ? "on" : "off");
        s->control = value & 0x7ef;
        s3c_uart_update(s);
        break;
    case S3C_UFCON:
        if (value & (1 << 1))			/* RxReset */
            s->rxlen = 0;
        s->fcontrol = value & 0xf1;
        s3c_uart_update(s);
        break;
    case S3C_UMCON:
#ifdef CONFIG_S3C_MODEM		/* not handled, openmoko modem.c not imported */
        if ((s->mcontrol ^ value) & (1 << 4)) {
            int afc = (value >> 4) & 1;
            for (i = 0; i < s->chr_num; i ++)
                qemu_chr_ioctl(s->chr[i], CHR_IOCTL_MODEM_HANDSHAKE, &afc);
        }
#endif
        s->mcontrol = value & 0x11;
        s3c_uart_update(s);
        break;
    case S3C_UTXH:
        ch = value & 0xff;
        for (i = 0; i < s->chr_num; i ++)
            qemu_chr_write(s->chr[i], &ch, 1);
        s3c_uart_empty(s, 1);
        s3c_uart_update(s);
        break;
    case S3C_UBRDIV:
        s->brdiv = value & 0xffff;
        s3c_uart_params_update(s);
        s3c_uart_update(s);
        break;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
    }
}

static CPUReadMemoryFunc *s3c_uart_readfn[] = {
    s3c_uart_read,
    s3c_uart_read,
    s3c_uart_read,
};

static CPUWriteMemoryFunc *s3c_uart_writefn[] = {
    s3c_uart_write,
    s3c_uart_write,
    s3c_uart_write,
};

static void s3c_uart_save(QEMUFile *f, void *opaque)
{
    struct s3c_uart_state_s *s = (struct s3c_uart_state_s *) opaque;
    qemu_put_8s(f, &s->data);
    qemu_put_buffer(f, s->rxfifo, sizeof(s->rxfifo));
    qemu_put_be32(f, s->rxstart);
    qemu_put_be32(f, s->rxlen);
    qemu_put_8s(f, &s->lcontrol);
    qemu_put_8s(f, &s->fcontrol);
    qemu_put_8s(f, &s->mcontrol);
    qemu_put_be16s(f, &s->control);
    qemu_put_be16s(f, &s->brdiv);
    qemu_put_8s(f, &s->errstat);
}

static int s3c_uart_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c_uart_state_s *s = (struct s3c_uart_state_s *) opaque;
    qemu_get_8s(f, &s->data);
    qemu_get_buffer(f, s->rxfifo, sizeof(s->rxfifo));
    s->rxstart = qemu_get_be32(f);
    s->rxlen = qemu_get_be32(f);
    qemu_get_8s(f, &s->lcontrol);
    qemu_get_8s(f, &s->fcontrol);
    qemu_get_8s(f, &s->mcontrol);
    qemu_get_be16s(f, &s->control);
    qemu_get_be16s(f, &s->brdiv);
    qemu_get_8s(f, &s->errstat);

    return 0;
}

struct s3c_uart_state_s *s3c_uart_init(struct s3c_freq_s * freq, target_phys_addr_t base,
                qemu_irq *irqs, qemu_irq *dma)
{
    int iomemtype;
    struct s3c_uart_state_s *s = (struct s3c_uart_state_s *)
            qemu_mallocz(sizeof(struct s3c_uart_state_s));

    s->freq = freq;
    s->base = base;
    s->irq = irqs;
    s->dma = dma;

    s3c_uart_reset(s);

    iomemtype = cpu_register_io_memory(0, s3c_uart_readfn,
                    s3c_uart_writefn, s);
    cpu_register_physical_memory(s->base, 0xfff, iomemtype);

    register_savevm("s3c24xx_uart", base, 0, s3c_uart_save, s3c_uart_load, s);

    return s;
}

void s3c_uart_attach(struct s3c_uart_state_s *s, CharDriverState *chr)
{
    if (s->chr_num >= UART_MAX_CHR)
        cpu_abort(cpu_single_env, "%s: Too many devices\n", __FUNCTION__);
    s->chr[s->chr_num ++] = chr;

    qemu_chr_add_handlers(chr, s3c_uart_is_empty,
                    s3c_uart_rx, s3c_uart_event, s);
}
