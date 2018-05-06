#include "s3c.h"
#include "hw.h"

/* Serial Peripheral Interface */
struct s3c_spi_state_s {
    target_phys_addr_t base;

    struct {
        qemu_irq irq;
        qemu_irq drq;
        qemu_irq miso;

        uint8_t control;
        uint8_t pin;
        uint8_t pre;

        int cs_pin;
        int clk_pin;
        int mosi_pin;
        uint8_t txbuf;
        uint8_t rxbuf;
        int bit;
    } chan[2];

    uint8_t (*txrx[2])(void *opaque, uint8_t value);
    uint8_t (*btxrx[2])(void *opaque, uint8_t value);
    void *opaque[2];
};

static void s3c_spi_update(struct s3c_spi_state_s *s)
{
    int i;
    for (i = 0; i < 2; i ++) {
        switch ((s->chan[i].control >> 5) & 3) {		/* SMOD */
        case 1:
            qemu_irq_raise(s->chan[i].irq);
            break;
        case 2:
            qemu_irq_raise(s->chan[i].drq);
            break;
        }
    }
}

void s3c_spi_reset(struct s3c_spi_state_s *s)
{
    memset(s->chan, 0, sizeof(s->chan));
    s->chan[0].pin = 0x02;
    s->chan[1].pin = 0x02;
    s3c_spi_update(s);
}

#define S3C_SPCON0	0x00	/* SPI channel 0 control register */
#define S3C_SPSTA0	0x04	/* SPI channel 0 status register */
#define S3C_SPPIN0	0x08	/* SPI channel 0 pin control register */
#define S3C_SPPRE0	0x0c	/* SPI channel 0 baudrate prescaler register */
#define S3C_SPTDAT0	0x10	/* SPI channel 0 Tx data register */
#define S3C_SPRDAT0	0x14	/* SPI channel 0 Rx data register */
#define S3C_SPCON1	0x20	/* SPI channel 1 control register */
#define S3C_SPSTA1	0x24	/* SPI channel 1 status register */
#define S3C_SPPIN1	0x28	/* SPI channel 1 pin control register */
#define S3C_SPPRE1	0x2c	/* SPI channel 1 baudrate prescaler register */
#define S3C_SPTDAT1	0x30	/* SPI channel 1 Tx data register */
#define S3C_SPRDAT1	0x34	/* SPI channel 1 Rx data register */

static uint32_t s3c_spi_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c_spi_state_s *s = (struct s3c_spi_state_s *) opaque;
    int ch;

    ch = addr >> 5;

    switch (addr) {
    case S3C_SPCON0:
    case S3C_SPCON1:
        return s->chan[ch].control;

    case S3C_SPSTA0:
    case S3C_SPSTA1:
        return 0x01;

    case S3C_SPPIN0:
    case S3C_SPPIN1:
        return s->chan[ch].pin;

    case S3C_SPPRE0:
    case S3C_SPPRE1:
        return s->chan[ch].pre;

    case S3C_SPTDAT0:
    case S3C_SPTDAT1:
        return s->chan[ch + 2].txbuf;

    case S3C_SPRDAT0:
    case S3C_SPRDAT1:
        if (s->txrx[ch] && (s->chan[ch].control & 0x19) == 0x19)
            s->chan[ch].rxbuf = s->txrx[ch](s->opaque[ch], 'Q');
        s3c_spi_update(s);
        return s->chan[ch].rxbuf;

    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

/* Table to convert from PC scancodes to raw scancodes.  */
static const unsigned char ps2_raw_keycode[128] = {
  0, 41, 49, 57, 47, 55, 63, 71, 79, 87, 65, 73,  0, 89,105,
 11, 43, 51, 59, 67, 58, 66, 74, 82, 75, 83, 91,107, 90, 25,
 13, 45, 53, 61, 69, 77, 85, 68, 76, 84, 92,  0, 18, 10, 12,
 14, 46, 54, 62, 70, 78, 86, 94, 93, 96,  0,  0,110, 44,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,108,  0,  0,
109,  0,111,  0,  0,106,  0,  0, 42,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,
};
static void spi_put_keycode(void *opaque, int keycode)
{
    extern struct s3c_state_s * g_s3c;
    struct s3c_spi_state_s *s = (struct s3c_spi_state_s *) opaque;
    int dkeycode = keycode;

    dkeycode = ps2_raw_keycode[keycode & 0X7f];
    if( keycode >= 0x80 )
        dkeycode |= 0x80;
    if( dkeycode != 0 && dkeycode != 0x80 ){
        s->chan[1].rxbuf = dkeycode;
        qemu_irq_raise(g_s3c->irq[S3C_PIC_EINT1]);
    } else 
        printf("%s: unsupported key=%02x dkey=%02x tx=%02x\n", __FUNCTION__,
            keycode, dkeycode,
            s->chan[1].txbuf);
}

static void s3c_spi_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c_spi_state_s *s = (struct s3c_spi_state_s *) opaque;
    int ch;

    ch = addr >> 5;

    switch (addr) {
    case S3C_SPCON0:
    case S3C_SPCON1:
        s->chan[ch].control = value & 0x7f;
        s3c_spi_update(s);
        break;

    case S3C_SPPIN0:
    case S3C_SPPIN1:
        s->chan[ch].pin = value & 0x07;
        break;

    case S3C_SPPRE0:
    case S3C_SPPRE1:
        s->chan[ch].pre = value & 0xff;
        break;

    case S3C_SPTDAT0:
    case S3C_SPTDAT1:
        s->chan[ch].txbuf = value & 0xff;
        if (s->txrx[ch] && (s->chan[ch].control & 0x19) == 0x18)
            s->chan[ch].rxbuf = s->txrx[ch](s->opaque[ch], value & 0xff);
        s3c_spi_update(s);
        break;

    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
    }
}

static CPUReadMemoryFunc *s3c_spi_readfn[] = {
    s3c_spi_read,
    s3c_spi_read,
    s3c_spi_read,
};

static CPUWriteMemoryFunc *s3c_spi_writefn[] = {
    s3c_spi_write,
    s3c_spi_write,
    s3c_spi_write,
};

static void s3c_spi_save(QEMUFile *f, void *opaque)
{
    struct s3c_spi_state_s *s = (struct s3c_spi_state_s *) opaque;
    int i;
    for (i = 0; i < 2; i ++) {
        qemu_put_8s(f, &s->chan[i].control);
        qemu_put_8s(f, &s->chan[i].pin);
        qemu_put_8s(f, &s->chan[i].pre);

        qemu_put_8s(f, &s->chan[i].txbuf);
        qemu_put_8s(f, &s->chan[i].rxbuf);
        qemu_put_be32(f, s->chan[i].cs_pin);
        qemu_put_be32(f, s->chan[i].clk_pin);
        qemu_put_be32(f, s->chan[i].mosi_pin);
        qemu_put_be32(f, s->chan[i].bit);
    }
}

static int s3c_spi_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c_spi_state_s *s = (struct s3c_spi_state_s *) opaque;
    int i;
    for (i = 0; i < 2; i ++) {
        qemu_get_8s(f, &s->chan[i].control);
        qemu_get_8s(f, &s->chan[i].pin);
        qemu_get_8s(f, &s->chan[i].pre);

        qemu_get_8s(f, &s->chan[i].txbuf);
        qemu_get_8s(f, &s->chan[i].rxbuf);
        s->chan[i].cs_pin = qemu_get_be32(f);
        s->chan[i].clk_pin = qemu_get_be32(f);
        s->chan[i].mosi_pin = qemu_get_be32(f);
        s->chan[i].bit = qemu_get_be32(f);
    }

    return 0;
}

static void s3c_spi_bitbang_cs(void *opaque, int line, int level)
{
    struct s3c_spi_state_s *s = (struct s3c_spi_state_s *) opaque;
    int ch = line;
    if (s->chan[ch].cs_pin || level) {
        if (s->chan[ch].bit && s->txrx[ch] && !s->btxrx[ch]) {
            s->chan[ch].txbuf <<= 8 - s->chan[ch].bit;
            s->chan[ch].rxbuf = s->txrx[ch](s->opaque[ch], s->chan[ch].txbuf);
        }
    } else if (!s->chan[ch].cs_pin || !level)
        s->chan[ch].bit = 0;

    /* SSn is active low.  */
    s->chan[ch].cs_pin = !level;
}

static void s3c_spi_bitbang_clk(void *opaque, int line, int level)
{
    struct s3c_spi_state_s *s = (struct s3c_spi_state_s *) opaque;
    int ch = line;
    if (!s->chan[ch].cs_pin)
        goto done;

    /* Detect CLK rising edge */
    if (s->chan[ch].clk_pin || !level)
        goto done;

    if (s->btxrx[ch]) {
        qemu_set_irq(s->chan[ch].miso,
                        s->btxrx[ch](s->opaque[ch], s->chan[ch].mosi_pin));
        goto done;
    }

    s->chan[ch].txbuf <<= 1;
    s->chan[ch].txbuf |= s->chan[ch].mosi_pin;

    qemu_set_irq(s->chan[ch].miso, (s->chan[ch].rxbuf >> 7) & 1);
    s->chan[ch].rxbuf <<= 1;

    if (++ s->chan[ch].bit == 8) {
        if (s->txrx[ch])
            s->chan[ch].rxbuf = s->txrx[ch](s->opaque[ch], s->chan[ch].txbuf);
        s->chan[ch].bit = 0;
    }

done:
    s->chan[ch].clk_pin = level;
}

static void s3c_spi_bitbang_mosi(void *opaque, int line, int level)
{
    struct s3c_spi_state_s *s = (struct s3c_spi_state_s *) opaque;
    int ch = line;
    s->chan[ch].mosi_pin = level;
}

static const struct {
    int cs, clk, miso, mosi;
} s3c_spi_pins[2] = {
    { S3C_GPG(2), S3C_GPE(13), S3C_GPE(11), S3C_GPE(12) },
    { S3C_GPG(3), S3C_GPG(7),  S3C_GPG(5),  S3C_GPG(6)  },
};

static void s3c_spi_bitbang_init(struct s3c_spi_state_s *s,
                struct s3c_gpio_state_s *gpio)
{
    int i;
    qemu_irq *cs = qemu_allocate_irqs(s3c_spi_bitbang_cs, s, 2);
    qemu_irq *clk = qemu_allocate_irqs(s3c_spi_bitbang_clk, s, 2);
    qemu_irq *mosi = qemu_allocate_irqs(s3c_spi_bitbang_mosi, s, 2);

    for (i = 0; i < 2; i ++) {
        s3c_gpio_out_set(gpio, s3c_spi_pins[i].cs, cs[i]);
        s3c_gpio_out_set(gpio, s3c_spi_pins[i].clk, clk[i]);
        s->chan[i].miso = s3c_gpio_in_get(gpio)[s3c_spi_pins[i].miso];
        s3c_gpio_out_set(gpio, s3c_spi_pins[i].mosi, mosi[i]);
    }
}

struct s3c_spi_state_s *s3c_spi_init(target_phys_addr_t base,
                qemu_irq irq0, qemu_irq drq0, qemu_irq irq1, qemu_irq drq1,
                struct s3c_gpio_state_s *gpio)
{
    int iomemtype;
    struct s3c_spi_state_s *s = (struct s3c_spi_state_s *)
            qemu_mallocz(sizeof(struct s3c_spi_state_s));

    s->base = base;
    s->chan[0].irq = irq0;
    s->chan[0].drq = drq0;
    s->chan[1].irq = irq1;
    s->chan[1].drq = drq1;

    s3c_spi_reset(s);

    iomemtype = cpu_register_io_memory(0, s3c_spi_readfn,
                    s3c_spi_writefn, s);
    cpu_register_physical_memory(s->base, 0xffffff, iomemtype);

    s3c_spi_bitbang_init(s, gpio);
    qemu_add_kbd_event_handler(spi_put_keycode, s);

    register_savevm("s3c24xx_spi", 0, 0, s3c_spi_save, s3c_spi_load, s);

    return s;
}

void s3c_spi_attach(struct s3c_spi_state_s *s, int ch,
                uint8_t (*txrx)(void *opaque, uint8_t value),
                uint8_t (*btxrx)(void *opaque, uint8_t value), void *opaque)
{
    if (ch & ~1)
        cpu_abort(cpu_single_env, "%s: No channel %i\n", __FUNCTION__, ch);
    s->txrx[ch] = txrx;
    s->btxrx[ch] = btxrx;
    s->opaque[ch] = opaque;
}
