#include "s3c.h"
#include "hw.h"

/* IIC-bus serial interface */

static void s3c_i2c_irq(struct s3c_i2c_state_s *s)
{
    s->control |= 1 << 4;
    if (s->control & (1 << 5))
        qemu_irq_raise(s->irq);
}

void s3c_i2c_reset(struct s3c_i2c_state_s *s)
{
    s->control = 0x00;
    s->status = 0x00;
    s->busy = 0;
    s->newstart = 0;
    s->mmaster = 0;
}

static void s3c_i2c_event(i2c_slave *i2c, enum i2c_event event)
{
    struct s3c_i2c_state_s *s = (struct s3c_i2c_state_s *) i2c;
    if (!(s->status & (1 << 4)))
        return;

    switch (event) {
    case I2C_START_RECV:
    case I2C_START_SEND:
        s->status |= 1 << 2;
        s3c_i2c_irq(s);
        break;
    case I2C_FINISH:
        s->status &= ~6;
        break;
    case I2C_NACK:
        s->status |= 1 << 0;
        break;
    default:
        break;
    }
}

static int s3c_i2c_tx(i2c_slave *i2c, uint8_t data)
{
    struct s3c_i2c_state_s *s = (struct s3c_i2c_state_s *) i2c;
    if (!(s->status & (1 << 4)))
        return 1;

    if ((s->status >> 6) == 0)
        s->data = data;						/* TODO */
    s->status &= ~(1 << 0);
    s3c_i2c_irq(s);

    return !(s->control & (1 << 7));
}

static int s3c_i2c_rx(i2c_slave *i2c)
{
    struct s3c_i2c_state_s *s = (struct s3c_i2c_state_s *) i2c;
    if (!(s->status & (1 << 4)))
        return 1;

    if ((s->status >> 6) == 1) {
        s->status &= ~(1 << 0);
        s3c_i2c_irq(s);
        return s->data;
    }

    return 0x00;
}

static void s3c_master_work(void *opaque)
{
    struct s3c_i2c_state_s *s = (struct s3c_i2c_state_s *) opaque;
    int start = 0, stop = 0, ack = 1;
    if (s->control & (1 << 4))				/* Interrupt pending */
        return;
    if ((s->status & 0x90) != 0x90)			/* Master */
        return;
    stop = ~s->status & (1 << 5);
    if (s->newstart && s->status & (1 << 5)) {		/* START */
        s->busy = 1;
        start = 1;
    }
    s->newstart = 0;
    if (!s->busy)
        return;

    if (start)
        ack = !i2c_start_transfer(s->bus, s->data >> 1, (~s->status >> 6) & 1);
    else if (stop)
        i2c_end_transfer(s->bus);
    else if (s->status & (1 << 6))
        ack = !i2c_send(s->bus, s->data);
    else {
        s->data = i2c_recv(s->bus);

        if (!(s->control & (1 << 7)))			/* ACK */
            i2c_nack(s->bus);
    }

    if (!(s->status & (1 << 5))) {
        s->busy = 0;
        return;
    }
    s->status &= ~1;
    s->status |= !ack;
    if (!ack)
        s->busy = 0;
    s3c_i2c_irq(s);
}

#define S3C_IICCON	0x00	/* IIC-Bus Control register */
#define S3C_IICSTAT	0x04	/* IIC-Bus Control / Status register */
#define S3C_IICADD	0x08	/* IIC-Bus Address register */
#define S3C_IICDS	0x0c	/* IIC-Bus Tx / Rx Data Shift register */

#define S3C2440_IICLC	0x10 /* IIC-Bus multi-master line control register */

static uint32_t s3c_i2c_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c_i2c_state_s *s = (struct s3c_i2c_state_s *) opaque;

    switch (addr) {
    case S3C_IICCON:
        return s->control;
    case S3C_IICSTAT:
        return s->status & ~(1 << 5);			/* Busy signal */
    case S3C_IICADD:
        return s->addy;
    case S3C_IICDS:
        return s->data;
    case S3C2440_IICLC:		/* s3c2440 only ! */
    	return s->mmaster;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

static void s3c_i2c_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c_i2c_state_s *s = (struct s3c_i2c_state_s *) opaque;

    switch (addr) {
    case S3C_IICCON:
        s->control = (s->control | 0xef) & value;
        if (s->busy)
            s3c_master_work(s);
        break;

    case S3C_IICSTAT:
        s->status &= 0x0f;
        s->status |= value & 0xf0;
        if (s->status & (1 << 5))
            s->newstart = 1;
        s3c_master_work(s);
        break;

    case S3C_IICADD:
        s->addy = value & 0x7f;
        i2c_set_slave_address(&s->slave, s->addy);
        break;

    case S3C_IICDS:
        s->data = value & 0xff;
        break;

    case S3C2440_IICLC:		/* s3c2440 only ! */
		s->mmaster = value & 0xff;
		break;

    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
    }
}

static CPUReadMemoryFunc *s3c_i2c_readfn[] = {
    s3c_i2c_read,
    s3c_i2c_read,
    s3c_i2c_read,
};

static CPUWriteMemoryFunc *s3c_i2c_writefn[] = {
    s3c_i2c_write,
    s3c_i2c_write,
    s3c_i2c_write,
};

static void s3c_i2c_save(QEMUFile *f, void *opaque)
{
    struct s3c_i2c_state_s *s = (struct s3c_i2c_state_s *) opaque;
    qemu_put_8s(f, &s->control);
    qemu_put_8s(f, &s->status);
    qemu_put_8s(f, &s->data);
    qemu_put_8s(f, &s->addy);
    qemu_put_8s(f, &s->mmaster);

    qemu_put_be32(f, s->busy);
    qemu_put_be32(f, s->newstart);

    i2c_slave_save(f, &s->slave);
}

static int s3c_i2c_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c_i2c_state_s *s = (struct s3c_i2c_state_s *) opaque;
    qemu_get_8s(f, &s->control);
    qemu_get_8s(f, &s->status);
    qemu_get_8s(f, &s->data);
    qemu_get_8s(f, &s->addy);
    qemu_get_8s(f, &s->mmaster);

    s->busy = qemu_get_be32(f);
    s->newstart = qemu_get_be32(f);

    i2c_slave_load(f, &s->slave);
    return 0;
}

//struct s3c_i2c_state_s *s3c_i2c_init(target_phys_addr_t base, qemu_irq irq)
void s3c_i2c_init(SysBusDevice * dev)
{
    int iomemtype;
    struct s3c_i2c_state_s *s = FROM_SYSBUS(struct s3c_i2c_state_s, dev);
#if 0
    s->slave.event = s3c_i2c_event;
    s->slave.send = s3c_i2c_tx;
    s->slave.recv = s3c_i2c_rx;
#endif
    s->bus = i2c_init_bus();
    sysbus_init_irq(dev, &s->irq);
    qdev_attach_child_bus(&dev->qdev, "i2c", s->bus);

    s3c_i2c_reset(s);

    iomemtype = cpu_register_io_memory(0, s3c_i2c_readfn,
                    s3c_i2c_writefn, s);
    sysbus_init_mmio(dev, 0xffffff, iomemtype);

    register_savevm("s3c24xx_i2c", 0, 0, s3c_i2c_save, s3c_i2c_load, s);
}

i2c_bus *s3c_i2c_bus(struct s3c_i2c_state_s *s)
{
    return s->bus;
}
