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
#include "hw.h"
#include "arm-misc.h"

/* Interrupt controller */
struct s3c_pic_state_s {
    target_phys_addr_t base;
    qemu_irq *parent_pic;
    qemu_irq *irqs;

    uint32_t srcpnd;
    uint32_t intpnd;
    uint32_t intmsk;
    uint32_t intmod;
    uint32_t priority;
    int intoffset;
    uint32_t subsrcpnd;
    uint32_t intsubmsk;
};

static void s3c_pic_update(struct s3c_pic_state_s *s)
{
    qemu_set_irq(s->parent_pic[ARM_PIC_CPU_FIQ],
                    s->srcpnd & s->intmod);
    qemu_set_irq(s->parent_pic[ARM_PIC_CPU_IRQ],
                    s->intpnd & ~s->intmsk & ~s->intmod);
}

/*
 * Performs interrupt arbitration and notifies the CPU.
 *
 * Since it's a complex logic which cannot be relied on by the OS
 * anyway - first because real hardware doesn't do it accurately,
 * second because it only matters when interrupts occur at the
 * same time which normally can't be predicted - we use a simpler
 * version for non-debug runs.
 */
#ifdef DEBUG
static const uint32_t s3c_arbmsk[6] = {
    0x0000000f,
    0x000003f0,
    0x0000fc00,
    0x003f0000,
    0x0fc00000,
    0xf0000000,
};

# define S3C_ARB_SEL(i)		((s->priority >> (7 + (i << 1))) & 3)
# define S3C_ARB_MODE(i)	((s->priority >> i) & 1)
# define S3C_ARB_SEL_SET(i, v)	\
    s->priority &= ~(3 << (7 + (i << 1))); \
    s->priority |= v << (7 + (i << 1));

static void s3c_pic_arbitrate(struct s3c_pic_state_s *s)
{
    uint32_t pnd = s->srcpnd & ~s->intmsk & ~s->intmod;
    int offset, i, arb;
    if (s->intpnd || !pnd) {
        s3c_pic_update(s);
        return;
    }

    if (pnd & s3c_arbmsk[0]) {
        offset = 0;
        arb = 0;
    } else if (pnd & 0x0ffffff0) {
        i = S3C_ARB_SEL(6);
        i ^= i << 1;
        if (!(pnd & s3c_arbmsk[1 + (i & 3)]))
            if (!(pnd & s3c_arbmsk[1 + (++ i & 3)]))
                if (!(pnd & s3c_arbmsk[1 + (++ i & 3)]))
                    i ++;

        if (S3C_ARB_MODE(6))
            S3C_ARB_SEL_SET(6, ((i + 1) & 3));
        offset = (i & 3) * 6 + 4;
        if (pnd & (1 << offset))
            goto known_offset;
        else if (!(pnd & (0x1f << offset))) {
            offset += 5;
            goto known_offset;
        }
        offset ++;
        arb = (i & 3) + 1;
    } else {
        arb = 5;
        offset = 28;
    }

    pnd >>= offset;
    i = S3C_ARB_SEL(arb);
    i ^= i << 1;
    if (!(pnd & (1 << (i & 3))))
        if (!(pnd & (1 << (++ i & 3))))
            if (!(pnd & (1 << (++ i & 3))))
                i ++;

    if (S3C_ARB_MODE(arb))
        S3C_ARB_SEL_SET(arb, ((i + 1) & 3));
    offset += i & 3;
known_offset:
    s->intoffset = offset;
    s->intpnd = 1 << offset;
    s3c_pic_update(s);
}
#else
inline static void s3c_pic_arbitrate(struct s3c_pic_state_s *s)
{
    uint32_t pnd = s->srcpnd & ~s->intmsk & ~s->intmod;
    if (pnd && !s->intpnd)
        s->intpnd = 1 << (s->intoffset = ffs(pnd) - 1);
    s3c_pic_update(s);
}
#endif

static const int s3c_sub_src_map[] = {
    [S3C_PICS_RXD0 & 31] = S3C_PIC_UART0,
    [S3C_PICS_TXD0 & 31] = S3C_PIC_UART0,
    [S3C_PICS_ERR0 & 31] = S3C_PIC_UART0,
    [S3C_PICS_RXD1 & 31] = S3C_PIC_UART1,
    [S3C_PICS_TXD1 & 31] = S3C_PIC_UART1,
    [S3C_PICS_ERR1 & 31] = S3C_PIC_UART1,
    [S3C_PICS_RXD2 & 31] = S3C_PIC_UART2,
    [S3C_PICS_TXD2 & 31] = S3C_PIC_UART2,
    [S3C_PICS_ERR2 & 31] = S3C_PIC_UART2,
    [S3C_PICS_TC   & 31] = S3C_PIC_ADC,
    [S3C_PICS_ADC  & 31] = S3C_PIC_ADC,
};

static void s3c_pic_subupdate(struct s3c_pic_state_s *s)
{
    int next;
    const int *sub = &s3c_sub_src_map[-1];
    uint32_t pnd = s->subsrcpnd & ~s->intsubmsk;
    while ((next = ffs(pnd))) {
        sub += next;
        pnd >>= next;
        s->srcpnd |= 1 << *sub;
    }
    s3c_pic_arbitrate(s);
}

static void s3c_pic_set_irq(void *opaque, int irq, int req)
{
    struct s3c_pic_state_s *s = (struct s3c_pic_state_s *) opaque;
    uint32_t mask;
    /* This interrupt controller doesn't clear any request signals
     * or register bits automatically.  */
    if (!req)
        return;

    if (irq & 32) {
        irq &= 31;
        s->subsrcpnd |= 1 << irq;
        if (s->intsubmsk & (1 << irq))
            return;
        else
            irq = s3c_sub_src_map[irq];
    }
    s->srcpnd |= (mask = 1 << irq);

    /* A FIQ */
    if (s->intmod & mask)
        qemu_irq_raise(s->parent_pic[ARM_PIC_CPU_FIQ]);
    else if (!s->intpnd && !(s->intmsk & mask)) {
#ifdef DEBUG
        s3c_pic_arbitrate(s);
#else
        s->intpnd = mask;
        s->intoffset = irq;
        qemu_irq_raise(s->parent_pic[ARM_PIC_CPU_IRQ]);
#endif
    }
}

void s3c_pic_reset(struct s3c_pic_state_s *s)
{
    s->srcpnd = 0;
    s->intpnd = 0;
    s->intmsk = 0xffffffff;
    s->intmod = 0;
    s->priority = 0x7f;
    s->intoffset = 0;
    s->subsrcpnd = 0;
    s->intsubmsk = 0x7ff;
    s3c_pic_update(s);
}

#define S3C_SRCPND	0x00	/* Source Pending register */
#define S3C_INTMOD	0x04	/* Source Mode register */
#define S3C_INTMSK	0x08	/* Interrupt Mask register */
#define S3C_PRIORITY	0x0c	/* Priority register */
#define S3C_INTPND	0x10	/* Interrupt Pending register */
#define S3C_INTOFFSET	0x14	/* Interrupt Offset register */
#define S3C_SUBSRCPND	0x18	/* Sub Source Pending register */
#define S3C_INTSUBMSK	0x1c	/* Interrupt Sub Mask register */

static uint32_t s3c_pic_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c_pic_state_s *s = (struct s3c_pic_state_s *) opaque;

    switch (addr) {
    case S3C_SRCPND:
        return s->srcpnd;
    case S3C_INTPND:
        return s->intpnd;
    case S3C_INTMSK:
        return s->intmsk;
    case S3C_INTMOD:
        return s->intmod;
    case S3C_PRIORITY:
        return s->priority;
    case S3C_INTOFFSET:
        return s->intoffset;
    case S3C_SUBSRCPND:
        return s->subsrcpnd;
    case S3C_INTSUBMSK:
        return s->intsubmsk;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

static void s3c_pic_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c_pic_state_s *s = (struct s3c_pic_state_s *) opaque;

    switch (addr) {
    case S3C_SRCPND:
        s->srcpnd &= ~value;
        if (value & s->intmod)
            s3c_pic_update(s);
        break;
    case S3C_INTPND:
        if (s->intpnd & value) {
            s->intpnd = 0;
            s->intoffset = 0;
            s3c_pic_arbitrate(s);
        }
        break;
    case S3C_INTMSK:
        s->intmsk = value;
        if (s->intpnd & value) {
            s->intpnd = 0;
            s->intoffset = 0;
        }
        s3c_pic_arbitrate(s);
        break;
    case S3C_INTMOD:
        s->intmod = value;
        break;
    case S3C_PRIORITY:
        s->priority = value;
        break;
    case S3C_SUBSRCPND:
        s->subsrcpnd &= ~value;
        break;
    case S3C_INTSUBMSK:
        s->intsubmsk = value;
        s3c_pic_subupdate(s);
        break;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
    }
}

static CPUReadMemoryFunc *s3c_pic_readfn[] = {
    s3c_pic_read,
    s3c_pic_read,
    s3c_pic_read,
};

static CPUWriteMemoryFunc *s3c_pic_writefn[] = {
    s3c_pic_write,
    s3c_pic_write,
    s3c_pic_write,
};

static void s3c_pic_save(QEMUFile *f, void *opaque)
{
    struct s3c_pic_state_s *s = (struct s3c_pic_state_s *) opaque;
    qemu_put_be32s(f, &s->srcpnd);
    qemu_put_be32s(f, &s->intpnd);
    qemu_put_be32s(f, &s->intmsk);
    qemu_put_be32s(f, &s->intmod);
    qemu_put_be32s(f, &s->priority);
    qemu_put_be32s(f, &s->subsrcpnd);
    qemu_put_be32s(f, &s->intsubmsk);
    qemu_put_be32(f, s->intoffset);
}

static int s3c_pic_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c_pic_state_s *s = (struct s3c_pic_state_s *) opaque;
    qemu_get_be32s(f, &s->srcpnd);
    qemu_get_be32s(f, &s->intpnd);
    qemu_get_be32s(f, &s->intmsk);
    qemu_get_be32s(f, &s->intmod);
    qemu_get_be32s(f, &s->priority);
    qemu_get_be32s(f, &s->subsrcpnd);
    qemu_get_be32s(f, &s->intsubmsk);
    s->intoffset = qemu_get_be32(f);
    s3c_pic_update(s);
    return 0;
}

struct s3c_pic_state_s *s3c_pic_init(target_phys_addr_t base,
                qemu_irq *arm_pic)
{
    int iomemtype;
    struct s3c_pic_state_s *s = (struct s3c_pic_state_s *)
            qemu_mallocz(sizeof(struct s3c_pic_state_s));

    s->base = base;
    s->parent_pic = arm_pic;
    s->irqs = qemu_allocate_irqs(s3c_pic_set_irq, s, S3C_PIC_MAX);

    s3c_pic_reset(s);

    iomemtype = cpu_register_io_memory(0, s3c_pic_readfn,
                    s3c_pic_writefn, s);
    cpu_register_physical_memory(s->base, 0xffffff, iomemtype);

    register_savevm("s3c24xx_pic", 0, 0, s3c_pic_save, s3c_pic_load, s);

    return s;
}

qemu_irq *s3c_pic_get(struct s3c_pic_state_s *s)
{
    return s->irqs;
}
