#include "s3c.h"
#include "sysbus.h"

struct s3c_mc_state_s {
    target_phys_addr_t mc_base;
    uint32_t mc_regs[13];
};

/* Memory controller */
#define S3C_BWSCON	0x00	/* Bus Width & Wait Control register */
#define S3C_BANKCON0	0x04	/* Bank 0 Control register */
#define S3C_BANKCON1	0x08	/* Bank 1 Control register */
#define S3C_BANKCON2	0x0c	/* Bank 2 Control register */
#define S3C_BANKCON3	0x10	/* Bank 3 Control register */
#define S3C_BANKCON4	0x14	/* Bank 4 Control register */
#define S3C_BANKCON5	0x18	/* Bank 5 Control register */
#define S3C_BANKCON6	0x1c	/* Bank 6 Control register */
#define S3C_BANKCON7	0x20	/* Bank 7 Control register */
#define S3C_REFRESH	0x24	/* SDRAM Refresh Control register */
#define S3C_BANKSIZE	0x28	/* Flexible Bank Size register */
#define S3C_MRSRB6	0x2c	/* Bank 6 Mode Set register */
#define S3C_MRSRB7	0x30	/* Bank 6 Mode Set register */

void s3c_mc_reset(struct s3c_mc_state_s *s)
{
    s->mc_regs[S3C_BWSCON >> 2] = 0x0000000;
    s->mc_regs[S3C_BANKCON0 >> 2] = 0x0700;
    s->mc_regs[S3C_BANKCON1 >> 2] = 0x0700;
    s->mc_regs[S3C_BANKCON2 >> 2] = 0x0700;
    s->mc_regs[S3C_BANKCON3 >> 2] = 0x0700;
    s->mc_regs[S3C_BANKCON4 >> 2] = 0x0700;
    s->mc_regs[S3C_BANKCON5 >> 2] = 0x0700;
    s->mc_regs[S3C_BANKCON6 >> 2] = 0x18008;
    s->mc_regs[S3C_BANKCON7 >> 2] = 0x18008;
    s->mc_regs[S3C_REFRESH >> 2] = 0xac0000;
    s->mc_regs[S3C_BANKSIZE >> 2] = 0x2;
    s->mc_regs[S3C_MRSRB6 >> 2] = 0x00;
    s->mc_regs[S3C_MRSRB7 >> 2] = 0x00;
}

static uint32_t s3c_mc_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c_mc_state_s *s = (struct s3c_mc_state_s *) opaque;

    switch (addr >> 2) {
    case S3C_BWSCON ... S3C_MRSRB7:
        return s->mc_regs[addr >> 2];
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

static void s3c_mc_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c_mc_state_s *s = (struct s3c_mc_state_s *) opaque;

    switch (addr >> 2) {
    case S3C_BWSCON ... S3C_MRSRB7:
        s->mc_regs[addr >> 2] = value;
        break;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
    }
}

static CPUReadMemoryFunc *s3c_mc_readfn[] = {
    s3c_mc_read,
    s3c_mc_read,
    s3c_mc_read,
};

static CPUWriteMemoryFunc *s3c_mc_writefn[] = {
    s3c_mc_write,
    s3c_mc_write,
    s3c_mc_write,
};

static void s3c_mc_save(QEMUFile *f, void *opaque)
{
    struct s3c_mc_state_s *s = (struct s3c_mc_state_s *) opaque;
    int i;
    for (i = 0; i < 13; i ++)
        qemu_put_be32s(f, &s->mc_regs[i]);
}

static int s3c_mc_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c_mc_state_s *s = (struct s3c_mc_state_s *) opaque;
    int i;
    for (i = 0; i < 13; i ++)
        qemu_get_be32s(f, &s->mc_regs[i]);
    return 0;
}

struct s3c_mc_state_s *s3c_mc_init(target_phys_addr_t base)
{
    struct s3c_mc_state_s *s = (struct s3c_mc_state_s *)qemu_mallocz(sizeof(struct s3c_mc_state_s));
    int iomemtype;
    s->mc_base = base;
    s3c_mc_reset(s);
    iomemtype = cpu_register_io_memory(0, s3c_mc_readfn, s3c_mc_writefn, s);
    cpu_register_physical_memory(s->mc_base, 0xffffff, iomemtype);
    register_savevm("s3c24xx_mc", 0, 0, s3c_mc_save, s3c_mc_load, s);
}