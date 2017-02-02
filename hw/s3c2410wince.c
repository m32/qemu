/*
 * mini2440 development board support
 *
 * Copyright Michel Pollet <buserror@gmail.com>
 *
 * This code is licensed under the GNU GPL v2.
 */

#include "hw.h"
#include "s3c.h"
#include "arm-misc.h"
#include "sysemu.h"
#include "i2c.h"
#include "qemu-timer.h"
#include "devices.h"
#include "audio/audio.h"
#include "boards.h"
#include "console.h"
#include "usb.h"
#include "net.h"
#include "sd.h"
#include "dm9000.h"
#include "eeprom24c0x.h"

#define mini2440_printf(format, ...)	\
    fprintf(stderr, "QEMU %s: " format, __FUNCTION__, ##__VA_ARGS__)

#define MINI2440_GPIO_BACKLIGHT		S3C_GPG(4)
#define MINI2440_GPIO_LCD_RESET		S3C_GPC(6)
#define MINI2440_GPIO_nSD_DETECT	S3C_GPG(8)
#define MINI2440_GPIO_WP_SD			S3C_GPH(8)
#define MINI2440_GPIO_DM9000		S3C_GPF(7)
#define MINI2440_GPIO_USB_PULLUP	S3C_GPC(5)

#define MINI2440_IRQ_nSD_DETECT		S3C_EINT(16)
#define MINI2440_IRQ_DM9000			S3C_EINT(7)

#define FLASH_NOR_SIZE (32*1024*1024)

#define BOOT_NONE	0
#define BOOT_NOR	1
#define BOOT_NAND	2

struct mini2440_board_s {
    struct s3c_state_s *cpu;
    unsigned int ram;
    struct ee24c08_s * eeprom;
    const char * kernel;
    SDState * mmc;
    NANDFlashState *nand;
    pflash_t * nor;
    int bl_level;
    int boot_mode;
};

/*
 * the 24c08 sits on 4 addresses on the bus, and uses the lower address bits
 * to address the 256 byte "page" of the eeprom. We thus need to use 4 i2c_slaves
 * and keep track of which one was used to set the read/write pointer into the data
 */
struct ee24c08_s;
typedef struct ee24cxx_page_s {
	i2c_slave i2c;
	struct ee24c08_s * eeprom;
	uint8_t page;
} ee24cxx_page_s;
typedef struct ee24c08_s {
	/* that memory takes 4 addresses */
	i2c_slave * slave[4];
	uint16_t ptr;
	uint16_t count;
	uint8_t data[1024];
} ee24c08;

static void ee24c08_event(i2c_slave *i2c, enum i2c_event event)
{
    ee24cxx_page_s *s = FROM_I2C_SLAVE(ee24cxx_page_s, i2c);

    if (!s->eeprom)
    	return;

    s->eeprom->ptr = s->page * 256;
    s->eeprom->count = 0;
}

static int ee24c08_tx(i2c_slave *i2c, uint8_t data)
{
    ee24cxx_page_s *s = FROM_I2C_SLAVE(ee24cxx_page_s, i2c);

    if (!s->eeprom)
    	return 0;
    if (s->eeprom->count++ == 0) {
    	/* first byte is address offset */
        s->eeprom->ptr = (s->page * 256) + data;
    } else {
    	mini2440_printf("write %04x=%02x\n", s->eeprom->ptr, data);
    	s->eeprom->data[s->eeprom->ptr] = data;
        s->eeprom->ptr = (s->eeprom->ptr & ~0xff) | ((s->eeprom->ptr + 1) & 0xff);
        s->eeprom->count++;
    }
    return 0;
}

static int ee24c08_rx(i2c_slave *i2c)
{
    ee24cxx_page_s *s = FROM_I2C_SLAVE(ee24cxx_page_s, i2c);
    uint8_t res;
    if (!s->eeprom)
    	return 0;

    res =  s->eeprom->data[s->eeprom->ptr];

    s->eeprom->ptr = (s->eeprom->ptr & ~0xff) | ((s->eeprom->ptr + 1) & 0xff);
    s->eeprom->count++;
    return res;
}

static void ee24c08_save(QEMUFile *f, void *opaque)
{
	struct ee24c08_s *s = (struct ee24c08_s *) opaque;
	int i;

    qemu_put_be16s(f, &s->ptr);
    qemu_put_be16s(f, &s->count);
    qemu_put_buffer(f, s->data, sizeof(s->data));

	for (i = 0; i < 4; i++)
		i2c_slave_save(f, s->slave[i]);
}

static int ee24c08_load(QEMUFile *f, void *opaque, int version_id)
{
	struct ee24c08_s *s = (struct ee24c08_s *) opaque;
	int i;

    qemu_get_be16s(f, &s->ptr);
    qemu_get_be16s(f, &s->count);
    qemu_get_buffer(f, s->data, sizeof(s->data));

	for (i = 0; i < 4; i++)
		i2c_slave_load(f, s->slave[i]);
    return 0;
}

static void ee24c08_page_init(i2c_slave *i2c)
{
	/* nothing to do here */
}

static I2CSlaveInfo ee24c08_info = {
    .init = ee24c08_page_init,
    .event = ee24c08_event,
    .recv = ee24c08_rx,
    .send = ee24c08_tx
};

static void ee24c08_register_devices(void)
{
    i2c_register_slave("24C08", sizeof(ee24cxx_page_s), &ee24c08_info);
}

device_init(ee24c08_register_devices);

static ee24c08 * ee24c08_init(i2c_bus * bus)
{
	ee24c08 *s = qemu_malloc(sizeof(ee24c08));
	int i = 0;

	printf("QEMU: %s\n", __FUNCTION__);

	memset(s->data, 0xff, sizeof(s->data));

	for (i = 0; i < 4; i++) {
		DeviceState *dev = i2c_create_slave(bus, "24C08", 0x50 + i);
		if (!dev) {
			mini2440_printf("failed address %02x\n", 0x50+i);
		}
		s->slave[i] = I2C_SLAVE_FROM_QDEV(dev);
		ee24cxx_page_s *ss = FROM_I2C_SLAVE(ee24cxx_page_s, s->slave[i]);
		ss->page = i;
		ss->eeprom = s;
	}
    register_savevm("ee24c08", -1, 0, ee24c08_save, ee24c08_load, s);
    return s;
}

/* Handlers for output ports */
static void mini2440_bl_switch(void *opaque, int line, int level)
{
	printf("QEMU: %s: LCD Backlight now %s (%d).\n", __FUNCTION__, level ? "on" : "off", level);
}

static void mini2440_bl_intensity(int line, int level, void *opaque)
{
    struct mini2440_board_s *s = (struct mini2440_board_s *) opaque;

    if ((level >> 8) != s->bl_level) {
        s->bl_level = level >> 8;
        printf("%s: LCD Backlight now at %04x\n", __FUNCTION__, level);
    }
}

static void mini2440_gpio_setup(struct mini2440_board_s *s)
{
	/* set the "input" pin values */
	s3c_gpio_set_dat(s->cpu->io, S3C_GPG(13), 1);
	s3c_gpio_set_dat(s->cpu->io, S3C_GPG(14), 1);
	s3c_gpio_set_dat(s->cpu->io, S3C_GPG(15), 0);

    s3c_gpio_out_set(s->cpu->io, MINI2440_GPIO_BACKLIGHT,
                    *qemu_allocate_irqs(mini2440_bl_switch, s, 1));

    s3c_timers_cmp_handler_set(s->cpu->timers, 1, mini2440_bl_intensity, s);

    /* Register the SD card pins to the lower SD driver */
 	sd_set_cb(s->mmc,
 			s3c_gpio_in_get(s->cpu->io)[MINI2440_GPIO_WP_SD],
			qemu_irq_invert(s3c_gpio_in_get(s->cpu->io)[MINI2440_IRQ_nSD_DETECT]));
}

#if 0
static void hexdump(const void* address, uint32_t len)
{
    const unsigned char* p = address;
    int i, j;

    for (i = 0; i < len; i += 16) {
	for (j = 0; j < 16 && i + j < len; j++)
	    fprintf(stderr, "%02x ", p[i + j]);
	for (; j < 16; j++)
	    fprintf(stderr, "   ");
	fprintf(stderr, " ");
	for (j = 0; j < 16 && i + j < len; j++)
	    fprintf(stderr, "%c", (p[i + j] < ' ' || p[i + j] > 0x7f) ? '.' : p[i + j]);
	fprintf(stderr, "\n");
    }
}
#endif

static int mini2440_load_from_nand(NANDFlashState *nand,
		uint32_t nand_offset, uint32_t s3c_base_offset, uint32_t size)
{
	uint8_t buffer[512];
	uint32_t src = 0;
	int page = 0;
	uint32_t dst = 0;

	if (!nand)
		return 0;

	for (page = 0; page < (size / 512); page++, src += 512 + 16, dst += 512) {
		if (nand_readraw(nand, nand_offset + src, buffer, 512)) {
			cpu_physical_memory_write(s3c_base_offset + dst, buffer, 512);
		} else {
			mini2440_printf("failed to load nand %d:%d\n",
			        nand_offset + src, 512 + 16);
			return 0;
		}
	}
	return (int) size;
}

/******************************************************************************
 * OAL (OEM Adaptation Layer) arguments structure definitions
 * OAL arguments are used to pass initialization data to the Windows CE kernel
 * These definitions must match what's expected by DeviceEmulator images
 *****************************************************************************/
enum defines_bsp_50 {
    RAM_BANK0_START = 0x30000000,
    RAM_BANK0_SIZE = (64 << 20),

    RAM_BANK1_START = (RAM_BANK0_START + RAM_BANK0_SIZE),
    RAM_BANK1_SIZE = (192 << 20),

    RAM_OFFSET_SP = 0x10000,
    RAM_OFFSET_BSP_ARGS = 0x20000,
    RAM_OFFSET_EBOOT = 0x21000,

    INITIAL_SP = RAM_BANK0_START + RAM_OFFSET_SP,
    BSP_ARGS_PTR = RAM_BANK0_START + RAM_OFFSET_BSP_ARGS,
};

enum {
  BSP_SCREEN_SIGNATURE = 0xde12de34,
  OAL_ARGUMENTS_SIGNATURE = 0x53475241,
  OAL_ARGUMENTS_VERSION = 1,
};

#define PACKED __attribute__((packed))

typedef struct oal_args_hdr_t {
  uint32_t magic;
  uint16_t version_major;
  uint16_t version_minor;
} PACKED oal_args_hdr_t;

typedef struct _oal_dev_location_t {
  int32_t iface_type;
  int32_t bus_number;
  int32_t logical_location;
  int32_t ptr_location;
  int32_t irq_number;
} PACKED oal_dev_location_t;

typedef struct oal_kitl_args_t {
  uint32_t flags;
  oal_dev_location_t location;
  union {
    struct {
      uint32_t baud_rate;
      uint32_t data_bits;
      uint32_t stop_bits;
      uint32_t parity;
    };
    struct {
      uint16_t mac[3];
      uint32_t ip_addr;
      uint32_t ip_mask;
      uint32_t ip_route;
    };
  };
} PACKED oal_kitl_args_t;

typedef struct wince_bsp_args_t {
  oal_args_hdr_t header;
  uint8_t serial[16];
  oal_kitl_args_t kitl;
  uint32_t screen_magic;
  uint16_t screen_width;
  uint16_t screen_height;
  uint16_t screen_bits_pp;
  uint16_t emulator_flags;
  uint8_t screen_rotation;
  uint8_t padding[15];

  uint32_t in_update_mode;
  uint32_t ram_disk_size;
  uint32_t ram_disk_addr;
} PACKED wince_bsp_args_t;

static void mini2440_wince_fixup(struct mini2440_board_s *s) {
    mini2440_printf("%s\n", __func__);
	wince_bsp_args_t args = {
        .header = {
            .magic = OAL_ARGUMENTS_SIGNATURE,
            .version_major = 1,
			.version_minor = 1,
        },

        .screen_magic = BSP_SCREEN_SIGNATURE,
        .screen_width = 320,
        .screen_height = 320,
        .screen_bits_pp = 16,

		.emulator_flags = 1,
    };

	memcpy(qemu_get_ram_ptr(RAM_OFFSET_BSP_ARGS), &args, sizeof(args));
}

static uint32_t GetCEKernelOffset(const char *filename)
{
	uint32_t offset = 0;
	FILE *fin = NULL;
	uint32_t hdr[3] = {};

	fin = fopen(filename, "rb");
	if (!fin)
	{
		fprintf(stderr, "%s: failed to open CE kernel file '%s'\n", __func__, filename);
		goto fail;
	}

	if (fseek(fin, 0x40, SEEK_SET))
	{
		fprintf(stderr, "%s: failed to seek to CE offset structure\n", __func__);
		goto fail;
	}

	if (fread(hdr, 3 * sizeof(uint32_t), 1, fin) != 1)
	{
		fprintf(stderr, "%s: failed to read CE kernel header\n", __func__);
		goto fail;
	}

	if (hdr[0] != 0x43454345) {
		fprintf(stderr, "%s: ECEC marker not found <%x>\n", __func__, hdr[0]);
		goto fail;
	}

	offset = hdr[1] - hdr[2];
	if ((offset < 0x80000000) || (offset >= 0x88000000))
	{
		fprintf(stderr, "%s: offset not in RAM range\n", __func__);
		goto fail;
	}
	offset -= 0x80000000;
	goto done;

fail:
	offset = RAM_OFFSET_EBOOT;
	fprintf(stderr, "%s: failed to read kernel offset, using %x\n", __func__, offset);

done:
	if (fin) {
		fclose(fin);
	}
	return offset;
}

static void mini2440_reset(void *opaque)
{
    struct mini2440_board_s *s = (struct mini2440_board_s *) opaque;
    int32_t image_size;

    mini2440_wince_fixup(s);
	s->cpu->env->regs[13] = RAM_BANK0_START | RAM_OFFSET_SP;
	s->cpu->env->regs[15] = 0x00000;

	fprintf(stderr, "%s:%d\n", __func__, __LINE__);

	/*
	 * if a kernel was explicitly specified, we load it too
	 */
	if (s->kernel) {
		uint32_t CEKernelOffset = GetCEKernelOffset(s->kernel);

	   	image_size = load_image(s->kernel, qemu_get_ram_ptr(CEKernelOffset));
        if (image_size <= 0) {
            mini2440_printf("failed to load kernel\n");
            goto fail;
        }
        if (image_size & (512 -1)) {
            /* round size to a NAND block size */
            image_size = (image_size + 512) & ~(512-1);
        }
        mini2440_printf("loaded kernel %s (size %x) offset=%x\n", s->kernel, image_size, CEKernelOffset);
        s->cpu->env->regs[15] = RAM_BANK0_START | CEKernelOffset;
	}

    return;
fail:
    mini2440_printf("FAIL\n");
    abort();
}

/* Typical touchscreen calibration values */
static const int mini2440_ts_scale[6] = {
    0, (90 - 960) * 256 / 1021, -90 * 256 * 32,
    (940 - 75) * 256 / 1021, 0, 75 * 256 * 32,
};

static void mini2440_init(ram_addr_t ram_size, const char *boot_device,
                          const char *kernel_filename,
                          const char *kernel_cmdline,
                          const char *initrd_filename, const char *cpu_model) {
  struct mini2440_board_s *mini =
      (struct mini2440_board_s *)qemu_mallocz(sizeof(struct mini2440_board_s));
  int sd_idx = drive_get_index(IF_SD, 0, 0);
  int nor_idx = drive_get_index(IF_PFLASH, 0, 0);
  int nand_idx = drive_get_index(IF_MTD, 0, 0);
  int nand_cid = 0x76; // 128MB flash == 0xf1

  mini->ram = 0x10000000;
  mini->kernel = kernel_filename;
  mini->mmc = sd_idx >= 0 ? sd_init(drives_table[sd_idx].bdrv, 0) : NULL;
  mini->boot_mode = BOOT_NONE;

  uint32_t sram_base = 0;
  /*
   * Use an environment variable to set the boot mode "switch"
   */
  const char *boot_mode = getenv("MINI2440_BOOT");

  if (boot_mode) {
    if (!strcasecmp(boot_mode, "nor")) {
      if (nor_idx < 0) {
        printf("%s MINI2440_BOOT(nor) error, no flash file specified",
               __func__);
        abort();
      }
      mini->boot_mode = BOOT_NOR;
    } else if (!strcasecmp(boot_mode, "nand")) {
      if (nor_idx < 0) {
        printf("%s MINI2440_BOOT(nand) error, no flash file specified",
               __func__);
        abort();
      }
      mini->boot_mode = BOOT_NAND;
    } else {
      printf("%s MINI2440_BOOT(%s) ignored, invalid value", __func__,
             boot_mode);
    }
  }

  /* Check the boot mode */
  switch (mini->boot_mode) {
  case BOOT_NAND: {
    mini2440_printf("BOOT MODE: NAND\n");
    sram_base = S3C_SRAM_BASE_NANDBOOT;
    int size = bdrv_getlength(drives_table[nand_idx].bdrv);
    switch (size) {
    case 2 * 65536 * (512 + 16):
      nand_cid = 0x76;
      break;
    case 4 * 65536 * (512 + 16):
      nand_cid = 0xf1;
      break;
    default:
      printf("%s: Unknown NAND size/id %d (%dMB) defaulting to old 64MB\n",
             __func__, size, ((size / (512 + 16)) * 512) / 1024 / 1024);
      break;
    }
  } break;
  case BOOT_NOR: {
    mini2440_printf("BOOT MODE: NOR\n");
    sram_base = S3C_SRAM_BASE_NORBOOT;
    int nor_size = bdrv_getlength(drives_table[nor_idx].bdrv);
    if (nor_size > FLASH_NOR_SIZE) {
      printf("%s: file too big (max=%x)\n", __func__, FLASH_NOR_SIZE);
	  nor_size = FLASH_NOR_SIZE;
    }
    printf("%s: Register parallel flash %d size 0x%x '%s'\n", __func__, nor_idx,
           nor_size, bdrv_get_device_name(drives_table[nor_idx].bdrv));
  } break;
  default: { mini2440_printf("BOOT MODE: UNKNOWN\n"); }
  }

  mini->cpu = s3c24xx_init(S3C_CPU_2410, 12000000 /* 12 mhz */, mini->ram,
                           sram_base, mini->mmc);

  /* Setup peripherals */
  mini2440_gpio_setup(mini);

	mini->eeprom = ee24c08_init(s3c_i2c_bus(mini->cpu->i2c));

	{
		NICInfo* nd;
		nd = &nd_table[0];
		if (!nd->model)
		    nd->model = "dm9000";
		if (strcmp(nd->model, "dm9000") == 0) {
			dm9000_init(nd, 0x20000000, 0x300, 0x304, s3c_gpio_in_get(mini->cpu->io)[MINI2440_IRQ_DM9000]);
		}
	}

  s3c_adc_setscale(mini->cpu->adc, mini2440_ts_scale);


  /* Setup NAND memory */
  //if (boot_mode == BOOT_NAND) {
      mini->nand = nand_init(NAND_MFR_SAMSUNG, 0x76);
      mini->cpu->nand->reg(mini->cpu->nand, mini->nand);
  //}

  mini->nor =
      pflash_cfi02_register(0, qemu_ram_alloc(FLASH_NOR_SIZE),
                            nor_idx != -1 ? drives_table[nor_idx].bdrv : NULL,
                            (64 * 1024), /* sector_len */
							FLASH_NOR_SIZE >> 16, /* num_blocks */
							1, /* nb_mappings */
							2, /* width */
							0x0001, 0x225b, 0x0000, 0x0000,
							0x5555, 0x2aaa);

  /* Setup initial (reset) machine state */
  fprintf(stderr, "%s: qemu_register_reset mini2440_reset\n", __func__);
  qemu_register_reset(mini2440_reset, mini);
   mini2440_reset(mini);
}

QEMUMachine s3cwince_machine = {
    "s3cwince",
    "S3C2410 Device Emulator for Windows Mobile 5.0",
    .init = mini2440_init,
};

