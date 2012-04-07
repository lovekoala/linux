#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gfp.h>
#include <linux/bootmem.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/sw_sys.h>
#include <asm/uaccess.h>

#include <asm/clkdev.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/leds.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/icst.h>
#include <asm/hardware/vic.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <mach/clkdev.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/system.h>

#define SW_SID_ADDR(N) (const volatile void __iomem *)(SW_VA_SID_IO_BASE+N*sizeof(int))

static void sw_get_sid(int *a, int *b, int *c, int *d)
{
	*a = readl(SW_SID_ADDR(0));
	*b = readl(SW_SID_ADDR(1));
	*c = readl(SW_SID_ADDR(2));
	*d = readl(SW_SID_ADDR(3));
}

static int read_chip_sid(char* page, char** start, off_t off, int count,
	int* eof, void* data)
{
	int a, b, c, d;
	sw_get_sid(&a, &b, &c, &d);
	return sprintf(page, "%08x-%08x-%08x-%08x\n", a, b, c, d);
}

static int read_chip_version(char* page, char** start, off_t off, int count,
	int* eof, void* data)
{
	enum sw_ic_ver ver = sw_get_ic_ver();

	switch (ver) {
	case MAGIC_VER_A:
		return sprintf(page, "A\n");
	case MAGIC_VER_B:
		return sprintf(page, "B\n");
	case MAGIC_VER_C:
		return sprintf(page, "C\n");
	default:
		return sprintf(page, "%d\n", ver);
	}

	return 0;
}


static int __init platform_proc_init(void)
{
	struct proc_dir_entry *sid_entry = NULL;
	struct proc_dir_entry *version_entry = NULL;

	sid_entry = create_proc_read_entry("sid", 0400,
			NULL, read_chip_sid, NULL);

	if (!sid_entry) {
		pr_err("Create sid at /proc failed\n");
	}

	version_entry = create_proc_read_entry("sw_ver", 0400,
			NULL, read_chip_version, NULL);

	if (!version_entry) {
		pr_err("Create version at /proc failed\n");
	}

	return 0;
}
module_init(platform_proc_init);


