#include "hw/hw.h"
#include "hw/i386/pc.h"
#include "hw/riscv/cpudevs.h"
#include "cpu.h"
#include "hw/timer/mc146818rtc.h"
#include "hw/ide.h"
#include "hw/block/fdc.h"

/* setup pci memory address space mapping into system address space */
void pc_pci_as_mapping_init(Object *owner, MemoryRegion *system_memory,
                            MemoryRegion *pci_address_space)
{
    /* Set to lower priority than RAM */
    memory_region_add_subregion_overlap(system_memory, 0x0,
                                        pci_address_space, -1);
}

void cpu_smm_register(cpu_set_smm_t callback, void *arg)
{

}

/* PC cmos mappings */

#define REG_EQUIPMENT_BYTE          0x14

static int cmos_get_fd_drive_type(FDriveType fd0)
{
    int val;

    switch (fd0) {
    case FDRIVE_DRV_144:
        /* 1.44 Mb 3"5 drive */
        val = 4;
        break;
    case FDRIVE_DRV_288:
        /* 2.88 Mb 3"5 drive */
        val = 5;
        break;
    case FDRIVE_DRV_120:
        /* 1.2 Mb 5"5 drive */
        val = 2;
        break;
    case FDRIVE_DRV_NONE:
    default:
        val = 0;
        break;
    }
    return val;
}

static void cmos_init_hd(ISADevice *s, int type_ofs, int info_ofs,
                         int16_t cylinders, int8_t heads, int8_t sectors)
{
    rtc_set_memory(s, type_ofs, 47);
    rtc_set_memory(s, info_ofs, cylinders);
    rtc_set_memory(s, info_ofs + 1, cylinders >> 8);
    rtc_set_memory(s, info_ofs + 2, heads);
    rtc_set_memory(s, info_ofs + 3, 0xff);
    rtc_set_memory(s, info_ofs + 4, 0xff);
    rtc_set_memory(s, info_ofs + 5, 0xc0 | ((heads > 8) << 3));
    rtc_set_memory(s, info_ofs + 6, cylinders);
    rtc_set_memory(s, info_ofs + 7, cylinders >> 8);
    rtc_set_memory(s, info_ofs + 8, sectors);
}

/* convert boot_device letter to something recognizable by the bios */
//static int boot_device2nibble(char boot_device)
//{
//    switch(boot_device) {
//    case 'a':
//    case 'b':
//        return 0x01; /* floppy boot */
//    case 'c':
//        return 0x02; /* hard drive boot */
//    case 'd':
//        return 0x03; /* CD-ROM boot */
//    case 'n':
//        return 0x04; /* Network boot */
//    }
//    return 0;
//}

/*
static int set_boot_dev(ISADevice *s, const char *boot_device)
{
#define PC_MAX_BOOT_DEVICES 3
    int nbds, bds[3] = { 0, };
    int i;

    nbds = strlen(boot_device);
    if (nbds > PC_MAX_BOOT_DEVICES) {
        return(1);
    }
    for (i = 0; i < nbds; i++) {
        bds[i] = boot_device2nibble(boot_device[i]);
        if (bds[i] == 0) {
            return(1);
        }
    }
    rtc_set_memory(s, 0x3d, (bds[1] << 4) | bds[0]);
    rtc_set_memory(s, 0x38, (bds[2] << 4) | (fd_bootchk ? 0x0 : 0x1));
    return(0);
}*/


typedef struct pc_cmos_init_late_arg {
    ISADevice *rtc_state;
    BusState *idebus[2];
} pc_cmos_init_late_arg;

typedef struct RTCCPUHotplugArg {
    Notifier cpu_added_notifier;
    ISADevice *rtc_state;
} RTCCPUHotplugArg;

static void rtc_notify_cpu_added(Notifier *notifier, void *data)
{
    RTCCPUHotplugArg *arg = container_of(notifier, RTCCPUHotplugArg,
                                         cpu_added_notifier);
    ISADevice *s = arg->rtc_state;

    /* increment the number of CPUs */
    rtc_set_memory(s, 0x5f, rtc_get_memory(s, 0x5f) + 1);
}


static void pc_cmos_init_late(void *opaque)
{
    pc_cmos_init_late_arg *arg = opaque;
    ISADevice *s = arg->rtc_state;
    int16_t cylinders;
    int8_t heads, sectors;
    int val;
    int i, trans;

    val = 0;
    if (ide_get_geometry(arg->idebus[0], 0,
                         &cylinders, &heads, &sectors) >= 0) {
        cmos_init_hd(s, 0x19, 0x1b, cylinders, heads, sectors);
        val |= 0xf0;
    }
    if (ide_get_geometry(arg->idebus[0], 1,
                         &cylinders, &heads, &sectors) >= 0) {
        cmos_init_hd(s, 0x1a, 0x24, cylinders, heads, sectors);
        val |= 0x0f;
    }
    rtc_set_memory(s, 0x12, val);

    val = 0;
    for (i = 0; i < 4; i++) {
        /* NOTE: ide_get_geometry() returns the physical
           geometry.  It is always such that: 1 <= sects <= 63, 1
           <= heads <= 16, 1 <= cylinders <= 16383. The BIOS
           geometry can be different if a translation is done. */
        if (ide_get_geometry(arg->idebus[i / 2], i % 2,
                             &cylinders, &heads, &sectors) >= 0) {
            trans = ide_get_bios_chs_trans(arg->idebus[i / 2], i % 2) - 1;
            assert((trans & ~3) == 0);
            val |= trans << (i * 2);
        }
    }
    rtc_set_memory(s, 0x39, val);

    qemu_unregister_reset(pc_cmos_init_late, opaque);
}


void pc_cmos_init(ram_addr_t ram_size, ram_addr_t above_4g_mem_size,
                  const char *boot_device,
                  ISADevice *floppy, BusState *idebus0, BusState *idebus1,
                  ISADevice *s)
{
    int val, nb, i;
    FDriveType fd_type[2] = { FDRIVE_DRV_NONE, FDRIVE_DRV_NONE };
    static pc_cmos_init_late_arg arg;
    static RTCCPUHotplugArg cpu_hotplug_cb;

    /* various important CMOS locations needed by PC/Bochs bios */

    /* memory size */
    /* base memory (first MiB) */
    val = MIN(ram_size / 1024, 640);
    rtc_set_memory(s, 0x15, val);
    rtc_set_memory(s, 0x16, val >> 8);
    /* extememorynded memory (next 64MiB) */
    if (ram_size > 1024 * 1024) {
        val = (ram_size - 1024 * 1024) / 1024;
    } else {
        val = 0;
    }
    if (val > 65535)
        val = 65535;
    rtc_set_memory(s, 0x17, val);
    rtc_set_memory(s, 0x18, val >> 8);
    rtc_set_memory(s, 0x30, val);
    rtc_set_memory(s, 0x31, val >> 8);
    /* memory between 16MiB and 4GiB */
    if (ram_size > 16 * 1024 * 1024) {
        val = (ram_size - 16 * 1024 * 1024) / 65536;
    } else {
        val = 0;
    }
    if (val > 65535)
        val = 65535;
    rtc_set_memory(s, 0x34, val);
    rtc_set_memory(s, 0x35, val >> 8);
    /* memory above 4GiB */
    val = above_4g_mem_size / 65536;
    rtc_set_memory(s, 0x5b, val);
    rtc_set_memory(s, 0x5c, val >> 8);
    rtc_set_memory(s, 0x5d, val >> 16);
    printf("qemu riscvpcat: CMOS- memory init successfully.\n");

    /* set the number of CPU */
    rtc_set_memory(s, 0x5f, smp_cpus - 1);
    /* init CPU hotplug notifier */
    cpu_hotplug_cb.rtc_state = s;
    cpu_hotplug_cb.cpu_added_notifier.notify = rtc_notify_cpu_added;
    qemu_register_cpu_added_notifier(&cpu_hotplug_cb.cpu_added_notifier);
    printf("qemu riscvpcat: CMOS- cpu init successfully.\n");

    //if (set_boot_dev(s, boot_device)) {
    //    exit(1);
    //}
    //printf("qemu riscvpcat: CMOS- boot device init successfully.\n");

    /* floppy type */
    if (floppy) {
        for (i = 0; i < 2; i++) {
    //        fd_type[i] = isa_fdc_get_drive_type(floppy, i);
        }
    }
    val = (cmos_get_fd_drive_type(fd_type[0]) << 4) |
        cmos_get_fd_drive_type(fd_type[1]);
    rtc_set_memory(s, 0x10, val);

    val = 0;
    nb = 0;
    if (fd_type[0] < FDRIVE_DRV_NONE) {
        nb++;
    }
    if (fd_type[1] < FDRIVE_DRV_NONE) {
        nb++;
    }
    switch (nb) {
    case 0:
        break;
    case 1:
        val |= 0x01; /* 1 drive, ready for boot */
        break;
    case 2:
        val |= 0x41; /* 2 drives, ready for boot */
        break;
    }
    val |= 0x02; /* FPU is there */
    val |= 0x04; /* PS/2 mouse installed */
    rtc_set_memory(s, REG_EQUIPMENT_BYTE, val);
    printf("qemu riscvpcat: floppy init successfully.\n");

    return;

    //
    // (AC) Below causes exception. check it later...
    //

    /* hard drives */
    arg.rtc_state = s;
    arg.idebus[0] = idebus0;
    arg.idebus[1] = idebus1;
    qemu_register_reset(pc_cmos_init_late, &arg);
    printf("qemu riscvpcat: CMOS- hard drives init successfully.\n");
}

