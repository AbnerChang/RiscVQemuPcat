/* 
 *  QEMU RISC-V sample-board support
 *
 *  Author: Sagar Karandikar, skarandikar@berkeley.edu
 *  Originally based on hw/mips/mips_malta.c
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "hw/hw.h"
#include "hw/i386/pc.h"
#include "hw/char/serial.h"
#include "hw/riscv/htif/htif.h"
#include "hw/block/fdc.h"
#include "net/net.h"
#include "hw/boards.h"
#include "hw/i2c/smbus.h"
#include "block/block.h"
#include "hw/block/flash.h"
#include "block/block_int.h" // move later
#include "hw/riscv/riscv.h"
#include "hw/riscv/cpudevs.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_ids.h"
#include "sysemu/char.h"
#include "sysemu/sysemu.h"
#include "sysemu/arch_init.h"
#include "qemu/log.h"
#include "hw/riscv/bios.h"
#include "hw/ide.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/timer/mc146818rtc.h"
#include "hw/timer/i8254.h"
#include "sysemu/blockdev.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"             /* SysBusDevice */
#include "qemu/host-utils.h"
#include "sysemu/qtest.h"
#include "qemu/error-report.h"
#include "hw/empty_slot.h"
#include "hw/usb.h"
//#include "net/net.h"
//#include "hw/boards.h"
#include "sysemu/kvm.h"
#include "hw/kvm/clock.h"
#include "sysemu/blockdev.h"
#include "hw/i2c/smbus.h"
#include "hw/xen/xen.h"
#include "exec/memory.h"
#include "hw/acpi/acpi.h"
#include "cpu.h"
#include "hw/irq.h"

#define TYPE_RISCV_BOARD "riscv-pcat-board"
#define RISCV_BOARD(obj) OBJECT_CHECK(BoardState, (obj), TYPE_RISCV_BOARD)

#define FLASH_MAP_UNIT_MAX 2

/* We don't have a theoretically justifiable exact lower bound on the base
 * address of any flash mapping. In practice, the IO-APIC MMIO range is
 * [0xFEE00000..0xFEE01000[ -- see IO_APIC_DEFAULT_ADDRESS --, leaving free
 * only 18MB-4KB below 4G. For now, restrict the cumulative mapping to 8MB in
 * size.
 */
#define FLASH_MAP_BASE_MIN ((hwaddr)(0x100000000ULL - 8*1024*1024))

void uhci_setup_mmio (MemoryRegion *SysteMemory);

static bool gigabyte_align = true;

typedef struct {
    SysBusDevice parent_obj;
} BoardState;

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;

uint64_t identity_translate(void *opaque, uint64_t addr)
{
    return addr;
}

static int64_t load_kernel (void)
{
    int64_t kernel_entry, kernel_high;
    int big_endian;
    big_endian = 0;

    if (load_elf(loaderparams.kernel_filename, identity_translate, NULL,
                 (uint64_t *)&kernel_entry, NULL, (uint64_t *)&kernel_high,
                 big_endian, ELF_MACHINE, 1) < 0) {
        fprintf(stderr, "qemu: could not load kernel '%s'\n",
                loaderparams.kernel_filename);
        exit(1);
    }
    return kernel_entry;
}

static void main_cpu_reset(void *opaque)
{
    RISCVCPU *cpu = opaque;
    cpu_reset(CPU(cpu));
}

void pc_acpi_smi_interrupt(void *opaque, int irq, int level)
{

}

void gsi_handler(void *opaque, int n, int level)
{
    GSIState *s = opaque;

    if (n < ISA_NUM_IRQS) {
        qemu_set_irq(s->i8259_irq[n], level);
    }
    qemu_set_irq(s->ioapic_irq[n], level);
}
#if 0
static void pic_irq_request(void *opaque, int irq, int level)
{
    CPUState *cs = first_cpu;

    if (level) {
      cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
      cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}
#endif
#if 0
qemu_irq *pc_allocate_cpu_irq(void)
{
    return qemu_allocate_irqs(pic_irq_request, NULL, 1);
}
#endif
#if 0
void pc_basic_device_init(ISABus *isa_bus, qemu_irq *gsi,
                          ISADevice **rtc_state,
                          ISADevice **floppy,
                          bool no_vmport,
                          uint32 hpet_irqs)
{
    int i;
    //DriveInfo *fd[MAX_FD];
    DeviceState *hpet = NULL;
    int pit_isa_irq = 0;
    qemu_irq pit_alt_irq = NULL;
    qemu_irq rtc_irq = NULL;
    //ISADevice *i8042, *port92, *vmmouse;
    ISADevice *pit = NULL;
    //MemoryRegion *ioport80_io = g_new(MemoryRegion, 1);
    //MemoryRegion *ioportF0_io = g_new(MemoryRegion, 1);

//    memory_region_init_io(ioport80_io, NULL, &ioport80_io_ops, NULL, "ioport80", 1);
//    memory_region_add_subregion(isa_bus->address_space_io, 0x80, ioport80_io);

//    memory_region_init_io(ioportF0_io, NULL, &ioportF0_io_ops, NULL, "ioportF0", 1);
//    memory_region_add_subregion(isa_bus->address_space_io, 0xf0, ioportF0_io);

    *rtc_state = rtc_init(isa_bus, 2000, rtc_irq);

//    qemu_register_boot_set(pc_boot_set, *rtc_state);

    if (!xen_enabled()) {
        if (kvm_irqchip_in_kernel()) {
            pit = kvm_pit_init(isa_bus, 0x40);
        } else {
            pit = pit_init(isa_bus, 0x40, pit_isa_irq, pit_alt_irq);
        }
        if (hpet) {
            /* connect PIT to output control line of the HPET */
            qdev_connect_gpio_out(hpet, 0, qdev_get_gpio_in(DEVICE(pit), 0));
        }
        //pcspk_init(isa_bus, pit);
    }

    for(i = 0; i < MAX_SERIAL_PORTS; i++) {
        if (serial_hds[i]) {
            serial_isa_init(isa_bus, i, serial_hds[i]);
        }
    }

    for(i = 0; i < MAX_PARALLEL_PORTS; i++) {
        if (parallel_hds[i]) {
            parallel_init(isa_bus, i, parallel_hds[i]);
        }
    }

 /*   i8042 = isa_create_simple(isa_bus, "i8042");
    i8042_setup_a20_line(i8042, &a20_line[0]);
    if (!no_vmport) {
        vmport_init(isa_bus);
        vmmouse = isa_try_create(isa_bus, "vmmouse");
    } else {
        vmmouse = NULL;
    }
    if (vmmouse) {
        DeviceState *dev = DEVICE(vmmouse);
        qdev_prop_set_ptr(dev, "ps2_mouse", i8042);
        qdev_init_nofail(dev);
    }
    port92 = isa_create_simple(isa_bus, "port92");
    port92_init(port92, &a20_line[1]);

    cpu_exit_irq = qemu_allocate_irqs(cpu_request_exit, NULL, 1);
    DMA_init(0, cpu_exit_irq);*/

/*    for(i = 0; i < MAX_FD; i++) {
        fd[i] = drive_get(IF_FLOPPY, 0, i);
    }
    *floppy = fdctrl_init_isa(isa_bus, fd);*/
}
#endif
//
//static void pcat_init(QEMUMachineInitArgs *args)
//{
/*    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *system_io = get_system_io();
    int i;
    ram_addr_t below_4g_mem_size, above_4g_mem_size;
    PCIBus *pci_bus;
    ISABus *isa_bus;
    PCII440FXState *i440fx_state;
    int piix3_devfn = -1;
    qemu_irq *cpu_irq;
    qemu_irq *gsi;
    qemu_irq *i8259;
    //qemu_irq *smi_irq;
    GSIState *gsi_state;
    //DriveInfo *hd[MAX_IDE_BUS * MAX_IDE_DEVS];
    //BusState *idebus[MAX_IDE_BUS];
    ISADevice *rtc_state;
    ISADevice *floppy;
    MemoryRegion *ram_memory;
    MemoryRegion *pci_memory;
    //DeviceState *icc_bridge;
    //PcGuestInfo *guest_info;

    if (xen_enabled() && xen_hvm_init(&ram_memory) != 0) {
      fprintf(stderr, "xen hardware virtual machine initialisation failed\n");
      printf("qemu riscv: xen hardware virtual machine initialisation failed\n");
      exit(1);
    }*/
    
/*    icc_bridge = qdev_create(NULL, TYPE_ICC_BRIDGE);
    object_property_add_child(qdev_get_machine(), "icc-bridge",
                              OBJECT(icc_bridge), NULL);

    pc_cpus_init(args->cpu_model, icc_bridge);

    if (kvm_enabled() && kvmclock_enabled) {
        kvmclock_create();
    }*/

    /* Check whether RAM fits below 4G (leaving 1/2 GByte for IO memory).
     * If it doesn't, we need to split it in chunks below and above 4G.
     * In any case, try to make sure that guest addresses aligned at
     * 1G boundaries get mapped to host addresses aligned at 1G boundaries.
     * For old machine types, use whatever split we used historically to avoid
     * breaking migration.
     */
/*    if (args->ram_size >= 0xe0000000) {
        ram_addr_t lowmem = gigabyte_align ? 0xc0000000 : 0xe0000000;
        above_4g_mem_size = args->ram_size - lowmem;
        below_4g_mem_size = lowmem;
    } else {
        above_4g_mem_size = 0;
        below_4g_mem_size = args->ram_size;
    }

    //
    // Initial PCI
    //
    pci_memory = g_new(MemoryRegion, 1);
    memory_region_init(pci_memory, NULL, "pci", UINT64_MAX);*/
#if 0
    guest_info = pc_guest_info_init(below_4g_mem_size, above_4g_mem_size);
    guest_info->has_acpi_build = false; // has_acpi_build;
    guest_info->has_pci_info = false; // has_pci_info;
    guest_info->isapc_ram_fw = false;

    if (smbios_type1_defaults) {
        /* These values are guest ABI, do not change */
        smbios_set_type1_defaults("QEMU", "RISC-V on PC/AT (2015)",
                                  args->machine->name);
    }
#endif

/*    gsi_state = g_malloc0(sizeof(*gsi_state));
    gsi = qemu_allocate_irqs(gsi_handler, gsi_state, GSI_NUM_PINS);
    pci_bus = i440fx_init(&i440fx_state, &piix3_devfn, &isa_bus, gsi,
                          system_memory, system_io, args->ram_size,
                          below_4g_mem_size,
                          above_4g_mem_size,
                          pci_memory, ram_memory);
    pci_bus = pci_bus;
    isa_bus_irqs(isa_bus, gsi);

    //
    // Initial 8259
    //
    if (kvm_irqchip_in_kernel()) {
        i8259 = kvm_i8259_init(isa_bus);
    } else if (xen_enabled()) {
        i8259 = xen_interrupt_controller_init();
    } else {
        cpu_irq = pc_allocate_cpu_irq();
        i8259 = i8259_init(isa_bus, cpu_irq[0]);
    }

    for (i = 0; i < ISA_NUM_IRQS; i++) {
        gsi_state->i8259_irq[i] = i8259[i];
    }*/

    //qdev_init_nofail(icc_bridge);
    //pc_register_ferr_irq(gsi[13]);

    //
    // Initial VGA.
    //
    //pc_vga_init(isa_bus, pci_enabled ? pci_bus : NULL);
    //
    // init basic PC hardware.
    //
    //pc_basic_device_init(isa_bus, gsi, &rtc_state, &floppy, xen_enabled(),0x4);
    //pc_nic_init(isa_bus, pci_bus);
#if 0
    ide_drive_get(hd, MAX_IDE_BUS);
    if (pci_enabled) {
        PCIDevice *dev;
        if (xen_enabled()) {
            dev = pci_piix3_xen_ide_init(pci_bus, hd, piix3_devfn + 1);
        } else {
            dev = pci_piix3_ide_init(pci_bus, hd, piix3_devfn + 1);
        }
        idebus[0] = qdev_get_child_bus(&dev->qdev, "ide.0");
        idebus[1] = qdev_get_child_bus(&dev->qdev, "ide.1");
    } else {
        for(i = 0; i < MAX_IDE_BUS; i++) {
            ISADevice *dev;
            char busname[] = "ide.0";
            dev = isa_ide_init(isa_bus, ide_iobase[i], ide_iobase2[i],
                               ide_irq[i],
                               hd[MAX_IDE_DEVS * i], hd[MAX_IDE_DEVS * i + 1]);
            /*
             * The ide bus name is ide.0 for the first bus and ide.1 for the
             * second one.
             */
            busname[4] = '0' + i;
            idebus[i] = qdev_get_child_bus(DEVICE(dev), busname);
        }
    }
#endif

#if 0
    pc_cmos_init(below_4g_mem_size, above_4g_mem_size, args->boot_order,
                 floppy, idebus[0], idebus[1], rtc_state);

    if (pci_enabled && usb_enabled(false)) {
        pci_create_simple(pci_bus, piix3_devfn + 2, "piix3-usb-uhci");
    }

    if (pci_enabled && acpi_enabled) {
        I2CBus *smbus;

        smi_irq = qemu_allocate_irqs(pc_acpi_smi_interrupt, first_cpu, 1);
        /* TODO: Populate SPD eeprom data.  */
        smbus = piix4_pm_init(pci_bus, piix3_devfn + 3, 0xb100,
                              gsi[9], *smi_irq,
                              kvm_enabled(), fw_cfg);
        smbus_eeprom_init(smbus, 8, NULL, 0);
    }

    if (pci_enabled) {
        pc_pci_device_init(pci_bus);
    }
#endif
//}

static bool MapRiscVFlash (char *FlashName, hwaddr phys_addr, hwaddr RomAddress, int FwSize)
{
  int sector_bits, sector_size;
  pflash_t *system_flash;
  MemoryRegion *flash_mem;
  void *fw_file_ptr;
  void *flash_ptr;

  sector_bits = 12;
  sector_size = 1 << sector_bits;
  system_flash = pflash_cfi01_register(
                     phys_addr,
                     NULL /* qdev */,
                     FlashName,
                     FwSize,
                     NULL /*bdrv*/,
                     sector_size,
                     FwSize >> sector_bits,
                     1      /* width */,
                     0x0000 /* id0 */,
                     0x0000 /* id1 */,
                     0x0000 /* id2 */,
                     0x0000 /* id3 */,
                     0      /* be */
                     );
  flash_mem = pflash_cfi01_get_memory(system_flash);
  printf("qemu riscv: system_flash at:%lx\n", flash_mem->addr);
  printf("qemu riscv: system_flash size:%lx\n", flash_mem->size.lo);

  flash_ptr = memory_region_get_ram_ptr(flash_mem);
  fw_file_ptr = rom_ptr(RomAddress);
  printf("qemu riscv: Copy firmware from %lx to %lx\n", (unsigned long int)fw_file_ptr, (unsigned long int)flash_ptr);

  memcpy(flash_ptr,
         fw_file_ptr,
         FwSize);
  return true;
}

static bool IbmPcInit (
    QEMUMachineInitArgs *args,
    hwaddr RomAddress,
    int FwSize,
    MemoryRegion *ram_memory,
    MemoryRegion *system_memory,
    MemoryRegion *system_io,
    qemu_irq *gIrq
    )
{
    hwaddr phys_addr;
    ram_addr_t below_4g_mem_size, above_4g_mem_size;
    char name[64];
    MemoryRegion *pci_memory;
    MemoryRegion *rom_memory;
    PCIBus *pci_bus;
    ISABus *isa_bus;
    PCII440FXState *i440fx_state;
    int piix3_devfn = -1;
    ISADevice *rtc_isa_device;
    qemu_irq rtc_irq = NULL;
    I2CBus *smbus;
    qemu_irq *smi_irq;

    //
    // Initiate firmware device on top of memory which
    // RISC-V can address.
    //
    phys_addr = 0x100000000ULL - FwSize;
    snprintf(name, sizeof name, "system.flash0");
    MapRiscVFlash (name, phys_addr, RomAddress, FwSize);
    phys_addr = 0x100000000ULL - FwSize;
    phys_addr |= 0xffffffff00000000;
    snprintf(name, sizeof name, "system.flash1");
    MapRiscVFlash (name, phys_addr, RomAddress, FwSize);

    //
    // Init basic PC hardware.
    //
    /* Check whether RAM fits below 4G (leaving 1/2 GByte for IO memory).
     * If it doesn't, we need to split it in chunks below and above 4G.
     * In any case, try to make sure that guest addresses aligned at
     * 1G boundaries get mapped to host addresses aligned at 1G boundaries.
     * For old machine types, use whatever split we used historically to avoid
     * breaking migration.
     */
    if (args->ram_size >= 0xe0000000) {
        ram_addr_t lowmem = gigabyte_align ? 0xc0000000 : 0xe0000000;
        above_4g_mem_size = args->ram_size - lowmem;
        below_4g_mem_size = lowmem;
    } else {
        above_4g_mem_size = 0;
        below_4g_mem_size = args->ram_size;
    }
    printf("qemu riscvpcat: Above 4g mem:%lx, below 4g mem:%lx.\n", above_4g_mem_size, below_4g_mem_size);
    pci_memory = g_new(MemoryRegion, 1);
    memory_region_init(pci_memory, NULL, "pci", UINT64_MAX);
    rom_memory = pci_memory;

    rom_memory = rom_memory;

    /*Initial Piix.*/
    pci_bus = i440fx_mmio_init(&i440fx_state, &piix3_devfn, &isa_bus, gIrq,
                          system_memory, system_io, args->ram_size,
                          below_4g_mem_size,
                          above_4g_mem_size,
                          pci_memory, ram_memory);
    if (pci_bus == NULL) {
        printf("qemu riscvpcat: PCI bus init failed.\n");
    } else {
        printf("qemu riscvpcat: PCI bus init successfully DevFun:%x.\n", piix3_devfn);
    }

    /*Add ACPI PCI function*/
    smi_irq = qemu_allocate_irqs(pc_acpi_smi_interrupt, first_cpu, 1);
    smbus = piix4_pm_init(pci_bus, piix3_devfn + 3, 0xb100,
                              gIrq[7], *smi_irq,
                              kvm_enabled(), NULL);
    printf("qemu riscvpcat: PM PCI device init successfully DevFun:%x.\n", piix3_devfn + 3);
    smbus = smbus;

    isa_bus_irqs(isa_bus, gIrq);
    rtc_isa_device = rtc_mmio_init(system_memory, isa_bus, 2015, rtc_irq);
    pc_cmos_init(below_4g_mem_size, above_4g_mem_size, args->boot_order,
            NULL, NULL, NULL, rtc_isa_device);
    printf("qemu riscvpcat: pc_cmos_init init successfully.\n");
    /* Add PCI VGA supprot.*/
    PCIDevice *pciVgadev = pci_vga_init(pci_bus);
    if (pciVgadev != NULL) {
      printf("qemu riscvpcat: pci_vga_init init successfully DevFun: %x.\n", pciVgadev->devfn);
    }
    /* Add EHCI support*/
    if (usb_enabled (true)) {
      //printf("qemu riscvpcat: PCI USB EHCI added on DevFun: 0x1d.\n"); 
      //ehci_create_ich9_with_companions(pci_bus, 0x1d);
      printf("qemu riscvpcat: PCI USB UHCI added on DevFun: %x.\n", piix3_devfn + 2);
      pci_create_simple(pci_bus, piix3_devfn + 2, "piix3-usb-uhci");
      uhci_setup_mmio (system_memory);
      //pci_create_simple(pci_bus, piix3_devfn + 2, "nec-usb-xhci");      
    }
    return TRUE; 
}

static void riscv_board_init(QEMUMachineInitArgs *args)
{
    ram_addr_t ram_size = args->ram_size;
    const char *cpu_model = args->cpu_model;
    const char *kernel_filename = args->kernel_filename;
    const char *kernel_cmdline = args->kernel_cmdline;
    const char *initrd_filename = args->initrd_filename;
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *system_io = get_system_io();
    MemoryRegion *ram_memory;
    MemoryRegion *main_mem = g_new(MemoryRegion, 1);
    RISCVCPU *cpu;
    CPURISCVState *env;
    qemu_irq *gIrq = NULL;
    int i;

    printf("qemu riscvpacat: riscv_board_init\n");
    printf("qemu riscvpacat: Memory size:%lx\n", (long)ram_size);

#ifdef CONFIG_RISCV_HTIF
    //DriveInfo *htifbd_drive;
    //char *htifbd_fname; // htif block device filename
#endif

    if (xen_enabled() && xen_hvm_init(&ram_memory) != 0) {
        fprintf(stderr, "xen hardware virtual machine initialisation failed\n");
        exit(1);
    }
    printf("qemu riscvpacat: xen hvm init successfully.\n");

    DeviceState *dev = qdev_create(NULL, TYPE_RISCV_BOARD);

    object_property_set_bool(OBJECT(dev), true, "realized", NULL);

    /* Make sure the first 3 serial ports are associated with a device. */
    for(i = 0; i < 3; i++) {
        if (!serial_hds[i]) {
            char label[32];
            snprintf(label, sizeof(label), "serial%d", i);
            serial_hds[i] = qemu_chr_new(label, "null", NULL);
        }
    }

    /* init CPUs */
    if (cpu_model == NULL) {
        cpu_model = "riscv-generic";
    }

    for (i = 0; i < smp_cpus; i++) {
        cpu = cpu_riscv_init(cpu_model);
        if (cpu == NULL) {
            fprintf(stderr, "Unable to find CPU definition\n");
            exit(1);
        }
        env = &cpu->env;

        /* Init internal devices */
        gIrq = cpu_riscvpcat_irq_init_cpu(env);
        cpu_riscv_clock_init(env);
        qemu_register_reset(main_cpu_reset, cpu);
    }
    cpu = RISCV_CPU(first_cpu);
    env = &cpu->env;

    /* register system main memory (actual RAM) */
    memory_region_init_ram(main_mem, NULL, "riscv_board.ram", ram_size);
    vmstate_register_ram_global(main_mem);
    memory_region_add_subregion(system_memory, 0x0, main_mem);
    if (bios_name) {
        printf("qemu riscv: BIOS:%s\n", bios_name);
        int bios_size;
        MemoryRegion *bios;
        bios_size = get_image_size(bios_name);
        if (bios_size > 0) {
            bios = g_malloc(sizeof(*bios));
            memory_region_init_ram(bios, NULL, "riscv.fw", bios_size);
            vmstate_register_ram_global(bios);
            memory_region_set_readonly(bios, true);
            if (rom_add_file_fixed(bios_name, 0, -1) != 0) {           
                fprintf(stderr, "qemu: could not load RISC-V firmware '%s'\n", bios_name);
                exit(1);
            }
        }
        IbmPcInit (args, 0, bios_size, ram_memory, system_memory, system_io, gIrq);
    }

    if (kernel_filename) {
        /* Write a small bootloader to the flash location. */
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        load_kernel();
    }

    // write memory amount in MiB to 0x0
    stl_p(memory_region_get_ram_ptr(main_mem), loaderparams.ram_size >> 20);

    // add serial device 0x3f8-0x3ff
    serial_mm_init(system_memory, 0x3f8, 0, env->irq[4], 1843200/16, serial_hds[0],
        DEVICE_NATIVE_ENDIAN);

#ifdef CONFIG_RISCV_HTIF
    // setup HTIF Block Device if one is specified as -hda FILENAME
    /*htifbd_drive = drive_get_by_index(IF_IDE, 0);
    if (NULL == htifbd_drive) {
        htifbd_fname = NULL;
    } else {
        htifbd_fname = (*(htifbd_drive->bdrv)).filename;
    }

    // add htif device 0x400 - 0x410
    htif_mm_init(system_memory, 0x400, env->irq[0], main_mem, htifbd_fname);*/
#else
    /* Create MMIO transports, to which virtio backends created by the
     * user are automatically connected as needed.  If no backend is
     * present, the transport simply remains harmlessly idle.
     * Each memory-mapped region is 0x200 bytes in size.
     */
    sysbus_create_simple("virtio-mmio", 0x400, env->irq[1]);
    sysbus_create_simple("virtio-mmio", 0x600, env->irq[2]);
    sysbus_create_simple("virtio-mmio", 0x800, env->irq[3]);
#endif

    /* Init internal devices */
    cpu_riscvpcat_irq_init_cpu(env);
    cpu_riscv_clock_init(env);
}

static int riscv_board_sysbus_device_init(SysBusDevice *sysbusdev)
{
    return 0;
}

static void riscv_board_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    k->init = riscv_board_sysbus_device_init;
}

static const TypeInfo riscv_board_device = {
    .name          = TYPE_RISCV_BOARD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BoardState),
    .class_init    = riscv_board_class_init,
};

static QEMUMachine riscv_board_machine = {
    .name = "board",
    .desc = "RISCV Board",
    .init = riscv_board_init,
    .max_cpus = 1,
    .is_default = 1,
};

static void riscv_board_register_types(void)
{
    type_register_static(&riscv_board_device);
}

static void riscv_board_machine_init(void)
{
    qemu_register_machine(&riscv_board_machine);
}

type_init(riscv_board_register_types)
machine_init(riscv_board_machine_init);
