/*
 * Generic Loongson-3 Platform support
 *
 * Copyright (c) 2016-2020 Huacai Chen (chenhc@lemote.com)
 * This code is licensed under the GNU GPL v2.
 *
 * Contributions are licensed under the terms of the GNU GPL,
 * version 2 or (at your option) any later version.
 */

/*
 * Generic virtualized PC Platform for Loongson-3 CPU.
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "cpu.h"
#include "elf.h"
#include "hw/boards.h"
#include "hw/char/serial.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "hw/mips/loongson3-virt.h"
#include "hw/empty_slot.h"
#include "hw/intc/i8259.h"
#include "hw/loader.h"
#include "hw/isa/superio.h"
#include "hw/pci/msi.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_host.h"
#include "hw/pci-host/gpex.h"
#include "hw/rtc/mc146818rtc.h"
#include "hw/usb.h"
#include "net/net.h"
#include "exec/address-spaces.h"
#include "sysemu/kvm.h"
#include "sysemu/qtest.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "qemu/log.h"
#include "qemu/error-report.h"

#define PM_CNTL_MODE          0x10


/* Loongson-3 has a 2MB flash rom */
#define LOONGSON_MAX_VCPUS      16

#define LOONGSON3_BIOSNAME "bios_loongson3.bin"

#define PCIE_IRQ_BASE       3

#define align(x) (((x) + 63) & ~63)

static const struct MemmapEntry virt_memmap[] = {
    [VIRT_LOWMEM] =      { 0x00000000,    0x10000000 },
    [VIRT_PM] =          { 0x10080000,         0x100 },
    [VIRT_FW_CFG] =      { 0x10080100,         0x100 },
    [VIRT_ISA_PIO] =     { 0x18000000,       0xc0000 },
    [VIRT_PCIE_ECAM] =   { 0x1a000000,     0x2000000 },
    [VIRT_BIOS_ROM] =    { 0x1fc00000,      0x200000 },
    [VIRT_UART] =        { 0x1fe001e0,           0x8 },
    [VIRT_PCIE_MMIO] =   { 0x40000000,    0x40000000 },
    [VIRT_HIGHMEM] =     { 0x80000000,           0x0 }, /* Variable */
};


static const struct MemmapEntry loader_memmap[] = {
    [LOADER_KERNEL] =    { 0x00000000,     0x4000000 },
    [LOADER_INITRD] =    {  0x4000000,           0x0 }, /* Variable */
    [LOADER_CMDLINE] =   { 0x0ff00000,      0x100000 },
};

static const struct MemmapEntry loader_rommap[] = {
    [LOADER_BOOTROM] =   { 0x1fc00000,        0x1000 },
    [LOADER_PARAM] =     { 0x1fc01000,       0x10000 },
};

static struct _loaderparams {
    uint64_t ram_size;
    const char *kernel_cmdline;
    const char *kernel_filename;
    const char *initrd_filename;
    uint64_t kernel_entry;
    uint64_t a0, a1, a2;
} loaderparams;

static uint64_t loongson3_pm_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void loongson3_pm_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    if (addr != PM_CNTL_MODE) {
        return;
    }

    switch (val) {
    case 0x00:
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        return;
    case 0xff:
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        return;
    default:
        return;
    }
}

static const MemoryRegionOps loongson3_pm_ops = {
    .read  = loongson3_pm_read,
    .write = loongson3_pm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static struct efi_memory_map_loongson *init_memory_map(void *g_map)
{
    struct efi_memory_map_loongson *emap = g_map;

    emap->nr_map = 2;
    emap->mem_freq = 300000000;

    emap->map[0].node_id = 0;
    emap->map[0].mem_type = 1;
    emap->map[0].mem_start = 0x0;
    emap->map[0].mem_size = (loaderparams.ram_size > 0x10000000
                            ? 256 : (loaderparams.ram_size >> 20)) - 16;

    emap->map[1].node_id = 0;
    emap->map[1].mem_type = 2;
    emap->map[1].mem_start = 0x90000000;
    emap->map[1].mem_size = (loaderparams.ram_size > 0x10000000
                            ? (loaderparams.ram_size >> 20) - 256 : 0);

    return emap;
}

static uint32_t get_cpu_freq(void)
{
    /* FIXME: Detect via CPUCFG or IOCTL for KVM */

    /*
    * TCG has a default CP0 Timer period 10 ns, which is 100MHz,
    * CPU frequency is defined as double of CP0 timer frequency.
    */
    return 200000000;
}

static struct efi_cpuinfo_loongson *init_cpu_info(void *g_cpuinfo_loongson)
{
    struct efi_cpuinfo_loongson *c = g_cpuinfo_loongson;

    c->cputype = Loongson_3A;
    c->processor_id = MIPS_CPU(first_cpu)->env.CP0_PRid;
    c->cpu_clock_freq = get_cpu_freq();

    c->cpu_startup_core_id = 0;
    c->nr_cpus = current_machine->smp.cpus;
    c->total_node = (current_machine->smp.cpus + 3) / 4;

    return c;
}

static struct system_loongson *init_system_loongson(void *g_system)
{
    struct system_loongson *s = g_system;

    s->ccnuma_smp = 0;
    s->sing_double_channel = 1;
    s->nr_uarts = 1;
    s->uarts[0].iotype = 2;
    s->uarts[0].int_offset = 2;
    s->uarts[0].uartclk = 25000000; /* Random value */
    s->uarts[0].uart_base = virt_memmap[VIRT_UART].base;

    return s;
}

static struct irq_source_routing_table *init_irq_source(void *g_irq_source)
{
    struct irq_source_routing_table *irq_info = g_irq_source;

    irq_info->node_id = 0;
    irq_info->PIC_type = 0;
    irq_info->dma_mask_bits = 64;
    irq_info->pci_mem_start_addr = virt_memmap[VIRT_PCIE_MMIO].base;
    irq_info->pci_mem_end_addr   = virt_memmap[VIRT_PCIE_MMIO].base +
                                   virt_memmap[VIRT_PCIE_MMIO].size - 1;
    irq_info->pci_io_start_addr  = virt_memmap[VIRT_ISA_PIO].base;

    return irq_info;
}

static struct interface_info *init_interface_info(void *g_interface)
{
    struct interface_info *interface = g_interface;

    interface->vers = 0x01;
    strcpy(interface->description, "UEFI_Version_v1.0");

    return interface;
}

static struct board_devices *board_devices_info(void *g_board)
{
    struct board_devices *bd = g_board;

    strcpy(bd->name, "Loongson-3A-VIRT-1w-V1.00-demo");

    return bd;
}

static struct loongson_special_attribute *init_special_info(void *g_special)
{
    struct loongson_special_attribute *special = g_special;

    strcpy(special->special_name, "2016-05-16");

    return special;
}

static void init_loongson_params(struct loongson_params *lp, void *p)
{
    lp->memory_offset = (unsigned long long)init_memory_map(p)
                        - (unsigned long long)lp;
    p += align(sizeof(struct efi_memory_map_loongson));

    lp->cpu_offset = (unsigned long long)init_cpu_info(p)
                     - (unsigned long long)lp;
    p += align(sizeof(struct efi_cpuinfo_loongson));

    lp->system_offset = (unsigned long long)init_system_loongson(p)
                        - (unsigned long long)lp;
    p += align(sizeof(struct system_loongson));

    lp->irq_offset = (unsigned long long)init_irq_source(p)
                     - (unsigned long long)lp;
    p += align(sizeof(struct irq_source_routing_table));

    lp->interface_offset = (unsigned long long)init_interface_info(p)
                           - (unsigned long long)lp;
    p += align(sizeof(struct interface_info));

    lp->boarddev_table_offset = (unsigned long long)board_devices_info(p)
                                - (unsigned long long)lp;
    p += align(sizeof(struct board_devices));

    lp->special_offset = (unsigned long long)init_special_info(p)
                         - (unsigned long long)lp;
    p += align(sizeof(struct loongson_special_attribute));
}

static void init_reset_system(struct efi_reset_system_t *reset)
{
    reset->Shutdown = 0xffffffffbfc000a8;
    reset->ResetCold = 0xffffffffbfc00080;
    reset->ResetWarm = 0xffffffffbfc00080;
}

static void init_boot_param(void)
{
    void *p;
    struct boot_params *bp;

    p = g_malloc0(loader_rommap[LOADER_PARAM].size);
    bp = p;

    bp->efi.smbios.vers = 1;
    init_reset_system(&(bp->reset_system));
    p += align(sizeof(struct boot_params));
    init_loongson_params(&(bp->efi.smbios.lp), p);

    rom_add_blob_fixed("params_rom", bp,
                       loader_rommap[LOADER_PARAM].size,
                       loader_rommap[LOADER_PARAM].base);

    g_free(bp);

    loaderparams.a2 = cpu_mips_phys_to_kseg0(NULL,
                                             loader_rommap[LOADER_PARAM].base);
}

static void init_boot_rom(void)
{
    const unsigned int boot_rom_code[] = {
        0x40086000,   /* mfc0    t0, CP0_STATUS                                   */
        0x240900E4,   /* li      t1, 0xe4         #set kx, sx, ux, erl            */
        0x01094025,   /* or      t0, t0, t1                                       */
        0x3C090040,   /* lui     t1, 0x40         #set bev                        */
        0x01094025,   /* or      t0, t0, t1                                       */
        0x40886000,   /* mtc0    t0, CP0_STATUS                                   */
        0x00000000,
        0x40806800,   /* mtc0    zero, CP0_CAUSE                                  */
        0x00000000,
        0x400A7801,   /* mfc0    t2, $15, 1                                       */
        0x314A00FF,   /* andi    t2, 0x0ff                                        */
        0x3C089000,   /* dli     t0, 0x900000003ff01000                           */
        0x00084438,
        0x35083FF0,
        0x00084438,
        0x35081000,
        0x314B0003,   /* andi    t3, t2, 0x3      #local cpuid                    */
        0x000B5A00,   /* sll     t3, 8                                            */
        0x010B4025,   /* or      t0, t0, t3                                       */
        0x314C000C,   /* andi    t4, t2, 0xc      #node id                        */
        0x000C62BC,   /* dsll    t4, 42                                           */
        0x010C4025,   /* or      t0, t0, t4                                       */
                    /* waitforinit:                                             */
        0xDD020020,   /* ld      v0, FN_OFF(t0)   #FN_OFF 0x020                   */
        0x1040FFFE,   /* beqz    v0, waitforinit                                  */
        0x00000000,   /* nop                                                      */
        0xDD1D0028,   /* ld      sp, SP_OFF(t0)   #FN_OFF 0x028                   */
        0xDD1C0030,   /* ld      gp, GP_OFF(t0)   #FN_OFF 0x030                   */
        0xDD050038,   /* ld      a1, A1_OFF(t0)   #FN_OFF 0x038                   */
        0x00400008,   /* jr      v0               #byebye                         */
        0x00000000,   /* nop                                                      */
        0x1000FFFF,   /* 1:  b   1b                                               */
        0x00000000,   /* nop                                                      */

                    /* Reset                                                    */
        0x3C0C9000,   /* dli     t0, 0x9000000010080010                           */
        0x358C0000,
        0x000C6438,
        0x358C1008,
        0x000C6438,
        0x358C0010,
        0x240D0000,   /* li      t1, 0x00                                         */
        0xA18D0000,   /* sb      t1, (t0)                                         */
        0x1000FFFF,   /* 1:  b   1b                                               */
        0x00000000,   /* nop                                                      */

                    /* Shutdown                                                 */
        0x3C0C9000,   /* dli     t0, 0x9000000010080010                           */
        0x358C0000,
        0x000C6438,
        0x358C1008,
        0x000C6438,
        0x358C0010,
        0x240D00FF,   /* li      t1, 0xff                                         */
        0xA18D0000,   /* sb      t1, (t0)                                         */
        0x1000FFFF,   /* 1:  b   1b                                               */
        0x00000000    /* nop                                                      */
    };

    rom_add_blob_fixed("boot_rom",
                        boot_rom_code, sizeof(boot_rom_code),
                        loader_rommap[LOADER_BOOTROM].base);
}

static void fw_cfg_boot_set(void *opaque, const char *boot_device,
                            Error **errp)
{
    fw_cfg_modify_i16(opaque, FW_CFG_BOOT_DEVICE, boot_device[0]);
}

static void fw_conf_init(unsigned long ram_size)
{
    FWCfgState *fw_cfg;
    hwaddr cfg_addr = virt_memmap[VIRT_FW_CFG].base;

    fw_cfg = fw_cfg_init_mem_wide(cfg_addr + 8, cfg_addr, 8, 0, NULL);
    fw_cfg_add_i16(fw_cfg, FW_CFG_NB_CPUS, (uint16_t)current_machine->smp.cpus);
    fw_cfg_add_i16(fw_cfg, FW_CFG_MAX_CPUS, (uint16_t)current_machine->smp.max_cpus);
    fw_cfg_add_i64(fw_cfg, FW_CFG_RAM_SIZE, (uint64_t)ram_size);
    fw_cfg_add_i32(fw_cfg, FW_CFG_MACHINE_VERSION, 0);
    fw_cfg_add_i32(fw_cfg, FW_CFG_CPU_FREQ, get_cpu_freq());
    qemu_register_boot_set(fw_cfg_boot_set, fw_cfg);
}

static int set_prom_cmdline(ram_addr_t initrd_offset, long initrd_size)
{
    hwaddr cmdline_vaddr;
    char memenv[32];
    char highmemenv[32];
    void *cmdline_buf;
    unsigned int *parg_env;
    int ret = 0;

    /* Allocate cmdline_buf for command line. */
    cmdline_buf = g_malloc0(loader_memmap[LOADER_CMDLINE].size);
    cmdline_vaddr = cpu_mips_phys_to_kseg0(NULL,
                                             loader_memmap[LOADER_CMDLINE].base);

    /*
     * Layout of cmdline_buf looks like this:
     * argv[0], argv[1], 0, env[0], env[1], ... env[i], 0,
     * argv[0]'s data, argv[1]'s data, env[0]'data, ..., env[i]'s data, 0
     */
    parg_env = (void *)cmdline_buf;

    ret = (3 + 1) * 4;
    *parg_env++ = cmdline_vaddr + ret;
    ret += (1 + snprintf(cmdline_buf + ret, 256 - ret, "g"));

    /* argv1 */
    *parg_env++ = cmdline_vaddr + ret;
    if (initrd_size > 0)
        ret += (1 + snprintf(cmdline_buf + ret, 256 - ret,
                "rd_start=0x" TARGET_FMT_lx " rd_size=%li %s",
                cpu_mips_phys_to_kseg0(NULL, initrd_offset),
                initrd_size, loaderparams.kernel_cmdline));
    else
        ret += (1 + snprintf(cmdline_buf + ret, 256 - ret, "%s",
                loaderparams.kernel_cmdline));

    /* argv2 */
    *parg_env++ = cmdline_vaddr + 4 * ret;

    /* env */
    sprintf(memenv, "%ld", loaderparams.ram_size > 0x10000000
            ? 256 : (loaderparams.ram_size >> 20));
    sprintf(highmemenv, "%ld", loaderparams.ram_size > 0x10000000
            ? (loaderparams.ram_size >> 20) - 256 : 0);

    rom_add_blob_fixed("cmdline", cmdline_buf,
                       loader_memmap[LOADER_CMDLINE].size,
                       loader_memmap[LOADER_CMDLINE].base);

    g_free(cmdline_buf);

    loaderparams.a0 = 2;
    loaderparams.a1 = cmdline_vaddr;

    return 0;
}

static uint64_t load_kernel(CPUMIPSState *env)
{
    long kernel_size;
    ram_addr_t initrd_offset;
    uint64_t kernel_entry, kernel_low, kernel_high, initrd_size;

    kernel_size = load_elf(loaderparams.kernel_filename, NULL,
                           cpu_mips_kseg0_to_phys, NULL,
                           (uint64_t *)&kernel_entry,
                           (uint64_t *)&kernel_low, (uint64_t *)&kernel_high,
                           NULL, 0, EM_MIPS, 1, 0);
    if (kernel_size < 0) {
        error_report("could not load kernel '%s': %s",
                     loaderparams.kernel_filename,
                     load_elf_strerror(kernel_size));
        exit(1);
    }

    /* load initrd */
    initrd_size = 0;
    initrd_offset = 0;
    if (loaderparams.initrd_filename) {
        initrd_size = get_image_size(loaderparams.initrd_filename);
        if (initrd_size > 0) {
            initrd_offset = (kernel_high + ~INITRD_PAGE_MASK) &
                            INITRD_PAGE_MASK;
            initrd_offset = MAX(initrd_offset,
                                loader_memmap[LOADER_INITRD].base);

            if (initrd_offset + initrd_size > ram_size) {
                error_report("memory too small for initial ram disk '%s'",
                             loaderparams.initrd_filename);
                exit(1);
            }

            initrd_size = load_image_targphys(loaderparams.initrd_filename,
                                              initrd_offset,
                                              ram_size - initrd_offset);
        }

        if (initrd_size == (target_ulong) -1) {
            error_report("could not load initial ram disk '%s'",
                         loaderparams.initrd_filename);
            exit(1);
        }
    }

    /* Setup prom cmdline. */
    set_prom_cmdline(initrd_offset, initrd_size);

    return kernel_entry;
}

static void main_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;
    CPUMIPSState *env = &cpu->env;

    cpu_reset(CPU(cpu));

    /* Loongson-3 reset stuff */
    if (loaderparams.kernel_filename) {
        if (cpu == MIPS_CPU(first_cpu)) {
            env->active_tc.gpr[4] = loaderparams.a0;
            env->active_tc.gpr[5] = loaderparams.a1;
            env->active_tc.gpr[6] = loaderparams.a2;
            env->active_tc.PC = loaderparams.kernel_entry;
        }
        env->CP0_Status &= ~((1 << CP0St_BEV) | (1 << CP0St_ERL));
    }
}

static void loongson3_isa_init(qemu_irq intc)
{
    qemu_irq *i8259;
    ISABus *isa_bus;

    isa_bus = isa_bus_new(NULL, get_system_memory(), get_system_io(), &error_abort);

    /* Interrupt controller */
    /* The 8259 -> IP3  */
    i8259 = i8259_init(isa_bus, intc);
    isa_bus_irqs(isa_bus, i8259);
    /* init other devices */
    isa_create_simple(isa_bus, "i8042");
    mc146818_rtc_init(isa_bus, 2000, NULL);
}

static inline void loongson3_pcie_init(MachineState *machine, DeviceState *pic)
{
    int i;
    qemu_irq irq;
    PCIBus *pci_bus;
    USBBus *usb_bus;
    DeviceState *dev;
    MemoryRegion *pio_alias;
    MemoryRegion *mmio_alias, *mmio_reg;
    MemoryRegion *ecam_alias, *ecam_reg;

    dev = qdev_create(NULL, TYPE_GPEX_HOST);

    qdev_init_nofail(dev);
    pci_bus = PCI_HOST_BRIDGE(dev)->bus;

    ecam_alias = g_new0(MemoryRegion, 1);
    ecam_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_init_alias(ecam_alias, OBJECT(dev), "pcie-ecam",
                             ecam_reg, 0, virt_memmap[VIRT_PCIE_ECAM].size);
    memory_region_add_subregion(get_system_memory(), virt_memmap[VIRT_PCIE_ECAM].base, ecam_alias);

    mmio_alias = g_new0(MemoryRegion, 1);
    mmio_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 1);
    memory_region_init_alias(mmio_alias, OBJECT(dev), "pcie-mmio",
                             mmio_reg, virt_memmap[VIRT_PCIE_MMIO].base,
                             virt_memmap[VIRT_PCIE_MMIO].size);
    memory_region_add_subregion(get_system_memory(), virt_memmap[VIRT_PCIE_MMIO].base, mmio_alias);

    pio_alias = g_new0(MemoryRegion, 1);
    memory_region_init_alias(pio_alias, OBJECT(dev), "pcie-pio",
                             get_system_io(), 0, virt_memmap[VIRT_ISA_PIO].size);
    memory_region_add_subregion(get_system_memory(), virt_memmap[VIRT_ISA_PIO].base, pio_alias);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 2, virt_memmap[VIRT_ISA_PIO].base);

    for (i = 0; i < GPEX_NUM_IRQS; i++) {
        irq = qdev_get_gpio_in(pic, PCIE_IRQ_BASE + i);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i, irq);
        gpex_set_irq_num(GPEX_HOST(dev), i, PCIE_IRQ_BASE + i);
    }

    pci_create_simple(pci_bus, -1, "pci-ohci");
    
    if (!pci_vga_init(pci_bus)) {
        usb_bus = usb_bus_find(-1);
        usb_create_simple(usb_bus, "usb-kbd");
        usb_create_simple(usb_bus, "usb-tablet");
    }

    for (i = 0; i < nb_nics; i++) {
        NICInfo *nd = &nd_table[i];

        if (!nd->model) {
            nd->model = g_strdup("virtio");
        }

        pci_nic_init_nofail(nd, pci_bus, nd->model, NULL);
    }
}

static void mips_loongson3_init(MachineState *machine)
{
    int i;
    long bios_size;
    MIPSCPU *cpu;
    CPUMIPSState *env;
    char *filename;
    const char *kernel_cmdline = machine->kernel_cmdline;
    const char *kernel_filename = machine->kernel_filename;
    const char *initrd_filename = machine->initrd_filename;
    ram_addr_t ram_size = machine->ram_size;
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    MemoryRegion *bios = g_new(MemoryRegion, 1);
    MemoryRegion *iomem = g_new(MemoryRegion, 1);

    if (!kvm_enabled()) {
        if (!machine->cpu_type) {
            machine->cpu_type = MIPS_CPU_TYPE_NAME("Loongson-3A1000");
        }
        if (!strstr(machine->cpu_type, "Loongson-3A1000")) {
            error_report("Loongson-3/TCG need cpu type Loongson-3A1000");
            exit(1);
        }
    } else {
        if (!machine->cpu_type) {
            machine->cpu_type = MIPS_CPU_TYPE_NAME("Loongson-3A4000");
        }
        if (!strstr(machine->cpu_type, "Loongson-3A4000")) {
            error_report("Loongson-3/KVM need cpu type Loongson-3A4000");
            exit(1);
        }
    }

    if (ram_size < 256 * 0x100000) {
        error_report("Loongson-3 need at least 256MB memory");
        exit(1);
    }

    /*
     * The whole MMIO range among configure registers doesn't generate
     * exception when accessing invalid memory. Create an empty slot to
     * emulate this feature.
     */
    empty_slot_init(0, 0x80000000);

    for (i = 0; i < machine->smp.cpus; i++) {
        /* init CPUs */
        cpu = MIPS_CPU(cpu_create(machine->cpu_type));

        /* Init internal devices */
        cpu_mips_irq_init_cpu(cpu);
        cpu_mips_clock_init(cpu);
        qemu_register_reset(main_cpu_reset, cpu);

        if (!kvm_enabled()) {
            DeviceState *dev;
            dev = qdev_create(NULL, "loongson.smp-ipi");
            qdev_init_nofail(dev);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x3ff01000 + 0x100 * i);
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, cpu->env.irq[6]);
        }
    }
    env = &MIPS_CPU(first_cpu)->env;

    /* Allocate RAM/BIOS, 0x00000000~0x10000000 is alias of 0x80000000~0x90000000 */
    memory_region_init_rom(bios, NULL, "loongson3.bios",
                           virt_memmap[VIRT_BIOS_ROM].size, &error_fatal);
    memory_region_init_alias(ram, NULL, "loongson3.lowmem",
                           machine->ram, 0, virt_memmap[VIRT_LOWMEM].size);
    memory_region_init_io(iomem, NULL, &loongson3_pm_ops,
                           NULL, "loongson3_pm", virt_memmap[VIRT_PM].size);

    memory_region_add_subregion(address_space_mem, virt_memmap[VIRT_LOWMEM].base, ram);
    memory_region_add_subregion(address_space_mem, virt_memmap[VIRT_BIOS_ROM].base, bios);
    memory_region_add_subregion(address_space_mem, virt_memmap[VIRT_HIGHMEM].base, machine->ram);
    memory_region_add_subregion(address_space_mem, virt_memmap[VIRT_PM].base, iomem);

    /*
     * We do not support flash operation, just loading pmon.bin as raw BIOS.
     * Please use -L to set the BIOS path and -bios to set bios name.
     */

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        loaderparams.kernel_entry = load_kernel(env);

        init_boot_rom();
        init_boot_param();
    } else {
        if (bios_name == NULL) {
                bios_name = LOONGSON3_BIOSNAME;
        }
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
        if (filename) {
            bios_size = load_image_targphys(filename, virt_memmap[VIRT_BIOS_ROM].base,
                                            virt_memmap[VIRT_BIOS_ROM].size);
            g_free(filename);
        } else {
            bios_size = -1;
        }

        if ((bios_size < 0 || bios_size > virt_memmap[VIRT_BIOS_ROM].size) &&
            !kernel_filename && !qtest_enabled()) {
            error_report("Could not load MIPS bios '%s'", bios_name);
            exit(1);
        }

    }

    fw_conf_init(ram_size);

    loongson3_isa_init(env->irq[3]);
    loongson3_pcie_init(machine, isa_pic);

    if (serial_hd(0)) {
        serial_mm_init(address_space_mem, virt_memmap[VIRT_UART].base, 0, env->irq[2],
                        115200, serial_hd(0), DEVICE_NATIVE_ENDIAN);
    }
}

static void mips_loongson3_machine_init(MachineClass *mc)
{
    mc->desc = "Generic Loongson-3 Virtulization Platform";
    mc->init = mips_loongson3_init;
    mc->block_default_type = IF_IDE;
    mc->max_cpus = LOONGSON_MAX_VCPUS;
    mc->default_ram_id = "loongson3.highram";
    mc->default_ram_size = 1200 * MiB;
    mc->kvm_type = mips_kvm_type;
    mc->minimum_page_bits = 14;
}

DEFINE_MACHINE("loongson3-virt", mips_loongson3_machine_init)
