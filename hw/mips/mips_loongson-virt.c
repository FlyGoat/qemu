
#include "qemu/osdep.h"
#include "qemu-common.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "cpu.h"
#include "elf.h"
#include "hw/boards.h"
#include "hw/block/flash.h"
#include "hw/char/serial.h"
#include "hw/empty_slot.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "hw/mips/loongson-virt.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci-host/gpex.h"
#include "hw/southbridge/piix.h"
#include "audio/audio.h"
#include "qemu/log.h"
#include "hw/loader.h"
#include "exec/address-spaces.h"
#include "sysemu/qtest.h"
#include "sysemu/reset.h"
#include "sysemu/sysemu.h"
#include "qemu/error-report.h"

#define ENVP_ADDR               0x80002000l
#define ARGP_NB_ENTRIES         16
#define ARGP_ENTRY_SIZE         256
#define ARGP_SIZE               (ARGP_NB_ENTRIES * (sizeof(int32_t) + ARGP_ENTRY_SIZE))
#define EFIP_ADDR               0x80010000l
#define EFIP_SIZE               0x80000

#define VIRT_BIOSNAME           "loongson-virt-bios.bin"

static struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} virt_memmap[] = {
    [VIRT_MEM_LOW_ALIAS] = {        0x0,    0x10000000 },
    [VIRT_PCIE_PIO] =      { 0x18000000,     0x2000000 },
    [VIRT_PCIE_ECAM] =     { 0x1a000000,     0x2000000 },
    [VIRT_ROM]      =      { 0x1d000000,     0x1000000 },
    [VIRT_ROM_ALIAS]  =    { 0x1fc00000,      0x100000 },
    [VIRT_REG_CFG0]  =     { 0x1fe00000,         0x1e0 },
    [VIRT_UART0]     =     { 0x1fe001e0,           0x8 },
    [VIRT_PCIE_MMIO] =     { 0x40000000,    0x40000000 },
    [VIRT_REG_CFG1]  =     { 0x3ff00000,        0x4000 },
    [VIRT_MEM_HI]    =     { 0x80000000,           0x0 },
};

static void GCC_FMT_ATTR(3, 4) prom_set(uint32_t *prom_buf, int index,
                                        const char *string, ...)
{
    va_list ap;
    int32_t table_addr;

    if (index >= ARGP_NB_ENTRIES) {
        return;
    }

    if (string == NULL) {
        prom_buf[index] = 0;
        return;
    }

    table_addr = sizeof(int32_t) * ARGP_NB_ENTRIES + index * ARGP_ENTRY_SIZE;
    prom_buf[index] = tswap32(ENVP_ADDR + table_addr);

    va_start(ap, string);
    vsnprintf((char *)prom_buf + table_addr, ARGP_ENTRY_SIZE, string, ap);
    va_end(ap);
}

static void param_init_emem(struct efi_memory_map_loongson *emem)
{
    int map_id = 0;

    emem->vers = 0x0;
    emem->mem_freq = 1600;

    /* Low MEM */
    emem->map[map_id].node_id = 0;
    emem->map[map_id].mem_type = 1;
    emem->map[map_id].mem_start = virt_memmap[VIRT_MEM_LOW_ALIAS].base;
    emem->map[map_id].mem_size = virt_memmap[VIRT_MEM_LOW_ALIAS].size / MiB;
    map_id++;

    /* High MEM without alias of low MEM */
    emem->map[map_id].node_id = 0;
    emem->map[map_id].mem_type = 2;
    emem->map[map_id].mem_start = virt_memmap[VIRT_MEM_HI].base + \
                                    virt_memmap[VIRT_MEM_LOW_ALIAS].size;
    emem->map[map_id].mem_size =  (virt_memmap[VIRT_MEM_HI].size - \
                                virt_memmap[VIRT_MEM_LOW_ALIAS].size) / MiB;
    map_id++;

    emem->nr_map = map_id;
}

static void param_init_ecpu(struct efi_cpuinfo_loongson *ecpu)
{
    /* TODO: SMP support */
    ecpu->vers = 0;
    ecpu->processor_id = 0x6305;
    ecpu->cputype = Loongson_3A;
    ecpu->total_node = 1;
    ecpu->cpu_startup_core_id = 0;
    ecpu->reserved_cores_mask = 0xffff & ~(1 << 0);
    ecpu->cpu_clock_freq = 20000000;
    ecpu->nr_cpus = 1;
}

static void param_init_esys(struct system_loongson *esys)
{
    /* dummy values */
    esys->vers = 0;
    esys->ccnuma_smp = 0;
    esys->sing_double_channel = 2;

    esys->nr_uarts = 1;
    esys->uarts[0].iotype = 1;
    esys->uarts[0].uartclk = 33000000;
    esys->uarts[0].int_offset = 2;
    esys->uarts[0].uart_base = virt_memmap[VIRT_UART0].base;

    esys->nr_sensors = 0;
    esys->has_ec = 0;
    esys->has_tcm = 0;
}

static void param_init_eirq(struct irq_source_routing_table *eirq)
{
    /*
     * Skip route config, nobody know it's spec and all open-sourced
     * kernel won't read it.
     */
    eirq->vers = 0;

    eirq->pci_mem_start_addr = virt_memmap[VIRT_PCIE_MMIO].base;
    eirq->pci_mem_end_addr = virt_memmap[VIRT_PCIE_MMIO].base + \
                               virt_memmap[VIRT_PCIE_MMIO].size + 1;
    eirq->pci_io_start_addr = virt_memmap[VIRT_PCIE_PIO].base;
    eirq->pci_io_end_addr = virt_memmap[VIRT_PCIE_PIO].base + \
                            virt_memmap[VIRT_PCIE_PIO].size + 1;
    eirq->pci_config_addr = virt_memmap[VIRT_PCIE_ECAM].base;

    eirq->dma_mask_bits = 64;
    eirq->dma_noncoherent = 0x0;
}

static void param_init_eboard(struct board_devices *eboard)
{
    char name[64];
    int i;
    strcpy(&name[0], "QEMU-EMU-VIRT-1w-VM-v1.0");
    for (i = 0; i < 64; i++)
        eboard->name[i] = name[i];
    eboard->num_resources = 0;
}

static void loongson_efi_param_init(void)
{
    struct boot_params *bootp;
    struct loongson_params *lp;
    struct efi_memory_map_loongson *emem;
	struct efi_cpuinfo_loongson *ecpu;
    struct system_loongson *esys;
	struct irq_source_routing_table *eirq;
    struct board_devices *eboard;
    char *efi_buf = g_malloc(EFIP_SIZE);
    ram_addr_t offset = 0;

    bootp = (struct boot_params *)(efi_buf);
    lp = &(bootp->efi.smbios.lp);

    offset += QEMU_ALIGN_UP(sizeof(struct boot_params), 8);

    emem = (struct efi_memory_map_loongson *)((ram_addr_t)lp + offset);
    lp->memory_offset = offset;
    offset += QEMU_ALIGN_UP(sizeof(struct efi_memory_map_loongson), 8);
    param_init_emem(emem);

    ecpu = (struct efi_cpuinfo_loongson *)((ram_addr_t)lp + offset);
    lp->cpu_offset = offset;
    offset += QEMU_ALIGN_UP(sizeof(struct efi_cpuinfo_loongson), 8);
    param_init_ecpu(ecpu);

    esys = (struct system_loongson *)((ram_addr_t)lp + offset);
    lp->system_offset = offset;
    offset += QEMU_ALIGN_UP(sizeof(struct system_loongson), 8);
    param_init_esys(esys);

    eirq = (struct irq_source_routing_table *)((ram_addr_t)lp + offset);
    lp->irq_offset = offset;
    offset += QEMU_ALIGN_UP(sizeof(struct irq_source_routing_table), 8);
    param_init_eirq(eirq);

    eboard = (struct board_devices *)((ram_addr_t)lp + offset);
    lp->boarddev_table_offset = offset;
    offset += QEMU_ALIGN_UP(sizeof(struct board_devices), 8);
    param_init_eboard(eboard);

    rom_add_blob_fixed("bootparam", efi_buf, EFIP_SIZE,
                       cpu_mips_kseg0_to_phys(NULL, EFIP_ADDR));
    g_free(efi_buf);
}

static int64_t load_kernel(CPUMIPSState *env)
{
    int64_t kernel_entry, kernel_low, kernel_high, initrd_size;
    int index = 0;
    long kernel_size;
    ram_addr_t initrd_offset;
    uint32_t *prom_buf;
    long prom_size;

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

    /* Setup prom parameters. */
    prom_size = ARGP_SIZE;
    prom_buf = g_malloc(prom_size);

    prom_set(prom_buf, index++, "%s", loaderparams.kernel_filename);
    if (initrd_size > 0) {
        prom_set(prom_buf, index++,
                 "rd_start=0x%" PRIx64 " rd_size=%" PRId64 " %s",
                 cpu_mips_phys_to_kseg0(NULL, initrd_offset),
                 initrd_size, loaderparams.kernel_cmdline);
    } else {
        prom_set(prom_buf, index++, "%s", loaderparams.kernel_cmdline);
    }

    rom_add_blob_fixed("prom_env", prom_buf, prom_size,
                       cpu_mips_kseg0_to_phys(NULL, ENVP_ADDR));

    g_free(prom_buf);

    loongson_efi_param_init();
    return kernel_entry;
}

static void write_bootloader(CPUMIPSState *env, uint8_t *base,
                             int64_t kernel_addr)
{
    uint32_t *p;

    /* Small bootloader */
    p = (uint32_t *)base;

    /* j 0x1fc00040 */
    stl_p(p++, 0x0bf00010);
    /* nop */
    stl_p(p++, 0x00000000);

    /* Second part of the bootloader */
    p = (uint32_t *)(base + 0x040);

    /* lui a0, 0 */
    stl_p(p++, 0x3c040000);
    /* ori a0, a0, 2 */
    stl_p(p++, 0x34840002);
    /* lui a1, high(ENVP_ADDR) */
    stl_p(p++, 0x3c050000 | ((ENVP_ADDR >> 16) & 0xffff));
    /* ori a1, a0, low(ENVP_ADDR) */
    stl_p(p++, 0x34a50000 | (ENVP_ADDR & 0xffff));
    /* lui a2, high(ENVP_ADDR + ARGP_SIZE) */
    stl_p(p++, 0x3c060000 | ((EFIP_ADDR >> 16) & 0xffff));
    /* ori a2, a2, low(ENVP_ADDR + ARGP_SIZE) */
    stl_p(p++, 0x34c60000 | (EFIP_ADDR & 0xffff));
    /* lui ra, high(kernel_addr) */
    stl_p(p++, 0x3c1f0000 | ((kernel_addr >> 16) & 0xffff));
    /* ori ra, ra, low(kernel_addr) */
    stl_p(p++, 0x37ff0000 | (kernel_addr & 0xffff));
    /* jr ra */
    stl_p(p++, 0x03e00008);
    /* nop */
    stl_p(p++, 0x00000000);
}

static void main_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;
    CPUMIPSState *env = &cpu->env;

    cpu_reset(CPU(cpu));
    /* TODO: DO reset */
    if (loaderparams.kernel_filename) {
        env->CP0_Status &= ~((1 << CP0St_BEV) | (1 << CP0St_ERL));
    }
}

static inline DeviceState *gpex_pcie_init(MemoryRegion *sys_mem,
                                          hwaddr ecam_base, hwaddr ecam_size,
                                          hwaddr mmio_base, hwaddr mmio_size,
                                          hwaddr pio_base, hwaddr pio_size)
{
    DeviceState *dev;
    MemoryRegion *ecam_alias, *ecam_reg;
    MemoryRegion *mmio_alias, *mmio_reg;
    MemoryRegion *pio_alias, *pio_reg;

    dev = qdev_create(NULL, TYPE_GPEX_HOST);

    qdev_init_nofail(dev);

    ecam_alias = g_new0(MemoryRegion, 1);
    ecam_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_init_alias(ecam_alias, OBJECT(dev), "pcie-ecam",
                             ecam_reg, 0, ecam_size);
    memory_region_add_subregion(sys_mem, ecam_base, ecam_alias);

    mmio_alias = g_new0(MemoryRegion, 1);
    mmio_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 1);
    memory_region_init_alias(mmio_alias, OBJECT(dev), "pcie-mmio",
                             mmio_reg, mmio_base, mmio_size);
    memory_region_add_subregion(sys_mem, mmio_base, mmio_alias);

    pio_alias = g_new0(MemoryRegion, 1);
    pio_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 2);
    memory_region_init_alias(pio_alias, OBJECT(dev), "pcie-pio",
                             pio_reg, 0, pio_size);
    memory_region_add_subregion(sys_mem, pio_base, pio_alias);

    return dev;
}


static void mips_loongson_virt_init(MachineState *machine)
{
    const char *kernel_filename = machine->kernel_filename;
    const char *kernel_cmdline = machine->kernel_cmdline;
    const char *initrd_filename = machine->initrd_filename;
    char *filename;
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *bios = g_new(MemoryRegion, 1);
    MemoryRegion *bios_alias = g_new(MemoryRegion, 1);
    MemoryRegion *ram_alias = g_new(MemoryRegion, 1);
    long bios_size;
    int64_t kernel_entry;
    PCIBus *pci_bus;
    ISABus *isa_bus;
    I2CBus *smbus;
    MIPSCPU *cpu;
    CPUMIPSState *env;
    DeviceState *dev;
    DeviceState *pci_host_dev;
    PCIHostState *pci;

    empty_slot_init(0, 0x80000000);

    /* init CPUs */
    cpu = MIPS_CPU(cpu_create(machine->cpu_type));
    env = &cpu->env;

    qemu_register_reset(main_cpu_reset, cpu);

    /* allocate BIOS ROM */
    memory_region_init_rom(bios, NULL, "loongson-virt.bios",
                            virt_memmap[VIRT_ROM].size, &error_fatal);
    memory_region_add_subregion(address_space_mem,
                                virt_memmap[VIRT_ROM].base, bios);
    /* alias BIOS Boot region */
    memory_region_init_alias(bios_alias, NULL, "loongson-virt.ram_lo",
                             bios, 0, virt_memmap[VIRT_ROM_ALIAS].size);
    memory_region_add_subregion(address_space_mem,
                                virt_memmap[VIRT_ROM_ALIAS].base, bios_alias);

    
    /* register RAM at high address */
    virt_memmap[VIRT_MEM_HI].size = machine->ram_size;
    memory_region_add_subregion(address_space_mem, virt_memmap[VIRT_MEM_HI].base,
                                machine->ram);
    /* alias for low mem */
    memory_region_init_alias(ram_alias, NULL, "loongson-virt.ram_alias",
                             machine->ram, 0, virt_memmap[VIRT_MEM_LOW_ALIAS].size);
    memory_region_add_subregion(address_space_mem,
                                virt_memmap[VIRT_MEM_LOW_ALIAS].base, ram_alias);

    if (kernel_filename) {
        loaderparams.ram_size = machine->ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        kernel_entry = load_kernel(env);
        write_bootloader(env, memory_region_get_ram_ptr(bios), kernel_entry);
    } else {
        if (bios_name == NULL) {
                bios_name = VIRT_BIOSNAME;
        }
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
        if (filename) {
            bios_size = load_image_targphys(filename, virt_memmap[VIRT_ROM].base,
                                            virt_memmap[VIRT_ROM].size);
            g_free(filename);
        } else {
            bios_size = -1;
        }

        if ((bios_size < 0 || bios_size > virt_memmap[VIRT_ROM].size) &&
            !kernel_filename && !qtest_enabled()) {
            error_report("Could not load MIPS bios '%s'", bios_name);
            exit(1);
        }
    }

    /* Init internal devices */
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);

    serial_mm_init(address_space_mem, virt_memmap[VIRT_UART0].base,
        0, env->irq[2], 115200, serial_hd(0), DEVICE_LITTLE_ENDIAN);

    pci_host_dev = gpex_pcie_init(address_space_mem,
                         virt_memmap[VIRT_PCIE_ECAM].base,
                         virt_memmap[VIRT_PCIE_ECAM].size,
                         virt_memmap[VIRT_PCIE_MMIO].base,
                         virt_memmap[VIRT_PCIE_MMIO].size,
                         virt_memmap[VIRT_PCIE_PIO].base,
                         virt_memmap[VIRT_PCIE_PIO].size);
    pci = PCI_HOST_BRIDGE(pci_host_dev);
    pci_bus = pci->bus;

    dev = piix4_create(pci_bus, &isa_bus, &smbus);
    qdev_connect_gpio_out_named(dev, "intr", 0, env->irq[3]);

    /* PIIX4 Style PCI IRQ */
    sysbus_connect_irq(SYS_BUS_DEVICE(pci_host_dev), 0,
                           qdev_get_gpio_in_named(dev, "isa", 10));
    gpex_set_irq_num(GPEX_HOST(pci_host_dev), 0, 10);
    sysbus_connect_irq(SYS_BUS_DEVICE(pci_host_dev), 1,
                           qdev_get_gpio_in_named(dev, "isa", 10));
    gpex_set_irq_num(GPEX_HOST(pci_host_dev), 1, 10);
    sysbus_connect_irq(SYS_BUS_DEVICE(pci_host_dev), 2,
                           qdev_get_gpio_in_named(dev, "isa", 11));
    gpex_set_irq_num(GPEX_HOST(pci_host_dev), 2, 11);
    sysbus_connect_irq(SYS_BUS_DEVICE(pci_host_dev), 3,
                           qdev_get_gpio_in_named(dev, "isa", 11));
    gpex_set_irq_num(GPEX_HOST(pci_host_dev), 3, 11);
}

static void mips_loongson_virt_machine_init(MachineClass *mc)
{
    mc->desc = "Loongson VIRT Machine";
    mc->init = mips_loongson_virt_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("gs464");
    mc->default_ram_size = 2048 * MiB;
    mc->default_ram_id = "loongson-virt.ram";
}

DEFINE_MACHINE("loongson-virt", mips_loongson_virt_machine_init)