
#ifndef HW_LOONGSON_VIRT_H
#define HW_LOONGSON_VIRT_H

enum {
    VIRT_MEM_LOW_ALIAS,
    VIRT_PCIE_PIO,
    VIRT_PCIE_ECAM,
    VIRT_ROM,
    VIRT_ROM_ALIAS,
    VIRT_REG_CFG0,
    VIRT_UART0,
    VIRT_PCIE_MMIO,
    VIRT_REG_CFG1,
    VIRT_MEM_HI,
};

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;


/* Loongson Boot Proctol Defines */
struct efi_memory_map_loongson{
    uint16_t vers;               /* version of efi_memory_map */
    uint32_t nr_map;             /* number of memory_maps */
    uint32_t mem_freq;           /* memory frequence */
    struct mem_map{
        uint32_t node_id;        /* node_id which memory attached to */
        uint32_t mem_type;       /* system memory, pci memory, pci io, etc. */
        uint64_t mem_start;      /* memory map start address */
        uint32_t mem_size;       /* each memory_map size, not the total size */
    } map[128];
}__attribute__((packed));

enum loongson_cpu_type
{
    Legacy_2E = 0x0,
    Legacy_2F = 0x1,
    Legacy_3A = 0x2,
    Legacy_3B = 0x3,
    Legacy_1A = 0x4,
    Legacy_1B = 0x5,
    Legacy_2G = 0x6,
    Legacy_2H = 0x7,
    Loongson_1A = 0x100,
    Loongson_1B = 0x101,
    Loongson_2E = 0x200,
    Loongson_2F = 0x201,
    Loongson_2G = 0x202,
    Loongson_2H = 0x203,
    Loongson_3A = 0x300,
    Loongson_3B = 0x301
};

/*
 * Capability and feature descriptor structure for MIPS CPU
 */
struct efi_cpuinfo_loongson {
    uint16_t vers;               /* version of efi_cpuinfo_loongson */
    uint32_t processor_id;       /* PRID, e.g. 6305, 6306 */
    enum loongson_cpu_type cputype; /* 3A, 3B, etc. */
    uint32_t total_node;         /* num of total numa nodes */
    uint16_t cpu_startup_core_id;   /* Core id */
    uint16_t reserved_cores_mask;
    uint32_t cpu_clock_freq;     /* cpu_clock */
    uint32_t nr_cpus;
}__attribute__((packed));

#define MAX_UARTS 64
struct uart_device {
    uint32_t iotype; /* see include/linux/serial_core.h */
    uint32_t uartclk;
    uint32_t int_offset;
    uint64_t uart_base;
}__attribute__((packed));

#define MAX_SENSORS 64
#define SENSOR_TEMPER  0x00000001
#define SENSOR_VOLTAGE 0x00000002
#define SENSOR_FAN     0x00000004
struct sensor_device {
    char name[32];  /* a formal name */
    char label[64]; /* a flexible description */
    uint32_t type;       /* SENSOR_* */
    uint32_t id;         /* instance id of a sensor-class */
    uint32_t fan_policy; /* see arch/mips/include/asm/mach-loongson/loongson_hwmon.h */
    uint32_t fan_percent;/* only for constant speed policy */
    uint64_t base_addr;  /* base address of device registers */
}__attribute__((packed));

struct system_loongson{
    uint16_t vers;               /* version of system_loongson */
    uint32_t ccnuma_smp;         /* 0: no numa; 1: has numa */
    uint32_t sing_double_channel;/* 1:single; 2:double */
    uint32_t nr_uarts;
    struct uart_device uarts[MAX_UARTS];
    uint32_t nr_sensors;
    struct sensor_device sensors[MAX_SENSORS];
    char has_ec;
    char ec_name[32];
    uint64_t ec_base_addr;
    char has_tcm;
    char tcm_name[32];
    uint64_t tcm_base_addr;
    uint64_t workarounds; /* see workarounds.h */
}__attribute__((packed));

struct irq_source_routing_table {
    uint16_t vers;
    uint16_t size;
    uint16_t rtr_bus;
    uint16_t rtr_devfn;
    uint32_t vendor;
    uint32_t device;
    uint32_t PIC_type;           /* conform use HT or PCI to route to CPU-PIC */
    uint64_t ht_int_bit;         /* 3A: 1<<24; 3B: 1<<16 */
    uint64_t ht_enable;          /* irqs used in this PIC */
    uint32_t node_id;            /* node id: 0x0-0; 0x1-1; 0x10-2; 0x11-3 */
    uint64_t pci_mem_start_addr;
    uint64_t pci_mem_end_addr;
    uint64_t pci_io_start_addr;
    uint64_t pci_io_end_addr;
    uint64_t pci_config_addr;
    uint32_t dma_mask_bits;
    uint16_t dma_noncoherent;
}__attribute__((packed));

struct interface_info{
    uint16_t vers;               /* version of the specificition */
    uint16_t size;
    uint8_t  flag;
    char description[64];
}__attribute__((packed));

#define MAX_RESOURCE_NUMBER 128
struct resource_loongson {
    uint64_t start;              /* resource start address */
    uint64_t end;                /* resource end address */
    char name[64];
    uint32_t flags;
};

struct archdev_data {};          /* arch specific additions */

struct board_devices{
    char name[64];               /* hold the device name */
    uint32_t num_resources;      /* number of device_resource */
    /* for each device's resource */
    struct resource_loongson resource[MAX_RESOURCE_NUMBER];
    /* arch specific additions */
    struct archdev_data archdata;
};

struct loongson_special_attribute{
    uint16_t vers;               /* version of this special */
    char special_name[64];       /* special_atribute_name */
    uint32_t loongson_special_type; /* type of special device */
    /* for each device's resource */
    struct resource_loongson resource[MAX_RESOURCE_NUMBER];
};

struct loongson_params{
    uint64_t memory_offset;      /* efi_memory_map_loongson struct offset */
    uint64_t cpu_offset;         /* efi_cpuinfo_loongson struct offset */
    uint64_t system_offset;      /* system_loongson struct offset */
    uint64_t irq_offset;         /* irq_source_routing_table struct offset */
    uint64_t interface_offset;   /* interface_info struct offset */
    uint64_t special_offset;     /* loongson_special_attribute struct offset */
    uint64_t boarddev_table_offset;  /* board_devices offset */
};

struct smbios_tables {
    uint16_t vers;               /* version of smbios */
    uint64_t vga_bios;           /* vga_bios address */
    struct loongson_params lp;
};

struct efi_reset_system_t{
    uint64_t ResetCold;
    uint64_t ResetWarm;
    uint64_t ResetType;
    uint64_t Shutdown;
    uint64_t DoSuspend; /* NULL if not support */
};

struct efi_loongson {
    uint64_t mps;                /* MPS table */
    uint64_t acpi;               /* ACPI table (IA64 ext 0.71) */
    uint64_t acpi20;             /* ACPI table (ACPI 2.0) */
    struct smbios_tables smbios; /* SM BIOS table */
    uint64_t sal_systab;         /* SAL system table */
    uint64_t boot_info;          /* boot info table */
};

struct boot_params{
    struct efi_loongson efi;
    struct efi_reset_system_t reset_system;
};

#endif