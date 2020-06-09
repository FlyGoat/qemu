/*
 * QEMU Xilinx OPB Interrupt Controller.
 *
 * Copyright (c) 2009 Edgar E. Iglesias.
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

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"

#define D(x)

#define R_ISR       0
#define R_IEN       1
#define R_SET       2
#define R_CLR       3
/* No regs between 0x10~0x20 */
#define R_MBOX0     8
#define NUM_MBOX    8
#define R_MAX       16    

#define TYPE_LOONGSON_IPI "loongson.smp-ipi"
#define LOONGSON_IPI(obj) OBJECT_CHECK(struct loongson_ipi, (obj), TYPE_LOONGSON_IPI)

struct loongson_ipi
{
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq parent_irq;

    uint32_t isr;
    uint32_t ien;
    uint32_t mbox[NUM_MBOX];
};

static uint64_t
pic_read(void *opaque, hwaddr addr, unsigned int size)
{
    struct loongson_ipi *p = opaque;
    uint64_t r = 0;

    addr >>= 2;
    switch (addr)
    {
        case R_ISR:
            r = p->isr;
            break;
        case R_IEN:
            r = p->ien;
            break;
        default:
            if (addr >= R_MBOX0 && addr < R_MAX) {
                r = p->mbox[addr - R_MBOX0];
            }
            break;
    }
    D(printf("%s %x=%x\n", __func__, addr * 4, r));
    return r;
}

static void
pic_write(void *opaque, hwaddr addr,
          uint64_t val64, unsigned int size)
{
    struct loongson_ipi *p = opaque;
    uint32_t value = val64;

    addr >>= 2;
    switch (addr)
    {
        case R_ISR:
            /* Do nothing */
            break;
        case R_IEN:
            p->ien = value;
            break;
        case R_SET:
            p->isr |= value;
            break;
        case R_CLR:
            p->isr &= ~value;
            break;
        default:
            if (addr >= R_MBOX0 && addr < R_MAX) {
                p->mbox[addr - R_MBOX0] = value;
            }
            break;
    }
    D(printf("%s %x=%x\n", __func__, addr * 4, value));
    p->isr &= p->ien;
    if (!!p->isr) {
        D(printf("ipi high\n"));
    } else {
        D(printf("ipi low\n"));
    }
    qemu_set_irq(p->parent_irq, !!p->isr);
}

static const MemoryRegionOps pic_ops = {
    .read = pic_read,
    .write = pic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void loongson_ipi_init(Object *obj)
{
    struct loongson_ipi *p = LOONGSON_IPI(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &p->parent_irq);

    memory_region_init_io(&p->mmio, obj, &pic_ops, p, "loongson.smp-ipi",
                          R_MAX * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &p->mmio);
}


static void loongson_ipi_class_init(ObjectClass *klass, void *data)
{
}

static const TypeInfo loongson_ipi_info = {
    .name          = TYPE_LOONGSON_IPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct loongson_ipi),
    .instance_init = loongson_ipi_init,
    .class_init    = loongson_ipi_class_init,
};

static void loongson_ipi_register_types(void)
{
    type_register_static(&loongson_ipi_info);
}

type_init(loongson_ipi_register_types)
