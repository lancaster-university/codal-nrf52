#include "CodalConfig.h"
#include "ErrorNo.h"
#include "CodalDmesg.h"
#include "sync_serial.h"
#include "codal_target_hal.h"

namespace codal
{

struct SyncSerial
{
    NRF_SPIM_Type *device;
    IRQn_Type irqn;
    SyncSerialMode modes;
};

static const SyncSerial serials[] = {
    //
    {NRF_SPIM0, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, SYNC_SERIAL_MODE_ALL},
    {NRF_SPIM1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, SYNC_SERIAL_MODE_ALL},
    {NRF_SPIM2, SPIM2_SPIS2_SPI2_IRQn,
     (SyncSerialMode)(SYNC_SERIAL_MODE_SPIM | SYNC_SERIAL_MODE_SPIS)},
#ifdef NRF_SPIM3
    {NRF_SPIM3, SPIM3_IRQn, SYNC_SERIAL_MODE_SPIM}
#endif
};

#define NUM_SYNC_SERIAL (int)(sizeof(serials) / sizeof(serials[0]))

static PVoidCallback irq_callback[NUM_SYNC_SERIAL];
static void *irq_callback_data[NUM_SYNC_SERIAL];
static uint8_t used_serials;

void *allocate_sync_serial(SyncSerialMode mode)
{
    int i = NUM_SYNC_SERIAL - 1;
    while (i >= 0)
    {
        if (!(used_serials & (1 << i)) && (serials[i].modes & mode))
        {
            used_serials |= 1 << i;
            return serials[i].device;
        }
        i--;
    }
    return NULL;
}

static int find_sync_serial(void *device)
{
    for (int i = 0; i < NUM_SYNC_SERIAL; ++i)
        if (serials[i].device == device)
            return i;
    target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);
    return -1;
}

IRQn_Type sync_serial_irqn(void *device)
{
    return serials[find_sync_serial(device)].irqn;
}

void set_sync_serial_irq(void *device, PVoidCallback fn, void *userdata)
{
    int i = find_sync_serial(device);
    irq_callback[i] = fn;
    irq_callback_data[i] = userdata;
}

#define DEF_IRQ(name, id)                                                                          \
    extern "C" void name() { irq_callback[id](irq_callback_data[id]); }

DEF_IRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler, 0)
DEF_IRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler, 1)
DEF_IRQ(SPIM2_SPIS2_SPI2_IRQHandler, 2)
#ifdef NRF_SPIM3
DEF_IRQ(SPIM3_IRQHandler, 3)
#endif

} // namespace codal