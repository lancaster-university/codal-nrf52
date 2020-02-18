#ifndef NRF_SYNC_SERIAL_H
#define NRF_SYNC_SERIAL_H

#include "NRF52SPI.h"

namespace codal
{

enum SyncSerialMode : uint8_t
{
    SYNC_SERIAL_MODE_I2CM = 0x01,
    SYNC_SERIAL_MODE_I2CS = 0x02,
    SYNC_SERIAL_MODE_SPIM = 0x04,
    SYNC_SERIAL_MODE_SPIS = 0x08,
    SYNC_SERIAL_MODE_UARTE = 0x10,
};

void *allocate_sync_serial(SyncSerialMode mode);
void *allocate_sync_serial(void *device);
IRQn_Type sync_serial_irqn(void *device);
void set_sync_serial_irq(void *device, PVoidCallback fn, void *userdata);

} // namespace codal

#endif
