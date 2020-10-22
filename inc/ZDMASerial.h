#ifndef ZDMA_SERIAL_H
#define ZDMA_SERIAL_H

#include "DMASerial.h"
#include "nrf.h"

namespace codal
{

class ZDMASerial : public DMASerial
{
protected:
    static void irq_handler(void *);
    void irq_handler();
    NRF_UARTE_Type *uart;

    void stopRX();
    void stopTX();

public:
    ZDMASerial(Pin &tx, Pin &rx, uint16_t id = DEVICE_ID_DMA_SERIAL);

    virtual int setBaud(uint32_t baud) override;
    virtual uint32_t getBaud() override;
    virtual int enableTxRx(bool tx, bool rx) override;
    virtual int startSend(const uint8_t *data, int len, DMASerialCallback doneHandler,
                          void *doneArg) override;
    virtual int startReceive(uint8_t *data, int len, DMASerialCallback doneHandler,
                             void *doneArg) override;
    virtual int abortDMA() override;
};
} // namespace codal

#endif