#ifndef DMA_SERIAL_H
#define DMA_SERIAL_H

#include "Pin.h"
#include "CodalComponent.h"
#include "CodalConfig.h"

namespace codal
{

#define DEVICE_ID_DMA_SERIAL 50

typedef void (*DMASerialCallback)(void *);

class DMASerial : public CodalComponent
{
protected:
    DMASerialCallback *txHandler, *rxHandler;
    void *txArg, *rxArg;

public:
    Pin &tx, &rx;

    SingleWireSerial(Pin &tx, Pin &rx, uint16_t id = DEVICE_ID_DMA_SERIAL) : tx(tx), rx(rx)
    {
        this->id = id;
        this->txHandler = this->rxHandler = NULL;
    }

    virtual int setBaud(uint32_t baud) = 0;
    virtual uint32_t getBaud() = 0;
    virtual int enableTx(bool) = 0;
    virtual int enableRx(bool) = 0;

    virtual int sendDMA(uint8_t *data, int len, DMASerialCallback doneHandler, void *doneArg)
    {
        if (!data || len <= 0)
            return DEVICE_INVALID_PARAMETER;
        if (txHandler)
            return DEVICE_BUSY;
        txHandler = doneHandler;
        txArg = doneArg;
        if (&tx == &rx)
            enableRx(false);
        enableTx(true);
        return DEVICE_OK;
    }

    virtual int receiveDMA(uint8_t *data, int len, DMASerialCallback doneHandler, void *doneArg)
    {
        if (!data || len <= 0)
            return DEVICE_INVALID_PARAMETER;
        if (rxHandler)
            return DEVICE_BUSY;
        rxHandler = doneHandler;
        rxArg = doneArg;
        if (&tx == &rx)
            enableTx(false);
        enableRx(true);
        return DEVICE_OK;
    }

    virtual int abortDMA() = 0;
};

} // namespace codal

#endif