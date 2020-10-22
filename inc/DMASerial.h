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
    DMASerialCallback txHandler, rxHandler;
    void *txArg, *rxArg;

public:
    Pin &tx, &rx;

    DMASerial(Pin &tx, Pin &rx, uint16_t id = DEVICE_ID_DMA_SERIAL) : tx(tx), rx(rx)
    {
        this->id = id;
        this->txHandler = this->rxHandler = NULL;
    }

    virtual int setBaud(uint32_t baud) = 0;
    virtual uint32_t getBaud() = 0;
    virtual int enableTxRx(bool tx, bool rx) = 0;

    virtual int startSend(const uint8_t *data, int len, DMASerialCallback doneHandler,
                          void *doneArg)
    {
        if (!data || len <= 0)
            return DEVICE_INVALID_PARAMETER;
        if (txHandler)
            return DEVICE_BUSY;
        txHandler = doneHandler;
        txArg = doneArg;
        return DEVICE_OK;
    }

    virtual int startReceive(uint8_t *data, int len, DMASerialCallback doneHandler, void *doneArg)
    {
        if (!data || len <= 0)
            return DEVICE_INVALID_PARAMETER;
        if (rxHandler)
            return DEVICE_BUSY;
        rxHandler = doneHandler;
        rxArg = doneArg;
        return DEVICE_OK;
    }

    virtual int abortDMA() = 0;

    // these two schedule() until transmission is complete
    int send(uint8_t *data, int len);
    int receive(uint8_t *data, int len);
};

} // namespace codal

#endif