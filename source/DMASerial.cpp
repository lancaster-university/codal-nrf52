#include "DMASerial.h"
#include "Event.h"
#include "CodalFiber.h"

using namespace codal;

static void sendDone(void *p)
{
    Event(((DMASerial *)p)->id, 1000);
}

int DMASerial::send(uint8_t *data, int len)
{
    int r = startSend(data, len, sendDone, this);
    if (r)
        return r;
    fiber_wait_for_event(this->id, 1000);
    return DEVICE_OK;
}

static void recvDone(void *p)
{
    Event(((DMASerial *)p)->id, 1001);
}

int DMASerial::receive(uint8_t *data, int len)
{
    int r = startReceive(data, len, recvDone, this);
    if (r)
        return r;
    fiber_wait_for_event(this->id, 1001);
    return DEVICE_OK;
}
