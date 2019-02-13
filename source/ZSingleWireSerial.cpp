#include "ZSingleWireSerial.h"
#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "core_cm4.h"
#include "CodalDmesg.h"

using namespace codal;

#define TX_CONFIGURED       ((uint16_t)0x02)
#define RX_CONFIGURED       ((uint16_t)0x04)
#define FIRST_BREAK         ((uint16_t)0x08)

#define SWS_BUFFER_SIZE     ((int)256)

ZSingleWireSerial* ZSingleWireSerial::instance = NULL;

#ifdef __cplusplus
extern "C" {
#endif

void UARTE0_UART0_IRQHandler_v()
{
    if (ZSingleWireSerial::instance == NULL)
    {
        NRF_UARTE0->EVENTS_RXDRDY = 0;
        NRF_UARTE0->EVENTS_ENDRX = 0;
        NRF_UARTE0->EVENTS_ENDTX = 0;
        NRF_UARTE0->EVENTS_ERROR = 0;
        return;
    }

    int eventValue = 0;
    if (NRF_UART0->EVENTS_RXDRDY)
    {
        // after each byte received we receive an RXRDY interrupt, we use this interrupt to count the number of bytes received.
        ZSingleWireSerial::instance->bytes_received++;
        NRF_UART0->EVENTS_RXDRDY = 0;
    }
    if (NRF_UARTE0->EVENTS_ENDRX)
    {
        NRF_UARTE0->EVENTS_ENDRX = 0;
        ZSingleWireSerial::instance->configureRxInterrupt(0);
        eventValue = SWS_EVT_DATA_RECEIVED;
    }
    else if (NRF_UARTE0->EVENTS_ENDTX)
    {
        NRF_UARTE0->EVENTS_ENDTX = 0;
        ZSingleWireSerial::instance->configureTxInterrupt(0);
        eventValue = SWS_EVT_DATA_SENT;
    }
    else if (NRF_UARTE0->EVENTS_ERROR && (NRF_UARTE0->INTENSET & UARTE_INTENSET_ERROR_Msk))
    {
        NRF_UARTE0->EVENTS_ERROR = 0;
        DMESG("ERR %d", NRF_UARTE0->ERRORSRC);
        // clear error src
        NRF_UARTE0->ERRORSRC = NRF_UARTE0->ERRORSRC;
        eventValue = SWS_EVT_ERROR;
    }

    if (eventValue > 0)
    {
        Event evt(0, eventValue, 0, CREATE_ONLY);
        if (ZSingleWireSerial::instance->cb)
            ZSingleWireSerial::instance->cb->fire(evt);
    }
}

#ifdef __cplusplus
}
#endif


void ZSingleWireSerial::configureRxInterrupt(int enable)
{
    if (enable)
        // for some reason RXD RDY event is not provided in the definitions (0x2)
        NRF_UARTE0->INTENSET |= (UARTE_INTENSET_ENDRX_Msk | UARTE_INTENSET_ERROR_Msk | 0x2);
    else
        NRF_UARTE0->INTENCLR |= (UARTE_INTENCLR_ENDRX_Msk |  0x2);
}

void ZSingleWireSerial::configureTxInterrupt(int enable)
{
    if (enable)
        NRF_UARTE0->INTENSET |= (UARTE_INTENSET_ENDTX_Msk | UARTE_INTENSET_ERROR_Msk);
    else
        NRF_UARTE0->INTENCLR |= (UARTE_INTENCLR_ENDTX_Msk);
}

int ZSingleWireSerial::configureTx(int enable)
{
    if (enable && !(status & TX_CONFIGURED))
    {
        NRF_P0->DIR |= (1 << p.name);
        NRF_P0->PIN_CNF[p.name] =  3 << 2;
        NRF_UARTE0->PSEL.TXD = p.name;
        NRF_UARTE0->EVENTS_ENDTX = 0;
        NRF_UARTE0->ENABLE = 8;
        while(!(NRF_UARTE0->ENABLE));
        status |= TX_CONFIGURED;
    }
    else if (status & TX_CONFIGURED)
    {
        NRF_UARTE0->TASKS_STOPTX = 1;
        while(NRF_UARTE0->TASKS_STOPTX);
        NRF_UARTE0->ENABLE = 0;
        while((NRF_UARTE0->ENABLE));

        NRF_UARTE0->PSEL.TXD = 0xFFFFFFFF;
        status &= ~TX_CONFIGURED;
    }

    return DEVICE_OK;
}

int ZSingleWireSerial::configureRx(int enable)
{
    if (enable && !(status & RX_CONFIGURED))
    {
        NRF_P0->DIR &= ~(1 << p.name);
        NRF_P0->PIN_CNF[p.name] =  3 << 2;
        NRF_UARTE0->PSEL.RXD = p.name;
        NRF_UARTE0->EVENTS_ENDRX = 0;
        NRF_UARTE0->EVENTS_ERROR = 0;
        NRF_UARTE0->ENABLE = 8;
        while(!(NRF_UARTE0->ENABLE));
        status |= RX_CONFIGURED;
    }
    else if (enable == 0 && status & RX_CONFIGURED)
    {
        NRF_UARTE0->TASKS_STOPRX = 1;
        while(NRF_UARTE0->TASKS_STOPRX);
        NRF_UARTE0->ENABLE = 0;
        while((NRF_UARTE0->ENABLE));
        NRF_UARTE0->PSEL.RXD = 0xFFFFFFFF;
        status &= ~RX_CONFIGURED;
    }

    return DEVICE_OK;
}

ZSingleWireSerial::ZSingleWireSerial(Pin& p) : DMASingleWireSerial(p)
{
    if (instance == NULL)
        instance = this;

    status = 0;

    NRF_UARTE0->CONFIG = 0;

    // these lines are disabled
    NRF_UARTE0->PSEL.CTS = 0xFFFFFFFF;
    NRF_UARTE0->PSEL.RTS = 0xFFFFFFFF;

    // this will be set to pin name depending on configure TX/RX
    NRF_UARTE0->PSEL.TXD = 0xFFFFFFFF;
    NRF_UARTE0->PSEL.RXD = 0xFFFFFFFF;

    setBaud(1000000);

    NVIC_DisableIRQ(UARTE0_UART0_IRQn);
    NVIC_SetPriority(UARTE0_UART0_IRQn, 1);
    NVIC_SetVector(UARTE0_UART0_IRQn, (uint32_t)UARTE0_UART0_IRQHandler_v);
    NVIC_EnableIRQ(UARTE0_UART0_IRQn);

    status |= DEVICE_COMPONENT_RUNNING;
}

int ZSingleWireSerial::putc(char c)
{
    return DEVICE_NOT_IMPLEMENTED;
}
int ZSingleWireSerial::getc()
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::send(uint8_t* data, int len)
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::receive(uint8_t* data, int len)
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::sendDMA(uint8_t* data, int len)
{
    if (!(status & TX_CONFIGURED))
        setMode(SingleWireTx);

    NRF_UARTE0->TXD.PTR = (uint32_t)data;
    NRF_UARTE0->TXD.MAXCNT = len;

    configureTxInterrupt(1);

    NRF_UARTE0->TASKS_STARTTX = 1;

    return DEVICE_OK;
}

int ZSingleWireSerial::receiveDMA(uint8_t* data, int len)
{
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireRx);

    NRF_UARTE0->RXD.PTR = (uint32_t)data;
    NRF_UARTE0->RXD.MAXCNT = len;

    this->bytes_received = 0;
    configureRxInterrupt(1);

    NRF_UARTE0->TASKS_STARTRX = 1;

    return DEVICE_OK;
}

int ZSingleWireSerial::abortDMA()
{
    configureTxInterrupt(0);
    configureRxInterrupt(0);

    NRF_UARTE0->RXD.MAXCNT = 0;
    NRF_UARTE0->TXD.MAXCNT = 0;

    return DEVICE_OK;
}

int ZSingleWireSerial::setBaud(uint32_t baud)
{
    if (baud == 1000000)
        NRF_UARTE0->BAUDRATE = 0x10000000;
    else if (baud == 38400)
        NRF_UARTE0->BAUDRATE = 0x009D5000;
    else if (baud == 9600)
        NRF_UARTE0->BAUDRATE = 0x00275000;
    else
        // 115200
        NRF_UARTE0->BAUDRATE = 0x01D7E000;

    return DEVICE_OK;
}

uint32_t ZSingleWireSerial::getBaud()
{
    if (NRF_UARTE0->BAUDRATE == 0x10000000)
        return 1000000;

    if (NRF_UARTE0->BAUDRATE == 0x009D5000)
        return 38400;

    if (NRF_UARTE0->BAUDRATE == 0x00275000)
        return 9600;

    if (NRF_UARTE0->BAUDRATE == 0x01D7E000)
        return 115200;

    return 0;
}

int ZSingleWireSerial::getBytesTransmitted()
{
    return 0;
}

int ZSingleWireSerial::getBytesReceived()
{
    return bytes_received;
}

int ZSingleWireSerial::sendBreak()
{
    return DEVICE_NOT_IMPLEMENTED;
}