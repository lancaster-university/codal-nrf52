#include "ZDMASerial.h"
#include "nrf.h"
#include "core_cm4.h"
#include "CodalDmesg.h"
#include "peripheral_alloc.h"
#include "codal_target_hal.h"

using namespace codal;

#define TX_CONFIGURED ((uint16_t)0x02)
#define RX_CONFIGURED ((uint16_t)0x04)
#define FIRST_BREAK ((uint16_t)0x08)

void ZDMASerial::irq_handler(void *inst)
{
    ((ZDMASerial *)inst)->irq_handler();
}

void ZDMASerial::irq_handler()
{
    if (uart->EVENTS_ENDRX)
    {
        uart->EVENTS_ENDRX = 0;
        stopRX();
    }
    else if (uart->EVENTS_ENDTX)
    {
        uart->EVENTS_ENDTX = 0;
        stopTX();
    }
    else if (uart->EVENTS_ERROR && (uart->INTEN & UARTE_INTENSET_ERROR_Msk))
    {
        uart->EVENTS_ERROR = 0;
        // if we're in reception mode, stop it (error doesn't automatically do so)
        uart->TASKS_STOPRX = 1;
        while (uart->TASKS_STOPRX)
            ;
        // clear error src
        uart->ERRORSRC = uart->ERRORSRC;
        // don't wait for ENDRX event, it takes additional 50uS to arrive
        stopRX();
    }
}

void ZDMASerial::stopRX()
{
    uart->INTENCLR = (UARTE_INTENCLR_ENDRX_Msk | UARTE_INTENSET_ERROR_Msk);
    DMASerialCallback f = this->rxHandler;
    if (!f)
        return;
    this->rxHandler = NULL;
    f(this->rxArg);
}

void ZDMASerial::stopTX()
{
    uart->INTENCLR = (UARTE_INTENCLR_ENDTX_Msk);
    DMASerialCallback f = this->txHandler;
    if (!f)
        return;
    this->txHandler = NULL;
    f(this->txArg);
}

int ZDMASerial::enableTx(bool enable)
{
    if (enable && !(status & TX_CONFIGURED))
    {
        NRF_P0->DIR |= (1 << tx.name);
        NRF_P0->PIN_CNF[tx.name] = 3 << 2; // this overrides DIR setting above
        uart->PSEL.TXD = tx.name;
        uart->EVENTS_ENDTX = 0;
        uart->ENABLE = 8;
        while (!(uart->ENABLE))
            ;
        status |= TX_CONFIGURED;
    }
    else if (status & TX_CONFIGURED)
    {
        uart->TASKS_STOPTX = 1;
        while (uart->TASKS_STOPTX)
            ;
        uart->ENABLE = 0;
        while ((uart->ENABLE))
            ;

        uart->PSEL.TXD = 0xFFFFFFFF;
        status &= ~TX_CONFIGURED;
    }

    return DEVICE_OK;
}

int ZDMASerial::enableRx(bool enable)
{
    if (enable && !(status & RX_CONFIGURED))
    {
        NRF_P0->DIR &= ~(1 << rx.name);
        NRF_P0->PIN_CNF[rx.name] = 3 << 2; // this overrides DIR setting above
        uart->PSEL.RXD = rx.name;
        uart->EVENTS_ENDRX = 0;
        uart->EVENTS_ERROR = 0;
        uart->ERRORSRC = uart->ERRORSRC;
        uart->ENABLE = 8;
        while (!(uart->ENABLE))
            ;
        status |= RX_CONFIGURED;
    }
    else if (enable == 0 && status & RX_CONFIGURED)
    {
        uart->TASKS_STOPRX = 1;
        while (uart->TASKS_STOPRX)
            ;
        uart->ENABLE = 0;
        while ((uart->ENABLE))
            ;
        uart->PSEL.RXD = 0xFFFFFFFF;
        status &= ~RX_CONFIGURED;
    }

    return DEVICE_OK;
}

ZDMASerial::ZDMASerial(Pin &tx, Pin &rx, uint16_t id) : DMASerial(tx, rx, id)
{
    uart = (NRF_UARTE_Type *)allocate_peripheral(PERI_MODE_UARTE);
    if (!uart)
        target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

    status = 0;

    uart->CONFIG = 0;

    // these lines are disabled
    uart->PSEL.CTS = 0xFFFFFFFF;
    uart->PSEL.RTS = 0xFFFFFFFF;

    // this will be set to pin name depending on configure TX/RX
    uart->PSEL.TXD = 0xFFFFFFFF;
    uart->PSEL.RXD = 0xFFFFFFFF;

    setBaud(1000000);

    IRQn_Type irqn = get_alloc_peri_irqn(uart);
    NVIC_DisableIRQ(irqn);
    NVIC_SetPriority(irqn, 1);
    set_alloc_peri_irq(uart, irq_handler, this);
    NVIC_EnableIRQ(irqn);

    status |= DEVICE_COMPONENT_RUNNING;
}

int ZDMASerial::startSend(const uint8_t *data, int len, DMASerialCallback doneHandler,
                          void *doneArg)
{
    int r = DMASerial::startSend(data, len, doneHandler, doneArg);
    if (r)
        return r;

    uart->TXD.PTR = (uint32_t)data;
    uart->TXD.MAXCNT = len;

    uart->INTENSET = (UARTE_INTENSET_ENDTX_Msk);

    uart->TASKS_STARTTX = 1;

    return DEVICE_OK;
}

int ZDMASerial::startReceive(uint8_t *data, int len, DMASerialCallback doneHandler, void *doneArg)
{
    int r = DMASerial::startReceive(data, len, doneHandler, doneArg);
    if (r)
        return r;

    uart->RXD.PTR = (uint32_t)data;
    uart->RXD.MAXCNT = len;

    uart->INTENSET = (UARTE_INTENSET_ENDRX_Msk | UARTE_INTENSET_ERROR_Msk);

    uart->TASKS_STARTRX = 1;

    return DEVICE_OK;
}

int ZDMASerial::abortDMA()
{
    stopTX();
    stopRX();

    uart->RXD.MAXCNT = 0;
    uart->TXD.MAXCNT = 0;

    return DEVICE_OK;
}

int ZDMASerial::setBaud(uint32_t baud)
{
    if (baud == 1000000)
        uart->BAUDRATE = 0x10000000;
    else if (baud == 38400)
        uart->BAUDRATE = 0x009D5000;
    else if (baud == 9600)
        uart->BAUDRATE = 0x00275000;
    else
        // 115200
        uart->BAUDRATE = 0x01D7E000;

    return DEVICE_OK;
}

uint32_t ZDMASerial::getBaud()
{
    if (uart->BAUDRATE == 0x10000000)
        return 1000000;

    if (uart->BAUDRATE == 0x009D5000)
        return 38400;

    if (uart->BAUDRATE == 0x00275000)
        return 9600;

    if (uart->BAUDRATE == 0x01D7E000)
        return 115200;

    return 0;
}
