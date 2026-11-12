#include "NRF52SerialUART.h"
#include "peripheral_alloc.h"
#include "NotifyEvents.h"

using namespace codal;

NRF52SerialUART::NRF52SerialUART(Pin& tx, Pin& rx)
 : NRF52SerialBase(tx, rx, NRF_UART0, PERI_MODE_UARTE),
   // UART0 and UARTE0 are one peripheral at one address
   p_uart_((NRF_UART_Type*)p_peri_)
{
    nrf_uart_config_t hal_config;
    hal_config.hwfc = NRF_UART_HWFC_DISABLED;
    hal_config.parity = NRF_UART_PARITY_EXCLUDED;
#if defined(UART_CONFIG_STOP_Msk)
    hal_config.stop = NRF_UART_STOP_ONE;
#endif
#if defined(UART_CONFIG_PARITYTYPE_Msk)
    hal_config.paritytype = NRF_UART_PARITYTYPE_EVEN;
#endif

    nrf_uart_baudrate_set(p_uart_, NRF_UART_BAUDRATE_115200);
    nrf_uart_configure(p_uart_, &hal_config);
    configurePins(tx, rx);

    nrf_uart_int_enable(p_uart_, NRF_UART_INT_MASK_TXDRDY);

    initIrq();

    nrf_uart_enable(p_uart_);
}

NRF52SerialUART::~NRF52SerialUART()
{
    nrf_uart_int_disable(p_uart_, NRF_UART_INT_MASK_RXDRDY |
                                  NRF_UART_INT_MASK_TXDRDY);
    disableIrq();

    nrf_uart_task_trigger(p_uart_, NRF_UART_TASK_STOPRX);
    nrf_uart_task_trigger(p_uart_, NRF_UART_TASK_STOPTX);

    nrf_uart_disable(p_uart_);
    nrf_uart_txrx_pins_disconnect(p_uart_);
}

void NRF52SerialUART::handleIrq()
{
    // Reading RXD lets the next byte in from the FIFO, and RXDRDY rises
    // again for it, so a late interrupt takes all that came meanwhile.
    while (nrf_uart_event_check(p_uart_, NRF_UART_EVENT_RXDRDY)){
        nrf_uart_event_clear(p_uart_, NRF_UART_EVENT_RXDRDY);
        dataReceived((char)nrf_uart_rxd_get(p_uart_));
    }

    if (nrf_uart_event_check(p_uart_, NRF_UART_EVENT_TXDRDY)){
        nrf_uart_event_clear(p_uart_, NRF_UART_EVENT_TXDRDY);

        is_tx_in_progress_ = false;
        if(txBufferedSize() > 0){
            dataTransmitted();
        }else{
            nrf_uart_task_trigger(p_uart_, NRF_UART_TASK_STOPTX);
        }
    }
}

int NRF52SerialUART::enableInterrupt(SerialInterruptType t)
{
    if (t == RxInterrupt){
        // initialiseRx() calls back here once the buffer is there
        if(!(status & CODAL_SERIAL_STATUS_RX_BUFF_INIT))
            return initialiseRx();

        nrf_uart_int_enable(p_uart_, NRF_UART_INT_MASK_RXDRDY);
        nrf_uart_task_trigger(p_uart_, NRF_UART_TASK_STARTRX);
    }else if(t == TxInterrupt){
        if(!is_tx_in_progress_ && txBufferedSize())
        {
            // To prevent the same data from being sent by the TXDRDY event
            // of the UART interrupt before processing the ring buffer.
            // Only the order in the Serial.dataTransmitted() function is different.
            uint16_t pre_txBuffTail = txBuffTail;
            txBuffTail = (txBuffTail + 1) % txBuffSize;
            putc((char)txBuff[pre_txBuffTail]);
            if(txBuffTail == txBuffHead){
                Event(DEVICE_ID_NOTIFY, CODAL_SERIAL_EVT_TX_EMPTY);
            }
        }
    }

    return DEVICE_OK;
}

int NRF52SerialUART::disableInterrupt(SerialInterruptType t)
{
    // TXDRDY carries the ring buffer out byte by byte, so it stays on;
    // NRF52SerialUART::configurePins relies on that
    if (t == RxInterrupt)
        nrf_uart_int_disable(p_uart_, NRF_UART_INT_MASK_RXDRDY);

    return DEVICE_OK;
}

int NRF52SerialUART::setBaudrate(uint32_t baudrate)
{
    nrf_uart_baudrate_t baud = NRF_UART_BAUDRATE_115200;

    switch(baudrate)
    {
        case 9600 : baud = NRF_UART_BAUDRATE_9600; break;
        case 31250 : baud = NRF_UART_BAUDRATE_31250; break;
        case 38400 : baud = NRF_UART_BAUDRATE_38400; break;
        case 57600 : baud = NRF_UART_BAUDRATE_57600; break;
        case 115200 : baud = NRF_UART_BAUDRATE_115200; break;
        case 230400 : baud = NRF_UART_BAUDRATE_230400; break;
        case 921600 : baud = NRF_UART_BAUDRATE_921600; break;
        case 1000000 : baud = NRF_UART_BAUDRATE_1000000; break;
    }

    nrf_uart_baudrate_set(p_uart_, baud);
    return DEVICE_OK;
}

void NRF52SerialUART::setTxRxPins(Pin& tx, Pin& rx)
{
    nrf_uart_txrx_pins_set(p_uart_, tx.name, rx.name);
}

int NRF52SerialUART::putc(char c)
{
    // The byte before is out once the handler has taken its TXDRDY or,
    // where the handler cannot run, once TXDRDY is up
    while (is_tx_in_progress_ && !nrf_uart_event_check(p_uart_, NRF_UART_EVENT_TXDRDY))
        /*wait*/;

    nrf_uart_event_clear(p_uart_, NRF_UART_EVENT_TXDRDY);
    is_tx_in_progress_ = true;
    nrf_uart_task_trigger(p_uart_, NRF_UART_TASK_STARTTX);
    nrf_uart_txd_set(p_uart_, (uint8_t)c);

    return DEVICE_OK;
}
