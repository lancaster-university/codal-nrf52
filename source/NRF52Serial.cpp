#include "NRF52Serial.h"
#include "sync_serial.h"
#include "NotifyEvents.h"

using namespace codal;

void nrf_uarte_handler(nrfx_uarte_event_t const * p_event, void * p_context)
{
    if (p_context == NULL)
        return;

    NRF52Serial* instance = (NRF52Serial*) p_context;

    if(p_event->type == NRFX_UARTE_EVT_RX_DONE){
        instance->dataReceived(instance->rx_byte_buf_);
        nrfx_uarte_rx(&instance->uart_instance, &instance->rx_byte_buf_, 1);
    }else if(p_event->type == NRFX_UARTE_EVT_TX_DONE){
        if(instance->txBufferedSize() > 0){
            instance->dataTransmitted();
        }
    }else if(p_event->type == NRFX_UARTE_EVT_ERROR){
        nrfx_uarte_rx(&instance->uart_instance, &instance->rx_byte_buf_, 1);
    }
}

int NRF52Serial::enableInterrupt(SerialInterruptType t)
{
    if(t == TxInterrupt){
        if(!nrfx_uarte_tx_in_progress(&uart_instance)){
            uart_instance.p_reg->INTENSET = (NRF_UARTE_INT_ENDTX_MASK |
                                            NRF_UARTE_INT_TXSTOPPED_MASK);
            // To prevent the same data from being sent by the TX_DONE event
            // of the UARTE interrupt before processing the ring buffer.
            // Only the order in the Serial.dataTransmitted() function is different.
            uint16_t pre_txBuffTail = txBuffTail;
            txBuffTail = (txBuffTail + 1) % txBuffSize;
            putc((char)txBuff[pre_txBuffTail]);
            if(txBuffTail == txBuffHead){
                Event(DEVICE_ID_NOTIFY, CODAL_SERIAL_EVT_TX_EMPTY);
                disableInterrupt(TxInterrupt);
            }
        }
    }else if (t == RxInterrupt){
        uart_instance.p_reg->INTENSET = (NRF_UARTE_INT_ENDRX_MASK | 
                                        NRF_UARTE_INT_ERROR_MASK |
                                        NRF_UARTE_INT_RXTO_MASK);
        nrfx_uarte_rx(&uart_instance, &rx_byte_buf_, 1);
    }
    return DEVICE_OK;
}

int NRF52Serial::disableInterrupt(SerialInterruptType t)
{
    if (t == TxInterrupt){
        uart_instance.p_reg->INTENCLR = (NRF_UARTE_INT_ENDTX_MASK |
                                         NRF_UARTE_INT_TXSTOPPED_MASK);
    }
    else if (t == RxInterrupt){
        uart_instance.p_reg->INTENCLR = (NRF_UARTE_INT_ENDRX_MASK | 
                                          NRF_UARTE_INT_ERROR_MASK |
                                          NRF_UARTE_INT_RXTO_MASK);
        uart_instance.p_reg->SHORTS &= ~(NRF_UARTE_SHORT_ENDRX_STARTRX);
    }
    return DEVICE_OK;
}

int NRF52Serial::setBaudrate(uint32_t baudrate)
{
    nrf_uarte_baudrate_t baud = NRF_UARTE_BAUDRATE_115200;

    switch(baudrate)
    {
        case 9600 : baud = NRF_UARTE_BAUDRATE_9600; break;
        case 38400 : baud = NRF_UARTE_BAUDRATE_38400; break;
        case 57600 : baud = NRF_UARTE_BAUDRATE_57600; break;
        case 115200 : baud = NRF_UARTE_BAUDRATE_115200; break;
        case 230400 : baud = NRF_UARTE_BAUDRATE_230400; break;
        case 921600 : baud = NRF_UARTE_BAUDRATE_921600; break;
        case 1000000 : baud = NRF_UARTE_BAUDRATE_1000000; break;
    }

    nrf_uarte_baudrate_set(uart_instance.p_reg, baud);
    return DEVICE_OK;
}

int NRF52Serial::configurePins(Pin& tx, Pin& rx)
{
    this->tx = tx;
    this->rx = rx;
    nrf_uarte_txrx_pins_set(uart_instance.p_reg, tx.name, rx.name);
    return DEVICE_OK;
}

int NRF52Serial::putc(char c)
{
    // Added because serial class functions, including printf, require blocking.
    while(nrfx_uarte_tx_in_progress(&uart_instance));

    int res = nrfx_uarte_tx(&uart_instance, (uint8_t*)&c, 1);
    switch(res)
    {
        case NRFX_ERROR_BUSY: res=DEVICE_BUSY; break;
        case NRFX_ERROR_INVALID_ADDR: res=DEVICE_NO_RESOURCES; break;
        case NRFX_ERROR_FORBIDDEN: res=DEVICE_CANCELLED; break;
        default: res=DEVICE_OK; break;
    }

    return res;
}

int NRF52Serial::getc()
{
    return this->getChar(codal::SerialMode::ASYNC);
}

int NRF52Serial::write(uint8_t *buffer, int bufferLen, SerialMode mode)
{
    if(txInUse())
        return DEVICE_SERIAL_IN_USE;

    if(bufferLen <= 0 || buffer == NULL)
        return DEVICE_INVALID_PARAMETER;

    lockTx();
    
    int res = nrfx_uarte_tx(&uart_instance, buffer, bufferLen);
    switch(res)
    {
        case NRFX_ERROR_BUSY: res=DEVICE_BUSY; break;
        case NRFX_ERROR_INVALID_ADDR: res=DEVICE_NO_RESOURCES; break;
        case NRFX_ERROR_FORBIDDEN: res=DEVICE_CANCELLED; break;
        default: res=DEVICE_OK; break;
    }
    if(res==DEVICE_OK){
        if(mode==SYNC_SPINWAIT){
            while(nrfx_uarte_tx_in_progress(&uart_instance));
        }else if(mode==SYNC_SLEEP){
            fiber_sleep(0);
        }
        res = bufferLen;
    }

    unlockTx();

    return res;    
}

/**
 * Constructor
 *
 * @param tx the pin instance to use for transmission
 *
 * @param rx the pin instance to use for reception
 *
 **/
NRF52Serial::NRF52Serial(Pin& tx, Pin& rx) 
 : Serial(tx, rx), rx_byte_buf_(0)
{
    memset(&this->uart_instance, 0, sizeof(nrfx_uarte_t));
    
    this->uart_instance.p_reg = (NRF_UARTE_Type*)allocate_sync_serial(SYNC_SERIAL_MODE_UARTE);
    
    if(this->uart_instance.p_reg == NRF_UARTE0){
        this->uart_instance.drv_inst_idx = NRFX_UARTE0_INST_IDX;
    }
#if defined(NRFX_UARTE1_ENABLED)
    else if(this->uart_instance.p_reg == NRF_UARTE1){
        this->uart_instance.drv_inst_idx = NRFX_UARTE1_INST_IDX;
    }
#endif

    nrfx_uarte_config_t uart_config;

    uart_config.pseltxd = (uint32_t) tx.name;
    uart_config.pselrxd = (uint32_t) rx.name;
    uart_config.pselcts = 0xFFFFFFFF;
    uart_config.pselrts = 0xFFFFFFFF;
    uart_config.p_context = this;
    uart_config.baudrate = NRF_UARTE_BAUDRATE_115200;
    uart_config.interrupt_priority = 1;

    nrf_uarte_config_t hal_config;
    hal_config.hwfc = NRF_UARTE_HWFC_DISABLED;
    hal_config.parity = NRF_UARTE_PARITY_EXCLUDED;
#if defined(UARTE_CONFIG_STOP_Msk)    
    hal_config.stop = NRF_UARTE_STOP_ONE;
#endif    
#if defined(UARTE_CONFIG_PARITYTYPE_Msk)
    hal_config.paritytype = NRF_UARTE_PARITYTYPE_EVEN;
#endif

    uart_config.hal_cfg = hal_config;

    nrfx_uarte_init(&this->uart_instance, &uart_config, nrf_uarte_handler);
}

NRF52Serial::~NRF52Serial()
{
    nrfx_uarte_uninit(&this->uart_instance);
}