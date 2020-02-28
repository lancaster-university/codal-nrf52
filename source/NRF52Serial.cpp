#include "NRF52Serial.h"
#include "peripheral_alloc.h"
#include "NotifyEvents.h"

using namespace codal;

extern int8_t target_get_irq_disabled();

/**
 * Constructor
 *
 * @param tx the pin instance to use for transmission
 *
 * @param rx the pin instance to use for reception
 *
 **/
NRF52Serial::NRF52Serial(Pin& tx, Pin& rx, NRF_UARTE_Type* device) 
 : Serial(tx, rx), is_configured_(false), is_tx_in_progress_(false)
{
    if(device != NULL)
        p_uarte_ = (NRF_UARTE_Type*)allocate_peripheral((void*)device);
    configurePins(tx,rx);
}

NRF52Serial::~NRF52Serial()
{
    nrf_uarte_int_disable(p_uarte_, NRF_UARTE_INT_RXDRDY_MASK|
                                    NRF_UARTE_INT_ENDRX_MASK |
                                    NRF_UARTE_INT_ENDTX_MASK |
                                    NRF_UARTE_INT_ERROR_MASK |
                                    NRF_UARTE_INT_RXTO_MASK  |
                                    NRF_UARTE_INT_TXSTOPPED_MASK);
    NVIC_DisableIRQ(get_alloc_peri_irqn(p_uarte_));

    // Make sure all transfers are finished before UARTE is disabled
    // to achieve the lowest power consumption.
    nrf_uarte_shorts_disable(p_uarte_, NRF_UARTE_SHORT_ENDRX_STARTRX);
    nrf_uarte_task_trigger(p_uarte_, NRF_UARTE_TASK_STOPRX);
    nrf_uarte_event_clear(p_uarte_, NRF_UARTE_EVENT_TXSTOPPED);
    nrf_uarte_task_trigger(p_uarte_, NRF_UARTE_TASK_STOPTX);
    while (!nrf_uarte_event_check(p_uarte_, NRF_UARTE_EVENT_TXSTOPPED))
    {}

    nrf_uarte_disable(p_uarte_);
    nrf_uarte_txrx_pins_disconnect(p_uarte_);
}

void NRF52Serial::_irqHandler(void *self_)
{
    NRF52Serial *self = (NRF52Serial *)self_;
    NRF_UARTE_Type *p_uarte = self->p_uarte_;

    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXDRDY)){
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXDRDY);
        self->dataReceivedDMA();
    }
    
    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDRX)){
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);
        self->updateRxBufferAfterENDRX();
        nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTRX);
    }else if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ERROR)){
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ERROR);
        (void) nrf_uarte_errorsrc_get_and_clear(p_uarte);
        // RX transaction abort
        nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPRX);
        self->updateRxBufferAfterENDRX();
        nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTRX);        
    }

    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXTO)){
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXTO);
        // nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_FLUSHRX);
        uint32_t rx_cnt = nrf_uarte_rx_amount_get(p_uarte);
        while(rx_cnt-- > 0){
            self->dataReceivedDMA();
        }
    }

    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDTX)){
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDTX);

        self->is_tx_in_progress_ = false;
        if(self->txBufferedSize() > 0){
            self->dataTransmitted();
        }else{
            // Transmitter has to be stopped by triggering STOPTX task to achieve
            // the lowest possible level of the UARTE power consumption.
            nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPTX);
        }
    }

    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_TXSTOPPED)){
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_TXSTOPPED);
        self->is_tx_in_progress_ = false;
    }
}


int NRF52Serial::enableInterrupt(SerialInterruptType t)
{
    if(is_configured_ == false)
        return DEVICE_INVALID_STATE;

    if (t == RxInterrupt){
        if(!(status & CODAL_SERIAL_STATUS_RX_BUFF_INIT))
            initialiseRx();
        if(status & CODAL_SERIAL_STATUS_RX_BUFF_INIT){
            nrf_uarte_rx_buffer_set(p_uarte_, rxBuff, rxBuffSize);
            nrf_uarte_int_enable(p_uarte_, NRF_UARTE_INT_ERROR_MASK |
                                            NRF_UARTE_INT_ENDRX_MASK);
            nrf_uarte_task_trigger(p_uarte_, NRF_UARTE_TASK_STARTRX);
        }           
    }else if(t == TxInterrupt){
        if(!is_tx_in_progress_ && txBufferedSize())
        {
            // To prevent the same data from being sent by the TX_DONE event
            // of the UARTE interrupt before processing the ring buffer.
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

int NRF52Serial::disableInterrupt(SerialInterruptType t)
{
    if(is_configured_ == false)
        return DEVICE_INVALID_STATE;

    if (t == RxInterrupt){
        nrf_uarte_int_disable(p_uarte_, NRF_UARTE_INT_ERROR_MASK |
                                        NRF_UARTE_INT_ENDRX_MASK);
    }else if (t == TxInterrupt){
        // IDLE:
        // Since UARTE (DMA) is used, there is no need to turn off and turn off interrupts.
        // In addition, using a function that does not use the codal::Serial structure,
        // such as printf and putc, causes problems, 
        // so it is not right to turn on and off the driver interrupts in this function.
    }

    return DEVICE_OK;
}

int NRF52Serial::setBaudrate(uint32_t baudrate)
{
    if(is_configured_ == false)
        return DEVICE_INVALID_STATE;

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

    nrf_uarte_baudrate_set(p_uarte_, baud);
    return DEVICE_OK;
}

int NRF52Serial::configurePins(Pin& tx, Pin& rx)
{
    int ret;

    this->tx = tx;
    this->rx = rx;

    if(p_uarte_ == NULL){
        p_uarte_ = (NRF_UARTE_Type*)allocate_peripheral(PERI_MODE_UARTE);
    }

    if(p_uarte_ != NULL){
        if(is_configured_ == true){
            nrf_uarte_txrx_pins_set(p_uarte_, tx.name, rx.name);
        }else{
            nrf_uarte_config_t hal_config;
            hal_config.hwfc = NRF_UARTE_HWFC_DISABLED;
            hal_config.parity = NRF_UARTE_PARITY_EXCLUDED;
        #if defined(UARTE_CONFIG_STOP_Msk)    
            hal_config.stop = NRF_UARTE_STOP_ONE;
        #endif    
        #if defined(UARTE_CONFIG_PARITYTYPE_Msk)
            hal_config.paritytype = NRF_UARTE_PARITYTYPE_EVEN;
        #endif

            nrf_uarte_baudrate_set(p_uarte_, NRF_UARTE_BAUDRATE_115200);
            nrf_uarte_configure(p_uarte_, &hal_config);
            nrf_uarte_txrx_pins_set(p_uarte_, tx.name, rx.name);

            nrf_uarte_event_clear(p_uarte_, NRF_UARTE_EVENT_RXDRDY);
            nrf_uarte_event_clear(p_uarte_, NRF_UARTE_EVENT_ENDRX);
            nrf_uarte_event_clear(p_uarte_, NRF_UARTE_EVENT_ENDTX);
            nrf_uarte_event_clear(p_uarte_, NRF_UARTE_EVENT_ERROR);
            nrf_uarte_event_clear(p_uarte_, NRF_UARTE_EVENT_RXTO);
            nrf_uarte_event_clear(p_uarte_, NRF_UARTE_EVENT_TXSTOPPED);
            nrf_uarte_int_enable(p_uarte_,  NRF_UARTE_INT_RXDRDY_MASK|
                                            NRF_UARTE_INT_ENDRX_MASK |
                                            NRF_UARTE_INT_ENDTX_MASK |
                                            NRF_UARTE_INT_ERROR_MASK |
                                            NRF_UARTE_INT_RXTO_MASK  |
                                            NRF_UARTE_INT_TXSTOPPED_MASK);

            set_alloc_peri_irq(p_uarte_, &_irqHandler, this);

            IRQn_Type IRQn = get_alloc_peri_irqn(p_uarte_);

            NVIC_SetPriority(IRQn, 1);
            NVIC_ClearPendingIRQ(IRQn);
            NVIC_EnableIRQ(IRQn);            

            nrf_uarte_enable(p_uarte_);

            is_configured_ = true;
        }
        ret = DEVICE_OK;
    }else{
        ret = DEVICE_NO_RESOURCES;
    }
    
    return ret;
}

int NRF52Serial::putc(char c)
{
    if(is_configured_ == false)
        return DEVICE_INVALID_STATE;

    int res = DEVICE_OK;

    while(!target_get_irq_disabled() && is_tx_in_progress_);

    if(target_get_irq_disabled()){
        nrf_uarte_event_clear(p_uarte_, NRF_UARTE_EVENT_ENDTX);
        nrf_uarte_event_clear(p_uarte_, NRF_UARTE_EVENT_TXSTOPPED);
    }
    is_tx_in_progress_ = true;
    nrf_uarte_tx_buffer_set(p_uarte_, (const uint8_t*)&c, 1);
    nrf_uarte_task_trigger(p_uarte_, NRF_UARTE_TASK_STARTTX);

    // Block for when not using Interrupt. (like codal::Serial::prtinf())
    if(target_get_irq_disabled()){
        bool endtx;
        bool txstopped;
        do
        {
            endtx     = nrf_uarte_event_check(p_uarte_, NRF_UARTE_EVENT_ENDTX);
            txstopped = nrf_uarte_event_check(p_uarte_, NRF_UARTE_EVENT_TXSTOPPED);
        }
        while ((!endtx) && (!txstopped));
        if (txstopped){
            res = DEVICE_INVALID_STATE;
        }else{
            // Transmitter has to be stopped by triggering the STOPTX task to achieve
            // the lowest possible level of the UARTE power consumption.
            nrf_uarte_task_trigger(p_uarte_, NRF_UARTE_TASK_STOPTX);
            while(!nrf_uarte_event_check(p_uarte_, NRF_UARTE_EVENT_TXSTOPPED))
            {}
        }
        is_tx_in_progress_ = false;
    }

    return res;
}

int NRF52Serial::getc()
{
    if(is_configured_ == false)
        return DEVICE_INVALID_STATE;

    return this->getChar(codal::SerialMode::ASYNC);
}

bool NRF52Serial::isConfigured() const
{
    return is_configured_;
}

void NRF52Serial::dataReceivedDMA()
{
    if(!(status & CODAL_SERIAL_STATUS_RX_BUFF_INIT))
        return;

    int delimeterOffset = 0;
    int delimLength = this->delimeters.length();

    //iterate through our delimeters (if any) to see if there is a match
    while(delimeterOffset < delimLength)
    {
        //fire an event if there is to block any waiting fibers
        if(this->delimeters.charAt(delimeterOffset) == this->rxBuff[rxBuffHead])
            Event(this->id, CODAL_SERIAL_EVT_DELIM_MATCH);

        delimeterOffset++;
    }

    uint16_t newHead = (rxBuffHead + 1) % rxBuffSize;

    //look ahead to our newHead value to see if we are about to collide with the tail
    if(newHead != rxBuffTail)
    {
        //if we are not, update our actual head.
        rxBuffHead = newHead;

        //if we have any fibers waiting for a specific number of characters, unblock them
        if(rxBuffHeadMatch >= 0)
            if(rxBuffHead == rxBuffHeadMatch)
            {
                rxBuffHeadMatch = -1;
                Event(this->id, CODAL_SERIAL_EVT_HEAD_MATCH);
            }

        status |= CODAL_SERIAL_STATUS_RXD;
    }
    else
        //otherwise, our buffer is full, send an event to the user...
        Event(this->id, CODAL_SERIAL_EVT_RX_FULL);    
}


void NRF52Serial::updateRxBufferAfterENDRX()
{
    if(is_configured_ == false)
        return;

    static uint8_t rx_dummy_byte_;

    int rx_buffered_size = rxBufferedSize();
    if(rx_buffered_size == 0){
        nrf_uarte_rx_buffer_set(p_uarte_, rxBuff, rxBuffSize);
    }else if(rxBuffSize > rx_buffered_size+1){
        nrf_uarte_rx_buffer_set(p_uarte_, &rxBuff[rxBuffHead],
            rxBuffSize-rxBuffHead);
    }else if(rxBuffSize == rx_buffered_size+1){ // rxBuff is full.
        // Set receive buffer using dummy byte buffer to keep DMA active.
        nrf_uarte_rx_buffer_set(p_uarte_, &rx_dummy_byte_, 1);
    }
}
