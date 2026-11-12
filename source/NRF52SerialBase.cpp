#include "NRF52SerialBase.h"
#include "ErrorNo.h"
#include "codal_target_hal.h"

using namespace codal;

NRF52SerialBase::NRF52SerialBase(Pin& tx, Pin& rx, void *device, PeripheralMode mode,
                                 uint8_t rxBufferSize, uint8_t txBufferSize, uint16_t id)
 : Serial(tx, rx, rxBufferSize, txBufferSize, id), p_peri_(NULL),
   is_tx_in_progress_(false)
{
    if(device != NULL)
        p_peri_ = allocate_peripheral(device);
    else
        p_peri_ = allocate_peripheral(mode);

    if(p_peri_ == NULL)
        target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);
}

NRF52SerialBase::~NRF52SerialBase()
{
    free_alloc_peri(p_peri_);
}

void NRF52SerialBase::initIrq()
{
    set_alloc_peri_irq(p_peri_, &_irqHandler, this);

    IRQn_Type IRQn = get_alloc_peri_irqn(p_peri_);

    // Priority 1 keeps the radio (priority 0) above these handlers, but the
    // heap and fiber critical sections mask priorities 1-7 and can hold them
    // off. See NRF52SerialUART for what that means for reception.
    NVIC_SetPriority(IRQn, 1);
    NVIC_ClearPendingIRQ(IRQn);
    NVIC_EnableIRQ(IRQn);
}

void NRF52SerialBase::disableIrq()
{
    NVIC_DisableIRQ(get_alloc_peri_irqn(p_peri_));
}

void NRF52SerialBase::_irqHandler(void *self_)
{
    NRF52SerialBase *self = (NRF52SerialBase *)self_;
    self->handleIrq();
}

int NRF52SerialBase::getc()
{
    return this->getChar(codal::SerialMode::ASYNC);
}

int NRF52SerialBase::configurePins(Pin& tx, Pin& rx)
{
    //Serial::redirect surrounds its call to this function with
    //disableInterrupt(TxInterrupt) and enableInterrupt(TxInterrupt)
    //but NRF52Serial's implementation of those doesn't change the interrupt.
    //When we get here tx is locked, but the tx interrupt is still working to empty the buffer
    while (txBufferedSize() > 0 || is_tx_in_progress_) /*wait*/;

    setTxRxPins(tx, rx);

    return DEVICE_OK;
}

int NRF52SerialBase::setSleep(bool doSleep)
{
    IRQn_Type IRQn = get_alloc_peri_irqn( p_peri_ );

    if ( doSleep && !(status & CODAL_SERIAL_STATUS_DEEPSLEEP) ) {
        // Disable the RX interrupt and IRQ. Clear the buffers
        disableInterrupt( RxInterrupt );

        // wait...
        while ( txBufferedSize() > 0 || is_tx_in_progress_ );

        NVIC_DisableIRQ( IRQn );

    } else {
        // Reconnect IRQs and re-set settings
        NVIC_ClearPendingIRQ( IRQn );
        NVIC_EnableIRQ( IRQn );

        enableInterrupt( RxInterrupt );

        if( txBufferedSize() > 0 )
            enableInterrupt( TxInterrupt );

        this->setBaud( this->baudrate );
    }

    return DEVICE_OK;
}
