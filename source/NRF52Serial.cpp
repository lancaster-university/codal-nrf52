#include "NRF52Serial.h"

volatile int tx_complete = 1;
volatile int rx_complete = 1;

using namespace codal;

void nrf_uarte_handler(nrfx_uarte_event_t const * p_event, void * p_context)
{
    if (p_context == NULL)
        return;

    NRF52Serial* instance = (NRF52Serial*) p_context;

    if (p_event->type == NRFX_UARTE_EVT_TX_DONE)
    {
        tx_complete = 1;
        instance->dataTransmitted();
    }

    if (p_event->type == NRFX_UARTE_EVT_RX_DONE)
    {
        rx_complete = 1;
        instance->dataReceived(instance->getc());
    }
}

int NRF52Serial::enableInterrupt(SerialInterruptType t)
{
    if (t == RxInterrupt)
        uart_instance.p_reg->INTENSET |= (UARTE_INTENSET_ENDRX_Msk);

    if (t == TxInterrupt)
        uart_instance.p_reg->INTENSET |= (UARTE_INTENSET_ENDTX_Msk);
    return DEVICE_OK;
}

int NRF52Serial::disableInterrupt(SerialInterruptType t)
{
    if (t == TxInterrupt)
        uart_instance.p_reg->INTENCLR |= (UARTE_INTENCLR_ENDTX_Msk);

    if (t == RxInterrupt)
        uart_instance.p_reg->INTENCLR |= (UARTE_INTENCLR_ENDRX_Msk);
    return DEVICE_OK;
}

int NRF52Serial::setBaudrate(uint32_t baudrate)
{
    nrf_uarte_baudrate_t baud = NRF_UARTE_BAUDRATE_115200;

    if (baud == 1000000)
        baud = NRF_UARTE_BAUDRATE_1000000;
    else if (baud == 38400)
        baud = NRF_UARTE_BAUDRATE_38400;
    else if (baud == 9600)
        baud = NRF_UARTE_BAUDRATE_9600;

    nrf_uarte_baudrate_set(this->uart_instance.p_reg, baud);
    return DEVICE_OK;
}

int NRF52Serial::configurePins(Pin& tx, Pin& rx)
{
    this->tx = tx;
    this->rx = rx;
    nrf_uarte_txrx_pins_set(this->uart_instance.p_reg, tx.name, rx.name);
    return DEVICE_OK;
}

int NRF52Serial::putc(char c)
{
    while (!tx_complete);

    tx_complete = 0;
    int res = nrfx_uarte_tx(&this->uart_instance, (uint8_t*)&c, 1);
    if (res != NRFX_SUCCESS)
        return DEVICE_NOT_SUPPORTED;

    return DEVICE_OK;
}

int NRF52Serial::getc()
{
    int c = DEVICE_NO_DATA;

    while (!rx_complete);
    rx_complete = 0;

    nrfx_uarte_rx(&this->uart_instance, (uint8_t *)&c, 1);
    return c;
}

/**
 * Constructor
 *
 * @param tx the pin instance to use for transmission
 *
 * @param rx the pin instance to use for reception
 *
 **/
NRF52Serial::NRF52Serial(Pin& tx, Pin& rx, NRF_UARTE_Type* uart) : Serial(tx, rx)
{
    memset(&this->uart_instance, 0, sizeof(nrfx_uarte_t));
    this->uart_instance.p_reg = uart;

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
    hal_config.stop = NRF_UARTE_STOP_ONE;
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