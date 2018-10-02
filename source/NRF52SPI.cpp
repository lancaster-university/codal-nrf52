/*
The MIT License (MIT)

Copyright (c) 2017 Lancaster University.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include "CodalConfig.h"
#include "NRF52SPI.h"
#include "ErrorNo.h"
#include "CodalDmesg.h"
#include "codal-core/inc/types/Event.h"
#include "CodalFiber.h"
#include "nrf_nvic.h"

#define USE_SPIM3 0

#ifdef TARGET_MCU_NRF52840
// SPIM3 has hardware bugs
// #define USE_SPIM3 1
#endif

#if USE_SPIM3
#define THE_SPIM NRF_SPIM3
#define THE_IRQ SPIM3_IRQn
#define THE_HANDLER SPIM3_IRQHandler_v
#define SZLIMIT 0xffff
#else
#define THE_SPIM NRF_SPIM0
#define THE_IRQ SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn
#define THE_HANDLER SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler_v
#define SZLIMIT 0xff
#endif

namespace codal
{

// in future, we might need once instance per SPIM instance (up to 4)
static NRF52SPI *instance;

/**
 * Constructor.
 */
NRF52SPI::NRF52SPI(Pin &mosi, Pin &miso, Pin &sclk)
    : codal::SPI(), mosi(mosi), miso(miso), sck(sclk)
{
    p_spim = THE_SPIM;
    IRQn = THE_IRQ;
    configured = 0;
    setFrequency(1000000);
    setMode(0);
    instance = this;
    doneHandler = NULL;
}

extern "C" void THE_HANDLER()
{
    if (nrf_spim_event_check(THE_SPIM, NRF_SPIM_EVENT_END))
    {
        nrf_spim_event_clear(THE_SPIM, NRF_SPIM_EVENT_END);
        instance->_irqDoneHandler();
    }
}

void NRF52SPI::_irqDoneHandler()
{
    if (doneHandler)
    {
        PVoidCallback done = doneHandler;
        doneHandler = NULL;
        done(doneHandlerArg);
    }
    else
    {
        Event(DEVICE_ID_SPI, 3);
    }
}

int NRF52SPI::xfer(uint8_t const *p_tx_buffer, uint32_t tx_length, uint8_t *p_rx_buffer,
                   uint32_t rx_length, PVoidCallback doneHandler, void *arg)
{
    config();
    
    if (tx_length > SZLIMIT || rx_length > SZLIMIT)
        return DEVICE_INVALID_PARAMETER;

    nrf_spim_tx_buffer_set(p_spim, p_tx_buffer, tx_length);
    nrf_spim_rx_buffer_set(p_spim, p_rx_buffer, rx_length);
    nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);
    nrf_spim_tx_list_disable(p_spim);
    nrf_spim_rx_list_disable(p_spim);

    if (doneHandler == NULL)
    {
        fiber_wake_on_event(DEVICE_ID_SPI, 3);
    }
    else
    {
        this->doneHandler = doneHandler;
        this->doneHandlerArg = arg;
    }

    nrf_spim_task_trigger(p_spim, NRF_SPIM_TASK_START);
    nrf_spim_int_enable(p_spim, NRF_SPIM_INT_END_MASK);

    if (doneHandler == NULL)
        schedule();

    return 0;
}

int NRF52SPI::transfer(const uint8_t *txBuffer, uint32_t txSize, uint8_t *rxBuffer,
                       uint32_t rxSize)
{
    if (txSize <= SZLIMIT && rxSize <= SZLIMIT)
        return xfer(txBuffer, txSize, rxBuffer, rxSize, NULL, NULL);
    else
        return codal::SPI::transfer(txBuffer, txSize, rxBuffer, rxSize);
}

int NRF52SPI::startTransfer(const uint8_t *txBuffer, uint32_t txSize, uint8_t *rxBuffer,
                            uint32_t rxSize, PVoidCallback doneHandler, void *arg)
{
    if (doneHandler == NULL)
        return DEVICE_INVALID_PARAMETER;

    if (txSize <= SZLIMIT && rxSize <= SZLIMIT)
        return xfer(txBuffer, txSize, rxBuffer, rxSize, doneHandler, arg);
    else
        return codal::SPI::startTransfer(txBuffer, txSize, rxBuffer, rxSize, doneHandler, arg);
}

void NRF52SPI::config()
{
    if (configured)
        return;
    configured = 1;

    sck.setDigitalValue(mode <= 1 ? 0 : 1);
    NRF_GPIO->PIN_CNF[sck.name] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                                  (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                  (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                  (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                  (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    uint32_t mosi_pin = 0xffffffff;
    uint32_t miso_pin = 0xffffffff;
    if (&mosi)
    {
        mosi.setDigitalValue(0);
        mosi_pin = mosi.name;
    }
    if (&miso)
    {
        miso.getDigitalValue();
        miso_pin = miso.name;
    }

    nrf_spim_disable(p_spim);
    nrf_spim_pins_set(p_spim, sck.name, mosi_pin, miso_pin);
    nrf_spim_frequency_set(p_spim, (nrf_spim_frequency_t)freq);
    nrf_spim_configure(p_spim, (nrf_spim_mode_t)mode, NRF_SPIM_BIT_ORDER_MSB_FIRST);
    nrf_spim_orc_set(p_spim, 0);
    nrf_spim_int_enable(p_spim, NRF_SPIM_INT_END_MASK | NRF_SPIM_INT_STOPPED_MASK);
    nrf_spim_enable(p_spim);

    NVIC_SetPriority(IRQn, 7);
    NVIC_ClearPendingIRQ(IRQn);
    NVIC_EnableIRQ(IRQn);

    DMESG("SPI config done f=%p", freq);
}

/** Set the frequency of the SPI interface
 *
 * @param frequency The bus frequency in hertz
 */
int NRF52SPI::setFrequency(uint32_t frequency)
{
#if USE_SPIM3
    if (frequency >= 32000000)
        freq = (nrf_drv_spi_frequency_t)0x14000000; // 32M
    else if (frequency >= 16000000)
        freq = (nrf_drv_spi_frequency_t)0x0A000000; // 16M
    else
#endif
    if (frequency >= 8000000)
        freq = NRF_DRV_SPI_FREQ_8M;
    else if (frequency >= 4000000)
        freq = NRF_DRV_SPI_FREQ_4M;
    else if (frequency >= 2000000)
        freq = NRF_DRV_SPI_FREQ_2M;
    else if (frequency >= 1000000)
        freq = NRF_DRV_SPI_FREQ_1M;
    else if (frequency >= 500000)
        freq = NRF_DRV_SPI_FREQ_500K;
    else if (frequency >= 250000)
        freq = NRF_DRV_SPI_FREQ_250K;
    else
        freq = NRF_DRV_SPI_FREQ_125K;
    return DEVICE_OK;
}

/** Set the mode of the SPI interface
 *
 * @param mode Clock polarity and phase mode (0 - 3)
 * @param bits Number of bits per SPI frame (4 - 16)
 *
 * @code
 * mode | POL PHA
 * -----+--------
 *   0  |  0   0
 *   1  |  0   1
 *   2  |  1   0
 *   3  |  1   1
 * @endcode
 */
int NRF52SPI::setMode(int mode, int bits)
{
    this->mode = mode;
    if (bits != 8)
        return DEVICE_INVALID_PARAMETER;
    return DEVICE_OK;
}

/**
 * Writes the given byte to the SPI bus.
 *
 * The CPU will busy wait until the transmission is complete.
 *
 * @param data The data to write.
 * @return Response from the SPI slave or DEVICE_SPI_ERROR if the the write request failed.
 */
int NRF52SPI::write(int data)
{
    sendCh = data;
    int ret = xfer(&sendCh, 1, &recvCh, 1, NULL, NULL);
    return (ret < 0) ? (int)DEVICE_SPI_ERROR : recvCh;
}
}
