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


#ifdef XTARGET_MCU_NRF52840
#define THE_SPIM NRF_SPIM3
#define THE_IRQ SPIM3_IRQn
#else
#define THE_SPIM NRF_SPIM0
#define THE_IRQ SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn
#endif

namespace codal
{

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
}

extern "C" void SPIM3_IRQHandler_v()
{
    DMESG("IRQ");
    if (nrf_spim_event_check(THE_SPIM, NRF_SPIM_EVENT_END))
    {
        nrf_spim_event_clear(THE_SPIM, NRF_SPIM_EVENT_END);
        Event(DEVICE_ID_SPI, 3);
    }
}

int NRF52SPI::xfer(uint8_t const *p_tx_buffer, uint16_t tx_length, uint8_t *p_rx_buffer,
                   uint16_t rx_length)
{
    config();
    DMESG("SPI Xxfr %p/%d %p/%d", p_tx_buffer, tx_length, p_rx_buffer, rx_length);
    nrf_spim_tx_buffer_set(p_spim, p_tx_buffer, tx_length);
    nrf_spim_rx_buffer_set(p_spim, p_rx_buffer, rx_length);
    nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);
    nrf_spim_tx_list_disable(p_spim);
    nrf_spim_rx_list_disable(p_spim);
    nrf_spim_task_trigger(p_spim, NRF_SPIM_TASK_START);
    //nrf_spim_int_enable(p_spim, NRF_SPIM_INT_END_MASK);

    #if 0
    DMESG("Xrxd: %p", p_spim->RXD.PTR);
    DMESG("X");
    DMESG("started: %d", nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_STARTED));
    DMESG("now wait 12.");

    fiber_sleep(10);

    DMESG("fr: %p", p_spim->FREQUENCY);

    DMESG("end: %d", nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_END));
    DMESG("started: %d", nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_STARTED));
    DMESG("stopped: %d", nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_STOPPED));
    #endif

    while (!nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_END)) {        
        fiber_sleep(1);
    }

    DMESG("done waiting.");

    // fiber_wait_for_event(DEVICE_ID_SPI, 3);
    return 0;
}

int NRF52SPI::transfer(const uint8_t *command, uint32_t commandSize, uint8_t *response,
                       uint32_t responseSize)
{
    if (commandSize)
    {
        int ret = xfer(command, commandSize, NULL, 0);
        if (ret)
            return ret;
    }

    if (responseSize)
    {
        int ret = xfer(NULL, 0, response, responseSize);
        if (ret)
            return ret;
    }

    return DEVICE_OK;
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
    //nrf_spim_int_enable(p_spim, NRF_SPIM_INT_END_MASK | NRF_SPIM_INT_STOPPED_MASK);
    nrf_spim_enable(p_spim);

    //NVIC_SetVector(IRQn, (uint32_t)irq_handler);
    /*
    sd_nvic_SetPriority(IRQn, 7);    
    sd_nvic_ClearPendingIRQ(IRQn);
    sd_nvic_EnableIRQ(IRQn);
    */

    DMESG("SPI config done f=%p", freq);
}

/** Set the frequency of the SPI interface
 *
 * @param frequency The bus frequency in hertz
 */
int NRF52SPI::setFrequency(uint32_t frequency)
{
    if (frequency <= 125000)
        freq = NRF_DRV_SPI_FREQ_125K;
    else if (frequency <= 250000)
        freq = NRF_DRV_SPI_FREQ_250K;
    else if (frequency <= 500000)
        freq = NRF_DRV_SPI_FREQ_500K;
    else if (frequency <= 1000000)
        freq = NRF_DRV_SPI_FREQ_1M;
    else if (frequency <= 2000000)
        freq = NRF_DRV_SPI_FREQ_2M;
    else if (frequency <= 4000000)
        freq = NRF_DRV_SPI_FREQ_4M;
    else if (frequency <= 8000000)
        freq = NRF_DRV_SPI_FREQ_8M;
    else if (frequency <= 16000000)
        freq = (nrf_drv_spi_frequency_t)0x0A000000; // 16M
    else
        freq = (nrf_drv_spi_frequency_t)0x14000000; // 32M
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
    // assert(bits == 8);
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
    int ret = xfer(&sendCh, 1, &recvCh, 1);
    return (ret < 0) ? (int)DEVICE_SPI_ERROR : recvCh;
}
}
