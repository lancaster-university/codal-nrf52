#include "NRF52SPI.h"
#include "hal/nrf_i2s.h"
#include "codal-core/inc/types/Event.h"
#include "ErrorNo.h"
#include "CodalFiber.h"

namespace codal
{

// base on https://electronut.in/nrf52-i2s-ws2812/

static int numIrq;

extern "C" void I2S_IRQHandler()
{
    if (NRF_I2S->EVENTS_TXPTRUPD != 0)
    {
        NRF_I2S->EVENTS_TXPTRUPD = 0;
        volatile uint32_t dummy = NRF_I2S->EVENTS_TXPTRUPD;
        if (numIrq == 0)
        {
            numIrq++;
            NRF_I2S->RXTXD.MAXCNT = 1;
        }
        else
        {
            NRF_I2S->ENABLE = 0;
            Event(DEVICE_ID_SPI, 2000);
        }
    }
}

void neopixel_send_buffer(Pin *pin, const uint8_t *data, unsigned size)
{
    if (NRF_I2S->ENABLE)
        return;

    int32_t iptr = 0, optr = 6;
    uint32_t len = optr + size + optr;
    uint32_t *expBuf = new uint32_t[len];
    memset(expBuf, 0, len * 4);

    while (iptr < (int)size)
    {
        uint32_t outp = 0x88888888;
        uint8_t inp = data[iptr];
        for (int i = 0; i < 8; ++i)
        {
            if (inp & (1 << i))
                outp |= 0xe << (i * 4);
        }
        outp = (outp >> 16) | (outp << 16);
        expBuf[optr++] = outp;
        iptr++;
    }

    NRF_I2S->CONFIG.RXEN = 0;
    NRF_I2S->CONFIG.TXEN = 1;
    NRF_I2S->CONFIG.MCKEN = 1;
    NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV10; // 3.2MHz
    NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_32X;
    NRF_I2S->CONFIG.SWIDTH = I2S_CONFIG_SWIDTH_SWIDTH_16Bit;
    NRF_I2S->CONFIG.ALIGN = I2S_CONFIG_ALIGN_ALIGN_Left;
    NRF_I2S->CONFIG.FORMAT = I2S_CONFIG_FORMAT_FORMAT_I2S;
    NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_Stereo;

#if 0
    NRF_I2S->PSEL.MCK = -1;
    NRF_I2S->PSEL.SCK = -1;
    NRF_I2S->PSEL.LRCK = -1;
    NRF_I2S->PSEL.SDOUT = (uint32_t)pin->name;
    NRF_I2S->PSEL.SDIN = -1;
#endif

    NRF_I2S->PSEL.MCK = -1;  // D9
    NRF_I2S->PSEL.SCK = 27;  // D10
    NRF_I2S->PSEL.LRCK = 6;  // D11
    NRF_I2S->PSEL.SDOUT = 8; // D12
    NRF_I2S->PSEL.SDIN = -1; // disconnect


    NRF_I2S->TXD.PTR = (uint32_t)expBuf;
    NRF_I2S->RXTXD.MAXCNT = len;

    NRF_I2S->INTEN = I2S_INTEN_TXPTRUPD_Msk;

    numIrq = 0;

    NVIC_SetPriority(I2S_IRQn, 1);
    NVIC_ClearPendingIRQ(I2S_IRQn);
    NVIC_EnableIRQ(I2S_IRQn);

    NRF_I2S->ENABLE = 1;
    NRF_I2S->TASKS_START = 1;

    fiber_wait_for_event(DEVICE_ID_SPI, 2000);

    delete expBuf;
}

} // namespace codal