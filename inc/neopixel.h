// WS2812B timings, +-0.15uS
// 0 - 0.40uS hi 0.85uS low
// 1 - 0.80uS hi 0.45uS low

#ifndef CODAL_NEOPIXEL_H
#define CODAL_NEOPIXEL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

__attribute__((noinline)) static void neopixel_send_buffer(Pin &pin, const uint8_t *ptr,
                                                           int numBytes)
{
    pin.setDigitalValue(0);

    auto port = pin.name < 32 ? NRF_P0 : NRF_P1;
    uint32_t PIN = 1 << (pin.name & 31);

    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t startTime = DWT->CYCCNT;
    // min. 50uS reset time; give it 100uS
    while ((DWT->CYCCNT - startTime) < 64 * 100)
        ;

    uint32_t mask = 0x80;
    int i = 0;

    target_disable_irq();
    uint32_t phase = DWT->CYCCNT;
    for (;;)
    {
        port->OUTSET = PIN;

        // phase = CPU_MHZ / 0.8
        // 52/25 are relative to phase
        uint32_t change = ptr[i] & mask ? phase + 52 : phase + 25;
        phase += 80;

        mask = mask >> 1;
        if (mask == 0)
        {
            mask = 0x80;
            i++;
        }

        while (DWT->CYCCNT < change)
            ;

        port->OUTCLR = PIN;

        if (i >= numBytes)
            break;

        while (DWT->CYCCNT < phase)
            ;
    }
    target_enable_irq();
}

#endif // CODAL_NEOPIXEL_H
