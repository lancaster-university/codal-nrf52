#include "NRF52Timer.h"
#include "CodalCompat.h"
#include "Timer.h"

#include "CodalDmesg.h"
#include "nrf.h"
#define MINIMUM_PERIOD  1

static codal::NRF52Timer *instance = NULL;

#ifdef __cplusplus
extern "C" {
#endif

void TIMER1_IRQHandler_v()
{
    bool isFallback = false;

    if (NRF_TIMER1->EVENTS_COMPARE[0] && NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE0_Msk)
    {
        isFallback = true;
        NRF_TIMER1->EVENTS_COMPARE[0] = 0;
    }

    if(NRF_TIMER1->EVENTS_COMPARE[1] && NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE1_Msk)
    {
        // NRF_TIMER1->INTENCLR = (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
        NRF_TIMER1->EVENTS_COMPARE[1] = 0;
    }

    if (instance)
    {
        instance->syncRequest();
        if (isFallback) {
            instance->sigma = 0;
            NRF_TIMER1->TASKS_CLEAR = 1;
        }
        instance->trigger();
    }
}
#ifdef __cplusplus
}
#endif

namespace codal
{
    NRF52Timer::NRF52Timer() : codal::Timer()
    {
        this->period = 10000000; // 10s fallback timer
        instance = this;

        // Timers use the HFCLK,
        NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
        NRF_CLOCK->TASKS_HFCLKSTART = 1;
        while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

        NRF_TIMER1->TASKS_STOP = 1;

        // 1 us precision.
        NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
        NRF_TIMER1->TASKS_CLEAR = 1;
        NRF_TIMER1->PRESCALER = 4;

        // 32 bit comparisons...
        NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit;//0x0003;

        // configure our well defined period for definitive interrupts.
        NRF_TIMER1->CC[0] = this->period;

        // enable compare for channels 0 and 1
        NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
        // configure automatic clear.
        // NRF_TIMER1->SHORTS = 0;

        // NVIC_SetVector(TIMER1_IRQn, (uint32_t) TIMER1_IRQHandler);
        NVIC_SetPriority(TIMER1_IRQn,0);
        NVIC_ClearPendingIRQ(TIMER1_IRQn);
        NVIC_EnableIRQ(TIMER1_IRQn);

        NRF_TIMER1->TASKS_START = 1;
    }

    /**
     * request to the physical timer implementation code to provide a trigger callback at the given time.
     * note: it is perfectly legitimate for the implementation to trigger before this time if convenient.
     * @param t Indication that t time units (typically microsends) have elapsed.
     */
    void NRF52Timer::triggerIn(CODAL_TIMESTAMP t)
    {
        if (t < MINIMUM_PERIOD)
            t = MINIMUM_PERIOD;

        NVIC_DisableIRQ(TIMER1_IRQn);
        NRF_TIMER1->CC[1] = t;
        NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
        sigma = 0;
        NRF_TIMER1->TASKS_CLEAR = 1;
        NVIC_EnableIRQ(TIMER1_IRQn);
    }

    /**
     * request to the physical timer implementation code to trigger immediately.
     */
    void NRF52Timer::syncRequest()
    {
        __disable_irq();
        NRF_TIMER1->TASKS_CAPTURE[2] = 1;
        uint32_t snapshot = NRF_TIMER1->CC[2];
        uint32_t elapsed = snapshot - sigma;
        sigma = snapshot;
        this->sync(elapsed);
        __enable_irq();
    }
}
