#ifndef NRF_TIMER_H
#define NRF_TIMER_H

#include "LowLevelTimer.h"
#include "nrf.h"

#define TIMER_CHANNEL_COUNT              4
namespace codal
{
    class NRFLowLevelTimer : public LowLevelTimer
    {
        IRQn_Type irqn;

        public:
        NRF_TIMER_Type *timer;

        NRFLowLevelTimer(NRF_TIMER_Type* timer, IRQn_Type irqn);

        virtual int setIRQPriority(int priority) override;

        virtual int enable() override;

        virtual int enableIRQ() override;

        virtual int disable() override;

        virtual int disableIRQ() override;

        virtual int reset() override;

        virtual int setMode(TimerMode t) override;

        virtual int setCompare(uint8_t channel, uint32_t value) override;

        virtual int offsetCompare(uint8_t channel, uint32_t value) override;

        virtual int clearCompare(uint8_t channel) override;

        virtual uint32_t captureCounter() override;

        virtual int setClockSpeed(uint32_t speedKHz) override;

        virtual int setBitMode(TimerBitMode t) override;

        virtual int setSleep(bool doSleep) override;
    };
}

#endif