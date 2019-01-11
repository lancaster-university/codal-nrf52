#include "NRFLowLevelTimer.h"

#define PRESCALE_VALUE_MAX  9

using namespace codal;

static inline uint32_t counter_value(NRF_TIMER_Type* t, uint8_t cc)
{
    t->TASKS_CAPTURE[cc] = 1;
    return t->CC[cc];
}

static NRFLowLevelTimer *instances[6] = { 0 };

void timer_handler(uint8_t instance_number)
{
    if (instances[instance_number] == NULL)
        return;

    uint8_t channel_bitmsk = 0;

    for(int i = 0; i < TIMER_CHANNEL_COUNT; i++)
    {
        if(instances[1]->timer->EVENTS_COMPARE[i])
        {
            channel_bitmsk |= 1 << i;
            instances[1]->timer->EVENTS_COMPARE[i] = 0;
        }
    }

    instances[1]->timer_pointer(channel_bitmsk);
}

#ifdef NRF52
extern "C" void TIMER0_IRQHandler_v()
{
    timer_handler(0);
}

extern "C" void TIMER1_IRQHandler_v()
{
    timer_handler(1);
}

extern "C" void TIMER2_IRQHandler_v()
{
    timer_handler(2);
}

extern "C" void TIMER3_IRQHandler_v()
{
    timer_handler(3);
}

extern "C" void TIMER4_IRQHandler_v()
{
    timer_handler(4);
}

#elif NRF51

#endif

// 1 channel is used to capture the timer value (channel 3 indexed from zero)
NRFLowLevelTimer::NRFLowLevelTimer(NRF_TIMER_Type* t, IRQn_Type irqn) : LowLevelTimer(3)
{
    this->timer = t;
    this->irqn = irqn;

    uint8_t instanceNumber = 0;

    if (t == NRF_TIMER1)
        instanceNumber = 1;
    if (t == NRF_TIMER2)
        instanceNumber = 2;
    if (t == NRF_TIMER3)
        instanceNumber = 3;
    if (t == NRF_TIMER4)
        instanceNumber = 4;

    instances[instanceNumber] = this;

    setBitMode(BitMode32);
}

int NRFLowLevelTimer::enable()
{
    NVIC_SetPriority(irqn, 1);
    NVIC_ClearPendingIRQ(irqn);


    timer->TASKS_START = 1;

    return DEVICE_OK;
}

int NRFLowLevelTimer::enableIRQ()
{
    NVIC_EnableIRQ(irqn);
    return DEVICE_OK;
}

int NRFLowLevelTimer::disable()
{
    disableIRQ();
    timer->TASKS_STOP = 1;
    return DEVICE_OK;
}

int NRFLowLevelTimer::disableIRQ()
{
    NVIC_DisableIRQ(irqn);
    return DEVICE_OK;
}

int NRFLowLevelTimer::reset()
{
    timer->TASKS_CLEAR = 1;
    while (timer->TASKS_CLEAR);
    return DEVICE_OK;
}

int NRFLowLevelTimer::setMode(TimerMode t)
{
    switch (t)
    {
        case TimerModeTimer:
            timer->MODE = 0;
            break;
        case TimerModeCounter:
            timer->MODE = 1;
            break;
        case TimerModeAlternateFunction:
            timer->MODE = 2;
            break;
    }

    return DEVICE_OK;
}

int NRFLowLevelTimer::setCompare(uint8_t channel, uint32_t value)
{
    if (channel > getChannelCount() - 1)
        return DEVICE_INVALID_PARAMETER;

    timer->CC[channel] = value;
    timer->INTENSET |= (1 << channel) << TIMER_INTENSET_COMPARE0_Pos;

    return DEVICE_OK;
}

int NRFLowLevelTimer::offsetCompare(uint8_t channel, uint32_t value)
{
    if (channel > getChannelCount() - 1)
        return DEVICE_INVALID_PARAMETER;

    timer->CC[channel] += value;
    timer->INTENSET |= (1 << channel) << TIMER_INTENSET_COMPARE0_Pos;

    return DEVICE_OK;
}

int NRFLowLevelTimer::clearCompare(uint8_t channel)
{
    if (channel > getChannelCount() - 1)
        return DEVICE_INVALID_PARAMETER;

    timer->INTENCLR |= (1 << channel) << TIMER_INTENCLR_COMPARE0_Pos;
    return DEVICE_OK;
}

uint32_t NRFLowLevelTimer::captureCounter()
{
    // 1 channel is used to capture the timer value (channel 3 indexed from zero)
    return counter_value(timer, 3);
}

int NRFLowLevelTimer::setClockSpeed(uint32_t speedKHz)
{
    // max speed is 16000Khz
    if (speedKHz > 16000)
        return DEVICE_INVALID_PARAMETER;

    uint32_t clockSpeed = 16000;
    uint8_t prescaleValue = 0;

    // snap to the lowest
    for (prescaleValue = 0; prescaleValue < PRESCALE_VALUE_MAX; prescaleValue++)
    {
        if (speedKHz < (clockSpeed / prescaleValue))
            continue;

        break;
    }

    timer->PRESCALER = prescaleValue;

    return DEVICE_OK;
}

int NRFLowLevelTimer::setBitMode(TimerBitMode t)
{
    switch (t)
    {
        case BitMode8:
            timer->BITMODE = 1;
            break;
        case BitMode16:
            timer->BITMODE = 0;
            break;
        case BitMode24:
            timer->BITMODE = 2;
            break;
        case BitMode32:
            timer->BITMODE = 2;
            break;
    }

    this->bitMode = t;
    return DEVICE_OK;
}