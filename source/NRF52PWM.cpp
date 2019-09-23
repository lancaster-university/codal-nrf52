#include "NRF52PWM.h"
#include "nrf.h"
#include "InterruptManager.h"
#include "cmsis.h"
#include "CodalDmesg.h"

// Handles on the instances of this class used the three PWM modules (if present)
static NRF52PWM *nrf52_pwm_driver[3] = { NULL };

void nrf52_pwm0_irq(void)
{
    // Simply pass on to the driver component handler.
    if (nrf52_pwm_driver[0])
        nrf52_pwm_driver[0]->irq();
}

void nrf52_pwm1_irq(void)
{
    // Simply pass on to the driver component handler.
    if (nrf52_pwm_driver[1])
        nrf52_pwm_driver[1]->irq();
}

void nrf52_pwm2_irq(void)
{
    // Simply pass on to the driver component handler.
    if (nrf52_pwm_driver[2])
        nrf52_pwm_driver[2]->irq();
}

NRF52PWM::NRF52PWM(NRF_PWM_Type *module, DataSource &source, int sampleRate, uint16_t id) : PWM(*module), upstream(source)
{
    // initialise state
    this->id = id;
    this->dataReady = 0;
    this->active = false;

    // Ensure PWM is currently disabled.
    disable();

    // Configure hardware for requested sample rate.
    setSampleRate(sampleRate);

    // Configure for a repeating, edge aligned PWM pattern.
    PWM.MODE = PWM_MODE_UPDOWN_Up;

    // Configure for a single pass over the data
    PWM.LOOP = 0;

    // For now, we just support a two channel grouped mode.
    // TODO: Fix this to support all modes.
    PWM.DECODER = (PWM_DECODER_LOAD_Grouped << PWM_DECODER_LOAD_Pos ) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos );

    // Configure PWM for
    PWM.SEQ[1].REFRESH = 0;
    PWM.SEQ[0].REFRESH = 0;
    PWM.SEQ[1].ENDDELAY = 0;
    PWM.SEQ[0].ENDDELAY = 0;

    /* Enable interrupts */
    PWM.INTENSET = (PWM_INTEN_SEQEND0_Enabled << PWM_INTEN_SEQEND0_Pos )
                |  (PWM_INTEN_SEQEND1_Enabled << PWM_INTEN_SEQEND1_Pos );

    // Route an interrupt to this object
    // This is heavily unwound, but non trivial to remove this duplication given all the constants...
    // TODO: build up some lookup table to deduplicate this.
    if (&PWM == NRF_PWM0)
    {
        nrf52_pwm_driver[0] = this;
        NVIC_SetVector( PWM0_IRQn, (uint32_t) nrf52_pwm0_irq );
        NVIC_SetPriority(PWM0_IRQn, 0);
        NVIC_ClearPendingIRQ(PWM0_IRQn);
        NVIC_EnableIRQ(PWM0_IRQn);
    }

    if (&PWM == NRF_PWM1)
    {
        nrf52_pwm_driver[1] = this;
        NVIC_SetVector( PWM1_IRQn, (uint32_t) nrf52_pwm1_irq );
        NVIC_SetPriority(PWM1_IRQn, 0);
        NVIC_ClearPendingIRQ(PWM1_IRQn);
        NVIC_EnableIRQ(PWM1_IRQn);
    }

    if (&PWM == NRF_PWM2)
    {
        nrf52_pwm_driver[2] = this;
        NVIC_SetVector( PWM2_IRQn, (uint32_t) nrf52_pwm2_irq );
        NVIC_SetPriority(PWM2_IRQn, 0);
        NVIC_ClearPendingIRQ(PWM2_IRQn);
        NVIC_EnableIRQ(PWM2_IRQn);
    }

    // Enable the PWM module
    enable();

    // Register with our upstream component
    upstream.connect(*this);
}

/**
 * Determine the DAC playback sample rate to the given frequency.
 * @return the current sample rate.
 */
int NRF52PWM::getSampleRate()
{
    return sampleRate;
}

/**
 * Determine the maximum unsigned vlaue that can be loaded into the PWM data values, for the current
 * frequency configuration.
 */
int NRF52PWM::getSampleRange()
{
    return PWM.COUNTERTOP;
}

/**
 * Change the DAC playback sample rate to the given frequency.
 * @param frequency The new sample playback frequency.
 */
int NRF52PWM::setSampleRate(int frequency)
{
    int period_ticks;
    int prescaler = 0;
    int clock_frequency = 16000000;

    // Calculate the necessary prescaler for this frequency
    while (prescaler <= 8)
    {
        period_ticks = (clock_frequency >> prescaler) / frequency;

        if (period_ticks > 0 && period_ticks < 32767)
            break;

        prescaler++;
    }

    // If the sample rate requested is outside the range of what the hardware can achieve, then there's nothign we can do.
    if (prescaler > 7)
    {
        DMESG("NRF52PWM: Invalid samplerate");
        return DEVICE_INVALID_PARAMETER;
    }

    // Decrement prescaler, as hardware indexes from zero.
    PWM.PRESCALER = prescaler;
    PWM.COUNTERTOP = period_ticks;

    // Update our internal record to reflect an accurate (probably rounded) samplerate.
    sampleRate =  (clock_frequency / (prescaler+1)) / period_ticks;

    return DEVICE_OK;
}

/**
 * Callback provided when data is ready.
 */
int NRF52PWM::pullRequest()
{
    dataReady++;

    if (!active)
        pull();

    return DEVICE_OK;
}

/**
 * Pull down a buffer from upstream, and schedule a DMA transfer from it.
 */
int NRF52PWM::pull()
{
    output = upstream.pull();
    dataReady--;

    PWM.SEQ[0].PTR = (uint32_t) &output[0];
    PWM.SEQ[0].CNT = output.length() / 2;
    PWM.TASKS_SEQSTART[0] = 1;

    active = true;

    return DEVICE_OK;
}

/**
 * Base implementation of a DMA callback
 */
void NRF52PWM::irq()
{
    // once the sequence has finished playing, load up the next buffer.
    if (PWM.EVENTS_SEQEND[0])
    {
        if (dataReady)
            pull();
        else
            active = false;

        PWM.EVENTS_SEQEND[0] = 0;
    }

    //Spurious read to cover the "events don't clear immediately" erratum
    (void) PWM.EVENTS_SEQEND[1];
}

/**
 * Enable this component
 */
void NRF52PWM::enable()
{
    enabled = true;
    PWM.ENABLE = 1;
}

/**
 * Disable this component
 */
void NRF52PWM::disable()
{
    enabled = false;
    PWM.ENABLE = 0;
}

/**
 * Direct output of given PWM channel to the given pin
 */
int
NRF52PWM::connectPin(Pin &pin, int channel)
{
    pin.setDigitalValue(0);
    PWM.PSEL.OUT[channel] = pin.name;

    // HACK: Enter high drive mode. This should move to specific Pin class...
    // A setMode(enum) to turn on off arbitrary stuff would be cool?
    uint32_t s = NRF_GPIO->PIN_CNF[pin.name] & 0xfffff8ff;
    s |= 0x00000300;
    NRF_GPIO->PIN_CNF[pin.name] = s;

    return DEVICE_OK;
}
