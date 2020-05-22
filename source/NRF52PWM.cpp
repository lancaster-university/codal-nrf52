#include "NRF52PWM.h"
#include "nrf.h"
#include "cmsis.h"
#include "CodalDmesg.h"

// 10 bit
#define SAMPLE_RESOLUTION 10
#define SAMPLE_MAX_VAL ((1 << SAMPLE_RESOLUTION) - 1)

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

    // By default, enable control of four independent channels
    setDecoderMode(PWM_DECODER_LOAD_Individual);

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
    return SAMPLE_MAX_VAL;
}

/**
 * Change the DAC playback sample rate to the given frequency.
 * @param frequency The new sample playback frequency.
 */
int NRF52PWM::setSampleRate(int frequency)
{
    return setPeriodUs(1000000 / frequency);
}

/**
 * Change the DAC playback sample rate to the given period.
 * @param period The new sample playback period, in microseconds.
 */
int NRF52PWM::setPeriodUs(int period)
{
    int prescaler = 0;
    int clock_frequency = 16000000;
    int period_ticks = (clock_frequency/1000000) * period;

    // Calculate necessary prescaler.
    while(period_ticks >> prescaler >= 32768)
        prescaler++;

    // If the sample rate requested is outside the range of what the hardware can achieve, then there's nothign we can do.
    if (prescaler > 7)
    {
        DMESG("NRF52PWM: Invalid samplerate");
        return DEVICE_INVALID_PARAMETER;
    }

    // Decrement prescaler, as hardware indexes from zero.
    PWM.PRESCALER = prescaler;
    PWM.COUNTERTOP = period_ticks >> prescaler;

    // Update our internal record to reflect an accurate (probably rounded) samplerate.
    period_ticks = period_ticks >> prescaler;
    period_ticks = period_ticks << prescaler;

    periodUs = period_ticks / (clock_frequency/1000000);
    sampleRate = 1000000 / periodUs;

    return DEVICE_OK;
}

/**
 * Determine the current DAC playback period.
 * @return period The sample playback period, in microseconds.
 */
int NRF52PWM::getPeriodUs()
{
    return periodUs;
}


/** 
 * Defines the mode in which the PWM module will operate, in terms of how it interprets data provided from the DataSource:
 * Valid options are:
 * 
 * PWM_DECODER_LOAD_Common          1st half word (16-bit) used in all PWM channels 0..3 
 * PWM_DECODER_LOAD_Grouped         1st half word (16-bit) used in channel 0..1; 2nd word in channel 2..3
 * PWM_DECODER_LOAD_Individual      1st half word (16-bit) in ch.0; 2nd in ch.1; ...; 4th in ch.3 
 * PWM_DECODER_LOAD_WaveForm        1st half word (16-bit) in ch.0; 2nd in ch.1; ...; 4th in COUNTERTOP
 * 
 * (See nrf52 product specificaiton for more details)
 * 
 * @param mode The mode for this PWM module to use.
 * @return DEVICE_OK, or DEVICE_INVALID_PARAMETER.
 */
int NRF52PWM::setDecoderMode(uint32_t mode)
{
    PWM.DECODER = (mode << PWM_DECODER_LOAD_Pos ) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos );

    return DEVICE_OK;
}
 
/**
 * Defines if the PWM modules should repeat the previous sample when the end of the stream is reached.
 * 
 * @ param repeat if true, the last PWM sample received is repeated if the stream underflows. If false, the PWM module will stop processing on stream underflow.
 */
void setLoop(bool repeat)
{
    //TODO
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

void NRF52PWM::prefill()
{
    if (!dataReady)
        return;

    dataReady--;
    if (!active) {
        active = true;
        nextBuffer = upstream.pull();
        active = false;
    } else {
        nextBuffer = upstream.pull();
    }
}

/**
 * Pull down a buffer from upstream, and schedule a DMA transfer from it.
 */
int NRF52PWM::pull()
{
    if (!nextBuffer.length())
        prefill();

    buffer = nextBuffer;
    nextBuffer = ManagedBuffer();

    uint32_t len = buffer.length() >> 1;
    if (len) {
        uint16_t *data = (uint16_t*) &buffer[0];
        uint32_t mul = PWM.COUNTERTOP;
        // assume unsigned 10 bit input data
        for (uint32_t i = 0; i < len; ++i) {
            data[i] = (SAMPLE_MAX_VAL - data[i]) * mul >> SAMPLE_RESOLUTION;
        }
        PWM.SEQ[0].PTR = (uint32_t) data;
        PWM.SEQ[0].CNT = len;
        active = true;
        PWM.TASKS_SEQSTART[0] = 1;
    } else {
        dataReady = 0;
        active = false;
        return DEVICE_OK;
    }

    prefill(); // pre-fetch next buffer

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
        // Spurious read to cover the "events don't clear immediately" erratum
        (void) PWM.EVENTS_SEQEND[0];
    }

    if (PWM.EVENTS_SEQEND[1])
    {
        PWM.EVENTS_SEQEND[1] = 0;
        (void) PWM.EVENTS_SEQEND[1];
    }
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

    return DEVICE_OK;
}
