#include "NRF52PWM.h"
#include "nrf.h"
#include "cmsis.h"
#include "CodalDmesg.h"

using namespace codal;

#define NRF52PWM_EMPTY_BUFFERSIZE 40
#define PWM_ZERO_DC 0x8000  // low output
static uint16_t emptyBuffer[NRF52PWM_EMPTY_BUFFERSIZE];

void nrf52_pwm0_irq(void)
{
    // Simply pass on to the driver component handler.
    if (NRF52PWM::nrf52_pwm_driver[0])
        NRF52PWM::nrf52_pwm_driver[0]->irq();
}

void nrf52_pwm1_irq(void)
{
    // Simply pass on to the driver component handler.
    if (NRF52PWM::nrf52_pwm_driver[1])
        NRF52PWM::nrf52_pwm_driver[1]->irq();
}

void nrf52_pwm2_irq(void)
{
    // Simply pass on to the driver component handler.
    if (NRF52PWM::nrf52_pwm_driver[2])
        NRF52PWM::nrf52_pwm_driver[2]->irq();
}

// Handles on the instances of this class used the three PWM modules (if present)
NRF52PWM* NRF52PWM::nrf52_pwm_driver[NRF52PWM_PWM_PERIPHERALS] = { NULL };

NRF52PWM::NRF52PWM(NRF_PWM_Type *module, DataSource &source, float sampleRate, uint16_t id) : PWM(*module), upstream(source)
{
    // initialise state
    this->id = id;
    this->dataReady = 0;
    this->active = false;
    this->streaming = true;
    this->repeatOnEmpty = true;
    this->bufferPlaying = 0;
    this->stopStreamingAfterBuf = 0;
    this->bufferCount = 0;
    this->irqTotalCount = 0;
    this->irqBeginTotalCount = 0;
    this->irqStopTotalCount = 0;
    this->errorFlag = 0;
    this->errorFlagCumlative = 0;
    this->state = PwmState::Inactive;

    // Clear empty buffer
    for (int i=0; i<NRF52PWM_EMPTY_BUFFERSIZE; i++)
        emptyBuffer[i] = PWM_ZERO_DC;

    // Ensure PWM is currently disabled.
    disable();

    // Configure hardware for requested sample rate.
    setSampleRate(sampleRate);

    // Configure for a repeating, edge aligned PWM pattern.
    PWM.MODE = PWM_MODE_UPDOWN_Up;

    // By default, enable control of four independent channels
    setDecoderMode(PWM_DECODER_LOAD_Individual);

    // Configure PWM for
    PWM.SEQ[1].REFRESH = 0;
    PWM.SEQ[0].REFRESH = 0;
    PWM.SEQ[1].ENDDELAY = 0;
    PWM.SEQ[0].ENDDELAY = 0;

    // Default to streaming mode.
    setStreamingMode(true);

    // Route an interrupt to this object
    // This is heavily unwound, but non trivial to remove this duplication given all the constants...
    // TODO: build up some lookup table to deduplicate this.
    if (&PWM == NRF_PWM0)
    {
        nrf52_pwm_driver[0] = this;
        NVIC_SetVector( PWM0_IRQn, (uint32_t) nrf52_pwm0_irq );
        NVIC_ClearPendingIRQ(PWM0_IRQn);
        NVIC_EnableIRQ(PWM0_IRQn);
    }

    if (&PWM == NRF_PWM1)
    {
        nrf52_pwm_driver[1] = this;
        NVIC_SetVector( PWM1_IRQn, (uint32_t) nrf52_pwm1_irq );
        NVIC_ClearPendingIRQ(PWM1_IRQn);
        NVIC_EnableIRQ(PWM1_IRQn);
    }

    if (&PWM == NRF_PWM2)
    {
        nrf52_pwm_driver[2] = this;
        NVIC_SetVector( PWM2_IRQn, (uint32_t) nrf52_pwm2_irq );
        NVIC_ClearPendingIRQ(PWM2_IRQn);
        NVIC_EnableIRQ(PWM2_IRQn);
    }

    // Enable the PWM module
    enable();

    // Register with our upstream component
    upstream.connect(*this);
}

#if NRF52PWM_STATS > 0
/**
  * Clear the statistics, typically used before starting to output PWM
  * This is flawed due to not zeroing historical data
  */
void NRF52PWM::zeroStats() {
    // copy values to make history
    for (int i = NRF52PWM_STATS - 1; i >= 0; i--) {
       memcpy(&stats[i], &stats[i - 1], sizeof(stats[0]));
    }

    // zero out current values
    stats[0].irqcount = stats[0].irq0 = stats[0].irq1 = stats[0].irqboth = 0;
    for (int i = 0; i < IRQSTATLEN; i++) {
        stats[0].irq_times[i] = 123456U;
        stats[0].pwmstart_time[i] = 123456U;
        stats[0].irq_seqend[i] = -1;
        stats[0].trypull_dur[i] = 0;
    }
    stats[0].irqinactive = 0;
    stats[0].irqprestart = 0;
    stats[0].irqpoststop = 0;
    stats[0].trypullcnt = 0;
    stats[0].pwmstarts = stats[0].pwmstops = 0;
    stats[0].preloadfails = 0;
    stats[0].dataReadyAtStop = 123456;
    stats[0].nodata = 0;
    stats[0].irqtotalcountatstart = stats[0].irqtotalcountatstop = 0;
    stats[0].zero_time = DWT->CYCCNT;
}


/**
  * Check the statistics for anomalies and if any are found
  * DMESG print whole structure.
  */
void NRF52PWM::anomalyCheckStats(int bufcnt) {
    struct Nrf52PwmStats *s = &stats[0];
    int issues = 0;

    if (s->irqcount != (uint32_t)bufcnt)
        issues++;
    if (s->irqtotalcountatstop != s->irqtotalcountatstart + s->irqcount)
        issues++;
#if NRF52PWM_STATS > 1
    // Look for interrupts occuring while PWM should be not running
    if (s->irqtotalcountatstart != 0 && stats[1].irqtotalcountatstop != s->irqtotalcountatstart)
        issues++;
#endif
    if (s->irq0 + s->irq1 > s->irqcount || s->irqboth > 0)
        issues++;
    if (s->irqinactive > 0 || s->irqprestart > 0 || s->irqpoststop != 1)
        issues++; 
    if (s->pwmstarts != 1 || s->pwmstops != 1)
        issues++;
    if (s->preloadfails > 0 || s->dataReadyAtStop > 0 || s->nodata != 1)
        issues++;

    // TODO - clock speed (MHz) and buffer size for maths
    uint32_t one_buffer_cyc = (uint32_t)(64 * 128 * periodUs);
    int32_t irq_intervals[IRQSTATLEN] = {0};
    if (s->pwmstarts > 0) {
        // The wrapping time values require care with maths and comparisons
        uint32_t z_to_ps = s->pwmstart_time[0] - s->zero_time;
        if ((int32_t)z_to_ps <= 0)
            issues++;
        if (s->irqcount > 0) {
            uint32_t prev_event_cyc = s->pwmstart_time[0];
            uint32_t exp_event_cyc = prev_event_cyc + one_buffer_cyc;
            for (size_t i = 0; i < s->irqcount; i++) {
                uint32_t time_to_irq = s->irq_times[i] - prev_event_cyc;
                irq_intervals[i] = (int32_t)time_to_irq;
                if (irq_intervals[i] <= 0)
                    issues++;
                if (irq_intervals[i] >= (int)one_buffer_cyc + 560 + 1000)
                    issues++;

                // TODO ponder re-enabling this
                // The first interrupts is often 9500 ish after start
                // or 7900 ish which seems impossible as min time should
                // be 128 * 1.25 * 64 = 10240 cycles
                // very very odd perhaps CYCCNT + somethnigelse is needed
                // if (s->irq_times[i] <= exp_event_cyc - 123U) {
                //     issues++;
                // } else if (s->irq_times[i] >= exp_event_cyc + 2345U) {
                //     issues++;
                // }
                prev_event_cyc = s->irq_times[i];
                exp_event_cyc += one_buffer_cyc;
            }
        }
    }
    uint32_t risky_cyc = (uint32_t)(one_buffer_cyc * 0.90f);
    for (int i = 0; i < IRQSTATLEN && s->trypull_dur[i] != 0; i++) {
        
        if (s->trypull_dur[i] >= risky_cyc) {
            issues++;
            break;
        }
    }

    int testtrigger = 0;
    // if (DWT->CYCCNT % 1000 < 10)
    //    testtrigger++;

    if (issues > 0 || testtrigger) {
        DMESG("NRF52PWM anomalyCheckStart(%d) issues=%d", bufcnt, issues);
        for (int si = NRF52PWM_STATS - 1; si >= 0; si--) {
            struct Nrf52PwmStats *s = &stats[si];
            DMESG("stats[%d]", si);
            DMESG("irqcount=%u irq0=%u irq1=%u irqboth=%u",
                  s->irqcount, s->irq0, s->irq1, s->irqboth);
            DMESG("irqinactive=%d irqprestart=%d irqpoststop=%d pwmstarts=%d pwmstops=%d",
                  s->irqinactive, s->irqprestart, s->irqpoststop, s->pwmstarts, s->pwmstops);
            DMESG("trypullcnt=%u preloadfails=%d dataReadyAtStop=%d nodata=%d",
                  s->trypullcnt, s->preloadfails, s->dataReadyAtStop, s->nodata);
            DMESG("TS %u zero", s->zero_time);
            for (int i = 0; i < IRQSTATLEN && s->pwmstart_time[i] != 123456U; i++) {
                DMESG("TS %u pwmstart fromzero %d",
                      s->pwmstart_time[i], s->pwmstart_time[i] - s->zero_time);
            }
            for (int i = 0; i < IRQSTATLEN && s->irq_times[i] != 123456U; i++) {
                if (si == 0) {
                    DMESG("TS %u interval %d irq %d",
                          s->irq_times[i], irq_intervals[i], s->irq_seqend[i]);
                } else {
                    DMESG("TS %u irq %d",
                          s->irq_times[i], s->irq_seqend[i]);
                }
            }
            for (int i = 0; i < IRQSTATLEN && s->trypull_dur[i] != 0; i++) {
                DMESG("TPDUR %u", s->trypull_dur[i]);
            }
            DMESGF("");
        }
    }
}
#endif

/**
 * Determine the DAC playback sample rate to the given frequency.
 * @return the current sample rate.
 */
float NRF52PWM::getSampleRate()
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
float NRF52PWM::setSampleRate(float frequency)
{
    return setPeriodUs(1000000 / frequency);
}

/**
 * Change the DAC playback sample rate to the given period.
 * @param period The new sample playback period, in microseconds.
 */
int NRF52PWM::setPeriodUs(float period)
{
    int prescaler = 0;
    int clock_frequency = 16000000;
    int period_ticks = (clock_frequency/1000000) * period;

    // Calculate necessary prescaler.
    while(period_ticks >> prescaler >= 32768)
        prescaler++;

    // If the sample rate requested is outside the range of what the hardware can achieve, then there's nothign we can do.
    if (prescaler > 7)
        return DEVICE_INVALID_PARAMETER;

    // Decrement prescaler, as hardware indexes from zero.
    PWM.PRESCALER = prescaler;
    PWM.COUNTERTOP = period_ticks >> prescaler;

    // Update our internal record to reflect an accurate (probably rounded) samplerate.
    period_ticks = period_ticks >> prescaler;
    period_ticks = period_ticks << prescaler;

    periodUs = (float)period_ticks / (clock_frequency/1000000);
    sampleRate = 1000000 / periodUs;

    return DEVICE_OK;
}

/**
 * Determine the current DAC playback period.
 * @return period The sample playback period, in microseconds.
 */
float NRF52PWM::getPeriodUs()
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
 * Defines if the PWM module should maintain playout ordering of buffers, or always play the most recent buffer provided.
 * 
 * @ param streamingMode If true, buffers will be streamed in order they are received. If false, the most recent buffer supplied always takes precedence.
 * @ param repeatOnEmpty If set to true, the last buffer received will be repeated when no data is available. If false, the PWM channel will be suspended.
 */
void NRF52PWM::setStreamingMode(bool streamingMode, bool repeatOnEmpty)
{
    this->streaming = streamingMode;
    this->repeatOnEmpty = repeatOnEmpty;
    setPwmLoopInten(streamingMode);
}

/**
 * Sets PWM hardware registers for chained buffers or simple one shot playback
 * 
 * @ param streamingMode If true, buffers will be streamed in order they are received. If false, the most recent buffer supplied always takes precedence.
 */
void NRF52PWM::setPwmLoopInten(bool streamingMode) {
    if (streamingMode)
    {
        // Clearing events here to prevent any immediate spurious interrupts
        PWM.EVENTS_SEQEND[0] = PWM.EVENTS_SEQEND[1] = 0;
        PWM.LOOP = 1;
        PWM.SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART0_Enabled << PWM_SHORTS_LOOPSDONE_SEQSTART0_Pos; 
        PWM.INTENSET = (PWM_INTEN_SEQEND0_Enabled << PWM_INTEN_SEQEND0_Pos ) | (PWM_INTEN_SEQEND1_Enabled << PWM_INTEN_SEQEND1_Pos);
    }
    else
    {
        PWM.LOOP = 0;
        PWM.SHORTS = 0; 
        PWM.INTENCLR = (PWM_INTEN_SEQEND0_Enabled << PWM_INTEN_SEQEND0_Pos ) | (PWM_INTEN_SEQEND1_Enabled << PWM_INTEN_SEQEND1_Pos);
    }
}


/**
 * Pull a buffer into the given double buffer slot, if one is available.
 * @param b The buffer to fill (either 0 or 1)
 * @return the number of buffers succesfully filled.
 */
int NRF52PWM::tryPull(uint8_t b)
{
#if NRF52PWM_STATS > 0
    uint32_t t1_cyc = DWT->CYCCNT;
#endif
    int filled = 0;

    if (stopStreamingAfterBuf)
    {
        state = PwmState::Stopping;
        // SHORTS must be disabled before STOP as "PWM could be immediately
        // started again if the LOOPSDONE event occurred in the same peripheral
        // clock cycle as the STOP task was triggered"
        setPwmLoopInten(false);  // includes SHORTS off
        PWM.TASKS_STOP = 1;
        while(PWM.EVENTS_STOPPED == 0);
        PWM.EVENTS_SEQEND[0] = PWM.EVENTS_SEQEND[1] = 0;
        state = PwmState::Stopped;
        irqStopTotalCount = irqTotalCount;
        uint32_t irqs = irqStopTotalCount - irqBeginTotalCount;
        if (irqs != bufferCount)
            errorFlag |= NRF52PWM_ERROR_MISMATCHIRQ;

#if NRF52PWM_STATS > 0
        stats[0].irqtotalcountatstop = irqTotalCount;
#endif

        active = false;
        bufferPlaying = 0;
        stopStreamingAfterBuf = 0;
        errorFlagCumlative |= errorFlag;

#if NRF52PWM_STATS > 0
        stats[0].pwmstops++;
#endif

        upstream.dataWanted(DATASTREAM_NOT_WANTED);  // all the data has been PWMed

        // If a Pull request has been made since we decided to stop, start to fill up the
        // hardware double buffer so that we don't stall.
#if NRF52PWM_STATS > 0
        stats[0].dataReadyAtStop = dataReady;
#endif
        if (dataReady > 0)
        {
            dataReady--;
            pullRequest();
        }
        goto tryPullReturn;
    }

    if (dataReady > 0) { 
        upstream.pull(buffer[b]);
        PWM.SEQ[b].PTR = (uint32_t) buffer[b].getBytes();
        PWM.SEQ[b].CNT = buffer[b].length() / 2;
        bufferCount++;

        dataReady--;

        filled = 1;
        goto tryPullReturn;
    }

    // If we're in active streaming mode, and have requested a buffer and failed to get one, we have an underflow.
    // Streaming mode is double buffered, so schedule ourself to stop after the next buffer is played, if we're so configured.
    if (streaming && active && !repeatOnEmpty)
    {
        state = PwmState::SendingLastBuffer;
        // The PWM doesn't seem to respond to changes in the SHORTS register while it's active...
        // instead, we provide an empty buffer to prevent partial repetition of any previous buffer.
#if NRF52PWM_STATS > 0
        stats[0].nodata++;
#endif
        // Zero the existing played data if buffer is larger
        size_t sampleCount = buffer[b].length() / sizeof(uint16_t);
        if (sampleCount > NRF52PWM_EMPTY_BUFFERSIZE) {
            uint16_t *bufferData = (uint16_t *) &buffer[b][0];
            for (size_t i = 0; i < sampleCount; i++)
                bufferData[i] = PWM_ZERO_DC;
        } else {
            PWM.SEQ[b].PTR = (uint32_t) emptyBuffer;
            PWM.SEQ[b].CNT = (uint32_t) NRF52PWM_EMPTY_BUFFERSIZE;
        }
        stopStreamingAfterBuf = 1;
    }
  tryPullReturn:

#if NRF52PWM_STATS > 0
    stats[0].trypull_dur[stats[0].trypullcnt++ % IRQSTATLEN] = DWT->CYCCNT - t1_cyc;
#endif
    return filled;
}

/**
 * Callback provided when data is ready.
 */
int NRF52PWM::pullRequest()
{
    dataReady++;

    // If we're not running in streaming mode, simply pull the requested buffer and schedule for DMA.
    if (!streaming)
    {
        int result = tryPull(0);
        if (result || repeatOnEmpty)
            PWM.TASKS_SEQSTART[0] = 1;
    }

    // If we're in streaming mode, ensure that we've preloaded both double buffers before initiating playout.
    // note: care needed here, as our upstream data source MAY recursively call pullRequest() again in response to us
    // pulling the first buffer...
    if (streaming && !active)
    {
        active = true;
#if NRF52PWM_STATS > 0
        stats[0].irqtotalcountatstart = irqTotalCount;
#endif
        errorFlag = 0;
        bufferCount = 0;
        irqBeginTotalCount = irqTotalCount;
        if (irqBeginTotalCount != irqStopTotalCount)
            errorFlag |= NRF52PWM_ERROR_IDLEIRQ;
        upstream.dataWanted(DATASTREAM_WANTED);

        tryPull(bufferPlaying);
        bufferPlaying = (bufferPlaying + 1) % 2;

        if (bufferPlaying !=0 && dataReady > 0)
        {
            tryPull(bufferPlaying);
            bufferPlaying = (bufferPlaying + 1) % 2;
        }

        // Check if we've preloaded both buffers
        if (bufferPlaying == 0) {
            setPwmLoopInten(streaming);
            PWM.TASKS_SEQSTART[0] = 1;
            state = PwmState::SendingBuffers;

#if NRF52PWM_STATS > 0
            stats[0].pwmstart_time[stats[0].pwmstarts] = DWT->CYCCNT;
            stats[0].pwmstarts++;
#endif
        } else {
            active = false;
#if NRF52PWM_STATS > 0
            stats[0].preloadfails++;
#endif
        }
    }

    return DEVICE_OK;
}

/**
 * Base implementation of a DMA callback
 */
void NRF52PWM::irq()
{
#if NRF52PWM_STATS > 0
    size_t statidx = stats[0].irqcount % IRQSTATLEN;
    stats[0].irq_times[statidx] = DWT->CYCCNT;
    stats[0].irqcount++;
    if (!active) {
        stats[0].irqinactive++;
    }
    if (stopStreamingAfterBuf) {
        stats[0].irqpoststop++;
    }
    if (stats[0].pwmstarts == 0) {
        stats[0].irqprestart++;
    }
#endif

    irqTotalCount++;  // this counter will eventually wrap
    if (state == PwmState::Inactive || state == PwmState::Priming)
        errorFlag |= NRF52PWM_ERROR_EARLYIRQ;
    else if (state == PwmState::Stopping || state == PwmState::Stopped)
        errorFlag |= NRF52PWM_ERROR_LATEIRQ;

    // once the sequence has finished playing, load up the next buffer.
    bool end0 = PWM.EVENTS_SEQEND[0];
    if (end0)
    {
        bufferPlaying = 1;
        tryPull(0);
#if NRF52PWM_STATS > 0
        stats[0].irq0++;
#endif
        PWM.EVENTS_SEQEND[0] = 0;
    }

    bool end1 = PWM.EVENTS_SEQEND[1];
    if (end1)
    {
        bufferPlaying = 0;
        tryPull(1);
#if NRF52PWM_STATS > 0
        stats[0].irq1++;
#endif
        PWM.EVENTS_SEQEND[1] = 0;
    }

    if (end0 && end1) {
        errorFlag |= NRF52PWM_ERROR_MULTIEVENT;
    }
#if NRF52PWM_STATS > 0
    if (end0 && end1) {
        stats[0].irqboth++;
    }
    stats[0].irq_seqend[statidx] = (end1 ? 0b10 : 0) + (end0 ? 0b01 : 0);
#endif
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
    if (channel >= NRF52PWM_PWM_CHANNELS)
        return DEVICE_INVALID_PARAMETER;

    // If the pin is already connected to the requested channel, we have nothing to do.
    // We optimise this in order to prevent any possible glitch to an already connected channel...
    if (PWM.PSEL.OUT[channel] == pin.name)
        return DEVICE_OK;

    pin.disconnect();
    pin.setDigitalValue(0);
    PWM.PSEL.OUT[channel] = pin.name;
    pin.connect(*this);

    pin.status |= IO_STATUS_ANALOG_OUT;
    return DEVICE_OK;
}

/**
* Method to release the given pin from a peripheral, if already bound.
* Device drivers should override this method to disconnect themselves from the give pin
* to allow it to be used by a different peripheral.
*
* @param pin the Pin to be released
*/
int
NRF52PWM::releasePin(Pin &pin)
{
    for (int channel = 0; channel < NRF52PWM_PWM_CHANNELS; channel++)
        if (PWM.PSEL.OUT[channel] == pin.name)
            PWM.PSEL.OUT[channel] = 0xFFFFFFFF;

    if (deleteOnRelease)
        delete this;

    return DEVICE_OK;
}

int NRF52PWM::disconnectPin(Pin &pin)
{
    return releasePin(pin);
}
