/*
The MIT License (MIT)

Copyright (c) 2020 Lancaster University.

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

#include "CodalCompat.h"
#include "CodalUtil.h"
#include "CodalDmesg.h"
#include "NRF52ADC.h"
#include "nrf.h"
#include "cmsis.h"

// Calculation to determine the optimal usable space for a DMA buffer for the given number of channels
#define NRF52ADC_DMA_ALIGNED_SIZED(c)   ((bufferSize - (bufferSize % (c * 2 * softwareOversample)))/2);

//
// Configuration to map a given pin to it's associated NRF52 analog ID.
//
static const KeyValueTableEntry nrf52_saadc_id_data[] = {
    {2,1},
    {3,2},
    {4,3},
    {5,4},
    {28,5},
    {29,6},
    {30,7},
    {31,8}
};
CREATE_KEY_VALUE_TABLE(nrf52_saadc_id, nrf52_saadc_id_data);

// Handle on the last (and probably only) instance of this class (NRF52 has only one SAADC module)
static NRF52ADC *nrf52_adc_driver = NULL;

//void nrf52_adc_irq(void)
extern "C" void SAADC_IRQHandler()
{
    // Simply pass on to the driver component handler.
    if (nrf52_adc_driver)
        nrf52_adc_driver->irq();
}

/**
 * Constructor
 * @param adc reference to the ADC hardware used by this channel
 * @param channel The analog identifier of this channel
 */
NRF52ADCChannel::NRF52ADCChannel(NRF52ADC &adc, uint8_t channel) : adc(adc), output(*this)
{
    this->channel = channel;
    this->size = buffer.length();
    this->bufferSize = NRF52_ADC_DMA_SIZE;
    this->gain = 2;
    this->bias = 0;
    this->status = 0;
    this->lastSample = 0;

    // Define our output stream as non-blocking.
    output.setBlocking(false);
}

/**
 *  Determine the data format of the buffers streamed out of this component.
 */
int NRF52ADCChannel::getFormat()
{
    return DATASTREAM_FORMAT_16BIT_SIGNED;
}

/**
 * Defines the data format of the buffers streamed out of this component.
 * @param format valid values include DATASTREAM_FORMAT_16BIT_SIGNED.
 */
int NRF52ADCChannel::setFormat(int format)
{
    if (format == DATASTREAM_FORMAT_16BIT_SIGNED)
        return DEVICE_OK;

    return DEVICE_NOT_SUPPORTED;
}

/**
 *  Determine the buffer size this channel will use for data streaming.
 *  @return the current size of the buffers used by this channel, in bytes.
 */
int NRF52ADCChannel::getBufferSize()
{
    return bufferSize;
}

/**
 *  Defines the buffer size this channel will use for data streaming.
 *  @param the new size of the buffers used by this channel, in bytes. Must be 16 bit aligned.
 *  @return DEVICE_OK on success, or DEVICE_INVALID_PARAMETER if bufferSize is not 16 bit aligned.
 */
int NRF52ADCChannel::setBufferSize(int bufferSize)
{
    if (bufferSize % 2 == 0)
    {
        this->bufferSize = bufferSize;
        return DEVICE_OK;
    }

    return DEVICE_INVALID_PARAMETER;
}

/**
 * Enable this channel
 */
void NRF52ADCChannel::enable()
{
    status |= NRF52_ADC_CHANNEL_STATUS_ENABLED;
}

/**
 * Disable this channel
 */
void NRF52ADCChannel::disable()
{
    status &= ~NRF52_ADC_CHANNEL_STATUS_ENABLED;
}

/**
 * Determines if this component is enabled
 */
int NRF52ADCChannel::isEnabled()
{
    return status & NRF52_ADC_CHANNEL_STATUS_ENABLED;
}


/**
 * Update our reference to a downstream component.
 *
 * @param component The new downstream component for this ADC.
 */
void NRF52ADCChannel::connect(DataSink& component)
{
    this->status |= NRF52_ADC_CHANNEL_STATUS_CONNECTED;
}

/**
 * Provide the next available ManagedBuffer to our downstream caller, if available.
 */
ManagedBuffer NRF52ADCChannel::pull()
{
    return buffer;
}

/**
 * Determine the value of the next sample read from this channel.
 * 
 * @return The next sample read from this channel;
 */
uint16_t NRF52ADCChannel::getSample()
{
    if (!isEnabled())
        return 0;

    // Get the currently active DMA buffer
    ManagedBuffer b = adc.getActiveDMABuffer();

    // Determine the offset of skip value of this channel within the buffer.
    int skip = adc.getActiveChannelCount();
    int offset = adc.getChannelOffset(channel);

    // Generate pointers to the start and end of the DMA buffer
    uint16_t *ptr = (uint16_t *) b.getBytes();
    uint16_t *end = (uint16_t *) (b.getBytes() + b.length());

    // Account for multiplexed offset
    ptr += offset;

    // Find the last entry in the buffer that's been written for this channel
    if (*ptr != 0x8888)
    {
        while (*ptr != 0x8888 && ptr < end)
            ptr += skip;

        ptr -= skip;

        lastSample = *((int16_t *) ptr);
    }

    // Clip off the negative rail
    // Protect against IRQ changing lastSample after clip
    int16_t sample = lastSample;
    if (sample < 0)
        sample = 0;

    return (uint16_t) sample;
}

/**
 * Define the gain level for the analog input.
 *
 * @param gain Value in the range 0..7. This corresponds to the following gain being applied to the imput channel:
 *
 * 0: 1/6
 * 1: 1/5 
 * 2: 1/4 (default)
 * 3: 1/3 
 * 4: 1/2 
 * 5: 1 
 * 6: 2 
 * 7: 4 
 *
 * @param bias Define the resistor bias used on this channel, in the range 0..3:
 * 
 * 0: No bias (default)
 * 1: Activate pull down to ground
 * 2: Activate pull up to VDD
 * 3: Activate pull to VDD/2
 */
int NRF52ADCChannel::setGain(int gain, int bias)
{
    // validate inputs
    if (gain < 0 || gain > 7 || bias < 0 || bias > 3)
        return DEVICE_INVALID_PARAMETER;

    this->gain = gain;
    this->bias = bias;

    configureGain();

    return DEVICE_OK;
}

/**
 * Configure the gain level and the resistor bias.
 *
 */
void NRF52ADCChannel::configureGain()
{
    NRF_SAADC->CH[channel].CONFIG = (bias << SAADC_CH_CONFIG_RESP_Pos) |
        (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
        (gain << SAADC_CH_CONFIG_GAIN_Pos) |
        (SAADC_CH_CONFIG_REFSEL_VDD1_4 << SAADC_CH_CONFIG_REFSEL_Pos) |
        (SAADC_CH_CONFIG_TACQ_3us << SAADC_CH_CONFIG_TACQ_Pos) |
        (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos );
}

/**
  * Determine the gain level for the analog input.
  *
  * @return The gain.
  */
int NRF52ADCChannel::getGain()
{
    return gain;
}

/**
  * Determine the bias for the analog input.
  *
  * @return The bias.
  */
int NRF52ADCChannel::getBias()
{
    return bias;
}

/**
 * Demultiplexes the current DMA output buffer into the buffer of this channel.
 * 
 * @param data the DMA buffer to read from
 * @param offset the offset into the buffer before the first sample
 * @param skip the number of samples to skip between each read of the DMA buffer
 * @param oversample the number of samples to aggregate in sofware into a final result.
 */
void NRF52ADCChannel::demux(ManagedBuffer dmaBuffer, int offset, int skip, int oversample)
{
    int length = dmaBuffer.length() / 2;
    int16_t *data = (int16_t *) &dmaBuffer[0];
    int16_t *end = data + length;
    data += offset;

    if ((status & NRF52_ADC_CHANNEL_STATUS_ENABLED) == 0)
        return;

    // If this buffer is too shot to contain information for us, ignore it.
    // n.b. The above test is safe, as a short buffer implies that our lastSample is already up to date.
    if (length <= offset)
        return;

    // Rewind end pointer to our last sample in the buffer. (likley one iteration)
    do end--;
    while ((end - data) % skip);
    
    // Record the most recent sample, in case we're asked later.
    lastSample = *end++;

    // Extract our channel data, if we're configured for data streaming.
    if (status & NRF52_ADC_CHANNEL_STATUS_CONNECTED)
    { 
        if (skip == 1)
        {
            // Optimise for a zero copy architeture for the common case of a single enabled stream.
            // Push the DMA buffer directly upstream and we're done.
            buffer = dmaBuffer;
            size = buffer.length();
            output.pullRequest();

        }else {
            // Otherwise, we have to manually demultiplex the data stream... 
            int sample = 0;
            int sampleCount = 0;

            int16_t *ptr = (int16_t *) &buffer[size];
            int l = buffer.length();
            while(data < end)
            {
                if (size == l)
                {
                    buffer = ManagedBuffer(bufferSize);
                    size = 0;
                    ptr = (int16_t *) &buffer[0];
                    l = buffer.length();
                }

                sample += *data;
                data += skip;
                sampleCount++;

                if (sampleCount == oversample)
                {
                    *ptr++ = (int16_t) (sample / oversample);
                    sampleCount = 0;
                    sample = 0;
                    size += 2;
                }

                if (size == l)
                    output.pullRequest();
            }
        }
    }
}

/**
 * Constructor for an instance of an analog to digital converter,
 *
 * @param timer the generic timer to be used to drive this ADC
 * @param samplePeriod the time between samples generated in the output buffer (in microseconds)
 * @param id The id to use for the message bus when transmitting events.
 */
NRF52ADC::NRF52ADC(NRFLowLevelTimer &adcTimer, int samplePeriod, uint16_t id) : timer(adcTimer), channels{NRF52ADCChannel(*this, 0), NRF52ADCChannel(*this, 1), NRF52ADCChannel(*this, 2), NRF52ADCChannel(*this, 3), NRF52ADCChannel(*this, 4), NRF52ADCChannel(*this, 5), NRF52ADCChannel(*this, 6), NRF52ADCChannel(*this, 7)}
{
    // Store our configuration data.
    this->id = id;
    this->samplePeriod = samplePeriod;
    this->softwareOversample = 1;
    this->bufferSize = NRF52_ADC_DMA_SIZE;
    this->enabledChannels = 0;
    this->running = false;

    // Initialise receive buffers
    dma[0] = allocateDMABuffer();
    dma[1] = allocateDMABuffer();
    activeDMA = 0;

    // Record a handle to this driver object, for use by the IRQ handler.
    nrf52_adc_driver = this;

    // Ensure SAADC module is disabled (typically needed for determinstic configuration)
    NVIC_DisableIRQ(SAADC_IRQn);
    NRF_SAADC->ENABLE = 0;

    // Enable 14 bit sampling (although it's rather nicely delivered as a 16 bit sample).
    NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_14bit << SAADC_RESOLUTION_VAL_Pos);

    // Configure for an interrupt on ND and STOP events.
    NRF_SAADC->INTENSET = (SAADC_INTENSET_STARTED_Enabled << SAADC_INTENSET_STARTED_Pos) |
        (SAADC_INTENSET_END_Enabled << SAADC_INTENSET_END_Pos) |
        (SAADC_INTENSET_STOPPED_Enabled << SAADC_INTENSET_STOPPED_Pos );

    // Configure a PPI channel to automatically launch a START event when an END event occurs
    // (continuous sampling, with double buffered DMA)
    NRF_PPI->CHENCLR = 1;
    NRF_PPI->CH[0].EEP = (uint32_t) &NRF_SAADC->EVENTS_END;
    NRF_PPI->CH[0].TEP = (uint32_t) &NRF_SAADC->TASKS_START;

    // Set samplerate and calculate oversampling values
    configureSampling();

    // Enable moderately high priority interrupt
    NVIC_SetPriority(SAADC_IRQn, 0);
    NVIC_ClearPendingIRQ(SAADC_IRQn);
    NVIC_EnableIRQ(SAADC_IRQn);

    timer.enable();
    timer.disableIRQ();
}

/**
 * Allocate a new DMA buffer
 * @return A newly allocated DMA buffer, initialized and ready for use.
 */
ManagedBuffer NRF52ADC::allocateDMABuffer()
{
    ManagedBuffer b(bufferSize, BufferInitialize::None);

    // Fill the buffer with values unused by the ADC hardware. We can use this to perform demuxing 
    // of a live DMA buffer as it is being filled. :)
    // We choose 0x88 as a 16 bit signed 0x8888 is an invalid 14 bit sample (either +ve or -ve)
    b.fill(0x88);

    return b;
}

void NRF52ADC::irq()
{
    if (NRF_SAADC->EVENTS_END || NRF_SAADC->EVENTS_STOPPED)
    {
        // Snapshot the buffer we just received into
        int completeBuffer = activeDMA;
        int offset = 0;

        // Flip to our other buffer.
        activeDMA = (activeDMA + 1) % 2;

        // truncate the buffer if we didn't fill it...
        dma[completeBuffer].truncate(NRF_SAADC->RESULT.AMOUNT*2);

        // Process the buffer.
        // TODO: Consider moving this outside the interrupt context...

        for (int channel = 0; channel < NRF52_ADC_CHANNELS; channel++)
        {
            if (NRF_SAADC->CH[channel].PSELP)
                channels[channel].demux(dma[completeBuffer], offset++, enabledChannels, softwareOversample);
        }

        // Indicate we've processed the interrupt
        if(NRF_SAADC->EVENTS_END)
            NRF_SAADC->EVENTS_END = 0;

        if(NRF_SAADC->EVENTS_STOPPED){
            NRF_SAADC->EVENTS_STOPPED = 0;
            this->running = false;
        }
    }

    if (NRF_SAADC->EVENTS_STARTED)
    {
        int nextDMA = (activeDMA + 1) % 2;
        dma[nextDMA] = allocateDMABuffer();
        NRF_SAADC->RESULT.PTR = (uint32_t) &dma[nextDMA][0];
        NRF_SAADC->EVENTS_STARTED = 0;
    }
}

/**
 * Enable this component
 */
void NRF52ADC::enable()
{
    startRunning();
}

/**
 * Disable this component
 */
void NRF52ADC::disable()
{
    stopRunning();
    NRF_SAADC->ENABLE = 0;

    // Disable all channels.
    for (int i = 0; i < NRF52_ADC_CHANNELS; i++)
        channels[i].disable();
    
    this->enabledChannels = 0;
}

/**
 * Determine the current sample rate.
 *
 * @return The current sample period, in microseconds.
 */
int NRF52ADC::getSamplePeriod()
{
    return samplePeriod;
}

/**
 * Define the sample period to use.
 *
 * @param sampleperiod - The sample period to use, in microseconds.
 */
int NRF52ADC::setSamplePeriod(int samplePeriod)
{
    bool wasRunning = stopRunning();
    // Store the sample rate for later.
    this->samplePeriod = samplePeriod;
    if (wasRunning)
        startRunning();
    return DEVICE_OK;
}

/**
 * Set samplerate and calculate oversampling values.
 *
 */
void NRF52ADC::configureSampling()
{
    // We use a generic timer to drive the ADC.
    // The ADC does have an internal clock, but only supports a single ADC channel, which is too limiting for us.
    // Why does it only support one channel? Who knows!  ¯\_(ツ)_/¯
    // Supporting just one single channel out of eight sounds like a really hard thing to build. :-/ 

    // Configure as a fixed period timer for the required period
    timer.setMode(TimerMode::TimerModeTimer);
    timer.setClockSpeed(16000);

	// Calculate the best level of oversampling that can be achieved at the requested samplerate.
	// The ADC can perform a sample and conversion in 5uS.
	int oversample = 0;
	int possibleSamples = samplePeriod / (5 * max(enabledChannels, 1));

	// The ADC can support up to 256x oversampling, in a power of two boundary.
	possibleSamples = min(possibleSamples, 256) >> 1;

    // Calculate a lower bound for the level of oversampling we can support
	while (possibleSamples != 0)
	{
		oversample++;
		possibleSamples = possibleSamples >> 1;
	}

    // Determine the closest timer value for the requested period, taking into account oversampling.
	float timerCount = (float) samplePeriod * 16.0f;

    // Update the ADC and timer hardware with the new configuration.
    // n.b. the nrf52 ADC only supports hardware oversampling for one channel... 
    // so use software oversampling if multiple channels are enabled.
    if (enabledChannels <= 1)
    {
        timerCount = timerCount / (1 << oversample);

        NRF_SAADC->OVERSAMPLE = oversample;
        softwareOversample = 1;
    }
    else
    {
        NRF_SAADC->OVERSAMPLE = 0;
#if CONFIG_ENABLED(NRF52ADC_SOFTWARE_OVERSAMPLING)
        oversample = min(oversample, 8);
        softwareOversample = 1 << oversample;
        timerCount = timerCount / (1 << oversample);
#endif
    }
        
    timer.setCompare(0, (int)timerCount);
    
    // Enable auto-reset
    timer.timer->SHORTS = 0x00000001;

    // use a PPI channel to link the timer overflow event with the ADC START task.
    // TODO: Add PPI channel to the resource registry...
    NRF_PPI->CHENCLR = 2;
    NRF_PPI->CH[1].EEP = (uint32_t) &timer.timer->EVENTS_COMPARE[0];
    NRF_PPI->CH[1].TEP = (uint32_t) &NRF_SAADC->TASKS_SAMPLE;

    // Reset the timer.
    timer.timer->TASKS_CLEAR = 1;
}

/**
 *  Determine the DMA buffer size this ADC will use. 
 *  @return the current size of the DMA buffer used in bytes.
 */
int NRF52ADC::getDmaBufferSize()
{
    return bufferSize;
}

/**
 *  Defines the DMA buffer size this ADC will use. 
 *  @param the new size of the DMA buffer used in bytes. Note that effective buffer size used may be less, to 16 bit align with the number of active channels.
 *  @return DEVICE_OK on success.
 */
int NRF52ADC::setDmaBufferSize(int bufferSize)
{
    bool wasRunning = stopRunning();
    this->bufferSize = bufferSize;
    if (wasRunning)
        startRunning();
    return DEVICE_OK;
}

/**
 * Gets the currently active DMA buffer (the one currently being filled)
 * @return the currently active DMA buffer
 */
ManagedBuffer NRF52ADC::getActiveDMABuffer()
{
    ManagedBuffer b;

    target_disable_irq();
    b = dma[activeDMA];
    target_enable_irq();

    return b;
}

/**
 * Determine the number of active DMA channels currently being multiplexed
 * @return the number of active channels
 */
int NRF52ADC::getActiveChannelCount()
{
    return enabledChannels;
}

/**
 * Determine the offset of the given DMA channels in the raw multiplexed data stream.
 * @return the offset of the channel in the stream
 */
int NRF52ADC::getChannelOffset(int channel)
{
    int offset = 0;

    for (int c = 0; c < NRF52_ADC_CHANNELS; c++)
    {
        if (NRF_SAADC->CH[c].PSELP)
            offset++;

        if (c == channel)
            return offset-1;
    }

    return DEVICE_INVALID_PARAMETER;
}

/**
 * Acquire a new ADC channel, if available, for the given pin.
 * @param pin The pin to attach.
 * @param activate If the channel should start activated or not.
 * @return a pointer to an NRF52ADCChannel on success, NULL if the given pin does not support analogue input, or if all channels are in use.
 */
NRF52ADCChannel* NRF52ADC::getChannel(Pin& pin, bool activate)
{
    int c;

    if (!nrf52_saadc_id.hasKey(pin.name))
        return NULL;

    c = nrf52_saadc_id.get(pin.name) - 1;

    if(activate)
        this->activateChannel(&channels[c]);

    return &channels[c];
}

/**
 * Activate a ADC channel
 * @param channel The channel to activate.
 * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if the given channel does not exist.
 */
int NRF52ADC::activateChannel(NRF52ADCChannel *channel)
{
    if(channel==NULL)
        return DEVICE_INVALID_PARAMETER;

    if (!channel->isEnabled())
    {
        stopRunning();
        channel->enable();
        startRunning();
    }
    return DEVICE_OK;
}

/**
 * Release a previously a new ADC channel, if available, for the given pin.
 * @param pin The pin to detach.
 * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if the given pin was not connected to the ADC.
 */
int NRF52ADC::releaseChannel(Pin& pin)
{
    int c;

    if (!nrf52_saadc_id.hasKey(pin.name))
        return DEVICE_INVALID_PARAMETER;

    c = nrf52_saadc_id.get(pin.name) - 1;

    if (channels[c].isEnabled())
    {
        bool wasRunning = stopRunning();
        channels[c].disable();
        if (wasRunning)
            startRunning();
    }
    return DEVICE_OK;
}

bool NRF52ADC::stopRunning()
{
    if ( !running)
      return false;

    //Disable PPI links
    NRF_PPI->CHENCLR = 3;

    while ( NRF_SAADC->STATUS & 1) /*wait for not busy*/;

    NRF_SAADC->TASKS_STOP = 1;
    while ( running) /*wait for stop*/;

    return true;
}


bool NRF52ADC::startRunning()
{
    if ( running)
        return true;

    // https://infocenter.nordicsemi.com/topic/errata_nRF52833_Rev1/ERR/nRF52833/Rev1/latest/anomaly_833_212.html
    volatile uint32_t temp1;
    volatile uint32_t temp2;
    volatile uint32_t temp3;

    temp1 = *(volatile uint32_t *)0x40007640ul;
    temp2 = *(volatile uint32_t *)0x40007644ul;
    temp3 = *(volatile uint32_t *)0x40007648ul;

    *(volatile uint32_t *)0x40007FFCul = 0ul; 
    *(volatile uint32_t *)0x40007FFCul; 
    *(volatile uint32_t *)0x40007FFCul = 1ul;

    *(volatile uint32_t *)0x40007640ul = temp1;
    *(volatile uint32_t *)0x40007644ul = temp2;
    *(volatile uint32_t *)0x40007648ul = temp3;

    NRF_SAADC->ENABLE = 0;

    // Enable 14 bit sampling (although it's rather nicely delivered as a 16 bit sample).
    NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_14bit << SAADC_RESOLUTION_VAL_Pos);

    // Configure for an interrupt on STARTED, END and STOP events.
    NRF_SAADC->INTENSET = (SAADC_INTENSET_STARTED_Enabled << SAADC_INTENSET_STARTED_Pos) |
        (SAADC_INTENSET_END_Enabled << SAADC_INTENSET_END_Pos) |
        (SAADC_INTENSET_STOPPED_Enabled << SAADC_INTENSET_STOPPED_Pos );

    // Configure channels
    enabledChannels = 0;
            
    for (int channel = 0; channel < NRF52_ADC_CHANNELS; channel++)
    {
        if ( channels[channel].isEnabled())
        {
            enabledChannels++;

            channels[channel].configureGain();

            NRF_SAADC->CH[channel].PSELP = channel+1;
            NRF_SAADC->CH[channel].PSELN = 0;
        }
        else
        {
            NRF_SAADC->CH[channel].PSELP = 0;
            NRF_SAADC->CH[channel].PSELN = 0;
        }
    }

    // Stop sampling if no channels are active.
    if (enabledChannels == 0)
    {
        return false;
    }

    NRF_SAADC->ENABLE = 1;

    // Recalculate DMA buffer size, ensure the our target DMA buffer sample count is a multiple of the channel count.
    // Recalculate OVERSAMPLE and timer settings accordingly.
    configureSampling();

    // TODO: define MAXCNT to be a multiple of the number of active channels, to keep DMA transfers easy to manage.
    dma[activeDMA] = allocateDMABuffer();
    volatile uint16_t *dmaLast = ((uint16_t *) &dma[activeDMA][0]) + (enabledChannels - 1);

    NRF_SAADC->RESULT.PTR = (uint32_t) &dma[activeDMA][0];
    NRF_SAADC->RESULT.MAXCNT = NRF52ADC_DMA_ALIGNED_SIZED(enabledChannels); 
    NRF_SAADC->TASKS_START = 1;

    //Enable PPI links
    NRF_PPI->CHENSET = 3;

    running = true;

    while ( *dmaLast == 0x8888) /*wait for first sample*/;

    return true;
}

/**
 * Puts the component in (or out of) sleep (low power) mode.
 */
int NRF52ADC::setSleep(bool doSleep)
{
    static bool wasRunning;
    static bool wasEnabled;

    if (doSleep)
    {
        wasRunning = stopRunning();
        wasEnabled = NVIC_GetEnableIRQ(SAADC_IRQn);
        if (wasEnabled)
            NVIC_DisableIRQ(SAADC_IRQn);
    }
    else
    {
        if (wasEnabled)
            NVIC_EnableIRQ(SAADC_IRQn);
        if (wasRunning)
            startRunning();

    }
   
    return DEVICE_OK;
}