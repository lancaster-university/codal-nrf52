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
#include "CodalDMESG.h"
#include "NRF52ADC.h"
#include "nrf.h"
#include "cmsis.h"

// Calculation to determine the optimal usable space for a DMA buffer fo rhte given number of channels
#define NRF52ADC_DMA_ALIGNED_SIZED(c)   ((bufferSize - (bufferSize % (c * 2)))/2);

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
 * @param channel The analog identifier of this channel
 */
NRF52ADCChannel::NRF52ADCChannel(uint8_t channel) : output(*this)
{
    this->channel = channel;
    this->size = 0;
    this->bufferSize = NRF52_ADC_DMA_SIZE;
    this->status = 0;
    this->format = DATASTREAM_FORMAT_16BIT_SIGNED;

    this->disable();

    // Configure for typical analogue capture settings
    NRF_SAADC->CH[channel].CONFIG = (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) |
        (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
        (SAADC_CH_CONFIG_GAIN_Gain1_2 << SAADC_CH_CONFIG_GAIN_Pos) |
        (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
        (SAADC_CH_CONFIG_TACQ_5us << SAADC_CH_CONFIG_TACQ_Pos) |
        (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos );

    // Define our output stream as non-blocking.
    output.setBlocking(false);
}

/**
 *  Determine the data format of the buffers streamed out of this component.
 */
int NRF52ADCChannel::getFormat()
{
    return format;
}

/**
 * Defines the data format of the buffers streamed out of this component.
 * @param format valid values include DATASTREAM_FORMAT_16BIT_SIGNED, DATASTREAM_FORMAT_16BIT_UNSIGNED
 */
int NRF52ADCChannel::setFormat(int format)
{
    if (format == DATASTREAM_FORMAT_16BIT_SIGNED || format ==  DATASTREAM_FORMAT_16BIT_UNSIGNED)
    {
        this->format = format;
        return DEVICE_OK;
    }

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
int NRF52ADCChannel::servicePendingRequests()
{
    int result = 0;

    if (status & NRF52_ADC_CHANNEL_STATUS_AWAIT_ENABLE)
    {
        DMESG("DEMUX: HARD_ENABLE %d", channel);
        NRF_SAADC->CH[channel].PSELP = channel+1;
        status &= ~NRF52_ADC_CHANNEL_STATUS_AWAIT_ENABLE;
        status |= NRF52_ADC_CHANNEL_STATUS_ENABLED;
        result=1;
    }

    if (status & NRF52_ADC_CHANNEL_STATUS_AWAIT_DISABLE)
    {
        DMESG("DEMUX: HARD_DISABLE %d", channel);
        NRF_SAADC->CH[channel].PSELP = 0;
        NRF_SAADC->CH[channel].PSELN = 0;
        status &= ~NRF52_ADC_CHANNEL_STATUS_AWAIT_DISABLE;
        status &= ~NRF52_ADC_CHANNEL_STATUS_ENABLED;
        result=1;
    }

    return result;
}

/**
 * Enable this channel
 */
void NRF52ADCChannel::enable()
{
    if (!isEnabled())
    {
        DMESG("NRF52ADC: ENABLING CHANNEL %d\n", channel);
        status |= NRF52_ADC_CHANNEL_STATUS_AWAIT_ENABLE;
    }
}


/**
 * Disable this channel
 */
void NRF52ADCChannel::disable()
{
    if (isEnabled()){
        DMESG("NRF52ADC: DISABLING CHANNEL %d\n", channel);
        status |= NRF52_ADC_CHANNEL_STATUS_AWAIT_DISABLE;
    }
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
 * Pass through any connect requests to our output buffer component.
 *
 * @param component The new downstream component for this microhone audio source.
 */
void NRF52ADCChannel::connect(DataSink& component)
{
    DMESG("NRF52ADC: CONNECT");
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
    // Flag that we need to take a sample
    status |= NRF52_ADC_CHANNEL_STATUS_AWAIT_SAMPLE;

    // Cycle the ADC, so that it pushes a DMA buffer.
    NRF_SAADC->INTENSET =  (SAADC_INTENSET_RESULTDONE_Enabled << SAADC_INTENSET_RESULTDONE_Pos);
    //NRF_SAADC->TASKS_STOP = 1;

    // Spin until the sample arrives, then return the result.
    while(status & NRF52_ADC_CHANNEL_STATUS_AWAIT_SAMPLE);

    // Clip off any samples below the negative rail...
    if (lastSample < 0)
        lastSample = 0;

    return (uint16_t) lastSample;
}

/**
 * Define the gain level for the analog input.
 *
 * @param gain Value in the range 0..7. This corresponds to  the following gain being applied to the imput channel:
 *
 * 0: 1/6
 * 1: 1/5
 * 2: 1/4
 * 3: 1/3
 * 4: 1/2
 * 5: 1
 * 6: 2
 * 7: 4
 *
 * Default is defined by NRF52_ADC_DEFAULT_GAIN.
 */
int NRF52ADCChannel::setGain(int gain)
{
    uint32_t r = NRF_SAADC->CH[channel].CONFIG & 0xfffff8ff;

    if (gain < 0 || gain > 7)
        return DEVICE_INVALID_PARAMETER;

    // mask off the current gain setting.
    r |= (gain << 8);

    // Update settings in the SAADC.
    NRF_SAADC->CH[channel].CONFIG = r;

    return DEVICE_OK;
}

/**
 * Demultiplexes the current DMA output buffer into the buffer of this channel.
 * 
 * @param data the DMA buffer to read from
 * @param offset the offset into the buffer before the first sample
 * @param skip the number of samples to skip between each read of the DMA buffer
 */
void NRF52ADCChannel::demux(ManagedBuffer dmaBuffer, int offset, int skip)
{
    int length = dmaBuffer.length() / 2;
    uint16_t *data = (uint16_t *) &dmaBuffer[0];
    uint16_t *end = data + length;
    data += offset;

    DMESG("DEMUX: channel %d, length %d, buffer %p", channel, dmaBuffer.length(),&dmaBuffer[0]);

    if ((status & NRF52_ADC_CHANNEL_STATUS_ENABLED) == 0)
        return;

    // Record the most recent sample, in case we're asked later.
    lastSample = *(data + length - skip);
    status &= ~NRF52_ADC_CHANNEL_STATUS_AWAIT_SAMPLE;

    // Extract our channel data, if we're configured for data streaming.
    if (status & NRF52_ADC_CHANNEL_STATUS_CONNECTED)
    { 
        if (skip == 1)
        {
            DMESG("DEMUX: channel %d ZERO COPY...", channel);
            // Optimise for a zero copy architeture for the common case of a single enabled stream.
            // Push the DMA buffer directly upstream and we're done.
            buffer = dmaBuffer;
            size = buffer.length();
            normalise();
            output.pullRequest();

        }else {
            
            DMESG("DEMUX: channel %d DEMUXING...", channel);
            // Otherwise, we have to manually demultiplex the data stream...  
            uint16_t *ptr = (uint16_t *) &buffer[size];
            int l = buffer.length();
            while(data < end)
            {
                if (size == l)
                {
                    buffer = ManagedBuffer(bufferSize);
                    size = 0;
                    ptr = (uint16_t *) &buffer[0];
                    l = buffer.length();
                }

                *ptr++ = *data;
                data += skip;
                size +=2;

                if (size == l)
                {
                    normalise();
                    output.pullRequest();
                }
            }
        }
    }
}

/**
 * Ensures encoding of an output buffer is in the requested format before a pullrequest is issued.
 *
 */
void NRF52ADCChannel::normalise()
{
    // Our buffer will be in signed 16bit format.
    if (format == DATASTREAM_FORMAT_16BIT_UNSIGNED)
    {
        DMESG("NRF52ADC: NORMALIZE TO UNSIGNED16");
        int16_t *in = (int16_t *) &buffer[0];
        int16_t *end = in + buffer.length() / 2;

        while(in < end)
        {
            *((uint16_t *)in) = *in > 0 ? (uint16_t) *in : 0;
            in++; 
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
NRF52ADC::NRF52ADC(NRFLowLevelTimer &adcTimer, int samplePeriod, uint16_t id) : timer(adcTimer), channels{0,1,2,3,4,5,6,7}
{
    // Store our configuration data.
    this->id = id;
    this->bufferSize = NRF52_ADC_DMA_SIZE;

    // Initialise receive buffers
    dma[0] = ManagedBuffer(bufferSize);
    dma[1] = ManagedBuffer(bufferSize);
    activeDMA = 0;

    // Record a handle to this driver object, for use by the IRQ handler.
    nrf52_adc_driver = this;

    // Ensure SAADC module is disabled (typically needed for determinstic configuration)
    NVIC_DisableIRQ(SAADC_IRQn);
    this->disable();

     // Set samplerate
    this->setSamplePeriod(samplePeriod);

    // Enable 10 bit sampling (although it's rather nicely delivered as a 16 bit sample).
    NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_10bit << SAADC_RESOLUTION_VAL_Pos);
    NRF_SAADC->OVERSAMPLE = 0;

    NRF_SAADC->INTENSET = (SAADC_INTENSET_STARTED_Enabled << SAADC_INTENSET_STARTED_Pos) |
        (SAADC_INTENSET_END_Enabled << SAADC_INTENSET_END_Pos) |
        (SAADC_INTENSET_STOPPED_Enabled << SAADC_INTENSET_STOPPED_Pos );

    NRF_PPI->CH[0].EEP = (uint32_t) &NRF_SAADC->EVENTS_END;
    NRF_PPI->CH[0].TEP = (uint32_t) &NRF_SAADC->TASKS_START;
    NRF_PPI->CHENSET = 1;

    // Enable moderately high priority interrupt
    NVIC_SetPriority(SAADC_IRQn, 0);
    NVIC_ClearPendingIRQ(SAADC_IRQn);
    NVIC_EnableIRQ(SAADC_IRQn);

    DMESG("NRF52ADC: ENABLING TIMER\n");
    timer.enableIRQ();
    timer.enable();
}


void NRF52ADC::irq()
{
    DMESG("ADC:IRQ");
    
    if (NRF_SAADC->EVENTS_END || NRF_SAADC->EVENTS_STOPPED)
    {
        // Snapshot the buffer we just received into
        int completeBuffer = activeDMA;
        int offset = 0;
        int channelsChanged = 0;

        // Flip to our other buffer.
        activeDMA = (activeDMA + 1) % 2;

        // truncate the buffer if we didn't fill it...
        DMESG("DMA AMOUNT: %d", NRF_SAADC->RESULT.AMOUNT*2);
        dma[completeBuffer].truncate(NRF_SAADC->RESULT.AMOUNT*2);

        // Process the buffer.
        // TODO: Consider moving this outside the interrupt context...

        for (int channel = 0; channel < NRF52_ADC_CHANNELS; channel++)
        {
            if (NRF_SAADC->CH[channel].PSELP)
                channels[channel].demux(dma[completeBuffer], offset++, enabledChannels);
            
            // Callback 
            channelsChanged += channels[channel].servicePendingRequests();
        }

        // Indicate we've processed the interrupt
        if(NRF_SAADC->EVENTS_END)
            NRF_SAADC->EVENTS_END = 0;

        // If we've been stopped, the number of active channels may have changed. 
        // Recalculate DMA buffer size, ensure the our target DMA buffer sample count is a multiple of the channel count
        if(NRF_SAADC->EVENTS_STOPPED){
            NRF_SAADC->EVENTS_STOPPED = 0;
         
            if (channelsChanged > 0 && enabledChannels > 0)
            {
                NRF_SAADC->ENABLE = 0;
                NRF_SAADC->RESULT.MAXCNT = NRF52ADC_DMA_ALIGNED_SIZED(enabledChannels);        
                NRF_SAADC->ENABLE = 1;
                DMESG("NRF52ADC: IRQ: EVENTS_RESTART: PTR: %p MAXCNT: %d", NRF_SAADC->RESULT.PTR, NRF_SAADC->RESULT.MAXCNT);
                NRF_SAADC->TASKS_START = 1;
            }
        }

        NRF_SAADC->EVENTS_RESULTDONE = 0;

        // Stop sampling if no channels are active.
        if (enabledChannels == 0)
            NRF_SAADC->ENABLE = 0;
    }
 
    if (NRF_SAADC->EVENTS_STARTED)
    {
        int nextDMA = (activeDMA + 1) % 2;
        dma[nextDMA] = ManagedBuffer(bufferSize);
        NRF_SAADC->RESULT.PTR = (uint32_t) &dma[nextDMA][0];
        DMESG("NRF52ADC: IRQ: EVENTS_STARTED: PTR: %p MAXCNT: %d", NRF_SAADC->RESULT.PTR, NRF_SAADC->RESULT.MAXCNT);
        NRF_SAADC->EVENTS_STARTED = 0;
    }

    if (NRF_SAADC->EVENTS_RESULTDONE && NRF_SAADC->INTENSET & (SAADC_INTENSET_RESULTDONE_Enabled << SAADC_INTENSET_RESULTDONE_Pos))
    {
        // A result has completed, and we've been flagged to interrupt on sample complete.
        // Clear the request and issue a STOP event to flush our buffer.
        NRF_SAADC->INTENCLR =  (SAADC_INTENSET_RESULTDONE_Enabled << SAADC_INTENSET_RESULTDONE_Pos);
        NRF_SAADC->EVENTS_RESULTDONE = 0;
        NRF_SAADC->TASKS_STOP = 1;
    }
}

/**
 * Enable this component
 */
void NRF52ADC::enable()
{

    if (NRF_SAADC->ENABLE == 0 && enabledChannels > 0)
    {
        DMESG("NRF52ADC: ENABLE\n");
    
        // TODO: define MAXCNT to be a multiple of the number of active channels, to keep DMA transfers easy to manage.
        dma[activeDMA] = ManagedBuffer(bufferSize);
        NRF_SAADC->RESULT.PTR = (uint32_t) &dma[activeDMA][0];
        NRF_SAADC->RESULT.MAXCNT = NRF52ADC_DMA_ALIGNED_SIZED(enabledChannels); 
        NRF_SAADC->ENABLE = 1;
        DMESG("ENABLE_START: PTR: %p MAXCNT: %d channels: %d bufferSize:%d", NRF_SAADC->RESULT.PTR, NRF_SAADC->RESULT.MAXCNT, enabledChannels,bufferSize);

        NRF_SAADC->TASKS_START = 1;

        DMESG("SAADC STARTED");
    }
}

/**
 * Disable this component
 */
void NRF52ADC::disable()
{
    DMESG("NRF52ADC: DISABLE\n");
    // Schedule all DMA transfers to stop after the next DMA transaction completes.
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
    // We use a generic timer to drive the ADC.
    // The ADC does have an internal clock, but only supports a single ADC channel, which is too limiting for us.
    // Why does it only support one channel? Who knows!  ¯\_(ツ)_/¯
    // Supporting just one single channel out of eight sounds like a really hard thing to build. :-/ 

    // Configure as a fixed period timer for the required period
    timer.setMode(TimerMode::TimerModeTimer);
    timer.setClockSpeed(1000000);
    timer.setCompare(0, samplePeriod);
    
    // Enable auto-reset
    timer.timer->SHORTS = 0x00000001;

    // use a PPI channel to link the timer overflow event with the ADC START task.
    // TODO: Add PPI channel to the resource registry...
    
    NRF_PPI->CH[1].EEP = (uint32_t) &timer.timer->EVENTS_COMPARE[0];
    NRF_PPI->CH[1].TEP = (uint32_t) &NRF_SAADC->TASKS_SAMPLE;
    NRF_PPI->CHENSET = 2;

    // Reset the timer.
    timer.timer->TASKS_CLEAR = 1;

    // Store the sample rate for later.
    this->samplePeriod = samplePeriod;

    return DEVICE_OK;
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
    this->bufferSize = bufferSize;
    return DEVICE_OK;
}

/**
 * Acquire a new ADC channel, if available, for the given pin.
 * @param pin The pin to attach.
 * @return a pointer to an NRF52ADCChannel on success, NULL if the given pin does not support analogue input, or if all channels are in use.
 */
NRF52ADCChannel* NRF52ADC::getChannel(Pin& pin)
{
    int c;

    if (!nrf52_saadc_id.hasKey(pin.name))
    {
        DMESGF("NRF52ADC: CHANNEL NOT FOUND");
        return NULL;
    }

    c = nrf52_saadc_id.get(pin.name) - 1;

    if (!channels[c].isEnabled())
    {
        DMESGF("NRF52ADC: ENABLING CHANNEL: %d", c);
        channels[c].enable();
        enabledChannels++;

        // If this is the first channel enabled, enable the ADC.
        // Otherwise, signal a STOP event to restart the ADC
        // with this new channel included.

        if (enabledChannels == 1)
        {
            channels[c].servicePendingRequests();
            this->enable();
        }
        else
        {
            NRF_SAADC->TASKS_STOP = 1;
        }
    }
    DMESG("NRF52ADC: CHANNEL RETURNING");
    return &channels[c];
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
        channels[c].disable();
        this->enabledChannels--;
    }
    
    return DEVICE_OK;
}

