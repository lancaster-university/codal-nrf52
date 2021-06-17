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

#include "CodalConfig.h"
#include "codal-core/inc/types/Event.h"
#include "Pin.h"
#include "DataStream.h"
#include "NRFLowLevelTimer.h"

#ifndef NRF5_ADC_H
#define NRF5_ADC_H

//
// Constants
//
#define NRF52_ADC_CHANNELS          8
#define NRF52_ADC_DMA_SIZE          512

//
// Event codes
//
#define NRF52_ADC_DATA_READY     1


using namespace codal;

//
// NRF52ADCChannel status codes
//
#define NRF52_ADC_CHANNEL_STATUS_ENABLED                0x10
#define NRF52_ADC_CHANNEL_STATUS_CONNECTED              0x20

class NRF52ADC;

class NRF52ADCChannel : public DataSource
{
private:

    NRF52ADC            &adc;
    ManagedBuffer       buffer;
    volatile int16_t    lastSample;
    int16_t             size;
    int16_t             bufferSize;
    volatile uint8_t    status;
    uint8_t             channel;
    uint8_t             gain;
    uint8_t             bias;
 
public:
    DataStream      output;

    /**
     * Constructor
     * @param adc reference to the ADC hardware used by this channel
     * @param channel The analog identifier of this channel
     */
    NRF52ADCChannel(NRF52ADC &adc, uint8_t channel);

    /**
     * Enable this component
     */
    void enable();

    /**
     * Disable this component
     */
    void disable();

    /**
     * Determines if this component is enabled
     */
    int isEnabled();


    /**
     * Update our reference to a downstream component.
     *
     * @param component The new downstream component for this ADC.
     */
	virtual void connect(DataSink &component);

    /**
     *  Determine the data format of the buffers streamed out of this component.
     */
    virtual int getFormat();

    /**
     * Defines the data format of the buffers streamed out of this component.
     * @param format valid values include DATASTREAM_FORMAT_16BIT_SIGNED
     */
    virtual int setFormat(int format);

    /**
     *  Determine the buffer size this channel will use for data streaming.
     *  @return the current size of the buffers used by this channel, in bytes.
     */
    int getBufferSize();

    /**
     *  Defines the buffer size this channel will use for data streaming.
     *  @param the new size of the buffers used by this channel, in bytes. Must be 16 bit aligned.
     */
    int setBufferSize(int bufferSize);

    /**
	 * Provide the next available ManagedBuffer to our downstream caller, if available.
	 */
	virtual ManagedBuffer pull();

    /**
     * Determine the value of the next sample read from this channel.
     * 
     * @return The next sample read from this channel;
     */
    uint16_t getSample();

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
    int setGain(int gain = 2, int bias = 0);

    /**
     * Determine the gain level for the analog input.
     *
     * @return The gain.
     */
    int getGain();

    /**
     * Determine the bias for the analog input.
     *
     * @return The bias.
     */
    int getBias();

    /**
     * Configure the gain level and the resistor bias.
     *
     */
    void configureGain();
    
    /**
    * Demultiplexes the current DMA output buffer into the buffer of this channel.
    * 
    * @param data the DMA buffer to read from
    * @param offset the offset into the buffer before the first sample
    * @param skip the number of samples to skip between each read of the DMA buffer
    * @param oversample the number of samples to aggregate in sofware into a final result.
    */
    void demux(ManagedBuffer dmaBuffer, int offset, int skip, int oversample);
};

#define NRF52ADC_STATUS_PERIOD_CHANGED              0x01        // Indicates that the period of the ADC may have changed.


class NRF52ADC : public CodalComponent
{

public:
	uint32_t            samplePeriod;                           // The ADC sample period, in microseconds.
    uint16_t            bufferSize;                             // The size of our buffer.
    uint8_t             enabledChannels;                        // Determines the number of currently active channels.
    uint8_t             activeDMA;                              // Index of the DMA buffer being actively used.
    NRFLowLevelTimer&   timer;                                  // The timer module used to drive this ADC.
    NRF52ADCChannel     channels[NRF52_ADC_CHANNELS];           // ADC channel objects
    ManagedBuffer       dma[2];                                 // Double buffered DMA receive buffers.
    int                 softwareOversample;                     // The level of software oversampling level in use.
    volatile bool       running;
   
public:
    /**
      * Constructor for an instance of an analog to digital converter,
      *
      * @param timer the generic timer to be used to drive this ADC
      * @param samplePeriod the time between samples generated in the output buffer (in microseconds)
      * @param id The id to use for the message bus when transmitting events.
      */
    NRF52ADC(NRFLowLevelTimer &adcTimer, int samplePeriod, uint16_t id = DEVICE_ID_SYSTEM_ADC);

    /**
     * Allocate a new DMA buffer
     * @return A newly allocated DMA buffer, initialized and ready for use.
     */
    ManagedBuffer allocateDMABuffer();

    /**
     * Interrupt callback when playback of DMA buffer has completed
     */
    void irq();

    /**
     * Enable this component
     */
    void enable();

    /**
     * Disable this component
     */
    void disable();

    /**
     * Determine the current sample rate.
     *
     * @return The current sample period, in microseconds.
     */
    int getSamplePeriod();

    /**
     * Define the sample period to use.
     *
     * @param sampleperiod - The sample period to use, in microseconds.
     */
    int setSamplePeriod(int samplePeriod);

    /**
     *  Determine the DMA buffer size this ADC will use. 
     *  @return the current size of the DMA buffer used in bytes.
     */
    int getDmaBufferSize();

    /**
     *  Defines the DMA buffer size this ADC will use. 
     *  @param the new size of the DMA buffer used in bytes. Note that effective buffer size used may be less, to 16 bit align with the number of active channels.
     */
    int setDmaBufferSize(int bufferSize);

    /**
     * Gets the currently active DMA buffer (the one currently being filled)
     * @return the currently active DMA buffer
     */
    ManagedBuffer getActiveDMABuffer();

    /**
     * Determine the number of active DMA channels currently being multiplexed
     * @return the number of active channels
     */
    int getActiveChannelCount();

    /**
     * Determine the offset of the given DMA channels in the raw multiplexed data stream.
     * @return the offset of the channel in the stream
     */
    int getChannelOffset(int channel);

    /**
     * Acquire a new ADC channel, if available, for the given pin.
     * @param pin The pin to attach.
     * @param activate If the channel should start activated or not.
     * @return a pointer to an NRF52ADCChannel on success, NULL if the given pin does not support analogue input, or if all channels are in use.
     */
    NRF52ADCChannel* getChannel(Pin& pin, bool activate = true);

    /**
     * Activate a ADC channel
     * @param channel The channel to activate.
     * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if the given channel does not exist.
     */
    int activateChannel(NRF52ADCChannel *channel);

    /**
     * Release a previously a new ADC channel, if available, for the given pin.
     * @param pin The pin to detach.
     * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if the given pin was not connected to the ADC.
     */
    int releaseChannel(Pin& pin);

    /**
      * Puts the component in (or out of) sleep (low power) mode.
      */
    virtual int setSleep(bool doSleep) override;

private:
    /**
     * Stop the ADC running, if it is running.
     * 
     * @return true if it was running.
     */
    bool stopRunning();

    /**
     * Start the ADC running, if stopped and at least one channel is enabled.
     *
     * @return true if it is running.
     */
    bool startRunning();

    /**
     * Set samplerate and calculate oversampling values.
     *
     */
    void configureSampling();
};

#endif
