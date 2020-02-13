#include "CodalConfig.h"
#include "codal-core/inc/types/Event.h"
#include "Timer.h"
#include "Pin.h"
#include "DataStream.h"
#include "nrf.h"

#ifndef NRF52PWM_H
#define NRF52PWM_H

#ifndef NRF52PWM_DEFAULT_FREQUENCY
#define NRF52PWM_DEFAULT_FREQUENCY 16000
#endif

using namespace codal;

class NRF52PWM : public CodalComponent, public DataSink
{

private:
    NRF_PWM_Type    &PWM;
    bool            enabled;
    bool            active;
    int             dataReady;
    int             sampleRate;

    void prefill();

public:

    // The stream component that is serving our data
    DataSource  &upstream;
    ManagedBuffer buffer, nextBuffer;

    /**
      * Constructor for an instance of a DAC.
      *
      * @param source The DataSource that will provide data.
      * @param sampleRate The frequency (in Hz) that data will be presented.
      * @param id The id to use for the message bus when transmitting events.
      */
    NRF52PWM(NRF_PWM_Type *module, DataSource &source, int sampleRate = NRF52PWM_DEFAULT_FREQUENCY, uint16_t id = DEVICE_ID_SYSTEM_DAC);

    /**
     * Callback provided when data is ready.
     */
	virtual int pullRequest();

    /**
     * Pull down a buffer from upstream, and schedule a DMA transfer from it.
     */
    int pull();

    /**
     * Determine the DAC playback sample rate to the given frequency.
     * @return the current sample rate.
     */
    int getSampleRate();

    /**
     * Determine the maximum unsigned vlaue that can be loaded into the PWM data values, for the current
     * frequency configuration.
     */
    int getSampleRange();

    /**
     * Change the DAC playback sample rate to the given frequency.
     * @param frequency The new sample playback frequency.
     */
    int setSampleRate(int frequency);

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
     * Direct output of given PWM channel to the given pin
     */
    int
    connectPin(Pin &pin, int channel);

};

#endif
