#include "CodalConfig.h"
#include "codal-core/inc/types/Event.h"
#include "Timer.h"
#include "NRF52Pin.h"
#include "DataStream.h"
#include "nrf.h"

#ifndef NRF52PWM_H
#define NRF52PWM_H

#ifndef NRF52PWM_DEFAULT_FREQUENCY
#define NRF52PWM_DEFAULT_FREQUENCY 16000
#endif

#define NRF52PWM_PWM_PERIPHERALS    3
#define NRF52PWM_PWM_CHANNELS       4

// This "using namespace codal" cannot be removed from this header file without
// breaking targets where some .cpp files depend on it. So this config flag is
// enabled by default and keeps the namespace for compatibility.
#if CONFIG_ENABLED(CODAL_USE_GLOBAL_NAMESPACE)
using namespace codal;
#endif

// #define NRF52PWM_STATS 4
#if NRF52PWM_STATS > 0
#define IRQSTATLEN 40 
#endif

#define NRF52PWM_ERROR_MALLOC      (1 << 0)
#define NRF52PWM_ERROR_EARLYIRQ    (1 << 1)
#define NRF52PWM_ERROR_LATEIRQ     (1 << 2)
#define NRF52PWM_ERROR_IDLEIRQ     (1 << 3)
#define NRF52PWM_ERROR_MISMATCHIRQ (1 << 4)
#define NRF52PWM_ERROR_MULTIEVENT  (1 << 5)

namespace codal
{
    enum class PwmState : uint8_t
    {
        Inactive = 0,
        Priming,
        SendingBuffers,
        SendingLastBuffer,
        Stopping,
        Stopped
    };

#if NRF52PWM_STATS > 0
    struct Nrf52PwmStats {
    // All times/durations are in cycles
    volatile uint32_t irqcount;             // counter
    volatile uint32_t irq0;                 // counter for SEQEND[0]
    volatile uint32_t irq1;                 // counter for SEQEND[1]
    volatile uint32_t irqboth;              // counter for double event
    volatile uint32_t irq_times[IRQSTATLEN];  // timestamps
    volatile int    irq_seqend[IRQSTATLEN];   // bitmask for events
    volatile int    irqinactive;            // interrupts while !active
    volatile int    irqprestart;            // interrupts before pwm start (zero expected!)
    volatile int    irqpoststop;            // interrupts after a stop (one expected)
    volatile uint32_t trypullcnt;           // counter
    volatile uint32_t trypull_dur[IRQSTATLEN];  // durations
    volatile int    pwmstarts;              // PWM starts
    volatile uint32_t pwmstart_time[IRQSTATLEN];  // timestamps
    volatile int    pwmstops;               // PWM stops
    volatile int    preloadfails;           // failing to load both buffers before start
    volatile int    dataReadyAtStop;        // preserved dataReady value
    volatile int    nodata;                 // upstream failed to produce data in time
    volatile uint32_t zero_time;            // stats zero timestamp
    volatile uint32_t irqtotalcountatstart; // record of irqtotalcount at start
    volatile uint32_t irqtotalcountatstop;  // record of irqtotalcount at stop
};
#endif

class NRF52PWM : public CodalComponent, public DataSink, public PinPeripheral
{

private:
    NRF_PWM_Type    &PWM;                   // The hardware PWM module used by this instance
    volatile bool   enabled;                // Determines if this PWM instance is enabled
    volatile bool   active;                 // Determines if this PWM instance is actively generating output
    volatile bool   streaming;              // Determines if the output is streamed, or discrete. Streamed mode maintains ordered, discrete repeats playout most recent data provided.
    volatile bool   repeatOnEmpty;          // Determines the behaviour of the PWM if a buffer underflow occurs.
    volatile int    dataReady;              // Count of the number of input buffers awaiting playout
    volatile float  sampleRate;             // Requested frequency in hertz
    volatile float  periodUs;               // Acutal period between output samples, in microseconds
    volatile uint8_t bufferPlaying;         // ID of the buffer currently being played (0 or 1). Output is hardware double buffered.
    volatile int8_t stopStreamingAfterBuf;  // When stopping, the last buffer ID to play beforhand. -1 if no stop is scheduled.
    ManagedBuffer   buffer[2];              // The ManagedBuffers currently being used by the PWM hardware
    volatile uint32_t bufferCount;          // Buffers processed
    volatile uint32_t irqTotalCount;        // Total count of interrupts
    volatile uint32_t irqBeginTotalCount;   // Count before preloading buffers
    volatile uint32_t irqStopTotalCount;    // Count at PWM stop
    volatile uint16_t errorFlag;            // Error bitfield per stream
    volatile uint16_t errorFlagCumlative;   // Error bitfield all streams
    volatile PwmState state;
#if NRF52PWM_STATS > 0
    struct Nrf52PwmStats stats[NRF52PWM_STATS];  // statistics per stream for debugging
#endif

public:  
    // The stream component that is serving our data
    DataSource      &upstream;

    // Handles on the instances of this class used the three PWM modules (if present)
    static NRF52PWM *nrf52_pwm_driver[NRF52PWM_PWM_PERIPHERALS];

    /**
      * Constructor for an instance of a PWM acting as a sink to a given (likely DMA enabled) datastream.
      * Use this constructor to create a PWM output who's values are controlled dynamically by an asynchronous data stream.
      *
      * @param source The DataSource that will provide data.
      * @param sampleRate The frequency (in Hz) that data will be presented.
      * @param id The id to use for the message bus when transmitting events.
      */
    NRF52PWM(NRF_PWM_Type *module, DataSource &source, float sampleRate = NRF52PWM_DEFAULT_FREQUENCY, uint16_t id = DEVICE_ID_SYSTEM_DAC);

#if NRF52PWM_STATS > 0
    /**
     * Clear the statistics, typically used before starting to output PWM
     */
    void zeroStats();

    /**
     * Check the statistics for anomalies and if any are found
     * DMESG print whole structure.
     */
    void anomalyCheckStats(int bufcnt);
#endif

    /**
     * Callback provided when data is ready.
     */
    virtual int pullRequest();

    /**
     * Determine the DAC playback sample rate to the given frequency.
     * @return the current sample rate.
     */
    float getSampleRate();

    /**
     * Determine the maximum unsigned vlaue that can be loaded into the PWM data values, for the current
     * frequency configuration.
     */
    int getSampleRange();

    /**
     * Change the DAC playback sample rate to the given frequency.
     * @param frequency The new sample playback frequency.
     */
    float setSampleRate(float frequency);

    /**
     * Determine the current DAC playback period.
     * @return period The sample playback period, in microseconds.
     */
    float getPeriodUs();


    /**
     * Change the DAC playback sample rate to the given period.
     * @param period The new sample playback period, in microseconds.
     */
    int setPeriodUs(float period);

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
    int setDecoderMode(uint32_t mode);
 
    /**
     * Defines if the PWM module should maintain playout ordering of buffers, or always play the most recent buffer provided.
     * 
     * @ param streamingMode If true, buffers will be streamed in order they are received. If false, the most recent buffer supplied always takes precedence.
     * @ param repeatOnEmpty If set to true, the last buffer received will be repeated when no data is available. If false, the PWM channel will be suspended.
     */
    void setStreamingMode(bool streamingMode, bool repeatOnEmpty = true);

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

   /**
    * Method to release the given pin from a peripheral, if already bound.
    * Device drivers should override this method to disconnect themselves from the give pin
    * to allow it to be used by a different peripheral.
    *
    * @param pin the Pin to be released
    */
    virtual int releasePin(Pin &pin) override;

    // Backward compat - deprecated.
    int disconnectPin(Pin &pin);

    private:
    /**
     * Pull a buffer into the given double buffer slot, if one is available.
     */
    int tryPull(uint8_t b);

    /**
     * Sets PWM hardware registers for chained buffers or simple one shot playback
     * 
     * @ param streamingMode If true, buffers will be streamed in order they are received. If false, the most recent buffer supplied always takes precedence.
     */
    void setPwmLoopInten(bool streamingMode);
};
} // namespace codal

#endif
