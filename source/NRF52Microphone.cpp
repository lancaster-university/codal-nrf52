/*
The MIT License (MIT)

Copyright (c) 2016 Lancaster University.

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
#include "NRF52Microphone.h"
#include "nrf.h"
#include "cmsis.h"

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
static NRF52Microphone *nrf52_microphone_driver = NULL;

void nrf52_adc_irq(void)
{
    // Simply pass on to the driver component handler.
    if (nrf52_microphone_driver)
        nrf52_microphone_driver->irq();
}


/**
 * Update our reference to a downstream component.
 * Pass through any connect requests to our output buffer component.
 *
 * @param component The new downstream component for this microhone audio source.
 */
void NRF52Microphone::connect(DataSink& component)
{
    output.connect(component);
}

/**
 * Constructor for an instance of a microhone input,
 *
 * @param pin The pin the microhone data input is connected to.
 * @param sampleRate the rate at which samples are generated in the output buffer (in Hz)
 * @param id The id to use for the message bus when transmitting events.
 */
NRF52Microphone::NRF52Microphone(Pin &pin, int sampleRate, uint16_t id) : inputPin(pin), output(*this)
{
    // Store our configuration data.
    this->id = id;

    // Record a handle to this driver object, for use by the IRQ handler.
    nrf52_microphone_driver = this;

    // Ensure the SAADC module is disabled.
    NVIC_DisableIRQ(SAADC_IRQn);
    this->disable();

    // Define our output stream as non-blocking.
    output.setBlocking(false);

    // Set samplerate
    this->setSampleRate(8*sampleRate);

    // Ensure SAADC module is disabled (typically needed for determinstic configuration)
    NRF_SAADC->ENABLE = 0;

    // Enable 14 bit sampling (although it's rather nicely delivered as a 16 bit sample).
    NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_14bit << SAADC_RESOLUTION_VAL_Pos);
    NRF_SAADC->OVERSAMPLE = 3;

    NRF_SAADC->INTENSET = (SAADC_INTENSET_STARTED_Enabled << SAADC_INTENSET_STARTED_Pos) |
        (SAADC_INTENSET_END_Enabled << SAADC_INTENSET_END_Pos) |
        (SAADC_INTENSET_STOPPED_Enabled << SAADC_INTENSET_STOPPED_Pos );

    NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) |
        (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
        (SAADC_CH_CONFIG_GAIN_Gain1_2 << SAADC_CH_CONFIG_GAIN_Pos) |
        (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
        (SAADC_CH_CONFIG_TACQ_5us << SAADC_CH_CONFIG_TACQ_Pos) |
        (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos );

    // Enable moderately high priority interrupt
    NVIC_SetVector(SAADC_IRQn, (uint32_t)nrf52_adc_irq);
    NVIC_SetPriority(SAADC_IRQn, 0);
    NVIC_ClearPendingIRQ(SAADC_IRQn);
    NVIC_EnableIRQ(SAADC_IRQn);
}

/**
 * Provide the next available ManagedBuffer to our downstream caller, if available.
 */
ManagedBuffer NRF52Microphone::pull()
{
    return outputBuffer;
}

void NRF52Microphone::irq()
{
    if (NRF_SAADC->EVENTS_STARTED)
    {
        // We've just started receiving data the inputBuffer.
        // So we don't miss any data, line up the next buffer.
        // (unless we've been asked to stop).

        if (enabled)
            startDMA();

        NRF_SAADC->EVENTS_STARTED = 0;
    }

    if (NRF_SAADC->EVENTS_END)
    {
        if (outputBuffer.length() > 0)
            output.pullRequest();

        NRF_SAADC->TASKS_START = 1;

        NRF_SAADC->EVENTS_END = 0;
    }

    if (NRF_SAADC->EVENTS_STOPPED)
    {
        NRF_SAADC->EVENTS_STOPPED = 0;
    }
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
 * Default is defined by NRF52_MICROPHONE_DEFAULT_GAIN.
 */
int NRF52Microphone::setGain(int gain)
{
    uint32_t r = NRF_SAADC->CH[0].CONFIG & 0xfffff8ff;

    if (gain < 0 || gain > 7)
        return DEVICE_INVALID_PARAMETER;

    this->gain = gain;

    // mask off the currnet gain setting.
    r |= (gain << 8);

    // Update settings in the SAADC.
    NRF_SAADC->CH[0].CONFIG = r;

    return DEVICE_OK;
}

/**
 * Define the sample rate to use.
 *
 * @param sampleRate - The sample rate to use, in Hz.
 */
int NRF52Microphone::setSampleRate(int sampleRate)
{
    // The NRF52 ADC uses an internal 16MHz clock, with an 11 bit CC counter (why 11? who knows!)
    // Calculate the nearest sample rate we can manage.
    uint32_t cc = 16000000 / sampleRate;

    // Reject invalid sample rates.
    if (cc < 80 || cc > 2047)
    {
        DMESG("NRF52Microphone: INVALID SAMPLE RATE\n");
        return DEVICE_INVALID_PARAMETER;
    }

    // Calculate the actual sample rate we managed, and store it.
    this->sampleRate = 16000000 / cc;

    // Indicate that we want to run from the internal timer, not an external task signal.
    cc |= 1 << SAADC_SAMPLERATE_MODE_Pos;

    // Configure the ADC.
    NRF_SAADC->SAMPLERATE = cc;

    return DEVICE_OK;
}

/**
 * Enable this component
 */
void NRF52Microphone::enable()
{
    if (!enabled && nrf52_saadc_id.hasKey(inputPin.name))
    {
        // Lookup the analog ID for the given pin.
        enabled = true;

        NRF_SAADC->CH[0].PSELP = nrf52_saadc_id.get(inputPin.name);
        NRF_SAADC->CH[0].PSELN = 0;

        startDMA();

        NRF_SAADC->RESULT.MAXCNT = NRF52_MICROPHONE_BUFFER_SIZE / 2;
        NRF_SAADC->ENABLE = 1;
        NRF_SAADC->TASKS_START = 1;
        NRF_SAADC->TASKS_SAMPLE = 1;
    }
    else
    {
        DMESG("NRF52Microphone: Illegal pin [%d]\n", inputPin.name);
    }
}

/**
 * Disable this component
 */
void NRF52Microphone::disable()
{
    // Schedule all DMA transfers to stop after the next DMA transaction completes.
    enabled = false;
    NRF_SAADC->TASKS_STOP = 1;
    NRF_SAADC->ENABLE = 0;
}

/**
 * Initiate a DMA transfer into the raw data buffer.
 */
void NRF52Microphone::startDMA()
{
    outputBuffer = inputBuffer;
    inputBuffer = ManagedBuffer(NRF52_MICROPHONE_BUFFER_SIZE);

    NRF_SAADC->RESULT.PTR = (uint32_t)&inputBuffer[0];
}
