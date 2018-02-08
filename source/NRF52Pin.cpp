/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

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

/**
  * Class definition for Pin.
  *
  * Commonly represents an I/O pin on the edge connector.
  */
#include "NRF52Pin.h"
#include "ErrorNo.h"
#include "nrf52.h"
#include "nrf52_bitfields.h"

using namespace codal;

/**
  * Constructor.
  * Create a Pin instance, generally used to represent a pin on the edge connector.
  *
  * @param id the unique EventModel id of this component.
  *
  * @param name the mbed PinName for this Pin instance.
  *
  * @param capability the capabilities this Pin instance should have.
  *                   (PIN_CAPABILITY_DIGITAL, PIN_CAPABILITY_ANALOG, PIN_CAPABILITY_AD, PIN_CAPABILITY_ALL)
  *
  * @code
  * Pin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_ALL);
  * @endcode
  */
NRF52Pin::NRF52Pin(int id, PinName name, PinCapability capability) : codal::_mbed::Pin(id, name, capability)
{
}

/**
  * Disconnect any attached mBed IO from this pin.
  *
  * Used only when pin changes mode (i.e. Input/Output/Analog/Digital)
  */
void NRF52Pin::disconnect()
{
    // Unlike the generic mbed implementation, we don't hold state for basic GPIO operaitons.
    // So for these we just need to remove our internal flag of the mode we're operating in.
    status &= ~IO_STATUS_DIGITAL_IN;
    status &= ~IO_STATUS_DIGITAL_OUT;

    // Now call our parent implementation to handle the other cases.
    Pin::disconnect();
}

/**
  * Configures this IO pin as a digital output (if necessary) and sets the pin to 'value'.
  *
  * @param value 0 (LO) or 1 (HI)
  *
  * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or DEVICE_NOT_SUPPORTED
  *         if the given pin does not have digital capability.
  *
  * @code
  * Pin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
  * P0.setDigitalValue(1); // P0 is now HI
  * @endcode
  */
int NRF52Pin::setDigitalValue(int value)
{
    // Check if this pin has a digital mode...
    if(!(PIN_CAPABILITY_DIGITAL & capability))
        return DEVICE_NOT_SUPPORTED;

    // Move into a Digital output state if necessary.
    if (!(status & IO_STATUS_DIGITAL_OUT)){
        disconnect();

        // Enable output mode.
        NRF_GPIO->DIRSET = 1 << name;

        // Record our mode, so we can optimise later.
        status |= IO_STATUS_DIGITAL_OUT;
    }

    // Write the value.
    if (value)
        NRF_GPIO->OUTSET = 1 << name;
    else
        NRF_GPIO->OUTCLR = 1 << name;

    return DEVICE_OK;
}

/**
  * Configures this IO pin as a digital input (if necessary) and tests its current value.
  *
  *
  * @return 1 if this input is high, 0 if input is LO, or DEVICE_NOT_SUPPORTED
  *         if the given pin does not have digital capability.
  *
  * @code
  * Pin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
  * P0.getDigitalValue(); // P0 is either 0 or 1;
  * @endcode
  */
int NRF52Pin::getDigitalValue()
{
    //check if this pin has a digital mode...
    if(!(PIN_CAPABILITY_DIGITAL & capability))
        return DEVICE_NOT_SUPPORTED;

    if(status & (IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE))
        return _mbed::Pin::getDigitalValue();

    if (!(status & IO_STATUS_DIGITAL_IN ))
    {
        disconnect();

        // Enable input mode, and input buffer
        NRF_GPIO->PIN_CNF[name] &= 0xfffffffc;

        // Record our mode, so we can optimise later.
        status |= IO_STATUS_DIGITAL_IN;
    }

    // return the current state of the pin
    return (NRF_GPIO->IN & (1 << name)) ? 1 : 0;
}

/**
  * Configures the pull of this pin.
  *
  * @param pull one of the mbed pull configurations: PullUp, PullDown, PullNone
  *
  * @return DEVICE_NOT_SUPPORTED if the current pin configurationdoes not support the requested mode,
  * otherwise DEVICE_OK.
  */
int NRF52Pin::setPull(PullMode pull)
{
    pullMode = pull;

    uint32_t s = NRF_GPIO->PIN_CNF[name] & 0xfffffff3;

    if (pull == PullMode::Down)
        s |= 0x00000004;

    if (pull == PullMode::Up)
        s |= 0x0000000c;

    NRF_GPIO->PIN_CNF[name] = s;

    return DEVICE_OK;
}

/**
  * Configures the pull of this pin.
  *
  * @param pull one of the mbed pull configurations: PullUp, PullDown, PullNone
  *
  * @return MICROBIT_NOT_SUPPORTED if the current pin configuration is anything other
  *         than a digital input, otherwise MICROBIT_OK.
  */
int NRF52Pin::setPull(PinMode pull)
{
    if (pull == PullUp)
        return this->setPull(PullMode::Up);

    if (pull == PullDown)
        return this->setPull(PullMode::Down);

    if (pull == PullNone)
        return this->setPull(PullMode::None);

    return DEVICE_OK;
}

/**
 * Configures this IO pin as a high drive pin (capable of sourcing/sinking greater current).
 * By default, pins are STANDARD drive.
 *
 * @param value true to enable HIGH DRIVE on this pin, false otherwise
 */
int NRF52Pin::setHighDrive(bool value)
{
    uint32_t s = NRF_GPIO->PIN_CNF[name] & 0xfffff8ff;

    if (value)
        s |= 0x00000300;

    NRF_GPIO->PIN_CNF[name] = s;

    return DEVICE_OK;
}

/**
 * Determines if this IO pin is a high drive pin (capable of sourcing/sinking greater current).
 * By default, pins are STANDARD drive.
 *
 * @return true if HIGH DRIVE is enabled on this pin, false otherwise
 */
bool NRF52Pin::getHighDrive()
{
    uint32_t s = NRF_GPIO->PIN_CNF[name] & 0x00000700;

    return (s == 0x00000300);
}

