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

#ifndef NRF52_PIN_H
#define NRF52_PIN_H

#include "MbedPin.h"
#include "CodalConfig.h"
#include "mbed.h"

/**
  * Class definition for Pin.
  *
  * Commonly represents an I/O pin on the edge connector.
  */
namespace codal
{
    class NRF52Pin : public _mbed::Pin
    {
        /**
         * Disconnect any attached mBed IO from this pin.
         *
         * Used only when pin changes mode (i.e. Input/Output/Analog/Digital)
         */
        virtual void disconnect();

        public:

        /**
         * Constructor.
         * Create a NRF52Pin instance, generally used to represent a pin.
         *
         * @param id the unique id of this component.
         * @param name the PinName for this NRF52Pin instance.
         * @param capability the capabilities this DevicePin instance should have.
         *                   (PIN_CAPABILITY_DIGITAL, PIN_CAPABILITY_ANALOG, PIN_CAPABILITY_AD, PIN_CAPABILITY_ALL)
         *
         * @code
         * NRF52Pin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_ALL);
         * @endcode
         */
        NRF52Pin(int id, PinName name, PinCapability capability);

        /**
         * Configures this IO pin as a digital output (if necessary) and sets the pin to 'value'.
         *
         * @param value 0 (LO) or 1 (HI)
         *
         * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or DEVICE_NOT_SUPPORTED
         *         if the given pin does not have digital capability.
         */
        virtual int setDigitalValue(int value);

        /**
         * Configures this IO pin as a digital input (if necessary) and tests its current value.
         *
         * @return 1 if this input is high, 0 if input is LO, or DEVICE_NOT_SUPPORTED
         *         if the given pin does not have digital capability.
         */
        virtual int getDigitalValue();

        /**
         * Configures the pull of this pin.
         *
         * @param pull one of the pull configurations: PullUp, PullDown, PullNone
         *
         * @return DEVICE_NOT_SUPPORTED if the current pin configuration is anything other
         *         than a digital input, otherwise DEVICE_OK.
         */
        virtual int setPull(PullMode pull);
        int setPull(PinMode pull);

        /**
         * Configures this IO pin as a high drive pin (capable of sourcing/sinking greater current).
         * By default, pins are STANDARD drive.
         *
         * @param value true to enable HIGH DRIVE on this pin, false otherwise
         */
        virtual int setHighDrive(bool value);

        /**
         * Determines if this IO pin is a high drive pin (capable of sourcing/sinking greater current).
         * By default, pins are STANDARD drive.
         *
         * @return true if HIGH DRIVE is enabled on this pin, false otherwise
         */
        virtual bool getHighDrive();
    };
}


#endif
