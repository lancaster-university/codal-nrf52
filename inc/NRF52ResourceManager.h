/*
The MIT License (MIT)

Copyright (c) 2026 Lancaster University & Micro:bit Educational Foundation.

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

#ifndef NRF52_RESOURCE_MANAGER
#define NRF52_RESOURCE_MANAGER

#include "NRF52PWM.h"

namespace codal
{
/*
 * Enumeration of the available interrupt priorities
 * When SoftDevice is enabled, only priorities 2, 3, 4, 5 and 7 are available.
 * Each of these available priorities are mapped to a variant of this enum.
 */
enum InterruptPriority
{
    HIGHEST = 2,
    HIGH = 3,
    MEDIUM = 4,
    LOW = 5,
    LOWEST = 7
};

enum ResourceResult
{
    OK,
    INVALID_ARGUMENTS,
    NO_PERIPHERAL_AVAILABLE,
};

class NRF52ResourceManager : public CodalComponent
{
private:
    /*
     * Private constructor: The resource manager is instantiated as a static and accessed through
     * the public `resourceManager()` function
     */
    NRF52ResourceManager();

    static NRF52ResourceManager _resourceManager;

public:
    /*
     * The Resource Manager manages access to the hardware peripherals, making sure that each
     * peripheral is only being used by one component at a time, and allowing dynamic reallocation
     * of peripherals when they're no longer required.
     * The resource manager exists as a static variable.
     * This function provides access to it through a reference.
     *
     * @return reference to the Resource Manager instance
     */
    NRF52ResourceManager &resourceManager();

    /*
     * Request a PWM driver from the Resource Manager
     *
     * @param[out] pwm        Pointer to be populated with driver if available
     * @param[in]  priority   The interrupt priority for this PWM's interrupt
     * @param[in]  source     The DataSource which will provide data to the PWM
     * @param[in]  sampleRate The frequency (in Hz) at which data will be sampled
     * @param[in]  id         Device id used to identify events from this device on the message bus
     *
     * @return Enum representing the result of the acquisition request
     */
    ResourceResult pwmAcquire(NRF52PWM *&pwm, InterruptPriority const priority,
                              DataSource const &source,
                              float const sampleRate = NRF52PWM_DEFAULT_FREQUENCY,
                              uint16_t const id = DEVICE_ID_SYSTEM_DAC);

    /*
     * Release a PWM driver back to the Resource Manager to be reused.
     * This function will set `pwm` to NULL so the driver can no longer be used
     *
     * @param[in,out] pwm The driver to be released
     */
    void pwmRelease(NRF52PWM *&pwm);
};
} // namespace codal

#endif // #ifndef NRF52_RESOURCE_MANAGER
