/*
The MIT License (MIT)

Copyright (c) 2017 Lancaster University.

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
#include "Button.h"
#include "Timer.h"
#include "ErrorNo.h"
#include "nrf.h"
#include "EventModel.h"
#include "CodalDmesg.h"
#include "codal_target_hal.h"


using namespace codal;

#ifdef NRF_P1
#define PORT (name < 32 ? NRF_P0 : NRF_P1)
#define PIN ((name) & 31)
#define NUM_PINS 48
#else
#define PORT (NRF_P0)
#define PIN (name)
#define NUM_PINS 32
#endif

volatile uint32_t interrupt_enable = 0;

static NRF52Pin *irq_pins[NUM_PINS];

MemorySource* NRF52Pin::pwmSource = NULL;
NRF52PWM* NRF52Pin::pwm = NULL;
uint16_t NRF52Pin::pwmBuffer[NRF52PIN_PWM_CHANNEL_MAP_SIZE] = {0,0,0,0};
int8_t NRF52Pin::pwmChannelMap[NRF52PIN_PWM_CHANNEL_MAP_SIZE] = {-1,-1,-1,-1};
uint8_t NRF52Pin::lastUsedChannel = 3;

NRF52ADC* NRF52Pin::adc = NULL;
TouchSensor* NRF52Pin::touchSensor = NULL;

#ifdef __cplusplus
extern "C" {
#endif
void GPIOTE_IRQHandler(void)
{
    if (NRF_GPIOTE->EVENTS_PORT)
    {
        NRF_GPIOTE->EVENTS_PORT = 0;
        for (uint8_t i = 0; i <= 31; i++)
        {
            if (interrupt_enable & (1 << i) && irq_pins[i] && NRF_P0->LATCH & (1 << i))
            {
                uint32_t currCnf = NRF_P0->PIN_CNF[i];
                uint8_t currSense = (currCnf & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos;
                
                // hi: latch indicates a state change... determine if we were looking for hi or lo.
                if (currSense == GPIO_PIN_CNF_SENSE_High)
                {
                    // swap!
                    NRF_P0->PIN_CNF[i] = (currCnf & ~GPIO_PIN_CNF_SENSE_Msk) | (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
                    irq_pins[i]->rise();
                }
                else
                {
                    // swap!
                    NRF_P0->PIN_CNF[i] = (currCnf & ~GPIO_PIN_CNF_SENSE_Msk) | (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
                    irq_pins[i]->fall();
                }
            }
        }
        // make sure to clear everything
        NRF_P0->LATCH = 0xffffffff;
        NRF_P1->LATCH = 0xffffffff;
    }
}

#ifdef __cplusplus
}
#endif

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
NRF52Pin::NRF52Pin(int id, PinNumber name, PinCapability capability) : codal::Pin(id, name, capability)
{
    this->pullMode = DEVICE_DEFAULT_PULLMODE;
    CODAL_ASSERT(name < NUM_PINS, 50);
    irq_pins[name] = this;

    // Power up in a disconnected, low power state.
    // If we're unused, this is how it will stay...
    this->status = 0x00;

    this->obj = NULL;

    NRF_GPIOTE->INTENSET    = GPIOTE_INTENSET_PORT_Set << GPIOTE_INTENSET_PORT_Pos;
    NVIC_SetPriority(GPIOTE_IRQn, 1);
    NVIC_EnableIRQ  (GPIOTE_IRQn);
}

/**
  * Disconnect any attached mBed IO from this pin.
  *
  * Used only when pin changes mode (i.e. Input/Output/Analog/Digital)
  */
void NRF52Pin::disconnect()
{
    if (status & IO_STATUS_ANALOG_IN)
    {

    }

    if (status & IO_STATUS_TOUCH_IN)
    {
        if (obj)
            delete ((TouchButton*)obj);
    }

    if (status & (IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE | IO_STATUS_INTERRUPT_ON_EDGE))
    {
        // disconnect pin cng
        PORT->PIN_CNF[PIN] &= ~(GPIO_PIN_CNF_SENSE_Msk);
        interrupt_enable &= ~(1 << this->name);

        // if (obj)
        //     delete ((PinTimeStruct*)obj);
    }

    status = 0;
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

    // Write the value, before setting as output - this way the pin state update will be atomic    
    if (value)
        PORT->OUTSET = 1 << PIN;
    else
        PORT->OUTCLR = 1 << PIN;


    // Move into a Digital output state if necessary.
    if (!(status & IO_STATUS_DIGITAL_OUT))
    {
        disconnect();

        uint32_t cnf = PORT->PIN_CNF[PIN];
        
        // output
        cnf |= 1;

        // high drive
        // cnf |= (3 << 8);
        // cnf &= ~(1 << 10);

        PORT->PIN_CNF[PIN] = cnf;

        // Record our mode, so we can optimise later.
        status |= IO_STATUS_DIGITAL_OUT;
    }

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

    if (!(status & (IO_STATUS_DIGITAL_IN | IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE | IO_STATUS_INTERRUPT_ON_EDGE)))
    {
        disconnect();

        // Enable input mode, and input buffer
        PORT->PIN_CNF[PIN] &= 0xfffffffc;

        // Record our mode, so we can optimise later.
        status |= IO_STATUS_DIGITAL_IN;
    }

    // return the current state of the pin
    return (PORT->IN & (1 << PIN)) ? 1 : 0;
}

/**
 * Configures this IO pin as a digital input with the specified internal pull-up/pull-down configuraiton (if necessary) and tests its current value.
 *
 * @param pull one of the mbed pull configurations: PullUp, PullDown, PullNone
 *
 * @return 1 if this input is high, 0 if input is LO, or DEVICE_NOT_SUPPORTED
 *         if the given pin does not have digital capability.
 *
 * @code
 * Pin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
 * P0.getDigitalValue(PullUp); // P0 is either 0 or 1;
 * @endcode
 */
int NRF52Pin::getDigitalValue(PullMode pull)
{
    setPull(pull);
    return getDigitalValue();
}
/**
 * Instantiates the components required for PWM if not previously created
 */
int NRF52Pin::initialisePWM()
{
    if (pwmSource == NULL)
        pwmSource = new MemorySource(65535);

    if (pwm == NULL)
        pwm = new NRF52PWM(NRF_PWM0, pwmSource->output, 1600);

    return DEVICE_OK;
}

/**
  * Configures this IO pin as an analog/pwm output, and change the output value to the given level.
  *
  * @param value the level to set on the output pin, in the range 0 - 1024
  *
  * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or DEVICE_NOT_SUPPORTED
  *         if the given pin does not have analog capability.
  */
int NRF52Pin::setAnalogValue(int value)
{
    // //check if this pin has an analogue mode...
     if(!(PIN_CAPABILITY_ANALOG & capability))
         return DEVICE_NOT_SUPPORTED;
    
    // //sanitise the level value
    if(value < 0 || value > DEVICE_PIN_MAX_OUTPUT)
         return DEVICE_INVALID_PARAMETER;

    int channel = -1;

    // find existing channel
    for (int i = 0; i < NRF52PIN_PWM_CHANNEL_MAP_SIZE; i++)
        if (pwmChannelMap[i] == name)
            channel = i;

    // no existing channel found
    if (channel == -1)
    {
        initialisePWM();

        // alloc new channel by round robin allocation
        channel = (lastUsedChannel + 1) % NRF52PIN_PWM_CHANNEL_MAP_SIZE;
        pwmChannelMap[channel] = name;
        lastUsedChannel = channel;
        pwm->connectPin(*this, channel);
    }

    status |= IO_STATUS_ANALOG_OUT;

    // set new value
    pwmBuffer[channel] = (int)((float)pwm->getSampleRange() * (1 - (float)value / (float)DEVICE_PIN_MAX_OUTPUT));

    pwmSource->play(pwmBuffer, NRF52PIN_PWM_CHANNEL_MAP_SIZE, 0);

    return DEVICE_OK;
}

/**
  * Configures this IO pin as an analog/pwm output (if necessary) and configures the period to be 20ms,
  * with a duty cycle between 500 us and 2500 us.
  *
  * A value of 180 sets the duty cycle to be 2500us, and a value of 0 sets the duty cycle to be 500us by default.
  *
  * This range can be modified to fine tune, and also tolerate different servos.
  *
  * @param value the level to set on the output pin, in the range 0 - 180.
  *
  * @param range which gives the span of possible values the i.e. the lower and upper bounds (center +/- range/2). Defaults to DEVICE_PIN_DEFAULT_SERVO_RANGE.
  *
  * @param center the center point from which to calculate the lower and upper bounds. Defaults to DEVICE_PIN_DEFAULT_SERVO_CENTER
  *
  * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or DEVICE_NOT_SUPPORTED
  *         if the given pin does not have analog capability.
  */
int NRF52Pin::setServoValue(int value, int range, int center)
{
    //check if this pin has an analogue mode...
    if(!(PIN_CAPABILITY_ANALOG & capability))
        return DEVICE_NOT_SUPPORTED;

    //sanitise the servo level
    if(value < 0 || range < 1 || center < 1)
        return DEVICE_INVALID_PARAMETER;

    //clip - just in case
    if(value > DEVICE_PIN_MAX_SERVO_RANGE)
        value = DEVICE_PIN_MAX_SERVO_RANGE;

    //calculate the lower bound based on the midpoint
    int lower = (center - (range / 2)) * 1000;

    value = value * 1000;

    //add the percentage of the range based on the value between 0 and 180
    int scaled = lower + (range * (value / DEVICE_PIN_MAX_SERVO_RANGE));

    return setServoPulseUs(scaled / 1000);
}

/**
  * Configures this IO pin as an analogue input (if necessary), and samples the Pin for its analog value.
  *
  * @return the current analogue level on the pin, in the range 0 - 1024, or
  *         DEVICE_NOT_SUPPORTED if the given pin does not have analog capability.
  *
  * @code
  * Pin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
  * P0.getAnalogValue(); // P0 is a value in the range of 0 - 1024
  * @endcode
  */
int NRF52Pin::getAnalogValue()
{

    // //check if this pin has an analogue mode...
    if(!(PIN_CAPABILITY_ANALOG & capability))
        return DEVICE_NOT_SUPPORTED;

    // Move into an analogue input state if necessary.
    if (!(status & IO_STATUS_ANALOG_IN))
    {
         disconnect();
         status |= IO_STATUS_ANALOG_IN;
    }

    if (adc)
    {
        NRF52ADCChannel *c = adc->getChannel(*this);

        if (c)
            return c->getSample() / 16;
    }

    return DEVICE_NOT_SUPPORTED;
}

/**
  * Determines if this IO pin is currently configured as an input.
  *
  * @return 1 if pin is an analog or digital input, 0 otherwise.
  */
int NRF52Pin::isInput()
{
    return (status & (IO_STATUS_DIGITAL_IN | IO_STATUS_ANALOG_IN)) == 0 ? 0 : 1;
}

/**
  * Determines if this IO pin is currently configured as an output.
  *
  * @return 1 if pin is an analog or digital output, 0 otherwise.
  */
int NRF52Pin::isOutput()
{
    return (PORT->DIR & (1 << PIN)) != 0 || (status & (IO_STATUS_DIGITAL_OUT | IO_STATUS_ANALOG_OUT)) != 0;
}

/**
  * Determines if this IO pin is currently configured for digital use.
  *
  * @return 1 if pin is digital, 0 otherwise.
  */
int NRF52Pin::isDigital()
{
    return (status & (IO_STATUS_DIGITAL_IN | IO_STATUS_DIGITAL_OUT)) == 0 ? 0 : 1;
}

/**
  * Determines if this IO pin is currently configured for analog use.
  *
  * @return 1 if pin is analog, 0 otherwise.
  */
int NRF52Pin::isAnalog()
{
    return (status & (IO_STATUS_ANALOG_IN | IO_STATUS_ANALOG_OUT)) == 0 ? 0 : 1;
}

/**
  * Configures this IO pin as a "makey makey" style touch sensor (if necessary)
  * and tests its current debounced state.
  *
  * Users can also subscribe to Button events generated from this pin.
  *
  * @return 1 if pin is touched, 0 if not, or DEVICE_NOT_SUPPORTED if this pin does not support touch capability.
  *
  * @code
  * DeviceMessageBus bus;
  *
  * Pin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_ALL);
  * if(P0.isTouched())
  * {
  *     //do something!
  * }
  *
  * // subscribe to events generated by this pin!
  * bus.listen(DEVICE_ID_IO_P0, DEVICE_BUTTON_EVT_CLICK, someFunction);
  * @endcode
  */
int NRF52Pin::isTouched()
{
    // //check if this pin has a touch mode...
    if(!(PIN_CAPABILITY_DIGITAL & capability))
        return DEVICE_NOT_SUPPORTED;

    // Move into a touch input state if necessary.
    if (!(status & IO_STATUS_TOUCH_IN)){
        disconnect();
        obj = new TouchButton(*this, *touchSensor);
        status |= IO_STATUS_TOUCH_IN;
    }

    return ((TouchButton *)obj)->isPressed();
}

/**
  * Configures this IO pin as an analog/pwm output if it isn't already, configures the period to be 20ms,
  * and sets the pulse width, based on the value it is given.
  *
  * @param pulseWidth the desired pulse width in microseconds.
  *
  * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or DEVICE_NOT_SUPPORTED
  *         if the given pin does not have analog capability.
  */
int NRF52Pin::setServoPulseUs(int pulseWidth)
{
    initialisePWM();

    if (pwm->getPeriodUs() != 20000)
        pwm->setPeriodUs(20000);
 
    return setAnalogPeriodUs(pulseWidth);
}

/**
  * Configures the PWM period of the analog output to the given value.
  *
  * @param period The new period for the analog output in microseconds.
  *
  * @return DEVICE_OK on success.
  */
int NRF52Pin::setAnalogPeriodUs(int period)
{
    if (status & IO_STATUS_ANALOG_OUT)
    {
        int oldRange = pwm->getSampleRange();
        pwm->setPeriodUs(period);

        for (int i = 0; i < NRF52PIN_PWM_CHANNEL_MAP_SIZE; i++)
        {
            float v = (float) pwmBuffer[i];
            v = v * (float)pwm->getSampleRange();
            v = v / (float) oldRange;
             
            pwmBuffer[i] = (uint16_t) v;
        }

        pwmSource->setMaximumSampleValue(pwm->getSampleRange());
        pwmSource->play(pwmBuffer, NRF52PIN_PWM_CHANNEL_MAP_SIZE, 0);

        return DEVICE_OK;
    }
    
    return DEVICE_NOT_SUPPORTED;
}

/**
  * Configures the PWM period of the analog output to the given value.
  *
  * @param period The new period for the analog output in milliseconds.
  *
  * @return DEVICE_OK on success, or DEVICE_NOT_SUPPORTED if the
  *         given pin is not configured as an analog output.
  */
int NRF52Pin::setAnalogPeriod(int period)
{
    return setAnalogPeriodUs(period*1000);
}

/**
  * Obtains the PWM period of the analog output in microseconds.
  *
  * @return the period on success, or DEVICE_NOT_SUPPORTED if the
  *         given pin is not configured as an analog output.
  */
uint32_t NRF52Pin::getAnalogPeriodUs()
{
    if (status & IO_STATUS_ANALOG_OUT)
        return pwm->getPeriodUs();

    return DEVICE_NOT_SUPPORTED;
}

/**
  * Obtains the PWM period of the analog output in milliseconds.
  *
  * @return the period on success, or DEVICE_NOT_SUPPORTED if the
  *         given pin is not configured as an analog output.
  */
int NRF52Pin::getAnalogPeriod()
{
    return getAnalogPeriodUs()/1000;
}

/**
  * Configures the pull of this pin.
  *
  * @param pull one of the mbed pull configurations: PullUp, PullDown, PullNone
  *
  * @return DEVICE_NOT_SUPPORTED if the current pin configuration is anything other
  *         than a digital input, otherwise DEVICE_OK.
  */
int NRF52Pin::setPull(PullMode pull)
{
    pullMode = pull;

    uint32_t s = PORT->PIN_CNF[PIN] & 0xfffffff3;

    if (pull == PullMode::Down)
        s |= 0x00000004;

    if (pull == PullMode::Up)
        s |= 0x0000000c;


    PORT->PIN_CNF[PIN] = s;

    return DEVICE_OK;
}

/**
  * This member function manages the calculation of the timestamp of a pulse detected
  * on a pin whilst in IO_STATUS_EVENT_PULSE_ON_EDGE or IO_STATUS_EVENT_ON_EDGE modes.
  *
  * @param eventValue the event value to distribute onto the message bus.
  */
void NRF52Pin::pulseWidthEvent(uint16_t eventValue)
{
    Event evt(id, eventValue, CREATE_ONLY);
    uint64_t now = evt.timestamp;
    uint64_t previous = ((PinTimeStruct *)obj)->last_time;

    if (previous != 0)
    {
        evt.timestamp -= previous;
        evt.fire();
    }

    ((PinTimeStruct *)obj)->last_time = now;
}

void NRF52Pin::rise()
{
    if(status & IO_STATUS_EVENT_PULSE_ON_EDGE)
        pulseWidthEvent(DEVICE_PIN_EVT_PULSE_LO);

    if(status & IO_STATUS_EVENT_ON_EDGE)
        Event(id, DEVICE_PIN_EVT_RISE, 0);

    if (status & IO_STATUS_INTERRUPT_ON_EDGE && gpio_irq)
        this->gpio_irq(1);
}

void NRF52Pin::fall()
{
    if(status & IO_STATUS_EVENT_PULSE_ON_EDGE)
        pulseWidthEvent(DEVICE_PIN_EVT_PULSE_HI);

    if(status & IO_STATUS_EVENT_ON_EDGE)
        Event(id, DEVICE_PIN_EVT_FALL, 0);

    if (status & IO_STATUS_INTERRUPT_ON_EDGE && gpio_irq)
        this->gpio_irq(0);
}

/**
  * This member function will construct an TimedInterruptIn instance, and configure
  * interrupts for rise and fall.
  *
  * @param eventType the specific mode used in interrupt context to determine how an
  *                  edge/rise is processed.
  *
  * @return DEVICE_OK on success
  */
int NRF52Pin::enableRiseFallEvents(int eventType)
{
    // if we are in neither of the two modes, configure pin as a TimedInterruptIn.
    if (!(status & (IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE | IO_STATUS_INTERRUPT_ON_EDGE)))
    {
        int v = getDigitalValue(pullMode);

        if (!this->obj)
            this->obj = new PinTimeStruct;

        ((PinTimeStruct*)obj)->last_time = 0;

        // PORT->DETECTMODE = 1; // latched-detect

        PORT->PIN_CNF[PIN] &= ~(GPIO_PIN_CNF_SENSE_Msk);
        if (v)
            PORT->PIN_CNF[PIN] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
        else
            PORT->PIN_CNF[PIN] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);

        PORT->LATCH = 1 << PIN; // clear any pending latch

        // configure as interrupt in
        interrupt_enable |= (1 << this->name);
    }

    status &= ~(IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE | IO_STATUS_INTERRUPT_ON_EDGE);

    // set our status bits accordingly.
    if(eventType == DEVICE_PIN_EVENT_ON_EDGE)
        status |= IO_STATUS_EVENT_ON_EDGE;
    else if(eventType == DEVICE_PIN_EVENT_ON_PULSE)
        status |= IO_STATUS_EVENT_PULSE_ON_EDGE;
    else if(eventType == DEVICE_PIN_INTERRUPT_ON_EDGE)
        status |= IO_STATUS_INTERRUPT_ON_EDGE;

    return DEVICE_OK;
}

/**
  * If this pin is in a mode where the pin is generating events, it will destruct
  * the current instance attached to this Pin instance.
  *
  * @return DEVICE_OK on success.
  */
int NRF52Pin::disableEvents()
{
    if (status & (IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE | IO_STATUS_TOUCH_IN | IO_STATUS_INTERRUPT_ON_EDGE))
        disconnect();

    return DEVICE_OK;
}

/**
  * Configures the events generated by this Pin instance.
  *
  * DEVICE_PIN_EVENT_ON_EDGE - Configures this pin to a digital input, and generates events whenever a rise/fall is detected on this pin. (DEVICE_PIN_EVT_RISE, DEVICE_PIN_EVT_FALL)
  * DEVICE_PIN_EVENT_ON_PULSE - Configures this pin to a digital input, and generates events where the timestamp is the duration that this pin was either HI or LO. (DEVICE_PIN_EVT_PULSE_HI, DEVICE_PIN_EVT_PULSE_LO)
  * DEVICE_PIN_EVENT_ON_TOUCH - Configures this pin as a makey makey style touch sensor, in the form of a Button. Normal button events will be generated using the ID of this pin.
  * DEVICE_PIN_EVENT_NONE - Disables events for this pin.
  *
  * @param eventType One of: DEVICE_PIN_EVENT_ON_EDGE, DEVICE_PIN_EVENT_ON_PULSE, DEVICE_PIN_EVENT_ON_TOUCH, DEVICE_PIN_EVENT_NONE
  *
  * @code
  * DeviceMessageBus bus;
  *
  * Pin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
  * P0.eventOn(DEVICE_PIN_EVENT_ON_PULSE);
  *
  * void onPulse(Event evt)
  * {
  *     int duration = evt.timestamp;
  * }
  *
  * bus.listen(DEVICE_ID_IO_P0, DEVICE_PIN_EVT_PULSE_HI, onPulse, MESSAGE_BUS_LISTENER_IMMEDIATE)
  * @endcode
  *
  * @return DEVICE_OK on success, or DEVICE_INVALID_PARAMETER if the given eventype does not match
  *
  * @note In the DEVICE_PIN_EVENT_ON_PULSE mode, the smallest pulse that was reliably detected was 85us, around 5khz. If more precision is required,
  *       please use the InterruptIn class supplied by ARM mbed.
  */
int NRF52Pin::eventOn(int eventType)
{
    switch(eventType)
    {
        case DEVICE_PIN_INTERRUPT_ON_EDGE:
        case DEVICE_PIN_EVENT_ON_EDGE:
        case DEVICE_PIN_EVENT_ON_PULSE:
            enableRiseFallEvents(eventType);
            break;

        case DEVICE_PIN_EVENT_ON_TOUCH:
            isTouched();
            break;

        case DEVICE_PIN_EVENT_NONE:
            disableEvents();
            break;

        default:
            return DEVICE_INVALID_PARAMETER;
    }

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
    uint32_t s = PORT->PIN_CNF[PIN] & 0xfffff8ff;

    if (value)
        s |= 0x00000300;

    PORT->PIN_CNF[PIN] = s;

    return DEVICE_OK;
}

/**
 * Determines if this IO pin is a high drive pin (capable of sourcing/sinking greater current).
 * By default, pins are STANDARD drive.
 *
 * @return true if HIGH DRIVE is enabled on this pin, false otherwise
 */
bool NRF52Pin::isHighDrive()
{
    uint32_t s = PORT->PIN_CNF[PIN] & 0x00000700;

    return (s == 0x00000300);
}

__attribute__((noinline))
static void get_and_set(NRF_GPIO_Type *port, uint32_t mask) {
    // 0 -> 1, only set when IN==0
    port->DIRSET = ~port->IN & mask;
}

__attribute__((noinline))
static void get_and_clr(NRF_GPIO_Type *port, uint32_t mask) {
    // 1 -> 0, only set when IN==1
    port->DIRSET = port->IN & mask;
}

int NRF52Pin::getAndSetDigitalValue(int value)
{
    uint32_t mask = 1 << PIN;

    if ((PORT->DIR & mask) == 0)
    {
        // set the value
        if (value)
            PORT->OUTSET = mask;
        else
            PORT->OUTCLR = mask;

        // pin in input mode, do the "atomic" set
        if (value)
            get_and_set(PORT, mask);
        else
            get_and_clr(PORT, mask);

        if (PORT->DIR & mask) {
            disconnect();
            setDigitalValue(value); // make sure 'status' is updated
            return 0;
        } else {

            return DEVICE_BUSY;
        }
    }

    return 0;
}
