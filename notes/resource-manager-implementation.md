# Implementation of the Resource Manager

## Summary
The idea of the resource manager is to make the handling and distribution of hardware peripherals more flexible.
Rather than designating a specific peripheral to a particular component / feature, components request the peripherals that they require from the resource manager.
When a peripheral is no longer need, it is passed back to the resource manager to be reused elsewhere.

## API proposal
### Resource manager instance
The resource manager needs to be globally accessible and unique, as it will represent the interface for accessing hardware periperals.
To achieve this, the constructor is private, and the resource manager instance exists as a static variable in the source file.
A public function is used to obtain a reference to the resource manager.

Open question: do we need to consider thread safety for the resource manager? CODAL is co-operative concurrency, so initial thought is that it shouldn't need it.

### Acquire / Release mechanism
I imagined the API for the resource manager would work via an acquire / release mechanism, where the caller would provide the configuration for the peripheral it intended to acquire.
The resource manager would find the next available peripheral of that type, configure it based on the arguments it was given and return the driver for that peripheral.

If a peripheral is no longer required, the corresponding release function can be called, which deconfigures the peripheral and makes it available for something else to acquire. Ideally, this function will also invalidate the handle to the peripheral, so it can no longer be accessed by the caller once it has been released.

As each peripheral requires its own specific configuration, it makes sense to have an acquire and release function for each peripheral type - trying to have a singular pair will result in an extremely complicated interface requiring an inherited base class etc.

Initially, I was planning to implement this through pointers: the acquire function would return a pointer to the driver, while the release function would take a pointer to that pointer, allowing the release function to make the pointer NULL and avoid potential use of the driver thereafter.
This does provide some safety - it prevents direct use-after-free of the initial pointer, as the MCU will hardfault if the nullified pointer is accessed, however it doesn't prevent the calling code from making a copy of the pointer which will be left dangling after `release()` is called.

C++ does provide the `std::unique_ptr` type, which has the benefit of preventing copying. Taking the `unique_ptr` by value in the release function guarantees that the variable it was previously stored in is now `nullptr`.

After trying out the `unique_ptr` style I've decided to go back to the raw pointers - the copy protection is not worth littering the code with `std::unique_ptr<TypeName>` everywhere.

### PWM configuration
After some consideration, I decided to include the interrupt priority in the set of configuration arguments for the `pwmAcquire()` function.
Currently, the PWM interrupt priorities are set in `MicroBit::init()`, which works if specific PWMs are designated for specific purposes, but no longer makes sense if PWMs are provided on an "as required" basis.
This does mean that we lose the clear comparison between the priorities in the `MicroBit::init()` function, however this could be addressed by `#define`ing priorities for the various components which use these peripherals within a config header.

### Failure handling
With a finite number of available peripherals, it's possible that a call to `acquire()` could fail.
In this situation, there needs to be some indication to the caller that they didn't get what they asked for.
It's possible to just return `nullptr`, however this doesn't give any insight into the reason for the failure.
As an example, another failure mode might be invalid arguments.
To address this, I've opted to require a reference from the caller to populate with the driver pointer, and for the acquire function to return an enum representing the result of the call.

### Peripheral - IRQ mapping
While implementing the logic for acquiring and releasing PWM peripherals I realised that there is no programmatic link between the peripheral and its IRQ.
In order to make the allocation of peripherals dynamic, it is necessary that a mapping exists to map the correct peripheral to its IRQn.
I've decided to do this on a per-peripheral type basis, so there is a PWM_IRQ_MAP and I will add others for other peripheral types.

### PWM releasing
I've realised that the release of a PWM peripheral is quite convoluted.
The NRF52PWM class routes and enables the interrupt on construction, but it's up to the holder of the driver to disable the interrupt when they're done. They're also responsible for disconnecting any connected pins and disabling the peripheral before deleting the driver.
To make matters more complicated, PWM peripherals can have up to four connected pins, pins can only be disconnected one at a time by passing the name of the pin to the `disconnect()` or `release()` function, and there is a `deleteOnRelease` flag in the class, which if set (done as an argument in `connect()`) immediately deletes the instance.
Disabling the interrupt automatically during the release is trivial, however disconnecting any pins would have to be done by the holder of the driver before they called release.
This could be fixed by adding a `disconnectAllPins` method to the PWM class, however in the interests of not changing too many things at the same time I will leave this up to the caller for now.
