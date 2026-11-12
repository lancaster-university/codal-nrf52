#ifndef NRF52_SERIAL_BASE_H
#define NRF52_SERIAL_BASE_H

#include "Pin.h"
#include "nrf.h"
#include "CodalComponent.h"
#include "CodalConfig.h"
#include "Serial.h"
#include "peripheral_alloc.h"

namespace codal
{
    /**
     * Shared machinery for the nRF52 serial drivers.
     *
     * Takes care of peripheral allocation, the NVIC hookup, pin reconfiguration,
     * reading a byte from the codal ring buffer and the low power transitions.
     * The hardware specific parts - interrupt handling, interrupt masks, the byte
     * output path and the peripheral teardown - live in the derived classes.
     */
    class NRF52SerialBase : public Serial
    {
        protected:

        void *p_peri_;
        volatile bool is_tx_in_progress_;

        /**
         * Constructor
         *
         * @param tx the pin instance to use for transmission
         *
         * @param rx the pin instance to use for reception
         *
         * @param device the peripheral instance to use, or NULL to take the next free one
         *
         * @param mode the peripheral mode to allocate when device is NULL
         *
         **/
        NRF52SerialBase(Pin& tx, Pin& rx, void *device, PeripheralMode mode,
                        uint8_t rxBufferSize = CODAL_SERIAL_DEFAULT_BUFFER_SIZE,
                        uint8_t txBufferSize = CODAL_SERIAL_DEFAULT_BUFFER_SIZE,
                        uint16_t id = DEVICE_ID_SERIAL);

        /**
         * Route the shared peripheral interrupt vector to this instance and
         * raise its NVIC priority above the application default.
         */
        void initIrq();

        /**
         * Detach this instance from the shared peripheral interrupt vector.
         */
        void disableIrq();

        static void _irqHandler(void *self);

        /**
         * Handles the peripheral events. Called from the shared IRQ trampoline.
         */
        virtual void handleIrq() = 0;

        /**
         * Connects TX and RX to the given pins.
         */
        virtual void setTxRxPins(Pin& tx, Pin& rx) = 0;

        virtual int configurePins(Pin& tx, Pin& rx) override;

        public:

        virtual int getc() override;

        /**
         * Puts the component in (or out of) sleep (low power) mode.
         *
         * If CODAL_SERIAL_STATUS_DEEPSLEEP is set, then the peripheral will remain active during deep
         * sleep, and will wake the processor if new data is received.
         */
        virtual int setSleep(bool doSleep) override;

        virtual ~NRF52SerialBase();
    };
}

#endif
