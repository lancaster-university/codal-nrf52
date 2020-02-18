#ifndef NRF52_SERIAL_H
#define NRF52_SERIAL_H

#include "Pin.h"
#include "nrf.h"
#include "CodalComponent.h"
#include "CodalConfig.h"
#include "Serial.h"
#include "nrfx_uarte.h"

namespace codal
{
    class NRF52Serial : public Serial
    {
        protected:
        virtual int enableInterrupt(SerialInterruptType t) override;
        virtual int disableInterrupt(SerialInterruptType t) override;
        virtual int setBaudrate(uint32_t baudrate) override;
        virtual int configurePins(Pin& tx, Pin& rx) override;

        public:

        uint8_t rx_byte_buf_;
        nrfx_uarte_t uart_instance;

        /**
         * Constructor
         *
         * @param tx the pin instance to use for transmission
         *
         * @param rx the pin instance to use for reception
         *
         **/
        NRF52Serial(Pin& tx, Pin& rx);

        int enable();
        int disable();

        virtual int putc(char) override;
        virtual int getc() override;

        int write(uint8_t *buffer, int bufferLen, SerialMode mode = DEVICE_DEFAULT_SERIAL_MODE);

        ~NRF52Serial();
    };
}

#endif