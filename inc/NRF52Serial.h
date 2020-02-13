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
        int tx_byte_buffer;
        int rx_byte_buffer;

        protected:
        virtual int enableInterrupt(SerialInterruptType t) override;
        virtual int disableInterrupt(SerialInterruptType t) override;
        virtual int setBaudrate(uint32_t baudrate) override;
        virtual int configurePins(Pin& tx, Pin& rx) override;

        public:

        nrfx_uarte_t uart_instance;

        /**
         * Constructor
         *
         * @param tx the pin instance to use for transmission
         *
         * @param rx the pin instance to use for reception
         *
         **/
        NRF52Serial(Pin& tx, Pin& rx, NRF_UARTE_Type* uart = NULL);

        int enable();
        int disable();

        virtual int putc(char) override;
        virtual int getc() override;

        ~NRF52Serial();
    };
}

#endif