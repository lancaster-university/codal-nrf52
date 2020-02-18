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

        /**
          * Sends a buffer of known length over the serial line.
          * 
          * Unlike the Serial.send () function, 
          * this function does not transfer 1 byte unit 
          * and directly connects the parameter buffer pointer to DMA.
          * Used when fast transfer is required.
          *
          * @param buffer a pointer to the first character of the buffer
          *
          * @param len the number of bytes that are safely available to read.
          *
          * @param mode the selected mode, one of: ASYNC, SYNC_SPINWAIT, SYNC_SLEEP. Each mode
          *        gives a different behaviour:
          *
          *            ASYNC - bytes are copied into the txBuff and returns immediately.
          *
          *            SYNC_SPINWAIT - bytes are copied into the txBuff and this method
          *                            will spin (lock up the processor) until all bytes
          *                            have been sent.
          *
          *            SYNC_SLEEP - bytes are copied into the txBuff and the fiber sleeps
          *                         until all bytes have been sent. This allows other fibers
          *                         to continue execution.
          *
          *         Defaults to SYNC_SLEEP.
          *
          * @return the number of bytes written, CODAL_SERIAL_IN_USE if another fiber
          *         is using the serial instance for transmission, DEVICE_INVALID_PARAMETER
          *         if buffer is invalid, or the given bufferLen is <= 0.
         **/
        int write(uint8_t *buffer, int bufferLen, SerialMode mode = DEVICE_DEFAULT_SERIAL_MODE);

        ~NRF52Serial();
    };
}

#endif