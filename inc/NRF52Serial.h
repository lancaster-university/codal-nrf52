#ifndef NRF52_SERIAL_H
#define NRF52_SERIAL_H

#include "Pin.h"
#include "nrf.h"
#include "CodalComponent.h"
#include "CodalConfig.h"
#include "Serial.h"
#include "hal/nrf_uarte.h"

namespace codal
{
    class NRF52Serial : public Serial
    {
        bool is_tx_in_progress_;

        NRF_UARTE_Type *p_uarte_;
        static void _irqHandler(void *self);

        /**
          * Update DMA RX buffer pointer through ring buffer management.
          * 
          * UARTE generates an event called ENDRX when the specified buffer is full.
          * It is necessary to command STARTRX again after clearing the ENDRX event.
          * This function is to set the start address and size of the ring buffer 
          * that can be received in consideration of the data unread by the user in the ring buffer.
        **/
        void updateRxBufferAfterENDRX();

        /**
          * DMA version of Serial::dataReceviced()
          * 
          * In the A function, only the part of getting data through the getc() function
          * and storing it in the ring buffer is removed.
          * Because DMA stores data directly in the buffer, 
          * this function only manages the ring buffer. (With Event function)
        **/
        void dataReceivedDMA();        

        protected:
        virtual int enableInterrupt(SerialInterruptType t) override;
        virtual int disableInterrupt(SerialInterruptType t) override;
        virtual int setBaudrate(uint32_t baudrate) override;
        virtual int configurePins(Pin& tx, Pin& rx) override;

        public:

        /**
         * Constructor
         *
         * @param tx the pin instance to use for transmission
         *
         * @param rx the pin instance to use for reception
         *
         **/
        NRF52Serial(Pin& tx, Pin& rx, NRF_UARTE_Type* device = NULL);

        virtual int putc(char) override;
        virtual int getc() override;

        ~NRF52Serial();
    };
}

#endif