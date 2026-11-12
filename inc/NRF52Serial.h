#ifndef NRF52_SERIAL_H
#define NRF52_SERIAL_H

#include "Pin.h"
#include "nrf.h"
#include "CodalComponent.h"
#include "CodalConfig.h"
#include "Serial.h"
#include "NRF52SerialBase.h"
#include "hal/nrf_uarte.h"

#ifndef CONFIG_SERIAL_DMA_BUFFER_SIZE
#define CONFIG_SERIAL_DMA_BUFFER_SIZE   32
#endif

namespace codal
{
    /**
     * The serial port on UARTE, driven through EasyDMA.
     *
     * EasyDMA costs one interrupt per buffer instead of one per byte, which
     * keeps the CPU and the SoftDevice happy at high baud rates. The price is
     * that RX progress is only reported at ENDRX: RXDRDY is a single flag, so
     * a delayed interrupt cannot tell how many bytes the DMA has already
     * written. A heap search or a fiber creation masks application interrupts
     * for a long time (see NRF52SerialUART), and the tail of a line then waits
     * in the DMA buffer until the buffer fills.
     *
     * Use NRF52SerialUART for a console, and this class where throughput
     * matters, or for a second port on UARTE1.
     */
    class NRF52Serial : public NRF52SerialBase
    {
        volatile int  bytesProcessed;
#if DEVICE_DMESG_BUFFER_SIZE > 0
        volatile uint32_t txWatchdogCount;
#endif
        uint8_t dmaBuffer[CONFIG_SERIAL_DMA_BUFFER_SIZE];

        NRF_UARTE_Type *p_uarte_;

        /**
         * Ensures all characters have been processed once a DMA buffer is fully received.
         *
         * This function is called upon an ENDRX hardware event, which is raised when a RX DMA buffer
         * has been filled. Normally there is nothing to do, but in the eventuality that an interrupt has been
         * missed (typically due to CPU contention), this function ensures the codal Serial ringbuffer is synchronised
         * and no characters are lost.
         */
        void updateRxBufferAfterENDRX();

        /**
          * Update DMA RX buffer pointer.
          * 
          * UARTE generates an RXSTARTED event once the DMA buffer geometry has been read.
          * This function implements a DMA enabled double buffer within the codal Serial ringbuffer.
        **/
        void updateRxBufferAfterRXSTARTED();

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
        virtual void handleIrq() override;
        virtual void setTxRxPins(Pin& tx, Pin& rx) override;
        virtual int enableInterrupt(SerialInterruptType t) override;
        virtual int disableInterrupt(SerialInterruptType t) override;

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
        virtual int setBaudrate(uint32_t baudrate) override;

        ~NRF52Serial();
    };
}

#endif
