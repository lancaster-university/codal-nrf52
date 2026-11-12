#ifndef NRF52_SERIAL_UART_H
#define NRF52_SERIAL_UART_H

#include "NRF52SerialBase.h"
#include "hal/nrf_uart.h"

namespace codal
{
    /**
     * The serial port on UART0, run without EasyDMA.
     *
     * With DMA the received bytes can only be counted by RXDRDY, a flag
     * that rises once however many bytes arrived while the interrupt
     * waited, so a late interrupt left bytes behind until more came.
     * Without DMA each byte is read from RXD, and RXDRDY rises again for
     * the next one in the six-byte FIFO. UART0 is the only UARTE instance
     * that can run this way, so the port is fixed to it.
     *
     * A late interrupt is the normal case here: the heap and fiber critical
     * sections in codal-core mask application interrupts (BASEPRI, priorities
     * 1-7) so that a long heap search can no longer miss the SoftDevice radio
     * deadline, which is what fixed the BLE panics. This UART interrupt is at
     * priority 1, so a heap search or a fiber creation can hold it off for a
     * long time. UARTE loses the byte count in that window and strands the
     * tail of a line; this driver drains what the FIFO kept.
     *
     * The tradeoff is one interrupt per byte instead of one per DMA buffer,
     * and reception still overflows if interrupts stay masked for longer than
     * six character times (~520us at 115200).
     *
     * UART0 and UARTE0 are one peripheral at one address, so this class and
     * NRF52Serial cannot be used on the same port at the same time. A second
     * port is still available through NRF52Serial on UARTE1.
     */
    class NRF52SerialUART : public NRF52SerialBase
    {
        NRF_UART_Type *p_uart_;

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
        NRF52SerialUART(Pin& tx, Pin& rx);

        virtual int putc(char) override;
        virtual int setBaudrate(uint32_t baudrate) override;

        ~NRF52SerialUART();
    };
}

#endif
