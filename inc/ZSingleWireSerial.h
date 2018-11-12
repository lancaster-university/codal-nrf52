#ifndef ZSINGLE_WIRE_SERIAL_H
#define ZSINGLE_WIRE_SERIAL_H

#include "Pin.h"
#include "CodalComponent.h"
#include "CodalConfig.h"
#include "SingleWireSerial.h"
#include "JACDAC.h"
#include "MemberFunctionCallback.h"

#define SINGLE_WIRE_SERIAL_EVT_RX_FULL      1
#define SINGLE_WIRE_SERIAL_EVT_TX_EMPTY     2020        // using shared notify id, hopefully no one else uses this...

namespace codal
{

    class ZSingleWireSerial : public DMASingleWireSerial
    {
        uint8_t* buf;
        uint16_t bufLen;

        protected:
        virtual int configureTx(int);
        virtual int configureRx(int);

        public:

        virtual void configureRxInterrupt(int enable);
        virtual void configureTxInterrupt(int enable);

        static ZSingleWireSerial* instance;

        ZSingleWireSerial(Pin& p);

        virtual int putc(char c);
        virtual int getc();

        virtual int send(uint8_t* data, int len);
        virtual int receive(uint8_t* data, int len);

        virtual int sendDMA(uint8_t* data, int len);
        virtual int receiveDMA(uint8_t* data, int len);
        virtual int abortDMA();

        virtual int setBaud(uint32_t baud);
        virtual uint32_t getBaud();

        virtual int sendBreak();
    };
}

#endif