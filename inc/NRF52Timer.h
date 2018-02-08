#ifndef NRF52_TIMER_H
#define NRF52_TIMER_H

#include "codal-core/inc/types/Event.h"
#include "codal-core/inc/driver-models/Timer.h"
#include "mbed.h"

namespace codal
{
    class NRF52Timer : public codal::Timer
    {

        public:

            uint32_t    period;
            uint32_t    sigma;

            /**
             * Constructor for an instance of Timer1.
             *
             * @param id The id to use for the message bus when transmitting events.
             */
            NRF52Timer();

            /**
             * request to the physical timer implementation code to provide a trigger callback at the given time.
             * note: it is perfectly legitimate for the implementation to trigger before this time if convenient.
             * @param t Indication that t time units (typically microsends) have elapsed.
             */
            virtual void triggerIn(CODAL_TIMESTAMP t);

            /**
             * request to the physical timer implementation code to trigger immediately.
             */
            virtual void syncRequest();
    };
}

#endif
