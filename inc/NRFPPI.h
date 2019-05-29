#ifndef CODAL_NRFPPI_H
#define CODAL_NRFPPI_H

#include "CodalConfig.h"

// we have way more PPI but this is a start...
#define PPI_CHANNEL_COUNT       10

namespace codal
{
    class NRFPPI
    {
        uint32_t channel_number;

        public:
        NRFPPI(uint32_t channel_number);

        int setEventEndpoint(uint32_t reg);

        int setTaskEndpoint(uint32_t reg);

        int enable();

        int disable();

        ~NRFPPI();
    };

    class NRFPPIFactory
    {
        public:
        static NRFPPI* allocate();

        static int free(NRFPPI*);
    };
}

#endif