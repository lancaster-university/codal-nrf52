#include "NRFPPI.h"
#include "ErrorNo.h"

#include "nrf.h"

using namespace codal;

static NRFPPI* instances[PPI_CHANNEL_COUNT] = { 0 };
static bool ppi_inited = false;

void enable()
{
    if (ppi_inited)
        return;
    ppi_inited = true;
    memset(instances, 0, sizeof(NRFPPI*) * PPI_CHANNEL_COUNT);
}

NRFPPI* NRFPPIFactory::allocate()
{
    for (int i = 0; i < PPI_CHANNEL_COUNT; i++)
    {
        if (instances[i] == NULL)
        {
            NRFPPI* ppi = new NRFPPI(i);
            instances[i] = ppi;
            return ppi;
        }
    }

    return NULL;
}

int NRFPPIFactory::free(NRFPPI* ppi)
{
    for (int i = 0; i < PPI_CHANNEL_COUNT; i++)
    {
        if (instances[i] == ppi)
        {
            delete ppi;
            instances[i] = NULL;
            return DEVICE_OK;
        }
    }

    return DEVICE_INVALID_PARAMETER;
}


NRFPPI::NRFPPI(uint32_t channel_number)
{
    this->channel_number = channel_number;
}

int NRFPPI::setEventEndpoint(uint32_t reg)
{
    NRF_PPI->CH[this->channel_number].EEP = reg;
    return DEVICE_OK;
}

int NRFPPI::setTaskEndpoint(uint32_t reg)
{
    NRF_PPI->CH[this->channel_number].TEP = reg;
    return DEVICE_OK;
}

int NRFPPI::enable()
{
    NRF_PPI->CHEN |= 1 << channel_number;
    return DEVICE_OK;
}

int NRFPPI::disable()
{
    NRF_PPI->CHEN &= ~(1 << channel_number);
    return DEVICE_OK;
}

NRFPPI::~NRFPPI()
{
    disable();
}

