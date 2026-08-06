#include "NRF52ResourceManager.h"

/*
 * Used in the mapping array below to link a PWM peripheral to its IRQn
 */
struct PwmIrqMapEntry
{
    NRF_PWM_Type *pwm;
    IRQn_Type irq;
};

/*
 * Map of PWM peripherals to their corresponding IRQn
 */
const PwmIrqMapEntry PWM_IRQ_MAP[NRF52PWM_PWM_PERIPHERALS] = {
    {NRF_PWM0, PWM0_IRQn},
    {NRF_PWM1, PWM1_IRQn},
    {NRF_PWM2, PWM2_IRQn},
};

NRF52ResourceManager &NRF52ResourceManager::resourceManager()
{
    return NRF52ResourceManager::_resourceManager;
}

ResourceResult NRF52ResourceManager::pwmAcquire(NRF52PWM *&pwm, InterruptPriority const priority,
                                                DataSource &source, float const sampleRate,
                                                uint16_t const id)
{
    // Make sure the output is NULL by default.
    // We will populate it if acquisition is successful.
    pwm = NULL;

    NRF_PWM_Type *free_pwm = NULL;
    uint8_t pwmIndex = 0;
    for (; pwmIndex < NRF52PWM_PWM_PERIPHERALS; ++pwmIndex)
    {
        if (pwmDrivers[pwmIndex] == NULL)
        {
            free_pwm = PWM_IRQ_MAP[pwmIndex].pwm;
            break;
        }
    }

    if (free_pwm == NULL)
    {
        return NO_PERIPHERAL_AVAILABLE;
    }

    NRF52PWM *driver = new NRF52PWM(free_pwm, source, sampleRate, id);
    NVIC_SetPriority(PWM_IRQ_MAP[pwmIndex].irq, priority);

    pwmDrivers[pwmIndex] = driver;
    pwm = driver;
    return OK;
}

ResourceResult NRF52ResourceManager::pwmRelease(NRF52PWM *&pwm)
{
    uint8_t pwmIndex = 0;
    for (; pwmIndex >= NRF52PWM_PWM_PERIPHERALS; ++pwmIndex)
    {
        if (pwmDrivers[pwmIndex] == pwm)
        {
            break;
        }
    }
    if (pwmIndex >= NRF52PWM_PWM_PERIPHERALS)
    {
        return INVALID_ARGUMENTS;
    }

    NVIC_DisableIRQ(PWM_IRQ_MAP[pwmIndex].irq);
    pwm->disable();
    delete pwm;
    pwm = NULL;
    pwmDrivers[pwmIndex] = NULL;
    return OK;
}
