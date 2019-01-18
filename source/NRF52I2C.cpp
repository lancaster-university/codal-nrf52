#include "NRF52I2C.h"
#include "codal_target_hal.h"
#include "CodalDmesg.h"
#include "hal/nrf_twim.h"

using namespace codal;

#define MAX_I2C_RETRIES 2 // TODO?

#define THE_TWIM NRF_TWIM1
#define THE_IRQ SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn
#define THE_HANDLER SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler

/**
 * Constructor.
 */
NRF52I2C::NRF52I2C(NRF52Pin &sda, NRF52Pin &scl) : codal::I2C(sda, scl), sda(sda), scl(scl)
{
    // put pins in input mode
    sda.getDigitalValue(PullMode::Up);
    scl.getDigitalValue(PullMode::Up);

    target_wait_us(10);

    nrf_twim_pins_set(THE_TWIM, scl.name, sda.name);
    nrf_twim_frequency_set(THE_TWIM, NRF_TWIM_FREQ_100K);
    nrf_twim_enable(THE_TWIM);

    target_wait_us(10);
}

/** Set the frequency of the I2C interface
 *
 * @param frequency The bus frequency in hertz
 */
int NRF52I2C::setFrequency(uint32_t frequency)
{
    auto freq = NRF_TWIM_FREQ_100K;
    if (frequency >= 250000)
        freq = NRF_TWIM_FREQ_250K;
    if (frequency >= 400000)
        freq = NRF_TWIM_FREQ_400K;
    nrf_twim_disable(THE_TWIM);
    nrf_twim_frequency_set(THE_TWIM, freq);
    nrf_twim_enable(THE_TWIM);

    DMESG("I2C FREQ: %d", frequency);
    return DEVICE_OK;
}

int NRF52I2C::waitForStop(int evt)
{
    int res = DEVICE_OK;
    while (!nrf_twim_event_check(THE_TWIM, (nrf_twim_event_t)evt))
    {
        if (nrf_twim_event_check(THE_TWIM, NRF_TWIM_EVENT_ERROR))
        {
            auto err = THE_TWIM->ERRORSRC;
            THE_TWIM->ERRORSRC = err;
            DMESG("I2C err %x", err); // 2 is address NACK

            nrf_twim_event_clear(THE_TWIM, NRF_TWIM_EVENT_ERROR);
            nrf_twim_task_trigger(THE_TWIM, NRF_TWIM_TASK_RESUME);
            nrf_twim_task_trigger(THE_TWIM, NRF_TWIM_TASK_STOP);
            evt = NRF_TWIM_EVENT_STOPPED;
            res = DEVICE_I2C_ERROR;
        }
    }
    return res;
}

/**
 * Issues a standard, I2C command write to the I2C bus.
 * This consists of:
 *  - Asserting a Start condition on the bus
 *  - Selecting the Slave address (as an 8 bit address)
 *  - Writing a number of raw data bytes provided
 *  - Asserting a Stop condition on the bus
 *
 * The CPU will busy wait until the transmission is complete.
 *
 * @param address The 8bit I2C address of the device to write to
 * @param data pointer to the bytes to write
 * @param len the number of bytes to write
 * @param repeated Suppresses the generation of a STOP condition if set. Default: false;
 *
 * @return DEVICE_OK on success, DEVICE_I2C_ERROR if the the write request failed.
 */
int NRF52I2C::write(uint16_t address, uint8_t *data, int len, bool repeated)
{
    address = address >> 1;

    nrf_twim_address_set(THE_TWIM, address);

    nrf_twim_event_clear(THE_TWIM, NRF_TWIM_EVENT_STOPPED);
    nrf_twim_event_clear(THE_TWIM, NRF_TWIM_EVENT_ERROR);

    nrf_twim_tx_buffer_set(THE_TWIM, data, len);

    if (repeated)
    {
        nrf_twim_shorts_set(THE_TWIM, NRF_TWIM_SHORT_LASTTX_SUSPEND_MASK);
        nrf_twim_event_clear(THE_TWIM, NRF_TWIM_EVENT_SUSPENDED);
    }
    else
        nrf_twim_shorts_set(THE_TWIM, NRF_TWIM_SHORT_LASTTX_STOP_MASK);

    nrf_twim_task_trigger(THE_TWIM, NRF_TWIM_TASK_RESUME);
    nrf_twim_task_trigger(THE_TWIM, NRF_TWIM_TASK_STARTTX);

    return waitForStop(repeated ? NRF_TWIM_EVENT_SUSPENDED : NRF_TWIM_EVENT_STOPPED);
}

/**
 * Issues a standard, 2 byte I2C command read to the I2C bus.
 * This consists of:
 *  - Asserting a Start condition on the bus
 *  - Selecting the Slave address (as an 8 bit address)
 *  - reading "len" bytes of raw 8 bit data into the buffer provided
 *  - Asserting a Stop condition on the bus
 *
 * The CPU will busy wait until the transmission is complete.
 *
 * @param address The 8bit I2C address of the device to read from
 * @param data pointer to store the the bytes read
 * @param len the number of bytes to read into the buffer
 * @param repeated Suppresses the generation of a STOP condition if set. Default: false;
 *
 * @return DEVICE_OK on success, DEVICE_I2C_ERROR if the the read request failed.
 */
int NRF52I2C::read(uint16_t address, uint8_t *data, int len, bool repeated)
{
    address = address >> 1;

    nrf_twim_address_set(THE_TWIM, address);

    nrf_twim_event_clear(THE_TWIM, NRF_TWIM_EVENT_STOPPED);
    nrf_twim_event_clear(THE_TWIM, NRF_TWIM_EVENT_ERROR);

    nrf_twim_rx_buffer_set(THE_TWIM, data, len);

    if (!repeated)
        nrf_twim_shorts_set(THE_TWIM, NRF_TWIM_SHORT_LASTRX_STOP_MASK);

    nrf_twim_task_trigger(THE_TWIM, NRF_TWIM_TASK_RESUME);
    nrf_twim_task_trigger(THE_TWIM, NRF_TWIM_TASK_STARTRX);

    if (!repeated)
    {
        return waitForStop(NRF_TWIM_EVENT_STOPPED);
    }
    else
    {
        int r = waitForStop(NRF_TWIM_EVENT_LASTRX);
        if (r != DEVICE_OK)
            return r;
        nrf_twim_task_trigger(THE_TWIM, NRF_TWIM_TASK_SUSPEND);
        return waitForStop(NRF_TWIM_EVENT_SUSPENDED);
    }
}

/**
 * Performs a typical register read operation to the I2C slave device provided.
 * This consists of:
 *  - Asserting a Start condition on the bus
 *  - Selecting the Slave address (as an 8 bit address, I2C WRITE)
 *  - Selecting a RAM register address in the slave
 *  - Asserting a Stop condition on the bus
 *  - Asserting a Start condition on the bus
 *  - Selecting the Slave address (as an 8 bit address, I2C READ)
 *  - Performing an 8 bit read operation (of the requested register)
 *  - Asserting a Stop condition on the bus
 *
 * The CPU will busy wait until the transmission is complete..
 *
 * @param address 8bit I2C address of the device to read from
 * @param reg The 8bit register address of the to read.
 * @param data A pointer to a memory location to store the result of the read operation
 * @param length The number of mytes to read
 * @param repeated Use a repeated START/START/STOP transaction if true, or independent
 * START/STOP/START/STOP transactions if fasle. Default: true
 *
 * @return DEVICE_OK or DEVICE_I2C_ERROR if the the read request failed.
 */
int NRF52I2C::readRegister(uint16_t address, uint8_t reg, uint8_t *data, int length, bool repeated)
{
    // write followed by a read...
    int ret = write(address, &reg, 1, repeated);

    if (ret)
        return ret;

    ret = read(address, data, length, false);
    return ret;
}
