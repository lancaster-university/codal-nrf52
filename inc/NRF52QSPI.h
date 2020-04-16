#ifndef CODAL_NRF52_QSPI_H
#define CODAL_NRF52_QSPI_H

#include "CodalConfig.h"
#include "codal-core/inc/driver-models/Pin.h"
#include "nrfx_qspi.h"

typedef enum {
    NRF_QSPI_INST_WREN     = 0x06, // Write enable
    NRF_QSPI_INST_RDSR     = 0x05, // Read status register
    NRF_QSPI_INST_WRSR     = 0x01, // Write status register
    NRF_QSPI_INST_FASTREAD = 0x0B, // Read bytes at higher speed
    NRF_QSPI_INST_READ2O   = 0x3B, // Dual-read output
    NRF_QSPI_INST_READ2IO  = 0xBB, // Dual-read input/output
    NRF_QSPI_INST_READ4O   = 0x6B, // Quad-read output
    NRF_QSPI_INST_READ4IO  = 0xEB, // Quad-read input/output
    NRF_QSPI_INST_PP       = 0x02, // Page program
    NRF_QSPI_INST_PP2O     = 0xA2, // Dual-page program output
    NRF_QSPI_INST_PP4O     = 0x32, // Quad-page program output
    NRF_QSPI_INST_PP4IO    = 0x38, // Quad-page program input/output
    NRF_QSPI_INST_SE       = 0x20, // Sector erase
    NRF_QSPI_INST_BE       = 0xD8, // Block erase
    NRF_QSPI_INST_CE       = 0xC7, // Chip erase
    NRF_QSPI_INST_DP       = 0xB9, // Enter deep power-down mode
    NRF_QSPI_INST_DPE      = 0xAB  // Exit deep power-down mode
} NrfQspiInst;

typedef enum {
    QSPI_CFG_BUS_SINGLE,
    QSPI_CFG_BUS_DUAL,
    QSPI_CFG_BUS_QUAD,
} QspiBusWidth;

/** Address size in bits
 */
typedef enum qspi_address_size {
    QSPI_CFG_ADDR_BITS_8,
    QSPI_CFG_ADDR_BITS_16,
    QSPI_CFG_ADDR_BITS_24,
    QSPI_CFG_ADDR_BITS_32,
} QspiAddressBits;



namespace codal
{

/**
 * Class definition for QSPI service, derived from ARM mbed.
 */
class QSPI
{
    Pin &sclk, &ss;
    Pin &io0, &io1, &io2, &io3;

    QspiBusWidth inst_width_;
    QspiBusWidth address_width_;
    QspiAddressBits address_bits_;
    QspiBusWidth alt_bytes_width_;
    uint8_t alt_bytes_size_;
    QspiBusWidth data_width_;
    uint8_t dummy_cycles_;

public:
    //TODO: Pins should be made available selectively. (for 2io-wire, 3io-wire)
    QSPI(Pin &sclk, Pin &ss, Pin &io0, Pin &io1, Pin &io2, Pin &io3);
    // ~QSPI();

    /** Configure the data transmission format
     *  @param inst_width Bus width used by instruction phase(Valid values are QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_DUAL, QSPI_CFG_BUS_QUAD)
     *  @param address_width Bus width used by address phase(Valid values are QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_DUAL, QSPI_CFG_BUS_QUAD)
     *  @param address_bits bits used by address phase(Valid values are QSPI_CFG_ADDR_BITS_8, QSPI_CFG_ADDR_BITS_16, QSPI_CFG_ADDR_BITS_24, QSPI_CFG_ADDR_BITS_32)
     *  @param alt_width Bus width used by alt phase(Valid values are QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_DUAL, QSPI_CFG_BUS_QUAD)
     *  @param alt_size Size in bits used by alt phase (must be a multiple of the number of bus lines indicated in alt_width)
     *  @param data_width Bus width used by data phase(Valid values are QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_DUAL, QSPI_CFG_BUS_QUAD)
     *  @param dummy_cycles Number of dummy clock cycles to be used after alt phase
     */
    virtual int configureFormat(QspiBusWidth inst_width,
                                QspiBusWidth address_width,
                                QspiAddressBits address_bits,
                                QspiBusWidth alt_bytes_width,
                                uint8_t alt_bytes_size,
                                QspiBusWidth data_width,
                                uint8_t dummy_cycles);

    /** Set the frequency of the QSPI interface
     *
     * @param frequency The bus frequency in hertz
     */
    virtual int setFrequency(int hz);

    virtual int setSpiMode(uint8_t mode);

    virtual int read(int addr, uint8_t *rx_buf, size_t length);

    virtual int write(int addr, uint8_t *tx_buf, size_t length);

    virtual int command_transfer(uint8_t inst, int addr, 
        uint8_t* tx_buf, size_t tx_len,
        uint8_t* rx_buf, size_t rx_len);
};



class NRF52QSPI : public QSPI
{
    nrfx_qspi_config_t cfg_;

    bool is_cfg_changed;

public:

    /**
     * Initialize QSPI instance with given pins.
     */
    NRF52QSPI(Pin &sck, Pin &ss, Pin &io0, Pin &io1, Pin &io2, Pin &io3);
    ~NRF52QSPI();

    virtual int configureFormat(QspiBusWidth inst_width,
                                QspiBusWidth address_width,
                                QspiAddressBits address_bits,
                                QspiBusWidth alt_bytes_width,
                                uint8_t alt_bytes_size,
                                QspiBusWidth data_width,
                                uint8_t dummy_cycles) override;

    virtual int setSpiMode(uint8_t mode) override;

    virtual int setFrequency(int hz) override;

    virtual int read(int addr, uint8_t *rx_buf, size_t length) override;

    virtual int write(int addr, uint8_t *tx_buf, size_t length) override;

    /* For only NRF52QSPI */
    virtual int erase4K(int addr, uint16_t cnt);
    virtual int erase64K(int addr, uint16_t cnt);

    virtual int configureWriteFormat(QspiBusWidth address_width, QspiBusWidth data_width);
    virtual int configureReadFormat(QspiBusWidth address_width, QspiBusWidth data_width, uint8_t rx_delay = 2);

    virtual int sendCustomInst(uint8_t inst, 
        uint8_t* tx_buf, uint8_t* rx_buf, size_t tx_rx_len, 
        bool long_frame_loop = false);
};
}

#endif
