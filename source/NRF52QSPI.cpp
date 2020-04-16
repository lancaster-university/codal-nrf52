#include "CodalConfig.h"
#include "NRF52QSPI.h"
#include "ErrorNo.h"
#include "CodalDmesg.h"
#include "codal-core/inc/types/Event.h"
#include "CodalFiber.h"
#include "peripheral_alloc.h"

namespace codal
{
QSPI::QSPI(Pin &sclk, Pin &ss, Pin &io0, Pin &io1, Pin &io2, Pin &io3)
    : sclk(sclk), ss(ss), io0(io0), io1(io1), io2(io2), io3(io3)
{}

int QSPI::configureFormat(QspiBusWidth inst_width,
                          QspiBusWidth address_width,
                          QspiAddressBits address_bits,
                          QspiBusWidth alt_bytes_width,
                          uint8_t alt_bytes_size,
                          QspiBusWidth data_width,
                          uint8_t dummy_cycles)
{
    (void) inst_width, (void) address_width, (void) address_bits,
    (void) alt_bytes_width, (void) alt_bytes_size, (void) data_width,
    (void) dummy_cycles;

    return DEVICE_NOT_IMPLEMENTED;
}

int QSPI::setFrequency(int hz)
{
    (void) hz;

    return DEVICE_NOT_IMPLEMENTED;
}

int QSPI::setSpiMode(uint8_t mode)
{
    (void) mode;

    return DEVICE_NOT_IMPLEMENTED;
}

int QSPI::read(int addr, uint8_t *rx_buf, size_t length)
{
    (void) addr, (void) rx_buf, (void) length;

    return DEVICE_NOT_IMPLEMENTED;
}

int QSPI::write(int addr, uint8_t *tx_buf, size_t length)
{
    (void) addr, (void) tx_buf, (void) length;

    return DEVICE_NOT_IMPLEMENTED;
}

int QSPI::command_transfer(uint8_t inst, int addr, 
                           uint8_t* tx_buf, size_t tx_len,
                           uint8_t* rx_buf, size_t rx_len)
{
    (void) inst, (void) addr, (void) tx_buf,
    (void) tx_len, (void) rx_buf, (void) rx_len;

    return DEVICE_NOT_IMPLEMENTED;
}




/**
 * Constructor.
 */
NRF52QSPI::NRF52QSPI(Pin &sclk, Pin &ss, Pin &io0, Pin &io1, Pin &io2, Pin &io3)
    : QSPI(sclk, ss, io0, io1, io2, io3)
{
    cfg_.phy_if.sck_freq = NRF_QSPI_FREQ_32MDIV1;

    cfg_.pins.sck_pin = (uint32_t)sclk.name;
    cfg_.pins.csn_pin = (uint32_t)ss.name;
    cfg_.pins.io0_pin = (uint32_t)io0.name;
    cfg_.pins.io1_pin = (uint32_t)io1.name;
    cfg_.pins.io2_pin = (uint32_t)io2.name;
    cfg_.pins.io3_pin = (uint32_t)io3.name;

    cfg_.irq_priority = 0;

    setSpiMode(0);
    setFrequency(32000000);
    cfg_.phy_if.sck_freq  = NRF_QSPI_FREQ_32MDIV1;
    cfg_.phy_if.sck_delay = 5;
    cfg_.phy_if.dpmen     = false;

    cfg_.prot_if.addrmode  = NRF_QSPI_ADDRMODE_24BIT;
    cfg_.prot_if.dpmconfig = false;

    is_cfg_changed = true;

    configureFormat(QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_SINGLE, 
        QSPI_CFG_ADDR_BITS_24, QSPI_CFG_BUS_DUAL, 
        8, QSPI_CFG_BUS_DUAL, 0);
}


NRF52QSPI::~NRF52QSPI()
{
    nrfx_qspi_uninit();
}


int NRF52QSPI::configureFormat(QspiBusWidth inst_width,
                            QspiBusWidth address_width,
                            QspiAddressBits address_bits,
                            QspiBusWidth alt_bytes_width,
                            uint8_t alt_bytes_size,
                            QspiBusWidth data_width,
                            uint8_t dummy_cycles)
{
    // (inst width) - (addr width) - (data width)
    // 1-1-1
    if (inst_width == QSPI_CFG_BUS_SINGLE &&
        address_width == QSPI_CFG_BUS_SINGLE &&
        data_width == QSPI_CFG_BUS_SINGLE) {
        if(cfg_.prot_if.writeoc != NRF_QSPI_WRITEOC_PP){
            cfg_.prot_if.writeoc = NRF_QSPI_WRITEOC_PP;
            is_cfg_changed = true;
        }else if(cfg_.prot_if.readoc != NRF_QSPI_READOC_FASTREAD){
            cfg_.prot_if.readoc = NRF_QSPI_READOC_FASTREAD;
            is_cfg_changed = true;
        }
    // 1-1-4
    } else if (inst_width == QSPI_CFG_BUS_SINGLE &&
        address_width == QSPI_CFG_BUS_SINGLE &&
        data_width == QSPI_CFG_BUS_QUAD) {
        if(cfg_.prot_if.writeoc != NRF_QSPI_WRITEOC_PP4O){
            cfg_.prot_if.writeoc = NRF_QSPI_WRITEOC_PP4O;
            is_cfg_changed = true;
        }else if(cfg_.prot_if.readoc != NRF_QSPI_READOC_READ4O){
            cfg_.prot_if.readoc = NRF_QSPI_READOC_READ4O;
            is_cfg_changed = true;
        }
    // 1-4-4
    } else if (inst_width == QSPI_CFG_BUS_SINGLE &&
        address_width == QSPI_CFG_BUS_QUAD &&
        data_width == QSPI_CFG_BUS_QUAD) {
        if(cfg_.prot_if.writeoc != NRF_QSPI_WRITEOC_PP4IO){
            cfg_.prot_if.writeoc = NRF_QSPI_WRITEOC_PP4IO;
            is_cfg_changed = true;
        }else if(cfg_.prot_if.readoc != NRF_QSPI_READOC_READ4IO){
            cfg_.prot_if.readoc = NRF_QSPI_READOC_READ4IO;
            is_cfg_changed = true;
        }
    // 1-1-2
    } else if (inst_width == QSPI_CFG_BUS_SINGLE &&
        address_width == QSPI_CFG_BUS_SINGLE &&
        data_width == QSPI_CFG_BUS_DUAL) {
        if(cfg_.prot_if.writeoc != NRF_QSPI_WRITEOC_PP2O){
            cfg_.prot_if.writeoc = NRF_QSPI_WRITEOC_PP2O;
            is_cfg_changed = true;
        }else if(cfg_.prot_if.readoc != NRF_QSPI_READOC_READ2O){
            cfg_.prot_if.readoc = NRF_QSPI_READOC_READ2O;
            is_cfg_changed = true;
        }
    // 1-2-2
    } else if (inst_width == QSPI_CFG_BUS_SINGLE &&
        address_width == QSPI_CFG_BUS_DUAL &&
        data_width == QSPI_CFG_BUS_DUAL) {
        // 1-2-2 write is not supported
        if(cfg_.prot_if.readoc != NRF_QSPI_READOC_READ2IO){
            cfg_.prot_if.readoc = NRF_QSPI_READOC_READ2IO;
            is_cfg_changed = true;
        }
    } else {
        return DEVICE_INVALID_PARAMETER;
    }
    
    // supporting only 24 or 32 bit address
    if (address_bits == QSPI_CFG_ADDR_BITS_24) {
        if(cfg_.prot_if.addrmode != NRF_QSPI_ADDRMODE_24BIT){
            cfg_.prot_if.addrmode = NRF_QSPI_ADDRMODE_24BIT;
            is_cfg_changed = true;
        }
    } else if (address_bits == QSPI_CFG_ADDR_BITS_32) {
        if(cfg_.prot_if.addrmode != NRF_QSPI_ADDRMODE_32BIT){
            cfg_.prot_if.addrmode = NRF_QSPI_ADDRMODE_32BIT;
            is_cfg_changed = true;
        }        
    } else {
        return DEVICE_INVALID_PARAMETER;
    }
    
    //Configure QSPI with new command format
    if(is_cfg_changed){
        nrfx_qspi_uninit();
        nrfx_err_t ret_status = nrfx_qspi_init(&cfg_, NULL, NULL);
        if (ret_status != NRFX_SUCCESS ) {
            if (ret_status == NRFX_ERROR_INVALID_PARAM) {
                return DEVICE_INVALID_PARAMETER;
            } else {
                return DEVICE_SPI_ERROR;
            }
        }
        is_cfg_changed = false;
    }

    return DEVICE_OK;
}


int NRF52QSPI::setSpiMode(uint8_t mode)
{
    cfg_.phy_if.spi_mode = mode == 0 ? NRF_QSPI_MODE_0 : NRF_QSPI_MODE_1;

    is_cfg_changed = true;

    return DEVICE_OK;
}


int NRF52QSPI::setFrequency(int hz)
{
    nrf_qspi_frequency_t freq = NRF_QSPI_FREQ_32MDIV16;

    // Convert hz to closest NRF frequency divider
    if (hz < 2130000)
        freq = NRF_QSPI_FREQ_32MDIV16; // 2.0 MHz, minimum supported frequency
    else if (hz < 2290000)
        freq = NRF_QSPI_FREQ_32MDIV15; // 2.13 MHz
    else if (hz < 2460000)
        freq = NRF_QSPI_FREQ_32MDIV14; // 2.29 MHz
    else if (hz < 2660000)
        freq = NRF_QSPI_FREQ_32MDIV13; // 2.46 Mhz
    else if (hz < 2900000)
        freq = NRF_QSPI_FREQ_32MDIV12; // 2.66 MHz
    else if (hz < 3200000)
        freq = NRF_QSPI_FREQ_32MDIV11; // 2.9 MHz
    else if (hz < 3550000)
        freq = NRF_QSPI_FREQ_32MDIV10; // 3.2 MHz
    else if (hz < 4000000)
        freq = NRF_QSPI_FREQ_32MDIV9; // 3.55 MHz
    else if (hz < 4570000)
        freq = NRF_QSPI_FREQ_32MDIV8; // 4.0 MHz
    else if (hz < 5330000)
        freq = NRF_QSPI_FREQ_32MDIV7; // 4.57 MHz
    else if (hz < 6400000)
        freq = NRF_QSPI_FREQ_32MDIV6; // 5.33 MHz
    else if (hz < 8000000)
        freq = NRF_QSPI_FREQ_32MDIV5; // 6.4 MHz
    else if (hz < 10600000)
        freq = NRF_QSPI_FREQ_32MDIV4; // 8.0 MHz
    else if (hz < 16000000)
        freq = NRF_QSPI_FREQ_32MDIV3; // 10.6 MHz
    else if (hz < 32000000)
        freq = NRF_QSPI_FREQ_32MDIV2; // 16 MHz
    else
        freq = NRF_QSPI_FREQ_32MDIV1; // 32 MHz

    cfg_.phy_if.sck_freq = freq;

    is_cfg_changed = true;

    return DEVICE_OK;
}


int NRF52QSPI::read(int addr, uint8_t *rx_buf, size_t length)
{
    // flash address and buffer length must be word unit
    if ((length & 0x03) || (addr & 0x03)) {
        return DEVICE_INVALID_PARAMETER;
    }

    if(nrfx_qspi_read(rx_buf, length, addr) != NRFX_SUCCESS){
        return DEVICE_SPI_ERROR;
    }

    return DEVICE_OK;
}

int NRF52QSPI::write(int addr, uint8_t *tx_buf, size_t length)
{
    // flash address and buffer length must be divisible by 4
    if ((length & 0x03) || (addr & 0x03)) {
        return DEVICE_INVALID_PARAMETER;
    }
	
    // write here does not return how much it transfered, we return transfered all
    if (nrfx_qspi_write(tx_buf, length, addr) != NRFX_SUCCESS ) {
        return DEVICE_SPI_ERROR;
    }
    return DEVICE_OK;
}


/* For only NRF52QSPI */
int NRF52QSPI::erase4K(int addr, uint16_t cnt)
{
    uint32_t remain_block = cnt;
    uint32_t block_addr = addr;
    nrfx_err_t status;
    int ret = DEVICE_OK;

    while(remain_block)
    {
        status = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, block_addr);
        if(status == NRFX_SUCCESS){
            remain_block--;
            block_addr += 4096;
        }else if(status == NRFX_ERROR_INVALID_ADDR){
            ret = DEVICE_INVALID_PARAMETER;
            break;
        }
    }

    return ret;
}


int NRF52QSPI::erase64K(int addr, uint16_t cnt)
{
    uint32_t remain_block = cnt;
    uint32_t block_addr = addr;
    nrfx_err_t status;
    int ret = DEVICE_OK;    

    while(remain_block)
    {
        status = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_64KB, block_addr);
        if(status == NRFX_SUCCESS){
            remain_block--;
            block_addr += sizeof(uint16_t);
        }else if(status == NRFX_ERROR_INVALID_ADDR){
            ret = DEVICE_INVALID_PARAMETER;
            break;
        }
    }
    return ret;
}


int NRF52QSPI::configureWriteFormat(QspiBusWidth address_width, QspiBusWidth data_width)
{
    // instruction width is fixed. 

    // address width = 1
    if (address_width == QSPI_CFG_BUS_SINGLE){
        // data width = 1
        if(data_width == QSPI_CFG_BUS_SINGLE){
            if(cfg_.prot_if.writeoc != NRF_QSPI_WRITEOC_PP){
                cfg_.prot_if.writeoc = NRF_QSPI_WRITEOC_PP;
                is_cfg_changed = true;
            }
        // data width = 2
        }else if(data_width == QSPI_CFG_BUS_DUAL){
            if(cfg_.prot_if.writeoc != NRF_QSPI_WRITEOC_PP2O){
                cfg_.prot_if.writeoc = NRF_QSPI_WRITEOC_PP2O;
                is_cfg_changed = true;
            }
        // data width = 4
        }else if(data_width == QSPI_CFG_BUS_QUAD){
            if(cfg_.prot_if.writeoc != NRF_QSPI_WRITEOC_PP4O){
                cfg_.prot_if.writeoc = NRF_QSPI_WRITEOC_PP4O;
                is_cfg_changed = true;
            }
        }
    // address width = 2
    }else if(address_width == QSPI_CFG_BUS_DUAL){
        return DEVICE_NOT_SUPPORTED;
    // address width = 4
    } else if(address_width == QSPI_CFG_BUS_QUAD){
        // data width = 4
        if(data_width == QSPI_CFG_BUS_QUAD){
            if(cfg_.prot_if.writeoc != NRF_QSPI_WRITEOC_PP4IO){
                cfg_.prot_if.writeoc = NRF_QSPI_WRITEOC_PP4IO;
                is_cfg_changed = true;
            }
        }
    } else {
        return DEVICE_INVALID_PARAMETER;
    }
      
    //Configure QSPI with new command format
    if(is_cfg_changed){
        nrfx_qspi_uninit();
        nrfx_err_t ret_status = nrfx_qspi_init(&cfg_, NULL, NULL);
        if (ret_status != NRFX_SUCCESS ) {
            if (ret_status == NRFX_ERROR_INVALID_PARAM) {
                return DEVICE_INVALID_PARAMETER;
            } else {
                return DEVICE_SPI_ERROR;
            }
        }
        is_cfg_changed = false;
    }

    return DEVICE_OK;
}


int NRF52QSPI::configureReadFormat(QspiBusWidth address_width, QspiBusWidth data_width, uint8_t rx_delay)
{
    // instruction width is fixed. 

    // address width = 1
    if (address_width == QSPI_CFG_BUS_SINGLE){
        // data width = 1
        if(data_width == QSPI_CFG_BUS_SINGLE){
            if(cfg_.prot_if.readoc != NRF_QSPI_READOC_FASTREAD){
                cfg_.prot_if.readoc = NRF_QSPI_READOC_FASTREAD;
                is_cfg_changed = true;
            }
        // data width = 2
        }else if(data_width == QSPI_CFG_BUS_DUAL){
            if(cfg_.prot_if.readoc != NRF_QSPI_READOC_READ2O){
                cfg_.prot_if.readoc = NRF_QSPI_READOC_READ2O;
                is_cfg_changed = true;
            }
        // data width = 4
        }else if(data_width == QSPI_CFG_BUS_QUAD){
            if(cfg_.prot_if.readoc != NRF_QSPI_READOC_READ4O){
                cfg_.prot_if.readoc = NRF_QSPI_READOC_READ4O;
                is_cfg_changed = true;
            }
        }
    // address width = 2
    }else if(address_width == QSPI_CFG_BUS_DUAL){
        // data width = 2
        if(cfg_.prot_if.readoc != NRF_QSPI_READOC_READ2IO){
            cfg_.prot_if.readoc = NRF_QSPI_READOC_READ2IO;
            is_cfg_changed = true;
        }        
    // address width = 4
    } else if(address_width == QSPI_CFG_BUS_QUAD){
        // data width = 4
        if(data_width == QSPI_CFG_BUS_QUAD){
            if(cfg_.prot_if.readoc != NRF_QSPI_READOC_READ4IO){
                cfg_.prot_if.readoc = NRF_QSPI_READOC_READ4IO;
                is_cfg_changed = true;
            }
        }
    } else {
        return DEVICE_INVALID_PARAMETER;
    }

    if(rx_delay > 7){
        return DEVICE_INVALID_PARAMETER;
    } else {
        NRF_QSPI->IFTIMING = (rx_delay << QSPI_IFTIMING_RXDELAY_Pos);
    }
      
    //Configure QSPI with new command format
    if(is_cfg_changed){
        nrfx_qspi_uninit();
        nrfx_err_t ret_status = nrfx_qspi_init(&cfg_, NULL, NULL);
        if (ret_status != NRFX_SUCCESS ) {
            if (ret_status == NRFX_ERROR_INVALID_PARAM) {
                return DEVICE_INVALID_PARAMETER;
            } else {
                return DEVICE_SPI_ERROR;
            }
        }
        is_cfg_changed = false;
    }

    return DEVICE_OK;
}


int NRF52QSPI::sendCustomInst(uint8_t inst, 
    uint8_t* tx_buf, uint8_t* rx_buf, size_t tx_rx_len,
    bool long_frame_loop)
{
    if(tx_rx_len > 0 && (!tx_buf && !rx_buf)){
        return DEVICE_INVALID_PARAMETER;
    }

    int ret = DEVICE_OK;
    bool is_long_frame = false;
    nrf_qspi_cinstr_conf_t cinstr_cfg;
    nrfx_err_t err_code;

    cinstr_cfg.opcode = inst;
    cinstr_cfg.io2_level = true;
    cinstr_cfg.io3_level = true;
    cinstr_cfg.wipwait = true;
    cinstr_cfg.wren = (tx_buf && tx_rx_len)?true:false;    

    if(!is_long_frame){
        tx_rx_len++; //for instruction (op code)
        cinstr_cfg.length = (nrf_qspi_cinstr_len_t)tx_rx_len;
        err_code = nrfx_qspi_cinstr_xfer(&cinstr_cfg, tx_buf, rx_buf);
    }else{
        cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_1B;
        err_code = nrfx_qspi_lfm_start(&cinstr_cfg);
        if(err_code == NRFX_SUCCESS){
            err_code = nrfx_qspi_lfm_xfer(tx_buf, rx_buf, tx_rx_len, long_frame_loop);
        }
    }
    
    if(err_code == NRFX_ERROR_BUSY){
        ret = DEVICE_BUSY;
    }else if(err_code == NRFX_ERROR_TIMEOUT){
        ret = DEVICE_SPI_ERROR;
    }

    return ret;
}

} // namespace codal
