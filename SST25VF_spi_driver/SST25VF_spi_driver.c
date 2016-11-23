//
//  SST25VF_spi_driver.c
//  
//
//  Created by David Switzer on 11/22/16.
//
//

#include "SST25VF_spi_driver.h"

#include <errno.h>

#include "constants.h"
#include "app_timer.h"
#include "spi_master.h"

// Memory organization -- 4KB eraseable sectors with 32KB overlay blocks and 64KB overlay eraseable blocks
// HOLD* pin pauses SPI communication when asserted low
// WP* pin enables lockdown function of SR when asserted low

// Device operation instruction opcodes
#define READ								0x03
#define READ_HIGH_SPEED						0x0B
#define SECTOR_ERASE						0x20
#define BLOCK_ERASE_32K						0x52
#define BLOCK_ERASE_64K						0xD8
#define CHIP_ERASE							0x60 // Opcode 0xC7 also does chip erase
#define WRITE_BYTE							0x02
#define WRITE_WORD_AAI						0xAD
#define READ_STATUS_REG						0x05
#define ENABLE_WRITE_STATUS_REG             0x50
#define WRITE_STATUS_REG					0x01
#define WRITE_ENABLE						0x06
#define WRITE_DISABLE						0x04
#define READ_ID								0x90 // Opcode 0xAB also does read ID
#define READ_JEDEC_ID						0x9F
#define ENABLE_BSY_OUT_AAI                  0x70
#define DISABLE_BSY_OUT_AAI                 0x80

uint32_t *m_spi_state = NULL;

// Address byte masks
#define A23TOA16(ADDR)                      (uint8_t)(((ADDR) & 0x00FF0000) >> 16)
#define A15TOA8(ADDR)                       (uint8_t)(((ADDR) & 0x0000FF00) >> 8)
#define A7TOA0(ADDR)                        (uint8_t)((ADDR) & 0x000000FF)

// Word byte masks
#define HIBYTE(W)                           (uint8_t)(((W) & 0xFF00) >> 8)
#define LOBYTE(W)                           (uint8_t)(((W) & 0x00FF))

#define AAI_DEVICE_BUSY_FLAG                0 // 0 indicates busy, 1 indicates ready

#define MAX_WAIT_MS                         1000

// Private command methods
int WREN(void);	// Must be executed prior to any writes to enable writing
int WRDI(void); // Disables writing of the device -- execute after completing a write
int EWSR(void); // Enables writing of the status register
int EBSY(void); // Enables BUSY flag output on MISO when doing AAI writes
int DBSY(void);	// Disables BUSY flag output on MISO when doing AAI writes

int WREN(void)
{
    uint8_t opcode = WRITE_ENABLE;
    uint8_t rx_byte = 0;
    int ret = 0;
    
    if (!spi_master_tx_rx(m_spi_state, 1, &opcode, &rx_byte)){
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int WRDI(void)
{
    uint8_t opcode = WRITE_DISABLE;
    uint8_t rx_byte = 0;
    int ret = 0;
    
    if (!spi_master_tx_rx(m_spi_state, 1, &opcode, &rx_byte)){
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int EWSR(void)
{
    uint8_t opcode = ENABLE_WRITE_STATUS_REG;
    uint8_t rx_byte = 0;
    int ret = 0;
    
    if (!spi_master_tx_rx(m_spi_state, 1, &opcode, &rx_byte)){
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int EBSY(void)
{
    uint8_t opcode = ENABLE_BSY_OUT_AAI;
    
    uint8_t rx_byte = 0;
    int ret = 0;
    
    if (!spi_master_tx_rx(m_spi_state, 1, &opcode, &rx_byte)){
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int DBSY(void)
{
    uint8_t opcode = DISABLE_BSY_OUT_AAI;
    
    uint8_t rx_byte = 0;
    int ret = 0;
    
    if (!spi_master_tx_rx(m_spi_state, 1, &opcode, &rx_byte)){
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int sst25vf_init(void)
{
    int ret = 0;
    
    // The SST25VF supports Modes 0 and 3 and is most significant bit first
    // so we set the boolean to false indicating NOT LSB first
    m_spi_state = spi_master_init(SPI0, SPI_MODE0, false);
    
    if (m_spi_state == NULL){
        ret = -EINVAL;
    }
    
    return ret;
}

int sst25vf_jedec_id_read(jedec_id *id)
{
    uint8_t jedec_cmd[4] = {READ_JEDEC_ID, 0xFF, 0xFF, 0xFF};
    uint8_t jedec_read[4] = {0};
    int ret = 0;
    
    if (spi_master_tx_rx(m_spi_state, 4, jedec_cmd, jedec_read)){
        id->manuf_id = jedec_read[1];
        id->mem_type = jedec_read[2];
        id->mem_cap = jedec_read[3];
    }
    else{
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int sst25vf_id_read(uint8_t *manuf_id, uint8_t *device_id)
{
    uint8_t id_cmd[6] = {READ_ID, 0x00, 0x00, 0x00, 0xFF, 0xFF};
    uint8_t id_read[6]= {0};
    int ret = 0;
    
    if (spi_master_tx_rx(m_spi_state, 6, id_cmd, id_read)){
        *manuf_id = id_read[4];
        *device_id = id_read[5];
    }
    else{
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int sst25vf_read_status_register(uint8_t *status)
{
    uint8_t status_read_cmd[2] = {READ_STATUS_REG, 0xFF};
    uint8_t status_read[2] = {0};
    int ret = 0;
    
    if (spi_master_tx_rx(m_spi_state, 2, status_read_cmd, status_read)){
        *status = status_read[1];
    }
    else{
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int sst25vf_write_status_register(uint8_t block_protect_mask)
{
    uint8_t wsr_cmd[2] = {WRITE_STATUS_REG, 0};
    uint8_t rx_buf[2] = {0};
    int ret;
    
    wsr_cmd[1] = block_protect_mask;
    // First enable writing of the status register
    ret = EWSR();
    if (ret == 0){
        // Now write the status register
        if (!spi_master_tx_rx(m_spi_state, 2, wsr_cmd, rx_buf)){
            ret = -ETIMEDOUT;
        }
    }
    
    return ret;
}

int sst5vf_set_enable_AAI_status(bool enable)
{
    uint8_t opcode;
    uint8_t rx_byte = {0};
    int ret = 0;
    
    if (enable){
        // We are sending EBSY
        opcode = ENABLE_BSY_OUT_AAI;
    }
    else{
        opcode = DISABLE_BSY_OUT_AAI;
    }
    
    if (!spi_master_tx_rx(m_spi_state, 1, &opcode, &rx_byte)){
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int sst25fv_wait_busy_clear(void)
{
    uint8_t status = 0;
    uint32_t start_tick = 0;
    uint32_t curr_tick = 0;
    uint32_t ms_waiting;
    
    int ret;
    
    app_timer_cnt_get(&start_tick);
    do{
        // Read the status register
        ret = sst25vf_read_status_register(&status);
        if (ret != 0){
            break;
        }
        
        // Get the current tick count
        app_timer_cnt_get(&curr_tick);
        // And calculate ms we've been waiting
        ms_waiting = TICKS_TO_MS(curr_tick - start_tick);
        
        if (ms_waiting > MAX_WAIT_MS){
            ret = -ETIMEDOUT;
            break;
        }
    } while ((status & BUSY) == BUSY);
    
    return ret;
}

int sst25vf_read_byte(READ_SPEED speed, uint32_t addr, uint8_t *byte_read)
{
    uint8_t read_cmd[5] = {READ, 0x00, 0x00, 0x00, 0xFF};
    uint8_t bytes_read[5] = {0};
    int ret = 0;
    
    // Unpack the address bytes into the read command buffer
    read_cmd[1] = A23TOA16(addr);
    read_cmd[2] = A15TOA8(addr);
    read_cmd[3] = A7TOA0(addr);
    
    if (spi_master_tx_rx(m_spi_state, 5, read_cmd, bytes_read)){
        *byte_read = bytes_read[4];
    }
    else{
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int sst25vf_read_range(READ_SPEED speed, uint32_t start_addr, uint8_t *buff, size_t read_count)
{
    uint8_t read_cmd[5] = {0x00, 0x00, 0x00, 0x00, 0xFF};
    size_t tx_count;
    int ret = 0;
    
    // Unpack the address bytes into slots 1-3
    read_cmd[1] = A23TOA16(start_addr);
    read_cmd[2] = A15TOA8(start_addr);
    read_cmd[3] = A7TOA0(start_addr);
    
    if (speed == READ_SPEED_HIGH){
        // We have READ_HIGH_SPEED, ADDR23..16, ADDR15..8, ADDR7..0, DUMMY
        read_cmd[0] = READ_HIGH_SPEED;
        tx_count = 5;
    }
    else{
        // No dummy byte
        read_cmd[0] = READ;
        tx_count = 4;
    }
    
    // We use asymmetric spi master read with TX count as set above and RX skip count == TX count
    if (!spi_master_asymmetric_tx_rx(m_spi_state, tx_count, read_count, tx_count, read_cmd, buff)){
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int sst25vf_set_write_enable(bool write_enable)
{
    uint8_t status = 0;
    bool is_WEL_set;
    int ret;
    
    if (write_enable){
        // First send WREN
        ret = WREN();
    }
    else{
        // Send WRDI
        ret = WRDI();
    }
    
    if (ret == 0){
        // Read status register
        ret = sst25vf_read_status_register(&status);
        if (ret == 0){
            // Get write bit status
            is_WEL_set = ((status & WEL) == WEL);
            // Now check that it has the right value
            if ((write_enable && !is_WEL_set) || (!write_enable && is_WEL_set)){
                ret = -EIO;
            }
        }
    }
    
    return ret;
}

int sst25vf_write_byte(uint32_t addr, uint8_t byte)
{
    uint8_t write_cmd[5];
    uint8_t read_buf[5] = {0};
    int ret = 0;
    
    write_cmd[0] = WRITE_BYTE;
    write_cmd[1] = A23TOA16(addr);
    write_cmd[2] = A15TOA8(addr);
    write_cmd[3] = A7TOA0(addr);
    write_cmd[4] = byte;
    
    if (!spi_master_tx_rx(m_spi_state, 5, write_cmd, read_buf)){
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int sst25vf_write_range_AAI(uint32_t start_addr, uint8_t *buff, size_t write_count)
{
    uint8_t aai_cmd_buff[6];
    size_t stream_count = write_count - 2;
    int ret = 0;
    
    // First enable writing
    ret = sst25vf_set_write_enable(true);
    if (ret != 0){
        goto done;
    }
    
    // Now enable hardware end of write detection
    ret = EBSY();
    if (ret != 0){
        goto exit_AAI_mode;
    }
    
    // Set up the pre-stream command buffer consisting of the write AAI opcode, the start
    // address and the first two bytes of the buffer
    aai_cmd_buff[0] = WRITE_WORD_AAI;
    aai_cmd_buff[1] = A23TOA16(start_addr);
    aai_cmd_buff[2] = A15TOA8(start_addr);
    aai_cmd_buff[3] = A7TOA0(start_addr);
    aai_cmd_buff[4] = buff[0];
    aai_cmd_buff[5] = buff[1];
    
    // Issue the SPI stream
    if (!spi_master_stream_buffer(m_spi_state, 6, aai_cmd_buff, stream_count, buff + 2, 2, AAI_DEVICE_BUSY_FLAG)){
        ret = -EIO;
    }
    
exit_AAI_mode:
    if (sst25vf_set_write_enable(false) != 0 || DBSY() != 0){
        ret = -EIO;
    }
    
done:
    return ret;
}

int sst25vf_sector_erase(uint16_t sector_addr)
{
    uint8_t erase_cmd[4];
    uint8_t read_bytes[4] = {0};
    int ret = 0;
    
    erase_cmd[0] = SECTOR_ERASE;
    
    // We only care about A23 - A12 for the sector erase address, but we'll send all bits
    // as we've received them
    erase_cmd[1] = A23TOA16(sector_addr);
    erase_cmd[2] = A15TOA8(sector_addr);
    erase_cmd[3] = A7TOA0(sector_addr);
    
    if (!spi_master_tx_rx(m_spi_state, 4, erase_cmd, read_bytes)){
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int sst25vf_32kblock_erase(uint16_t block_addr)
{
    uint8_t erase_cmd[4];
    uint8_t read_bytes[4] = {0};
    int ret = 0;
    
    erase_cmd[0] = BLOCK_ERASE_32K;
    
    erase_cmd[1] = A23TOA16(block_addr);
    erase_cmd[2] = A15TOA8(block_addr);
    erase_cmd[3] = A7TOA0(block_addr);
    
    if (!spi_master_tx_rx(m_spi_state, 4, erase_cmd, read_bytes)){
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int sst25vf_64kblock_erase(uint16_t block_addr)
{
    uint8_t erase_cmd[4];
    uint8_t read_bytes[4] = {0};
    int ret = 0;
    
    erase_cmd[0] = BLOCK_ERASE_64K;
    
    erase_cmd[1] = A23TOA16(block_addr);
    erase_cmd[2] = A15TOA8(block_addr);
    erase_cmd[3] = A7TOA0(block_addr);
    
    if (!spi_master_tx_rx(m_spi_state, 4, erase_cmd, read_bytes)){
        ret = -ETIMEDOUT;
    }
    
    return ret;
}

int sst25vf_chip_erase(void)
{
    uint8_t opcode = CHIP_ERASE;
    uint8_t rx_byte = {0};
    int ret = 0;
    
    if (!spi_master_tx_rx(m_spi_state, 1, &opcode, &rx_byte)){
        ret = -ETIMEDOUT;
    }
    
    return ret;
}
