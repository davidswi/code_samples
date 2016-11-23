//
//  SST25VF_spi_driver.h
//  
//
//  Created by David Switzer on 11/22/16.
//
//

#ifndef SST25VF_spi_driver_h
#define SST25VF_spi_driver_h

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Status register flags
#define BUSY 	(1 << 0)	// When set, internal write operation in progress -- read only
#define WEL		(1 << 1) 	// When set, device is memory write enabled -- read only

// NOTE: BP2-BP0 write protection truth table is shown on p.5 of SST25VF data sheet
#define BP0		(1 << 2)  // When set, upper 1/16 of Flash is write protected -- read/write
#define BP1		(1 << 3)	// When set, upper 1/8 of Flash is write protected -- read/write
#define BP2 	(1 << 4)  // When set, upper 1/2 of Flash is write protected -- read/write
#define BP3		(1 << 5) 	// Don't care for 8Mbit part
#define AAI 	(1 << 6) 	// When set, in auto address increment programming mode -- read only
#define BPL 	(1 << 7) 	// When set, BP3-0 are read only, when clear they are read-write -- read/write

#define DEFAULT_POWER_UP_STATUS_REG_VALUE   0x1C

// The data sheet specifies the following long running operation times
#define SECTOR_ERASE_TIME_MS                25
#define BLOCK_ERASE_TIME_MS                 25
#define CHIP_ERASE_TIME                     50

// Byte program time in us
#define BYTE_PROGRAM_US                     10

typedef struct{
    uint8_t manuf_id;
    uint8_t mem_type;
    uint8_t mem_cap;
} jedec_id;

typedef enum{
    READ_SPEED_NORMAL,
    READ_SPEED_HIGH
} READ_SPEED;

// Setup
int sst25vf_init(void);

// Device identification
int sst25vf_jedec_id_read(jedec_id *id);
int sst25vf_id_read(uint8_t *manuf_id, uint8_t *device_id);

// Status register
int sst25vf_read_status_register(uint8_t *status);
int sst25vf_write_status_register(uint8_t block_protect_mask);
int sst5vf_set_enable_AAI_status(bool enable);
int sst25fv_wait_busy_clear(void);

// Read functions
int sst25vf_read_byte(READ_SPEED speed, uint32_t addr, uint8_t *byte_read);
int sst25vf_read_range(READ_SPEED speed, uint32_t start_addr, uint8_t *buff, size_t read_count);

// Write functions
int sst25vf_set_write_enable(bool write_enable);

// Note that write byte will only succeed if the following conditions are met:
// 1. Write enable has been set
// 2. The address being written are in the erased (0xFF) state
// 3. The region in which the address resides is not write protected
// Also wait busy clear should be called to poll for write completion or a delay of TWB should be executed
int sst25vf_write_byte(uint32_t addr, uint8_t byte);

// Prior to calling the write range function, write enable should be set to false. The AAI function will take care of
// enabling write prior to sending data and disabling it after the data has been written.
int sst25vf_write_range_AAI(uint32_t start_addr, uint8_t *buff, size_t write_count);

// Erase functions -- write enable must be set prior to calling these functions and wait busy clear should be called
// to poll for erase completion or the appropriate delay time should be injected (TSE, TBE, TCE)
int sst25vf_sector_erase(uint16_t sector_addr);
int sst25vf_32kblock_erase(uint16_t block_addr);
int sst25vf_64kblock_erase(uint16_t block_addr);
int sst25vf_chip_erase(void);

#endif /* SST25VF_spi_driver_h */
