#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_RP2350_Namespace.h"
#include <AP_Common/Bitmask.h>
#include <AP_FlashStorage/AP_FlashStorage.h>

#include <hardware/flash.h>

#define STORAGE_SIZE HAL_STORAGE_SIZE

/*
  One flash erase block per AP_FlashStorage sector, so that erasing a sector
  is a single block erase command rather than sixteen 4k sector erases.
*/
#define STORAGE_SECTOR_SIZE FLASH_BLOCK_SIZE

/*
  One dirty line per AP_FlashStorage block, rather than the small power of
  two other HALs use. A dirty line costs a whole block in the flash log
  however few of its bytes changed, so matching the two means a line write is
  exactly one flash page and never straddles two.
*/
#define STORAGE_LINE_SIZE 254
#define STORAGE_NUM_LINES (STORAGE_SIZE/STORAGE_LINE_SIZE)

static_assert(STORAGE_SIZE % STORAGE_LINE_SIZE == 0,
              "storage size must be a whole number of lines");

class RP2350::Storage : public AP_HAL::Storage
{
public:
    void init() override {}
    void read_block(void *dst, uint16_t src, size_t n) override;
    void write_block(uint16_t dst, const void* src, size_t n) override;

    void _timer_tick(void) override;
    bool healthy(void) override;
    bool get_storage_ptr(void *&ptr, size_t &size) override;

private:
    void _storage_open(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    void _flash_load(void);
    void _flash_write(uint16_t line);

    bool _flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length);
    bool _flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length);
    bool _flash_erase_sector(uint8_t sector);
    bool _flash_erase_ok(void);

    // byte offset of one of our two sectors within the flash chip
    static uint32_t sector_offset(uint8_t sector);

    bool _initialised;
    bool _flash_failed;
    uint32_t _last_empty_ms;

    uint8_t _buffer[STORAGE_SIZE] __attribute__((aligned(4)));
    Bitmask<STORAGE_NUM_LINES> _dirty_mask;

    AP_FlashStorage _flash{_buffer,
                           STORAGE_SECTOR_SIZE,
                           FUNCTOR_BIND_MEMBER(&Storage::_flash_write_data, bool, uint8_t, uint32_t, const uint8_t *, uint16_t),
                           FUNCTOR_BIND_MEMBER(&Storage::_flash_read_data, bool, uint8_t, uint32_t, uint8_t *, uint16_t),
                           FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_sector, bool, uint8_t),
                           FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_ok, bool)};
};
