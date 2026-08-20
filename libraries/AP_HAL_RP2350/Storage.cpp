#include "Storage.h"

#include <string.h>

#include <hardware/flash.h>
#include <pico/flash.h>

using namespace RP2350;

extern const AP_HAL::HAL& hal;

// end of the firmware image, from the SDK's linker script
extern "C" char __flash_binary_end;

/*
  The two storage sectors live at the very top of the flash chip, above the
  firmware. A UF2 or picotool load only rewrites the sectors the image itself
  covers, so parameters survive a firmware update.
*/
#ifndef HAL_RP2350_STORAGE_OFFSET
#define HAL_RP2350_STORAGE_OFFSET (PICO_FLASH_SIZE_BYTES - 2*STORAGE_SECTOR_SIZE)
#endif

/*
  How long to wait for the other core to park itself. This covers only the
  handshake, not the erase or program, which run with both cores stopped.
*/
#ifndef HAL_RP2350_FLASH_TIMEOUT_MS
#define HAL_RP2350_FLASH_TIMEOUT_MS 1000
#endif

static_assert(STORAGE_SECTOR_SIZE % FLASH_SECTOR_SIZE == 0,
              "storage sector must be a whole number of flash sectors");
static_assert(HAL_RP2350_STORAGE_OFFSET % FLASH_SECTOR_SIZE == 0,
              "storage must start on a flash sector boundary");

/*
  flash_safe_execute() hands its callback a single void*, so the operation is
  described by one of these.
*/
struct flash_op {
    uint32_t offset;
    const uint8_t *data;
    uint32_t length;
};

static void do_flash_program(void *p)
{
    const struct flash_op *op = (const struct flash_op *)p;
    flash_range_program(op->offset, op->data, op->length);
}

static void do_flash_erase(void *p)
{
    const struct flash_op *op = (const struct flash_op *)p;
    flash_range_erase(op->offset, op->length);
}

uint32_t Storage::sector_offset(uint8_t sector)
{
    return HAL_RP2350_STORAGE_OFFSET + sector * STORAGE_SECTOR_SIZE;
}

void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }
    _dirty_mask.clearall();
    _flash_load();
    _initialised = true;
}

/*
  mark some lines as dirty. Note that there is no attempt to avoid the race
  condition between this code and the _timer_tick() code below, which both
  update _dirty_mask. If we lose the race then the result is that a line is
  written more than once, but it won't result in a line not being written.
*/
void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
    const uint16_t end = loc + length;
    for (uint16_t line=loc/STORAGE_LINE_SIZE;
         line <= end/STORAGE_LINE_SIZE;
         line++) {
        // a line index of STORAGE_NUM_LINES is possible when the write ends
        // exactly on the last line, and Bitmask ignores it
        _dirty_mask.set(line);
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    _storage_open();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        _storage_open();
        memcpy(&_buffer[loc], src, n);
        _mark_dirty(loc, n);
    }
}

void Storage::_timer_tick(void)
{
    if (!_initialised) {
        return;
    }
    if (_dirty_mask.empty()) {
        _last_empty_ms = AP_HAL::millis();
        return;
    }

    // write out the first dirty line. We don't write more than one to keep
    // the latency of this call to a minimum
    uint16_t i;
    for (i=0; i<STORAGE_NUM_LINES; i++) {
        if (_dirty_mask.get(i)) {
            break;
        }
    }
    if (i == STORAGE_NUM_LINES) {
        // this shouldn't be possible
        return;
    }

    _flash_write(i);
}

void Storage::_flash_load(void)
{
    // the storage region sits above the firmware, so catch it being overrun
    // by an image that has grown into it rather than corrupting either
    if ((uint32_t)&__flash_binary_end - XIP_BASE > HAL_RP2350_STORAGE_OFFSET) {
        AP_HAL::panic("flash storage overlaps firmware");
    }
    if (!_flash.init()) {
        AP_HAL::panic("unable to init flash storage");
    }
}

/*
  write one storage line. This also updates _dirty_mask.
*/
void Storage::_flash_write(uint16_t line)
{
    if (_flash.write(line*STORAGE_LINE_SIZE, STORAGE_LINE_SIZE)) {
        // mark the line clean
        _dirty_mask.clear(line);
    }
}

/*
  callback to write data to flash.

  The bootrom routine behind flash_range_program() only takes whole 256 byte
  pages, which is why AP_FlashStorage uses its 256 byte block layout on this
  board. The check below is what guarantees that: a misaligned write would
  otherwise silently corrupt a neighbouring page.
*/
bool Storage::_flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
    if ((offset % FLASH_PAGE_SIZE) != 0 || (length % FLASH_PAGE_SIZE) != 0) {
        return false;
    }
    struct flash_op op { sector_offset(sector) + offset, data, length };
    if (flash_safe_execute(do_flash_program, &op, HAL_RP2350_FLASH_TIMEOUT_MS) != PICO_OK) {
        _flash_failed = true;
        return false;
    }
    return true;
}

/*
  callback to read data from flash. The flash is memory mapped through XIP,
  so this is a plain copy.
*/
bool Storage::_flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    memcpy(data, (const void *)(XIP_BASE + sector_offset(sector) + offset), length);
    return true;
}

/*
  callback to erase a flash sector. This stops both cores for the length of a
  block erase, which is why AP_FlashStorage only calls it when
  _flash_erase_ok() below allows it.
*/
bool Storage::_flash_erase_sector(uint8_t sector)
{
    struct flash_op op { sector_offset(sector), nullptr, STORAGE_SECTOR_SIZE };
    if (flash_safe_execute(do_flash_erase, &op, HAL_RP2350_FLASH_TIMEOUT_MS) != PICO_OK) {
        _flash_failed = true;
        return false;
    }
    return true;
}

bool Storage::_flash_erase_ok(void)
{
    // only allow erase while disarmed
    return !hal.util->get_soft_armed();
}

/*
  consider storage healthy if we have nothing to write sometime in the last
  2 seconds
*/
bool Storage::healthy(void)
{
    return _initialised && !_flash_failed &&
        AP_HAL::millis() - _last_empty_ms < 2000;
}

bool Storage::get_storage_ptr(void *&ptr, size_t &size)
{
    if (!_initialised) {
        return false;
    }
    ptr = _buffer;
    size = sizeof(_buffer);
    return true;
}
