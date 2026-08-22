/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Ben La Monica
 */

#pragma once

#include <inttypes.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/SPIDevice.h>
#include "HAL_RP2350_Namespace.h"

#ifndef AP_HAL_SPI_ENABLED
#define AP_HAL_SPI_ENABLED (defined(HAL_RP2350_SPI_BUSES) && defined(HAL_RP2350_SPI_DEVICES))
#endif

#if AP_HAL_SPI_ENABLED

#include "DeviceBus.h"
#include "Semaphores.h"

#include <hardware/spi.h>

namespace RP2350
{

// one SPI peripheral and the three signals it drives, from an RP2350_SPIBUS
// line in hwdef.dat
struct SPIBusDesc {
    uint8_t spi;      // 0 for spi0, 1 for spi1
    uint8_t sck;
    uint8_t mosi;
    uint8_t miso;
};

// one device on a bus, from an RP2350_SPIDEV line
struct SPIDeviceDesc {
    const char *name;
    uint8_t bus;      // index into HAL_RP2350_SPI_BUSES, not the spi number
    uint8_t devid;    // identifies the sensor in DEVID parameters
    uint8_t cs;
    uint8_t mode;     // SPI mode, 0-3
    uint32_t lspeed;
    uint32_t hspeed;
};

class SPIBus : public DeviceBus
{
public:
    SPIBus(uint8_t _bus);
    uint8_t bus;      // index into HAL_RP2350_SPI_BUSES
};

class SPIDevice : public AP_HAL::SPIDevice
{
public:
    SPIDevice(SPIBus &_bus, const SPIDeviceDesc &_device_desc);

    bool set_speed(AP_HAL::Device::Speed speed) override;

    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len) override;
    bool transfer_fullduplex(uint8_t *send_recv, uint32_t len) override;

    bool set_chip_select(bool set) override;

    AP_HAL::Semaphore *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;
    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                  uint32_t period_usec) override;

private:
    // put the peripheral into this device's speed and mode, then select it
    void select(void);
    void deselect(void);

    SPIBus &bus;
    const SPIDeviceDesc &device_desc;
    spi_inst_t *spi;

    // whichever of the descriptor's two speeds set_speed() last chose
    uint32_t frequency;

    // set while set_chip_select() is holding the line down across more than
    // one transfer
    bool cs_forced;
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager
{
public:
    AP_HAL::SPIDevice *get_device_ptr(const char *name) override;

    // let a caller walk the devices this board declares without having to
    // include the generated table; used by examples/RP2350_SPI
    uint8_t get_count() override;
    const char *get_device_name(uint8_t idx) override;

private:
    SPIBus *buses;
};

}

#endif // AP_HAL_SPI_ENABLED
