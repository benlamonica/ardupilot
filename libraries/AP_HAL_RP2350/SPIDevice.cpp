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

#include "SPIDevice.h"

#if AP_HAL_SPI_ENABLED

#include <AP_Math/AP_Math.h>
#include <string.h>

#include <hardware/gpio.h>

#include "Scheduler.h"

using namespace RP2350;

// units the hwdef speed columns are written in
#define MHZ (1000U*1000U)
#define KHZ (1000U)

static const SPIBusDesc bus_desc[] = { HAL_RP2350_SPI_BUSES };
static const SPIDeviceDesc device_desc[] = { HAL_RP2350_SPI_DEVICES };

/*
  Bring up one SPI peripheral. The bit rate and frame format set here are
  placeholders: every transfer sets both for the device it is aimed at, since
  devices sharing a bus rarely agree on either.
*/
SPIBus::SPIBus(uint8_t _bus) :
    DeviceBus(AP_HAL::Scheduler::PRIORITY_SPI), bus(_bus)
{
    const SPIBusDesc &desc = bus_desc[_bus];
    spi_inst_t *spi = spi_get_instance(desc.spi);

    spi_init(spi, 1*MHZ);
    gpio_set_function(desc.sck, GPIO_FUNC_SPI);
    gpio_set_function(desc.mosi, GPIO_FUNC_SPI);
    gpio_set_function(desc.miso, GPIO_FUNC_SPI);
}

SPIDevice::SPIDevice(SPIBus &_bus, const SPIDeviceDesc &_device_desc) :
    bus(_bus),
    device_desc(_device_desc)
{
    spi = spi_get_instance(bus_desc[_device_desc.bus].spi);
    set_device_bus(bus.bus);
    set_device_address(_device_desc.devid);
    set_speed(AP_HAL::Device::SPEED_LOW);
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed _speed)
{
    frequency = (_speed == AP_HAL::Device::SPEED_HIGH) ? device_desc.hspeed
                                                       : device_desc.lspeed;
    return true;
}

/*
  Put the peripheral into this device's speed and mode, then assert its chip
  select. Both of these do nothing while set_chip_select() is holding the line
  down, which is how a driver keeps several transfers inside one selection.
*/
void SPIDevice::select(void)
{
    if (cs_forced) {
        return;
    }
    /*
      Both the bit rate and the frame format are reprogrammed per transfer,
      because the peripheral holds one setting and the devices sharing it need
      not agree. It is a handful of register writes, and it has to happen
      before the chip select goes low: spi_set_format() disables the
      peripheral while it writes, which would glitch a selected device.
    */
    spi_set_baudrate(spi, frequency);
    spi_set_format(spi, 8,
                   (device_desc.mode & 2) ? SPI_CPOL_1 : SPI_CPOL_0,
                   (device_desc.mode & 1) ? SPI_CPHA_1 : SPI_CPHA_0,
                   SPI_MSB_FIRST);
    gpio_put(device_desc.cs, 0);
}

void SPIDevice::deselect(void)
{
    if (cs_forced) {
        return;
    }
    gpio_put(device_desc.cs, 1);
}

/*
  Hold the chip select across more than one transfer, or let it go again. The
  caller must hold the bus semaphore, as for any other transfer.

  Drivers use this to make a multi part exchange atomic, but also on its own:
  the DPS310 can come up in a state where its product ID reads back wrong, and
  toggling the line without clocking anything is what shakes it out. That is
  why this cannot just be left unimplemented - the sensor then fails to probe
  on some boots and not others.
*/
bool SPIDevice::set_chip_select(bool set)
{
    if (set) {
        if (cs_forced) {
            return true;
        }
        // ordered so that select() runs before the flag makes it a no-op
        select();
        cs_forced = true;
        return true;
    }
    if (!cs_forced) {
        return false;
    }
    cs_forced = false;
    deselect();
    return true;
}

/*
  Send then receive, as one transaction with the chip select held across both
  halves - the shape of a register read.
*/
bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    select();
    /*
      Clocked as two SDK calls rather than by assembling one combined buffer,
      which is what the other HALs do. On the wire it is the same transaction,
      and it keeps a long write off the bus thread's stack: a driver that
      pushes a configuration blob in a single call would otherwise size a
      variable length buffer there (the BMI270 sends 328 bytes at init).
    */
    if (send != nullptr && send_len > 0) {
        spi_write_blocking(spi, send, send_len);
    }
    if (recv != nullptr && recv_len > 0) {
        // zeros are clocked out while the device answers
        spi_read_blocking(spi, 0, recv, recv_len);
    }
    deselect();
    return true;
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    select();
    spi_write_read_blocking(spi, send, recv, len);
    deselect();
    return true;
}

/*
  The in place form. Overriding this matters: the base class implements it as
  transfer(buf, len, buf, len), which for this driver would clock 2*len bytes
  rather than exchanging len.
*/
bool SPIDevice::transfer_fullduplex(uint8_t *send_recv, uint32_t len)
{
    // safe to read and write one buffer, as the SDK's loop cannot receive a
    // byte before it has sent it, so the write index never passes the read one
    return transfer_fullduplex(send_recv, send_recv, len);
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
    return &bus.semaphore;
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(uint32_t period_usec,
                                                                    AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb, this);
}

bool SPIDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

uint8_t SPIDeviceManager::get_count()
{
    return ARRAY_SIZE(device_desc);
}

const char *SPIDeviceManager::get_device_name(uint8_t idx)
{
    if (idx >= ARRAY_SIZE(device_desc)) {
        return nullptr;
    }
    return device_desc[idx].name;
}

AP_HAL::SPIDevice *SPIDeviceManager::get_device_ptr(const char *name)
{
    uint8_t i;
    for (i = 0; i < ARRAY_SIZE(device_desc); i++) {
        if (strcmp(device_desc[i].name, name) == 0) {
            break;
        }
    }
    if (i == ARRAY_SIZE(device_desc)) {
        return nullptr;
    }
    const SPIDeviceDesc &desc = device_desc[i];

    SPIBus *busp;
    for (busp = (SPIBus *)buses; busp; busp = (SPIBus *)busp->next) {
        if (busp->bus == desc.bus) {
            break;
        }
    }

    if (busp == nullptr) {
        // first device asked for on this bus, so bring the bus up
        busp = NEW_NOTHROW SPIBus(desc.bus);
        if (busp == nullptr) {
            return nullptr;
        }
        /*
          Deassert every chip select on this bus before any of them is used.
          A floating select line is read as asserted by the device holding it,
          which would put two devices on the wires at once.
        */
        for (uint8_t d = 0; d < ARRAY_SIZE(device_desc); d++) {
            if (device_desc[d].bus != desc.bus) {
                continue;
            }
            gpio_init(device_desc[d].cs);
            gpio_put(device_desc[d].cs, 1);
            gpio_set_dir(device_desc[d].cs, GPIO_OUT);
        }
        busp->next = buses;
        buses = busp;
    }

    return NEW_NOTHROW SPIDevice(*busp, desc);
}

#endif // AP_HAL_SPI_ENABLED
