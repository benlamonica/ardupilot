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

/*
  Dump the low registers of every SPI device this board declares.

  This answers the first question on a new bus - are the wires right and is
  the chip answering - separately from whether a sensor driver works, which is
  what the per-sensor examples such as BARO_generic test. When a sensor will
  not probe, run this first: an all 0x00 or all 0xff dump is a wiring or chip
  select fault, while plausible values mean the bus is fine and the problem is
  above it.

  What to expect for the parts this board has been used with:

    dps310   register 0x0d (PROD_ID) reads 0x10

  A device is read at its low speed, since this runs before anything has
  established that the fast one works.
*/

#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// how many registers to dump, from register zero
#define DUMP_LEN 16

// most SPI sensors mark a read by setting the top bit of the register address
#define SPI_READ_FLAG 0x80

void setup(void)
{
    hal.console->printf("\nRP2350 SPI test\n");

    const uint8_t n = hal.spi->get_count();
    if (n == 0) {
        hal.console->printf("no SPI devices declared - check RP2350_SPIBUS and "
                            "RP2350_SPIDEV in hwdef.dat\n");
        return;
    }
    hal.console->printf("%u device(s):", (unsigned)n);
    for (uint8_t i = 0; i < n; i++) {
        hal.console->printf(" %s", hal.spi->get_device_name(i));
    }
    hal.console->printf("\n");
}

static void dump_device(const char *name)
{
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev = hal.spi->get_device(name);
    if (!dev) {
        hal.console->printf("%-10s could not be opened\n", name);
        return;
    }
    dev->set_read_flag(SPI_READ_FLAG);
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    uint8_t buf[DUMP_LEN] {};
    bool ok;
    {
        // the bus semaphore has to be held across a transfer, as another
        // thread may be talking to a different device on the same wires
        WITH_SEMAPHORE(dev->get_semaphore());
        ok = dev->read_registers(0, buf, sizeof(buf));
    }
    if (!ok) {
        hal.console->printf("%-10s read failed\n", name);
        return;
    }

    hal.console->printf("%-10s", name);
    for (uint8_t i = 0; i < sizeof(buf); i++) {
        hal.console->printf(" %02x", (unsigned)buf[i]);
    }
    hal.console->printf("\n");
}

void loop(void)
{
    const uint8_t n = hal.spi->get_count();
    if (n == 0) {
        hal.scheduler->delay(1000);
        return;
    }

    hal.console->printf("          ");
    for (uint8_t i = 0; i < DUMP_LEN; i++) {
        hal.console->printf(" %02x", (unsigned)i);
    }
    hal.console->printf("   <- register\n");

    for (uint8_t i = 0; i < n; i++) {
        dump_device(hal.spi->get_device_name(i));
    }
    hal.console->printf("\n");

    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
