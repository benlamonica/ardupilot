#!/usr/bin/env python3
'''
setup hwdef.h for RP2350

AP_FLAKE8_CLEAN
'''

import argparse
import os
import re
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef  # noqa:E402


class RP2350HWDef(hwdef.HWDef):
    '''
    Handles plain "define NAME VALUE" lines (see the shared hwdef.HWDef base
    class) plus RP2350_SERIAL. The remaining directives (SPI/I2C/ADC/RCOUT)
    will follow the same pattern as AP_HAL_ESP32/hwdef/scripts/esp32_hwdef.py
    as those drivers land.
    '''

    def __init__(self, **kwargs):
        super(RP2350HWDef, self).__init__(**kwargs)

        # list of RP2350_SERIAL declarations
        self.rp2350_serials = []

        # list of RP2350_ADC_PIN declarations
        self.rp2350_adcs = []

    def write_hwdef_header_content(self, f):
        for d in self.alllines:
            if d.startswith('define '):
                f.write('#define %s\n' % d[7:])

        self.write_SERIAL_config(f)
        self.write_ADC_config(f)

    def process_line(self, line, depth):
        '''process one line of pin definition file'''
        # the base class does not retain the lines it processes, but
        # write_hwdef_header_content() needs them
        self.all_lines.append(line)
        self.alllines.append(line)

        a = self.split_line(line, posix=False)
        if a[0] == 'RP2350_SERIAL':
            self.process_line_rp2350_serial(line, depth, a)

        if a[0] == 'RP2350_ADC_PIN':
            self.process_line_rp2350_adc(line, depth, a)

        super(RP2350HWDef, self).process_line(line, depth, a)

    # RP2350_SERIAL support:
    def process_line_rp2350_serial(self, line, depth, a):
        self.rp2350_serials.append(a[1:])

    # RP2350_ADC_PIN support:
    def process_line_rp2350_adc(self, line, depth, a):
        self.rp2350_adcs.append(a[1:])

    def write_ADC_config(self, f):
        '''write the ADC pin table'''
        adclist = []
        for adc in self.rp2350_adcs:
            if len(adc) != 3:
                self.error(f"Badly formed RP2350_ADC_PIN line {adc} {len(adc)=} want=3")
            (channel, scaling, ardupin) = adc
            # the RP2350A brings out 4 inputs plus the temperature sensor, the
            # RP2350B 8 plus the sensor; which package this is is not known
            # here, so only the wider limit can be checked
            if not re.match(r'^\d+$', channel) or int(channel) > 8:
                self.error(f"Bad RP2350_ADC_PIN channel {channel} (want an ADC input 0-8)")
            if not re.match(r'^\d+$', ardupin) or int(ardupin) > 254:
                self.error(f"Bad RP2350_ADC_PIN pin {ardupin} (want an ArduPilot pin number)")
            try:
                float(scaling)
            except ValueError:
                self.error(f"Bad RP2350_ADC_PIN scaling {scaling} (want a divider ratio)")
            adclist.append(f"{{ .channel={channel}, .scaling={scaling}, .ardupin={ardupin} }}")

        self.write_device_table(f, 'ADC pins', 'HAL_RP2350_ADC_PINS', adclist)

    def check_serial_pin(self, port, num, pin, want_rx):
        '''check one RP2350_SERIAL GPIO against the bank 0 function table and
        return it as an int'''
        if not re.match(r'^\d+$', pin):
            self.error(f"Bad RP2350_SERIAL pin {pin} (want a GPIO number)")
        n = int(pin)
        # bank 0 lays the UART pins out in groups of four as TX,RX,CTS,RTS,
        # alternating between uart0 and uart1 every second group. The CTS/RTS
        # pins of each group double as TX/RX through the UART_AUX function,
        # which the SDK's UART_FUNCSEL_NUM() selects, so what matters is only
        # that a TX pin is even, an RX pin is odd, and the group belongs to
        # the requested UART.
        if n > 47 or (n % 2 == 1) != want_rx or ((n + 4) // 8) % 2 != num:
            self.error(f"GPIO {n} cannot be {port} {'RX' if want_rx else 'TX'}")
        return n

    def write_SERIAL_config(self, f):
        '''write the hardware UART table'''
        seriallist = []
        for serial in self.rp2350_serials:
            if len(serial) != 3:
                self.error(f"Badly formed RP2350_SERIAL line {serial} {len(serial)=} want=3")
            (port, rxpin, txpin) = serial
            m = re.match(r'^uart([01])$', port)
            if m is None:
                self.error(f"Bad RP2350_SERIAL port {port} (want uart0 or uart1)")
            num = int(m.group(1))
            rx = self.check_serial_pin(port, num, rxpin, True)
            tx = self.check_serial_pin(port, num, txpin, False)
            seriallist.append(f"{{ .port={num}, .rx={rx}, .tx={tx} }}")

        self.write_device_table(f, 'hardware UARTs', 'HAL_RP2350_UART_DEVICES', seriallist)


if __name__ == '__main__':

    parser = argparse.ArgumentParser("rp2350_hwdef.py")
    parser.add_argument(
        '-D', '--outdir', type=str, default="/tmp", help='Output directory')
    parser.add_argument(
        'hwdef', type=str, nargs='+', default=None, help='hardware definition file')
    parser.add_argument(
        '--quiet', action='store_true', default=False, help='quiet running')

    args = parser.parse_args()

    c = RP2350HWDef(
        outdir=args.outdir,
        hwdef=args.hwdef,
        quiet=args.quiet,
    )
    c.run()
