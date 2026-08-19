#!/usr/bin/env python3
'''
setup hwdef.h for RP2350

AP_FLAKE8_CLEAN
'''

import os
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef  # noqa:E402


class RP2350HWDef(hwdef.HWDef):
    '''
    Phase 1 only needs plain "define NAME VALUE" lines out of hwdef.dat (see
    the shared hwdef.HWDef base class for that handling); there are no
    RP2350-specific directives yet (no SPI/I2C/sensor bus declarations),
    since Phase 1 doesn't wire up any real peripherals through hwdef. Those
    will be added here, following the same pattern as
    AP_HAL_ESP32/hwdef/scripts/esp32_hwdef.py, once Phase 2 lands real
    sensor drivers.
    '''

    def write_hwdef_header_content(self, f):
        for d in self.alllines:
            if d.startswith('define '):
                f.write('#define %s\n' % d[7:])
