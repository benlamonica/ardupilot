#!/usr/bin/env python3
"""
Log USB enumeration of an RP2350 ArduPilot board with timestamps.

Start it, then plug the board in. Each boot stage shows up as a different
USB device, so the log says how far the board got:

  RP2350 Boot   ROM BOOTSEL (button held, or no valid image in flash)
  <name>-BL     AP bootloader (~3 s window after power-up)
  <name>        application

Nothing at all after plug-in means the board stopped before the bootloader
brought up USB (or never powered). "-BL" followed by nothing means the
application hung before its USB came up.

Uses the macOS IOUSB registry (ioreg) or Linux sysfs; no extra packages.

AP_FLAKE8_CLEAN
"""

import argparse
import glob
import os
import platform
import re
import subprocess
import time

RPI_VID = 0x2E8A


def devices_darwin():
    out = subprocess.run(['ioreg', '-p', 'IOUSB', '-l', '-w0'],
                         capture_output=True, text=True).stdout
    devs = {}
    cur = None
    for line in out.splitlines():
        if '+-o ' in line:
            cur = {}
            m = re.search(r'\+-o (.+?)@([0-9a-fA-F]+)', line)
            if m:
                cur['name'] = m.group(1)
                cur['loc'] = m.group(2)
                devs[m.group(2)] = cur
            continue
        if cur is None:
            continue
        m = re.search(r'"(USB Product Name|USB Serial Number|USB Vendor Name|idVendor|idProduct)" = (.+)$', line)
        if m:
            cur[m.group(1)] = m.group(2).strip().strip('"')
    result = {}
    for loc, d in devs.items():
        vid = int(d.get('idVendor', '0'))
        pid = int(d.get('idProduct', '0'))
        product = d.get('USB Product Name', d.get('name', '?'))
        serial = d.get('USB Serial Number', '')
        result[(loc, product, serial)] = (vid, pid, product, serial)
    return result


def devices_linux():
    result = {}
    for path in glob.glob('/sys/bus/usb/devices/*'):
        def rd(name):
            try:
                with open(os.path.join(path, name)) as f:
                    return f.read().strip()
            except OSError:
                return ''
        vid = rd('idVendor')
        if not vid:
            continue
        product = rd('product') or '?'
        serial = rd('serial')
        result[(os.path.basename(path), product, serial)] = (int(vid, 16), int(rd('idProduct') or '0', 16), product, serial)
    return result


def stage(vid, product):
    if vid == RPI_VID and 'boot' in product.lower():
        return 'BOOTSEL (ROM)'
    if product.endswith('-BL') or product.endswith('_BL'):
        return 'AP BOOTLOADER'
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--all', action='store_true', help='log every USB device, not just likely RP2350/ArduPilot ones')
    parser.add_argument('--interval', type=float, default=0.1, help='poll interval in seconds')
    parser.add_argument('--log', default=None, help='also append the log to this file')
    args = parser.parse_args()

    if platform.system() == 'Darwin':
        poll = devices_darwin
    elif platform.system() == 'Linux':
        poll = devices_linux
    else:
        raise SystemExit('only macOS and Linux are supported')

    logf = open(args.log, 'a') if args.log else None
    t0 = time.monotonic()

    def log(msg):
        line = '%s +%8.3fs  %s' % (time.strftime('%H:%M:%S'), time.monotonic() - t0, msg)
        print(line, flush=True)
        if logf:
            logf.write(line + '\n')
            logf.flush()

    def interesting(info):
        vid, _, product, _ = info
        return args.all or vid == RPI_VID or vid == 0x1209 or 'ardupilot' in product.lower() \
            or stage(vid, product) is not None or product.startswith('OpenDrone')

    prev = poll()
    log('watching USB; %d devices present. Plug the board in now (Ctrl-C to stop).' % len(prev))
    for key, info in prev.items():
        if interesting(info):
            log('  present: %s (%04x:%04x) serial=%s' % (info[2], info[0], info[1], info[3]))

    try:
        while True:
            time.sleep(args.interval)
            cur = poll()
            for key in cur.keys() - prev.keys():
                info = cur[key]
                if interesting(info):
                    s = stage(info[0], info[2]) or 'APPLICATION?'
                    log('CONNECT    %-40s %04x:%04x serial=%s  [%s]' % (info[2], info[0], info[1], info[3], s))
            for key in prev.keys() - cur.keys():
                info = prev[key]
                if interesting(info):
                    log('DISCONNECT %-40s %04x:%04x serial=%s' % (info[2], info[0], info[1], info[3]))
            prev = cur
    except KeyboardInterrupt:
        log('stopped')


if __name__ == '__main__':
    main()
