# encoding: utf-8

# flake8: noqa

"""
Waf tool for RP2350 build
"""

from waflib import Task
from waflib.TaskGen import after_method, feature

import os
import sys
import traceback

import hal_common

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_RP2350/hwdef/scripts'))
import rp2350_hwdef

def configure(cfg):
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()
    def bldpath(path):
        return bldnode.make_node(path).abspath()

    cfg.load('cmake')

    env = cfg.env
    env.AP_HAL_RP2350 = srcpath('libraries/AP_HAL_RP2350/targets/rp2350')
    env.AP_PROGRAM_FEATURES += ['rp2350_ap_program']

    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')

    try:
        env.PICO_SDK_PATH = os.environ['PICO_SDK_PATH']
    except KeyError:
        env.PICO_SDK_PATH = srcpath('modules/pico_sdk')
    if not os.path.exists(os.path.join(env.PICO_SDK_PATH, 'pico_sdk_init.cmake')):
        cfg.fatal("Pico-SDK not found at %s - run Tools/scripts/rp2350_get_pico_sdk.sh "
                  "or set PICO_SDK_PATH" % env.PICO_SDK_PATH)
    print("USING PICO SDK:" + str(env.PICO_SDK_PATH))

    try:
        env.FREERTOS_KERNEL_PATH = os.environ['FREERTOS_KERNEL_PATH']
    except KeyError:
        env.FREERTOS_KERNEL_PATH = srcpath('modules/freertos_kernel')
    rp2350_port = os.path.join(env.FREERTOS_KERNEL_PATH,
                               'portable/ThirdParty/GCC/RP2350_ARM_NTZ')
    if not os.path.exists(rp2350_port):
        cfg.fatal("FreeRTOS RP2350 port not found at %s - run "
                  "Tools/scripts/rp2350_get_freertos.sh or set FREERTOS_KERNEL_PATH"
                  % rp2350_port)
    print("USING FREERTOS KERNEL:" + str(env.FREERTOS_KERNEL_PATH))

    try:
        hwdef_obj = generate_hwdef_h(env)
    except Exception:
        traceback.print_exc()
        cfg.fatal("Failed to process hwdef.dat")
    hal_common.process_hwdef_results(cfg, hwdef_obj)

def generate_hwdef_h(env):
    '''run rp2350_hwdef.py'''
    hwdef_dir = os.path.join(env.SRCROOT, 'libraries/AP_HAL_RP2350/hwdef')

    if len(env.HWDEF) == 0:
        env.HWDEF = os.path.join(hwdef_dir, env.BOARD, 'hwdef.dat')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    hwdef = [env.HWDEF]
    if env.HWDEF_EXTRA:
        hwdef.append(env.HWDEF_EXTRA)

    hwdef_obj = rp2350_hwdef.RP2350HWDef(
        outdir=hwdef_out,
        hwdef=hwdef,
        quiet=False,
    )
    hwdef_obj.run()

    return hwdef_obj

def pre_build(self):
    """Configure pico-sdk as lib target"""
    lib_vars = {
        'ARDUPILOT_CMD': self.cmd,
        'WAF_BUILD_TARGET': self.targets,
        'ARDUPILOT_LIB': self.bldnode.find_or_declare('lib/').abspath(),
        'ARDUPILOT_BIN': self.bldnode.find_or_declare('lib/bin').abspath(),
        'PICO_BOARD': self.env.PICO_BOARD or 'pico2',
        'PICO_PLATFORM': 'rp2350-arm-s',
        'PICO_SDK_PATH': self.env.PICO_SDK_PATH,
        'FREERTOS_KERNEL_PATH': self.env.FREERTOS_KERNEL_PATH,
    }
    pico_sdk = self.cmake(
            name='pico-sdk',
            cmake_vars=lib_vars,
            cmake_src='libraries/AP_HAL_RP2350/targets/rp2350',
            cmake_bld='pico-sdk_build',
            )

    pico_sdk_showinc = pico_sdk.build('showinc', target='pico-sdk_build/includes.list')
    pico_sdk_showinc.post()

    class load_generated_includes(Task.Task):
        """After includes.list/defines.list are generated, apply them to env"""
        always_run = True
        def run(tsk):
            bld = tsk.generator.bld
            includes = bld.bldnode.find_or_declare('pico-sdk_build/includes.list').read().split()
            bld.env.prepend_value('INCLUDES', includes)
            defines = bld.bldnode.find_or_declare('pico-sdk_build/defines.list').read().split()
            bld.env.prepend_value('DEFINES', defines)

    tsk = load_generated_includes(env=self.env)
    tsk.set_inputs(self.path.find_resource('pico-sdk_build/includes.list'))
    self.add_to_group(tsk)

@feature('rp2350_ap_program')
@after_method('process_source')
def rp2350_firmware(self):
    self.link_task.always_run = True
    pico_sdk = self.bld.cmake('pico-sdk')

    build = pico_sdk.build('all', target='pico-sdk_build/ardupilot.elf')
    build.post()

    build.cmake_build_task.set_run_after(self.link_task)
