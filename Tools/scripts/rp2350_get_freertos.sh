#!/usr/bin/env bash
# Fetch the FreeRTOS kernel used by the RP2350 HAL into modules/freertos_kernel.
#
# Cloned on demand rather than tracked as a git submodule, so developers not
# building for RP2350 don't pay for it on every clone, matching how
# Tools/scripts/rp2350_get_pico_sdk.sh handles the Pico-SDK.
#
# This uses Raspberry Pi's fork rather than upstream FreeRTOS-Kernel: the
# RP2350 ports (RP2350_ARM_NTZ, RP2350_RISC-V) live there and are not in any
# upstream release as of V11.3.0, which ships only an RP2040 port.

set -e

REPO="https://github.com/raspberrypi/FreeRTOS-Kernel.git"
# pinned rather than tracking main, so builds are reproducible
COMMIT="4f7299d6ea746b27a9dd19e87af568e34bd65b15"

if [ ! -d modules ]; then
    echo "this script needs to be run from the root of your repo, giving up."
    exit 1
fi

cd modules

if [ ! -d freertos_kernel ]; then
    echo "cloning FreeRTOS kernel into modules/freertos_kernel"
    git clone --filter=blob:none --no-checkout "$REPO" freertos_kernel
fi

cd freertos_kernel
git fetch --depth 1 origin "$COMMIT"
git checkout -f "$COMMIT"

cd ../..

echo
echo "FreeRTOS kernel ready at modules/freertos_kernel"
