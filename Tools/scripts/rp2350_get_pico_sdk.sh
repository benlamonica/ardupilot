#!/usr/bin/env bash
# Fetch the Raspberry Pi Pico-SDK used by the RP2350 HAL into modules/pico_sdk.
#
# The SDK is cloned on demand rather than tracked as a git submodule, so
# developers not building for RP2350 don't pay for it on every clone. This
# mirrors how Tools/scripts/esp32_get_idf.sh handles the ESP-IDF.

set -e

TAG="2.3.0"

if [ ! -d modules ]; then
    echo "this script needs to be run from the root of your repo, giving up."
    exit 1
fi

cd modules

if [ ! -d pico_sdk ]; then
    echo "cloning pico-sdk $TAG into modules/pico_sdk"
    git clone -b "$TAG" --depth 1 https://github.com/raspberrypi/pico-sdk.git pico_sdk
else
    echo "found modules/pico_sdk, fetching $TAG"
    cd pico_sdk
    git fetch --depth 1 origin "refs/tags/$TAG:refs/tags/$TAG"
    git checkout -f "$TAG"
    cd ..
fi

cd pico_sdk
# tinyusb provides the USB CDC console; cyw43-driver and lwip are needed by
# the Pico W / Pico 2 W board headers.
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip

cd ../..

echo
echo "pico-sdk ready at modules/pico_sdk"
echo "configure with: ./waf configure --board rp2350generic"
