#!/bin/sh
cp -r ./build/Debug/osx-pl2303.kext /tmp
cd /tmp
kextload osx-pl2303.kext
#avrdude -p m128 -c avrisp2 -P /dev/tty.PL2303-151
kextunload osx-pl2303.kext

