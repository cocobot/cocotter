#!/bin/sh

cargo build --release && cargo objcopy --release -- -O binary canon.bin && stm32flash -R -b 230400 -w canon.bin -v /dev/ttyUSB0