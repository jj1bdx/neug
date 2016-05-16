# Pre-compiled ELF executable images of NeuG 1.0.4

by Kenji Rikitake 16-MAY-2016

## LICENSE

GPLv3. See `COPYING` in this directory for the details of the license.

## Note well

Use of USB VID/PID for FSIJ is restricted under their license. In summary: for experimental and individual use only. For production, contact FSIJ. See `README` at the top level of the NeuG distribution for the further details.

## Compiled source

See <https://github.com/jj1bdx/neug/> for the source code of the executable files.

## NO WARRANTY

See Section 15 of the file `COPYING`.

## Compiled environment

* OS X 10.11.4
* HomeBrew nitsky/stm32/arm-none-eabi-gcc: stable 20150921
* The elf code is tested with STM32 ST Dongle and Flying Stone Technology FST-01
* source code used: tag `jj1bdx-1.0.4-20160516`

## Commands used

Compilation for STM32 ST Dongle:

```sh
./configure --vidpid=234b:0001 --target=ST_DONGLE
make clean
make
```

Compilation for FST-01:

```sh
./configure --vidpid=234b:0001 --target=FST_01
make clean
make
```

Writing the ELF file to an STM32 ST Dongle without protection:

```sh
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg
# OpenOCD commands
reset init
flash write_image erase neug-st-dongle-1.0.4-20160516.elf
reset
```

Writing the ELF file to an FST-01 with protection:

```
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg
# OpenOCD commands
reset init
stm32f1x unlock 0
reset init
sh write_image erase neug-fst-01-1.0.4-20160516.elf
stm32f1x lock 0
reset
```
