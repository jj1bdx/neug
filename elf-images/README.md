# Pre-compiled ELF executable images of NeuG 1.0.4

by Kenji Rikitake 23-JUL-2016

## LICENSE

GPLv3. See `COPYING` in this directory for the details of the license.

## Note well

Use of USB VID/PID for FSIJ is restricted under their license. In summary: for experimental and individual use only. For production, contact FSIJ. See `README` at the top level of the NeuG distribution for the further details.

## Compiled source

See <https://github.com/jj1bdx/neug/> for the source code of the executable files.

## NO WARRANTY

See Section 15 of the file `COPYING`.

## Target boards and devices

* [Boruit STM32F103CBT6 board (from Akizuki Denshi, in Japanese)](http://akizukidenshi.com/catalog/g/gK-05820/)
* [FST-01](http://www.gniibe.org/FST-01/fst-01.html) ([at Flying Stone Technology, in Japanese)](http://www.gniibe.org/shop/gnuk_1_2_x-on-fst-01.html)) ([at Seeed Studio](http://www.seeedstudio.com/wiki/FST-01))
* [ST Dongle, aka STM32 Nucleo Dongle](https://gist.github.com/jj1bdx/b78b3747196fd303f49d829a7e36ed8d) ([Japanese article in Qiita](http://qiita.com/jj1bdx/items/2f32d8c8649d7825a9a3))

## Compilation environment

* OS X 10.11.6
* HomeBrew nitsky/stm32/arm-none-eabi-gcc: stable 20150921
* The elf code is tested with STM32 ST Dongle and Flying Stone Technology FST-01
* source code used: tag `jj1bdx-1.0.4-20160723`

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

Compilation for Boruit STM32F103:

```sh
./configure --vidpid=234b:0001 --target=BORUIT_STM32F103
make clean
make
```

Writing the ELF file to an STM32 ST Dongle without protection:

```sh
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg
# OpenOCD commands
reset init
flash write_image erase neug-st-dongle-1.0.4-20160723-ged5ac05.elf
reset
```

Writing the ELF file to an FST-01 with protection:

```
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg
# OpenOCD commands
reset init
stm32f1x unlock 0
reset init
sh write_image erase neug-fst-01-1.0.4-20160723-g34d5a7a.elf
stm32f1x lock 0
reset
```
