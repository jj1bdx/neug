# Installation of ELF executable images of NeuG

by Kenji Rikitake 24-OCT-2017

## Note well: use of FSIJ USB VID/PID

Yutaka Niibe, the Chairman of FSIJ, discourages redistributing binaries of NeuG executables with USB VID/PID. See <http://lists.alioth.debian.org/pipermail/gnuk-users/2017q4/000603.html> for his article describing the potential licensing issues on binaries including VID and PID of FSIJ.

Use of USB VID/PID for FSIJ is restricted under their license. In summary: for experimental and individual use only. For production, contact FSIJ. See `README` at the top level of the NeuG distribution for the further details.

## Target boards and devices tested

* [Boruit STM32F103CBT6 board (from Akizuki Denshi, in Japanese)](http://akizukidenshi.com/catalog/g/gK-05820/)
* [FST-01](http://www.gniibe.org/FST-01/fst-01.html) ([at Flying Stone Technology, in Japanese)](http://www.gniibe.org/shop/gnuk_1_2_x-on-fst-01.html)) ([at Seeed Studio](http://www.seeedstudio.com/wiki/FST-01))
* [ST Dongle, aka STM32 Nucleo Dongle](https://gist.github.com/jj1bdx/b78b3747196fd303f49d829a7e36ed8d) ([Japanese article in Qiita](http://qiita.com/jj1bdx/items/2f32d8c8649d7825a9a3))

## Compilation environment

* [GNU Arm Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) will work
* Tested on macOS and FreeBSD (Port devel/gcc-arm-embedded)

## Commands used

Compilation for STM32 ST Dongle:

```sh
make distclean
./configure --vidpid=234b:0001 --target=ST_DONGLE
make clean
make
```

Compilation for FST-01:

```sh
make distclean
./configure --vidpid=234b:0001 --target=FST_01
make clean
make
```

Compilation for Boruit STM32F103:

```sh
make distclean
./configure --vidpid=234b:0001 --target=BORUIT_STM32F103
make clean
make
```

Writing the ELF file to an STM32 ST Dongle without protection:

```sh
cd build
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg
# OpenOCD commands via telnet (localhost TCP port 4444)
reset init
flash write_image erase neug.elf
reset
```

Writing the ELF file to an FST-01 with protection:

```sh
cd build
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg
# OpenOCD commands via telnet (localhost TCP port 4444)
reset init
stm32f1x unlock 0
reset init
sh write_image erase neug.elf
stm32f1x lock 0
reset
```
