Imagine-IoT! maker files repo!
=======================================

#### DALI, program to control a Dali driver with a Nucleo board

## First Program STM32
* First Program => clockpulse 1200 bits/sec => 1 period = 1 bit 
* 1 period = 0,833ms => 833us (HALDelay cannot be used)
* "One" = 1 period => 417ms low + 417ms high (signal from low to high = 1)
* "Zero: = 1 period => 417ms high + 417ms low (signal from high to low = 0)

## Second Program Arduino UNO
* Excample

License
=======
The files in this repository are distributed under various licenses.
Each file lists the license that applies to it, with the exception of
the bootloader binaries. These are distributed under the GPLv2, see
https://github.com/Optiboot/optiboot for the full terms and the sources
used to compile the bootloader (see the commit log of this repository to
find out the exact version used).