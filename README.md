# CryptoAuthLib

This repository contains the CryptoAuthLib HAL implementation for CC2650.

In the `hal` folder you will find two implementations, one direcly based
on the [cc26xxware](http://processors.wiki.ti.com/index.php/CC26xxware)
implemenation provided by Texas Instruments and another using the
[Contiki i2c](https://github.com/contiki-ng/contiki-ng/blob/develop/arch/platform/srf06-cc26xx/sensortag/board-i2c.c) abstraction for the Sensortag board. The implemenation based on
the Contiki primitives reduces the size of the HAL when used with a
Contiki image.

It has been tested with a [TI Sensortag CC2650](http://www.ti.com/tool/cc2650stk).
