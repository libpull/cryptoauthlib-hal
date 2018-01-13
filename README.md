# CryptoAuthLib HAL

This repository contains a [CryptoAuthLib](https://github.com/MicrochipTech/cryptoauthlib/tree/b31ed52daf2260329a131da19f8cc40a4d32d150) HAL implementation for CC2650.

In the `hal` folder you will find two implementations, one direcly based
on the [cc26xxware](http://processors.wiki.ti.com/index.php/CC26xxware)
implemenation provided by Texas Instruments and another using the
[Contiki i2c](https://github.com/contiki-ng/contiki-ng/blob/develop/arch/platform/srf06-cc26xx/sensortag/board-i2c.c) abstraction for the Sensortag board. The implemenation based on
the Contiki primitives reduces the size of the HAL when used with a
Contiki image.

This implementation has been tested with a 
[TI Sensortag CC2650](http://www.ti.com/tool/cc2650stk), the
[ATECC508A](https://www.microchip.com/wwwproducts/en/ATECC508A),
and the [ATSHA204A](https://www.microchip.com/wwwproducts/en/ATSHA204A)
connected via the i2c bus.

You can find some examples on how to use this HAL in [this](https://github.com/pull-iot/cryptoauthlib-contiki-examples)
repository.
