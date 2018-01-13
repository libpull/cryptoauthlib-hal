# CryptoAuthLib

This repository contains the HAL to use Atmel CryptoAuthLib with CC2650.

It has been tested with a [TI Sensortag CC2650](http://www.ti.com/tool/cc2650stk).

If you are planning to use it with Contiki you should also patch the
library to use Contiki memory allocation primitives. The patches are
contained in the `patch` folder and can be applied calling the
`autogen.sh` script.
