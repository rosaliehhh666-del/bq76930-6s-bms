# Reference and licensing

LibreSolar/bms-firmware, commit `779d20199bff2666e01371ce2f562dcda4d6bda3`:
https://github.com/LibreSolar/bms-firmware/tree/779d20199bff2666e01371ce2f562dcda4d6bda3

Copyright (c) The Libre Solar Project Contributors.
Licensed under Apache License, Version 2.0; a copy is included as
`LICENSE.Apache-2.0` in this directory.

The generic NMC OCV points in bms_core.c derive from that project. The separation
of charging/discharging permission and OCV-plus-coulomb SOC design were also
informed by its bms_common.c, bms_soc.c and bq769x0 driver. These STM32 SPL
modules are a new portable implementation, not a Zephyr port or verbatim copy
of its complete driver. Changes include a six-cell channel map, explicit
communication failure handling, CRC enforcement for BQ7693003, signed ADC
offset, bounded discrete precharge, firmware output interlock, separate USB
INA226 monitoring and local UART/CAN/Modbus protocols.

Existing STMicroelectronics SPL/CMSIS/startup files keep their original notices
and licenses. The original template's unrelated Source/ examples are retained
but are not part of this firmware build. No claim is made that the generic OCV
curve is Samsung INR18650-30Q characterization data.
