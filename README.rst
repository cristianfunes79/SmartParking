.. _i2c_fujitsu_fram:

SmartParking Sample
###################

Overview
********
This is a sample firmware to work with hmc5883l device.
This assumes the slave address of hmc5883l is 0x1E.
It also includes unit tests to build with Ceedling.

Building and Running
********************

Load python virtual env before starting
`source ~/zephyrproject/.venv/bin/activate`

Then run
`west build -p always -b esp32 .`

To flash
`west flash`

Firmware is build under Zephyr west tool for esp32.
Tests can be executed using Ceedling framework.
