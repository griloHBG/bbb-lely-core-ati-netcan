## bbb-lely-core-ati-netcan
A project for Beaglebone Black that uses Lely-Core and ATI NetCANOEM force-torque sensor

# *This is work in progress*

## To get Lely-Core

Follow Lely-Core instructions: https://opensource.lely.com/canopen/docs/cross-compilation/#arm-on-linux

And, for me, I had to add ```--disable-tools``` also: https://gitlab.com/lely_industries/lely-core/-/issues/75

## About the ATI Force-Torque sensor

Its code is mixed in here, but I will configure this as a submodule or something alike to isolate the pieces. I have also a previous git that I'm updating: https://github.com/griloHBG/ati-netcanoem-cpp

## About the motor

A lot of information about the Maxon Motor and its controller is available here: https://github.com/griloHBG/passivity_project
