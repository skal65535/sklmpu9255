# sklmpu9255

This is a simple C++ interface to MPU9255 gyro class and AK8963 Magnotometer devices
found in WaveShare devices e.g.

There are several libraries available around (see below), but i couldn't
find a small and compact code that would do just what i needed.

So i rewrote one :)

The code has been tested with an MPU9255, but probably also works with an
MPU9250 or MPU6500 and maybe others.

## requirements

  The library is in C++ and uses CMake (v 3.5+) as a build system.

  The tests uses GoogleTest framework (https://google.github.io/googletest/)

  The 'show_rpy' tool uses SDL2 for visualisation.

  This code is released under MIT License (see LICENSE file)

## Compiling and installing

## API

There's one object MPU. After a succesful call to init(), one can retrieve
measurements. Note that the calls can be slow-ish (~10ms), so it's a good
idea to run the calls in a separate thread usually. `show_rpy` does that by
default.

Quaternion filtering.

The library is NOT multithread-safe, far from. Some locks in the I2C
functions could be useful, but that's a TODO.


## Examples

  `show_rpy` is useful for testing the devices (using `-noshow` option) but
also for visualising the input values.

There's also some tests in the tests/ directory, which 

## Data sheets and technical docs

*  https://stanford.edu/class/ee267/misc/MPU-9255-Datasheet.pdf
* https://download.mikroe.com/documents/datasheets/ak8963c-datasheet.pdf
* https://mryslab.github.io/pymata_rh/mpu9250/

## Links to other libraries

* Waveshare's lib: https://www.waveshare.com/wiki/10_DOF_IMU_Sensor_(C)
* https://github.com/hideakitai/MPU9250
* https://github.com/Tarsier-Marianz/Tarsier_MPU9255

## TODO
  * make the code multithread safe, esp. the I2C functions.
