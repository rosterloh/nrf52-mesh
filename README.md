# Zephyr Mesh Server

Bluetooth mesh server application on [nrf52_adafruit_feather](http://docs.zephyrproject.org/boards/arm/nrf52_adafruit_feather/doc/nrf52_adafruit_feather.html)

### Building
    $ source ../zephyr/zephyr-env.sh
    $ mkdir build && cd build
    $ cmake -GNinja ..
    $ ninja

### Configuring
    $ ninja menuconfig

### Flashing
    $ ninja flash

### Debugging
    $ ninja debug