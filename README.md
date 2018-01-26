# Zephyr Mesh Server

Bluetooth mesh server application on [nrf52_pca10040](http://docs.zephyrproject.org/boards/arm/nrf52_pca10040/doc/nrf52_pca10040.html)

### Building
    $ source ../zephyr/zephyr-env.sh
    $ mkdir build && cd build
    $ cmake -GNinja -DBOARD=nrf52_pca10040 ..
    $ ninja

### Flashing
    $ ninja flash