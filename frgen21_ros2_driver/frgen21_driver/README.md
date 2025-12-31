# FRGen21 Driver C++ Library

This is a shared library to interface with the the ZF FRGen21 Imaging Radar Sensor.

## Getting started

### Prerequisites
#### Dependencies

* Boost (C++ libraries): ```sudo apt install libboost-all-dev ```

### Installation
* `mkdir build`
* `cd build`
* `cmake ..`
* `make`
* `sudo make install`

### Add library to your project
* add this to your CMakeLists.txt:
```
find_package(frgen21_driver REQUIRED)
target_link_libraries(your_executable PUBLIC frgen21_driver::frgen21_driver)
```


## Contribution

### Set up git pre-commit hook
1. ```sh pip install pre-commit ```
2. ``` pre-commit install ```


## Contact

Yannik Motzet - [yannik.motzet@zf.com](mailto:yannik.motzet@zf.com)
