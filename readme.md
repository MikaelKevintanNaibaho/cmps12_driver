# CMPS12 I2C DRIVER LIBRARY

library ini berisikan fungsi untuk interaksi dengan CMPS12 Compasss Modul dengan menggunakan I2C communication dalam linux systems

## Overview
library terdiri dari file sebagai berikut:

- 'cmps12_i2c.h': Header file, isinya prototype dari fungsi dan definisi
- 'cmps12_i2c.c': implementasi dari fungsi yang digunakan untuk berinteraksi dengan CMPS12 Compass Module.

## Compilation

untuk meng-compile library dengan "main program", gunakan Makefile. Makefile membuat compilation prosses jadi simple dengan mengautomasi build steps.

1. buka terminal
2. navigasi ke directory/folder yang berisikan library source files dan Makefile.

```bash
make all
```

command ini untuk meng-compile source file menjadi executable bernama "cmps12_reader", pada direktori 'build/bin'.

## Usage

setelah dicompile, run 'cmps12_reader' executable .
```bash
cd build/bin
./cmps12_reader
```

atau 
```bash
./build/bin/cmps12_reader
```

## Dependencies
library ini bergatung pada dependensi berikut:
- Linux System dengan I2C support yang sudah di-enabled
- 'gcc' compiler
- 'libm' untuk fungsi matematika

## Formating dan Static Analysis
format source file dengan menggunakan 'clang-format' dan lakukan static analisis dengan 'cppcheck'. gunakan command dibawah ini:
```bash
make format # format sorce file
make cppcheck #untuk melakukan static analysis
```

## Functions

#### Device Management

* **int cmps12_open_i2c_device(const char *device)**
    ```c
    int cmps12_open_i2c_device(const char *device);
    ```
  * Opens the I2C device specified by `device` and initializes communication with the CMPS12 sensor.
  * Returns file descriptor on success, or -1 on failure.
* **int cmps12_release_i2c_device(int file)**
    ```c
    int cmps12_release_i2c_device(int file);
    ```
  * Closes the I2C device identified by the file descriptor `file`.
  * Returns 0 on success, or -1 on failure.
Open I2C device dispesifikasi oleh 'device' dan set menjadi I2C_SLAVE_FORCE Mode.


#### Register Access

* **int cmps12_read_register(int file, uint8_t register_addr, uint8_t *data, ssize_t data_size)**
  * Reads data from a specific register on the CMPS12 sensor.
  * Inputs:
    * `file`: File descriptor of the open I2C device.
    * `register_addr`: Address of the register to be read.
    * `data`: Pointer to a buffer where the read data will be stored.
    * `data_size`: Size (in bytes) of the data to be read.
  * Returns 0 on success, or a negative value on failure (refer to documentation for specific error codes).
* **int cmps12_write_register(int file, uint8_t register_addr, uint8_t data, ssize_t data_size)**
  * Writes data to a specific register on the CMPS12 sensor.
  * Inputs:
    * `file`: File descriptor of the open I2C device.
    * `register_addr`: Address of the register to be written to.
    * `data`: Data to be written to the register.
    * `data_size`: Size (in bytes) of the data to be written.
  * Returns 0 on success, or a negative value on failure (refer to documentation for specific error codes).

#### Sensor Readings

**Compass**

* **uint8_t cmps12_read_bearing_8_bit(int cmps12_file)**
  * Reads the 8-bit compass bearing data from the CMPS12 sensor and converts it to degrees (0-255).
  * Returns the compass bearing in degrees on success, or 0 on failure.
* **uint16_t cmps12_read_bearing_16_bit_quaternion(int cmps12_file)**
  * Reads the 16-bit compass bearing data from the CMPS12 sensor (quaternion format) and converts it to degrees (0-359.9).
  * Returns the compass bearing in degrees on success, or 0 on failure.
* **uint16_t cmps12_read_bearing_16_bit_BNO055(int cmps12_file)**
  * Reads the 16-bit compass bearing data from the BNO055 sensor (assuming it's connected to the CMPS12) and converts it to degrees (0-360).
  * Returns the compass bearing in degrees on success, or 0 on failure.

**Orientation**

* **Orientation cmps12_read_orientation_quaternion(int cmps12_file)**
  * Reads compass bearing, pitch, and roll data from the CMPS12 sensor (quaternion format) and stores them in an `Orientation` struct.
  * Returns an `Orientation` struct containing sensor readings on success, or an empty struct on failure.
* **Orientation cmps12_read_orientation_BNO055(int cmps12_file)**
  * Reads compass bearing, pitch, and roll data from the BNO055 sensor (assuming it's connected to the CMPS12) and stores them in an `Orientation` struct.
  * Returns an `Orientation` struct containing sensor readings on success, or an empty struct on failure.


## Data Structures
library ini menggunakan data struktur sebagai berikut:

- SensorData: Struktur berisikan sensor data untuk x,y,z komponen
- Orientation: Struktur berisikan orientasi data (bearing, pith, roll).
- AllSensorData: Struktur berisikan semua sensor data (magnetometer, acclerometer, gyro, dan orientasi)

## Example Usage
```c
#include "cmps12_i2c.h"

int main() {
    int file = cmps12_init("/dev/i2c-1");
    if (file < 0) {
        fprintf(stderr, "Failed to initialize CMPS12 Compass Module\n");
        return -1;
    }

    // Read sensor data
    AllSensorData data = cmps12_read_all_data(file);

    // Process sensor data...

    cmps12_release_i2c_device(file);
    return 0;
}
```

## Author
**MIKAEL KEVINTAN NAIBAHO**
*mikaelkevintannaibaho@gmail.com*

