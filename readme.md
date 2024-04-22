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
**cmps12_open_i2c_device**
```c
int cmps12_open_i2c_device(const char *device);
```
Open I2C device dispesifikasi oleh 'device' dan set menjadi I2C_SLAVE_FORCE Mode.

**cmps12_release_i2c_device**
```c
int cmps12_release_i2c_device(int file);
```
untuk menutup I2C file decriptor

**cmps12_read_register**
```c
int cmps12_read_register(int file, uint8_t register_addr,uint8_t *data, ssize_t data_size);
```
read data dari spesifik register address

**cmps12_write_register**
```c
int cmps12_write_register(int file, uint8_t register_addr, uint8_t data, ssize_t data_size);
```
Write data ke spesifik register address

**cmps12_init**
```c
int cmps12_init(const char *device);
```
untuk meng-inisialisasi CMPS12 Compass Module dengan membuka I2C device

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

