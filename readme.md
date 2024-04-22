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
