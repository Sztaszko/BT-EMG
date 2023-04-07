# BT-EMG
## Introduction
BT-EMG is a research project of evaluation of the usability wireless EMG sensor in comparison to reference products.

Data is send from ESP32 via Bluetooth and read with a `read.py` script.

## Packages installation
Use `pip install -r requirements.txt`

If PyBluez 3.0 is unavailable use the most recent version from GH: `pip install git+https://github.com/pybluez/pybluez.git#egg=pybluez`

Python 3.7.8 version is used.

## Usage
read.py [-h] [T] [Addr] [P]

positional arguments:

  T           Measurement time
  Addr        Bluetooth device address
  P           Bluetooth device port

optional arguments:

  -h, --help  show this help message and exit

Script for research purposes only.