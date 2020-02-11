# Intro

This python driver for MLX90640 and MLX90641 aims to facilitate the interfacing on a PC.

Currently this driver supports 3 type of interfaces:
- EVB90640-41 ==> https://www.melexis.com/en/product/EVB90640-41/Evaluation-Board-MLX90640
- Raspberry Pi with I2C on any GPIO pin.
- Raspberry Pi on built-in hardware I2C bus.


## Dependencies

Driver:
- Python3
- pySerial


## Getting started

### Installation


```bash
pip install mlx9064x-driver
```

### Running the driver demo

* Connect the EVB to your PC.  
* pen a terminal and run following command:  


```bash
mlx9064x-dump-frame auto
```

This program takes 1 optional argument.

```bash
mlx9064x-dump-frame <communication-port>
```

`<communication-port>` can be:
- `auto` (default) search for comport with EVB90640 hardware as interface to MLX90640 or MLX90641.
- `COMx` comport on windows platform for EVB90640 HW, where x is the comport number.
- `/dev/ttyUSBx` comport on linux platform for EVB90640 HW, where x is the comport number.
- `/dev/ttyACMx` comport on raspberry pi linux platform for EVB90640 HW, where x is the comport number.
- `I2C-1` on raspberry pi use the I2C hardware; it requires raspi-config to enable i2c hardware.
- `I2CBB-<sda>-<scl>` [I2CBB-03-05] on raspberry pi, with **B**it**B**anging software I2C on any GPIO pin. 
     - `<sda>` is a 2-digit number of the physical pin (default: 03)
     - `<scl>` is a 2-digit number of the physical pin (default: 05)

Note: Physical pin numbers: see right side numbers in this picture: https://pinout.xyz/

### Usage

Below you can find an example on how to read a frame of the MLX90640 senor with I2C address 0x33 and frame rate 8Hz. Please look into EVB90640.py for more advanced features.

```python
import mlx.mlx90640 as mlx

dev = mlx.Mlx9064x('COM4', i2c_addr=0x33, frame_rate=8.0) # establish communication between EVB90640 and
                                                          # PC, the I2C address of the MLX90640 sensor is
                                                          # 0x33 and change the frame rate to 8Hz
dev.init()                      # read EEPROM and pre-compute calibration parameters.
frame = dev.read_frame()        # Read a frame from MLX90640
                                # In case EVB90640 hw is used, the EVB will buffer up to 4 frames, so possibly you get a cached frame.
f = dev.do_compensation(frame)  # calculates the temperatures for each pixel
```

## Issues and new Features

In case you have any problems with usage of the plugin, please open an issue on GitHub.  
Provide as many valid information as possible, as this will help us to resolve Issues faster.  
We would also like to hear your suggestions about new features which would help your Continuous Integration run better.

## Raspberry pi 3B+

This driver is validated to work on a Rapberry Pi 3B+ configuration with raspbian buster february 2020 release.

### Installation

- `sudo raspi-config`
    - 'enable i2c' in interface; in case you want to connect MLX9064x on the I2C bus of RPi.
    - optional: 'enable ssh' in interface; now you can login remotely over the network.
- `sudo pip3 install virtualenv`
- optional: `sudo apt-get update`
- optional: `sudo apt-get install python3-opencv`
- optional: `sudo apt-get install qt5-default`
- optional: `sudo apt-get install libatlas-base-dev`
- optional: `sudo apt-get install python3-scipy`
- `cd <mlx90640-py>`
- `virtualenv --system-site-packages pyvenv`
- `. pyvenv/bin/activate`
- `pip install .`


#### Note:
- output of `sudo apt-get update` must look like:
```
Get:1 http://archive.raspberrypi.org/debian buster InRelease [25.2 kB]
Get:2 http://raspbian.raspberrypi.org/raspbian buster InRelease [15.0 kB]
Get:3 http://archive.raspberrypi.org/debian buster/main armhf Packages [259 kB]
Get:4 http://raspbian.raspberrypi.org/raspbian buster/main armhf Packages [13.0 MB]
Fetched 13.3 MB in 17s (767 kB/s)
Reading package lists... Done
```


