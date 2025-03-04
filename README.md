# fpvLiberator
Get Live Video from DJI HD FPV Goggles. C++ version!

## Compiling from source

Build using [Cmake](https://cmake.org/download/) for easy compilation on any system. 

## Dependencies
- `libusb`
  ```bash
  sudo apt install libusb-1.0-0-dev
  ```
- `OpenCV` (any version)
- `ffmpeg`
  ```bash
  sudo apt install ffmpeg
  ```

## Usage
```
sudo ./fpvLiberator_shm
```
or
```
./fpvLiberator | ffplay -i - -analyzeduration 1 -probesize 32 -sync ext
```
