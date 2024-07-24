# fpvLiberator
Get Live Video from DJI HD FPV Goggles. C++ version!

## Compiling from source

Builds using [Cmake](https://cmake.org/download/) for easy compilation on any system. 

## Dependencies
- `libusb`
- `OpenCV`
- `ffmpeg`

## Usage

### ROS
> Admin privilage needed. Login as root using `su`
```
su
rosrun fpvout fpvout_node
```

### Standalone
- Build
```
mkdir -p standalone/build
cd standalone/build
cmake ..
make
```
- Execute
```
cd standalone/build
sudo ./fpvLiberator_shm
```
- Pipe version
```
./fpvLiberator | ffplay -i - -analyzeduration 1 -probesize 32 -sync ext
```
