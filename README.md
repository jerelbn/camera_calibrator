# Camera Calibrator
This is used to calibrate USB cameras - intrinsic and stereo extrinsic calibrations.

## Setup
1. Install Docker - https://docs.docker.com/engine/install/ubuntu/
1. Install Visual Studio Code - https://code.visualstudio.com/download
1. Install local Nvidia drivers
    * For OpenGL Nvidia drivers you may need to change `Line 80` of `Dockerfile` to use the correct driver for your GPU
1. Install nvidia-docker2 - https://github.com/NVIDIA/nvidia-docker
1. To view gui windows, run `xhost +` on the host

## Run
1. Create `build` folder in base directory.
1. Go to build folder `cd build`
1. Run CMake `cmake ..`
1. Build the exectuable `make -j$(nproc)`
1. Get usage instructions with `./camera_calibrator --help`

## Additional Info
1. These calibrations required either a flat chessboard pattern or asymmetric circles grid pattern
1. For thermal calibration:
    * Cut and sand blast sheet metal about the size of your laptop
    * Spray black chalk paint on this with the asymmetric circles pattern taped off (metal circles, black background)
    * Rest the metal on a running laptop and it will be heated enough for the pattern to be detected
