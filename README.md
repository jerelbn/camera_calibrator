# Camera Calibrator
This is used to calibrate USB cameras - intrinsic and stereo extrinsic calibrations.

## Setup
1. Install Docker - https://docs.docker.com/engine/install/ubuntu/
1. Install Visual Studio Code - https://code.visualstudio.com/download
1. Install local Nvidia drivers
    * `RViz` requires OpenGL Nvidia drivers, so you may need to change `Line 80` of `Dockerfile` to use the correct driver for your GPU
1. Install nvidia-docker2 - https://github.com/NVIDIA/nvidia-docker
1. To view gui windows, run `xhost +` on the host

## Run
1. Create `build` folder in base directory.
1. Go to build folder `cd build`
1. Run CMake `cmake ..`
1. Build the exectuable `make -j$(nproc)`
1. Get usage instructions with `./camera_calibrator --help`
