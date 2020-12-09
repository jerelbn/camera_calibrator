FROM ubuntu:18.04

# Temporarily disable interactive installs
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt update \
    && apt install -yq \
        # Generic
        sudo \
        bash-completion \
        wget \
        build-essential \
        cmake \
        pkg-config \
        unzip \
        yasm \
        git \
        checkinstall \
        lsb-release \
        language-pack-en-base \
        # Python
        python-pip \
        python-numpy \
        python3-dev \
        python3-pip \
        python3-numpy \
        # Image I/O
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        # Video
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libavresample-dev \
        libxvidcore-dev \
        x264 \
        libx264-dev \
        libfaac-dev \
        libmp3lame-dev \
        libtheora-dev  \
        libfaac-dev \
        libmp3lame-dev \
        libvorbis-dev \
        # GStreamer
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer1.0-0 \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        gstreamer1.0-doc \
        gstreamer1.0-tools \
        gstreamer1.0-x \
        gstreamer1.0-alsa \
        gstreamer1.0-gl \
        gstreamer1.0-gtk3 \
        gstreamer1.0-qt5 \
        gstreamer1.0-pulseaudio \
        # Speech codecs
        libopencore-amrnb-dev \
        libopencore-amrwb-dev \
        # Camera interfaces
        libdc1394-22 \
        libdc1394-22-dev \
        libxine2-dev \
        libv4l-dev \
        v4l-utils \
        # Graphical
        libgtk-3-dev \
        freeglut3-dev \
        libnvidia-gl-450 \
        # Parallelism library for CPU
        libtbb-dev \
        # Optimization libraries
        libatlas-base-dev \
        libblas-dev \
        liblapack-dev \
        libsuitesparse-dev \
        libcxsparse3 \
        gfortran \
        # Optional
        libprotobuf-dev \
        protobuf-compiler \
        libgoogle-glog-dev \
        libgflags-dev \
        libgphoto2-dev \
        libhdf5-dev \
        doxygen \
        ccache \
        libcanberra-gtk-module \
        libcanberra-gtk3-module \
    && apt autoremove -yq \
    && apt clean -yq \
    && rm -rf /var/lib/apt/lists/*

# Download and install OpenCV
RUN wget -q https://github.com/opencv/opencv/archive/4.4.0.zip -O opencv.zip \
    && unzip -q opencv.zip \
    && rm -rf opencv.zip
RUN wget -q https://github.com/opencv/opencv_contrib/archive/4.4.0.zip -O opencv_contrib.zip \
    && unzip -q opencv_contrib.zip \
    && rm -rf opencv_contrib.zip

RUN mkdir opencv-4.4.0/build \
    && cd /opencv-4.4.0/build \
    && cmake \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D INSTALL_PYTHON_EXAMPLES=OFF \
        -D INSTALL_C_EXAMPLES=OFF \
        -D WITH_TBB=ON \
        -D ENABLE_FAST_MATH=1 \
        -D WITH_V4L=ON \
        -D WITH_QT=OFF \
        -D WITH_OPENGL=ON \
        -D WITH_GSTREAMER=ON \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D OPENCV_PC_FILE_NAME=opencv.pc \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D OPENCV_EXTRA_MODULES_PATH=/opencv_contrib-4.4.0/modules \
        -D BUILD_EXAMPLES=OFF ..

RUN cd /opencv-4.4.0/build \
    && make -j $(nproc)

RUN cd /opencv-4.4.0/build \
    && make install \
    && cd / \
    && rm -rf opencv-4.4.0 opencv_contrib-4.4.0

# Enable interactive installs
ARG DEBIAN_FRONTEND=interactive

# Add non root user
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && adduser $USERNAME sudo \
    && adduser $USERNAME dialout \
    && adduser $USERNAME plugdev \
    && echo "user:user" | chpasswd

# Startup the user and bash
USER $USERNAME
ENV SHELL /bin/bash

# Show git branch in terminal
RUN echo "\n# Show git branch in terminal\nexport GIT_PS1_SHOWDIRTYSTATE=1" \
         "\nsource /usr/lib/git-core/git-sh-prompt" \
         "\nexport PS1='\\\[\\\033[01;36m\\\]\\\u\\\[\\\033[01;32m\\\]" \
         "\\\W\\\[\\\033[00;31m\\\]\$(__git_ps1)\\\[\\\033[01;36m\\\] \\\$\\\[\\\033[01;0m\\\] '" \
         >> ~/.bashrc
