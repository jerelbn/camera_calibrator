FROM ubuntu:22.04

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
        git \
        lsb-release \
        language-pack-en-base \
        # Python
        python3-dev \
        python3-pip \
        python3-numpy \
        # Camera interfaces
        libv4l-dev \
        v4l-utils \
        # Graphical
        libgtk-3-dev \
        freeglut3-dev \
        # Optimization libraries
        libatlas-base-dev \
        libblas-dev \
        liblapack-dev \
        libsuitesparse-dev \
        libcxsparse3 \
        gfortran \
        libeigen3-dev \
        libopencv-dev \
        # Optional
        doxygen \
        ccache \
        libcanberra-gtk-module \
        libcanberra-gtk3-module \
    && apt autoremove -yq \
    && apt clean -yq \
    && rm -rf /var/lib/apt/lists/*

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
