FROM nvidia/cuda:11.7.0-cudnn8-devel-ubuntu20.04

LABEL maintainer="Kornel Mrozowski 01123923@pw.edu.pl"

RUN apt update && \
    apt upgrade -y && \
# tzdata
    ln -fs /usr/share/zoneinfo/Europe/Warsaw /etc/localtime && \
    apt-get install -y tzdata && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    apt install -y curl gnupg lsb-release sudo vim git ninja-build wget build-essential ffmpeg libeigen3-dev libglew-dev && \
# adding user ground
    adduser --disabled-password --gecos '' ground && \
    adduser ground sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
WORKDIR /home/ground
USER ground

RUN mkdir /home/ground/sdk && \
    sudo apt update && sudo apt install -y locales && \
    sudo locale-gen en_US en_US.UTF-8 && \
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    echo "export LANG=en_US.UTF-8" >> /home/ground/.bashrc
WORKDIR /home/ground/sdk

# Install ROS Noetic
RUN sudo bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&\
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    sudo apt update && \
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration && \
    sudo apt install -y ros-noetic-desktop-full ros-noetic-nmea-comms && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    sudo rosdep init && \
    rosdep update

RUN sudo apt-get update &&\
    sudo apt-get install -y software-properties-common &&\
    sudo add-apt-repository --update -y ppa:ubuntu-toolchain-r/test &&\
    sudo apt install -y \
 clang-12 \
 gcc-11 \
 g++-11 \
 libfmt-dev \
 bash-completion \
 dirmngr \
 htop \
 gdb \
 git-lfs \
 x11-apps \
 libgmp3-dev \
 pybind11-dev \
 gitk \
 meld \
 terminator

RUN wget -O opencv.zip https://codeload.github.com/opencv/opencv/zip/4.6.0 && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.6.0.zip && \
    unzip -qq opencv.zip && unzip -qq opencv_contrib.zip &&\
    mv opencv-4.6.0 opencv && \
    mv opencv_contrib-4.6.0 opencv_contrib && \
    rm opencv.zip opencv_contrib.zip && \
    cd opencv && \
    cmake \
 -D CMAKE_C_COMPILER=/usr/bin/gcc-11 \
 -D CMAKE_CXX_COMPILER=/usr/bin/g++-11 \
 -D CMAKE_BUILD_TYPE=RELEASE \
 -D CMAKE_INSTALL_PREFIX=/usr \
 -D OPENCV_EXTRA_MODULES_PATH=/home/ground/sdk/opencv_contrib/modules \
 -D OPENCV_ENABLE_NONFREE=ON \
 -D OPENCV_GENERATE_PKGCONFIG=ON \
 -D BUILD_LIST=\
# https://docs.opencv.org/4.x/index.html
core,\
imgcodecs,\
imgproc,\
calib3d,\
videoio,\
highgui,\
stereo,\
rgbd,\
shape,\
surface_matching,\
viz,\
ximgproc,\
cudev,cudaimgproc,\
cudaarithm,\
cudawarping,\
cudabgsegm,\
cudacodec,\
cudastereo \
 -D BUILD_SHARED_LIBS=OFF \
 -D WITH_GSTREAMER=ON \
 -D WITH_EIGEN=ON \
 -D WITH_V4L=ON \
 -D WITH_LIBV4L=ON \
 -D WITH_VTK=ON \
 -D WITH_OPENGL=ON \
 -D WITH_CUBLAS=ON \
 -D WITH_CUDA=ON \
 -D INSTALL_C_EXAMPLES=OFF \
 -D INSTALL_PYTHON_EXAMPLES=OFF \
 -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
 -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.7 \
 -D ENABLE_FAST_MATH=ON \
 -D CUDA_FAST_MATH=ON \
 -D CUDA_ARCH_BIN=5.3,7.2,8.6 \
 -D OPENCV_DNN_CUDA=ON \
 -D WITH_CUDNN=ON \
 -D CUDNN_LIBRARY=/usr/lib/x86_64-linux-gnu/libcudnn.so.8.4.0 \
 -D CUDNN_INCLUDE_DIR=/usr/include \
 -D BUILD_NEW_PYTHON_SUPPORT=ON \
 -D BUILD_TESTS=ON \
 -D BUILD_TBB=OFF \
 -D BUILD_opencv_python2=OFF \
 -D BUILD_opencv_python3=ON \
 -D BUILD_opencv_cudev=ON \
 -D CMAKE_BUILD_TYPE=RELEASE \
 -D BUILD_DOCS=OFF \
 -D BUILD_EXAMPLES=OFF \
 -D BUILD_opencv_apps=OFF \
 -D BUILD_PERF_TESTS=OFF \
 -D FORCE_VTK=OFF \
 -D opencv_cudev=ON \
 -D BUILD_JPEG=ON \
 -D BUILD_ZLIB=ON \
 -D BUILD_PNG=ON \
 -D BUILD_TIFF=ON \
 -D WITH_TIFF=ON \
 -D WITH_PNG=ON \
 -D WITH_CAROTENE=OFF \
 -D WITH_FFMPEG=OFF \
 -D WITH_GDAL=OFF \
 -D WITH_IPP=OFF \
 -D WITH_OPENEXR=OFF \
 -D WITH_OPENGL=OFF \
 -D WITH_QT=OFF \
 -D WITH_TBB=OFF \
 -D WITH_XINE=OFF \
 -D BUILD_BUILD_JASPER=OFF \
 -D WITH_ITT=OFF \
 -D WITH_LAPACK=OFF \
 -D WITH_OPENCL=OFF \
 -D WITH_OPENCLAMDFFT=OFF \
 -D WITH_OPENCLAMDBLAS=OFF \
 -D WITH_VA_INTEL=OFF \
 -D WITH_WEBP=OFF \
 -D WITH_JASPER=OFF \
 -G Ninja \
 -S . \
 -B build && \
    ninja -j$(nproc) -l$(nproc) -C build && \
    sudo ninja -C build install && \
    sudo ldconfig && \
    sudo apt-get update && \
    cd /home/ground/sdk/ && \
    sudo rm -r /home/ground/sdk/* && \
    echo "================= opencv installed !!! ===================" && \
    echo "================= opencv installed !!! ===================" && \
    echo "================= opencv installed !!! ==================="

ARG ONBOARD_SDK_VERSION

# install Onboard-SDK
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin &&\
    sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600 &&\
    wget https://developer.download.nvidia.com/compute/cuda/11.7.0/local_installers/cuda-repo-ubuntu2004-11-7-local_11.7.0-515.43.04-1_amd64.deb &&\
    sudo dpkg -i cuda-repo-ubuntu2004-11-7-local_11.7.0-515.43.04-1_amd64.deb &&\
    sudo cp /var/cuda-repo-ubuntu2004-11-7-local/cuda-*-keyring.gpg /usr/share/keyrings/ &&\
    sudo apt-get update &&\
    sudo apt-get -y install cuda

# RUN version=3.25 \
#     build=1 \
#     wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz &&\
#     tar -xzvf cmake-$version.$build.tar.gz &&\
#     cd cmake-$version.$build/ &&\
#     ./bootstrap &&\
#     make -j$(nproc) -l$(nproc) &&\
#     sudo make install &&\
#     cd .. &&\
#     rm -rf cmake-$version.$build/

# # PCL
RUN git clone https://github.com/PointCloudLibrary/pcl.git &&\
    cd pcl &&\
    cmake -S. -Bbuild -GNinja &&\
    cmake --build build &&\
    sudo ninja -C build install &&\
    cd .. &&\
    rm -rf pcl

# ORB_SLAM3
RUN sudo apt update &&\
    sudo apt install -y \
 libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols \
 libegl1-mesa-dev libc++-dev libjpeg-dev libpng-dev \
 libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev &&\
    git clone --recurse-submodules https://github.com/stevenlovegrove/Pangolin.git &&\
    cd Pangolin/ &&\
    git checkout v0.8 &&\
    cmake -S . -B build -G Ninja &&\
    ninja -j$(nproc) -l$(nproc) -C build &&\
    sudo sudo ninja -C build install &&\
    cd .. &&\
    rm -rf Pangolin
    # git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git && \
    # cd ORB_SLAM3 && \
    # find ./build.sh -type f -print0 | xargs -0 sed -i 's#CMAKE_BUILD_TYPE=Release#CMAKE_BUILD_TYPE=Release -D CMAKE_CXX_STANDARD=14 -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda -GNinja#g' && \
    # find ./build.sh -type f -print0 | xargs -0 sed -i 's#-j4#-j#g' && \
    # find ./build.sh -type f -print0 | xargs -0 sed -i 's#make -j#ninja -j4 -l4; cd ../; rm -r build#g' && \

RUN git clone https://github.com/dji-sdk/Onboard-SDK.git --single-branch --branch $ONBOARD_SDK_VERSION;\
    cd Onboard-SDK &&0\
    CUDA_HOME=/usr/local/cuda-11.7 \
 LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu \
 PATH=$PATH:$CUDA_HOME/bin \
 CC=/usr/bin/gcc-11 \
 cmake -G Ninja ..;\
    ninja -j$(nproc) -l$(nproc);\
    sudo ninja install;\
    cd ../..;\
    sudo rm -rf /root/sdk/* && \
    echo "================= Onboard-SDK installed !!! ===================" && \
    echo "================= Onboard-SDK installed !!! ===================" && \
    echo "================= Onboard-SDK installed !!! ==================="
    # chmod +x build.sh

# RUN sudo apt update &&\
#     sudo apt install -y nvidia-cuda-toolkit &&\
#     git clone https://github.com/fixstars/libSGM.git &&\
#     cd libSGM &&\
#     git checkout 3.0.0 &&\
#     find -type f -name CMakeLists.txt -print0 | xargs -0 sed -i 's#VERSION 3.18#VERSION 3.16#g' &&\
#     cmake \
#  -S. -Bbuild -GNinja \
#  -DCMAKE_CUDA_ARCHITECTURES=86 \
#  -DBUILD_OPENCV_WRAPPER=ON \
#  -DENABLE_SAMPLES=OFF \
#  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
#  -DCMAKE_INSTALL_PREFIX=/usr/local &&\
#     sudo ninja -C build install &&\
#     sudo apt remove -y nvidia-cuda-toolkit &&\
#     sudo rm -rf /root/sdk/*

# upgrade cmake 3.16 to 3.25
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null &&\
    sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" &&\
    sudo apt update &&\
    sudo apt install -y cmake clang-12

# add environmental variables and aliases
WORKDIR /home/ground/
ARG MASTER_URI
ARG ROS_IP
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc &&\
    echo "export ROS_MASTER_URI=${MASTER_URI}" >> ~/.bashrc &&\
    echo "export ROS_IP=${ROS_IP}" >> ~/.bashrc &&\
    mkdir repos
