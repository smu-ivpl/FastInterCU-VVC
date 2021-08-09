# Fast algorithm for Inter Prediction using MLT-CNN 
This project is based on [VTM reference software for VVC](https://vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM) version 11.0.

## Environment
- Ubuntu 16.04
- NVIDIA Tesla V100 x2
- CUDA 10.1
- CMake 3.5.1
- OpenCV 4.0.0
- LibTorch 1.7.1

## Before build this project...
1. Download [LibTorch](https://pytorch.org/get-started/locally/) (*Check your CUDA version*)
	
	- Example for dowloading LibTorch with CUDA 10.1
	```
    wget https://download.pytorch.org/libtorch/cu101/libtorch-cxx11-abi-shared-with-deps-1.7.1%2Bcu101.zip
    unzip libtorch-cxx11-abi-shared-with-deps-1.7.1%2Bcu101.zip
    ```
2. Build OpenCV
	<details>
	<summary>Build instruction</summary>
    This instruction is from [sunkyoo's blog](https://sunkyoo.github.io/opencv4cvml/OpenCV4Linux.html).
    
    ```
    $ cd ~
	$ mkdir opencv
	$ cd opencv
    ```
    ```
    $ wget -O opencv-4.0.0.zip https://github.com/opencv/opencv/archive/4.0.0.zip
	$ wget -O opencv_contrib-4.0.0.zip https://github.com/opencv/opencv_contrib/archive/4.0.0.zip
    $ unzip opencv-4.0.0.zip
	$ unzip opencv_contrib-4.0.0.zip
    ```
    ```
    $ mkdir build
	$ cd build
    ```
    ```
    $ cmake \
        -D CMAKE_BUILD_TYPE=Release \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D BUILD_WITH_DEBUG_INFO=OFF \
        -D BUILD_EXAMPLES=ON \
        -D BUILD_opencv_python3=ON \
        -D INSTALL_PYTHON_EXAMPLES=ON \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.0.0/modules \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D WITH_TBB=ON \
        ../opencv-4.0.0/
    ```
    You can see the message like this,
    ```
    - Configuring done
	-- Generating done
	-- Build files have been written to: /home/user/opencv/build
    ```
    Then
    ```
    $ make -j
    ```
    ```
    $ sudo make install
	$ sudo ldconfig
    ```
    ```
    $ pkg-config --list-all | grep opencv
	opencv4                        OpenCV - Open Source Computer Vision Library
    ```
    
    </details>
3. Change Library Torch path in [CMakeLists.txt](https://github.com/smu-ivpl/FastInterCU-VVC/blob/main/vtm-mlt-cpp/CMakeLists.txt#L58)

    ```
    # LibTorch
    list(APPEND CMAKE_PREFIX_PATH "[YOUR LibTorch PATH]")
    find_package(Torch REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
    ```


## Build instructions
Please check [here](https://vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM#build-instructions).

