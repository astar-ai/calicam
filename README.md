# CaliCam: Calibrated Fisheye Camera (Stereo & Mono)


For more information see
[https://astar.ai](https://astar.ai).

Youtube [Demo Video](https://www.youtube.com/watch?v=pBh3_6uaY0s).

The following steps have been tested and passed on Ubuntu **16.04.5**.

### 1. Theoretical Background

Fisheye Camera Model:
C. Mei and P. Rives, Single View Point Omnidirectional Camera Calibration From Planar Grids, ICRA 2007.

### 2. OpenCV Installation

#### 2.1 Required Dependencies

	sudo apt-get install git cmake build-essential pkg-config libgtk2.0-dev libssl-dev libv4l-dev v4l-utils libjpeg-dev libtiff5-dev libjasper-dev libpng-dev libavcodec-dev libavformat-dev libswscale-dev libxvidcore-dev libx264-dev libxine2-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libqt4-dev libgtk-3-dev mesa-utils libgl1-mesa-dri libqt4-opengl-dev libatlas-base-dev gfortran libeigen3-dev python2.7-dev python3-dev python-numpy python3-numpy 

#### 2.2 Install OpenCV 3.0 (or Above)

	cd
	mkdir library
	cd library

	git clone https://github.com/opencv/opencv_contrib.git
	cd opencv_contrib
	git checkout tags/3.0.0
	cd ..

	git clone https://github.com/opencv/opencv.git
	cd opencv
	git checkout tags/3.0.0
	mkdir build
	cd build
	cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
	make -j
	sudo make install
	
	echo 'export LD_LIBRARY_PATH=LD_LIBRARY_PATH:/lib:/usr/lib:/usr/local/lib' >> ~/.bashrc
	source ~/.bashrc

### 3. Compile

	git clone https://github.com/astar-ai/calicam.git
	cd calicam
	chmod 777 ./compile.sh
	./compile.sh

### 4. Run

	./calicam

### 5. Calibration Parameter File
To run CaliCam in the **LIVE** mode, you need to download the calibration parameter file from online.
Each CaliCam stereo/mono camera has a **UNIQUE** parameter file. Please download the corresponding parameter file by following the instructions at [https://astar.ai/collections/astar-products](https://astar.ai/collections/astar-products).

### 6. Operation

#### 6.1 'Raw Image' window
There are 3 trackbars to adjust the vertical **FoV**, **width**, and **height** for the output image.

#### 6.2 'Disparity Image' window
There are 2 trackbars to adjust the **numDisparities** and **blockSize** for [OpenCV stereo matching functions](https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereobm). 

#### 6.3 Exit
Press 'q' or 'Esc' key to exit.

### 7. Live Mode
To run CaliCam in a live mode, please change the variable live to true:

	bool      live = true;

and run

	./calicam YOUR_CALIBRATION_FILE.yml

**Note: [A\*SLAM](https://github.com/astar-ai/aslam) can run with ANY CaliCam stereo camera under the TRIAL mode.**

In TRIAL mode, the map cannot be saved to or loaded from the hard drive.
