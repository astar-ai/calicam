# CaliCam: Calibrated Fisheye Stereo & Mono Camera

<p align="center">
  <img src="http://astar.support/dotai/CaliCam_How_Stereo.jpg">
</p>

For more information see
[https://astar.ai](https://astar.ai).

Youtube [Demo Video](https://www.youtube.com/watch?v=pBh3_6uaY0s).

The following steps have been tested and passed on Ubuntu **16.04**.

### 1. Theoretical Background

Fisheye Camera Model:
C. Mei and P. Rives, Single View Point Omnidirectional Camera Calibration From Planar Grids, ICRA 2007.

### 2. OpenCV Dependencies

Required at leat 3.0. Tested with OpenCV 3.4.0.

### 3. Run C++ Code
#### Compile
	mkdir build && cd build
	cmake ..
	make
#### Run
	./calicam

### 4. Run Python Code

	python calicam.py

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
To run CaliCam in a live mode, please change the variable live to true. Python code does **NOT** support the live mode.

	bool      live = true;

and run

	./calicam YOUR_CALIBRATION_FILE.yml

<!---
**Note: [A\*SLAM](https://github.com/astar-ai/aslam) can run with ANY CaliCam stereo camera under the TRIAL mode.**

In TRIAL mode, the map cannot be saved to or loaded from the hard drive.

### 8. Reference: A Visualized Course of Linear Algebra

**Inverse of Matrix**
<p align="center">
  <img src="http://astar.support/dotai/inv.gif">
</p>

**Eigenvector**
<p align="center">
  <img src="http://astar.support/dotai/eigen.gif">
</p>

**Singular Vector Decomposition**
<p align="center">
  <img src="http://astar.support/dotai/svd.gif">
</p>

For more information see
[http://linear-algebra.org](http://linear-algebra.org/en/index.html).
--->
