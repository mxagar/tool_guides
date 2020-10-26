# Aruco Tutorials: Marker Creation, Calibration (Charuco), Detection, Pose Estimation

This project tests the ARUCO module from OpenCV.
Code is taken from the samples in the OpenCV contrib repository:
`C:\OpenCV\git_repos\opencv_contrib\modules\aruco\samples`

[Tutorial Overview](https://docs.opencv.org/master/d9/d6d/tutorial_table_of_content_aruco.html):

1. [Detection of ArUco Markers (& Creation of Markers)](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)

2. [Detection of ArUco Boards](https://docs.opencv.org/master/d9/d6d/tutorial_table_of_content_aruco.html)

3. [Detection of ChArUco Corners (& Creation of Boards)](https://docs.opencv.org/master/df/d4a/tutorial_charuco_detection.html)

4. [Detection of Diamond Markers](https://docs.opencv.org/master/d5/d07/tutorial_charuco_diamond_detection.html)

5. [Calibration with ArUco and ChArUco](https://docs.opencv.org/master/da/d13/tutorial_aruco_calibration.html)

6. [Aruco module FAQ](https://docs.opencv.org/master/d1/dcb/tutorial_aruco_faq.html)

## Basic procedure & program calls

Create markers:

```bash
.\create_marker.exe "marker23.png" -d=10 -id=23 -si=true
.\create_marker.exe "marker23.png" -d=10 -id=24 -si=true
.\create_marker.exe "marker23.png" -d=10 -id=25 -si=true
.\create_marker.exe "marker23.png" -d=10 -id=26 -si=true
.\create_marker.exe "marker23.png" -d=10 -id=27 -si=true
.\create_marker.exe "marker28.png" -d=10 -id=28 -si=true
```

... then, I manually arranged them in a PDF + print.

Detect printed markers (without pose):

``` bash
.\detect_markers -d=10
```

Create a Charuco board for calibration:

``` bash
.\create_board_charuco "charuco_board.png" -w=5 -h=7 -sl=200 -ml=120 -d=10 -si=true
```

Detect the printed Charuco board:

``` bash
.\detect_board_charuco -w=5 -h=7 -sl=200 -ml=120 -d=10
.\detect_board_charuco -dp="..\\..\\data\\detector_params.yml" -w=5 -h=7 -sl=200 -ml=120 -d=10
```

Calibrate the camera with the Charuco board: we get the cam params in camera_params.txt

``` bash
.\calibrate_camera_charuco "camera_params.txt" -dp="..\\..\\data\\detector_params.yml" -w=5 -h=7 -sl=200 -ml=120 -d=10
```

... then, I renamed and moved the TXT/YAML file to `data/`.

Detect Charuco board and its pose with camera params (calibrated):

``` bash
.\detect_board_charuco -c="..\\..\\data\\camera_params_logitech_c270.txt" -dp="..\\..\\data\\detector_params.yml" -w=5 -h=7 -sl=200 -ml=120 -d=10
```

Detect makers and their pose with camera params (calibrated):

``` bash
.\detect_markers -c="..\\..\\data\\camera_params_logitech_c270.txt" -d=10
```
