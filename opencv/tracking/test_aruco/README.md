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

Create markers

```bash
.\create_marker.exe "marker23.png" -d=10 -id=23 -si=true
.\create_marker.exe "marker23.png" -d=10 -id=24 -si=true
.\create_marker.exe "marker23.png" -d=10 -id=25 -si=true
.\create_marker.exe "marker23.png" -d=10 -id=26 -si=true
.\create_marker.exe "marker23.png" -d=10 -id=27 -si=true
.\create_marker.exe "marker28.png" -d=10 -id=28 -si=true
```

Then, I manually arrenged them in a PDF + print.

Detect printed markers (without pose)

```bash
.\detect_markers -d=10
```

Create a Charuco board for calibration

```bash
.\create_board_charuco "charuco_board.png" -w=5 -h=7 -sl=200 -ml=120 -d=10 -si=true
    w: number of squares in X direction
    h: number of squares in Y direction
    sl: square side length in pixels
    ml: maker side length in pixels
    d: standard dictionary id (for the aruco markers between the squares)
```

Detect the printed Charuco board:

```bash
.\detect_board_charuco -w=5 -h=7 -sl="0.037" -ml="0.0225" -d=10
    w: number of squares in X direction
    h: number of squares in Y direction
    sl: square side length in meters: 3.7 cm; if floating point, use ""
    ml: maker side length in meters: 2.25 cm; if floating point, use ""
    d: standard dictionary id (for the aruco markers between the squares)
    
    NOTE: we can detect the Charuco plate with or without camera calibration parameters
    If calibrated, camera parameters must be passed
        -c="..\\..\\data\\calib.txt"
    Calibrated detection is more precise, and additionally the pose can be obtained

.\detect_board_charuco -dp="..\\..\\data\\detector_params.yml" -w=5 -h=7 -sl="0.037" -ml="0.0225" -d=10
    Same are before, but with detector parameters saved in a YAML file.
    Explanation of detector params:
    https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
    An example of the file is in
    ...\opencv_contrib\modules\aruco\samples\detector_params.yml
```

Calibrate the camera with the Charuco board: we get the cam params in camera_params.txt:

```bash
.\calibrateCamera
    all options are shown

.\calibrateCamera "camera_params.txt" -w=5 -h=7 -sl="0.037" -ml="0.0225" -d=10
.\calibrate_camera_charuco.exe "camera_params.txt" -w=5 -h=7 -sl="0.036" -ml="0.0215" -d=10
    w: number of squares in X direction
    h: number of squares in Y direction
    sl: square side length in meters: 3.7 cm; if floating point, use ""
    ml: maker side length in meters: 2.25 cm; if floating point, use ""
    d: standard dictionary id (for the aruco markers between the squares)

    Process:
        1. We record images where the Charuco borad is visible
        hitting c, ESC to stop
        2. Calibration is performed with recorded images:
        markers & corners are detected and camera parameters drived
        3. Output file is written to disk: camera parameters

        We obtain a "camera_params.txt" file with the intrinsic cam params:
            image w, h
            cameraMatrix: 3x3
            distCoeffs: 1x5
            average reprojection error
            YAML format, so we could save it as "camera_params.yml"
                %YAML:1.0
                ---
                calibration_time: "Mon Mar  1 15:13:22 2021"
                image_width: 640
                image_height: 480
                flags: 0
                camera_matrix: !!opencv-matrix
                    rows: 3
                    cols: 3
                    dt: d
                    data: [ 8.3471154598558451e+02, 0., 3.0134459862617894e+02, 0.,
                        8.3225261010847692e+02, 2.3526876916667248e+02, 0., 0., 1. ]
                distortion_coefficients: !!opencv-matrix
                    rows: 1
                    cols: 5
                    dt: d
                    data: [ -9.2873893812588384e-02, 1.9195097946982878e+00,
                        4.6497855133461545e-04, -8.5207499488468650e-03,
                        -1.1956186282075466e+01 ]
                avg_reprojection_error: 5.2780606891873671e-01

        then, I renamed and moved the TXT/YAML file to data/
```

Detect Charuco board and its pose with camera params (calibrated):

```bash
.\detect_board_charuco -c="..\\..\\data\\camera_params.txt" -dp="..\\..\\data\\detector_params.yml" -w=5 -h=7 -sl="0.037" -ml="0.0225" -d=10
    Explanation of detector params:
    https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
    An example of the file is in
    ...\opencv_contrib\modules\aruco\samples\detector_params.yml
```

Detect makers and their pose with camera params (calibrated):

```bash
.\detect_markers -c="..\\..\\data\\camera_params.txt" -d=10 -l="0.0525"
    markers of standard dictionary 10 searched
    but camera calibration parameters used
    and marker side length 5.25 cm; this value is necessary for the correct scaling!
```