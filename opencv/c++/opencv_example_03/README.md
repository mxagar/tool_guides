# OpenCV: Camera Calibration

This example examines the calibration functionalities of OpenCV as collected in

https://docs.opencv.org/4.4.0/d6/d55/tutorial_table_of_content_calib3d.html

`gen_pattern.py, svgfig.py`

- utils for creating calibration patterns: chessboard, circles (a/symmetric)

`02_calibration.cpp -> calibration.exe`

- usable application
- we can capture images of the calibration plate live
and calibration is run + calibration file written
- rectification is also done

`01_camera_calibration.cpp -> camera_calibration.exe`

- from the tutorials
- a configuration file and a file with image paths are required
- a Settings object is instantiated and we query nextImage() from it in a loop;
- apart from it, the code seems to be similar yp 02_calibration.cpp

`image_creator.cpp, image_reader.cpp`

- utils for creating and reading
- files that contain lists of image filenames as XML/YAML, ...

    `.\imagelist_creator imagelist.yaml *.png`

    `imagelist.yaml` is created with all `*.png` filenames in folder

    `.\imagelist_reader.exe image_list.yaml`
    
    images from `image_list.yaml` are read and displayed