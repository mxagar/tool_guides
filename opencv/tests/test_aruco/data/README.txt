Create markers
	.\create_marker.exe "marker23.png" -d=10 -id=23 -si=true
	.\create_marker.exe "marker23.png" -d=10 -id=24 -si=true
	.\create_marker.exe "marker23.png" -d=10 -id=25 -si=true
	.\create_marker.exe "marker23.png" -d=10 -id=26 -si=true
	.\create_marker.exe "marker23.png" -d=10 -id=27 -si=true
	.\create_marker.exe "marker28.png" -d=10 -id=28 -si=true

	then, I manually arrenged them in a PDF + print

Detect printed markers (without pose)
	.\detect_markers -d=10

Create a Charuco board for calibration
	.\create_board_charuco "charuco_board.png" -w=5 -h=7 -sl=200 -ml=120 -d=10 -si=true

Detect the printed Charuco board
	.\detect_board_charuco -w=5 -h=7 -sl=200 -ml=120 -d=10
	.\detect_board_charuco -dp="..\\..\\data\\detector_params.yml" -w=5 -h=7 -sl=200 -ml=120 -d=10

Calibrate the camera with the Charuco board: we get the cam params in camera_params.txt
	.\calibrate_camera_charuco "camera_params.txt" -dp="..\\..\\data\\detector_params.yml" -w=5 -h=7 -sl=200 -ml=120 -d=10

	then, I renamed and moved the TXT/YAML file to data/

Detect Charuco board and its pose with camera params (calibrated):
	.\detect_board_charuco -c="..\\..\\data\\camera_params_logitech_c270.txt" -dp="..\\..\\data\\detector_params.yml" -w=5 -h=7 -sl=200 -ml=120 -d=10

Detect makers and their pose with camera params (calibrated):
	.\detect_markers -c="..\\..\\data\\camera_params_logitech_c270.txt" -d=10
