# بسم اله الرحمن الرحیم - هست کلید در گنج حکیم

# opencvCalib
OpenCV chessboard geometric distortion calibration (not best for fisheye cameras)

inputs: default.xml file including information on the used calibration pattern and input video of different views of it

outputs: 
* log/calib.xml (output calibration result)
* log/path.csv (calculated path (position/angles) of the calibration pattern)
* log/screenshot000xx.bmp (used images in the calibration process)
* out_camera_data.xml (detailed output)

In this program, the emphasis is on the asymetric circular calibration pattern.

For fisheye cameras, other calibration patterns like opencv "charuco" or kalibr "april" is also widely used.

By my experience, agisoft-lens "wide chess" pattern is a better choice(if focus in close range is not a problem).

This routine derives the calibration(intrinsic parameters) and path(extrinsic parameters) of the calibration pattern simultaneously.

To derive the chessboard path.csv using external calibration results, first use this routine to derive selected images from the chessboard video, then use chessPoseTracker.


