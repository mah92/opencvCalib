//ALLAH
//Inputs: default.xml, describing the input video
//outputs: out_camera_data.xml, log/calib.xml, log/path.csv, ...

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core/core.hpp>
//#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "./Settings.h"

using namespace cv;
using namespace std;

double normalize_angle(double angle){
    while(angle > M_PI) angle -= 2*M_PI;
    while(angle <= -M_PI) angle += 2*M_PI;
    return angle;
}

//verified, output in rad
void CI2B2Eu(Mat& CI2B, double &phi, double &theta, double &psi) 
{
	float a11 = CI2B.at<double>(0, 0);
    float a12 = CI2B.at<double>(0, 1);
    float a13 = CI2B.at<double>(0, 2);

	float a21 = CI2B.at<double>(1, 0);
    float a22 = CI2B.at<double>(1, 1);
    float a23 = CI2B.at<double>(1, 2);

	float a31 = CI2B.at<double>(2, 0);
    float a32 = CI2B.at<double>(2, 1);
    float a33 = CI2B.at<double>(2, 2);
	
	// gamma:phi:eu[0], beta: theta:eu[1], alpha:psi:eu[2],
	double s = sqrt(a11*a11 + a12*a12);
	if(s < 1e-3) {
		phi = atan2(a21, a22);// * 180. / CV_PI;
		theta = CV_PI/2;
		psi = 0;
	} else {
		phi = atan2(a23, a33);// * 180. / CV_PI;
		//theta = atan2(-a13, s);// * 180. / CV_PI; //khodam
		theta = asin(-a13);// * 180. / CV_PI; //javad
		psi = atan2(a12, a11);// * 180. / CV_PI;
	}

}

static void help()
{
    cout <<  "This is a camera calibration sample." << endl
	  <<  "Usage: camera_calibration [configuration_file -- default ./default.xml]"  << endl
	   <<  "Near the sample file you'll find the configuration file, which has detailed help of "
	       "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}

//used by opencv itself
static inline void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
	x = default_value;
    else
	x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
			    vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
			    vector<float>& reprojErrs,  double& totalAvgErr);
// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
			      const vector<Mat>& rvecs, const vector<Mat>& tvecs,
			      const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
			      double totalAvgErr );

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
			   vector<vector<Point2f> > imagePoints );

vector<double> imageTimes;
char address_str[200];

int main(int argc, char* argv[])
{
    help();

    //file_read///////////////////////////////////////////
    Settings s;
    const string inputSettingsFile = argc > 1 ? argv[1] : "default.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened()) {
		cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
		return -1;
    }

    fs["Settings"] >> s;
    fs.release();                                         // close Settings file

    if (!s.goodInput) {
		cout << "Invalid input detected. Application stopping. " << endl;
		return -1;
    }

    /////////////////////////////////////////////////////////

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    //! [get_input]
    for(;;) {
		Mat view;
		bool blinkOutput = false;

		view = s.getNextImage();
		static int imageIndex = -1;
		imageIndex++;

		//-----  If no more image, or got enough, then stop calibration and show result -------------
		if( mode == CAPTURING && imagePoints.size() >= (size_t)s.nrFrames ) {
			if( runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints))
				mode = CALIBRATED;
			else
				mode = DETECTION;
		}
		
		if(view.empty()){          // If there are no more images stop the loop
			// if calibration threshold was not reached yet, calibrate now
			if( mode != CALIBRATED && !imagePoints.empty() )
			runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
			break;
		}
		//! [get_input]

		imageSize = view.size();  // Format input image.
		if( s.flipVertical )    flip( view, view, 0 );

		//! [find_pattern]
		vector<Point2f> image_points;

		bool found;

		int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

		if(!s.useFisheye) {
			// fast check erroneously fails with high distortions like fisheye
			chessBoardFlags |= CALIB_CB_FAST_CHECK;
		}

		switch( s.calibrationPattern ) {// Find feature points on the input format
		case Settings::CHESSBOARD:
			found = findChessboardCorners( view, s.boardSize, image_points, chessBoardFlags);
			break;
		case Settings::CIRCLES_GRID:
			found = findCirclesGrid( view, s.boardSize, image_points );
			break;
		case Settings::ASYMMETRIC_CIRCLES_GRID:
			found = findCirclesGrid( view, s.boardSize, image_points, CALIB_CB_ASYMMETRIC_GRID );
			break;
		default:
			found = false;
			break;
		}

		//printf("(%f, %f)\n", image_points[0].x, image_points[0].y);

		//! [find_pattern]
		//! [pattern_found]
		if ( found) {                // If done with success,
			// improve the found corners' coordinate accuracy for chessboard
			if( s.calibrationPattern == Settings::CHESSBOARD){
				Mat viewGray;
				cvtColor(view, viewGray, COLOR_BGR2GRAY);
				cornerSubPix( viewGray, image_points, Size(11,11),
					  Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
			}

			if( mode == CAPTURING &&  // For camera only take new samples after delay time
				(!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) ){
				imageTimes.push_back(imageIndex*s.timeStep);
				imagePoints.push_back(image_points);
				sprintf(address_str, "./log/screenshot%05d.bmp", imagePoints.size()-1);
				imwrite(std::string(address_str), view);
				prevTimestamp = clock();
				blinkOutput = s.inputCapture.isOpened();
			}

			// Draw the corners.
			drawChessboardCorners( view, s.boardSize, Mat(image_points), found );
		}

		//! [pattern_found]
		//----------------------------- Output Text ------------------------------------------------
		//! [output_text]
		string msg = (mode == CAPTURING) ? "100/100" :
						   mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
		int baseLine = 0;
		Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
		Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

		if( mode == CAPTURING ) {
			if(s.showUndistorsed)
				msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
			else
				msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
		}

		putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

		if( blinkOutput )
			bitwise_not(view, view);
		//! [output_text]
		//------------------------- Video capture  output  undistorted ------------------------------
		//! [output_undistorted]
		if( mode == CALIBRATED && s.showUndistorsed ) {
			Mat temp = view.clone();
			if (s.useFisheye)
				cv::fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs);
			else
				undistort(temp, view, cameraMatrix, distCoeffs);
		}
		//! [output_undistorted]
		//------------------------------ Show image and check for input commands -------------------
		//! [await_input]
		imshow("Image View", view);
		char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

		if( key  == ESC_KEY )
			break;

		if( key == 'u' && mode == CALIBRATED )
			s.showUndistorsed = !s.showUndistorsed;

		if( s.inputCapture.isOpened() && key == 'g' ) {
			mode = CAPTURING;
			imagePoints.clear();
		}
		//! [await_input]
	}

	// -----------------------Show the undistorted image for the image list ------------------------
	//! [show_results]
	if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed ) {
		Mat view, rview, map1, map2;

		if (s.useFisheye) {
			Mat newCamMat;
			fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
																		Matx33d::eye(), newCamMat, 1);
			fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize,
													 CV_16SC2, map1, map2);
		} else {
			initUndistortRectifyMap(
				cameraMatrix, distCoeffs, Mat(),
				getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
					CV_16SC2, map1, map2);
		}

		for(size_t i = 0; i < s.imageList.size(); i++ ) {
			view = imread(s.imageList[i], IMREAD_COLOR);
			if(view.empty())
			continue;
			remap(view, rview, map1, map2, INTER_LINEAR);
			imshow("Image View", rview);
			char c = (char)waitKey();
			if( c  == ESC_KEY || c == 'q' || c == 'Q' )
				break;
		}
    }
    //! [show_results]

    return 0;
}

//! [compute_errors]
static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
					 const vector<vector<Point2f> >& imagePoints,
					 const vector<Mat>& rvecs, const vector<Mat>& tvecs,
					 const Mat& cameraMatrix , const Mat& distCoeffs,
					 vector<float>& perViewErrors, bool fisheye)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i ) {
		if (fisheye) {
			fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix, distCoeffs);
		} else {
			projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
		}
		err = norm(imagePoints[i], imagePoints2, NORM_L2);

		size_t n = objectPoints[i].size();
		perViewErrors[i] = (float) std::sqrt(err*err/n);
		totalErr        += err*err;
		totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

//! [compute_errors]
//! [board_corners]
//in asymetric pattern, most negative point is the point opposite of text, x in the shortest direction towards text
//0,0,0 is center of picture(circles, not A4)
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
									 int patternType = Settings::CHESSBOARD)
{
    corners.clear();

    switch(patternType) {
			
	//original, switched x, y so that x, y become normal as other images and z becomes inward
	/*case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
		for( int i = 0; i < boardSize.height; i++ )
        	for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((i % 2 + 2*j)*squareSize),
                                          float(i*squareSize), 0));
		break;
	*/
			
	//moved origin to corner-center of image
	case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
		for( int i = 0; i < boardSize.height; i++ )
        	for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID: 
	//Important Note: In assymetric circular pattern, 
	//square size is horizontal-vertical distance between circles in diagons,
	//half the distance of circles in rows-cols
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                //corners.push_back(Point3f(float((i % 2 + 2*j)*squareSize), float(i*squareSize), 0)); //corner(one of circles)
                //corners.push_back(Point3f(float((2*1.21875 + i % 2 + 2*j)*squareSize), float((2*1.58482 + i)*squareSize), 0)); //corner(opposite of text, image corner)
				corners.push_back(Point3f(float((-3.5 + i % 2 + 2*j)*squareSize), float((-5 + i)*squareSize), 0)); //center
		break;
			
    default:
		break;
    }
}
/////////////////////////////////////////////////////////////

//! [calls runCalibration and saveCameraParams]
bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
			   vector<vector<Point2f> > imagePoints)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
			     totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
	 << ". avg re projection error = " << totalAvgErr << endl;

    if (ok)
	saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
			 totalAvgErr);
    return ok;
}

//! [board_corners]
static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
			    vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
			    vector<float>& reprojErrs,  double& totalAvgErr)
{
    //! [fixed_aspect]
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CALIB_FIX_ASPECT_RATIO )
	cameraMatrix.at<double>(0,0) = s.aspectRatio;
    //! [fixed_aspect]
    if (s.useFisheye) {
		distCoeffs = Mat::zeros(4, 1, CV_64F);
	} else {
    	distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
    objectPoints.resize(imagePoints.size(),objectPoints[0]);


    //Find intrinsic and extrinsic camera parameters
    double rms;

    if (s.useFisheye) {
		Mat _rvecs, _tvecs;
		rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
								 _tvecs, s.flag);

		rvecs.reserve(_rvecs.rows);
		tvecs.reserve(_tvecs.rows);
		for(int i = 0; i < int(objectPoints.size()); i++){
			rvecs.push_back(_rvecs.row(i));
			tvecs.push_back(_tvecs.row(i));
		}
	} else {
    	rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
			  s.flag);
    }

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
					    distCoeffs, reprojErrs, false/*s.useFisheye*/);

    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
			      const vector<Mat>& rvecs, const vector<Mat>& tvecs,
			      const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
			      double totalAvgErr )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf), "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
	fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;

    if( s.flag & CALIB_FIX_ASPECT_RATIO )
	fs << "fix_aspect_ratio" << s.aspectRatio;

    if (s.flag)
    {
	std::stringstream flagsStringStream;
	if (s.useFisheye)
		{
			flagsStringStream << "flags:"
				<< (s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
				<< (s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
				<< (s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
				<< (s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
				<< (s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
				<< (s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
		} else {
	flagsStringStream << "flags:"
			  << (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
			  << (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
			  << (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
			  << (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
			  << (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
			  << (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
			  << (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
			  << (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
			  << (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
	}
	//fs.writeComment(flagsStringStream.str());
	printf("flagsStringStream:%s\n", flagsStringStream.str().c_str());
    }

    fs << "flags" << s.flag;

    fs << "fisheye_model" << s.useFisheye;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;


    double ox = cameraMatrix.at<double>(0,2);
    double oy = cameraMatrix.at<double>(1,2);
    double f = cameraMatrix.at<double>(0,0);
    double fovX = 2*atan(((imageSize.width-1.)/2.)/f)*180./M_PI;
    double fovY = 2*atan(((imageSize.height-1.)/2.)/f)*180./M_PI;


    FILE *calibFile;
    calibFile = fopen("./log/calib.xml", "w");
    fprintf(calibFile, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    fprintf(calibFile, "<simParams>\n");
    fprintf(calibFile,
	    "<!-- ox: %f, oy:%f, fovX:%f, fovY:%f -->\n"
	    "<idealParams resolutionScale=\"1.\" width=\"%i\" height=\"%i\" oxOffset=\"%f\" oyOffset=\"%f\" f=\"%f\" />\n"
	    "<staticParams  maxLensIterations=\"20\" maxPixError=\"0.1\" k1=\"%f\" k2=\"%f\" t1=\"%f\" t2=\"%f\" k3=\"%f\" k4=\"%f\" k5=\"%f\" k6=\"%f\" />\n",
	    ox, oy, fovX, fovY,
	    imageSize.width, imageSize.height,
	    ox - (imageSize.width/2.-0.5), oy - (imageSize.height/2.-0.5),
	    f,
	    s.fixK1?0:distCoeffs.at<double>(0,0), s.fixK2?0:distCoeffs.at<double>(0,1),
			s.calibZeroTangentDist?0:distCoeffs.at<double>(0,2), s.calibZeroTangentDist?0:distCoeffs.at<double>(0,3),
	    		s.fixK3?0:distCoeffs.at<double>(0,4), s.fixK4?0:distCoeffs.at<double>(0,5), s.fixK5?0:distCoeffs.at<double>(0,6), s.fixK5?0:distCoeffs.at<double>(0,7));
    fprintf(calibFile, "</simParams>\n");
    fclose(calibFile);

    fs << "avg_reprojection_error" << totalAvgErr;
    if (s.writeExtrinsics && !reprojErrs.empty())
	fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    //.csv////////////////////////////////////////////////////////////
    if(s.writeExtrinsics && !rvecs.empty() && !tvecs.empty() )
    {
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
		Mat CP2C(3, 3, CV_64FC1); //pattern(inertia) to camera = CI2C = CI2B
		Mat CC2P(3, 3, CV_64FC1); //camera to pattern(inertia) = CC2I = CB2I
		double phi, theta, psi;
		double s;

		bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
		bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

		for( size_t i = 0; i < rvecs.size(); i++ )
		{
			Mat r = bigmat(Range(int(i), int(i+1)), Range(0,3));
			Mat t = bigmat(Range(int(i), int(i+1)), Range(3,6));

			/////////////////////////////////////////
			//rvec & tvec are parts of 4x4 homgenuous matrix transferring pointsfrom world to cam coordinates
			//so rvec:=CP2C, tvec:=(Ptrn_origin - Cam_origin)| in cam coords (cam origin at center of lens)
			//Verified by experiment
			//see: https://docs.opencv.org/3.1.0/dc/d2c/tutorial_real_time_pose.html
			//     https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/src/PnPProblem.cpp
			//note: "Learning OpenCV3" by Bradsky is misleading!!!			

			if(needReshapeR)
				rvecs[i].reshape(1, 1).copyTo(r);
			else {
				//*.t() is MatExpr (not Mat) so we can use assignment operator
				CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
				r = rvecs[i].t();
			}

			//r is rotation mat of camera in chess board coordinates
			//experimentally verified, also noted by Bradsky in "Learning OpenCV3"
			Rodrigues(r, CP2C);
			CI2B2Eu(CP2C, phi, theta, psi); //extrinsic params, pose of camera in pattern(inertia)
			//printf("%i: cam in inertia: %.0f, %.0f, %.0f  ", i, phi*180./M_PI, theta*180./M_PI, psi*180./M_PI);
			
			r.at<double>(0,0) = phi*180./M_PI;
			r.at<double>(0,1) = theta*180./M_PI;
			r.at<double>(0,2) = psi*180./M_PI;

			CC2P = CP2C.t();
			//CI2B2Eu(CC2P, phi, theta, psi); //inertia(pattern) in camera
			//printf("<->  %.0f, %.0f, %.0f: pattern in cam\n", phi*180./M_PI, theta*180./M_PI, psi*180./M_PI);

			//////////////////////////////////////////
			if(needReshapeT)
				tvecs[i].reshape(1, 1).copyTo(t);
			else {
				CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
				t = tvecs[i].t();
			}
			
			t = (CC2P*(-t.t())).t(); //convert to cam in pattern
			
			//printf("%i: cam in inertia: %f, %f, %f\n", i, t.at<double>(0,0), t.at<double>(0,1), t.at<double>(0,2));
		}
		
		//fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
		fs << "extrinsic_parameters" << bigmat;

		int len = imageTimes.size();

		FILE *pathFile;
		pathFile = fopen("./log/path.csv", "w");
		for(int j = 0; j < len; j++) {
			fprintf(pathFile, "%f, %f, %f, %f, %f, %f, %f\n",
						imageTimes[j], bigmat.at<double>(j,3), bigmat.at<double>(j,4), bigmat.at<double>(j,5),
							bigmat.at<double>(j,0), bigmat.at<double>(j,1), bigmat.at<double>(j,2));
		}
		fclose(pathFile);
    }

    //Write raw image points/////////////////////////////////////////////////////////
    if(s.writePoints && !imagePoints.empty() ) {
	Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
	for( size_t i = 0; i < imagePoints.size(); i++ ) {
	    Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
	    Mat imgpti(imagePoints[i]);
	    imgpti.copyTo(r);
	}
	fs << "image_points" << imagePtMat;
    }
}
