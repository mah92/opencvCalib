#include "./Settings.h"



void Settings::read(const FileNode& node)                          //Read serialization for this class
{
    node["BoardSize_Width" ] >> boardSize.width;
    node["BoardSize_Height"] >> boardSize.height;
    node["Calibrate_Pattern"] >> patternToUse;
    node["Square_Size"]  >> squareSize;
    node["Calibrate_NrOfFrameToUse"] >> nrFrames;
    node["Calibrate_FixAspectRatio"] >> aspectRatio;
    node["Write_DetectedFeaturePoints"] >> writePoints;
    node["Write_extrinsicParameters"] >> writeExtrinsics;
    node["Write_outputFileName"] >> outputFileName;
    node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
    node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
    node["Calibrate_UseFisheyeModel"] >> useFisheye;
    node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
    node["Show_UndistortedImage"] >> showUndistorsed;
    node["Input"] >> input;
    node["Input_Delay"] >> delay;
    node["Fix_K1"] >> fixK1;
    node["Fix_K2"] >> fixK2;
    node["Fix_K3"] >> fixK3;
    node["Fix_K4"] >> fixK4;
    node["Fix_K5"] >> fixK5;

    validate();
}

void Settings::write(FileStorage& fs) const                        //Write serialization for this class
{
    fs << "{"
       << "BoardSize_Width"  << boardSize.width
       << "BoardSize_Height" << boardSize.height
       << "Square_Size"         << squareSize
       << "Calibrate_Pattern" << patternToUse
       << "Calibrate_NrOfFrameToUse" << nrFrames
       << "Calibrate_FixAspectRatio" << aspectRatio
       << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
       << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

       << "Write_DetectedFeaturePoints" << writePoints
       << "Write_extrinsicParameters"   << writeExtrinsics
       << "Write_outputFileName"  << outputFileName

       << "Show_UndistortedImage" << showUndistorsed

       << "Input_FlipAroundHorizontalAxis" << flipVertical
       << "Input_Delay" << delay
       << "Input" << input
       << "}";
}

void Settings::validate()
{
    goodInput = true;
    if (boardSize.width <= 0 || boardSize.height <= 0) {
	cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
	goodInput = false;
    }

    if (squareSize <= 10e-6) {
	cerr << "Invalid square size " << squareSize << endl;
	goodInput = false;
    }

    if (nrFrames <= 0) {
	cerr << "Invalid number of frames " << nrFrames << endl;
	goodInput = false;
    }

    if (input.empty())      // Check for valid input
	inputType = INVALID;
    else {
	if (input[0] >= '0' && input[0] <= '9') {
	    stringstream ss(input);
	    ss >> cameraID;
	    inputType = CAMERA;
	} else {
	    if (isListOfImages(input) && readStringList(input, imageList)) {
		inputType = IMAGE_LIST;
		nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
	    } else
		inputType = VIDEO_FILE;
	}

	if (inputType == CAMERA)
	    inputCapture.open(cameraID);
	if (inputType == VIDEO_FILE)
	    inputCapture.open(input);
	if (inputType != IMAGE_LIST && !inputCapture.isOpened())
	    inputType = INVALID;

	if(inputCapture.isOpened()) {
	    timeStep = inputCapture.get(CV_CAP_PROP_FPS);
	    if(timeStep > 0.0001)
		timeStep = 1./timeStep;
	    else
		timeStep = 0; //invalid
	}
    }

    if (inputType == INVALID) {
	cerr << " Input does not exist: " << input;
	goodInput = false;
    }

    flag = 0;
    if(calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
    if(calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
    if(aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
    if(fixK1)                  flag |= CALIB_FIX_K1;
    if(fixK2)                  flag |= CALIB_FIX_K2;
    if(fixK3)                  flag |= CALIB_FIX_K3;
    if(fixK4)                  flag |= CALIB_FIX_K4;
    if(fixK5)                  flag |= CALIB_FIX_K5;
    if(!fixK4 || !fixK5) flag|= CALIB_RATIONAL_MODEL; //enable K4, K5

    /*if (useFisheye) {
			// the fisheye model has its own enum, so overwrite the flags
			flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC;
			if(fixK1)                   flag |= fisheye::CALIB_FIX_K1;
			if(fixK2)                   flag |= fisheye::CALIB_FIX_K2;
			if(fixK3)                   flag |= fisheye::CALIB_FIX_K3;
			if(fixK4)                   flag |= fisheye::CALIB_FIX_K4;
			if (calibFixPrincipalPoint) flag |= fisheye::CALIB_FIX_PRINCIPAL_POINT;
		}*/

    calibrationPattern = NOT_EXISTING;
    if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
    if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
    if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
    if (calibrationPattern == NOT_EXISTING) {
	cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
	goodInput = false;
    }
    atImageList = 0;
}

Mat Settings::getNextImage()
{
    Mat result;
    if( inputCapture.isOpened() ) {
	Mat view0;
	inputCapture >> view0;
	view0.copyTo(result);
    }
    else if( atImageList < imageList.size() )
	result = imread(imageList[atImageList++], IMREAD_COLOR);

    return result;
}

bool Settings::readStringList( const string& filename, vector<string>& l )
{
    l.clear();
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
	return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
	return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
	l.push_back((string)*it);
    return true;
}

bool Settings::isListOfImages( const string& filename)
{
    string s(filename);
    // Look for file extension
    if( s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos )
	return false;
    else
	return true;
}
