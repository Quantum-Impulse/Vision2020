//This is the opencv pre-program before the pi configuration 
//
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <String>
#include <cmath>

using namespace std;
using namespace cv;

//math constants
const double M_PI = 3.141592653; //PI
const double CONVERT_TO_DEGREES = (M_PI / 180.0); // convert radians to degrees 

//Angles in radians
//image size ratioed to 16:9
int imageWidth = 1280;
int imageHeight = 720;

//Center(x,y) of the image
double centerX = (imageWidth / 2) - .5;
double centerY = (imageHeight / 2) - .5;

//Lifecam 3000 from datasheet
//Datasheet: https://dl2jx7zfbtwvr.cloudfront.net/specsheets/WEBC1010.pdf
//convert degrees '68.5' to radians 
double diagonalView = (68.5 * M_PI) / 180.0;

//16:9 aspect ratio
int horizontalAspect = 16;
int verticalAspect = 9;

//Reasons for using diagonal aspect is to calculate horizontal field of view.
double diagonalAspect = hypot(horizontalAspect, verticalAspect);

//Calculations: http://vrguy.blogspot.com/2013/04/converting-diagonal-field-of-view-and.html
double horizontalView = atan(tan(diagonalView / 2) * (horizontalAspect / diagonalAspect)) * 2;
double verticalView = atan(tan(diagonalView / 2) * (verticalAspect / diagonalAspect)) * 2;

//Focal Length calculations : https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_165
double H_FOCAL_LENGTH = imageWidth / (2 * tan((horizontalView / 2)));
double V_FOCAL_LENGTH = imageHeight / (2 * tan((verticalView / 2)));

// Tracked objects centroid coordinates
int theObject[2] = { 0,0 };
double yaw = 0, pitch = 0;
//bounding rectangle of the object, we will use the center of this as its position
Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
RotatedRect rotatedRectangle;



// Include center point of your rectangle, size of your rectangle and the degrees of rotation  
//cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true) * 0.02, true);
void DrawRotatedRectangle(cv::Mat& image, vector< vector<Point>> cnt){
	cv::Scalar color = cv::Scalar(255.0, 0.0, 0.0); // red because it is RGB
	cv::RotatedRect rotatedRectangle;
	rotatedRectangle = cv::fitEllipse(image);

	// We take the edges that OpenCV calculated for us
	//cv::Point2f vertices2f[4];
	//rotatedRectangle.points(vertices2f);

	// Convert them so we can use them in a fillConvexPoly
	//cv::Point vertices[4];
	//for (int i = 0; i < 4; ++i) {
	//	vertices[i] = vertices2f[i];
	//}

	// Now we can fill the rotated rectangle with our specified color
	//cv::fillConvexPoly(image,
		//vertices,
	//	4,
	//	color);
}

//Uses trig and focal length of camera to find yaw.
//Link to further explanation : https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_298
double calculateYaw(double pixelX, double CenterX, double hFocalLength) {
	double yaw = CONVERT_TO_DEGREES * (atan((pixelX - CenterX) / hFocalLength));
	return yaw; 
}

//Link to further explanation: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_298
double calculatePitch(double pixelY, double  CenterY, double vFocalLength) {
	double pitch = CONVERT_TO_DEGREES * (atan((pixelY - CenterY) / vFocalLength));
	//just stopped working have to do this:
	pitch *= -1.0;
	return pitch;
}

double calculateDistance(double heightOfCamera, double heightOfTarget ,double pitch) {
	double heightOfTargetFromCamera = heightOfTarget - heightOfCamera;
	/*
	// Uses trig and pitch to find distance to target
  
    d = distance
    h = height between camera and target
    a = angle = pitch
    tan a = h/d (opposite over adjacent)
    d = h / tan a
                         .
                        /|
                       / |
                      /  |h
                     /a  |
              camera -----
                       d
	*/
	double distance = fabs(heightOfTargetFromCamera / tan((pitch* M_PI) / 180.0));
	return distance;
}

void searchForMovement(Mat thresholdImage, Mat &cameraFeed) {
	/* Notice how we use the '&' operator for the camerafeed. This is because we wish
	 to take the values passed into the function and manipulate them, rather than just
	 working with a copy. eg. we draw to the cameraFeed in this function which is then
	 displayed in the main() function. */
	bool objectDetected = false;
	Mat temp;
	thresholdImage.copyTo(temp);
		// these two vectors needed for the output of findContours

	vector< vector<Point>> contours;
	vector<Vec4i> hierarchy;
		//findContours of filtered image using openCV findCOntours function
		//findContours(temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE); //retrieves all contours
	findContours(temp, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);// retrieves external contours
	
		//if contours vector is not empty, we found some objects
	if (contours.size() > 0) {
		objectDetected = true;
	}
	else { objectDetected = false; }

	if (objectDetected) {
		//largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is what we looking for.
		vector< vector<Point>> largestContourVec;
		largestContourVec.push_back(contours.at( contours.size() - 1));
		
		//making a bounding rectagle around the largest contour then find its centriod
		//this will be the object's final estimated position.
		objectBoundingRectangle = boundingRect(largestContourVec.at(0));
		int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
		int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;

		//update the objects position by changing the 'theObject' array values
		theObject[0] = xpos, theObject[1] = ypos;

	}
	//make some temp x and y varibles so we don't have to type out so much
	int x = theObject[0];
	int y = theObject[1];

	//Calculates yaw of contour (horizontal position in degrees)
	//
	//Calculates pitch of contour (Vertical position in degrees)
	//

	//draw some crosshairs on the object
	//rectangle(cameraFeed, objectBoundingRectangle, Scalar(255, 0, 0), 3, 8, 0);
	circle(cameraFeed, Point(x, y), 20, Scalar(0, 255, 0), 2);
	line(cameraFeed, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	line(cameraFeed, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	line(cameraFeed, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	line(cameraFeed, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	//draws the rotated rectangle contours and might have to blur to make drawing more 'rectangleish'
	drawContours(cameraFeed, contours, 0, Scalar(255, 0, 0), 1, LINE_AA);

	// Displays yaw and pitch of tracked object
	//putText(cameraFeed, "Tracking object at (" + x, Point(x, y), 1, 1, Scalar(0, 0, 255), 2, LINE_AA);
	//putText(cameraFeed, "Yaw = " + yaw, Point(x - 30, y), 1, 1, Scalar(0, 0, 255), 2);
	//putText(cameraFeed, "Pitch = " + pitch, Point(x - 30, y + 30), 1, 1, Scalar(0, 0, 255), 2);
	
}

int main(int argc, char** argv)
{
	VideoCapture cap(0); // capture the video from web cam

	if (!cap.isOpened()) {
		cout << "Cannnot open the webcam" << endl;
		return -1;
	}

	namedWindow("Control HSV Range", WINDOW_NORMAL); //create a window called "Control"

	int iLowH = 44;
	int iHighH = 60;

	int iLowS = 0;
	int iHighS = 30;

	int iLowV = 130;
	int iHighV = 255;

	//create trackerbars in "Control" windows
	createTrackbar("LowH", "Control HSV Range", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control HSV Range", &iHighH, 179);

	createTrackbar("Lows", "Control HSV Range", &iLowS, 255); //Saturation (0 - 179)
	createTrackbar("Highs", "Control HSV Range", &iHighS, 255);

	createTrackbar("LowV", "Control HSV Range", &iLowV, 255); //Value (0 - 179)
	createTrackbar("HighV", "Control HSV Range", &iHighV, 255);

	while (true) {
		Mat imgOriginal;

		bool bSuccess = cap.read(imgOriginal); //read a new frame from video

		if (!bSuccess) //if not success, break loop
		{ 
			cout << "Cannot read a frame from the video stream" << endl;
			break;
		}

		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

		Mat imgThresholded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
		
		//morphological opening (remove small holes objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		searchForMovement(imgThresholded, imgOriginal);
		cout << "Yaw = " << yaw << endl;
		cout << "Pitch = " << pitch << endl;
		yaw = calculateYaw(theObject[0], centerX, H_FOCAL_LENGTH);
		pitch = calculatePitch(theObject[1], centerY, V_FOCAL_LENGTH);
		

		imshow("Threshold Image", imgThresholded); //show the threshold image
		imshow("Original", imgOriginal); // show the original image

		if (waitKey(30) == 27) // wait for the 'esc' key press for 30ms. If 'esc' key is pressed, break loops 
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}
	return 0;
}