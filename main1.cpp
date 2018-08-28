//Written by  Kyle Hounslow 2013

// modified by: Ahmad Kaifi, Hassan Althobaiti

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.
#define _CRT_SECURE_NO_WARNINGS

#include <sstream>
#include <string>
#include <iostream>
#include <vector>

#include "Object.h"


//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = 240 * 240;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

//The following for canny edge detec
Mat dst, detected_edges;
Mat src, src_gray;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
const char* window_name = "Edge Map";

Point cameraCenter;

int roi_width, roi_height;

void on_trackbar(int, void*)
{//This function gets called whenever a
 // trackbar position is changed

}

string intToString(int number) {

	std::stringstream ss;
	ss << number;
	return ss.str();
}

void createTrackbars() {
	//create window for trackbars
	namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "H_MIN", H_MIN);
	sprintf(TrackbarName, "H_MAX", H_MAX);
	sprintf(TrackbarName, "S_MIN", S_MIN);
	sprintf(TrackbarName, "S_MAX", S_MAX);
	sprintf(TrackbarName, "V_MIN", V_MIN);
	sprintf(TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH),
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->
	createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
	createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
	createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
	createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
	createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
	createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);
}

void drawObject(vector<Object> theObjects, Mat &frame, Mat &temp, vector< vector<Point> > contours, vector<Vec4i> hierarchy) {

	for (int i = 0; i<theObjects.size(); i++) {
		cv::drawContours(frame, contours, i, theObjects.at(i).getColor(), 3, 8, hierarchy);
		cv::circle(frame, cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos()), 5, theObjects.at(i).getColor());
		cv::putText(frame, intToString(theObjects.at(i).getXPos()) + " , " + intToString(theObjects.at(i).getYPos()), cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos() + 20), 1, 1, theObjects.at(i).getColor());
		cv::putText(frame, theObjects.at(i).getType(), cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos() - 20), 1, 2, theObjects.at(i).getColor());
	}
}

void drawObject(vector<Object> theObjects, Mat &frame) {

	for (int i = 0; i<theObjects.size(); i++) {

		cv::circle(frame, cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos()), 10, cv::Scalar(0, 0, 255));
		cv::putText(frame, intToString(theObjects.at(i).getXPos()) + " , " + intToString(theObjects.at(i).getYPos()), cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos() + 20), 1, 1, Scalar(0, 255, 0));
		cv::putText(frame, theObjects.at(i).getType(), cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos() - 30), 1, 2, theObjects.at(i).getColor());
	}
}

void morphOps(Mat &thresh) {

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}
void trackFilteredObject(Mat threshold, Mat HSV, Mat &cameraFeed)
{
	vector <Object> objects;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	/*contour_list = [];

	for (int i = 0; i < contours.size(); i++) {
	approx = approxPolyDP(contours[i], 0.01*arcLength(contours[i], true), true);
	area = contourArea(contours[i]);
	if ((len(approx) > 8) & (area > 30)) {
	contour_list.append(contours[i])
	}
	}   */


	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA)
				{
					Object object;

					object.setXPos(moment.m10 / area);
					object.setYPos(moment.m01 / area);

					objects.push_back(object);

					objectFound = true;

				}
				else objectFound = false;
			}
			//let user know you found an object
			if (objectFound == true)
			{
				//draw object location on screen
				drawObject(objects, cameraFeed);
			}
		}
		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	}
}

Point trackFilteredObject(Object theObject, Mat threshold, Mat HSV, Mat &cameraFeed) {

	Point result;
	vector <Object> objects;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<vector<Point> > balls;
	vector<Vec4i> hierarchy;
	vector<vector<Point>> test{ 0 };
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = true;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS) {
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
				//
				std::vector<Vec3f> circles;
				// image, circles, method, dp, minDist, param1, param2, minRadius, maxRadius
				/*
				Mat tmp;
				cvtColor(HSV, tmp, COLOR_HSV2BGR);
				cvtColor(tmp, tmp, COLOR_BGR2GRAY);
				HoughCircles(tmp, circles, HOUGH_GRADIENT, 1, src_gray.rows / 8, 80, 80, 0, 0);
				*/
				// 

				//circle
				vector<vector<Point> > contours_poly(contours.size());
				vector<Point2f>centers(contours.size());
				vector<float>radius(contours.size());
				vector<vector<Point> > trash(contours.size());

				for (size_t i = 0; i < contours.size(); i++)
				{
					approxPolyDP(contours[i], contours_poly[i], 3, true);
					minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
					if (float(contourArea(contours[i])) >= float(0.8) * radius[i] * radius[i] * float(3.14)) {
						// centers[i]가 중심점을 어떻게 잡고 있는지 체크해야함.
						std::cout << "x좌표: " << centers[i].x << "          y좌표: " << centers[i].y << std::endl;
						
						balls.push_back(contours[i]);
						/*
						std::cout << "contour 넓이" << std::endl;
						std::cout << contourArea(contours[i]) << std::endl;
						std::cout << "circle 넓이" << std::endl;
						std::cout << radius[i] * radius[i] * 3.14 << std::endl;
						std::cout << "반지름" << std::endl;
						std::cout << radius[i] << std::endl;
						*/
					}
					else {
						//std::cout << "원이 아닙니다" << std::endl;
						balls.push_back(trash[i]);
					}
				}

				/*Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);

				for (size_t i = 0; i< contours.size(); i++)
				{
				Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
				drawContours(drawing, contours_poly, (int)i, color);
				rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
				circle(drawing, centers[i], (int)radius[i], color, 2);
				}

				imshow("Contours", drawing);*/

				//

				//for (size_t i = 0; i < contours_poly.size(); i++) // contours.size = num of objects = i
				//{

				// cv::Rect bBox;
				// bBox = cv::boundingRect(contours[i]);

				// float ratio = (float)bBox.width / (float)bBox.height;
				/*
				if (1.0f < ratio < 1.5f)
				ratio = 1.0f / ratio;
				*/
				// Searching for a bBox almost square and big
				//ratio > 0.3 && bBox.area() >= 100
				/*
				if (ratio > 0.9 && ratio < 1.1 && bBox.area() >= 100)
				{
				balls.push_back(contours[i]);
				}*/
				// balls.push_back(contours_poly[i]);
				//}

				Moments moment = moments((cv::Mat)balls[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA) {

					Object object;

					object.setXPos(moment.m10 / area);
					object.setYPos(moment.m01 / area);
					object.setType(theObject.getType());
					object.setColor(theObject.getColor());
					//
					// std::cout << object.getType() << std::endl;
					// std::cout << object.getXPos() << std::endl;
					// std::cout << object.getYPos() << std::endl;
					// std::cout << std::endl;
					//
					Point center(cvRound(object.getXPos()), cvRound(object.getYPos()));
					result = center;
					// std::cout << "This is test" << std::endl;
					// std::cout << center << std::endl;

					objects.push_back(object);

					objectFound = true;

				}
				else objectFound = false;
			}
			//let user know you found an object


			if (objectFound == true) {

				drawObject(objects, cameraFeed, temp, contours, hierarchy);
			}

		}
		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	}

	return result;
}

String roiDetector(Point center) {
	if ((center.x - (cameraCenter.x - roi_width / 2)) < 0) {
		return "왼쪽으로 가세요.";
	}
	else if ((center.x - (cameraCenter.x + roi_width / 2)) > 0) {
		return "오른쪽으로 가세요.";
	}
	else {
		return "정상 범주안에 있습니다.";
	}
	
}
int main(int argc, char* argv[])
{
	cameraCenter.x = 320;
	cameraCenter.y = 240;
	
	roi_width = 120;
	roi_height = 120;

	//if we would like to calibrate our filter values, set to true.
	bool calibrationMode = false;

	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	Mat threshold;
	Mat HSV;

	if (calibrationMode) {
		//create slider bars for HSV filtering
		createTrackbars();
	}
	//video capture object to acquire webcam feed
	VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(0);
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	waitKey(1000);
	while (1) {
		//store image to matrix
		capture.read(cameraFeed);

		src = cameraFeed;

		if (!src.data)
		{
			return -1;
		}

		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);

		if (calibrationMode == true) {

			//if in calibration mode, we track objects based on the HSV slider values.
			cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
			inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
			morphOps(threshold);
			imshow(windowName2, threshold);

			//the folowing for canny edge detec
			/// Create a matrix of the same type and size as src (for dst)
			dst.create(src.size(), src.type(
				//need to find the appropriate color range values
				// calibrationMode must be false
			));
			/// Convert the image to grayscale
			cvtColor(src, src_gray, CV_BGR2GRAY);
			/// Create a window
			namedWindow(window_name, CV_WINDOW_AUTOSIZE);
			/// Create a Trackbar for user to enter threshold
			createTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold);
			/// Show the image
			trackFilteredObject(threshold, HSV, cameraFeed);
		}
		else {
			//create some temp fruit objects so that
			//we can use their member functions/information
			Object blue("blue"), yellow("yellow"), red("red"), green("green");

			Point centerR, centerB, centerG, centerY;
			//first find blue objects
			cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
			inRange(HSV, blue.getHSVmin(), blue.getHSVmax(), threshold);
			morphOps(threshold);

			centerB = trackFilteredObject(blue, threshold, HSV, cameraFeed);
			// std::cout << centerB << std::endl;
			// std::cout << roiDetector(centerB) << std::endl;

			// std::cout << test1 << std::endl;

			//then red
			cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
			inRange(HSV, red.getHSVmin(), red.getHSVmax(), threshold);
			morphOps(threshold);
			centerR = trackFilteredObject(red, threshold, HSV, cameraFeed);

			// std::cout << centerR << std::endl;
			// std::cout << roiDetector(centerR) << std::endl;
			//then greens
			cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
			inRange(HSV, green.getHSVmin(), green.getHSVmax(), threshold);
			morphOps(threshold);
			centerG = trackFilteredObject(green, threshold, HSV, cameraFeed);

			std::cout << roiDetector((centerB + centerG)/2) << std::endl;

			//std::cout << "Blue" << "          " << "Green" << "          " << "Red" << std::endl;
			//std::cout << centerB << "     " << centerG << "     " << centerR << std::endl;
			// std::cout << "R-G    " << centerR - centerG << std::endl;
		}
		//show frames
		//imshow(windowName2,threshold);

		imshow(windowName, cameraFeed);
		//imshow(windowName1,HSV);

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(100);
	}
	return 0;
}