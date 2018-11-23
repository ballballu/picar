/////////////////////////////2018.5.18 
#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include <string>
#include "driver.h"
#include "linev15.h"
#include <unistd.h>
#include <csignal>
#include "math.h"
#include <vector>
#include "cv.h"
#include "highgui.h"
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

driver* d = new driver();

int main()
{
    // hardware init
	d->setMotion(0.0f, 0.0f);
	d->allStop();
	d->launchSpeedMode();
	usleep(500000);

	Mat frame, threshold_img, gray_img, color_img;
	Mat refined_threshold_img, refined_color_img;

	VideoCapture capture(1);
	capture >> frame;

    // array: [number of lines detected, number of setpoints determined, setpoint1 coordinates, setpoint2 coordinates]
    int *line_setpoint = new int[6];
    // array: [target steer value, target speed value]
    double *steer_speed = new double[2];
    // frame counter in case of command blockings
    int frame_counter = 0;

    // whether the parking sign is detected
    bool isParkingSign = false;
    // whether the car is successfully parked
    bool isParked = false;

	int img_height = frame.rows;
	int img_width = frame.cols;
	int origintop= img_height/2;
	int extend = -200;
	int extendbottom = 200;
	int shrinkbottom = 100; 
	int shrinktop = 150;

	
	vector<Point2f> corners(4);
	corners[0] = Point2f(0, origintop);
	corners[1] = Point2f(img_width - 1, origintop);
	corners[2] = Point2f(0, img_height - 1);
	corners[3] = Point2f(img_width - 1, img_height - 1);
	vector<Point2f> corners_trans(4);
	corners_trans[0] = Point2f(-shrinktop, extend); // top left  
	corners_trans[1] = Point2f(img_width+ shrinktop, extend); // top right  
	corners_trans[2] = Point2f(shrinkbottom, img_height - 1 + extendbottom); // bottom left  
	corners_trans[3] = Point2f(img_width - 1 - shrinkbottom, img_height - 1 + extendbottom); // bottom right

    // 
	Mat mapperspective = getPerspectiveTransform(corners, corners_trans);

    // cruise
	while (1)
	{
		capture >> frame;

		if(frame_counter == 5)
		{
			isParkingSign = check_parking_sign(frame);
            if(isParkingSign)
            {
                d->setMotion(0.0f, 0.0f);
				usleep(500000);
                break;
            }
            else
            {
				frame_counter = 0;
				
				warpPerspective(frame, color_img, mapperspective, Size(img_width, img_height));
				cvtColor(color_img, gray_img, CV_BGR2GRAY);						//
				GaussianBlur(gray_img, gray_img, Size(7, 7), 1.5, 1.5);         
				threshold(gray_img,threshold_img,110,255,THRESH_BINARY_INV);	//
				// imshow("1",edge);
				GaussianBlur(threshold_img, threshold_img, Size(7, 7), 1.5, 1.5);			//
				Canny(threshold_img, threshold_img, 0, 30, 3);								//
			
				//
				refined_threshold_img=threshold_img(Rect(img_width/5,img_height/2+10, img_width/2+10, img_height/3)); 
				//imshow("edges",refined_threshold_img);
				//waitKey(0);
				refined_color_img=color_img(Rect(img_width/5,img_height/2+10, img_width/2+10, img_height/3));
                
				
                line_setpoint = determine_setpoint(refined_threshold_img, refined_color_img);
                car_control_cruise(line_setpoint, img_width/2+10, img_height/3, d, refined_color_img);
				//imshow("detected lines",refined_color_img);
				//waitKey(300);
            }
		}
		else
			frame_counter++;
	}

    // parking
/*
	while (1)
  	{
	capture >> frame;

		if(frame_counter == 5)
		{
           if(isParked)
            {
                d->setMotion(0.0f, 0.0f);
                break;
            }
            else
            {
                frame_counter = 0;
                car_control_parking(line_setpoint);
            }
		}
		else
			frame_counter++;
    }
*/

	destroyAllWindows();
	return 0;
}
