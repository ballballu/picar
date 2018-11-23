/////////////////////////////2018.6.13后置摄像头寻找车位
#include "cv.h"
#include "highgui.h"
#include "opencv2/opencv.hpp"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include <iostream>  
#include <string>
#include <vector>

#include <unistd.h>
#include <csignal>
#include <cmath>
#include <unistd.h>
#include "driver.h"
using namespace cv;
using namespace std;

vector<Point> drawline(Mat, Mat);
int backcar(int, double, double, double, double, driver*);
float reback(float);
double ti(double, driver*);

int main()
{
	Mat backframe;
	Mat undistort;
	Mat backedges;
	Mat backsingle;			//经过各种处理后再透视变换的,黑白
	Mat backthree;			//视频直接透视变换的，彩色

	VideoCapture capture(0);
	capture >> backframe;
	vector<Point> parkloc;
	int realn;
	double kline;
	int chooseparknum = 1;		//选择几号停车位(1234)
	int dx = 0;				//车位与中轴在x上的距离
	int dy;				//车位与中轴在y上的距离
	int avery = 0;					//车位y坐标均值
	int ymax = 200;				//极限220 余量为200	
	int ymin = 120;				//极限100(65) 留余量为120
	int backcounter = 0;
	int ccc = 2;

	/////////////////////////////////////////读入xml文件校正后置摄像头
	CvCapture* capturecv = cvCreateCameraCapture(0);
	IplImage *image = cvQueryFrame(capturecv);
	CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");
	CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");

	IplImage* mapxi = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	IplImage* mapyi = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);

	cvInitUndistortMap(intrinsic, distortion, mapxi, mapyi);                              //用于计算畸变映射 在已经有了内参数(内参数矩阵 和 畸变参数)之后 输出原图像各点被映射的位置(x，y值)
																						  //mapx给出点被映射的x值，mapy给出点被映射的y值
	Mat mapx(mapxi, true);
	Mat mapy(mapyi, true);

	/////////////////////////////////////////

	int img_height = backframe.rows;
	int img_width = backframe.cols;
	int origintop = img_height / 2;
	int extend = -200;
	int extendbottom = 200; //底部纵向拉伸的量
	int shrinkbottom = 120;  //底部横向收缩的量
	int shrinktop = 150;

	////////////////////////////////////////透视变换部分2018.5.19
	vector<Point2f> corners(4);
	corners[0] = Point2f(0, origintop);
	corners[1] = Point2f(img_width - 1, origintop);
	corners[2] = Point2f(0, img_height - 1);
	corners[3] = Point2f(img_width - 1, img_height - 1);
	vector<Point2f> corners_trans(4);
	corners_trans[0] = Point2f(-shrinktop, extend);			 // top left  
	corners_trans[1] = Point2f(img_width + shrinktop, extend);				 // top right  
	corners_trans[2] = Point2f(shrinkbottom, img_height - 1 + extendbottom);   // bottom left  
	corners_trans[3] = Point2f(img_width - 1 - shrinkbottom, img_height - 1 + extendbottom); // bottom right

	Mat mapperspective = getPerspectiveTransform(corners, corners_trans);		//计算透视变换矩阵

																				//////////////////////////////////////////
	driver* d = new driver();
	d->launchSpeedMode();
	sleep(1);
	d->setServo(0);
	waitKey(500);
	d->setMotor(0.0);
	waitKey(1000);

	while (1)
	{
		capture >> backframe;
		if (backcounter > 10)
		{
			backcounter = 0;
			remap(backframe, undistort, mapx, mapy, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(255, 0, 0));

			cvtColor(undistort, backedges, CV_BGR2GRAY);						//转为灰度图
			GaussianBlur(backedges, backedges, Size(7, 7), 1.5, 1.5);			//高斯滤波
			threshold(backedges, backedges, 100, 255, THRESH_BINARY_INV);				//二值化
			Canny(backedges, backedges, 0, 30, 3);								//canny边缘检测

			warpPerspective(backedges, backsingle, mapperspective, Size(img_width, img_height));		//透视变换
			warpPerspective(undistort, backthree, mapperspective, Size(img_width, img_height));

			////////////////////////////车位识别定位

			parkloc = drawline(backsingle, backthree);
			realn = parkloc[0].x;
			kline = parkloc[0].y / 10000.0;
			if ((realn != 999) && (realn != 0))
			{
				for (int i = 1; i < realn + 1; i++)
				{
					circle(backthree, parkloc[i], 5, Scalar(0, 255, 0), -1, 8, 0);
					avery = parkloc[i].y + avery;
				}
				dy = avery / realn;
				cout << "YY  " << dy << endl;
				avery = 0;
				circle(backthree, parkloc[chooseparknum], 5, Scalar(255, 255, 0), -1, 8, 0);
				//////////////////////////保证小车在下一次动作前在驻车区间
				cout << "k= " << kline << endl;
				cout << "ccc= " << ccc << endl;
				if (ccc == 1)			//车停下后，自动调整y距离，在70-100之间
				{
					if (dy < 70)//离车位太远了，要靠近，车后退
					{
						//后退
						d->setServo(0);
						usleep(500000);
						d->setMotor(-0.08);
						usleep(200000);
						d->setMotor(0);
						usleep(500000);
						cout << "pull " << endl;
					}
					if (dy > 100)//离车位太近了，要远离，车前进
					{
						//前进
						d->setServo(0);
						usleep(500000);
						d->setMotor(0.07);
						usleep(1800000);
						d->setMotor(0);
						usleep(500000);
						cout << "push " << endl;
					}
					if ((dy < 100) && (dy > 70))
					{
						ccc = 2;	//	可以开始运动
					}
				}

				else if (ccc == 2)			//第一次运动 根据选择的车位向左或右一大步
				{
					if (chooseparknum < 3)
					{
						ccc = 3;
						d->setServo(-0.5);
						usleep(500000);
						d->setMotor(-0.2);
						usleep(4500000);
						d->setMotor(0);
						usleep(500000);

						d->setServo(-1.0);
						usleep(500000);
						d->setMotor(0.20);
						usleep(3200000);
						d->setMotor(0);
						usleep(500000);

						d->setServo(-reback(kline) * 3);
						usleep(500000);
						d->setMotor(0.2);
						usleep(2500000);
						d->setMotor(0);
						usleep(500000);


						d->setServo(0);
						usleep(500000);
					}

					if (chooseparknum > 2)
					{
						ccc = 3;
						d->setServo(0.5);
						usleep(500000);
						d->setMotor(-0.2);
						usleep(4500000);
						d->setMotor(0);
						usleep(500000);

						d->setServo(1.0);
						usleep(500000);
						d->setMotor(0.20);
						usleep(3100000);
						d->setMotor(0);
						usleep(500000);

						d->setServo(-reback(kline) * 3);
						usleep(500000);
						d->setMotor(0.2);
						usleep(2500000);
						d->setMotor(0);
						usleep(500000);

						d->setServo(0);
						usleep(500000);
					}
				}

				else if (ccc == 3)	//调整车辆朝向为正向面对车库	
				{
					if (abs(kline) < 0.1)
						ccc = 4;
					else
					{
						ti(kline, d);
					}
				}

				else if (ccc == 4)
				{
					if (dy < 140)//离车位太远了，要靠近，车后退 ymin=120
					{
						//后退
						d->setServo(0);
						usleep(500000);
						d->setMotor(-0.1);
						usleep(2000000);
						d->setMotor(0);
						usleep(500000);
					}
					if (dy > 200)//离车位太近了，要远离，车前进 ymax=220
					{
						//前进
						d->setServo(0);
						usleep(500000);
						d->setMotor(0.1);
						usleep(2000000);
						d->setMotor(0);
						usleep(500000);
					}
					if ((dy < 200) && (dy > 140))
					{
						ccc = 0;	//	可以开始s
					}
				}

				///////////////////////////////倒车算法 S 分段精度
				else if (ccc == 0)
				{
					if ((chooseparknum == 1))
					{
						dx = 320 - parkloc[1].x;
						cout << "XX= " << dx << endl;
						int xx = reback(dx);
						if (dx > 65)
						{
							//backcar(chooseparknum, -0.5, -1.0, -0.2, 0.18, d);
							d->setServo(-0.5*xx);
							usleep(500000);
							d->setMotor(-0.2);
							usleep(3000000);
							d->setMotor(0);
							usleep(500000);

							d->setServo(-1.0*xx);
							usleep(500000);
							d->setMotor(0.20);
							usleep(2700000);
							d->setMotor(0);
							usleep(500000);

							d->setServo(-reback(kline) * 5);
							usleep(500000);
							d->setMotor(0.2);
							usleep(2000000);
							d->setMotor(0);
							usleep(500000);

							d->setServo(0);
							usleep(500000);

							ccc = 3;
						}
						else if (dx < 66)
						{
							cout << "OK!!!!!!!= " << dx << endl;
							d->setServo(-(dx - 20) / 250);			//dx》0说明小车要右打轮后退 轮子左打轮是正，故要负
							usleep(500000);
							d->setMotor(-0.15);
							usleep(4000000);
							d->setMotor(0);
							usleep(500000);

							d->setServo((dx - 20) / 100);			//dx》0说明小车要右打轮后退 轮子左打轮是正，故要负
							usleep(500000);
							d->setMotor(-0.15);
							usleep(4000000);
							d->setMotor(0);
							usleep(500000);

							ccc = 999;
						}
					}

					else if ((chooseparknum == 2))
					{
						if (realn > 2)
						{
							dx = 320 - parkloc[2].x;
						}
						else
						{
							dx = 320 - parkloc[realn].x;
						}
						cout << "XX= " << dx << endl;
						int xx = reback(dx);
						if (dx > 65)
						{
							//backcar(chooseparknum, -0.5, -1.0, -0.2, 0.18, d);
							d->setServo(0.5*xx);
							usleep(500000);
							d->setMotor(-0.2);
							usleep(3000000);
							d->setMotor(0);
							usleep(500000);

							d->setServo(1.0*xx);
							usleep(500000);
							d->setMotor(0.20);
							usleep(2700000);
							d->setMotor(0);
							usleep(500000);

							d->setServo(-reback(kline) * 5);
							usleep(500000);
							d->setMotor(0.2);
							usleep(2000000);
							d->setMotor(0);
							usleep(500000);

							d->setServo(0);
							usleep(500000);

							ccc = 3;
						}
						else if (dx < 66)
						{
							cout << "OK!!!!!!!= " << dx << endl;
							d->setServo((dx - 25) / 200);			//dx》0说明小车要右打轮后退 轮子左打轮是正，故要负
							usleep(500000);
							d->setMotor(-0.15);
							usleep(4000000);
							d->setMotor(0);
							usleep(500000);

							d->setServo(-(dx - 25) / 200);			//dx》0说明小车要右打轮后退 轮子左打轮是正，故要负
							usleep(500000);
							d->setMotor(-0.15);
							usleep(4000000);
							d->setMotor(0);
							usleep(500000);

							ccc = 999;
						}
					}

					else if ((chooseparknum == 4))
					{
						if (realn>4)
						{
							dx = parkloc[4].x - 320;
						}
						else
						{
							dx = parkloc[realn].x - 320;
						}
						cout << "XX= " << dx << endl;
						int xx = reback(dx);
						if (dx > 65)
						{
							//backcar(chooseparknum, -0.5, -1.0, -0.2, 0.18, d);
							d->setServo(0.5*xx);
							usleep(500000);
							d->setMotor(-0.2);
							usleep(3000000);
							d->setMotor(0);
							usleep(500000);

							d->setServo(1.0*xx);
							usleep(500000);
							d->setMotor(0.20);
							usleep(2500000);
							d->setMotor(0);
							usleep(500000);

							d->setServo(-reback(kline) * 5);
							usleep(500000);
							d->setMotor(0.2);
							usleep(2000000);
							d->setMotor(0);
							usleep(500000);

							d->setServo(0);
							usleep(500000);

							ccc = 3;
						}
						else if (dx < 66)
						{
							cout << "OK!!!!!!!" << endl;
							d->setServo((dx - 30) / 200);			//dx》0说明小车要右打轮后退 轮子左打轮是正，故要负
							usleep(500000);
							d->setMotor(-0.15);
							usleep(4000000);
							d->setMotor(0);
							usleep(500000);

							d->setServo(-(dx - 30) / 200);			//dx》0说明小车要右打轮后退 轮子左打轮是正，故要负
							usleep(500000);
							d->setMotor(-0.15);
							usleep(4000000);
							d->setMotor(0);
							usleep(500000);
						}
					}
				}
			}
			/////////////////////////
			//imshow("back", backframe);
			//imshow("undistort", undistort);
			//imshow("backsingle", backsingle);
			cout << "backcounter++;" << backcounter++ << endl;
			imshow("backthree", backthree);

			if (waitKey(30) >= 0)
				break;

		}
		else
		{
			backcounter++;
		}
	}
	return 0;
}

vector<Point> drawline(Mat inputpic, Mat outputpic)
{
	//拟合直线部分
	Point point0, point1, point2;
	double kline;	//拟合直线的斜率	

					//分割数字部分
	int n = 0;									//中点数
	int ny = 0;									//经过y筛选后的中点数量
	int realn = 0;								//实际真正的中点数,不算最左面的，即如果是整个车位，那么5条真正的线，realn=4
	vector<double> xline, yline, preline;					//把提取到的中点的x,y坐标提取出来,preline是用来对y坐标排序好计算y均值的
	vector<double> realxline, realyline;		//挑选出的代表真正中点的x,y坐标
	double distance;			//聚类的距离标准，这里认为每个点左右30像素以内的点应该都是实际对应同一条真正的线的中点
	vector<Point> turnloc;		//返回给主函数的数字停车位的位置
	vector<Point> cannot;
	cannot.push_back(Point(999, 999));

	//霍夫变换
	double k;		//霍夫变换直线斜率
	vector<Vec4i> lines2;
	HoughLinesP(inputpic, lines2, 1, CV_PI / 180, 120, 100, 10);		//HoughLinesP给出的是两个点的xy坐标，HoughLines给出的是直线的r和theta

	vector<Point> prelocation;		//存放直线拟合的点，之后要做处理（分割数字部分）
	vector<Point> location;		//用于直线拟合的点的集合
	Vec4f  niheline;			//储放输出的拟合直线

								//计算提取出的直线斜率，去除掉横着的线（只保留正向一定范围内的直线），并依次在图中绘制出每条线段
	for (size_t i = 0; i < lines2.size(); i++)
	{
		Vec4i l = lines2[i];
		Point2i loc;					//存储符合条件的直线的中点
		if ((l[2] - l[0]) == 0)
		{
			loc.x = (l[2] + l[0]) / 2;
			loc.y = (l[3] + l[1]) / 2;
			line(outputpic, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(12, 128, 255), 5, CV_AA);
			prelocation.push_back(loc);			//将中点坐标压进拟合直线的点集
			yline.push_back(loc.y);
			n++;
		}
		else
		{
			k = (l[3] - l[1]) / (l[2] - l[0]);
			if (k*k > 0.5)											//k的平方大于0.5是多次实验得到的经验值，排除了非常明显的横线，同时保留了一定弯度的曲线
			{
				loc.x = (l[2] + l[0]) / 2;
				loc.y = (l[3] + l[1]) / 2;
				line(outputpic, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(12, 128, 255), 5, CV_AA);
				prelocation.push_back(loc);
				yline.push_back(loc.y);
				n++;
			}
		}
	}
	if (n != 0)
	{
		preline = yline;
		sort(preline.begin(), preline.end());		//对得到的中点按y坐标排序
		for (int i = 0; i < n; i++)
		{
			distance = abs(yline[i] - preline[n / 2]);		// preline[n / 2]认为是y的均值
			if (distance < 50)	//取y值中位数认为是y均值，在均值上下50以内的是合理的点，大于50的认为是干扰,就不要了
			{
				location.push_back(prelocation[i]);
				xline.push_back(prelocation[i].x);
				ny++;
			}
		}

		//拟合直线
		if (ny > 0)
		{
			fitLine(location, niheline, CV_DIST_L2, 0, 0.01, 0.01);		//niheline 输出参数的前半部分给出的是直线的方向，而后半部分给出的是直线上的一点 //距离类型  // 距离参数  // 径向的精度参数  // 角度精度参数  
																		//求拟合直线的斜率
			point0.x = niheline[2];
			point0.y = niheline[3];
			kline = niheline[1] / niheline[0];

			///////////////分割出数字区域部分,前提是中点数>10
			//假设具有鲁棒性，那么每个点左右30像素以内的点应该都是实际对应同一条真正的线的中点

			sort(xline.begin(), xline.end());		//对得到的中点按x坐标排序
			realxline.push_back(xline[0]);			//这里直接将最小的push进去了，就是假设x坐标最小的一定对应一条真正的线，有一定风险，可能会崩
			for (int i = 0; i < ny - 1; i++)
			{
				distance = xline[i + 1] - xline[i];
				if (distance > 30)
				{
					realxline.push_back(xline[i + 1]);
					realn++;
				}
			}

			//找停车位数字的中心
			turnloc.push_back(Point(realn, kline * 10000));//因为Point要整形数（我要的点的坐标）所以给kline*10000，主函数再变回来,乘10000保证精度为0.0001
			for (int i = 0; i < realn; i++)
			{
				double parkx = (realxline[i] + realxline[i + 1]) / 2;
				double parky = kline * (parkx - point0.x) + point0.y;
				turnloc.push_back(Point(parkx, parky));
			}
			imshow("create", outputpic);
			return turnloc;
		}
		else
			return cannot;
	}
	else
		return cannot;
}

double ti(double kline, driver *d)
{
	cout << "ti  " << endl;
	if (kline < -0.1)
	{
		d->setServo(0.1);
		usleep(500000);
		d->setMotor(0.1);
		usleep(2000000);
		d->setMotor(0);
		usleep(500000);

		d->setServo(-0.1);
		usleep(500000);
		d->setMotor(-0.1);
		usleep(2000000);
		d->setMotor(0);
		usleep(500000);

		d->setServo(0);
		usleep(500000);
	}
	if (kline > 0.1)
	{
		d->setServo(-0.15);
		usleep(500000);
		d->setMotor(0.1);
		usleep(2000000);
		d->setMotor(0);
		usleep(500000);

		d->setServo(0.15);
		usleep(500000);
		d->setMotor(-0.1);
		usleep(2000000);
		d->setMotor(0);
		usleep(500000);

		d->setServo(0);
		usleep(500000);
	}
	return 0;
}


int backcar(int chooseparknum, double srevo1, double srevo2, double carspeed1, double carspeed2, driver *d)
{
	int turn = 1; 		//小车去12车位（为1）or34（为-1）
	if ((chooseparknum == 3) || (chooseparknum == 4))
	{
		turn = -1;
	}

	///////////////// U型轨迹
	d->setServo(srevo1*turn);
	waitKey(500);
	d->setMotor(carspeed1);
	waitKey(3000);
	d->setMotor(0);
	waitKey(500);

	d->setServo(srevo2*turn);
	waitKey(500);
	d->setMotor(carspeed2);
	waitKey(2500);
	d->setMotor(0);
	waitKey(500);

	d->setServo(0);
	waitKey(500);
	d->setMotor(0.2);
	waitKey(2000);
	d->setMotor(0);
	waitKey(500);
	d->setServo(0);
	waitKey(500);

	return 0;
}

float reback(float x)
{
	if (abs(x) < 0.15)
		return x;
	else if (x < 0)
		return -1;
	else
		return 1;
}