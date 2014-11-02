#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <vector>
#include <stdio.h>
#include <cmath>
#include <cstdlib>
#include "utils.h"
#include <fstream>

using namespace std;
using namespace cv;

vector<Vec3f> getCircles(Mat img)
{
	Mat img_grey(img.clone());
	// Assure we are working with monochrome images
	cvtColor(img, img_grey, CV_BGR2GRAY);

    GaussianBlur(img_grey, img_grey, Size(9, 9), 4, 4);
	//Sobel(img_grey, img_grey, -1, 1, 0, 3, 1, 0, BORDER_DEFAULT);
	Canny(img_grey, img_grey, 47, 47, 3);
	//imshow("Hough Circle Transform 111", img_grey);
	//waitKey(0);

	vector<Vec3f> circles;

    // Apply the Hough Transform to find the circles
    HoughCircles(img_grey, circles, CV_HOUGH_GRADIENT, 1, img_grey.rows/8, 50, 25, 0, 0);

	return circles;
}

void displayCircles(Mat img, vector<Vec3f> circles)
{
	// Draw the circles detected
	Mat out(img.clone());
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle(out, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		// circle outline
		circle(out, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	}

	// Show your results
	namedWindow("Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);
	imshow("Hough Circle Transform 111", out);
	waitKey(0);
}

bool getCirclesVerify(Mat img)
{
	vector<Vec3f> circles = getCircles(img);
	printf("%d\n", circles.size());
	// Draw the circles detected
	Mat out(img.clone());
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle(out, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		// circle outline
		circle(out, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	}

	// Show your results
	namedWindow("Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);
	imshow("Hough Circle Transform Demo", out);
	char k = waitKey(0);
	if (k == 'c')
	{
		// Save result
		imwrite("../TestData/testImg.png", img);

		ofstream fout("C:\\TennisCourt\\TennisCourt\\TestData\\testCircles.txt");
		cout << circles.size() << endl;
		fout << circles.size() << endl;
		//fprintf(cirfile, "%u\n", circles.size());
		for (size_t i = 0; i < circles.size(); i++)
		{
			fout << circles[i][0] << " " << circles[i][1] << " " << circles[i][2] << endl;
			cout << circles[i][0] << " " << circles[i][1] << " " << circles[i][2] << endl;
		}
		fout.close();
		return true;
	}
	return false;
}

bool getCirclesTest(Mat *img)
{
	*img = imread("../TestData/testImg.png");
	vector<Vec3f> circles = getCircles(*img);
	printf("%d\n", (int) circles.size());

	vector<Vec3f> circles_exp;
	ifstream fin("C:\\TennisCourt\\TennisCourt\\TestData\\testCircles.txt");
	size_t cir_no;
	fin >> cir_no;
	cout << cir_no << endl;
	circles_exp.resize(cir_no);
	for (size_t i = 0; i < circles_exp.size(); i++)
	{
		cout << i << " " << circles_exp.size() << endl;
		fin >> circles_exp[i][0] >> circles_exp[i][1] >> circles_exp[i][2];
		cout << circles_exp[i][0] << " " << circles_exp[i][1] << " " << circles_exp[i][2] << endl;
	}

	cout << endl;
	cout << circles.size() << endl;
	for (size_t i = 0; i < circles.size(); i++)
	{
		cout << circles[i][0] << " " << circles[i][1] << " " << circles[i][2] << endl;
	}
	cout << endl;

	//displayCircles(img, circles);
	//displayCircles(img, circles_exp);

	if (circles.size() != circles_exp.size())
	{
		return false;
	}
	
	const double eps = 2;
	for (size_t i = 0; i < circles_exp.size(); ++i)
	{
		bool found = false;
		for (size_t j = 0; j < circles.size(); ++j)
		{
			double err = 0.0;
			for (size_t k = 0; k < 3; ++k)
			{
				err += std::min(abs(circles_exp[i][k] - circles[j][k]), abs((circles_exp[i][k] - circles[j][k]) / circles_exp[i][k]));
			}
			//cout << i << " " << j << " " << err << endl;
			if (err < eps)
			{
				found = true;
				break;
			}
		}
		if (!found)
		{
			return false;
		}
	}

	return true;
}