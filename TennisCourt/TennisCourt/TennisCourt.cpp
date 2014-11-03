#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ballFinder.h"
#include <iostream>
#include <cstdio>
#include "utils.h"

using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		printf("USAGE: TennisCourt type\n");
		return 0;
	}
	string type = argv[1];
	if (type == "generate")
	{
		Mat img = imread("TennisCourt/TestData/testBall.JPG");

		if (getCirclesVerify(img))
			printf("Test results updated!\n");
		else
			printf("Test results discarded!\n");
	}
	else if (type == "test")
	{
		Mat img;
		if (!getCirclesTest(&img))
		{
			printf("Test result mismatch! Verify new results.\n");
			getCirclesVerify(img);
		}
		else
		{
			printf("TEST OK!\n");
		}
	}
	return 0;
}
