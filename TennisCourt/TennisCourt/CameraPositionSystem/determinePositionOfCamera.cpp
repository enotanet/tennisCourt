#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;

Mat makeRotationMatrix(double yz, double xz, double xy)
{
  Mat rotXY(3, 3, CV_64FC1),
      rotYZ(3, 3, CV_64FC1),
      rotXZ(3, 3, CV_64FC1);

  //make XY matrix
  rotXY.at<double>(0, 0) = cos(xy);
  rotXY.at<double>(0, 1) = -sin(xy);
  rotXY.at<double>(0, 2) = 0;
  rotXY.at<double>(1, 0) = sin(xy);
  rotXY.at<double>(1, 1) = cos(xy);
  rotXY.at<double>(1, 2) = 0;
  rotXY.at<double>(2, 0) = 0;
  rotXY.at<double>(2, 1) = 0;
  rotXY.at<double>(2, 2) = 1;

  //make YZ matrix
  rotYZ.at<double>(0, 0) = 1;
  rotYZ.at<double>(0, 1) = 0;
  rotYZ.at<double>(0, 2) = 0;
  rotYZ.at<double>(1, 0) = 0;
  rotYZ.at<double>(1, 1) = cos(yz);
  rotYZ.at<double>(1, 2) = -sin(yz);
  rotYZ.at<double>(2, 0) = 0;
  rotYZ.at<double>(2, 1) = sin(yz);
  rotYZ.at<double>(2, 2) = cos(yz);

  //make XZ matrix
  rotXZ.at<double>(0, 0) = cos(xz);
  rotXZ.at<double>(0, 1) = 0;
  rotXZ.at<double>(0, 2) = -sin(xz);
  rotXZ.at<double>(1, 0) = 0;
  rotXZ.at<double>(1, 1) = 1;
  rotXZ.at<double>(1, 2) = 0;
  rotXZ.at<double>(2, 0) = sin(xz);
  rotXZ.at<double>(2, 1) = 0;
  rotXZ.at<double>(2, 2) = cos(xz);

  return rotXY * rotXZ * rotYZ;
}

int main()
{
  char filename[] = "out_camera_data.xml";
  FileStorage fs(filename, FileStorage::READ);

  if(!fs.isOpened())
  {
    cout << "missing output camera intrinsic paramaters xml\n" << endl;
    return -1;
  }

  FileNode fn = fs["Camera_Matrix"];

  Mat camera_matrix;
  fn >> camera_matrix;

  cout << "Camera matrix:" << endl;
  cout << camera_matrix << endl;

  fs.release();
	return 0;
}
