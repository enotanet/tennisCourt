//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/highgui/highgui.hpp>
//
//#include <iostream>
//
//using namespace std;
//using namespace cv;
//
//char name[] = "chessboard1.jpg";
//
//int main()
//{
//  Mat view = imread(name, CV_LOAD_IMAGE_GRAYSCALE);
//  Size s(7, 7);
//  vector<Point2f> pointBuf;
//
//  bool found = findChessboardCorners(view, s, pointBuf,
//                                     CV_CALIB_CB_ADAPTIVE_THRESH |
//                                     CV_CALIB_CB_FAST_CHECK |
//                                     CV_CALIB_CB_NORMALIZE_IMAGE);
//
//  cout << "Chessboard size: " << s.height << ", " << s.width << endl;
//  cout << "FOUND chessboard: " << (found ? "true" : "false")<< endl;
//
//  namedWindow("dan", CV_WINDOW_FREERATIO);
//  imshow("dan", view);
//  waitKey(0);
//
//  return 0;
//}
