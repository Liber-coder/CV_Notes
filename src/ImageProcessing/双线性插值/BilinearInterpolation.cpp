#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/highgui.hpp>
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;

// bilinear
// x方向放大倍数rx
// y方向放大倍数ry
cv::Mat bilinear(cv::Mat img, double rx, double ry, bool is_align){
  // get height and width
  int width = img.cols;
  int height = img.rows;
  int channel = img.channels();

  // get resized shape
  int resized_width = (int)(width * rx);
  int resized_height = (int)(height * ry);
  int x_before, y_before;
  double dx, dy;
  double val;

  // output image
  cv::Mat out = cv::Mat::zeros(resized_height, resized_width, CV_8UC3);

  // bi-linear interpolation
  for (int y = 0; y < resized_height; y++){
    if(is_align){
      // 对齐几何中心
      dy = (y+0.5)/ry - 0.5;
    }
    else{
      // 没对齐几何中心
      dy = y / ry;
    }
    y_before = (int)floor(dy);
    dy -= y_before;
    y_before = fmin(y_before, height - 1);
    y_before = max(y_before, 0);

    for (int x = 0; x < resized_width; x++){
      if(is_align){
        // 对齐几何中心
        dx = ((x+0.5)/rx - 0.5);
      }
      else{
        // 没对齐几何中心
        dx = x / rx;
      }
      x_before = (int)floor(dx);
      dx -= x_before;
      x_before = min(x_before, width - 1);
      x_before = max(x_before, 0);

      // compute bi-linear
      for (int c = 0; c < channel; c++){
        val = (1. - dx) * (1. - dy) * img.at<cv::Vec3b>(y_before, x_before)[c];
        if(y_before + 1 < height){
          val += (1. - dx) * dy * img.at<cv::Vec3b>(y_before + 1, x_before)[c];
          if(x_before + 1 < width){
            val += dx * dy * img.at<cv::Vec3b>(y_before + 1, x_before + 1)[c];
          }
        }
        if(x_before + 1 < width){
          val += dx * (1. - dy) * img.at<cv::Vec3b>(y_before, x_before + 1)[c];
        }
        // assign pixel to new position
        out.at<cv::Vec3b>(y, x)[c] = (uchar)val;
      }
    }
  }
  return out;
}

int main(int argc, const char* argv[]){
  // read image
  cv::Mat img = cv::imread("../imori.jpg", cv::IMREAD_COLOR);

  // bilinear
  cv::Mat out = bilinear(img, 10.0, 10.0, false);
  cv::Mat out_align = bilinear(img, 10.0, 10.0, true);
  cv::Mat out_cv;
  cv::resize(img, out_cv, Size(), 10.0, 10.0, INTER_LINEAR);

  //cv::imwrite("out.jpg", out);
  cv::imshow("out", out);
  cv::imshow("out_align", out_align);
  cv::imshow("out_cv", out_cv);

  cv::waitKey(0);
  int aa;
  cin >> aa;
  cv::destroyAllWindows();

  return 0;
}

