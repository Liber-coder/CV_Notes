#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <math.h>


// median filter
cv::Mat median_filter(cv::Mat img, int kernel_size){
  int height = img.rows;
  int width = img.cols;
  int channel = img.channels();

  // prepare output
  cv::Mat out = cv::Mat::zeros(height, width, CV_8UC3);

  // prepare kernel
  int pad = floor(kernel_size / 2);
  
  // filtering
  double v = 0;
  int vs[kernel_size * kernel_size];
  int count = 0;
  
  for (int y = 0; y < height; y++){
    for (int x = 0; x < width; x++){
      for (int c = 0; c < channel; c++){
      v = 0;
      count = 0;
      
      for (int i = 0; i < kernel_size * kernel_size; i++){
        vs[i] = 999;
      }
      
      // get neighbor pixels
      for (int dy = -pad; dy < pad + 1; dy++){
        for (int dx = -pad; dx < pad + 1; dx++){
          if (((y + dy) >= 0) && ((x + dx) >= 0)){
            vs[count++] = (int)img.at<cv::Vec3b>(y + dy, x + dx)[c];
          }
        }
      }

      // get and assign median
      std::sort(vs, vs + (kernel_size * kernel_size));
      out.at<cv::Vec3b>(y, x)[c] = (uchar)vs[int(floor(count / 2)) + 1];
      }
    }
  }
  return out;
}

int main(int argc, const char* argv[]){
  // read image
  cv::Mat img = cv::imread("../imori_noise.jpg", cv::IMREAD_COLOR);

  // median filter
  cv::Mat out = median_filter(img, 3);
  
  //cv::imwrite("out.jpg", out);
  cv::imshow("answer", out);
  cv::waitKey(0);
  cv::destroyAllWindows();

  return 0;
}