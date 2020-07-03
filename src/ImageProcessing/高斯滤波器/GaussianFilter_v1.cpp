#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;

// 方法一：将二维高斯滤波
cv::Mat gaussian_filter(cv::Mat img, double sigma, int kernel_size){
  int height = img.rows;
  int width = img.cols;
  int channel = img.channels();

  // prepare output
  cv::Mat out = cv::Mat::zeros(height, width, CV_8UC3);

  // prepare kernel
  int pad = floor(kernel_size / 2); // 用于坐标系转换
  int _x = 0, _y = 0;
  double kernel_sum = 0;
  
  // get gaussian kernel
  float kernel[kernel_size][kernel_size];

  for (int y = 0; y < kernel_size; y++){
    for (int x = 0; x < kernel_size; x++){
      _y = y - pad;
      _x = x - pad; 
      kernel[y][x] = 1 / (2 * M_PI * sigma * sigma) * exp( - (_x * _x + _y * _y) / (2 * sigma * sigma));
      kernel_sum += kernel[y][x];
    }
  }
  // 归一化到1
  for (int y = 0; y < kernel_size; y++){
    for (int x = 0; x < kernel_size; x++){
      kernel[y][x] /= kernel_sum;
    }
  }
  

  // filtering
  for (int y = 0; y < height; y++){
    for (int x = 0; x < width; x++){
      for (int c = 0; c < channel; c++){
        double v = 0;
        for (int dy = -pad; dy < pad + 1; dy++){
          for (int dx = -pad; dx < pad + 1; dx++){
            int xx = x + dx;
            int yy = y + dy;
            // 超过边缘的就不处理了
            if ( 0 <= xx && xx < width && 0 <= yy && yy < height)
            {
              v += (double)img.ptr<Vec3b>(yy)[xx][c] * kernel[dy + pad][dx + pad];
            }
          }
        }
        out.ptr<Vec3b>(y)[x][c] = v;
      }
    }
  }
  return out;
}

// 方法二：优化。将二维高斯滤波分离为两个一维高斯滤波，可以加速
cv::Mat gaussian_filter_1D(cv::Mat img, double sigma, int kernel_size){
  int height = img.rows;
  int width = img.cols;
  int channel = img.channels();

  // prepare output
  cv::Mat out_y = cv::Mat::zeros(height, width, CV_8UC3);// y轴卷积后的结果
  cv::Mat out = cv::Mat::zeros(height, width, CV_8UC3);

  // prepare kernel
  int pad = floor(kernel_size / 2); // 用于坐标系转换
  int _x = 0, _y = 0;
  
  // get gaussian kernel
  float kernel_x[kernel_size];
  float kernel_y[kernel_size];
  double kernel_sum = 0;
  for (int x = 0; x < kernel_size; x++){
    _x = x - pad; 
    kernel_x[x] = 1 / sqrt(2 * M_PI * sigma * sigma) * exp( - (_x * _x) / (2 * sigma * sigma));
    kernel_sum += kernel_x[x];
  }
  // 归一化到1
  for (int x = 0; x < kernel_size; x++){
    kernel_x[x] /= kernel_sum;
  }

  kernel_sum = 0;
  for (int y = 0; y < kernel_size; y++){
    _y = y - pad;
    kernel_y[y] = 1 / sqrt(2 * M_PI * sigma * sigma) * exp( - (_y * _y) / (2 * sigma * sigma));
    kernel_sum += kernel_y[y];
  }
  // 归一化到1
  for (int y = 0; y < kernel_size; y++){
    kernel_y[y] /= kernel_sum;
  }
  
  // filtering_y
  for (int y = 0; y < height; y++){
    for (int x = 0; x < width; x++){
      for (int c = 0; c < channel; c++){
        double v = 0;
        for (int dy = -pad; dy < pad + 1; dy++){
          int yy = y + dy;
          // 超过边缘的就不处理了
          if (0 <= yy && yy < height)
          {
            v += (double)img.ptr<Vec3b>(y + dy)[x][c] * kernel_y[dy + pad];
          }
        }
        out_y.ptr<Vec3b>(y)[x][c] = v;
      }
    }
  }

  // filtering_x
  // 注意，这里取的是上次的结果来卷积，而不是img
  for (int y = 0; y < height; y++){
    for (int x = 0; x < width; x++){
      for (int c = 0; c < channel; c++){
        double v = 0;
        for (int dx = -pad; dx < pad + 1; dx++){
          int xx = x + dx;
          // 超过边缘的就不处理了
          if (0 <= xx && xx < width )
          {
            v += (double)out_y.ptr<Vec3b>(y)[xx][c] * kernel_x[dx + pad];
          }
        }
        out.ptr<Vec3b>(y)[x][c] = v;
      }
    }
  }

  return out;
}

int main(int argc, const char* argv[]){
  // read image
  cv::Mat img = cv::imread("../imori_noise.jpg", cv::IMREAD_COLOR);
  cv::imshow("raw", img);

  clock_t start = clock();
  // gaussian filter
  // cv::Mat out = gaussian_filter(img, 1.3, 3); // 3600 us
  cv::Mat out = gaussian_filter_1D(img, 1.3, 3); // 5400 us
  clock_t end = clock();
  printf("time: %d us\n", (end-start)); // us //(CLOCKS_PER_SEC/1000)
  
  //cv::imwrite("out.jpg", out);
  cv::namedWindow("answer", cv::WINDOW_NORMAL);
  cv::imshow("answer", out);
  cv::waitKey(0);
  cv::destroyAllWindows();

  return 0;
}
