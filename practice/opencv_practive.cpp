#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string.h>
#include <Eigen/Dense>

cv::Mat to_gray(cv::Mat img){
// using channel to judeg rgb
    if (img.channels() == 1){
        return img.clone();
    }
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

cv::Mat img_slop(cv::Mat img, cv::Mat Ix, cv::Mat Iy){
    // first approach
    img.copyTo(Ix);
    img.copyTo(Iy);
    for (auto i = 0; i < img.cols; i ++ ){
        for (auto j = 0; j < img.rows; j++){
            if (j >= 1 && j < img.cols - 1){
                Ix.at<double>(i, j) = img.at<uchar>(j, i + 1) - img.at<uchar>(j, i - 1);
            }
            if (i >= 1 && i < img.rows - 1){
                Iy.at<double>(i, j) = img.at<uchar>(j + 1, i) - img.at<uchar>(j - 1, i);
            }
        }
    }
    // second approach
    
    // third approach
}

int main(){
    // read image and operate with image
    
    cv::Mat img = cv::imread("/Users/huangshuoqiu/Downloads/standard-llm.png");
    cv::Mat gray = to_gray(img);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    
    cv::imshow("colored", img);
    cv::waitKey(0);

    cv::imshow("gray", gray);
    cv::waitKey(0);
    
    cv::destroyAllWindows();


    std::cout << "success!" << std::endl;
    return 1;
}
