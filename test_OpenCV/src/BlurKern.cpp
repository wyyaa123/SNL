#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    //实现单方向卷积:空域卷积
    Mat img = imread("D:\\data\\lenna.jpg", 0);
    Mat outimg = img.clone();
    int h = img.rows;
    int w = img.cols;
    int wk = 24; //卷积核长度
    Mat borderimg;
    copyMakeBorder(img, borderimg, 0, 0, wk, 0, BORDER_CONSTANT, Scalar::all(0));
    auto src = borderimg.data + wk;
    auto dst = outimg.data;
    for(int i = 0; i < h; i++) {
        for(int j = 0; j < w; j++) {
            int sum = 0;
            for(int k = 0; k < wk; k++) {
                sum += src[j - k];//卷积核加权，sum类型的范围不要溢出
            }
            dst[j] = sum / (wk);
        }
        src += w + wk;
        dst += w;
    }
    imwrite("outimg.jpg", outimg);
    return 0;
}
