#include <opencv2/opencv.hpp>
#include <iostream>

cv::KalmanFilter kalmanfilter(2,2);
cv::Mat last_measurement(2,1,CV_32FC1);
cv::Mat current_measurement(2,1,CV_32FC1);

cv::Mat last_prediction(2,1,CV_32FC1);
cv::Mat current_prediction(2,1,CV_32FC1);

cv::Mat frame(800,800,CV_8UC3);

void onMouseMove(int event,int x,int y,int flag,void* data)
{

    last_prediction = current_prediction;
    last_measurement = current_measurement;

    current_measurement.at<float>(0) = x;
    current_measurement.at<float>(1) = y;

    std::cout << current_measurement << std::endl;

    kalmanfilter.correct(current_measurement);

    current_prediction = kalmanfilter.predict();
    //kalmanfilter.predict();

    cv::line(frame,cv::Point(last_measurement.at<float>(0),last_measurement.at<float>(1)),cv::Point(current_measurement.at<float>(0),current_measurement.at<float>(1)),cv::Scalar(0,255,0),2);
    cv::line(frame,cv::Point(last_prediction.at<float>(0),last_prediction.at<float>(1)),cv::Point(current_prediction.at<float>(0),current_prediction.at<float>(1)),cv::Scalar(0,0,255),2);

    std::cout << "on mouse move:" << current_prediction <<std::endl;

}

int main(int argc,char** argv)
{
    //cv::Mat frame(800,800,CV_8UC3);

    cv::namedWindow("test");
    cv::setMouseCallback("test",onMouseMove,NULL);

    cv::Mat F(2,2,CV_32F,cv::Scalar(0));
    //m.at<float>(0,0) = 1;
    //m.at<float>(1,1) = 1;

    cv::setIdentity(F,cv::Scalar(1));

    cv::Mat H(2,2,CV_32F);
    cv::setIdentity(H,cv::Scalar(1));

    cv::Mat p(2,2,CV_32F);
    cv::setIdentity(p,cv::Scalar(0.1));

    kalmanfilter.measurementMatrix = H;
    kalmanfilter.transitionMatrix = F;
    kalmanfilter.processNoiseCov = p;

    while (true)
    {
        cv::imshow("test",frame);

        if (cv::waitKey(30) == 113)
            break;
    }

    cv::destroyAllWindows();

    return 0;
}
