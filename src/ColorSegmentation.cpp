#include "ColorSegmentation.h"
#include <math.h>

namespace holovision{
const std::string ColorSegmentation::MEAN_BALL_PATH = 
    "../src/segmentation_mean_ball.bin";
const std::string ColorSegmentation::MEAN_BG_PATH = 
    "../src/segmentation_mean_bg.bin";
const std::string ColorSegmentation::CV_BALL_PATH = 
    "../src/segmentation_cv_ball.bin";
const std::string ColorSegmentation::CV_BG_PATH = 
    "../src/segmentation_cv_bg.bin";

Eigen::Vector3f read_bin_v(std::string path){
    Eigen::Vector3f v;
    std::ifstream bin_stream(path, std::ios::binary);
    if (!bin_stream) {
        throw std::runtime_error("Couldn't open bin file");
    }
    for(auto i=0; i < 3; i++){
        double x; //chek double or float
        assert(bin_stream.read(reinterpret_cast<char*>(&x), sizeof x));
        v(i) =  (float) x; // approximation here
    }
    return v;
}

Eigen::Matrix3f read_bin_m(std::string path){
    Eigen::Matrix3f v;
    std::ifstream bin_stream(path, std::ios::binary);
    if (!bin_stream) {
        throw std::runtime_error("Couldn't open bin file");
    }
    for(auto row=0; row<3; row++)
    for(auto col=0; col < 3; col++){
        double x; //chek double or float
        assert(bin_stream.read(reinterpret_cast<char*>(&x), sizeof x));
        v(row, col) = (float) x;
    }
    return v;
}

ColorSegmentation::ColorSegmentation(){
    mean_ball = read_bin_v(MEAN_BALL_PATH);
    mean_bg = read_bin_v(MEAN_BG_PATH);
    cv_ball = read_bin_m(CV_BALL_PATH);
    cv_bg = read_bin_m(CV_BG_PATH);
}

double get_likelihood(pcl::PointXYZRGB &pt, Eigen::Vector3f &mean, Eigen::Matrix3f &cv){
    Eigen::Vector3f c;
    c << pt.r, pt.g, pt.b;
    Eigen::Vector3f c_minus_mean = c - mean;
    double exp_value = exp(-0.5*c_minus_mean.transpose()*(cv.inverse())*c_minus_mean);
    double likelihood = 1.0 / (pow(M_PI*2.0,3/2)*(pow(cv.determinant(),1/2)))*exp_value;
    return likelihood;
}

bool ColorSegmentation::predict(pcl::PointXYZRGB &pt){
    double likelihood_ball = get_likelihood(pt, mean_ball, cv_ball);
    double likelihood_bg = get_likelihood(pt, mean_bg, cv_bg);

    double neuman_test = likelihood_ball/likelihood_bg;
    return neuman_test > 1.0;
}

void ColorSegmentation::segmentColors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr original, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered){
    for(pcl::PointXYZRGB pt: original->points){
        if(predict(pt)){
            filtered->points.push_back(std::move(pt));
        }
    }
    filtered->width = filtered->points.size();
    filtered->height = 1;
}

}