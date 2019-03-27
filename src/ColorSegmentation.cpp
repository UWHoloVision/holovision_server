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

Eigen::VectorXf get_likelihood(Eigen::MatrixXf& points,  Eigen::Vector3f &mean, Eigen::Matrix3f &cv){
    //points is 3xN
    Eigen::MatrixXf c_minus_mean = points.colwise() - mean; // 3xN
    Eigen::MatrixXf a1= c_minus_mean.transpose()*(cv.inverse()); // Nx3
    Eigen::VectorXf a2 = (a1.array() * c_minus_mean.transpose().array()).rowwise().sum().matrix();
    Eigen::VectorXf  exp_value = exp((-0.5*a2).array()).matrix();
    double multiplier = 1.0 / (pow(M_PI*2.0,3/2)*(pow(cv.determinant(),1/2)));
    Eigen::VectorXf likelihood = multiplier * exp_value;
    return likelihood;
}

void ColorSegmentation::segmentColors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr original, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered){
    Eigen::MatrixXf pts(3, original->points.size());
    for(auto i = 0; i < original->points.size(); i++){
        pts(0, i) = original->points[i].r;
        pts(1, i) = original->points[i].g;
        pts(2, i) = original->points[i].b;
    }
    Eigen::VectorXf likelihood_ball = get_likelihood(pts, mean_ball, cv_ball);
    Eigen::VectorXf likelihood_bg = get_likelihood(pts, mean_bg, cv_bg);
    
    for(auto i = 0; i< original->points.size(); i++){
        pcl::PointXYZRGB pt = original->points[i];
        if(likelihood_ball[i]/likelihood_bg[i] > 1.0){
            filtered->points.push_back(std::move(pt));
        }
    }
    filtered->width = filtered->points.size();
    filtered->height = 1;
}

}